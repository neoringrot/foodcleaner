#include "tb_tca9554.h"

#include "tca9554.h"
#include "bldc_ctrl.h"    /* M1 g_grind_ctrl / M2 g_stir_ctrl closed-loop control */
#include "i2c.h"          /* hi2c1 */

/* Front-panel keypad (U9) -> BLDC motor testbed. See tb_tca9554.h for the wiring,
 * the LED behaviour, the button->motor map and the single-owner constraint
 * against the membrane driver. Both motors are driven CLOSED-LOOP through
 * bldc_ctrl (M1 = g_grind_ctrl, M2 = g_stir_ctrl); this file only maps button
 * edges to BldcCtrl_Start/Stop/SpeedStep. */

/* ---- Public state --------------------------------------------------------- */
volatile tb_tca9554_btn_t tb_btn[TB_TCA9554_BTN_COUNT];
volatile uint8_t          tb_btn_mask;   /* debounced pressed mask */
volatile uint8_t          tb_led_mask;   /* current lit-LED mask   */

/* ---- Private state -------------------------------------------------------- */
static tca9554_t s_sw;    /* U9 - DIS-SW inputs   */
static tca9554_t s_led;   /* U8 - DIS-LED outputs */

/* Debounce, per key (P0..P7 of U9). */
static uint8_t s_cand[TB_TCA9554_BTN_COUNT];   /* candidate level being counted */
static uint8_t s_cnt[TB_TCA9554_BTN_COUNT];    /* consecutive-sample counter    */

/* ---- Small helpers -------------------------------------------------------- */

/* Translate the lit-LED mask into the raw U8 output-port value, honouring the
 * LED active level. Default (active-high): lit bit -> 1. */
static uint8_t led_mask_to_port(uint8_t lit_mask)
{
#if TB_TCA9554_LED_ACTIVE_HIGH
	return lit_mask;
#else
	return (uint8_t)~lit_mask;
#endif
}

/* Read U9 and return the *logical pressed* mask (bit set = key pressed). With
 * TB_TCA9554_SW_ACTIVE_HIGH==0 the expander's polarity-inversion register
 * already flips the bits, so the raw input reads 1 on press either way. */
static HAL_StatusTypeDef read_pressed(uint8_t *pressed)
{
	uint8_t raw;
	HAL_StatusTypeDef st = TCA9554_ReadInput(&s_sw, &raw);
	if (st != HAL_OK)
		return st;

	*pressed = raw;
	return HAL_OK;
}

/* Run the action bound to a fresh press of SW(index+1). Odd buttons drive M1
 * (grinder, g_grind_ctrl), even buttons drive M2 (stirrer, g_stir_ctrl); both
 * go through the same closed-loop API. BldcCtrl enforces start-only-while-
 * stopped and the soft-lock re-arm rule internally. */
static void handle_press(uint8_t idx)
{
	switch (idx)
	{
	/* --- M1 (grinder, odd panel buttons SW1/3/5/7) --- */
	case 0: /* SW1: M1 forward  */
		BldcCtrl_Start(&g_grind_ctrl, 0U);
		break;
	case 2: /* SW3: M1 stop     */
		BldcCtrl_Stop(&g_grind_ctrl);
		break;
	case 4: /* SW5: M1 reverse  */
		BldcCtrl_Start(&g_grind_ctrl, 1U);
		break;
	case 6: /* SW7: M1 speed up */
		BldcCtrl_SpeedStep(&g_grind_ctrl);
		break;

	/* --- M2 (stirrer, even panel buttons SW2/4/6/8) --- */
	case 1: /* SW2: M2 forward  */
		BldcCtrl_Start(&g_stir_ctrl, 0U);
		break;
	case 3: /* SW4: M2 stop     */
		BldcCtrl_Stop(&g_stir_ctrl);
		break;
	case 5: /* SW6: M2 reverse  */
		BldcCtrl_Start(&g_stir_ctrl, 1U);
		break;
	case 7: /* SW8: M2 speed up */
		BldcCtrl_SpeedStep(&g_stir_ctrl);
		break;

	default:
		break;
	}
}

/* ---- Lifecycle ------------------------------------------------------------ */
void TB_TCA9554_Init(void)
{
	uint8_t seed = 0x00U;
	uint8_t i;

	tb_btn_mask = 0x00U;
	tb_led_mask = 0xFFU;                     /* all LEDs lit by default */

	/* LEDs -> 0x38 (TCA9554_U9_ADDR, netlist "U9"): all outputs, all LEDs lit.
	 * (Bench-confirmed swap vs netlist; see tb_tca9554.h header.) */
	(void)TCA9554_Init(&s_led, &hi2c1, TCA9554_U9_ADDR,
	                   TCA9554_ALL_OUTPUTS, led_mask_to_port(tb_led_mask));

	/* Keypad -> 0x39 (TCA9554_U8_ADDR, netlist "U8"): all inputs. */
	(void)TCA9554_Init(&s_sw, &hi2c1, TCA9554_U8_ADDR,
	                   TCA9554_ALL_INPUTS, 0x00U);

#if !TB_TCA9554_SW_ACTIVE_HIGH
	/* Active-low keypad: invert input readings in hardware so a press reads 1. */
	(void)TCA9554_SetPolarity(&s_sw, 0xFFU);
#endif

	/* Seed debounce from the current state so a key held at boot does not fire a
	 * spurious edge on the first poll. */
	if (read_pressed(&seed) != HAL_OK)
		seed = 0x00U;

	tb_btn_mask = seed;
	for (i = 0; i < TB_TCA9554_BTN_COUNT; i++)
	{
		uint8_t level = (uint8_t)((seed >> i) & 1U);
		s_cand[i]        = level;
		s_cnt[i]         = TB_TCA9554_DEBOUNCE_SAMPLES;
		tb_btn[i].pressed   = level;
		tb_btn[i].edge      = 0U;
		tb_btn[i].press_cnt = 0U;
	}

	/* Both motors' speed ladders live in their bldc_ctrl instances
	 * (BldcCtrl_Init, called from StartMotorTask), so nothing motor-related is
	 * set up here -- this keypad only maps button edges to those controllers. */
}

void TB_TCA9554_Poll(void)
{
	uint8_t sample = 0x00U;
	uint8_t i;
	uint8_t led_changed = 0U;

	if (read_pressed(&sample) != HAL_OK)
		return;                              /* bus hiccup: skip this cycle */

	for (i = 0; i < TB_TCA9554_BTN_COUNT; i++)
	{
		uint8_t level = (uint8_t)((sample >> i) & 1U);
		uint8_t prev  = (uint8_t)((tb_btn_mask >> i) & 1U);

		tb_btn[i].edge = 0U;                 /* one-poll pulse; clear each cycle */

		/* Debounce: require the same reading for N consecutive polls. */
		if (level != s_cand[i])
		{
			s_cand[i] = level;
			s_cnt[i]  = 1U;
			continue;
		}
		if (s_cnt[i] < TB_TCA9554_DEBOUNCE_SAMPLES)
		{
			s_cnt[i]++;
			if (s_cnt[i] < TB_TCA9554_DEBOUNCE_SAMPLES)
				continue;                    /* not yet stable */
		}
		else
		{
			continue;                        /* already accepted at this level */
		}

		/* Debounced level just became stable; commit it. */
		if (level)
			tb_btn_mask |= (uint8_t)(1U << i);
		else
			tb_btn_mask &= (uint8_t)~(1U << i);
		tb_btn[i].pressed = level;

		/* Rising edge (release -> press): run the bound action once. */
		if (level && !prev)
		{
			tb_btn[i].edge = 1U;
			tb_btn[i].press_cnt++;
			handle_press(i);
		}
	}

	/* LED = lit unless its key is held. Recompute and write U8 only on change. */
	{
		uint8_t new_led = (uint8_t)~tb_btn_mask;   /* held bit -> LED off */
		if (new_led != tb_led_mask)
		{
			tb_led_mask = new_led;
			led_changed = 1U;
		}
	}
	if (led_changed)
		(void)TCA9554_WriteOutput(&s_led, led_mask_to_port(tb_led_mask));
}
