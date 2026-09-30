#include "tb_tca9554.h"

#include "keypad.h"       /* U31/U32 owner: debounce, press/long events, LEDs */
#include "bldc_ctrl.h"    /* M1 g_grind_ctrl / M2 g_stir_ctrl closed-loop control */

/* Front-panel keypad -> BLDC motor testbed. Since REV02 this file does NO bus
 * traffic of its own: Devices/ExtGpio/keypad.* owns U31(SW)/U32(LED), and this
 * testbed only consumes its press events. See tb_tca9554.h. Both motors are
 * driven CLOSED-LOOP through bldc_ctrl (M1 = g_grind_ctrl, M2 = g_stir_ctrl);
 * this file only maps button edges to BldcCtrl_Start/Stop/SpeedStep. */

/* ---- Public state --------------------------------------------------------- */
volatile tb_tca9554_btn_t tb_btn[TB_TCA9554_BTN_COUNT];
volatile uint8_t          tb_btn_mask;   /* debounced pressed mask */
volatile uint8_t          tb_led_mask;   /* current lit-LED mask   */

/* Debug enable (testbench only): keypad-free start/stop of the two motors.
 * See tb_tca9554.h for the contract. */
volatile uint8_t tb_keypad_motor_en = 1U;   /* 0 = 버튼 눌러도 모터 안 돌린다 */
volatile uint8_t tb_grind_en;
volatile uint8_t tb_grind_rev;
volatile uint8_t tb_grind_spd_req;
volatile uint8_t tb_stir_en;
volatile uint8_t tb_stir_rev;
volatile uint8_t tb_stir_spd_req;

/* ---- Private state -------------------------------------------------------- */
/* Previous tb_*_en level, for edge detection of the debug-enable inputs. */
static uint8_t s_grind_en_prev;
static uint8_t s_stir_en_prev;

/* ---- Small helpers -------------------------------------------------------- */

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

/* Apply one motor's debug-enable inputs as if the panel buttons were pressed
 * (testbench only; called from TB_TCA9554_Poll). Edge-triggered so it mimics a
 * button press exactly rather than re-issuing Start/Stop every poll:
 *   en 0->1 : BldcCtrl_Start (dir from *rev)   -- like SW1/SW2 (fwd) / SW5/SW6 (rev)
 *   en 1->0 : BldcCtrl_Stop                    -- like SW3/SW4
 *   *spd_req: one BldcCtrl_SpeedStep, self-cleared -- like SW7/SW8
 * BldcCtrl_Start/Stop enforce the start-only-while-stopped and re-arm rules
 * internally, so the raw levels here are always safe to act on. */
static void apply_enable(BldcCtrl_t *c, volatile uint8_t *en, uint8_t *en_prev,
                         volatile uint8_t *rev, volatile uint8_t *spd_req)
{
	uint8_t en_now = (*en) ? 1U : 0U;

	if (en_now && !*en_prev)
		BldcCtrl_Start(c, (*rev) ? 1U : 0U);   /* rising edge: start */
	else if (!en_now && *en_prev)
		BldcCtrl_Stop(c);                      /* falling edge: stop */
	*en_prev = en_now;

	if (*spd_req)
	{
		BldcCtrl_SpeedStep(c);                 /* one rung per request */
		*spd_req = 0U;
	}
}

/* ---- Lifecycle ------------------------------------------------------------ */
void TB_TCA9554_Init(void)
{
	uint8_t i;

	/* U31/U32 are brought up by Keypad_Init() (called first in StartMotorTask);
	 * nothing here touches the bus. Mirror the driver's current snapshot so the
	 * debugger views are valid before the first Poll. */
	tb_btn_mask = Keypad_GetMask();
	tb_led_mask = Keypad_GetLedMask();

	/* Seed debug-enable edge state to the current levels so a variable left set
	 * from a previous run does not fire a spurious Start/Stop on the first poll. */
	s_grind_en_prev = tb_grind_en ? 1U : 0U;
	s_stir_en_prev  = tb_stir_en  ? 1U : 0U;

	for (i = 0; i < TB_TCA9554_BTN_COUNT; i++)
	{
		tb_btn[i].pressed   = Keypad_IsPressed(i);
		tb_btn[i].edge      = 0U;
		tb_btn[i].press_cnt = 0U;
	}

	/* Both motors' speed ladders live in their bldc_ctrl instances
	 * (BldcCtrl_Init, called from StartMotorTask), so nothing motor-related is
	 * set up here -- this keypad only maps button edges to those controllers. */
}

void TB_TCA9554_Poll(void)
{
	uint8_t i;

	/* Debug-enable inputs (testbench only): drive the motors without the keypad. */
	apply_enable(&g_grind_ctrl, &tb_grind_en, &s_grind_en_prev,
	             &tb_grind_rev, &tb_grind_spd_req);
	apply_enable(&g_stir_ctrl,  &tb_stir_en,  &s_stir_en_prev,
	             &tb_stir_rev,  &tb_stir_spd_req);

	/* Consume the keypad driver's debounced press events. Keypad_Tick() already
	 * ran this cycle (StartMotorTask, before the mode switch), so a press taken
	 * here acts before the BldcCtrl_Tick calls that follow. */
	for (i = 0; i < TB_TCA9554_BTN_COUNT; i++)
	{
		tb_btn[i].pressed = Keypad_IsPressed(i);
		tb_btn[i].edge    = Keypad_TakePress(i);

		if (tb_btn[i].edge)
		{
			tb_btn[i].press_cnt++;
			if (tb_keypad_motor_en)
				handle_press(i);   /* 0 이면 관측만 - 채널 대응 확인용 */
		}
	}

	/* Debugger views; the LED latch itself is owned by keypad.c
	 * (KEYPAD_LED_FOLLOW_PRESS: lit unless its key is held). */
	tb_btn_mask = Keypad_GetMask();
	tb_led_mask = Keypad_GetLedMask();
}
