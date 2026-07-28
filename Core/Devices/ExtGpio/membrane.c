#include "membrane.h"

/* Front-panel membrane keypad (U9) driving the LED latch (U8). See membrane.h
 * for the wiring, the polling rationale and the toggle behaviour. */

static tca9554_t          s_sw;    /* U9 - DIS-SW inputs   */
static tca9554_t          s_led;   /* U8 - DIS-LED outputs */
static membrane_key_cb_t  s_cb;

/* Debounce state, per key (P0..P7 of U9). */
static uint8_t s_pressed;                       /* debounced pressed mask (bit i = SWi+1) */
static uint8_t s_cand[MEMBRANE_KEY_COUNT];      /* candidate level being counted           */
static uint8_t s_cnt[MEMBRANE_KEY_COUNT];       /* consecutive-sample counter              */

static uint8_t s_led_mask;                       /* current LED latch (bit i = LEDi+1)     */

/* Translate the DIS-LED latch mask into the raw U8 output-port value, honouring
 * the LED active level. */
static uint8_t led_mask_to_port(uint8_t mask)
{
#if MEMBRANE_LED_ACTIVE_HIGH
	return mask;
#else
	return (uint8_t)~mask;
#endif
}

/* Read U9 and return the *logical pressed* mask (bit set = key pressed),
 * independent of electrical polarity. With MEMBRANE_SW_ACTIVE_HIGH==0 the
 * expander's polarity-inversion register already flips the bits, so the raw
 * input reads 1 on press either way. */
static HAL_StatusTypeDef read_pressed(uint8_t *pressed)
{
	uint8_t raw;
	HAL_StatusTypeDef st = TCA9554_ReadInput(&s_sw, &raw);
	if (st != HAL_OK)
		return st;

	*pressed = raw;   /* polarity handled in HW via the inversion register */
	return HAL_OK;
}

HAL_StatusTypeDef Membrane_Init(membrane_key_cb_t cb)
{
	HAL_StatusTypeDef st;
	uint8_t i;

	s_cb       = cb;
	s_led_mask = 0x00U;

	/* U8: all pins outputs, LEDs off. */
	st = TCA9554_Init(&s_led, &hi2c1, TCA9554_U8_ADDR,
	                  TCA9554_ALL_OUTPUTS, led_mask_to_port(0x00U));
	if (st != HAL_OK)
		return st;

	/* U9: all pins inputs. */
	st = TCA9554_Init(&s_sw, &hi2c1, TCA9554_U9_ADDR,
	                  TCA9554_ALL_INPUTS, 0x00U);
	if (st != HAL_OK)
		return st;

	/* For active-low keypads, invert the input readings in hardware so a press
	 * reads as 1 and edge detection stays polarity-agnostic. */
#if !MEMBRANE_SW_ACTIVE_HIGH
	st = TCA9554_SetPolarity(&s_sw, 0xFFU);
	if (st != HAL_OK)
		return st;
#endif

	/* Seed debounce from the current state so a key already held at boot does
	 * not register a spurious edge on the first poll. */
	st = read_pressed(&s_pressed);
	if (st != HAL_OK) {
		s_pressed = 0x00U;
		return st;
	}

	for (i = 0; i < MEMBRANE_KEY_COUNT; i++) {
		s_cand[i] = (uint8_t)((s_pressed >> i) & 1U);
		s_cnt[i]  = MEMBRANE_DEBOUNCE_SAMPLES;
	}

	return HAL_OK;
}

HAL_StatusTypeDef Membrane_Poll(void)
{
	uint8_t sample, i, changed = 0U;
	HAL_StatusTypeDef st = read_pressed(&sample);
	if (st != HAL_OK)
		return st;

	for (i = 0; i < MEMBRANE_KEY_COUNT; i++) {
		uint8_t level = (uint8_t)((sample >> i) & 1U);
		uint8_t prev  = (uint8_t)((s_pressed >> i) & 1U);

		/* Debounce: require the same reading for N consecutive polls. */
		if (level != s_cand[i]) {
			s_cand[i] = level;
			s_cnt[i]  = 1U;
			continue;
		}
		if (s_cnt[i] < MEMBRANE_DEBOUNCE_SAMPLES) {
			s_cnt[i]++;
			if (s_cnt[i] < MEMBRANE_DEBOUNCE_SAMPLES)
				continue;   /* not yet stable */
		} else {
			continue;       /* already accepted at this level */
		}

		/* Debounced level just became stable; commit it. */
		if (level)
			s_pressed |= (uint8_t)(1U << i);
		else
			s_pressed &= (uint8_t)~(1U << i);

		/* Rising edge (release -> press) toggles the LED latch. */
		if (level && !prev) {
			s_led_mask ^= (uint8_t)(1U << i);
			changed = 1U;
			if (s_cb != NULL)
				s_cb(i, (uint8_t)((s_led_mask >> i) & 1U));
		}
	}

	if (changed)
		return TCA9554_WriteOutput(&s_led, led_mask_to_port(s_led_mask));

	return HAL_OK;
}

/* ---------------------------------------------------------------------- */
/* LED latch access                                                        */
/* ---------------------------------------------------------------------- */
uint8_t Membrane_GetLed(uint8_t key)
{
	if (key >= MEMBRANE_KEY_COUNT)
		return 0U;
	return (uint8_t)((s_led_mask >> key) & 1U);
}

uint8_t Membrane_GetLedMask(void)
{
	return s_led_mask;
}

HAL_StatusTypeDef Membrane_SetLed(uint8_t key, uint8_t on)
{
	if (key >= MEMBRANE_KEY_COUNT)
		return HAL_ERROR;

	if (on)
		s_led_mask |= (uint8_t)(1U << key);
	else
		s_led_mask &= (uint8_t)~(1U << key);

	return TCA9554_WriteOutput(&s_led, led_mask_to_port(s_led_mask));
}

HAL_StatusTypeDef Membrane_SetLedMask(uint8_t mask)
{
	s_led_mask = mask;
	return TCA9554_WriteOutput(&s_led, led_mask_to_port(s_led_mask));
}
