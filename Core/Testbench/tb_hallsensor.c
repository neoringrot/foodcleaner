#include "tb_hallsensor.h"

/* Monitor testbed for the 4 Hall-sensor channels on the U24 TCA9554A. See
 * tb_hallsensor.h for the U24 pin map, the HALL-INT1 (PF9) interrupt note and
 * the single-owner (StartDefaultTask) polling constraint.
 *
 * Reads go through the production hallsensor.c driver (HallSensor_ServiceInt /
 * _Update / _GetMask), so the values here match the firmware exactly; this
 * testbed just gates the reads behind tb_hall_enable and splits the 8-bit mask
 * into the four named channels (trigger group P0..P4, 교반원점 P5, 수거통 P6,
 * 리프트하단 P7) with rising-edge counters for probing. */

/* Runtime switch -- set from the debugger while running. Default off/safe. */
volatile uint8_t tb_hall_enable = 0;

/* Latest snapshot. */
volatile uint8_t tb_hall_mask = 0;
volatile uint8_t tb_hall_pinlevel = 0;   /* actual P0..P7 pin voltage (~mask) */

volatile uint8_t tb_hall_trig[TB_HALL_TRIG_COUNT] = {0};
volatile uint8_t tb_hall_trig_mask = 0;

volatile uint8_t tb_hall_hard_food = 0;   /* P0 강음(강한음식물) */
volatile uint8_t tb_hall_collect   = 0;   /* P1 모음 */
volatile uint8_t tb_hall_stop      = 0;   /* P2 정지 */
volatile uint8_t tb_hall_drain     = 0;   /* P3 배수 */
volatile uint8_t tb_hall_run       = 0;   /* P4 동작 */

volatile uint8_t tb_hall_stir_home   = 0;
volatile uint8_t tb_hall_bin         = 0;
volatile uint8_t tb_hall_lift_bottom = 0;

volatile uint32_t tb_hall_trig_events[TB_HALL_TRIG_COUNT] = {0};
volatile uint32_t tb_hall_stir_home_events   = 0;
volatile uint32_t tb_hall_bin_events         = 0;
volatile uint32_t tb_hall_lift_bottom_events = 0;

volatile uint32_t tb_hall_samples    = 0;
volatile uint32_t tb_hall_int_events = 0;
volatile uint32_t tb_hall_changes    = 0;

/* Previous mask, for per-bit rising-edge detection. */
static uint8_t s_prev_mask;
static uint8_t s_have_prev;   /* 0 until the first enabled poll seeds s_prev_mask */

/* Fan the detected mask out into the four named channels. */
static void decode_mask(uint8_t mask)
{
	uint8_t i;

	tb_hall_mask = mask;
	/* Voltage-domain view: HW polarity inversion is 0xFF, so the true pin level
	 * is the bit-complement of the detected mask. See tb_hall_pinlevel doc. */
	tb_hall_pinlevel = (uint8_t)~mask;

	for (i = 0; i < TB_HALL_TRIG_COUNT; i++)
		tb_hall_trig[i] = (uint8_t)((mask >> i) & 1U);
	tb_hall_trig_mask = (uint8_t)(mask & 0x1FU);

	/* Named aliases for the five triggers (same bits as tb_hall_trig[]). */
	tb_hall_hard_food = tb_hall_trig[TB_HALL_BIT_HARD_FOOD];
	tb_hall_collect   = tb_hall_trig[TB_HALL_BIT_COLLECT];
	tb_hall_stop      = tb_hall_trig[TB_HALL_BIT_STOP];
	tb_hall_drain     = tb_hall_trig[TB_HALL_BIT_DRAIN];
	tb_hall_run       = tb_hall_trig[TB_HALL_BIT_RUN];

	tb_hall_stir_home   = (uint8_t)((mask >> TB_HALL_BIT_STIR_HOME)   & 1U);
	tb_hall_bin         = (uint8_t)((mask >> TB_HALL_BIT_BIN)         & 1U);
	tb_hall_lift_bottom = (uint8_t)((mask >> TB_HALL_BIT_LIFT_BOTTOM) & 1U);
}

/* Bump the rising-edge counters for bits that went 0->1 between prev and now. */
static void count_edges(uint8_t prev, uint8_t now)
{
	uint8_t rose = (uint8_t)(now & (uint8_t)~prev);   /* 0->1 bits */
	uint8_t i;

	if (rose == 0U)
		return;

	for (i = 0; i < TB_HALL_TRIG_COUNT; i++)
	{
		if (rose & (uint8_t)(1U << i))
			tb_hall_trig_events[i]++;
	}
	if (rose & (uint8_t)(1U << TB_HALL_BIT_STIR_HOME))
		tb_hall_stir_home_events++;
	if (rose & (uint8_t)(1U << TB_HALL_BIT_BIN))
		tb_hall_bin_events++;
	if (rose & (uint8_t)(1U << TB_HALL_BIT_LIFT_BOTTOM))
		tb_hall_lift_bottom_events++;
}

void TB_HallSensor_Init(void)
{
	uint8_t i;

	tb_hall_enable = 0;

	tb_hall_mask      = 0;
	tb_hall_pinlevel  = 0;
	tb_hall_trig_mask = 0;
	for (i = 0; i < TB_HALL_TRIG_COUNT; i++)
	{
		tb_hall_trig[i]        = 0;
		tb_hall_trig_events[i] = 0;
	}
	tb_hall_hard_food = 0;
	tb_hall_collect   = 0;
	tb_hall_stop      = 0;
	tb_hall_drain     = 0;
	tb_hall_run       = 0;

	tb_hall_stir_home   = 0;
	tb_hall_bin         = 0;
	tb_hall_lift_bottom = 0;

	tb_hall_stir_home_events   = 0;
	tb_hall_bin_events         = 0;
	tb_hall_lift_bottom_events = 0;

	tb_hall_samples    = 0;
	tb_hall_int_events = 0;
	tb_hall_changes    = 0;

	s_prev_mask = 0;
	s_have_prev = 0;
}

void TB_HallSensor_Poll(void)
{
	uint8_t mask;

	/* Disabled: monitoring stopped. Keep the last snapshot untouched so it can
	 * still be inspected, and issue no U24 read from this testbed. Drop the edge
	 * history so re-enabling seeds a fresh baseline instead of firing spurious
	 * edges against a stale mask. */
	if (!tb_hall_enable)
	{
		s_have_prev = 0;
		return;
	}

	/* Enabled: service the HALL-INT1 (PF9) edge for low latency, then do the
	 * periodic read as a safety net for a missed edge. Both go through the
	 * production driver; HallSensor_GetMask() returns the polarity-normalised
	 * P0..P7 snapshot they refresh. */
	if (HallSensor_ServiceInt())
		tb_hall_int_events++;
	(void)HallSensor_Update();
	mask = HallSensor_GetMask();

	/* First enabled poll only seeds the baseline so a magnet already present at
	 * enable time is not miscounted as a fresh edge. */
	if (!s_have_prev)
	{
		s_prev_mask = mask;
		s_have_prev = 1;
	}
	else if (mask != s_prev_mask)
	{
		count_edges(s_prev_mask, mask);
		tb_hall_changes++;
		s_prev_mask = mask;
	}

	decode_mask(mask);
	tb_hall_samples++;
}
