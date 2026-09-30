#include "keypad.h"
#include "gpio_ctrl.h"   /* U31 INT = PF10 = GPIO_EXTI_HALL_INT2 */
#include "i2c.h"         /* hi2c1 */

/* 전면 멤브레인 버튼 8 + LED 8 (U31/U32, I2C1). 배선·극성·이벤트 모델은
 * keypad.h 참조. 이 파일은 "입력 처리"까지만 한다 - SWn 이 무엇을 하는지는
 * 여기 없다(기능 배정 I10·I14 미확정). */

/* ---- 공개 상태(디버거 watch) ----------------------------------------- */
volatile keypad_btn_t g_keypad_btn[KEYPAD_KEY_COUNT];
volatile uint8_t      g_keypad_mask;
volatile uint8_t      g_keypad_led_mask;
volatile uint32_t     g_keypad_bus_err;

/* ---- 내부 상태 -------------------------------------------------------- */
static tca9554_t s_sw;                        /* U31 0x39 - DIS-SW 입력  */
static tca9554_t s_led;                       /* U32 0x38 - DIS-LED 출력 */

static uint8_t   s_cand[KEYPAD_KEY_COUNT];    /* 후보 레벨               */
static uint32_t  s_cand_ms[KEYPAD_KEY_COUNT]; /* 후보가 처음 보인 시각   */

static uint32_t  s_last_poll_ms;
static uint32_t  s_last_change_ms;            /* 마지막 입력 변화 시각    */
static uint32_t  s_last_led_ms;
static uint8_t   s_led_written;               /* U32 에 반영된 래치      */
static uint8_t   s_inited;

/* ---- 작은 헬퍼 -------------------------------------------------------- */

/* 점등 마스크 -> U32 출력 포트 값. 이 보드는 ACTIVE-HIGH(핀 HIGH = 점등). */
static uint8_t led_mask_to_port(uint8_t lit_mask)
{
#if KEYPAD_LED_ACTIVE_HIGH
	return lit_mask;
#else
	return (uint8_t)~lit_mask;
#endif
}

/* U31 을 읽어 "논리 눌림 마스크"를 돌려준다. KEYPAD_SW_ACTIVE_HIGH==0 이면
 * Init 에서 U31 의 극성반전 레지스터를 켜 두므로 raw 가 이미 1=눌림이다. */
static HAL_StatusTypeDef read_pressed(uint8_t *pressed)
{
	uint8_t raw;
	HAL_StatusTypeDef st = TCA9554_ReadInput(&s_sw, &raw);
	if (st != HAL_OK)
		return st;

	*pressed = raw;
	return HAL_OK;
}

/* 디바운스가 끝난 레벨 변화를 반영하고 이벤트를 래치한다. */
static void commit_level(uint8_t i, uint8_t level, uint32_t now_ms)
{
	if (level)
	{
		g_keypad_mask |= (uint8_t)(1U << i);
		g_keypad_btn[i].pressed    = 1U;
		g_keypad_btn[i].press_ms   = now_ms;
		g_keypad_btn[i].long_fired = 0U;
		g_keypad_btn[i].press_cnt++;

		/* 짧게/길게가 배타여야 하는 키는 뗄 때 발행한다(keypad.h 참조). */
		if (((KEYPAD_LONG_ONLY_MASK) & (1U << i)) == 0U)
		{
			g_keypad_btn[i].press_evt    = 1U;
			g_keypad_btn[i].press_evt_ms = now_ms;
		}
	}
	else
	{
		g_keypad_mask &= (uint8_t)~(1U << i);
		g_keypad_btn[i].pressed = 0U;

		/* long-only 키: 길게누름을 채우지 못하고 떼면 그때 짧은 눌림 1회. */
		if ((((KEYPAD_LONG_ONLY_MASK) & (1U << i)) != 0U) &&
		    !g_keypad_btn[i].long_fired)
		{
			g_keypad_btn[i].press_evt    = 1U;
			g_keypad_btn[i].press_evt_ms = now_ms;
		}
		g_keypad_btn[i].long_fired = 0U;
	}
}

/* 한 번의 버스 읽기 결과를 디바운스한다. */
static void scan(uint8_t sample, uint32_t now_ms)
{
	uint8_t i;

	for (i = 0; i < KEYPAD_KEY_COUNT; i++)
	{
		uint8_t level = (uint8_t)((sample >> i) & 1U);

		if (level != s_cand[i])
		{
			s_cand[i]        = level;  /* 새 후보 - 타이머 재시작 */
			s_cand_ms[i]     = now_ms;
			s_last_change_ms = now_ms; /* 활성 구간 연장          */
			continue;
		}

		if (level == g_keypad_btn[i].pressed)
			continue;                  /* 이미 그 레벨로 채택돼 있다 */

		if ((uint32_t)(now_ms - s_cand_ms[i]) >= KEYPAD_DEBOUNCE_MS)
		{
			commit_level(i, level, now_ms);
			s_last_change_ms = now_ms;
		}
	}
}

/* 길게누름 판정 + 미소비 이벤트 폐기. 버스 읽기와 무관하게 매 Tick 돈다. */
static void age_events(uint32_t now_ms)
{
	uint8_t i;

	for (i = 0; i < KEYPAD_KEY_COUNT; i++)
	{
		if (g_keypad_btn[i].pressed && !g_keypad_btn[i].long_fired &&
		    (uint32_t)(now_ms - g_keypad_btn[i].press_ms) >= KEYPAD_LONG_PRESS_MS)
		{
			g_keypad_btn[i].long_fired  = 1U;
			g_keypad_btn[i].long_evt    = 1U;
			g_keypad_btn[i].long_evt_ms = now_ms;
		}

#if KEYPAD_EVENT_TTL_MS > 0
		if (g_keypad_btn[i].press_evt &&
		    (uint32_t)(now_ms - g_keypad_btn[i].press_evt_ms) >= KEYPAD_EVENT_TTL_MS)
			g_keypad_btn[i].press_evt = 0U;

		if (g_keypad_btn[i].long_evt &&
		    (uint32_t)(now_ms - g_keypad_btn[i].long_evt_ms) >= KEYPAD_EVENT_TTL_MS)
			g_keypad_btn[i].long_evt = 0U;
#endif
	}
}

/* 래치가 바뀌었거나 새로고침 주기가 되면 U32 에 1회 쓴다. */
static void led_update(uint32_t now_ms)
{
	uint8_t due = 0U;

#if KEYPAD_LED_FOLLOW_PRESS
	g_keypad_led_mask = g_keypad_mask;   /* 누른 키만 점등 */
#endif

	if (g_keypad_led_mask != s_led_written)
		due = 1U;
#if KEYPAD_LED_REFRESH_MS > 0
	else if ((uint32_t)(now_ms - s_last_led_ms) >= KEYPAD_LED_REFRESH_MS)
		due = 1U;
#endif

	if (!due)
		return;

	if (TCA9554_WriteOutput(&s_led, led_mask_to_port(g_keypad_led_mask)) == HAL_OK)
	{
		s_led_written = g_keypad_led_mask;
		s_last_led_ms = now_ms;
	}
	else
	{
		g_keypad_bus_err++;
	}
}

/* ---- 수명주기 -------------------------------------------------------- */
HAL_StatusTypeDef Keypad_Init(void)
{
	HAL_StatusTypeDef st_led, st_sw;
	uint32_t now  = HAL_GetTick();
	uint8_t  seed = 0x00U;
	uint8_t  i;

	g_keypad_mask     = 0x00U;
	g_keypad_bus_err  = 0U;
	g_keypad_led_mask = 0x00U;   /* 전 소등으로 출발(부팅 순간 전등 번쩍임 방지) */
	s_last_poll_ms    = now;
	s_last_change_ms  = now;
	s_last_led_ms     = now;

	/* U32(LED): 전 출력. 출력 래치를 먼저 쓰고 방향을 바꾸므로 POR(0xFF) 를
	 * 지나가며 LED 가 번쩍이지 않는다(TCA9554_Init 참조). */
	st_led = TCA9554_Init(&s_led, &hi2c1, TCA9554_U32_LED_ADDR,
	                      TCA9554_ALL_OUTPUTS, led_mask_to_port(g_keypad_led_mask));
	s_led_written = g_keypad_led_mask;

	/* U31(SW): 전 입력. */
	st_sw = TCA9554_Init(&s_sw, &hi2c1, TCA9554_U31_SW_ADDR,
	                     TCA9554_ALL_INPUTS, 0x00U);

#if !KEYPAD_SW_ACTIVE_HIGH
	/* ACTIVE-LOW 키패드: 극성반전으로 "누름 = 1" 정규화. */
	if (st_sw == HAL_OK)
		st_sw = TCA9554_SetPolarity(&s_sw, 0xFFU);
#endif

	/* 부팅 시 눌려 있던 키가 가짜 엣지를 내지 않게 현재 상태로 시드. */
	if (st_sw != HAL_OK || read_pressed(&seed) != HAL_OK)
		seed = 0x00U;

	g_keypad_mask = seed;
	for (i = 0; i < KEYPAD_KEY_COUNT; i++)
	{
		uint8_t level = (uint8_t)((seed >> i) & 1U);

		s_cand[i]    = level;
		s_cand_ms[i] = now;

		g_keypad_btn[i].pressed      = level;
		g_keypad_btn[i].press_evt    = 0U;
		g_keypad_btn[i].long_evt     = 0U;
		g_keypad_btn[i].long_fired   = level;   /* 눌린 채 부팅 = 길게누름 금지 */
		g_keypad_btn[i].press_ms     = now;
		g_keypad_btn[i].press_evt_ms = now;
		g_keypad_btn[i].long_evt_ms  = now;
		g_keypad_btn[i].press_cnt    = 0U;
	}

	/* 시드 읽기가 INT 를 이미 해제했으므로 남은 플래그는 버린다. */
	gpio_ctrl_exti_flag_clear(GPIO_EXTI_HALL_INT2);

	s_inited = 1U;

	if (st_led != HAL_OK)
		return st_led;
	return st_sw;
}

uint8_t Keypad_IsPresent(void)
{
	return (uint8_t)(TCA9554_IsPresent(&s_sw) && TCA9554_IsPresent(&s_led));
}

void Keypad_Tick(uint32_t now_ms)
{
	uint8_t sample = 0x00U;
	uint8_t due;

	if (!s_inited)
		return;

	/* U31 INT(PF10, 오픈드레인 액티브로우)는 입력이 "바뀌었다"만 알려준다.
	 * 어느 방향인지는 포트를 읽어야 알 수 있고, 읽는 순간 INT 가 풀린다 -
	 * HallSensor_ServiceInt() 와 같은 구조다. 폴 주기 전이라도 INT 가 떴으면
	 * 즉시 읽어 반응 지연을 줄인다. */
	due = gpio_ctrl_exti_flag_get(GPIO_EXTI_HALL_INT2);
	if (due)
	{
		gpio_ctrl_exti_flag_clear(GPIO_EXTI_HALL_INT2);
		s_last_change_ms = now_ms;   /* 활성 구간 진입 */
	}

	if (!due)
	{
		/* 활성(누른 키가 있거나 방금 변화가 있었다) 여부로 폴 주기를 고른다.
		 * 유휴 구간에서 MotorTask 에 블로킹 I2C 를 매번 넣지 않기 위함이다 -
		 * keypad.h 의 "버스 폴 주기" 주석 참조. */
		uint8_t  active   = (uint8_t)(g_keypad_mask != 0U ||
		                    (uint32_t)(now_ms - s_last_change_ms) < KEYPAD_ACTIVE_MS);
		uint32_t period   = active ? KEYPAD_POLL_MS : KEYPAD_IDLE_POLL_MS;

		due = (uint8_t)((uint32_t)(now_ms - s_last_poll_ms) >= period);
	}

	if (due)
	{
		s_last_poll_ms = now_ms;
		if (read_pressed(&sample) == HAL_OK)
			scan(sample, now_ms);
		else
			g_keypad_bus_err++;   /* 버스 딸꾹질: 이번 사이클은 건너뛴다 */
	}

	age_events(now_ms);
	led_update(now_ms);
}

/* ---- 입력 조회 -------------------------------------------------------- */
uint8_t Keypad_GetMask(void)
{
	return g_keypad_mask;
}

uint8_t Keypad_IsPressed(uint8_t key)
{
	if (key >= KEYPAD_KEY_COUNT)
		return 0U;
	return g_keypad_btn[key].pressed;
}

uint32_t Keypad_HeldMs(uint8_t key, uint32_t now_ms)
{
	if (key >= KEYPAD_KEY_COUNT || !g_keypad_btn[key].pressed)
		return 0U;
	return (uint32_t)(now_ms - g_keypad_btn[key].press_ms);
}

uint8_t Keypad_TakePress(uint8_t key)
{
	if (key >= KEYPAD_KEY_COUNT || !g_keypad_btn[key].press_evt)
		return 0U;

	g_keypad_btn[key].press_evt = 0U;
	return 1U;
}

uint8_t Keypad_TakeLongPress(uint8_t key)
{
	if (key >= KEYPAD_KEY_COUNT || !g_keypad_btn[key].long_evt)
		return 0U;

	g_keypad_btn[key].long_evt = 0U;
	return 1U;
}

void Keypad_FlushEvents(void)
{
	uint8_t i;

	for (i = 0; i < KEYPAD_KEY_COUNT; i++)
	{
		g_keypad_btn[i].press_evt = 0U;
		g_keypad_btn[i].long_evt  = 0U;
	}
}

/* ---- LED -------------------------------------------------------------- */
uint8_t Keypad_GetLedMask(void)
{
	return g_keypad_led_mask;
}

HAL_StatusTypeDef Keypad_SetLed(uint8_t key, uint8_t on)
{
	if (key >= KEYPAD_KEY_COUNT)
		return HAL_ERROR;

	if (on)
		g_keypad_led_mask |= (uint8_t)(1U << key);
	else
		g_keypad_led_mask &= (uint8_t)~(1U << key);

	return HAL_OK;   /* 실제 기록은 다음 Keypad_Tick() 의 led_update() */
}

HAL_StatusTypeDef Keypad_SetLedMask(uint8_t mask)
{
	g_keypad_led_mask = mask;
	return HAL_OK;
}
