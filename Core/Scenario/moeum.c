/* ==========================================================================
 * moeum.c - "모음"(1단계 헹굼 세척) 시나리오 상태머신 구현.
 * 설계/시퀀스 개요는 moeum.h 참조. 코드 미검증 값은 moeum.h의 MOEUM_* 매크로로
 * 분리되어 있어 벤치에서 상수만 조정하면 된다.
 *
 * 하드웨어 소유권:
 *   - 배수문   : WDoor (U5, DRV8871).  Enable/Close/Open/Stop/Disable.
 *   - 급수밸브 : GPIO_OUT_VALVE_DRY_IN (PB13, VALVE-DRY-IN, 건조통 급수)
 *               + GPIO_OUT_WATER_ON (PE2, 급수 펌프/메인 enable). 둘 함께 ON
 *               (수위센서 검증이 PE2 ON 상태에서 통수됨; MOEUM_FILL_USE_WATER_ON).
 *   - 교반     : g_stir_ctrl (M2/U16, BLDC 폐루프). Start(reverse)/Stop + target.
 *   - 리미트   : WDoor_AtClose()(PF3) / WDoor_AtOpen()(PF4).
 *   - 수위     : GPIO_EXTI_WATER_SEN1/2 (PF6/PF7).
 *   - 시작입력 : HallSensor_Get(MOEUM_HS_START_IDX)  (docx: HS2=모음).
 * ========================================================================== */

#include "moeum.h"

#include "gpio_ctrl.h"
#include "wdoor.h"
#include "bldc_ctrl.h"
#include "hallsensor.h"

MoeumCtx g_moeum;

/* ---- 내부 헬퍼 ----------------------------------------------------------- */

/* 급수 밸브 세트 ON/OFF. 모음 급수는 VALVE_DRY_IN(PB13, 건조통 급수)에
 * WATER_ON(PE2, 급수 펌프/메인 enable)을 함께 쓴다 — 수위센서 검증이 PE2 ON
 * 상태에서 통수·감지되었기 때문(MOEUM_FILL_USE_WATER_ON로 분리). */
static void moeum_fill_on(void)
{
	gpio_ctrl_on(GPIO_OUT_VALVE_DRY_IN);
#if MOEUM_FILL_USE_WATER_ON
	gpio_ctrl_on(GPIO_OUT_WATER_ON);
#endif
}

static void moeum_fill_off(void)
{
	gpio_ctrl_off(GPIO_OUT_VALVE_DRY_IN);
#if MOEUM_FILL_USE_WATER_ON
	gpio_ctrl_off(GPIO_OUT_WATER_ON);
#endif
}

/* 모든 액추에이터 안전 정지. 배수문은 위치를 그대로 유지(웜기어 자기유지)하고
 * 모터/밸브 전원만 내린다. DONE/ERROR/Abort 공통 경로. */
static void moeum_all_off(void)
{
	BldcCtrl_Stop(&g_stir_ctrl);
	moeum_fill_off();
	WDoor_Stop();       /* coast */
	WDoor_Disable();    /* VM off */
}

/* 교반을 지정 방향으로 25RPM 기동. BldcCtrl_Start는 spd_idx의 래더값(20)을
 * target으로 넣으므로, 직후 목표를 25로 덮어써 PI가 25로 슬루하도록 한다. */
static void moeum_stir_spin(uint8_t reverse)
{
	BldcCtrl_Stop(&g_stir_ctrl);           /* 방향 전환은 정지 후에만 가능    */
	BldcCtrl_Start(&g_stir_ctrl, reverse); /* 0=CW, 1=CCW                     */
	g_stir_ctrl.target_out_rpm = (uint16_t)MOEUM_STIR_OUT_RPM;
}

/* 교반 서브-FSM 시작(CCW부터). */
static void moeum_stir_begin(MoeumCtx *c, uint32_t now)
{
	c->stir_active      = 1U;
	c->stir_phase       = (uint8_t)MOEUM_STIR_CCW;
	c->stir_phase_since = now;
	c->stir_cycles      = 0U;
	moeum_stir_spin(1U);                   /* CCW                            */
}

static void moeum_stir_end(MoeumCtx *c)
{
	c->stir_active = 0U;
	BldcCtrl_Stop(&g_stir_ctrl);
}

/* 교반 CCW 3s -> 정지 1s -> CW 3s 를 반복. 한 cycle(=7s) 완료 시 stir_cycles++.
 * stir_active 인 동안 매 MotorTick 호출된다(STIR/DRAIN_OPEN/DRAIN_WAIT 공용). */
static void moeum_stir_tick(MoeumCtx *c, uint32_t now)
{
	uint32_t el = now - c->stir_phase_since;

	switch ((MoeumStirPhase)c->stir_phase)
	{
	case MOEUM_STIR_CCW:
		if (el >= (uint32_t)MOEUM_STIR_CCW_MS)
		{
			BldcCtrl_Stop(&g_stir_ctrl);       /* 정지 구간 진입             */
			c->stir_phase       = (uint8_t)MOEUM_STIR_DELAY;
			c->stir_phase_since = now;
		}
		break;

	case MOEUM_STIR_DELAY:
		if (el >= (uint32_t)MOEUM_STIR_DELAY_MS)
		{
			moeum_stir_spin(0U);               /* CW 기동                    */
			c->stir_phase       = (uint8_t)MOEUM_STIR_CW;
			c->stir_phase_since = now;
		}
		break;

	case MOEUM_STIR_CW:
	default:
		if (el >= (uint32_t)MOEUM_STIR_CW_MS)
		{
			c->stir_cycles++;                  /* 1 cycle 완료               */
			moeum_stir_spin(1U);               /* 다음 cycle: CCW 재기동      */
			c->stir_phase       = (uint8_t)MOEUM_STIR_CCW;
			c->stir_phase_since = now;
		}
		break;
	}
}

/* 수위 감지 판정. 폴리시(에지/레벨/극성) 미확정 - 하드웨어결정 §1-1.
 * EXTI 하강에지 플래그 OR 설정 활성레벨을 "물 도달"로 본다. */
static uint8_t moeum_water_present(void)
{
	if (gpio_ctrl_exti_flag_get(GPIO_EXTI_WATER_SEN1) ||
	    gpio_ctrl_exti_flag_get(GPIO_EXTI_WATER_SEN2))
	{
		return 1U;
	}
#if MOEUM_WATER_USE_LEVEL
#if MOEUM_WATER_ACTIVE_LOW
	return (uint8_t)((gpio_ctrl_exti_read(GPIO_EXTI_WATER_SEN1) == 0U) ||
	                 (gpio_ctrl_exti_read(GPIO_EXTI_WATER_SEN2) == 0U));
#else
	return (uint8_t)((gpio_ctrl_exti_read(GPIO_EXTI_WATER_SEN1) == 1U) ||
	                 (gpio_ctrl_exti_read(GPIO_EXTI_WATER_SEN2) == 1U));
#endif
#else
	return 0U;   /* 에지 전용(기본): 레벨 폴백 비활성 */
#endif
}

/* 상태 진입 + 진입 액션. state_since 갱신은 여기서 일괄 처리. */
static void moeum_enter(MoeumCtx *c, MoeumState s, uint32_t now)
{
	c->state       = (uint8_t)s;
	c->state_since = now;

	switch (s)
	{
	case MOEUM_DOOR_CLOSE:
		WDoor_Enable();
		WDoor_Close((uint8_t)MOEUM_DOOR_DUTY_PCT);
		break;

	case MOEUM_FILL:
		c->water_reached = 0U;                 /* 잔류 감지 무시             */
		gpio_ctrl_exti_flag_clear(GPIO_EXTI_WATER_SEN1);
		gpio_ctrl_exti_flag_clear(GPIO_EXTI_WATER_SEN2);
		moeum_fill_on();                       /* 급수 ON(PB13 + PE2)       */
		break;

	case MOEUM_FILL_EXTRA:
		/* 밸브는 계속 ON, 2초 후 OFF (아래 tick) */
		break;

	case MOEUM_STIR:
		moeum_stir_begin(c, now);
		break;

	case MOEUM_DRAIN_OPEN:
		WDoor_Enable();
		WDoor_Open((uint8_t)MOEUM_DOOR_DUTY_PCT);
		break;                                 /* 교반은 계속 진행           */

	case MOEUM_DRAIN_WAIT:
		break;                                 /* 교반 지속, 1분 대기        */

	case MOEUM_DONE:
	case MOEUM_ERROR:
		moeum_all_off();                       /* 배수문은 열린 위치 유지     */
		break;

	case MOEUM_IDLE:
	default:
		break;
	}
}

/* ---- API ----------------------------------------------------------------- */

void Moeum_Init(void)
{
	g_moeum.state         = (uint8_t)MOEUM_IDLE;
	g_moeum.state_since   = 0U;
	g_moeum.start_req     = 0U;
	g_moeum.dbg_force_start = 0U;
	g_moeum.water_reached = 0U;
	g_moeum.hs_prev       = 0U;
	g_moeum.stir_active   = 0U;
	g_moeum.stir_phase    = (uint8_t)MOEUM_STIR_CCW;
	g_moeum.stir_phase_since = 0U;
	g_moeum.stir_cycles   = 0U;
	g_moeum.drain_open_since = 0U;
	moeum_all_off();
}

void Moeum_Start(void)
{
	/* IDLE에서만 수락. 진행 중 재요청은 무시. */
	if (g_moeum.state == (uint8_t)MOEUM_IDLE)
	{
		g_moeum.start_req = 1U;
	}
}

void Moeum_Abort(void)
{
	moeum_all_off();
	g_moeum.stir_active = 0U;
	g_moeum.start_req   = 0U;
	g_moeum.state       = (uint8_t)MOEUM_IDLE;
	g_moeum.state_since = HAL_GetTick();
}

/* 100ms, StartDefaultTask - 센서만. 상태 전이/모터 구동은 하지 않는다. */
void Moeum_SenseTick(void)
{
	/* 시작 트리거: HS2(모음) 눌림 상승에지  OR  디버거 강제(dbg_force_start).
	 * 둘 중 무엇이든 Moeum_Start()로 수렴 -> IDLE일 때만 start_req 래치.
	 * (디버거에서 g_moeum.start_req=1 을 직접 세팅해도 MotorTick가 동일하게 소비) */
	uint8_t hs      = HallSensor_Get((uint8_t)MOEUM_HS_START_IDX);
	uint8_t hs_edge = (uint8_t)((hs != 0U) && (g_moeum.hs_prev == 0U));
	if (hs_edge || (g_moeum.dbg_force_start != 0U))
	{
		g_moeum.dbg_force_start = 0U;   /* 강제 트리거는 1회성 */
		Moeum_Start();
	}
	g_moeum.hs_prev = hs;

	/* 수위 감지 래치(MotorTick가 FILL 진입 시 클리어) */
	if (moeum_water_present())
	{
		g_moeum.water_reached = 1U;
	}
}

/* 1ms, StartMotorTask - 상태머신 소유 + 모든 액추에이터 구동. */
void Moeum_MotorTick(uint32_t now_ms)
{
	MoeumCtx *c  = &g_moeum;
	uint32_t  el = now_ms - c->state_since;

	/* 교반 서브-FSM은 stir_active 인 모든 상태에서 지속 구동 */
	if (c->stir_active)
	{
		moeum_stir_tick(c, now_ms);
	}

	switch ((MoeumState)c->state)
	{
	case MOEUM_IDLE:
		if (c->start_req)
		{
			c->start_req = 0U;
			moeum_enter(c, MOEUM_DOOR_CLOSE, now_ms);
		}
		break;

	case MOEUM_DOOR_CLOSE:
		if (WDoor_AtClose())
		{
			WDoor_Stop();
			WDoor_Disable();
			moeum_enter(c, MOEUM_FILL, now_ms);
		}
#if MOEUM_DOOR_LIMIT_OPTIONAL
		else if (el >= (uint32_t)MOEUM_DOOR_BENCH_MS)
		{
			/* 육안모드: 방향 미확정으로 리미트 미도달일 수 있음 -> 구동을 보여준
			 * 뒤 ERROR 없이 다음 단계로 진행 (방향 확정 후 플래그 0으로) */
			WDoor_Stop();
			WDoor_Disable();
			moeum_enter(c, MOEUM_FILL, now_ms);
		}
#else
		else if (el >= (uint32_t)MOEUM_DOOR_TIMEOUT_MS)
		{
			moeum_enter(c, MOEUM_ERROR, now_ms);
		}
#endif
		break;

	case MOEUM_FILL:
		if (c->water_reached)
		{
			moeum_enter(c, MOEUM_FILL_EXTRA, now_ms);  /* 밸브 ON 유지        */
		}
		else if (el >= (uint32_t)MOEUM_FILL_TIMEOUT_MS)
		{
			moeum_fill_off();
			moeum_enter(c, MOEUM_ERROR, now_ms);
		}
		break;

	case MOEUM_FILL_EXTRA:
		if (el >= (uint32_t)MOEUM_FILL_EXTRA_MS)
		{
			moeum_fill_off();                          /* 추가급수 종료        */
			moeum_enter(c, MOEUM_STIR, now_ms);
		}
		break;

	case MOEUM_STIR:
		if (c->stir_cycles >= (uint16_t)MOEUM_STIR_CYCLES)
		{
			moeum_enter(c, MOEUM_DRAIN_OPEN, now_ms);   /* 교반 지속 채로 개방 */
		}
		break;

	case MOEUM_DRAIN_OPEN:
		if (WDoor_AtOpen())
		{
			WDoor_Stop();
			WDoor_Disable();
			c->drain_open_since = now_ms;
			moeum_enter(c, MOEUM_DRAIN_WAIT, now_ms);
		}
#if MOEUM_DOOR_LIMIT_OPTIONAL
		else if (el >= (uint32_t)MOEUM_DOOR_BENCH_MS)
		{
			/* 육안모드: 리미트 미도달이어도 구동 관찰 후 진행 (교반은 계속) */
			WDoor_Stop();
			WDoor_Disable();
			c->drain_open_since = now_ms;
			moeum_enter(c, MOEUM_DRAIN_WAIT, now_ms);
		}
#else
		else if (el >= (uint32_t)MOEUM_DOOR_TIMEOUT_MS)
		{
			moeum_enter(c, MOEUM_ERROR, now_ms);
		}
#endif
		break;

	case MOEUM_DRAIN_WAIT:
		if ((now_ms - c->drain_open_since) >= (uint32_t)MOEUM_DRAIN_STIR_MS)
		{
			moeum_stir_end(c);                          /* 1분 경과 -> 교반 정지 */
			moeum_enter(c, MOEUM_DONE, now_ms);
		}
		break;

	case MOEUM_DONE:
		/* 배수문 열린 상태로 종료. 다음 사이클 대기. */
		moeum_enter(c, MOEUM_IDLE, now_ms);
		break;

	case MOEUM_ERROR:
	default:
		/* 안전 정지 상태 유지. 복구는 Moeum_Abort()/Moeum_Init(). */
		break;
	}
}

MoeumState Moeum_GetState(void)
{
	return (MoeumState)g_moeum.state;
}

uint8_t Moeum_IsBusy(void)
{
	MoeumState s = (MoeumState)g_moeum.state;
	return (uint8_t)((s != MOEUM_IDLE) && (s != MOEUM_DONE) && (s != MOEUM_ERROR));
}
