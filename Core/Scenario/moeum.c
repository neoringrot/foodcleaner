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
 *   - 리미트   : WDoor_ReachedClose()(PF3) / WDoor_ReachedOpen()(PF4).
 *               EXTI 하강엣지 래치 OR 레벨. 이동 시작마다 WDoor_LimitArm()으로
 *               무장 — 수위(WATER_SEN)와 동일한 엣지 처리 방식.
 *   - 수위     : GPIO_EXTI_WATER_SEN1/2 (PF6/PF7).
 *   - 시작입력 : HallSensor_Get(MOEUM_HS_START_IDX)  (HS5=모음, 2026-08-25 재정의).
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
	/* ★교반 서브-FSM 플래그부터 내린다. BldcCtrl_Stop() 만으로는 모자란다 -
	 * stir_active 가 남아 있으면 다음 MotorTick 에서 moeum_stir_tick() 이 다시
	 * BldcCtrl_Start() 를 걸어 모터가 계속 돈다.
	 * 실측: 에러 발생 후에도 교반이 멈추지 않았다. 비상정지 경로는 호출부에서
	 * 따로 내리고 있었지만 에러/완료(MOEUM_ERROR/MOEUM_DONE) 진입 경로가 이것을
	 * 빠뜨렸다. 빠뜨릴 수 없도록 "전부 끈다"의 정의 안으로 옮긴다. */
	g_moeum.stir_active = 0U;
	BldcCtrl_Stop(&g_stir_ctrl);
	moeum_fill_off();
	WDoor_Stop();       /* coast */
	WDoor_Disable();    /* VM off */
}

/* 교반을 지정 방향으로 MOEUM_STIR_OUT_RPM(30)으로 기동. BldcCtrl_Start는 spd_idx의
 * 래더값(20)을 target으로 넣으므로, 직후 목표를 덮어써 PI가 30으로 슬루하도록 한다. */
static void moeum_stir_spin(uint8_t reverse)
{
	BldcCtrl_Stop(&g_stir_ctrl);           /* 방향 전환은 정지 후에만 가능    */
	BldcCtrl_Start(&g_stir_ctrl, reverse); /* 0=CW, 1=CCW                     */
	g_stir_ctrl.target_out_rpm = (uint16_t)MOEUM_STIR_OUT_RPM;
}

/* 교반 서브-FSM 시작(사양대로 CW부터). */
static void moeum_stir_begin(MoeumCtx *c, uint32_t now)
{
	c->stir_active      = 1U;
	c->stir_phase       = (uint8_t)MOEUM_STIR_CW;
	c->stir_phase_since = now;
	c->stir_cycles      = 0U;
	moeum_stir_spin(0U);                   /* CW                             */
}

static void moeum_stir_end(MoeumCtx *c)
{
	c->stir_active = 0U;
	BldcCtrl_Stop(&g_stir_ctrl);
}

/* 교반 1cycle = CW 4s -> 정지(딜레이) 1s -> CCW 4s = 9s. MOEUM_STIR_CYCLES(22) 반복.
 * CCW 가 끝나는 지점이 cycle 경계이며 그때 stir_cycles++ 한다.
 * stir_active 인 동안 매 MotorTick 호출된다(STIR/DRAIN_OPEN/DRAIN_WAIT 공용). */
static void moeum_stir_tick(MoeumCtx *c, uint32_t now)
{
	uint32_t el = now - c->stir_phase_since;

	switch ((MoeumStirPhase)c->stir_phase)
	{
	case MOEUM_STIR_CW:
		if (el >= (uint32_t)MOEUM_STIR_CW_MS)
		{
			BldcCtrl_Stop(&g_stir_ctrl);       /* 정지(딜레이) 구간 진입     */
			c->stir_phase       = (uint8_t)MOEUM_STIR_DELAY;
			c->stir_phase_since = now;
		}
		break;

	case MOEUM_STIR_DELAY:
		if (el >= (uint32_t)MOEUM_STIR_DELAY_MS)
		{
			moeum_stir_spin(1U);               /* CCW 기동                   */
			c->stir_phase       = (uint8_t)MOEUM_STIR_CCW;
			c->stir_phase_since = now;
		}
		break;

	case MOEUM_STIR_CCW:
	default:
		if (el >= (uint32_t)MOEUM_STIR_CCW_MS)
		{
			c->stir_cycles++;                  /* 1 cycle(9s) 완료           */
			moeum_stir_spin(0U);               /* 다음 cycle: CW 재기동       */
			c->stir_phase       = (uint8_t)MOEUM_STIR_CW;
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
		WDoor_LimitArm();                      /* stale 리미트 엣지 제거      */
		WDoor_Enable();
		WDoor_Close((uint8_t)WDOOR_CLOSE_DUTY); /* 닫힘: 80% 고정 (wdoor.h)   */
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
		WDoor_LimitArm();                      /* stale 리미트 엣지 제거      */
		WDoor_Enable();
		WDoor_Open(WDoor_OpenDutyAt(0U));      /* 열림: 킥 80% (tick에서 갱신) */
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
	g_moeum.abort_req     = 0U;
	g_moeum.dbg_force_stop = 0U;
	g_moeum.lid_guard     = 0U;
	g_moeum.lid_low_cnt   = 0U;
	g_moeum.stir_active   = 0U;
	g_moeum.stir_phase    = (uint8_t)MOEUM_STIR_CW;   /* 사이클 첫 위상 */
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
	g_moeum.abort_req   = 0U;
	g_moeum.dbg_force_stop = 0U;
	g_moeum.lid_guard   = 0U;
	g_moeum.lid_low_cnt = 0U;
	g_moeum.state       = (uint8_t)MOEUM_IDLE;
	g_moeum.state_since = HAL_GetTick();
}

/* 4.5 정지 요청(외부 트리거용: 앱 PROTO_ACT_STOP / 디버거. 맴브레인 버튼 제어는
 * 2026-08-18 폐지). BUSY일 때만 유효 - MotorTick가 소비하여 즉시 안전정지 후
 * IDLE(모음은 히터 없어 식힘 불필요). */
void Moeum_RequestStop(void)
{
	g_moeum.abort_req = 1U;
}

/* 100ms, StartDefaultTask - 센서만. 상태 전이/모터 구동은 하지 않는다. */
void Moeum_SenseTick(void)
{
#if MOEUM_HS_TRIGGER_INTERNAL
	/* 시작 트리거: HS5(모음) 눌림 상승에지  OR  디버거 강제(dbg_force_start).
	 * 둘 중 무엇이든 Moeum_Start()로 수렴 -> IDLE일 때만 start_req 래치.
	 * (디버거에서 g_moeum.start_req=1 을 직접 세팅해도 MotorTick가 동일하게 소비) */
	uint8_t hs      = HallSensor_Get((uint8_t)MOEUM_HS_START_IDX);
	uint8_t hs_edge = (uint8_t)((hs != 0U) && (g_moeum.hs_prev == 0U));
	uint8_t force   = g_moeum.dbg_force_start;
	if (hs_edge || force)
	{
		g_moeum.dbg_force_start = 0U;   /* 강제 트리거는 1회성 */
		if (g_moeum.state == (uint8_t)MOEUM_IDLE)
		{
			/* 실제 HS 상승에지 시작만 투입구 감시 무장(벤치 강제시작 제외). */
			g_moeum.lid_guard   = (uint8_t)(hs_edge && (force == 0U));
			g_moeum.lid_low_cnt = 0U;
		}
		Moeum_Start();
	}
	g_moeum.hs_prev = hs;

#if MOEUM_LID_OPEN_ABORT
	/* 4.5 투입구 개방 감시: 무장 상태에서 처리 중 마개가 모음 위치를 벗어나면
	 * (HS LOW 연속 N회) 비상정지 요청. */
	if (g_moeum.lid_guard && Moeum_IsBusy())
	{
		if (hs == 0U)
		{
			if (g_moeum.lid_low_cnt < 255U) { g_moeum.lid_low_cnt++; }
			if (g_moeum.lid_low_cnt >= (uint8_t)MOEUM_LID_CONFIRM_SAMPLES)
			{ g_moeum.abort_req = 1U; }
		}
		else { g_moeum.lid_low_cnt = 0U; }
	}
#endif
#else  /* !MOEUM_HS_TRIGGER_INTERNAL - 중재자(mode_arbiter.c)가 트리거를 소유 */
	/* HS5 상승에지 시작과 마개 이탈 감시는 중재자가 HS1~5를 통째로 디코딩해
	 * 처리한다(Moeum_Start() / Jungji_Request(HS_LOST)). 여기서 HS를 또 읽으면
	 * 같은 이벤트를 두 번 처리하게 되므로 읽지 않는다. hs_prev/lid_guard 는
	 * 이 경로에서 쓰이지 않아 stale 될 일도 없다.
	 * 벤치 강제 시작만 그대로 유효(HS 없이 시나리오를 돌려보는 용도). */
	if (g_moeum.dbg_force_start != 0U)
	{
		g_moeum.dbg_force_start = 0U;
		g_moeum.lid_guard       = 0U;   /* 강제 시작은 투입구 감시 무장 안 함 */
		g_moeum.lid_low_cnt     = 0U;
		Moeum_Start();
	}
#endif
	/* 4.5 정지 모사: 벤치/외부 트리거로 dbg_force_stop 세팅 시 정지(디버거 전용 훅). */
	if (g_moeum.dbg_force_stop != 0U)
	{
		g_moeum.dbg_force_stop = 0U;
		g_moeum.abort_req = 1U;
	}

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

	/* 4.5 비상정지 요청 소비: 처리 중이면 전 모터/밸브 정지 후 IDLE(히터 없어 식힘
	 * 불필요). 교반 tick보다 먼저 처리해 정지 틱에 교반을 다시 돌리지 않는다. */
	if (c->abort_req)
	{
		c->abort_req = 0U;
		if (Moeum_IsBusy())
		{
			moeum_all_off();
			c->stir_active = 0U;
			c->lid_guard   = 0U;
			c->lid_low_cnt = 0U;
			/* TODO(음성): "처리 중단/추가 투입 불가" 안내 멘트 (미구현) */
			moeum_enter(c, MOEUM_IDLE, now_ms);
		}
	}

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
		/* 닫힘은 시간이 정상 종료 조건(WDOOR_CLOSE_MS 4.2s). 그 전에 WHALL-CLOSE가
		 * 인식되면 거기서 멈춘다 -- 테스트벤치 tb_wdoor(reverse=0)와 동일. */
		if (WDoor_ReachedClose() || (el >= (uint32_t)WDOOR_CLOSE_MS))
		{
			WDoor_Stop();
			WDoor_Disable();
			moeum_enter(c, MOEUM_FILL, now_ms);
		}
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
		/* 열림은 WHALL-OPEN 인식이 정상 종료 조건, WDOOR_OPEN_MAX_MS(6s)는 미인식
		 * 대비 상한. duty는 킥(80%,2s) -> 유지(65%)로 매 tick 재지령한다 --
		 * 테스트벤치 tb_wdoor(reverse=1)와 동일. */
		if (WDoor_ReachedOpen() || (el >= (uint32_t)WDOOR_OPEN_MAX_MS))
		{
			WDoor_Stop();
			WDoor_Disable();
			c->drain_open_since = now_ms;
			moeum_enter(c, MOEUM_DRAIN_WAIT, now_ms);
		}
		else
		{
			WDoor_Open(WDoor_OpenDutyAt(el));
		}
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
