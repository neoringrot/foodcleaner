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
 *   - 교반     : g_stir_ctrl (M2/U16) — ★개정4 ④: **rotation_port(RotStir)만 만진다**.
 *               WASH(본교반) → DRAIN(배수교반) 회전수 운전. 이 파일은 BldcCtrl 을 직접 안 부른다.
 *   - 리미트   : WDoor_ReachedClose()(PF3) / WDoor_ReachedOpen()(PF4).
 *               EXTI 하강엣지 래치 OR 레벨. 이동 시작마다 WDoor_LimitArm()으로
 *               무장 — 수위(WATER_SEN)와 동일한 엣지 처리 방식.
 *   - 수위     : GPIO_EXTI_WATER_SEN1/2 (PF6/PF7).
 *   - 시작입력 : HallSensor_Get(MOEUM_HS_START_IDX)  (HS5=모음, 2026-08-25 재정의).
 * ========================================================================== */

#include "moeum.h"
#include "dongjak.h"                /* ★G3: 에러 코드표(DjErrCode)·해제 분류 공유, 동작 래치 중 시작 금지 */
#include "voice.h"
#include "voice_table.h"

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
	/* ★교반 구동 주체부터 내린다. BldcCtrl_Stop() 만으로는 모자란다 -
	 * 구동 주체(개정4 ④부터 RotStir, 종전 stir_active+moeum_stir_tick)가 살아 있으면
	 * 다음 MotorTick 에서 다시 BldcCtrl_Start() 를 걸어 모터가 계속 돈다.
	 * 실측: 에러 발생 후에도 교반이 멈추지 않았다. 비상정지 경로는 호출부에서
	 * 따로 내리고 있었지만 에러/완료(MOEUM_ERROR/MOEUM_DONE) 진입 경로가 이것을
	 * 빠뜨렸다. 빠뜨릴 수 없도록 "전부 끈다"의 정의 안으로 옮긴다. */
	g_moeum.stir_active = 0U;
	RotStir_Stop(&g_moeum.rot);             /* ★개정4 ④: 비활성 + 교반 정지(슬립). 준비 탐색 교반도 */
	moeum_fill_off();
	WDoor_Stop();       /* coast */
	WDoor_Disable();    /* VM off */
}

/* ★R3 개정4 ④ [2026-09-21, §0.26] — 교반은 회전수 운전(rotation_port)이 구동한다.
 * 종전 moeum_stir_spin/begin/end/tick(정3/정지1/역3/정지1 × MOEUM_STIR_CYCLES)은 삭제했다.
 * 이 함수는 앱 보고(protocol_r0.c)가 읽는 옛 필드를 엔진 상태로 채우기만 한다. */
static void moeum_rot_mirror(MoeumCtx *c)
{
	c->stir_active      = RotStir_IsActive(&c->rot);
	c->stir_phase       = (uint8_t)((c->rot.out == 1U) ? MOEUM_STIR_CW
	                               : (c->rot.out == 2U) ? MOEUM_STIR_CCW
	                                                    : MOEUM_STIR_DELAY);
	c->stir_phase_since = c->rot.out_since;
	c->stir_cycles      = (uint16_t)RotStir_Legs(&c->rot);
}

/* 수위 감지 판정. 폴리시(에지/레벨/극성) 미확정 - 하드웨어결정 §1-1.
 * EXTI 하강에지 플래그 OR 설정 활성레벨을 "물 도달"로 본다. */
static uint8_t moeum_water_present(void)
{
	/* ★REV02: PF7 이 가이드 홀로 재배정되어 SEN2 가 없다 - SEN1(PF6) 단독 판정. */
	if (gpio_ctrl_exti_flag_get(GPIO_EXTI_WATER_SEN1))
	{
		return 1U;
	}
#if MOEUM_WATER_USE_LEVEL
#if MOEUM_WATER_ACTIVE_LOW
	return (uint8_t)(gpio_ctrl_exti_read(GPIO_EXTI_WATER_SEN1) == 0U);
#else
	return (uint8_t)(gpio_ctrl_exti_read(GPIO_EXTI_WATER_SEN1) == 1U);
#endif
#else
	return 0U;   /* 에지 전용(기본): 레벨 폴백 비활성 */
#endif
}

/* 상태 진입 + 진입 액션. state_since 갱신은 여기서 일괄 처리.
 * ★§0.30: 헹굼 구간(PREP~DRAIN_WAIT)은 rinse.c 가 진행하고 여기서는 **미러**로만 바뀐다
 * (moeum_rinse_mirror). 진입 액션이 있는 것은 DONE/ERROR 뿐이다. */
static void moeum_enter(MoeumCtx *c, MoeumState s, uint32_t now)
{
	c->state       = (uint8_t)s;
	c->state_since = now;

	switch (s)
	{
	case MOEUM_DONE:
	case MOEUM_ERROR:
		moeum_all_off();                       /* 배수문은 열린 위치 유지     */
		break;

	case MOEUM_IDLE:
	default:
		break;
	}
}

/* ★G3·G4 [2026-09-22 §0.33] MOEUM_ERROR 진입 + 원인코드(DjErrCode 값) 1곳 기록. */
static void moeum_fail(MoeumCtx *c, DjErrCode code, uint32_t now)
{
	c->err_code = (uint8_t)code;
	c->last_err = (uint8_t)code;
	c->err_clear_req = 0U;                 /* 에러 전에 들어온 해제 요청이 새 에러를 곧바로 풀지 않도록 */
	moeum_enter(c, MOEUM_ERROR, now);
}

/* ---- ★§0.30 공통 헹굼(rinse.c) 연결 -------------------------------------- */

/* rinse 명령 비트 적용. 적용 순서가 의미를 가진다: 교반 정지 → 회전수 운전 → 문 → 급수. */
static void moeum_rinse_apply(MoeumCtx *c, const RinseOutputs *o)
{
	if (o->cmd & RINSE_CMD_STIR_MANUAL) { RotStir_Manual(o->stir_dir, o->stir_rpm); }
	if (o->cmd & RINSE_CMD_STIR_STOP)   { RotStir_Stop(&c->rot); }
	if (o->cmd & RINSE_CMD_ROT_WASH)    { RotStir_Start (&c->rot, ZG_ROT_WASH,  o->rot_rpm); c->rot_st = ROT_ST_RUNNING; }
	if (o->cmd & RINSE_CMD_ROT_DRAIN)   { RotStir_Switch(&c->rot, ZG_ROT_DRAIN, o->rot_rpm); c->rot_st = ROT_ST_RUNNING; }

	if (o->cmd & RINSE_CMD_DOOR_STOP)   { WDoor_Stop(); WDoor_Disable(); }
	if (o->cmd & RINSE_CMD_DOOR_CLOSE)
	{
		WDoor_LimitArm();                          /* stale 리미트 엣지 제거      */
		WDoor_Enable();
		WDoor_Close((uint8_t)WDOOR_CLOSE_DUTY);    /* 닫힘: 80% 고정 (wdoor.h)   */
	}
	if (o->cmd & RINSE_CMD_DOOR_OPEN)
	{
		WDoor_LimitArm();
		WDoor_Enable();
		WDoor_Open(o->door_duty);                  /* 열림: 킥 80% (DUTY 로 갱신) */
	}
	if (o->cmd & RINSE_CMD_DOOR_DUTY)   { WDoor_Open(o->door_duty); }

	if (o->cmd & RINSE_CMD_FILL_ON)
	{
		c->water_reached = 0U;                     /* 잔류 감지 무시             */
		gpio_ctrl_exti_flag_clear(GPIO_EXTI_WATER_SEN1);
		moeum_fill_on();                           /* 급수 ON(PB13 + PE2)       */
	}
	if (o->cmd & RINSE_CMD_FILL_OFF)    { moeum_fill_off(); }
}

/* 앱·watch 가 보는 MoeumState 를 rinse 단계에서 채운다(값 1~6·9 불변 — 재배열은 8단계).
 * MOEUM_FILL_EXTRA(3)는 더 이상 나타나지 않는다(C017 추가급수 0 s). 진단 필드도 복사한다. */
static void moeum_rinse_mirror(MoeumCtx *c, uint32_t now)
{
	MoeumState m;
	switch ((RinseStep)c->rinse.step)
	{
	case RINSE_CLOSE:      m = MOEUM_DOOR_CLOSE; break;
	case RINSE_FILL:       m = MOEUM_FILL;       break;
	case RINSE_WASH:       m = MOEUM_STIR;       break;
	case RINSE_DRAIN_OPEN: m = MOEUM_DRAIN_OPEN; break;
	case RINSE_DRAIN:      m = MOEUM_DRAIN_WAIT; break;
	default:               m = MOEUM_PREP;       break;   /* PREP_* (종료 단계는 호출부가 처리) */
	}
	if (c->state != m) { c->state = m; c->state_since = now; }

	c->door_close_by    = c->rinse.door_close_by;
	c->door_open_by     = c->rinse.door_open_by;
	c->door_close_ms    = c->rinse.door_close_ms;
	c->door_open_ms     = c->rinse.door_open_ms;
	c->drain_open_since = c->rinse.drain_open_since;
}

/* 헹굼 구간 한 tick. 종료(완료/에러)면 DONE/ERROR 로 보낸다. */
static void moeum_rinse_step(MoeumCtx *c, uint32_t now)
{
	RinseInputs  in;
	RinseOutputs o;
	RinseStep    st;

	/* ★개정4 ④: 회전수 운전(WASH/DRAIN)은 rinse 보다 먼저 한 tick 진행한다. 비활성이면 IDLE. */
	c->rot_st = RotStir_Tick(&c->rot, true, now);
	moeum_rot_mirror(c);

	in.door_closed = WDoor_ReachedClose();
	in.door_open   = WDoor_ReachedOpen();
	in.water       = c->water_reached;
	in.rot_st      = c->rot_st;
	in.stir_pos    = BldcCtrl_Position(&g_stir_ctrl);   /* §0.38 순이동 감시 */
	in.stir_rpm    = g_stir_ctrl.meas_out_rpm;          /* §0.41 과속 보조 A */
	st = Rinse_Tick(&c->rinse, &in, now, &o);
	moeum_rinse_apply(c, &o);

	/* ★2026-09-22 음성 05-04/05-05 "배수부 문 닫힘/열림 위치가 확인되지 않습니다. 점검해 주세요".
	 * **경고만 하고 진행한다** — 녹음에 "멈춥니다"가 없고 시간 종료가 설계상 정상 경로다(wdoor.h).
	 * 열림 경고와 01-01(정상)은 같은 tick 에 요청되지만 에러는 FIFO·정상은 1칸으로 따로 쌓여
	 * 05-05 → 01-01 순서로 둘 다 나간다. */
	if (o.ev & RINSE_EV_CLOSE_TMO) { (void)Voice_Play(VOICE_E_WDOOR_CLOSE, VOICE_PRIO_ERROR); }
	if (o.ev & RINSE_EV_OPEN_TMO)  { (void)Voice_Play(VOICE_E_WDOOR_OPEN,  VOICE_PRIO_ERROR); }
	/* 음성: "30초간 배수 타임이 시작됩니다".
	 * ⚠ 개정4 ④ 부터 배수 구간은 30초가 아니라 **DRAIN 10회 운전(≈70초)** 이다 —
	 * 녹음 01-01 의 "30초" 와 어긋난다. 녹음 교체/사양 확인은 N8(7단계) 로 넘긴다. */
	if (o.ev & RINSE_EV_DRAIN_OPENED) { (void)Voice_Play(VOICE_DRAIN_START, VOICE_PRIO_NORMAL); }

	moeum_rinse_mirror(c, now);                    /* 종료 단계면 아래에서 DONE/ERROR 로 덮는다 */
	switch (st)
	{
	case RINSE_FINISHED:
		moeum_rot_mirror(c);
		/* 음성 2개를 **이어서**: "배수 타임이 끝났습니다"(1.7s) →
		 * "세척 완료 및 배수가 끝났습니다…"(7.8s). 합 약 9.5초.
		 * ★2026-09-22: Voice_Play 를 두 번 부르면 정상 큐(1칸)가 덮어써져
		 *   앞의 DRAIN_END 가 빠졌다. Voice_PlaySeq2 로 한 번에 순서를 걸었다. */
		(void)Voice_PlaySeq2(VOICE_DRAIN_END, VOICE_MOEUM_DONE);
		moeum_enter(c, MOEUM_DONE, now);           /* 배수문 열린 상태로 종료     */
		break;

	case RINSE_PREP_FAULT:
		/* E01 교반가이드 미확인. 원인은 g_moeum.rinse.step == RINSE_PREP_FAULT 로 본다.
		 * (이 시점엔 급수·배수문이 아직 안 움직였으므로 MOEUM_ERROR 의 전부 정지와 R3
		 *  절대조건 ① "교반만 차단"의 결과가 같다.) rinse 관측값은 다음 시작까지 남는다. */
		(void)Voice_Play(VOICE_E_GUIDE, VOICE_PRIO_ERROR);
		moeum_fail(c, DJ_ERR_GUIDE, now);          /* E01 — 해제형 */
		break;

	case RINSE_FAULT:
		if (c->rinse.fault == (uint8_t)RINSE_FAULT_FILL)
		{
			(void)Voice_Play(VOICE_E_FILL, VOICE_PRIO_ERROR);    /* "급수가 확인되지 않습니다" */
			moeum_fail(c, DJ_ERR_RINSE_FILL, now);                /* E04 — 래치형 */
		}
		else if (c->rinse.fault == (uint8_t)RINSE_FAULT_GUIDE_LOST)
		{
			(void)Voice_Play(VOICE_E_GUIDE, VOICE_PRIO_ERROR);      /* 05-01 — 운전 중 가이드 소실(§0.38) */
			moeum_fail(c, DJ_ERR_GUIDE, now);                     /* E01 — 해제형 */
		}
		else if (c->rinse.fault == (uint8_t)RINSE_FAULT_OVERSPEED)
		{
			(void)Voice_Play(VOICE_E_STIR_SPEED, VOICE_PRIO_ERROR); /* 05-02 */
			moeum_fail(c, DJ_ERR_OVERSPEED, now);                 /* E02 — 해제형(★§0.33) */
		}
		else
		{
			/* 회전수 운전 실패·cnt 미도달(E09 — 위치 무진행·정지 미확인·역방향 이동·count_mismatch).
			 * 세부 원인은 g_moeum.rinse.fault / g_moeum.rot.failed 로 본다.
			 * 음성: "교반 모터 상태를 확인해 주세요" — 검토서 §13.3 06-01 = E09-교반. */
			(void)Voice_Play(VOICE_E_M_STIR, VOICE_PRIO_ERROR);
			moeum_fail(c, DJ_ERR_ROTATION, now);                  /* E09 — 래치형 */
		}
		break;

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
	g_moeum.err_code      = 0U;
	g_moeum.err_clear_req = 0U;
	g_moeum.last_err      = 0U;
	g_moeum.dbg_force_stop = 0U;
	g_moeum.lid_guard     = 0U;
	g_moeum.lid_low_cnt   = 0U;
	g_moeum.stir_active   = 0U;
	g_moeum.stir_phase    = (uint8_t)MOEUM_STIR_DELAY; /* [미러] 정지 */
	g_moeum.stir_phase_since = 0U;
	g_moeum.stir_cycles   = 0U;
	g_moeum.rot_st        = ROT_ST_IDLE;
	Rinse_Reset(&g_moeum.rinse);
	g_moeum.drain_open_since = 0U;
	g_moeum.door_close_by = (uint8_t)MOEUM_DOOR_BY_NONE;
	g_moeum.door_open_by  = (uint8_t)MOEUM_DOOR_BY_NONE;
	g_moeum.door_close_ms = 0U;
	g_moeum.door_open_ms  = 0U;
	moeum_all_off();
}

void Moeum_Start(void)
{
	/* IDLE에서만 수락. 진행 중 재요청은 무시. ★2026-09-22: 완료 유지(DONE, MOEUM_DONE_HOLD_MS)
	 * 중에도 받는다 — MotorTick 이 곧바로 IDLE 로 넘기고 다음 tick 에 시작한다. */
	/* ★G3: 동작 쪽 래치형 에러가 남아 있으면 시작하지 않는다(자기 쪽 에러는 state 로 이미 걸린다). */
	if (((g_moeum.state == (uint8_t)MOEUM_IDLE) || (g_moeum.state == (uint8_t)MOEUM_DONE)) &&
	    (Dongjak_IsLatched() == 0U))
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
	/* ★G3: 래치형 에러(E04·E09)는 마개 이탈·모드 전환으로 풀리지 않는다 — MOEUM_ERROR 유지. */
	if (Moeum_IsLatched() != 0U) { return; }
	g_moeum.err_code    = 0U;
	g_moeum.err_clear_req = 0U;
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
		else if (((MoeumState)c->state == MOEUM_ERROR) && (Moeum_IsLatched() == 0U))
		{
			c->err_code = 0U;                   /* ★G3: 해제형(E01)은 정지 요청으로 풀린다 */
			moeum_enter(c, MOEUM_IDLE, now_ms);
		}
	}

	switch ((MoeumState)c->state)
	{
	case MOEUM_IDLE:
		if (c->start_req)
		{
			RinseConfig  cfg;
			RinseOutputs o;
			c->start_req = 0U;
			c->last_err  = 0U;                  /* ★G4: 새 운전에서만 지운다 */
			/* 음성: "음식물을 염분세척하여 보관하겠습니다" (§0.14.11).
			 * Moeum_Start() 가 아니라 여기에 두는 이유 - Start() 는 SenseTick
			 * (100ms) 에서도 불릴 수 있는 반면 이 전이는 MotorTask 단일 문맥이고
			 * start_req 소비와 같은 틱이라 정확히 1회다. */
			(void)Voice_Play(VOICE_MOEUM_START, VOICE_PRIO_NORMAL);
			/* ★§0.30: 헹굼 전 구간(준비 탐색 R001 → 배수 R016)을 rinse.c 가 진행한다. */
			cfg.repeats         = (uint8_t)MOEUM_RINSE_REPEATS;
			cfg.rot_rpm         = (uint16_t)MOEUM_STIR_OUT_RPM;
			cfg.fill_timeout_ms = (uint32_t)MOEUM_FILL_TIMEOUT_MS;
			cfg.prep_after_fill = 0U;                  /* 모음: 시작 직후·급수 전 탐색 */
			Rinse_Begin(&c->rinse, &cfg, now_ms, &o);
			moeum_rinse_apply(c, &o);
			moeum_rinse_mirror(c, now_ms);             /* -> MOEUM_PREP */
		}
		break;

	case MOEUM_PREP:
	case MOEUM_DOOR_CLOSE:
	case MOEUM_FILL:
	case MOEUM_FILL_EXTRA:
	case MOEUM_STIR:
	case MOEUM_DRAIN_OPEN:
	case MOEUM_DRAIN_WAIT:
		moeum_rinse_step(c, now_ms);
		break;

	case MOEUM_DONE:
		/* 배수문 열린 상태로 종료. ★2026-09-22: MOEUM_DONE_HOLD_MS(3 s) 동안 완료(7)를 유지해 앱이
		 * 볼 수 있게 한 뒤 IDLE. 그 사이 시작 요청이 오면 바로 IDLE 로 — start_req 는 남아 있어
		 * 다음 tick 의 MOEUM_IDLE 이 소비한다. rinse 관측값(cnt 40·FINISHED)은 다음 시작까지 남는다. */
		if (c->start_req || ((uint32_t)(now_ms - c->state_since) >= (uint32_t)MOEUM_DONE_HOLD_MS))
		{
			moeum_enter(c, MOEUM_IDLE, now_ms);
		}
		break;

	case MOEUM_ERROR:
		/* 안전 정지 상태 유지. 복구: err_clear_req(앱 CLEAR_ERR) — 모든 에러.
		 * 해제형(E01)은 Moeum_Abort()(마개 이탈·모드 전환)·정지 요청으로도 풀린다(★G3). */
		if (c->err_clear_req)
		{
			c->err_clear_req = 0U;
			c->err_code      = 0U;
			moeum_enter(c, MOEUM_IDLE, now_ms);
		}
		break;

	default:
		break;
	}
}

/* ★G3 [2026-09-22 §0.33] 에러 해제 요청(앱 CLEAR_ERR). MOEUM_ERROR 에서만 MotorTick 이 소비한다. */
void Moeum_ClearError(void)
{
	g_moeum.err_clear_req = 1U;
}

uint8_t Moeum_IsLatched(void)
{
	return (uint8_t)(((MoeumState)g_moeum.state == MOEUM_ERROR) && Dongjak_ErrIsLatched(g_moeum.err_code));
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
