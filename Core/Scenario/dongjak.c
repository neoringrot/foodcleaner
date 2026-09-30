/* ==========================================================================
 * dongjak.c - "동작"(2단계: 건조 -> 분쇄 -> 배출) 시나리오 상태머신 구현.
 * ★기준 원본: doc/R1/zerogeo_scenario.docx ([2단계] 동작, 2026-07-30). 최신·우선.
 * 설계/시퀀스/상수 개요는 dongjak.h 참조. 미검증 값은 DJ_* 매크로로 분리.
 *
 * 하드웨어 소유권:
 *   - 히터     : GPIO_OUT_HT_POWER (PA12). 히스테리시스 114 ON/117 OFF (210=HW 바이메탈).
 *   - 교반     : g_stir_ctrl  (M2/U16). 헹굼=WASH/DRAIN 회전수 운전(rotation_port),
 *               건조=20RPM (정5/정지2)×5->역3(C035), 식힘=20RPM 정3/1/역3/1(C043),
 *               배출=30RPM CW6/1/CCW6. 110분 교반 RPM 전환은 C036 폐기.
 *   - 분쇄     : g_grind_ctrl (M1/U11). 분쇄 허가(dj_grind_allowed) 후 1500CW5/2 120초
 *               ->1000CW연속, 110분↑ 2000CCW4/2.
 *   - 배수문   : WDoor (U5). 헹굼/잔수배수/최종 완전개방.
 *   - 배출문   : TDoor (U7). 배출(개방 후 2분간 DC 미구동, 이후 닫음).
 *   - 수증기   : 냄새관로 스텝(STEP1)+흡입 스텝(STEP2)+FAN_VAPOR. 100℃ ON/84℃ OFF.
 *   - 팬       : FAN_VAPOR(수증기 방수팬, THERM3 100/84℃) / FAN_EXHAUST(배기팬, 시나리오
 *               전체 10초ON·5초OFF 독립 duty, C056) / BLDC_FAN(분쇄 시작 OR ≥80℃ 때 30/10, C057).
 *   - 온도     : 처리통 g_therm_c_d10[DJ_TEMP_CH]=CH0(히터/분쇄/식힘) / 수증기는
 *               g_therm_c_d10[DJ_VAPOR_TEMP_CH]=CH2(THERM3,J23). 수거통: g_bin_fill_pct.
 *
 * 안전 락(HW, MCU 아님)은 dongjak.h 참조 - FW는 감시/구동만.
 * ========================================================================== */

#include "dongjak.h"
#include "moeum.h"                  /* ★G3: 모음 래치형 에러 중이면 동작도 시작하지 않는다 */
#include "jungji.h"                 /* ★G1: 잔열 냉각을 jungji 가 쥐고 있으면 에러 중 팬을 건드리지 않는다 */
#include "voice.h"
#include "voice_table.h"

#include "gpio_ctrl.h"
#include "heater_hyst.h"
#include "wdoor.h"
#include "tdoor.h"
#include "bldc_ctrl.h"
#include "hallsensor.h"
#include "thermistor.h"
#include "adc_ctrl.h"                 /* ★§0.37 D5: PC3 weight-ADC (defaultTask 가 ADC1 단독 소유) */
#include "step_motor.h"
#if DJ_TEST_BEEP
#include "lm4871.h"                 /* [TEST] 단계 비프용 스피커(전역 lm4871) */
#endif

/* defaultTask가 갱신하는 센서 스냅샷(정의: freertos.c). 같은 태스크에서 읽는다. */
extern volatile int16_t g_therm_c_d10[];
extern volatile uint8_t g_bin_fill_pct;

DongjakCtx g_dongjak;

/* ---- 수증기 스테퍼 핸들 (냄새관로=STEP1, 흡입제어=STEP2 / 역할 TBD) ---- */
static StepMotor_HandleTypeDef dj_duct =
{
	.port = { o_STEP1_M1_GPIO_Port, o_STEP1_M2_GPIO_Port,
	          o_STEP1_M3_GPIO_Port, o_STEP1_M4_GPIO_Port },
	.pin  = { o_STEP1_M1_Pin, o_STEP1_M2_Pin,
	          o_STEP1_M3_Pin, o_STEP1_M4_Pin },
};
static StepMotor_HandleTypeDef dj_air =
{
	.port = { o_STEP2_M1_GPIO_Port, o_STEP2_M2_GPIO_Port,
	          o_STEP2_M3_GPIO_Port, o_STEP2_M4_GPIO_Port },
	.pin  = { o_STEP2_M1_Pin, o_STEP2_M2_Pin,
	          o_STEP2_M3_Pin, o_STEP2_M4_Pin },
};

static void dj_enter(DongjakCtx *c, DongjakState s, uint32_t now); /* 전방선언 */

/* DJ_ERROR 진입 + 원인코드 1곳 기록. 복구는 Dongjak_ClearError()로 IDLE 복귀. */
static void dj_fail(DongjakCtx *c, DjErrCode code, uint32_t now)
{
	c->err_code = (uint8_t)code;
	c->last_err = (uint8_t)code;           /* ★G4: 해제 뒤에도 남는다 */
	c->err_from = c->state;                /* ★G2: 에러 중 배기팬 유지 판정 */
	c->err_clear_req = 0U;                 /* 에러 전에 들어온 해제 요청이 새 에러를 곧바로 풀지 않도록 */
	c->weight_mon    = 0U;                 /* §0.37 D5: 감시 종료(f_weight 는 남긴다) */
	dj_enter(c, DJ_ERROR, now);
}

#if DJ_TEST_BEEP
/* [TEST 전용 — 차후 제거] 단계 비프: n회 삑(블로킹). 전환점(7/8/9분)에서만 호출.
 * LM4871_Beep은 busy-wait 블로킹이라 호출 동안 MotorTick이 잠시 멈춘다(테스트 허용). */
static void dj_test_beep(uint8_t n)
{
	uint8_t i;
	for (i = 0U; i < n; i++)
	{
		LM4871_Beep(&lm4871, (uint32_t)DJ_TEST_BEEP_HZ, (uint32_t)DJ_TEST_BEEP_MS);
		if ((uint8_t)(i + 1U) < n) { HAL_Delay((uint32_t)DJ_TEST_BEEP_GAP_MS); }
	}
}
#endif

/* ---- 저수준 헬퍼 --------------------------------------------------------- */

/* 급수 밸브 세트 ON/OFF. 동작 급수는 VALVE_DRY_IN(PB13, 건조통 급수)에
 * WATER_ON(PE2, 급수 펌프/메인 enable)을 함께 쓴다 — 수위센서 검증이 PE2 ON
 * 상태에서 통수·감지되었기 때문(DJ_FILL_USE_WATER_ON로 분리, 모음과 동일). */
static void dj_fill_on(void)
{
	gpio_ctrl_on(GPIO_OUT_VALVE_DRY_IN);
#if DJ_FILL_USE_WATER_ON
	gpio_ctrl_on(GPIO_OUT_WATER_ON);
#endif
}

static void dj_fill_off(void)
{
	gpio_ctrl_off(GPIO_OUT_VALVE_DRY_IN);
#if DJ_FILL_USE_WATER_ON
	gpio_ctrl_off(GPIO_OUT_WATER_ON);
#endif
}

/* 배수구세척솔 밸브(VALVE_DRAIN_CLN, PB14) ON/OFF. 배수문 개방과 함께 열어야
 * 실제 배수가 되는 구조(README §5) — WDoor 개폐 래퍼가 이 헬퍼를 동반 호출한다. */
static void dj_drain_valve(uint8_t open)
{
#if DJ_DRAIN_VALVE_USE
	if (open) { gpio_ctrl_on(GPIO_OUT_VALVE_DRAIN_CLN); }
	else      { gpio_ctrl_off(GPIO_OUT_VALVE_DRAIN_CLN); }
#else
	(void)open;
#endif
}

/* 배수문 개폐 래퍼. 동작 시나리오에서 WDoor를 여는 것은 항상 "배수" 의도이므로
 * 배수밸브를 함께 열고, 닫는 것은 "밀폐" 의도이므로 밸브를 함께 닫는다.
 * (모터는 리미트 도달 후 호출부가 Stop/Disable; 밸브는 다음 Close까지 유지된다.) */
static void dj_wdoor_open(void)
{
	WDoor_LimitArm();                    /* 이전 이동의 stale 리미트 엣지 제거 */
	WDoor_Enable(); WDoor_Open(WDoor_OpenDutyAt(0U));  /* 킥 80% -> tick 갱신 */
	dj_drain_valve(1U);
}

static void dj_wdoor_close(void)
{
	WDoor_LimitArm();
	WDoor_Enable(); WDoor_Close((uint8_t)WDOOR_CLOSE_DUTY);  /* 닫힘 80% 고정 */
	dj_drain_valve(0U);
}

/* 배수문 이동 완료 판정 — 테스트벤치 tb_drv8871(tb_wdoor_*)과 **동일 규칙**이며
 * 값은 wdoor.h의 WDOOR_* 프로파일 한 곳에서 온다(2026-08-25 벤치 확정).
 *   opening=1 (열림) : WHALL-OPEN 인식이 정상 종료. 미인식 대비 상한이
 *                      WDOOR_OPEN_MAX_MS(6s). duty는 호출부가 매 tick
 *                      WDoor_OpenDutyAt(el)로 킥(80%,2s)->유지(65%) 갱신.
 *   opening=0 (닫힘) : WDOOR_CLOSE_MS(4.2s) 경과가 정상 종료 조건. 그 전에
 *                      WHALL-CLOSE가 인식되면 거기서 종료.
 * 상한/시간 종료도 "정상 종료"다 — 벤치와 같이 모터만 세우고 다음 단계로 간다
 * (구 고정 타임아웃 ERROR 낙하 없음). el = 해당 이동 구동 경과(ms).
 * at_limit은 호출부에서 WDoor_Reached*() (arm 이후 EXTI 하강엣지 OR 레벨)로 넘긴다. */
static uint8_t dj_wdoor_done(uint8_t at_limit, uint8_t opening, uint32_t el)
{
	uint32_t lim = opening ? (uint32_t)WDOOR_OPEN_MAX_MS : (uint32_t)WDOOR_CLOSE_MS;
	if (at_limit) { return 1U; }
	if (el < lim) { return 0U; }
#if DJ_WDOOR_MISS_VOICE
	/* ★2026-09-22: 리미트 없이 시간으로 끝남 → 안내만(진행은 그대로). 호출부 전부가 1 을 받은
	 * tick 에 곧바로 다음 상태로 가므로 이 분기는 이동당 1회다(4곳: 헹굼 닫힘·열림, 건조 닫힘, 배출 후 열림). */
	(void)Voice_Play(opening ? VOICE_E_WDOOR_OPEN : VOICE_E_WDOOR_CLOSE, VOICE_PRIO_ERROR);
#endif
	return 1U;
}

/* 배출문(TDoor) 이동 완료 판정 — tb_drv8871(tb_tdoor_*)과 동일 규칙, 값은
 * tdoor.h의 TDOOR_* 프로파일에서 온다. 배수문과 달리 **THALL 인식으로 즉시
 * 서지 않는다**(닫힘): 인식 시점에 추가회전 창을 걸어두고 TDOOR_CLOSE_OVERRUN_MS(2.8 s)를
 * 더 돌아 자석을 지난 뒤 종료한다. 추가회전 창은 래치라 문이 자석을 지나 레벨이
 * 풀려도 다 채운다. 열림은 TDOOR_OPEN_OVERRUN_MS(200 ms) 뒤 종료한다. 방향별 상한(TDOOR_OPEN_MAX_MS/TDOOR_CLOSE_MAX_MS,
 * 20s)은 구동 전체의 하드 상한이라 추가회전보다 우선한다.
 * el = 해당 위상(개방/닫힘) 구동 경과(ms). 이동 개시마다 dj_tdoor_arm(). */
static uint32_t dj_tdoor_over_at;      /* 추가회전 종료 시각(el 기준). dj_tdoor_hit 일 때만 유효 */
/* THALL 인식 래치. 종전엔 over_at == 0 을 "미인식"으로 썼는데, 추가회전이 0 일 때 el 0 에서
 * 인식하면 over_at 도 0 이 되어 인식이 지워진다 — 그래서 플래그를 따로 둔다(2026-09-22). */
static uint8_t  dj_tdoor_hit;

static void dj_tdoor_arm(void)
{
	dj_tdoor_over_at = 0U;
	dj_tdoor_hit     = 0U;
}

static uint8_t dj_tdoor_done(uint8_t at_limit, uint8_t opening, uint32_t el)
{
	uint32_t cap = opening ? (uint32_t)TDOOR_OPEN_MAX_MS
	                       : (uint32_t)TDOOR_CLOSE_MAX_MS;

	if (el >= cap)                                 /* 하드 상한(추가회전 잘림) */
	{
#if DJ_TDOOR_MISS_VOICE
		/* ★2026-09-22: 20 s 동안 THALL 을 한 번도 못 봄 → 06-03 안내만(진행은 그대로).
		 * 호출부(개방·닫힘)는 1 을 받은 tick 에 다음 위상으로 가므로 이동당 1회다. */
		if (!dj_tdoor_hit) { (void)Voice_Play(VOICE_E_M_TDOOR, VOICE_PRIO_ERROR); }
#endif
		return 1U;
	}

	if (at_limit && !dj_tdoor_hit)
	{
		dj_tdoor_hit     = 1U;
		dj_tdoor_over_at = el + (opening ? (uint32_t)TDOOR_OPEN_OVERRUN_MS
		                                 : (uint32_t)TDOOR_CLOSE_OVERRUN_MS); /* 추가회전 래치 */
	}
	return (uint8_t)(dj_tdoor_hit && (el >= dj_tdoor_over_at));
}

static void dj_all_off(void)
{
	BldcCtrl_Stop(&g_grind_ctrl);
	RotStir_Stop(&g_dongjak.rot);   /* ★개정4 ④: 회전수 운전 비활성 + 교반 정지(슬립) */
	gpio_ctrl_off(GPIO_OUT_HT_POWER);
	dj_fill_off();
	dj_drain_valve(0U);
	gpio_ctrl_off(GPIO_OUT_FAN_VAPOR);
	gpio_ctrl_off(GPIO_OUT_FAN_EXHAUST);
	gpio_ctrl_off(GPIO_OUT_BLDC_FAN);
	WDoor_Stop();   WDoor_Disable();
	TDoor_Stop();   TDoor_Disable();
	StepMotor_Release(&dj_duct);
	StepMotor_Release(&dj_air);
}

/* ★N14 극성 정정 [2026-09-22, 구현현황 §0.32] — 인자 `ccw` 는 **위에서 본 방향**이다(0 = 시계 = 정회전,
 * 1 = 반시계 = 역회전, 0.3.0 정의). DRV8306 방향 비트는 모터마다 극성이 달라(M2 CW = reverse 1,
 * M1 CW = reverse 0) rotation_port.h 의 ROT_M2/M1_CW_IS_REVERSE 로 여기서 한 번만 푼다.
 * 종전에는 인자를 DRV 비트로 그대로 넘겨 **식힘·배출 교반이 위에서 반시계로 거꾸로** 돌았다
 * (분쇄는 극성이 0 이라 우연히 맞았다). 호출부는 원래 "0 = CW" 로 쓰고 있어 고치지 않았다. */
static uint8_t dj_dir_bit(uint8_t ccw, uint8_t cw_is_reverse)
{
	return (ccw != 0U) ? (uint8_t)!cw_is_reverse : cw_is_reverse;
}

static void dj_stir_spin(uint8_t ccw, uint16_t rpm)
{
	BldcCtrl_Stop(&g_stir_ctrl);
	BldcCtrl_Start(&g_stir_ctrl, dj_dir_bit(ccw, ROT_M2_CW_IS_REVERSE ? 1U : 0U));
	g_stir_ctrl.target_out_rpm = rpm;
}

static void dj_grind_spin(uint8_t ccw, uint16_t rpm)
{
	BldcCtrl_Stop(&g_grind_ctrl);
	BldcCtrl_Start(&g_grind_ctrl, dj_dir_bit(ccw, ROT_M1_CW_IS_REVERSE ? 1U : 0U));
	g_grind_ctrl.target_out_rpm = rpm;
}

/* ★R3 C031 / N2 해소 [2026-09-20] — 분쇄 허가 **관측**. 제어가 아니다.
 * REV02 는 분쇄(M1) ENABLE 을 회로가 3입력 AND 한다(넷리스트 검증):
 *
 *   PE7(M1-Enable) ─────────────────► U15.A
 *   Bimetal-70(J7) ─► U17(INV) ─[SB13]─► U15.B      SB13 = 쇼트(실장)
 *                                       U15.Y ──► U26.2A
 *   W-HALL-CLOSE ──► U34(INV) ──────► U26.2B
 *                                       U26.2Y ──► U16.24 ENABLE(DRV8306)
 *   SB11(PE7 직결 우회) = 오픈(미실장)
 *
 * 최종 ENABLE 노드(`M1-BLDC-EN`)에는 STM32 핀이 없고, port_contract 가 U16-24
 * 되읽기를 **순환 조건**으로 금지했다. 그래서 **같은 AND 를 PF0+PF3 로 재계산**한다.
 * 두 입력 모두 10k 풀업(R35/R32→3V3) + 접점 GND 라 **active-LOW** — PF0 = 0 은 **70℃ 초과(K2 락 = 분쇄 허가)**
 * (사용자 확인 2026-09-22: "PF0 는 70도를 넘어가면 active low 로 락"). PF1(50℃)은 반대로 active-HIGH 다.
 * 사용자 회신[2026-09-20]: SB11=오픈 · SB13=쇼트 · J7=70℃ 실장.
 *
 * 주의: 이 값이 0 인데 분쇄를 명령하면 ENABLE 이 LOW 라 FG 가 오지 않아
 * bldc_ctrl 의 잼 판정(stall_ms 1500 × retry_max 4)이 약 10초 만에 성립해
 * BLDC_LOCKED 로 소프트락된다. dongjak 은 그 상태를 읽지 않으므로 그 사이클의
 * 분쇄가 끝까지 죽는다. **반드시 관측 후 명령할 것.** */
static uint8_t dj_grind_allowed(void)
{
#if DJ_DRY_GATE_BYPASS
	return 1U;                                       /* [벤치 전용] dongjak.h 주석 참조 */
#else
	return (uint8_t)((gpio_ctrl_exti_read(GPIO_EXTI_BIMETAL_70)  == 0U) &&
	                 (gpio_ctrl_exti_read(GPIO_EXTI_WHALL_CLOSE) == 0U));
#endif
}

static uint8_t dj_water_present(void)
{
	/* ★REV02: PF7 이 가이드 홀로 재배정되어 SEN2 가 없다 - SEN1(PF6) 단독 판정. */
	if (gpio_ctrl_exti_flag_get(GPIO_EXTI_WATER_SEN1))
	{
		return 1U;
	}
#if DJ_WATER_USE_LEVEL
#if DJ_WATER_ACTIVE_LOW
	return (uint8_t)(gpio_ctrl_exti_read(GPIO_EXTI_WATER_SEN1) == 0U);
#else
	return (uint8_t)(gpio_ctrl_exti_read(GPIO_EXTI_WATER_SEN1) == 1U);
#endif
#else
	return 0U;
#endif
}

/* ---- (구) 교반 패턴 A: 건조 CW3/정지2 ×5 -> CCW3 -------------------------
 * ★2026-08-26: 건조 교반이 식힘/배출과 같은 3구간 패턴(30RPM CW6/정지1/CCW6)으로
 * 통일되면서 dj_stir_dry_begin/_tick 과 반복 카운터 stir_reps 를 제거했다.
 * ★2026-09-20 R3 2단계로 다시 갈렸다: 헹굼·건조·식힘은 4구간(dj_stir_alt_tick),
 * 배출만 3구간(dj_stir313_tick). 110분↑ RPM 전환은 C036 으로 폐기.
 * ★R3 C035 로 건조는 패턴 A(정5/정지2 ×5 -> 역3)가 dj_stir_alt_tick(group 5)로 복원됐다. */
/* ---- 교반 패턴 B: CW/정지/CCW 3구간 (현재 배출 전용) ------------------------
 * stir_phase 0=CW, 1=정지, 2=CCW. rpm과 구간 길이는 호출부가 지정한다.
 * (2026-08-26 에 교반을 전부 이 3구간으로 통일했으나 R3 에서 다시 갈렸다.)
 * 현재 교반 경로:
 *     (헹굼은 ★개정4 ④ 부터 회전수 운전 — rotation_port RotStir, dj_rinse_step() / rinse.c)
 *     (건조는 ★개정4 ⑤ 부터 PROCESS 회전수 운전 — RotStir_StartProcess/TickEx, 분쇄 동기)
 *     dj_stir_cool_tick()    - 식힘 80℃↓ (DJ_COOL_S313_*, 20RPM, 4구간)
 *     dj_stir313_tick()      - 배출      (DJ_S313_*, RPM은 호출부, 3구간)
 *   이 3구간 FSM(dj_stir3_tick)을 쓰는 것은 배출(6s/1s/6s)뿐이다. */
/* ★§0.37 D1: 배출 교반은 회로(U30 SEL=B, AUX)가 돌린다 — 아래 3구간 FSM 은 DJ_DISCH_STIR_BY_CIRCUIT=0(참고) 빌드에서만. */
#if !DJ_DISCH_STIR_BY_CIRCUIT
static void dj_stir313_begin(DongjakCtx *c, uint32_t now, uint16_t rpm)
{
	c->stir_phase = 0U;
	c->stir_since = now;
	dj_stir_spin(0U, rpm);                     /* CW                          */
}

static void dj_stir3_tick(DongjakCtx *c, uint32_t now, uint16_t rpm,
                          uint32_t cw_ms, uint32_t stop_ms, uint32_t ccw_ms)
{
	uint32_t el = now - c->stir_since;
	switch (c->stir_phase)
	{
	case 0:  /* CW */
		if (el >= cw_ms)
		{
			BldcCtrl_Stop(&g_stir_ctrl);
			c->stir_phase = 1U; c->stir_since = now;
		}
		break;
	case 1:  /* 정지 */
		if (el >= stop_ms)
		{
			dj_stir_spin(1U, rpm);             /* CCW                         */
			c->stir_phase = 2U; c->stir_since = now;
		}
		break;
	case 2:  /* CCW */
	default:
		if (el >= ccw_ms)
		{
			dj_stir_spin(0U, rpm);             /* CW 재시작                   */
			c->stir_phase = 0U; c->stir_since = now;
		}
		break;
	}
}

/* 배출: CW 6s / 정지 1s / CCW 6s (RPM은 호출부). R3 배출 교반은 제어권이 회로로
 * 이관 예정이고 P46 이 미확정이라(검토서 §4.9) R2 패턴을 그대로 둔다. 2026-09-20
 * 2단계로 헹굼·건조·식힘이 전부 4구간(dj_stir_alt_tick)으로 옮겨가면서 이 3구간
 * FSM 의 유일한 사용자가 배출이 됐다. */
static void dj_stir313_tick(DongjakCtx *c, uint32_t now, uint16_t rpm)
{
	dj_stir3_tick(c, now, rpm, (uint32_t)DJ_S313_CW_MS,
	              (uint32_t)DJ_S313_STOP_MS, (uint32_t)DJ_S313_CCW_MS);
}
#endif /* !DJ_DISCH_STIR_BY_CIRCUIT */

/* ---- 교반 패턴 C: 4구간 + 정회전 묶음 (R3 표준) ------------------------
 * ★I06 해소(2026-09-20). 09.16 레퍼런스 `src/controller.c` 의 alternate() 를 그대로
 * 옮긴 것이다. 원문 주석: "Each direction change has its own stop interval;
 * delayed ticks do not skip it."
 *
 *   phase 0 = 정(fwd_ms)  -> 1 = 정지(stop_ms)
 *   phase 1 에서 정회전 묶음(group)이 안 찼으면 -> 0 으로 되돌아간다
 *   묶음이 차면        -> 2 = 역(rev_ms) -> 3 = 정지(stop_ms) -> 0 (묶음 카운터 리셋)
 *
 * group=1 이면 정/정지/역/정지 4구간(헹굼·식힘), group=5 면 (정+정지)×5 -> 역 -> 정지
 * (건조). R2의 3구간(dj_stir3_tick)과 달리 **역→정 전환에도 정지가 들어간다** —
 * 이것이 I06 의 답이고, 그래서 헹굼 1cycle 이 13s 가 아니라 8s 다.
 * 구동 명령은 전이 시점에만 낸다(매 tick 재지령 금지 — BLDC 슬루 보호). */
static void dj_stir_alt_apply(uint8_t phase, uint16_t rpm)
{
	switch (phase)
	{
	case 0:  dj_stir_spin(0U, rpm); break;          /* 정(CW)                  */
	case 2:  dj_stir_spin(1U, rpm); break;          /* 역(CCW)                 */
	default: BldcCtrl_Stop(&g_stir_ctrl); break;    /* 1·3 = 정지              */
	}
}

static void dj_stir_alt_begin(DongjakCtx *c, uint32_t now, uint16_t rpm)
{
	c->stir_phase = 0U;
	c->stir_reps  = 0U;
	c->stir_mode  = 0U;
	c->stir_since = now;
	dj_stir_alt_apply(0U, rpm);
}

static void dj_stir_alt_tick(DongjakCtx *c, uint32_t now, uint16_t rpm,
                             uint32_t fwd_ms, uint32_t stop_ms, uint32_t rev_ms,
                             uint8_t group)
{
	uint32_t dur = (c->stir_phase == 0U) ? fwd_ms
	             : (c->stir_phase == 2U) ? rev_ms
	                                     : stop_ms;
	if ((now - c->stir_since) < dur) { return; }

	c->stir_since = now;
	if ((c->stir_phase == 1U) && (++c->stir_reps < group))
	{
		c->stir_phase = 0U;                         /* 묶음 미달: 정회전 반복  */
	}
	else if (c->stir_phase == 3U)
	{
		c->stir_phase = 0U; c->stir_reps = 0U;      /* 1cycle 완료             */
	}
	else
	{
		c->stir_phase++;
	}
	dj_stir_alt_apply(c->stir_phase, rpm);
}

/* 식힘 80℃ 미만: 20RPM 정3/정지1/역3/정지1 (R3 C043, group=1).
 * RPM 은 dry_rpm(20), 구간은 rinse_*(3/1/3) — 레퍼런스 ZG_D013 과 같은 조합. */
static void dj_stir_cool_tick(DongjakCtx *c, uint32_t now)
{
	dj_stir_alt_tick(c, now, (uint16_t)DJ_COOL_STIR_RPM, (uint32_t)DJ_COOL_S313_CW_MS,
	                 (uint32_t)DJ_COOL_S313_STOP_MS, (uint32_t)DJ_COOL_S313_CCW_MS, 1U);
}

/* (삭제) dj_stir_rinse_tick() — 헹굼 30RPM 정3/정지1/역3/정지1 시간 패턴(R3 C013).
 * ★개정4 ④ [2026-09-21, §0.26]: 헹굼 교반은 회전수 운전(WASH/DRAIN)으로 교체됐다.
 * DJ_RINSE_CW/STOP/CCW_MS 는 앱 보고(protocol_r0.c)·tb_rinse 패턴 모드용으로만 남는다. */

/* (삭제) dj_stir_dry_tick()·dj_stir_initial_tick() — 건조 교반 시간 패턴(C034 초기 동시회전 5/2 토글,
 * C035 (정5+정지2)×5 -> 역3). ★개정4 ⑤ [2026-09-22, §0.31]: 건조 교반·분쇄는 PROCESS 회전수 운전
 * (rotation_port RotStir_StartProcess/TickEx, 분쇄 동기)으로 교체됐다. 상수는 ZG_TRACKED_ONLY. */

/* ---- 히터: 114↓ON / 117↑OFF 히스테리시스 (+210 SW 보조, 신스펙 항목5) ----
 * 판정은 tb_heat와 공유하는 순수함수 Heater_HystDecide()가 담당(임계 이원화 방지).
 * 정책: **센서 에러 시 안전 OFF**(과열 방지). 밴드 안(114~117)은 HOLD=핀 유지. */
static void dj_heater_tick(DongjakCtx *c, uint32_t now)
{
	/* ★R3 C041(ST39 / P31 / TB-F08): 120분(heat_off_ms) 이후 **재가열 금지**.
	 * R2 는 DJ_COOLDOWN 에서 DJ_HEAT 로 돌아오는 전이가 없어 구조적으로 이미
	 * 충족했지만, 레퍼런스(controller.c 의 `cooling` 래치)처럼 명시한다.
	 * 해제는 **새 운전이 시작되는 곳뿐**이다 — 시나리오 시작(DJ_IDLE→RINSE1_CLOSE)
	 * 과 디버그 HEAT 직행(scn_start 를 now 로 다시 잡는 = 새 운전). DJ_HEAT /
	 * DJ_COOLDOWN 안에서는 어떤 경로로도 풀리지 않는다. */
	if (c->heat_off_latch) { gpio_ctrl_off(GPIO_OUT_HT_POWER); return; }

	static const HeaterHyst_t cfg =
	{
		.on_d10     = (int16_t)DJ_TEMP_HEATER_ON_D10,
		.off_d10    = (int16_t)DJ_TEMP_HEATER_OFF_D10,
		.safety_d10 = (int16_t)DJ_TEMP_SAFETY_D10,
	};
	/* 써미스터 에러 시 히터 강제 OFF - 온도 미상 상태에서 가열 금지. temp_valid는
	 * SenseTick가 갱신(드라이버 5회 연속 불량 디바운스 후 0). temp_d10 자체는 직전
	 * 유효값을 유지하므로 여기서 별도로 유효성을 봐야 한다. */
	if (!c->temp_valid)
	{
		gpio_ctrl_off(GPIO_OUT_HT_POWER);
		return;
	}

#if DJ_HEATER_REQUIRE_DRAIN_CLOSED
	/* ★C026 H1 (§0.36): 배수문 닫힘(PF3 = U35 입력과 같은 센서)이 아니면 명령하지 않는다 — 회로가 어차피
	 * 막으므로 핀을 ON 으로 두면 heater_since(공정 시계)가 가짜로 흐른다. 복귀 뒤엔 히스테리시스 그대로
	 * (밴드 안이면 HOLD = OFF 유지 → on 임계에서 다시 ON). */
	c->heat_door_ok = WDoor_AtClose();
	if (!c->heat_door_ok)
	{
		gpio_ctrl_off(GPIO_OUT_HT_POWER);
		return;
	}
#endif

	switch (Heater_HystDecide(&cfg, c->temp_d10, c->temp_valid))
	{
	case HEATER_CMD_ON:   gpio_ctrl_on(GPIO_OUT_HT_POWER);  break;
	case HEATER_CMD_OFF:  gpio_ctrl_off(GPIO_OUT_HT_POWER); break;
	case HEATER_CMD_HOLD:
	default:              /* 밴드 안(114~117): 핀 유지 */ break;
	}

	/* ★R3 C005 (C212, 검토서 §17.2) — 공정 시계 기산점. 히터 ON 이 **실제로 핀에
	 * 적용된 것**을 출력 래치 되읽기로 확인한 첫 tick 에 한 번만 찍는다(= "기판에 성공
	 * 전달"). 이후 온도 조절로 꺼졌다 켜져도 다시 찍지 않는다. 해제는 새 운전 시작뿐.
	 * ⚠ 래치는 MCU 핀까지다 — 실제 히터 전원(U35: HT-POWER AND 배수문 닫힘)은 계약서
	 *   heater_clock 대로 PCB 에서 전류로 확인할 것(HW미검증 1-31). */
	if (!c->heater_started && gpio_ctrl_is_on(GPIO_OUT_HT_POWER))
	{
		c->heater_started = 1U;
		c->heater_since   = now;
	}
}

/* ★R3 C005·C041 — 공정 시계. 히터가 아직 한 번도 안 켜졌으면 0 (벤더와 동일). */
static uint32_t dj_heater_elapsed(const DongjakCtx *c, uint32_t now)
{
	return c->heater_started ? (now - c->heater_since) : 0U;
}

/* ---- 분쇄: 모드(시간·온도) 결정 후 토글/연속 구동 ---------------------- *
 * 80℃ 미만 = OFF. 110분↑ = 2000 CCW 토글. 개시 3분 = 1500 CW 토글, 이후 =
 * 170℃↑ 1000 CW 4/2초 토글(고온제어) / 그 외 1000 CW 연속. (식힘 = 1000 CCW 연속) */
static void dj_grind_apply_mode(DongjakCtx *c, DjGrindMode m, uint32_t now)
{
	if (c->grind_mode == (uint8_t)m) { return; }
	c->grind_mode  = (uint8_t)m;
	c->grind_phase = (uint8_t)DJ_GR_RUN;
	c->grind_since = now;
	switch (m)
	{
	/* ★⑤ COARSE/FINE/FINAL 은 건조 PROCESS 의 **표시 라벨**이다(dj_process_mirror) — 분쇄 구동은
	 * rotation_port 가 한다. 이 함수로 라벨 모드에 들어가는 경로는 없다. */
	case DJ_GM_COOL:   dj_grind_spin(1U, (uint16_t)DJ_GRIND_FINE_RPM);   break; /* 식힘 80℃↑ 연속 */
	case DJ_GM_COARSE:
	case DJ_GM_FINE:
	case DJ_GM_FINAL:  break;
	case DJ_GM_OFF:
	default:           BldcCtrl_Stop(&g_grind_ctrl);                     break;
	}
}

/* ★개정4 ⑤ [2026-09-22, §0.31] — 건조 PROCESS 회전수 운전의 앱·watch 미러.
 * (삭제: dj_grind_toggle()·dj_grind_heat_tick() — 초기 1500 5/2 토글 120초 → 1000 연속 → 110분 2000 역 4/2.)
 * grind_mode 는 **라벨**이다: 초기 30회 = COARSE, 이후 = FINE, late = FINAL. 모터 폴트 감시(§0.29)와
 * 식힘팬(dj_fan_bldc_tick)이 grind_mode != OFF 를 "분쇄 중"으로 본다 — PROCESS 동안 그대로 성립한다. */
static void dj_process_mirror(DongjakCtx *c)
{
	c->grind_mode  = (uint8_t)(RotStir_Late(&c->rot)        ? DJ_GM_FINAL
	                         : RotStir_InitialDone(&c->rot) ? DJ_GM_FINE : DJ_GM_COARSE);
	c->grind_phase = (uint8_t)((c->rot.out != 0U) ? DJ_GR_RUN : DJ_GR_STOP);
	c->grind_since = c->rot.out_since;
	c->stir_phase  = (uint8_t)((c->rot.out == 1U) ? DJ_STIR_FWD
	                         : (c->rot.out == 2U) ? DJ_STIR_REV : DJ_STIR_STOPPED);
	c->stir_since  = c->rot.out_since;
	c->stir_reps   = (uint8_t)RotStir_Legs(&c->rot);   /* 현 구간 운전 수(초기 0~29 / CCW 0~1 / CW 0~9) */
	c->stir_mode   = 0U;
}

/* ★⑤ 벤더 ZG_D005~D010 허가 = grind_allowed && outlet_closed. outlet = 배출문 닫힘(T-HALL-CLOSE). */
static uint8_t dj_dry_permitted(DongjakCtx *c)
{
	c->grind_allow = dj_grind_allowed();             /* [관측] watch 용으로도 남긴다 */
#if DJ_DRY_REQUIRE_OUTLET
	return (uint8_t)(c->grind_allow && TDoor_AtClose());
#else
	return c->grind_allow;
#endif
}

/* ---- 수증기: 100℃ 개방(방수팬->관로->흡입), 84℃ 역순 폐쇄 (docx 7.1) --
 * 온도 소스는 수증기 전용 THERM3(J23, PC2) = c->vapor_temp_d10 (히터/분쇄와 별개).
 * FAN_VAPOR(방수팬)만 100℃에서 먼저 ON한 뒤 STEP1(관로)/STEP2(흡입)을 개방, 84℃에서
 * 스텝 역순 폐쇄 후 FAN_VAPOR OFF. (FAN_EXHAUST 배기팬은 THERM3 분리, fanx 10s/5s 독립) */
/* 스텝 시작 램프: idx(현재 개폐동작 내 스텝 순번) 기준 유효 스텝 간격 반환.
 * 첫 DJ_STEP_RAMP_STEPS 스텝은 START->RUN 간격으로 선형 가속(자기기동 한계 존중),
 * 이후 RUN 유지. vapor_step_cnt가 개폐 동작마다 0으로 리셋되므로 각 개/폐 이동의
 * 시작마다 램프가 다시 적용된다. air=1이면 STEP2 전용(더 느린) 페이싱을 쓴다. */
static uint32_t dj_step_interval(uint16_t idx, uint8_t air)
{
	uint32_t run   = air ? (uint32_t)DJ_STEP2_INTERVAL_MS : (uint32_t)DJ_STEP_INTERVAL_MS;
	uint32_t start = air ? (uint32_t)DJ_STEP2_START_MS    : (uint32_t)DJ_STEP_START_MS;
	if (idx < (uint16_t)DJ_STEP_RAMP_STEPS)
	{
		uint32_t span = start - run;
		return start - (span * (uint32_t)idx) / (uint32_t)DJ_STEP_RAMP_STEPS;
	}
	return run;
}

/* 개/폐 1회 구동시간(ms). 테스트벤치(tb_stepmotor)와 동일 규칙:
 *   STEP1(관로) 방향 무관 15s / STEP2(흡입) 열림 1s · 닫힘 2s.
 * 기구 스토퍼에 밀어붙이는 방식이라 "시간"이 스펙이고, 실제 스텝수는 페이싱
 * (dj_step_interval)에 딸린 결과값이다. */
static uint32_t dj_move_ms(uint8_t phase)
{
	switch ((DjVaporPhase)phase)
	{
	case DJ_VP_OPEN_AIR:  return (uint32_t)DJ_STEP2_OPEN_MS;   /* 흡입 열림 1s  */
	case DJ_VP_CLOSE_AIR: return (uint32_t)DJ_STEP2_CLOSE_MS;  /* 흡입 닫힘 2s  */
	default:              return (uint32_t)DJ_STEP1_RUN_MS;    /* 관로 개/폐 15s */
	}
}

/* 개/폐 동작 시작: 램프 인덱스·페이싱·구동시간 기준점을 한 번에 리셋. */
static void dj_vapor_move_begin(DongjakCtx *c, uint8_t phase, uint32_t now)
{
	c->vapor_step_cnt   = 0U;
	c->vapor_step_since = now;
	c->vapor_move_since = now;
	c->vapor_phase      = phase;
}

static void dj_vapor_tick(DongjakCtx *c, uint32_t now)
{
	int16_t  t  = c->vapor_temp_d10;
	/* STEP2(흡입) 위상(OPEN_AIR·CLOSE_AIR)은 느린 페이싱, STEP1(관로)은 기본. */
	uint8_t  air = (uint8_t)((c->vapor_phase == (uint8_t)DJ_VP_OPEN_AIR) ||
	                         (c->vapor_phase == (uint8_t)DJ_VP_CLOSE_AIR));
	uint8_t  due = (uint8_t)((now - c->vapor_step_since) >= dj_step_interval(c->vapor_step_cnt, air));
	/* 종료는 스텝수가 아니라 경과시간으로 판정(벤치와 동일). */
	uint8_t  done = (uint8_t)((now - c->vapor_move_since) >= dj_move_ms(c->vapor_phase));

	switch ((DjVaporPhase)c->vapor_phase)
	{
	case DJ_VP_CLOSED:
		if (t >= (int16_t)DJ_TEMP_VAPOR_ON_D10)
		{
			gpio_ctrl_on(GPIO_OUT_FAN_VAPOR);        /* 1) 방수팬 ON (팬 먼저)  */
			/* FAN_EXHAUST는 THERM3 루프에서 분리됨(dj_fan_exhaust_tick가 10s/5s duty로 독립 제어) */
			dj_vapor_move_begin(c, (uint8_t)DJ_VP_OPEN_DUCT, now);
		}
		break;
	case DJ_VP_OPEN_DUCT:                             /* 2) 냄새 관로 OPEN (15s) */
		if (done)
		{
#if DJ_STEP_RELEASE_DUCT_ON_OPEN
			StepMotor_Release(&dj_duct);      /* STEP1 코일 해제 → STEP2에 레일 전류 양보 */
#endif
			dj_vapor_move_begin(c, (uint8_t)DJ_VP_OPEN_AIR, now);
		}
		else if (due)
		{
			StepMotor_Step(&dj_duct, 1); c->vapor_step_cnt++; c->vapor_step_since = now;
		}
		break;
	case DJ_VP_OPEN_AIR:                              /* 3) 흡입 제어 OPEN (1s)  */
		if (done)      { c->vapor_phase = (uint8_t)DJ_VP_OPEN; }
		else if (due)  { StepMotor_Step(&dj_air, 1); c->vapor_step_cnt++; c->vapor_step_since = now; }
		break;
	case DJ_VP_OPEN:
		/* ★R3 C058(BG3 / P43): 84℃ 역순 폐쇄(흡입→관로→방수팬 OFF)는 **한 운전에
		 * 1회만**. R2는 온도가 84℃를 재차 가로지를 때마다 반복됐다. 래치는
		 * vapor_close_done 이고 시나리오 시작(DJ_IDLE→RINSE1_CLOSE)에서만 풀린다.
		 * 100℃ 개방(BG4)은 xlsx 판정 '존치'라 래치하지 않는다 — 래치 이후에는
		 * 개방만 반복될 수 있다는 뜻이므로 시나리오 검증에서 재확인할 것
		 * (구현현황 §0.12 확인사항 ①). */
		if (!c->vapor_close_done && (t <= (int16_t)DJ_TEMP_VAPOR_OFF_D10))
		{
			c->vapor_close_done = 1U;
			dj_vapor_move_begin(c, (uint8_t)DJ_VP_CLOSE_AIR, now);
		}
		break;
	case DJ_VP_CLOSE_AIR:                             /* 1') 흡입 CLOSE (2s)     */
		if (done)
		{
			StepMotor_Release(&dj_air);
			dj_vapor_move_begin(c, (uint8_t)DJ_VP_CLOSE_DUCT, now);
		}
		else if (due)
		{
			StepMotor_Step(&dj_air, -1); c->vapor_step_cnt++; c->vapor_step_since = now;
		}
		break;
	case DJ_VP_CLOSE_DUCT:                            /* 2') 관로 CLOSE (15s)    */
	default:
		if (done)
		{
			StepMotor_Release(&dj_duct);
			gpio_ctrl_off(GPIO_OUT_FAN_VAPOR);   /* 3') 방수팬 OFF (배기팬은 fanx 독립) */
			c->vapor_phase = (uint8_t)DJ_VP_CLOSED;
		}
		else if (due)
		{
			StepMotor_Step(&dj_duct, -1); c->vapor_step_cnt++; c->vapor_step_since = now;
		}
		break;
	}
}

static void dj_vapor_off(DongjakCtx *c)
{
	gpio_ctrl_off(GPIO_OUT_FAN_VAPOR);           /* FAN_EXHAUST는 fanx가 독립 관리 */
	StepMotor_Release(&dj_duct);
	StepMotor_Release(&dj_air);
	c->vapor_phase = (uint8_t)DJ_VP_CLOSED;
}

/* 건조 → 식힘 전환(히터 종료 래치 + PROCESS 정지 + 수증기 닫기). 120분 백스톱과
 * [TEST] DJ_TEST_SHORT 의 30 s 동시 운전 종료(§0.47)가 같이 쓴다. */
static void dj_heat_to_cool(DongjakCtx *c, uint32_t now)
{
	c->heat_off_latch = 1U;                  /* R3 C041: 재가열 금지 래치 */
	RotStir_Stop(&c->rot);                   /* ★⑤ PROCESS 종료 — 교반·분쇄 정지(슬립) */
	c->grind_mode = (uint8_t)DJ_GM_OFF;      /* 식힘이 DJ_GM_COOL 을 새로 건다 */
	dj_vapor_off(c);
	dj_enter(c, DJ_COOLDOWN, now);
}

/* ---- BLDC 모터 냉각팬: 30/10초 duty (R3 C057 / BG2 · P41·P42) -----------
 * ★R3 C057 로 구동 **조건**이 넓어졌다. R2는 `grind_mode != OFF` 단독이었고,
 * 분쇄가 서면 팬을 끄면서 fanb_since 까지 리셋해 duty 위상이 매번 처음으로
 * 돌아갔다. R3 요구는 두 가지다:
 *   (1) "분쇄 시작 **OR** 온도 ≥ 80℃"  — 분쇄가 없어도 뜨거우면 돈다.
 *   (2) 분쇄 간헐 정지 구간에도 **duty 주기를 유지** — 기산점을 리셋하지 않는다.
 * 그래서 '조건 성립'(fanb_active)과 'duty 출력'(fanb_on)을 분리했다. fanb_since
 * 는 조건이 OFF->ON 으로 바뀔 때만 다시 잡힌다. 1차 분쇄의 5s/2s 토글은
 * grind_mode 를 COARSE 로 유지한 채 회전만 끊으므로 (2)가 자동 충족된다.
 * 임계는 DJ_TEMP_BLDCFAN_ON_D10(800 = 80.0℃), 판정 채널은 THERM1(temp_d10).
 * 호출 구간은 DJ_HEAT / DJ_COOLDOWN 뿐이다(= 레퍼런스의 process_active).
 * DJ_ABORTED 의 비상 냉각은 별도 경로라 여기서 다루지 않는다. */
static void dj_fan_bldc_tick(DongjakCtx *c, uint32_t now)
{
	uint8_t hot = (uint8_t)(c->temp_valid &&
	                        (c->temp_d10 >= (int16_t)DJ_TEMP_BLDCFAN_ON_D10));
	uint8_t run = (uint8_t)(hot || (c->grind_mode != (uint8_t)DJ_GM_OFF));

	if (!run)                                           /* 조건 해제: 팬 정지    */
	{
		if (c->fanb_on) { gpio_ctrl_off(GPIO_OUT_BLDC_FAN); c->fanb_on = 0U; }
		c->fanb_active = 0U;
		return;
	}
	if (!c->fanb_active)                                /* 조건 성립 전이: ON 개시 */
	{
		c->fanb_active = 1U; c->fanb_since = now;
		gpio_ctrl_on(GPIO_OUT_BLDC_FAN); c->fanb_on = 1U;
		return;
	}
	uint32_t el = now - c->fanb_since;                  /* 조건 유지 중: 30/10 토글 */
	if (c->fanb_on)
	{
		if (el >= (uint32_t)DJ_FANB_ON_MS)
		{ gpio_ctrl_off(GPIO_OUT_BLDC_FAN); c->fanb_on = 0U; c->fanb_since = now; }
	}
	else
	{
		if (el >= (uint32_t)DJ_FANB_OFF_MS)
		{ gpio_ctrl_on(GPIO_OUT_BLDC_FAN);  c->fanb_on = 1U; c->fanb_since = now; }
	}
}

/* ---- 배기팬(FAN_EXHAUST): 시나리오 전체 10초 ON / 5초 OFF 독립 duty -------
 * THERM3 수증기 루프에서 분리(2026-08-17). 시나리오 시작 시 begin으로 ON 개시,
 * 처리 상태(RINSE1_CLOSE~DISCHARGE) 동안 tick이 10/5로 토글. 종료(DONE/ERROR/
 * ABORTED)에서 dj_all_off가 OFF.
 * ★R3 C056(BG1): duty 15분/2분 → **10초/5초**(dongjak.h 주석 참조). 종료 시점을
 * '배출문 실제 닫힘'으로 옮기는 것은 6단계(배출) 소관이라 아직 현행 유지. */
static void dj_fan_exhaust_begin(DongjakCtx *c, uint32_t now)
{
	gpio_ctrl_on(GPIO_OUT_FAN_EXHAUST);
	c->fanx_on = 1U;
	c->fanx_since = now;
}

static void dj_fan_exhaust_tick(DongjakCtx *c, uint32_t now)
{
	uint32_t el = now - c->fanx_since;
	if (c->fanx_on)
	{
		if (el >= (uint32_t)DJ_FANX_ON_MS)
		{ gpio_ctrl_off(GPIO_OUT_FAN_EXHAUST); c->fanx_on = 0U; c->fanx_since = now; }
	}
	else
	{
		if (el >= (uint32_t)DJ_FANX_OFF_MS)
		{ gpio_ctrl_on(GPIO_OUT_FAN_EXHAUST);  c->fanx_on = 1U; c->fanx_since = now; }
	}
}

/* 배기팬 10/5 duty 구간(C056): RINSE1_CLOSE~DISCHARGE · DJ_PREP(enum 끝). 자가세척(DJ_SELFCLEAN)은 끈다(§0.30.4 ④). */
static uint8_t dj_exhaust_range(uint8_t s)
{
	return (uint8_t)((((DongjakState)s >= DJ_RINSE1_CLOSE) && ((DongjakState)s <= DJ_DISCHARGE)) ||
	                 ((DongjakState)s == DJ_PREP));
}

/* 에러 중 배기(공기정화)팬을 이어 갈지 — 에러 직전이 배기 구간이고, 배출문이 이미 닫힌 뒤의 E08 이 아닐 때.
 * 벤더는 O009(배출문 실제 닫힘)에서 purification_done — E08(O010)은 그 뒤라 팬이 이미 끝났다(§0.37). */
static uint8_t dj_error_keep_exhaust(const DongjakCtx *c)
{
	return (uint8_t)(dj_exhaust_range(c->err_from) && (c->err_code != (uint8_t)DJ_ERR_RELEASE));
}

/* ★R3 예외처리 G1·G2 [2026-09-22, §0.33] — DJ_ERROR 중 팬. 벤더 0.3.0 은 에러 단계에서도
 * process_active 가 살아 있어 background()(공기정화팬 10/5)와 motor_fan()(80℃↑ 또는 분쇄 → 30/10)이
 * 계속 돈다 — 에러로 끄는 것은 히터·교반·분쇄·급수뿐이다(controller.c 479). 종전 우리 코드는
 * dj_all_off 로 팬까지 꺼서 **건조 190℃ 에서 E09 가 나면 냉각 없이 멈췄다.**
 *   - G1 모터 냉각팬: dj_fan_bldc_tick(80℃↑ 30/10). 진입 시 grind_mode = OFF 라 조건은 온도뿐.
 *                    써미스터가 무효면 jungji 와 같이 **고온으로 보고 연속 ON**.
 *   - G2 배기팬: 에러 직전 상태가 배기 구간이었으면 10/5 를 그대로 잇는다(위상 유지).
 * 마개 이탈 등으로 jungji 가 잔열 냉각을 쥐고 있으면(Jungji_IsCooling) 손대지 않는다 — 두 곳이 같은
 * 핀을 번갈아 쓰지 않도록. jungji 가 식었다고 놓으면 다음 tick 부터 여기가 다시 쥔다. */
static void dj_error_fans(DongjakCtx *c, uint32_t now)
{
	if (Jungji_IsCooling() != 0U) { return; }

	if (dj_error_keep_exhaust(c)) { dj_fan_exhaust_tick(c, now); }

	if (!c->temp_valid)
	{
		gpio_ctrl_on(GPIO_OUT_BLDC_FAN);   /* 온도 모름 = 안전측 */
		c->fanb_on = 1U; c->fanb_active = 0U;
	}
	else
	{
		dj_fan_bldc_tick(c, now);
	}
}

static uint8_t dj_error_hot(const DongjakCtx *c)
{
	return (uint8_t)(!c->temp_valid || (c->temp_d10 >= (int16_t)DJ_TEMP_BLDCFAN_ON_D10));
}

/* DJ_ERROR 해제(ClearError·해제형 정지 요청). 뜨거우면 IDLE 대신 DJ_ABORTED 로 보내 식힘팬을 잇는다
 * (IDLE 은 팬을 쥐지 않는다). err_code 는 지우고 last_err 는 남긴다(G4). */
static void dj_error_release(DongjakCtx *c, uint32_t now)
{
	c->err_code  = (uint8_t)DJ_ERR_NONE;
	c->lid_guard = 0U; c->lid_low_cnt = 0U;
	dj_enter(c, dj_error_hot(c) ? DJ_ABORTED : DJ_IDLE, now);
}

/* 식힘 "80℃ 미만 도달"(분쇄 OFF·교반313 전환) 판정:
 *   (THERM1 && THERM2 둘 다 유효 && <80℃)  ||  HW 80℃ 바이메탈(PF0) 하강엣지.
 *   ⚠ REV02 PF0 는 70℃ 를 **넘을 때** LOW 로 떨어진다(active-low 락) — 하강엣지는 "식음"이 아니라
 *     "가열 중 70℃ 통과"다. DJ_COOL_USE_BIMETAL80 을 다시 켜려면 상승엣지(식어서 70℃ 아래)로 바꿔야 한다.
 * 보수적(A안): 어느 써미스터든 에러(미상)면 '아직 안 식음'으로 보아 정지하지 않되,
 * 바이메탈 하강엣지가 오면 그걸로 정지(안전 폴백). 엣지는 COOLDOWN 진입 시 stale
 * 클리어된 이후의 것만 유효. DJ_COOL_USE_THERM2=0이면 THERM1 단독(구동작). */
static uint8_t dj_cool_reached(DongjakCtx *c)
{
#if DJ_COOL_USE_BIMETAL80
	if (gpio_ctrl_exti_flag_get(GPIO_EXTI_BIMETAL_70)) { return 1U; }  /* 바이메탈: 안전 폴백 */
#endif
	/* THERM1 유효 && <80℃ */
	if (!(c->temp_valid && (c->temp_d10 < (int16_t)DJ_TEMP_COOL_GRIND_OFF_D10))) { return 0U; }
#if DJ_COOL_USE_THERM2
	/* THERM2도 유효 && <80℃ 라야 '식음' 확정(둘 중 하나라도 미달/에러면 계속 냉각) */
	if (!(c->temp2_valid && (c->temp2_d10 < (int16_t)DJ_TEMP_COOL_GRIND_OFF_D10))) { return 0U; }
#endif
	return 1U;
}

/* ---- 헹굼 · 자가세척 — ★§0.30 공통 헹굼(rinse.c) 연결 ----------------------
 * 헹굼 FSM(준비 탐색 R001~R006 · 회차 R007~R019)은 rinse.c 가 가진다. 이 파일은
 *   ① 명령 비트 적용(dj_rinse_apply — 배수문을 열 때 배수밸브 동반은 동작 고유 규칙),
 *   ② 앱·watch 용 상태 미러(DJ_PREP / DJ_RINSE1_x / DJ_RINSE2_x — 번호 불변, 재배열은 8단계),
 *   ③ 종료 처리(음성·에러 코드·다음 단계)
 * 만 한다. 처리 헹굼은 DJ_RINSE_COUNT(1, §0.35 C004 고정)회, 자가세척은 DJ_SELFCLEAN_REPEATS(1)회다.
 * 자가세척(DJ_SELFCLEAN) 중에는 미러를 쓰지 않는다 — 세부 단계는 g_dongjak.rinse.step 으로 본다. */

/* 명령 비트 적용. 순서: 교반 정지 → 회전수 운전 → 문 → 급수. */
static void dj_rinse_apply(DongjakCtx *c, const RinseOutputs *o)
{
	if (o->cmd & RINSE_CMD_STIR_MANUAL) { RotStir_Manual(o->stir_dir, o->stir_rpm); }
	if (o->cmd & RINSE_CMD_STIR_STOP)   { RotStir_Stop(&c->rot); }
	if (o->cmd & RINSE_CMD_ROT_WASH)    { RotStir_Start (&c->rot, ZG_ROT_WASH,  o->rot_rpm); c->rot_st = ROT_ST_RUNNING; }
	if (o->cmd & RINSE_CMD_ROT_DRAIN)   { RotStir_Switch(&c->rot, ZG_ROT_DRAIN, o->rot_rpm); c->rot_st = ROT_ST_RUNNING; }

	if (o->cmd & RINSE_CMD_DOOR_STOP)   { WDoor_Stop(); WDoor_Disable(); }
	if (o->cmd & RINSE_CMD_DOOR_CLOSE)  { dj_wdoor_close(); }             /* 밀폐: 배수밸브 OFF 동반 */
	if (o->cmd & RINSE_CMD_DOOR_OPEN)   { dj_wdoor_open();  }             /* 배수: 배수밸브 ON 동반, 킥 80 % */
	if (o->cmd & RINSE_CMD_DOOR_DUTY)   { WDoor_Open(o->door_duty); }     /* 킥 80 %(2 s) -> 65 % 유지 */

	if (o->cmd & RINSE_CMD_FILL_ON)
	{
		c->water_reached = 0U;                        /* 잔류 감지 무시 */
		gpio_ctrl_exti_flag_clear(GPIO_EXTI_WATER_SEN1);
		dj_fill_on();
	}
	if (o->cmd & RINSE_CMD_FILL_OFF)    { dj_fill_off(); }
}

/* 처리 헹굼 단계 -> DongjakState 미러. 회차는 rinse.done(0 = 1차, 1 이상 = 2차). */
static void dj_rinse_mirror(DongjakCtx *c, uint32_t now)
{
	uint8_t      second = (uint8_t)(c->rinse.done != 0U);
	DongjakState m;

	switch ((RinseStep)c->rinse.step)
	{
	case RINSE_CLOSE:      m = second ? DJ_RINSE2_CLOSE : DJ_RINSE1_CLOSE; break;
	case RINSE_FILL:       m = second ? DJ_RINSE2_FILL  : DJ_RINSE1_FILL;  break;
	case RINSE_WASH:       m = second ? DJ_RINSE2_STIR  : DJ_RINSE1_STIR;  break;
	case RINSE_DRAIN_OPEN: m = second ? DJ_RINSE2_OPEN  : DJ_RINSE1_OPEN;  break;
	case RINSE_DRAIN:      m = second ? DJ_RINSE2_DRAIN : DJ_RINSE1_DRAIN; break;
	default:               m = DJ_PREP;                                     break;
	}
	if (c->state != m) { c->state = m; c->state_since = now; }
}

/* 헹굼/자가세척 한 tick. selfclean: 0 = 처리 헹굼(끝나면 DJ_HEAT), 1 = 자가세척(끝나면 DJ_DONE). */
static void dj_rinse_step(DongjakCtx *c, uint32_t now, uint8_t selfclean)
{
	RinseInputs  in;
	RinseOutputs o;
	RinseStep    st;

	/* ★개정4 ④: 회전수 운전은 rinse 보다 먼저 한 tick 진행한다(비활성이면 IDLE). */
	c->rot_st = RotStir_Tick(&c->rot, true, now);      /* WASH/DRAIN: permitted = true */
	if (c->rot.prof != ZG_ROT_NONE)
	{
		c->stir_phase = (uint8_t)((c->rot.out == 1U) ? DJ_STIR_FWD
		                        : (c->rot.out == 2U) ? DJ_STIR_REV : DJ_STIR_STOPPED);
		c->stir_since = c->rot.out_since;
	}

	in.door_closed = WDoor_ReachedClose();
	in.door_open   = WDoor_ReachedOpen();
	in.water       = c->water_reached;
	in.rot_st      = c->rot_st;
	in.stir_pos    = BldcCtrl_Position(&g_stir_ctrl);   /* §0.38 순이동 감시 */
	in.stir_rpm    = g_stir_ctrl.meas_out_rpm;          /* §0.41 과속 보조 A */
	st = Rinse_Tick(&c->rinse, &in, now, &o);
	dj_rinse_apply(c, &o);

#if DJ_WDOOR_MISS_VOICE
	/* ★2026-09-22: 리미트 없이 시간으로 끝남 → 안내만(진행은 그대로). 이동당 1회. */
	if (o.ev & RINSE_EV_CLOSE_TMO) { (void)Voice_Play(VOICE_E_WDOOR_CLOSE, VOICE_PRIO_ERROR); }
	if (o.ev & RINSE_EV_OPEN_TMO)  { (void)Voice_Play(VOICE_E_WDOOR_OPEN,  VOICE_PRIO_ERROR); }
#endif

	switch (st)
	{
	case RINSE_FINISHED:
		if (selfclean)
		{
			/* 벤더 M001: 자가세척 완료 음성(04-02 "자가세척 기능이 끝났습니다"). 배수문은 열린 채
			 * 끝난다 — 종전 배출 뒤 "배수부 개방"(DJ_DS_OPEN_W)을 이 배수가 대신한다. */
			(void)Voice_Play(VOICE_SELFCLEAN_END, VOICE_PRIO_NORMAL);
			dj_enter(c, DJ_DONE, now);
		}
		else
		{
			dj_enter(c, DJ_HEAT, now);                 /* 2차 배수 완료 -> 건조 (배수문 닫기부터) */
		}
		break;

	case RINSE_PREP_FAULT:
		/* E01. 처리 헹굼에선 급수·배수문·히터·분쇄가 아직 안 움직였으므로 DJ_ERROR 의 전부 정지와
		 * "교반만 차단"(R3 절대조건 ①)의 결과가 같다. 자가세척에선 물이 찬 채 멈춘다(배수문 닫힘). */
		(void)Voice_Play(VOICE_E_GUIDE, VOICE_PRIO_ERROR);
		dj_fail(c, DJ_ERR_GUIDE, now);
		break;

	case RINSE_FAULT:
		if (c->rinse.fault == (uint8_t)RINSE_FAULT_FILL)
		{
			/* 음성: "급수가 확인되지 않습니다" - 에러 우선순위(§0.14.11). */
			(void)Voice_Play(VOICE_E_FILL, VOICE_PRIO_ERROR);
			dj_fail(c, DJ_ERR_RINSE_FILL, now);
		}
		else if (c->rinse.fault == (uint8_t)RINSE_FAULT_GUIDE_LOST)
		{
			/* ★§0.38 C094·C060: 운전 중 가이드 소실(B006) = E01(해제형). 음성 05-01. */
			(void)Voice_Play(VOICE_E_GUIDE, VOICE_PRIO_ERROR);
			dj_fail(c, DJ_ERR_GUIDE, now);
		}
		else if (c->rinse.fault == (uint8_t)RINSE_FAULT_OVERSPEED)
		{
			/* ★§0.33 G3: 과속 = 벤더 E02(해제형). 음성 05-02 "교반 속도가 설정 기준을 벗어나…". */
			(void)Voice_Play(VOICE_E_STIR_SPEED, VOICE_PRIO_ERROR);
			dj_fail(c, DJ_ERR_OVERSPEED, now);
		}
		else
		{
			/* 음성: "교반 모터 상태를 확인해 주세요" — 검토서 §13.3 06-01 = E09-교반. */
			(void)Voice_Play(VOICE_E_M_STIR, VOICE_PRIO_ERROR);
			dj_fail(c, DJ_ERR_ROTATION, now);
		}
		break;

	default:
		if (!selfclean) { dj_rinse_mirror(c, now); }
		break;
	}
}

/* ---- 배출 서브-FSM (docx 10·11번) --------------------------------------
 * 130분 개방(CW) → 교반 배출(HW 2분) → 교반 정지·개방 유지 → 135분 닫기(CCW)
 * → 배수부 개방. 배출문 방향은 열림=정방향(CW, TDoor_Open=DRV8871_Forward) /
 * 닫힘=역방향(CCW, TDoor_Close=DRV8871_Reverse)으로 임시 정의(벤치 확인 대상).
 * 배출문 개폐는 DJ_DISCH_DOOR_NO_TIMEOUT=1일 때 타임아웃 없이 THALL 리미트
 * 인식까지 계속 구동한다(리미트 미인식으로 DJ_ERROR 낙하 → 전체 정지 금지).
 * 시간 상한은 개방 쪽만 135분 백스톱이 담당하고, 탈출구는 정지요청(DJ_ABORTED). */
static void dj_disc_enter(DongjakCtx *c, DjDischPhase p, uint32_t now)
{
	c->disc_phase = (uint8_t)p;
	c->disc_since = now;
}

/* 배출문 닫기 개시 시점 도달? = 공정 시계 135분(DJ_T_LATCH_MS, ★heater_since 기준) AND 배출 진입
 * 후 최소 배출 창(DJ_DISCH_WINDOW_MS) 경과. 후자는 수거통 확인 대기 등으로 배출
 * 진입이 135분 이후로 밀렸을 때 "열자마자 닫힘"을 막는 가드. */
static uint8_t dj_disch_close_due(const DongjakCtx *c, uint32_t now)
{
	return (uint8_t)((dj_heater_elapsed(c, now) >= (uint32_t)DJ_T_LATCH_MS) &&
	                 ((now - c->state_since) >= (uint32_t)DJ_DISCH_WINDOW_MS));
}

/* 닫기 개시: 교반 정지 → 배출문 역방향(CCW) 구동. 방향 반전 전 코스트로 한 번
 * 끊어 DRV8871 급반전 인러시를 피한다. 이후 CLOSE_T가 리미트까지 계속 돌린다. */
static void dj_disch_begin_close(DongjakCtx *c, uint32_t now)
{
	BldcCtrl_Stop(&g_stir_ctrl);
	TDoor_Stop();
	c->disc_pulse_off = 0U;                        /* 간헐 구동 위상 리셋      */
	TDoor_LimitArm();                              /* 개방 이동의 stale 엣지 제거 */
	dj_tdoor_arm();                                /* 추가회전 래치 초기화      */
	TDoor_Enable(); TDoor_Close((uint8_t)TDOOR_DUTY);
	dj_disc_enter(c, DJ_DS_CLOSE_T, now);
}

/* 리미트 미인식으로 길어질 때의 간헐(펄스) 구동 유지 — DJ_DISCH_DOOR_PULSE.
 * DJ_DISCH_DOOR_PULSE_AFTER_MS까지는 손대지 않으므로(연속 구동) 정상 개폐
 * 시간대의 동작은 기존과 동일하고, 그 이후에만 ON/OFF를 반복해 스톨 열을 줄인다.
 * 휴지 구간엔 코스트 + VM(EN) OFF. 상태가 바뀌는 순간에만 드라이버를 건드린다.
 * ★리미트 엣지 래치는 여기서 재무장하지 않는다 — 재무장하면 휴지 구간에 도달한
 * 엣지가 지워져 영영 인식되지 않는다(무장은 이동 개시 1회뿐).
 * el = 해당 위상(개방/닫힘) 구동 경과, opening: 1=개방(CW) 0=닫힘(CCW). */
static void dj_tdoor_keep_driving(DongjakCtx *c, uint8_t opening, uint32_t el)
{
#if DJ_DISCH_DOOR_PULSE
	uint8_t want_off = 0U;
	if (el >= (uint32_t)DJ_DISCH_DOOR_PULSE_AFTER_MS)
	{
		uint32_t period = (uint32_t)DJ_DISCH_DOOR_PULSE_ON_MS +
		                  (uint32_t)DJ_DISCH_DOOR_PULSE_OFF_MS;
		uint32_t ph     = (el - (uint32_t)DJ_DISCH_DOOR_PULSE_AFTER_MS) % period;
		want_off = (uint8_t)(ph >= (uint32_t)DJ_DISCH_DOOR_PULSE_ON_MS);
	}
	if (want_off == c->disc_pulse_off) { return; }             /* 전환 없음     */
	c->disc_pulse_off = want_off;
	if (want_off) { TDoor_Stop(); TDoor_Disable(); }           /* 휴지          */
	else
	{
		TDoor_Enable();                                        /* 재구동        */
		if (opening) { TDoor_Open ((uint8_t)TDOOR_DUTY); }
		else         { TDoor_Close((uint8_t)TDOOR_DUTY); }
	}
#else
	(void)c; (void)opening; (void)el;
#endif
}

static void dj_discharge_tick(DongjakCtx *c, uint32_t now)
{
	uint32_t el  = now - c->disc_since;
	/* ★§0.30 공통 해제 입력 ① — 배출문이 열린 뒤(EXPEL 이후) 받은 TIMER-OUT(PF8) 하강엣지를 기억한다.
	 * EXPEL 이 플래그를 소비·클리어하기 **전에** 본다. SW 백업(DJ_DISCHARGE_STIR_MS)·135분 백스톱으로
	 * 넘어간 경우는 세우지 않는다 — 하드웨어 DONE 이 없으면 Init-RST 도 없기 때문이다. */
	/* ★§0.37 D2: **닫기 시작 전**(EXPEL·HOLD)에 받은 엣지만 센다. 닫는 중·뒤에 온 엣지는 DONE 과 닫힘 원샷이 겹치지
	 * 않았다는 뜻이라(U26-1 AND 불성립) Init-RST 가 없다. */
	if ((((DjDischPhase)c->disc_phase == DJ_DS_EXPEL) || ((DjDischPhase)c->disc_phase == DJ_DS_HOLD)) &&
	    gpio_ctrl_exti_flag_get(GPIO_EXTI_TIMER_OUT))
	{
		c->disc_timer_done = 1U;
	}
	/* ★§0.37 D1: 회로(AUX) 교반 관측 — MCU 는 명령하지 않고 FG 만 센다. */
	c->disc_aux_fg = g_stir_ctrl.cfg->h->fg_edges - c->disc_fg_base;
	switch ((DjDischPhase)c->disc_phase)
	{
	case DJ_DS_OPEN_T:                                /* 배출문 개방(CW)        */
		if (dj_tdoor_done(TDoor_ReachedOpen(), 1U, el))
		{
			c->disc_open_ms = el;                    /* 종료까지 실측 시간 기록  */
			TDoor_Stop(); TDoor_Disable();           /* 개방 후 DC 미구동(HW 2분타이머 구간) */
			/* 배출문 열림 = HW 2분 타이머 시작. 그 종료는 TIMER-OUT(PF8) 하강엣지로
			 * 인식(신스펙 항목8). 진입 직전 stale 엣지 플래그 제거. */
			gpio_ctrl_exti_flag_clear(GPIO_EXTI_TIMER_OUT);
#if !DJ_DISCH_STIR_BY_CIRCUIT
			dj_stir313_begin(c, now, (uint16_t)DJ_STIR_RPM);
#endif
			dj_disc_enter(c, DJ_DS_EXPEL, now);
		}
		else if (dj_disch_close_due(c, now))
		{
			/* 135분 백스톱: 개방 리미트 미인식이어도 여기서 개방 구동을 접고
			 * 닫기(CCW)로 전환한다. disc_open_ms는 0(미도달)으로 남는다. */
			dj_disch_begin_close(c, now);
		}
		else { dj_tdoor_keep_driving(c, 1U, el); }   /* 개방(CW) 유지 - 길어지면 간헐 */
		break;

	case DJ_DS_EXPEL:                                 /* 교반 배출(HW 2분타이머) */
#if !DJ_DISCH_STIR_BY_CIRCUIT
		dj_stir313_tick(c, now, (uint16_t)DJ_STIR_RPM);
#endif
		/* 2분 완료 = TIMER-OUT 하강엣지(항목8), 미도달 대비 SW 백업(P36 상한 뒤, D2). */
		if (gpio_ctrl_exti_flag_get(GPIO_EXTI_TIMER_OUT)
#if DJ_DISCH_STIR_SW_BACKUP
		    || (el >= (uint32_t)DJ_DISCHARGE_STIR_MS)
#endif
		   )
		{
			if (!gpio_ctrl_exti_flag_get(GPIO_EXTI_TIMER_OUT)) { c->disc_by_backup = 1U; }  /* D2 */
			gpio_ctrl_exti_flag_clear(GPIO_EXTI_TIMER_OUT);
			c->weight_mon = 0U;                      /* D5: 무게 결과 고정(O007) */
			BldcCtrl_Stop(&g_stir_ctrl);             /* 2분 완료 -> 교반 정지(D1: 회로 구간엔 무시됨) */
#if DJ_DISCH_CLOSE_AT_LATCH
			dj_disc_enter(c, DJ_DS_HOLD, now);       /* 문은 열어둔 채 135분 대기 */
#else
			dj_disch_begin_close(c, now);            /* (구 동작) 즉시 닫기      */
#endif
		}
		else if (dj_disch_close_due(c, now))         /* 135분: 2분 판정 실패 백스톱 */
		{
			gpio_ctrl_exti_flag_clear(GPIO_EXTI_TIMER_OUT);
			c->disc_by_backup = 1U;
			c->weight_mon     = 0U;
			dj_disch_begin_close(c, now);
		}
		break;

	case DJ_DS_HOLD:                                  /* 교반 정지, 개방 유지    */
		/* 배출문은 열린 채(코스트) 135분까지 대기. HW 2분 래치는 이미 해제됨. */
		if (dj_disch_close_due(c, now)) { dj_disch_begin_close(c, now); }
		break;

	case DJ_DS_CLOSE_T:                               /* 배출문 닫음(CCW, 래치 해제) */
		if (dj_tdoor_done(TDoor_ReachedClose(), 0U, el))
		{
			c->disc_close_ms = el;                   /* 종료까지 실측 시간 기록  */
			/* ★§0.30 공통 해제 입력 ② — THALL 을 실제로 봤는가(추가회전 래치가 걸렸는가). 20 s 상한으로
			 * 끝났으면 T-HALL-PULSE 도 없었다고 본다. */
			c->disc_close_hall = dj_tdoor_hit;
			TDoor_Stop(); TDoor_Disable();
#if DJ_SELFCLEAN_ENABLE
			/* ★§0.37 D2·D5: 해제 추정 = 닫기 전 TIMER-OUT(백업 아님) AND 닫힘 THALL [AND 무게 이력]. */
			if (!(c->disc_timer_done && !c->disc_by_backup && c->disc_close_hall
#if DJ_RELEASE_REQUIRE_WEIGHT
			      && c->f_weight
#endif
			     ))
			{
				/* 벤더 O010 → X008: 공통 해제(K2·K3) 미성립 → 잠금 유지, **정상 완료로 기록하지 않는다**
				 * (cycle_count 불변, 03-05 없음). 자가세척도 하지 않는다 — K3 가 서 있으면 급수가 안 된다.
				 * 음성 05-06 "배출이 확인되지 않아 잠금을 유지합니다. 점검을 요청해 주세요". */
				(void)Voice_Play(VOICE_E_DISCHARGE_LOCK, VOICE_PRIO_ERROR);
				dj_fail(c, DJ_ERR_RELEASE, now);
				break;
			}
			/* ★§0.30 자동 자가세척 (09.16 O016 → R001~R019 → M001, 0.3.0 동일) — **매 정상
			 * 처리·배출·공통 해제 후**. 처리 1회는 여기서 완료로 센다(자가세척이 실패해도 처리는 끝났다).
			 * 음성 2개를 이어서: 03-05 "처리가 끝나고, 배출이 완료되었습니다…" → 04-01
			 * "자가세척 기능이 시작됩니다". Voice_Play 두 번은 정상 큐(1칸)에서 앞의 것이 빠진다. */
			c->cycle_count++;
			(void)Voice_PlaySeq2(VOICE_DISCHARGE_DONE, VOICE_SELFCLEAN_START);
			dj_enter(c, DJ_SELFCLEAN, now);
#else
			dj_wdoor_open();                             /* 배수부 완전개방(배수밸브 동반) */
			dj_disc_enter(c, DJ_DS_OPEN_W, now);
#endif
		}
		/* 닫힘은 THALL 인식 + 2.8s 추가회전(TDOOR_CLOSE_OVERRUN_MS), 상한 TDOOR_CLOSE_MAX_MS(20s). */
		else { dj_tdoor_keep_driving(c, 0U, el); }
		break;

	case DJ_DS_OPEN_W:
	default:
		if (dj_wdoor_done(WDoor_ReachedOpen(), 1U, el))
		{
			WDoor_Stop(); WDoor_Disable();
			c->cycle_count++;
			/* 음성: "처리가 끝나고, 배출이 완료되었습니다…" (§0.14.11).
			 * 배수부 개방까지 끝난 시나리오 완료 시점이다. DJ_DONE 은 IDLE 로 내려가는
			 * 상태(★2026-09-22 부터 3 s 유지)라 폴링이 아니라 이 전이에 둔다. */
			(void)Voice_Play(VOICE_DISCHARGE_DONE, VOICE_PRIO_NORMAL);
			dj_enter(c, DJ_DONE, now);
		}
		else { WDoor_Open(WDoor_OpenDutyAt(el)); }   /* 킥 80%(2s) -> 65% 유지 */
		break;
	}
}

/* ---- 메인 상태 진입 ------------------------------------------------------ */
static void dj_enter(DongjakCtx *c, DongjakState s, uint32_t now)
{
	c->state       = (uint8_t)s;
	c->state_since = now;

	switch (s)
	{
	/* ★§0.30: DJ_PREP · DJ_RINSE1_x · DJ_RINSE2_x 는 rinse.c 단계의 미러라 진입 액션이 없다
	 * (dj_rinse_mirror 가 state 만 바꾼다). 헹굼 시작은 DJ_IDLE 의 Rinse_Begin. */

	case DJ_SELFCLEAN:
	{
		/* ★§0.30 자가세척 — 음식물 없이 헹굼 1회(벤더 O016 → R007~R019 → M001).
		 * 벤더 계약: 자가세척 중 heater=false · grind=STOP · cam_forward=false(REV02 캠 없음),
		 * 다시 가열하지 않는다(C090). 배출까지 돌던 팬·수증기 부하도 여기서 내린다.
		 * 진입 전에 공통 해제(K3)를 확인했으므로(DJ_DS_CLOSE_T) 온도를 기다리지 않고 급수한다 — K3 는
		 * 식는 중에는 다시 서지 않는다(사용자 HW 확인 2026-09-22).
		 * 순서는 벤더 self_clean_guide_pending: 배수문 닫힘 → 급수 → 준비 탐색 → WASH → 배수. */
		RinseConfig  cfg;
		RinseOutputs o;
		gpio_ctrl_off(GPIO_OUT_HT_POWER);
		BldcCtrl_Stop(&g_grind_ctrl);
		c->grind_mode = (uint8_t)DJ_GM_OFF;
		gpio_ctrl_off(GPIO_OUT_FAN_VAPOR);
		gpio_ctrl_off(GPIO_OUT_FAN_EXHAUST);
		gpio_ctrl_off(GPIO_OUT_BLDC_FAN);
		c->fanb_on = 0U; c->fanb_active = 0U; c->fanx_on = 0U;
		StepMotor_Release(&dj_duct);
		StepMotor_Release(&dj_air);
		cfg.repeats         = (uint8_t)DJ_SELFCLEAN_REPEATS;
		cfg.rot_rpm         = (uint16_t)DJ_RINSE_STIR_RPM;
		cfg.fill_timeout_ms = (uint32_t)DJ_FILL_TIMEOUT_MS;
		cfg.prep_after_fill = 1U;
		Rinse_Begin(&c->rinse, &cfg, now, &o);
		dj_rinse_apply(c, &o);
		break;
	}

	case DJ_HEAT:
		c->heat_started = 0U;                        /* 먼저 배수문 닫힘 확인   */
		c->t_low_seen = 0U; c->t_high_seen = 0U;     /* [TEST] §0.47 단축 시험 관측 리셋 */
		c->cool_exit_by = 0U;                        /* §0.48 식힘 종료 사유 리셋 */
		c->pair_started = 0U; c->pair_since = 0U;
		c->dry_gate     = 0U;                        /* R3 C028: 허가 재대기    */
#if DJ_TEST_BEEP
		c->test_beeped_hi = 0U;                      /* [TEST] HISPEED 비프 재무장 */
#endif
		dj_wdoor_close();
		break;

	case DJ_COOLDOWN:
		gpio_ctrl_off(GPIO_OUT_HT_POWER);            /* 히터 OFF               */
		c->cool_exit_by = 0U;                        /* §0.48 종료 사유 — 진입마다 리셋(식힘부터 점프 포함) */
		/* 식힘 진입 = 아직 뜨거움: 교반은 "지속 CW"(1회만 기동, 토글 아님),
		 * 분쇄는 tick에서 1000 CCW. 80℃ 미만으로 식으면 cool_phase=1 전환. */
		c->cool_phase = 0U;
		gpio_ctrl_exti_flag_clear(GPIO_EXTI_BIMETAL_70); /* 바이메탈 stale 엣지 제거 */
		/* ★80℃'이상' 구간의 지속 CW 는 2026-08-26 변경 대상이 아니다(지시는
		 * '식힘 교반 80℃ 미만'). 그래서 20RPM 전용 상수를 따로 쓴다. */
		dj_stir_spin(0U, (uint16_t)DJ_COOL_HOT_STIR_RPM);
#if DJ_TEST_BEEP
		dj_test_beep(2U);                            /* [TEST] 8분대(120분) 삑삑 */
#endif
		break;

	case DJ_BIN_CHECK:
		BldcCtrl_Stop(&g_stir_ctrl);
		BldcCtrl_Stop(&g_grind_ctrl);
		gpio_ctrl_off(GPIO_OUT_BLDC_FAN);    c->fanb_on = 0U;
		/* 내부정화팬(fanx)은 배출 완료(래치 해제)까지 유지 - 끄지 않음 */
		c->bin_warned = 0U;                          /* 03-04 안내 래치 초기화 */
		break;

	case DJ_DISCHARGE:
#if DJ_TEST_BEEP
		dj_test_beep(3U);                            /* [TEST] 9분대(130분) 삑삑삑 */
#endif
		/* 배출문 개방 = 정방향(CW). 리미트(THALL_OPEN, PF5) 인식까지 계속 구동. */
		c->disc_open_ms = 0U; c->disc_close_ms = 0U; c->disc_pulse_off = 0U;
		c->disc_timer_done = 0U; c->disc_close_hall = 0U;   /* ★§0.30 공통 해제 추정 리셋 */
		c->disc_by_backup  = 0U;                             /* ★§0.37 D2 */
		/* ★§0.37 D1: 교반은 회로 소관 — MCU 는 세워 두고(폐루프·잼 판정 없음) FG 만 본다. */
		BldcCtrl_Stop(&g_stir_ctrl);
		c->disc_fg_base = g_stir_ctrl.cfg->h->fg_edges; c->disc_aux_fg = 0U;
		/* ★§0.37 D5: 무게 이력 초기화 + 감시 시작(O002 — 문 열기 시작). 샘플은 SenseTick. */
		c->f_weight = 0U; c->weight_armed = 0U; c->weight_run_since = 0U; c->weight_run_ms = 0U;
		c->weight_mon = (uint8_t)DJ_WEIGHT_ENABLE;
		TDoor_LimitArm();                            /* 리미트 엣지 래치 무장    */
		dj_tdoor_arm();                              /* 추가회전 래치 초기화      */
		TDoor_Enable(); TDoor_Open((uint8_t)TDOOR_DUTY);
		dj_disc_enter(c, DJ_DS_OPEN_T, now);
		break;

	case DJ_ABORTED:
		/* 4.5 비상정지: 전 액추에이터 즉시 OFF. 냉각팬은 tick에서 온도로 관리. */
		dj_all_off();
		c->fanb_on = 0U; c->fanx_on = 0U;   /* 배기팬 duty도 정지(dj_all_off가 GPIO OFF) */
		c->lid_guard = 0U;               /* 감시 해제(재트리거 방지)             */
		/* TODO(음성): "처리 중단/추가 투입 불가" 안내 멘트 (미구현) */
		break;

	case DJ_DONE:
		dj_all_off();
		c->fanb_on = 0U; c->fanx_on = 0U;   /* 배기팬 duty 정지 */
		break;

	case DJ_ERROR:
		/* 히터·교반·분쇄·급수·밸브·문·스텝을 끈다(벤더 에러 단계와 같다). 팬은 tick 의
		 * dj_error_fans 가 다시 쥔다(G1·G2) — 배기팬은 위상(fanx_on·fanx_since)을 살려 두고 곧바로 복원. */
		dj_all_off();
		c->grind_mode  = (uint8_t)DJ_GM_OFF;   /* 모터 냉각팬 조건 = 온도만 */
		c->fanb_on     = 0U; c->fanb_active = 0U;
		if (dj_error_keep_exhaust(c) && c->fanx_on) { gpio_ctrl_on(GPIO_OUT_FAN_EXHAUST); }
		else if (!dj_error_keep_exhaust(c))         { c->fanx_on = 0U; }
		break;

	case DJ_IDLE:
	default:
		break;
	}
}

/* ---- API ----------------------------------------------------------------- */
void Dongjak_Init(void)
{
	DongjakCtx *c = &g_dongjak;
	c->state = (uint8_t)DJ_IDLE; c->state_since = 0U; c->scn_start = 0U;
	c->start_req = 0U; c->dbg_force_start = 0U; c->dbg_enter_heat = 0U;
	c->dbg_enter_cool = 0U; c->dbg_enter_selfclean = 0U;
#if DJ_TEST_BEEP
	c->dbg_beep = 0U;
#endif
	c->temp_d10 = 0; c->temp_valid = 0U; c->temp2_d10 = 0; c->temp2_valid = 0U; c->vapor_temp_d10 = 0;
	c->water_reached = 0U; c->bin_fill_pct = 0U; c->hs_prev = 0U; c->bin_present = 0U; c->bin_warned = 0U;
	c->abort_req = 0U; c->dbg_force_stop = 0U; c->lid_guard = 0U; c->lid_low_cnt = 0U;
	c->err_code = (uint8_t)DJ_ERR_NONE; c->err_clear_req = 0U;
	c->last_err = (uint8_t)DJ_ERR_NONE; c->err_from = (uint8_t)DJ_IDLE;
	c->cycle_count = 0U;
	c->heat_started = 0U; c->heat_since = 0U;
	c->dry_gate = 0U; c->grind_allow = 0U;        /* R3 C028/C031: 허가 게이트 리셋 */
	c->t_low_seen = 0U; c->t_high_seen = 0U; c->pair_started = 0U; c->pair_since = 0U; /* [TEST] §0.47 */
	c->cool_exit_by = 0U;                                   /* §0.48 */
	c->heat_door_ok = 0U;                         /* §0.36 C026 H1 */
	c->heater_started = 0U; c->heater_since = 0U; /* R3 C005: 공정 시계 리셋        */
	Rinse_Reset(&c->rinse);                       /* §0.27 준비 탐색               */
#if DJ_TEST_BEEP
	c->test_beeped_hi = 0U;                       /* [TEST] 비프 래치 리셋 */
#endif
	c->stir_phase = 0U; c->stir_since = 0U; c->stir_reps = 0U; c->stir_mode = 0U;
	c->heat_off_latch = 0U;
	c->grind_mode = (uint8_t)DJ_GM_OFF; c->grind_phase = 0U; c->grind_final_ph = 0U;
	c->grind_since = 0U; c->grind_start = 0U; c->cool_phase = 0U;
	c->vapor_phase = (uint8_t)DJ_VP_CLOSED; c->vapor_step_since = 0U;
	c->vapor_move_since = 0U; c->vapor_step_cnt = 0U; c->vapor_close_done = 0U;
	c->fanb_on = 0U; c->fanb_active = 0U; c->fanb_since = 0U;
	c->fanx_on = 0U; c->fanx_since = 0U;
	c->disc_phase = 0U; c->disc_since = 0U;
	c->disc_open_ms = 0U; c->disc_close_ms = 0U; c->disc_pulse_off = 0U;
	c->disc_timer_done = 0U; c->disc_close_hall = 0U;
	c->disc_by_backup = 0U; c->disc_fg_base = 0U; c->disc_aux_fg = 0U;           /* §0.37 */
	c->weight_raw = 0U; c->weight_mon = 0U; c->weight_armed = 0U; c->weight_base = 0U;
	c->weight_delta = 0U; c->weight_run_since = 0U; c->weight_run_ms = 0U; c->f_weight = 0U;
	StepMotor_Init(&dj_duct);        /* 코일 해제 + 시퀀스 인덱스 리셋         */
	StepMotor_Init(&dj_air);
	dj_all_off();
}

void Dongjak_Start(void)
{
	/* ★G3: 모음 쪽에 래치형 에러가 남아 있어도 시작하지 않는다(벤더: 오류 대기 중엔 새 모드 선택을 받지 않음).
	 * 자기 쪽 래치형 에러는 state == DJ_ERROR 라 아래 조건에서 이미 걸린다. */
	/* ★2026-09-22: 완료 유지(DJ_DONE, DJ_DONE_HOLD_MS) 중에도 받는다 — MotorTick 이 곧바로 IDLE 로 넘기고 시작. */
	if (((g_dongjak.state == (uint8_t)DJ_IDLE) || (g_dongjak.state == (uint8_t)DJ_DONE)) &&
	    (Moeum_IsLatched() == 0U)) { g_dongjak.start_req = 1U; }
}

/* ★G3 [2026-09-22 §0.33] 에러 해제 분류(dongjak.h DJ_ERR_LATCH_ENABLE 주석). 모음도 같은 표를 쓴다. */
uint8_t Dongjak_ErrIsLatched(uint8_t code)
{
#if DJ_ERR_LATCH_ENABLE
	return (uint8_t)((code != (uint8_t)DJ_ERR_NONE) && (code != (uint8_t)DJ_ERR_GUIDE) &&
	                 (code != (uint8_t)DJ_ERR_OVERSPEED));
#else
	(void)code;
	return 0U;
#endif
}

uint8_t Dongjak_IsLatched(void)
{
	return (uint8_t)((g_dongjak.state == (uint8_t)DJ_ERROR) && Dongjak_ErrIsLatched(g_dongjak.err_code));
}

void Dongjak_Abort(void)
{
	dj_all_off();
	g_dongjak.start_req    = 0U;
	g_dongjak.dbg_enter_heat = 0U;
	g_dongjak.dbg_enter_cool = 0U;
	g_dongjak.dbg_enter_selfclean = 0U;
	g_dongjak.abort_req    = 0U;
	g_dongjak.dbg_force_stop = 0U;
	g_dongjak.lid_guard    = 0U;
	g_dongjak.lid_low_cnt  = 0U;
	g_dongjak.heat_started = 0U;
	g_dongjak.heat_off_latch = 0U;
	g_dongjak.dry_gate     = 0U;                  /* R3 C028: 허가 게이트 리셋    */
	g_dongjak.grind_allow  = 0U;
	g_dongjak.heater_started = 0U;                /* R3 C005: 공정 시계 리셋      */
	g_dongjak.heater_since   = 0U;
	g_dongjak.stir_reps    = 0U;
	g_dongjak.stir_mode    = 0U;
	g_dongjak.grind_mode   = (uint8_t)DJ_GM_OFF;
	g_dongjak.cool_phase   = 0U;
	g_dongjak.vapor_phase  = (uint8_t)DJ_VP_CLOSED;
	g_dongjak.vapor_close_done = 0U;
	g_dongjak.fanb_on      = 0U;
	g_dongjak.fanb_active  = 0U;
	g_dongjak.fanx_on      = 0U;
	g_dongjak.weight_mon   = 0U;                  /* §0.37 D5 */
	/* ★G3·G4: 래치형 에러는 마개 이탈·모드 전환·정지로 풀리지 않는다 — DJ_ERROR 와 err_code 를 그대로
	 * 둔다(출력은 위 dj_all_off 로 이미 꺼짐, 잔열 냉각은 jungji 가 쥔다). 해제형·비에러는 종전대로 IDLE.
	 * last_err 는 어느 쪽이든 남긴다. */
	if (Dongjak_IsLatched() != 0U) { return; }
	g_dongjak.err_code     = (uint8_t)DJ_ERR_NONE;
	g_dongjak.err_clear_req = 0U;
	g_dongjak.state        = (uint8_t)DJ_IDLE;
	g_dongjak.state_since  = HAL_GetTick();
}

/* 4.5 정지 요청(외부 트리거용: 앱 PROTO_ACT_STOP / 디버거. 맴브레인 버튼 제어는
 * 2026-08-18 폐지). BUSY일 때만 유효 - MotorTick가 소비하여 DJ_ABORTED(안전 식힘)로
 * 전이한다. 마개 HS3·이탈 경로는 jungji가 담당. */
void Dongjak_RequestStop(void)
{
	g_dongjak.abort_req = 1U;
}

/* DJ_ERROR -> IDLE 복구 요청(외부 리셋/확인 버튼용). DJ_ERROR 상태에서만 MotorTick가
 * 소비하며(err_clear_req), 그 외 상태에선 무시된다. err_code는 복구 시 NONE으로 클리어. */
void Dongjak_ClearError(void)
{
	g_dongjak.err_clear_req = 1U;
}

/* [디버그/벤치 전용] 헹굼을 건너뛰고 DJ_HEAT부터 시작. 헤더 설명 참조.
 * dj_enter로 정상 진입시켜 상태/타이밍을 일관되게 세팅하고, skip_door=1이면
 * heat_started 0->1 전이(도어확인 성공 경로)를 그대로 복제해 교반 BLDC까지 즉시
 * 개시한다(수동 poke의 '교반 미시작' 함정 회피). g_app_mode=DONGJAK에서만 tick이
 * 돌아 실제로 진행된다. */
void Dongjak_DebugEnterHeat(uint8_t skip_door)
{
	DongjakCtx *c   = &g_dongjak;
	uint32_t    now = HAL_GetTick();

	RotStir_Stop(&c->rot);                           /* ★개정4 ④: 헹굼 중 점프해도 회전수 운전 해제 */
	Rinse_Reset(&c->rinse);                          /* ★2026-09-22 앱 동기화: 지난 회차 cnt·단계를 지운다(헹굼 안 함) */
	c->scn_start = now;                              /* 시나리오 경과 0부터        */
	c->vapor_close_done = 0U;                        /* R3 C058: 84℃ 폐쇄 래치 해제 */
	c->heat_off_latch   = 0U;                        /* R3 C041: 새 운전이므로 해제 */
	c->heater_started   = 0U;                        /* R3 C005: 첫 히터 ON 에서 시계 개시 */
	dj_fan_exhaust_begin(c, now);                    /* 배기팬 10/5 duty 개시(벤치도 동일) */
	dj_enter(c, DJ_HEAT, now);                       /* heat_started=0 + 배수문 닫힘 */

	if (skip_door)
	{
		WDoor_Stop(); WDoor_Disable();               /* 도어 구동 즉시 정지        */
		c->heat_started = 1U; c->heat_since = now;   /* 가열중으로 강제            */
		/* ★R3 C028: 여기서도 교반을 켜지 않는다 — 허가 게이트를 똑같이 따른다.
		 * 70℃ 바이메탈을 붙일 수 없는 벤치에서는 DJ_DRY_GATE_BYPASS=1 을 쓸 것. */
		c->dry_gate = 0U;
		BldcCtrl_Stop(&g_stir_ctrl);
		c->grind_mode  = (uint8_t)DJ_GM_OFF; c->grind_start = 0U;
		c->vapor_phase = (uint8_t)DJ_VP_CLOSED;
		c->fanb_on = 0U; c->fanb_active = 0U; c->fanb_since = now;
	}
}

/* [디버그/벤치 전용] 헹굼·건조를 건너뛰고 **식힘 교반(80℃ 미만)** 부터 시작.
 * 2026-08-26 신설 - 앱 '동작 (식힘부터)' 버튼(PROTO_ACT_DJ_COOL).
 *
 * DJ_HEAT 직행(Dongjak_DebugEnterHeat)과 두 가지가 다르다:
 *
 * 1) scn_start 를 now 가 아니라 **now - DJ_T_COOLDOWN_MS(120분)** 로 잡는다.
 *    식힘은 시나리오 중간 구간이라 경과를 0 으로 리셋하면 (a) 앱 타임라인이
 *    "120분 구간인데 경과 0분"으로 어긋남 표시를 내고(enums.dj_state_range(14)
 *    의 하한이 식힘 마커다), (b) 130분 배출 전환까지 130분을 통째로 기다리게
 *    된다. 120분 지점에서 시작한 것으로 만들면 10분 뒤 배출로 자연히 이어진다.
 *    ※ uint32 modular 연산이라 부팅 직후(now < 120분)에도 (now - scn_start)
 *      = DJ_T_COOLDOWN_MS 로 정확히 나온다.
 *
 * 2) 진입 후 cool_phase 를 **1(식음)** 로 강제한다. dj_enter(DJ_COOLDOWN) 은
 *    cool_phase=0(뜨거움: 분쇄 1000 CCW + 교반 지속 CW 20RPM)으로 들어가므로,
 *    그대로 두면 온도가 80℃ 밑으로 내려갈 때까지 목적 동작이 시작되지 않는다.
 *    분쇄 OFF + 교반(R3 C043: 20RPM 정3/정지1/역3)까지 여기서 세워 둔다.
 *
 * g_app_mode=DONGJAK 에서만 tick 이 돌아 실제로 진행된다. */
void Dongjak_DebugEnterCool(void)
{
	DongjakCtx *c   = &g_dongjak;
	uint32_t    now = HAL_GetTick();

	RotStir_Stop(&c->rot);                           /* ★개정4 ④: 회전수 운전 해제 */
	Rinse_Reset(&c->rinse);                          /* ★2026-09-22 앱 동기화: 지난 회차 cnt·단계를 지운다 */
	dj_vapor_off(c);                                 /* 수증기 루프 종료(HEAT 전용) */
	dj_fan_exhaust_begin(c, now);                    /* 배기팬 10/5 duty 개시     */
	dj_enter(c, DJ_COOLDOWN, now);                   /* 히터 OFF + cool_phase=0   */
	c->scn_start = now - (uint32_t)DJ_T_COOLDOWN_MS; /* 경과 = 120분 지점(앱 표시)  */
	/* ★R3 C005: 130분 배출 판정은 이제 heater_since 기준이다 — 공정 시계도 120분 지점으로
	 * 맞춰야 10분 뒤 배출로 이어진다(히터를 켠 적이 없는 벤치 점프라 합성값이다). */
	c->heater_started = 1U;
	c->heater_since   = now - (uint32_t)DJ_T_COOLDOWN_MS;

	/* '80℃ 미만' 구간으로 강제: 분쇄 OFF + 교반 3구간 패턴 개시.
	 * (R3 C043 이후 이 구간은 20RPM 정3/정지1/역3 = DJ_COOL_* 상수) */
	dj_grind_apply_mode(c, DJ_GM_OFF, now);
	dj_stir_alt_begin(c, now, (uint16_t)DJ_COOL_STIR_RPM);
	c->cool_phase = 1U;
}

/* [디버그/벤치 전용] 처리·배출을 건너뛰고 자가세척부터. 헤더 설명 참조 [2026-09-22 앱 동기화].
 * 정상 경로(DJ_DS_CLOSE_T → dj_enter(DJ_SELFCLEAN))와 같은 진입 함수를 쓴다 — 히터·분쇄·팬·스텝
 * 정리와 Rinse_Begin(repeats = DJ_SELFCLEAN_REPEATS, 급수 뒤 준비 탐색)이 그대로 적용된다.
 * 음성은 04-01(시작)만 — 03-05(배출 완료)는 배출이 없었으므로 내지 않는다. */
void Dongjak_DebugEnterSelfclean(void)
{
	DongjakCtx *c   = &g_dongjak;
	uint32_t    now = HAL_GetTick();
	DongjakState s  = (DongjakState)c->state;

	if (!((s == DJ_IDLE) || (s == DJ_DONE) ||
	      ((s == DJ_ERROR) && !Dongjak_ErrIsLatched(c->err_code))))
	{
		return;                                      /* 처리 중·래치형 에러 — 시작 규칙과 같이 거른다 */
	}
	if (Moeum_IsLatched() != 0U) { return; }        /* ★G3: 모음 쪽 래치형 에러도 막는다 */

	RotStir_Stop(&c->rot);
	c->err_code  = (uint8_t)DJ_ERR_NONE;
	c->last_err  = (uint8_t)DJ_ERR_NONE;             /* 새 운전 */
	c->scn_start = now;                              /* 경과 0 부터 */
	c->heater_started = 0U;                          /* 히터는 켜지 않는다(C090) — 공정 시계 없음 */
	c->heat_off_latch = 0U;
	(void)Voice_Play(VOICE_SELFCLEAN_START, VOICE_PRIO_NORMAL);
	dj_enter(c, DJ_SELFCLEAN, now);
}

/* 100ms, StartDefaultTask - 센서만. */
#if DJ_WEIGHT_ENABLE
/* ★R3 C050·C078 D5 [2026-09-22, §0.37] — 무게 유효 변화 연속 판정(벤더 O005 / UW6 2.108 s 를 SW 로).
 * 100 ms SenseTick. 감시 구간 플래그(weight_mon)는 MotorTick 이 배출 시작에 세우고 타이머 완료에 내린다.
 * 기준 = 구간 첫 샘플. 변화 = |raw - 기준| ≥ DJ_WEIGHT_DELTA_COUNTS(TBD: N20). 끊기면 0 부터(합산 금지),
 * 한 번 래치되면 다음 배출 시작까지 유지. 실패한 읽기는 연속을 끊는다(안전측). */
static void dj_weight_sense(DongjakCtx *c, uint32_t now)
{
	uint16_t raw = 0U;
	uint8_t  ok  = (uint8_t)(AdcCtrl_Read(ADC_CH_WEIGHT, &raw) == HAL_OK);

	if (ok) { c->weight_raw = raw; }
	if (!c->weight_mon) { c->weight_armed = 0U; c->weight_run_since = 0U; c->weight_run_ms = 0U; return; }
	if (!ok) { c->weight_run_since = 0U; c->weight_run_ms = 0U; return; }
	if (!c->weight_armed) { c->weight_base = raw; c->weight_armed = 1U; c->weight_delta = 0U; return; }

	c->weight_delta = (uint16_t)((raw > c->weight_base) ? (raw - c->weight_base) : (c->weight_base - raw));
	if (c->weight_delta >= (uint16_t)DJ_WEIGHT_DELTA_COUNTS)
	{
		if (c->weight_run_since == 0U) { c->weight_run_since = now | 1U; }
		c->weight_run_ms = (uint16_t)(((now - c->weight_run_since) > 0xFFFFU) ? 0xFFFFU : (now - c->weight_run_since));
		if (c->weight_run_ms >= (uint16_t)DJ_WEIGHT_CONTINUOUS_MS) { c->f_weight = 1U; }   /* 래치 */
	}
	else
	{
		c->weight_run_since = 0U; c->weight_run_ms = 0U;   /* 합산 금지(TB-J05) */
	}
}
#endif

void Dongjak_SenseTick(void)
{
#if DJ_HS_TRIGGER_INTERNAL
	uint8_t hs      = HallSensor_Get((uint8_t)DJ_HS_START_IDX);
	uint8_t hs_edge = (uint8_t)((hs != 0U) && (g_dongjak.hs_prev == 0U));
	uint8_t force   = g_dongjak.dbg_force_start;
	if (hs_edge || force)
	{
		g_dongjak.dbg_force_start = 0U;
		if (g_dongjak.state == (uint8_t)DJ_IDLE)
		{
			/* 실제 HS 상승에지 시작만 투입구 감시 무장(벤치 강제시작은 HS 없이
			 * 돌 수 있어 무장 시 즉시 오정지). */
			g_dongjak.lid_guard   = (uint8_t)(hs_edge && (force == 0U));
			g_dongjak.lid_low_cnt = 0U;
		}
		Dongjak_Start();
	}
	g_dongjak.hs_prev = hs;

#if DJ_LID_OPEN_ABORT
	/* 4.5 투입구 개방 감시: 실제 HS 시작으로 무장된 상태에서 처리 중 마개가 동작
	 * 위치를 벗어나면(HS LOW 연속 N회) 비상정지 요청. */
	if (g_dongjak.lid_guard && Dongjak_IsBusy())
	{
		if (hs == 0U)
		{
			if (g_dongjak.lid_low_cnt < 255U) { g_dongjak.lid_low_cnt++; }
			if (g_dongjak.lid_low_cnt >= (uint8_t)DJ_LID_CONFIRM_SAMPLES)
			{ g_dongjak.abort_req = 1U; }
		}
		else { g_dongjak.lid_low_cnt = 0U; }
	}
#endif
#else  /* !DJ_HS_TRIGGER_INTERNAL - 중재자(mode_arbiter.c)가 트리거를 소유 */
	/* HS2 시작 에지와 마개 이탈(HS_LOST) 감시는 중재자가 담당한다. 여기서 또
	 * 읽으면 같은 이벤트가 이중 처리된다. 벤치 강제 시작만 유효. */
	if (g_dongjak.dbg_force_start != 0U)
	{
		g_dongjak.dbg_force_start = 0U;
		g_dongjak.lid_guard       = 0U;
		g_dongjak.lid_low_cnt     = 0U;
		Dongjak_Start();
	}
#endif
	/* 4.5 정지 모사: 벤치/외부 트리거로 dbg_force_stop 세팅 시 정지(디버거 전용 훅). */
	if (g_dongjak.dbg_force_stop != 0U)
	{
		g_dongjak.dbg_force_stop = 0U;
		g_dongjak.abort_req = 1U;
	}

	{
		/* CH0(처리통): 값은 직전 유효값 유지하되, 에러(5회 연속 불량으로 디바운스됨)는
		 * temp_valid로 전파 -> 히터가 온도 미상 상태에서 강제 OFF 되도록. */
		int16_t t = g_therm_c_d10[DJ_TEMP_CH];
		if (t != THERMISTOR_ERR_D10) { g_dongjak.temp_d10 = t; g_dongjak.temp_valid = 1U; }
		else                         { g_dongjak.temp_valid = 0U; }
		/* CH1(THERM2): 식힘 분쇄정지 보조. 값 유지 + 에러는 temp2_valid로 전파(에러 시
		 * 식힘 판정에서 '안 식음' 취급 - 보수적). */
		int16_t t2 = g_therm_c_d10[DJ_TEMP2_CH];
		if (t2 != THERMISTOR_ERR_D10) { g_dongjak.temp2_d10 = t2; g_dongjak.temp2_valid = 1U; }
		else                          { g_dongjak.temp2_valid = 0U; }
		/* 수증기 제어는 별도 채널(THERM3/J23) - 에러 시 직전 유효값 유지 */
		int16_t tv = g_therm_c_d10[DJ_VAPOR_TEMP_CH];
		if (tv != THERMISTOR_ERR_D10) { g_dongjak.vapor_temp_d10 = tv; }
	}
	if (dj_water_present()) { g_dongjak.water_reached = 1U; }
	g_dongjak.bin_fill_pct = g_bin_fill_pct;
#if DJ_BIN_USE_HS7
	g_dongjak.bin_present = (uint8_t)(HallSensor_Get((uint8_t)DJ_HS_BIN_IDX) ==
	                                  (uint8_t)DJ_BIN_PRESENT_LEVEL);
#else
	g_dongjak.bin_present = 1U;
#endif
#if DJ_WEIGHT_ENABLE
	dj_weight_sense(&g_dongjak, HAL_GetTick());      /* ★§0.37 D5 — ADC1 소유 태스크(defaultTask)에서 */
#endif
}

/* 1ms, StartMotorTask - 상태머신 소유 + 모든 액추에이터 구동. */
void Dongjak_MotorTick(uint32_t now_ms)
{
	DongjakCtx *c  = &g_dongjak;

	/* [디버그] DJ_HEAT 점프 트리거 소비(1회성). el/sel 계산 전에 처리해 scn_start/
	 * state_since가 now로 리셋되도록 한다. 이 참조가 링커의 함수 제거도 막는다. */
	if (c->dbg_enter_heat)
	{
		uint8_t sk = (uint8_t)(c->dbg_enter_heat == 2U);
		c->dbg_enter_heat = 0U;
		Dongjak_DebugEnterHeat(sk);
	}
	if (c->dbg_enter_cool)
	{
		c->dbg_enter_cool = 0U;
		Dongjak_DebugEnterCool();
	}
	if (c->dbg_enter_selfclean)
	{
		c->dbg_enter_selfclean = 0U;
		Dongjak_DebugEnterSelfclean();
	}

#if DJ_TEST_BEEP
	/* [TEST] 수동 비프 트리거: 디버거에서 g_dongjak.dbg_beep = N 쓰면 즉시 N회 삑.
	 * 스피커/오디오 경로를 시나리오 진행(7분 대기) 없이 바로 검증하는 용도. */
	if (c->dbg_beep)
	{
		uint8_t nb = c->dbg_beep;
		c->dbg_beep = 0U;
		dj_test_beep(nb);
	}
#endif

	uint32_t    el = now_ms - c->state_since;
	/* ★R3 C005·C041: 110/120/130분 판정은 heater_since 기준 공정 시계(dongjak.h 주석).
	 * 변수명 sel 은 유지 — 뜻만 "시나리오 경과" → "최초 히터 ON 이후 경과" 로 바뀌었다. */
	uint32_t    sel = dj_heater_elapsed(c, now_ms);

	/* 4.5 비상정지 요청 소비: 처리 중이면 안전정지(DJ_ABORTED)로 전이.
	 * DJ_ERROR에서 정지요청이 오면 에러를 해제하고 IDLE로 복구(정지=리셋 겸용). */
	if (c->abort_req)
	{
		c->abort_req = 0U;
		if ((DongjakState)c->state == DJ_ERROR)
		{
			/* ★G3: 래치형 에러(E03·E04·E08·E09)는 정지 요청으로 풀지 않는다 — ClearError 로만. */
			if (!Dongjak_ErrIsLatched(c->err_code)) { dj_error_release(c, now_ms); }
		}
		else if (Dongjak_IsBusy() && ((DongjakState)c->state != DJ_ABORTED))
		{
			dj_enter(c, DJ_ABORTED, now_ms);
		}
	}

	/* ★모터 폴트 → 에러 정지 [2026-09-22]. bldc_ctrl 은 nFAULT 또는 잼 재시도 소진 시
	 * BLDC_LOCKED(소프트락)로 떨어지고 모터를 세우지만, 시나리오가 그 값을 보지 않아
	 * 나머지 공정(히터·타임라인)이 그대로 진행됐다. 여기서 **다른 구동보다 먼저** 본다 —
	 * 분쇄 토글이 BldcCtrl_Stop() 을 부르면 LOCKED 가 풀려 흔적이 사라지기 때문이다
	 * (BldcCtrl_Tick 은 이 함수 **다음**에 돌므로 직전 tick 의 LOCKED 가 여기서 보인다).
	 *  - 분쇄(M1): 시나리오가 분쇄를 명령 중일 때만(grind_mode != OFF). 검토서 §13.3 06-04.
	 *  - 교반(M2): 처리 중 전 구간. 헹굼의 회전수 운전 중이면 E09(P55 5 s)보다 먼저 잡힌다. 06-01.
	 * ⚠ 허가 없이 분쇄를 명령하면(ENABLE LOW → FG 없음) 약 10 s 뒤 LOCKED 가 된다 — 그 경우도
	 *   여기서 에러로 멈춘다. 벤치에서 DJ_DRY_GATE_BYPASS=1 로 두면 이 에러를 보게 된다. */
	if (Dongjak_IsBusy() && ((DongjakState)c->state != DJ_ABORTED))
	{
		if ((c->grind_mode != (uint8_t)DJ_GM_OFF) &&
		    (g_grind_ctrl.state == (uint8_t)BLDC_LOCKED))
		{
			(void)Voice_Play(VOICE_E_M_GRIND, VOICE_PRIO_ERROR);  /* "분쇄 모터 상태를 확인해 주세요" */
			dj_fail(c, DJ_ERR_GRIND_MOTOR, now_ms);
		}
		else if ((g_stir_ctrl.state == (uint8_t)BLDC_LOCKED)
#if DJ_DISCH_STIR_BY_CIRCUIT
		         && ((DongjakState)c->state != DJ_DISCHARGE)   /* ★§0.37 D1: 배출 중 교반은 회로 소관 */
#endif
		        )
		{
			(void)Voice_Play(VOICE_E_M_STIR, VOICE_PRIO_ERROR);   /* "교반 모터 상태를 확인해 주세요" */
			dj_fail(c, DJ_ERR_STIR_MOTOR, now_ms);
		}
	}

	/* 배기팬(FAN_EXHAUST) 10s/5s duty(C056): 처리 상태(RINSE1_CLOSE~DISCHARGE·DJ_PREP) 동안 매 틱 갱신.
	 * IDLE/DONE/ERROR/ABORTED에서는 돌지 않는다(시작은 dj_fan_exhaust_begin). */
	if (dj_exhaust_range(c->state))                     /* DJ_PREP 은 enum 끝에 있다 */
	{
		dj_fan_exhaust_tick(c, now_ms);
	}

	switch ((DongjakState)c->state)
	{
	case DJ_IDLE:
		if (c->start_req)
		{
			c->start_req = 0U;
			c->scn_start = now_ms;
			c->last_err  = (uint8_t)DJ_ERR_NONE;    /* ★G4: 새 운전에서만 지운다(벤더 latched_error = NULL) */
			c->vapor_close_done = 0U;                /* R3 C058: 운전당 1회 래치 해제 */
			c->heat_off_latch   = 0U;                /* R3 C041: 재가열 금지 래치 해제 */
			c->heater_started   = 0U;                /* R3 C005: 공정 시계는 첫 히터 ON 에서 */
			/* 음성: "염분 세척 후 전 과정을 자동으로 처리하겠습니다" (§0.14.11). */
			(void)Voice_Play(VOICE_DONGJAK_START, VOICE_PRIO_NORMAL);
			dj_fan_exhaust_begin(c, now_ms);         /* 배기팬 10/5 duty 개시(시나리오 시작) */
			{
				/* ★§0.30: 준비 탐색(R001) → 헹굼 DJ_RINSE_COUNT 회를 rinse.c 가 진행한다. */
				RinseConfig  cfg;
				RinseOutputs o;
				cfg.repeats         = (uint8_t)DJ_RINSE_COUNT;
				cfg.rot_rpm         = (uint16_t)DJ_RINSE_STIR_RPM;
				cfg.fill_timeout_ms = (uint32_t)DJ_FILL_TIMEOUT_MS;
				cfg.prep_after_fill = 0U;            /* 시작 직후·1차 급수 전 탐색 */
				Rinse_Begin(&c->rinse, &cfg, now_ms, &o);
				dj_rinse_apply(c, &o);
				c->state = DJ_PREP; c->state_since = now_ms;
			}
		}
		break;

	case DJ_PREP:
	case DJ_RINSE1_CLOSE:
	case DJ_RINSE1_FILL:
	case DJ_RINSE1_FILL_EXTRA:
	case DJ_RINSE1_STIR:
	case DJ_RINSE1_OPEN:
	case DJ_RINSE1_DRAIN:
	case DJ_RINSE2_CLOSE:
	case DJ_RINSE2_FILL:
	case DJ_RINSE2_FILL_EXTRA:
	case DJ_RINSE2_STIR:
	case DJ_RINSE2_OPEN:
	case DJ_RINSE2_DRAIN:
		dj_rinse_step(c, now_ms, 0U);              /* ★§0.30 rinse.c — 끝나면 DJ_HEAT */
		break;

	case DJ_SELFCLEAN:
		dj_rinse_step(c, now_ms, 1U);              /* ★§0.30 자가세척 — 끝나면 DJ_DONE */
		break;

	case DJ_HEAT:
		if (!c->heat_started)
		{
			if (dj_wdoor_done(WDoor_ReachedClose(), 0U, el))
			{
				WDoor_Stop(); WDoor_Disable();
				c->heat_started = 1U; c->heat_since = now_ms;
				/* ★R3 C028 [2026-09-20]: 교반·분쇄를 여기서 시작하지 **않는다**.
				 * 레퍼런스 ZG_D005~D007 은 정지, ZG_D008(허가 성립)에서 동시 개시다.
				 * 헹굼 교반이 돌고 있으므로 **명시적으로 멈춘다**(종전에는 여기서
				 * dj_stir_alt_begin() 으로 곧장 건조 교반을 켰다). */
				c->dry_gate = 0U;
				RotStir_Stop(&c->rot);               /* 헹굼 운전 잔여 해제 + 교반 정지 */
				c->grind_mode = (uint8_t)DJ_GM_OFF; c->grind_start = 0U;
				c->vapor_phase = (uint8_t)DJ_VP_CLOSED;
				/* 배기팬(fanx)은 시나리오 시작 시 이미 개시됨 — 재초기화하지 않아 10초/5초
				 * duty를 시작부터 연속 유지(THERM3 무관 독립). */
				c->fanb_on = 0U; c->fanb_active = 0U; c->fanb_since = now_ms;
			}
			break;
		}
		/* ★R3 C040: 우선순위 1순위 = **120분**. 다른 구동을 보기 전에 빠진다.
		 * (R2 는 이 판정이 맨 아래에 있어 같은 tick 에 분쇄·교반 지령이 한 번 더
		 *  나갔다. 레퍼런스 controller.c 는 step switch 보다 먼저 본다.) */
		if (sel >= (uint32_t)DJ_T_COOLDOWN_MS)       /* 120분: 식힘 (단축 시험에서도 백스톱) */
		{
			dj_heat_to_cool(c, now_ms);
			break;
		}
		/* 가열 진행: 히터/분쇄/교반/수증기/팬 동시 제어 */
		dj_heater_tick(c, now_ms);
#if DJ_TEST_BEEP
		/* [TEST] 110분대(HISPEED) 진입 시 1회 삑 (상태전이가 아니라 HEAT 내부라 래치 필요) */
		if ((sel >= (uint32_t)DJ_T_HISPEED_MS) && (c->test_beeped_hi == 0U))
		{
			c->test_beeped_hi = 1U;
			dj_test_beep(1U);
		}
#endif
		/* ★개정4 ⑤ [2026-09-22, §0.31] — 건조 교반·분쇄 = PROCESS 회전수 운전 (벤더 ZG_D005~D010).
		 *  D005~D007 : 허가(grind_allowed && outlet_closed) 전에는 교반·분쇄 **둘 다 정지**(C028, 사용자 (가)).
		 *              히터·수증기·배기팬은 승온을 위해 계속 돈다. 허가가 끝내 안 오면 120분 판정이 빼낸다.
		 *  D008      : 허가 **최초 성립** = PROCESS 시작(엔진 초기화, 초기 CW 30회의 기준점).
		 *              교반 30rpm, 분쇄 1200rpm **같은 방향·같은 가동 구간**(2회전 + 정지 2 s).
		 *  D009      : 초기 30회 뒤 CCW 2회 / CW 10회 반복.
		 *  D010      : heater_since+110분 && 초기 완료 → 두 모터 정지 확인 + 2 s 후 **분쇄만 교반 반대 방향 2000rpm**.
		 * 허가를 잃으면 엔진이 두 모터를 세우고 기다린다 — **카운트는 유지**, 복귀하면 이어서 돈다(벤더 permitted).
		 * (종전 §0.19: 허가 상실 → 초기 120초부터 재시작. 벤더 0.3.0 은 재시작하지 않는다.)
		 * 분쇄 ENABLE 이 회로에서 끊겨도 엔진 반환이 0 이 되어 명령이 먼저 내려간다 — 소프트락(§0.29) 방지. */
		{
			uint8_t ok = dj_dry_permitted(c);
#if DJ_TEST_SHORT
			/* ★[TEST] §0.47 업체 HEAT: 50℃·70℃ 접점을 **둘 다 실제로** 본 뒤에만 개시(각각 래치).
			 * PF1(50℃) = 50℃ 초과 HIGH / PF0(70℃) = 70℃ 초과 LOW — 극성이 서로 반대다(gpio_ctrl.h). */
			if (gpio_ctrl_exti_read(GPIO_EXTI_BIMETAL_50) != 0U) { c->t_low_seen  = 1U; }
			if (gpio_ctrl_exti_read(GPIO_EXTI_BIMETAL_70) == 0U) { c->t_high_seen = 1U; }
#endif
			if (!c->dry_gate)
			{
				if (!ok
#if DJ_TEST_SHORT
				    || !c->t_low_seen || !c->t_high_seen
#endif
				   )
				{
					dj_vapor_tick(c, now_ms);
					dj_fan_bldc_tick(c, now_ms);
					break;                              /* 분쇄·교반 지령 없이 물러난다 */
				}
				c->dry_gate    = 1U;                    /* ★허가 최초 성립: PROCESS 개시 */
				c->grind_start = now_ms;
				RotStir_StartProcess(&c->rot, (uint16_t)DJ_PROCESS_STIR_RPM,
				                     (uint16_t)DJ_GRIND_PROCESS_RPM, (uint16_t)DJ_GRIND_FINAL_RPM);
			}
			c->rot_st = RotStir_TickEx(&c->rot, (ok != 0U),
#if DJ_TEST_SHORT
			                           false,               /* [TEST] §0.47 후반 분쇄 생략(업체 PAIR) */
#else
			                           (sel >= (uint32_t)DJ_T_HISPEED_MS),
#endif
			                           now_ms);
			if (c->rot_st == ROT_ST_FAILED)
			{
				/* E09 — 위치 무진행·정지 미확인(교반 또는 분쇄) > P55, 역방향 이동. 두 모터는 이미 정지. */
				(void)Voice_Play(VOICE_E_M_STIR, VOICE_PRIO_ERROR);
				dj_fail(c, DJ_ERR_ROTATION, now_ms);
				break;
			}
			dj_process_mirror(c);
#if DJ_TEST_SHORT
			/* ★[TEST] §0.47 업체 PAIR: 두 모터가 **실제로 함께 돈** 순간(FG 측정 rpm > 0 둘 다)부터 30 s. */
			if (!c->pair_started && (g_stir_ctrl.meas_out_rpm > 0U) && (g_grind_ctrl.meas_out_rpm > 0U))
			{
				c->pair_started = 1U;
				c->pair_since   = now_ms;
			}
			if (c->pair_started && ((now_ms - c->pair_since) >= (uint32_t)DJ_TEST_PAIR_MS))
			{
				dj_heat_to_cool(c, now_ms);
				break;
			}
#endif
		}
		dj_vapor_tick(c, now_ms);          /* FAN_VAPOR+FAN_EXHAUST(THERM3) + 스텝 */
		dj_fan_bldc_tick(c, now_ms);       /* 분쇄 동작 중 식힘팬 30/10           */
		break;

	case DJ_COOLDOWN:
		/* 히터 OFF(진입 시). 신스펙 식힘:
		 *  - 80℃ 이상(아직 뜨거움): 교반 = 지속 CW 20RPM, 분쇄 = 1000 CCW 연속.
		 *  - 80℃ 미만(식음)      : 분쇄 OFF, 교반 = **20RPM 정3/정지1/역3** 반복
		 *    (R3 C043. R2는 30RPM CW6/정지1/CCW6 였다 — dj_stir_cool_tick 주석 참조).
		 *    ★2026-09-20 2단계: 역→정 전환 정지가 더해져 4구간 8초 cycle 이다(I06). */
		if (c->cool_phase == 0U)
		{
#if DJ_TEST_SHORT
			dj_grind_apply_mode(c, DJ_GM_OFF, now_ms);     /* [TEST] §0.47 업체 COOL: 분쇄 OFF */
#elif DJ_COOL_GRIND_REQUIRE_PERMIT
			/* ★C033 H2 (§0.36): 명령 AND 허가(벤더 465). 허가 상실 = 분쇄만 정지(에러 아님), 복귀하면 재기동. */
			dj_grind_apply_mode(c, dj_dry_permitted(c) ? DJ_GM_COOL : DJ_GM_OFF, now_ms);
#else
			dj_grind_apply_mode(c, DJ_GM_COOL, now_ms);    /* 1000 CCW 연속       */
#endif
			/* 교반은 진입 시 지속 CW로 이미 기동됨 — 유지(재기동 안 함).           */
			if (dj_cool_reached(c))                        /* 센서<80℃ ‖ 바이메탈 하강엣지 */
			{
				dj_grind_apply_mode(c, DJ_GM_OFF, now_ms);           /* 분쇄 OFF     */
				dj_stir_alt_begin(c, now_ms, (uint16_t)DJ_COOL_STIR_RPM); /* R3 C043: 20RPM 4구간 개시 */
				c->cool_phase = 1U;
			}
		}
		else                                                /* 식음: 분쇄 OFF + 교반 3/1/3 */
		{
			dj_grind_apply_mode(c, DJ_GM_OFF, now_ms);
			dj_stir_cool_tick(c, now_ms);                   /* R3 C043: 20RPM 정3/정지1/역3 */
		}
		dj_fan_bldc_tick(c, now_ms);                       /* 분쇄 동작 중 식힘팬(80℃↑ COOL 동안) */
		/* ★§0.37 D4 (C045·C046): 130분은 최소 시점 — 식힘 완료(cool_phase 1)까지 기다린다. */
#if DJ_TEST_SHORT
		/* [TEST] §0.47·§0.48 cooling_safe = 우리 기준(THERM1·2 < 80℃) **또는** 식힘 진입 후 30 s. 130분 무시.
		 * 30 s 로 끝나면 80℃ 이상인 채로 배출한다(벤치 전용 — dev_test.h 주석). */
		if (c->cool_phase != 0U)                               { c->cool_exit_by = 1U; }
		else if ((now_ms - c->state_since) >= (uint32_t)DJ_TEST_COOL_MAX_MS) { c->cool_exit_by = 2U; }
		if (c->cool_exit_by != 0U)
#else
		if ((sel >= (uint32_t)DJ_T_DISCHARGE_MS)            /* 130분: 배출 전 확인 */
#if DJ_DISCH_REQUIRE_COOLED
		    && (c->cool_phase != 0U)
#endif
		   )
#endif
		{
			dj_enter(c, DJ_BIN_CHECK, now_ms);
		}
		break;

	case DJ_BIN_CHECK:
#if DJ_BIN_CHECK_ENABLE
		/* ★2026-09-22: 수거통 유무(HS7) + 처리횟수 + 분말높이로 판정.
		 * 수거통이 없으면 배출문을 열지 않고 대기하며 03-04 를 **없어질 때마다 1회** 안내한다
		 * (넣었다 다시 빼면 또 안내). 넣으면 곧바로 진행한다.
		 * 6회↑ 이거나 가득 차면 비움 안내(음성 미구현 TODO) 후 대기 -> 비우면 진행. */
		if (!c->bin_present)
		{
			if (!c->bin_warned)
			{
				(void)Voice_Play(VOICE_BIN_MISSING, VOICE_PRIO_ERROR);
				c->bin_warned = 1U;
			}
		}
		else
		{
			c->bin_warned = 0U;
			if ((c->cycle_count < (uint16_t)DJ_MAX_CYCLES) &&
			    (c->bin_fill_pct < (uint8_t)DJ_BIN_FULL_PCT))
			{
				dj_enter(c, DJ_DISCHARGE, now_ms);
			}
		}
#else
		dj_enter(c, DJ_DISCHARGE, now_ms);
#endif
		break;

	case DJ_DISCHARGE:
		dj_discharge_tick(c, now_ms);
		break;

	case DJ_ABORTED:
		/* 4.5 안전 식힘: 고온(≥80℃)이면 BLDC 식힘팬으로 냉각, 식으면 OFF 후 IDLE
		 * 복귀. (FAN_EXHAUST는 수증기 덕트 폐쇄 상태라 비상냉각에 미사용) */
		if (c->temp_d10 >= (int16_t)DJ_TEMP_BLDCFAN_ON_D10)
		{
			gpio_ctrl_on(GPIO_OUT_BLDC_FAN);
		}
		else
		{
			gpio_ctrl_off(GPIO_OUT_BLDC_FAN);
			dj_enter(c, DJ_IDLE, now_ms);
		}
		break;

	case DJ_DONE:
		/* ★2026-09-22: DJ_DONE_HOLD_MS(3 s) 동안 완료(17)를 유지해 앱이 볼 수 있게 한 뒤 IDLE.
		 * 그 사이 시작 요청이 오면 바로 IDLE — start_req 는 남아 다음 tick 의 DJ_IDLE 이 소비한다.
		 * rinse 관측값(cnt 40·FINISHED)은 다음 시작까지 남는다. */
		if (c->start_req || ((uint32_t)(now_ms - c->state_since) >= (uint32_t)DJ_DONE_HOLD_MS))
		{
			dj_enter(c, DJ_IDLE, now_ms);
		}
		break;

	case DJ_ERROR:
		/* 에러 복구: err_clear_req(=Dongjak_ClearError, 앱 CLEAR_ERR) 세팅 시 IDLE(대기)로 복귀.
		 * 액추에이터는 진입 시 dj_all_off로 이미 정지. 이후 새 시작(HS2/강제)으로
		 * 처음부터 재개한다. err_code는 복구 직전까지, last_err 는 다음 시작까지 남는다.
		 * ★G3: 해제형(E01)은 마개 이탈·정지 요청으로도 풀린다(Dongjak_Abort / abort_req). */
		if (c->err_clear_req)
		{
			c->err_clear_req = 0U;
			dj_error_release(c, now_ms);          /* ★G1: 뜨거우면 DJ_ABORTED 경유 */
		}
		else
		{
			dj_error_fans(c, now_ms);             /* ★G1·G2 */
		}
		break;

	default:
		break;
	}
}

DongjakState Dongjak_GetState(void) { return (DongjakState)g_dongjak.state; }

uint8_t Dongjak_IsBusy(void)
{
	DongjakState s = (DongjakState)g_dongjak.state;
	return (uint8_t)((s != DJ_IDLE) && (s != DJ_DONE) && (s != DJ_ERROR));
}
