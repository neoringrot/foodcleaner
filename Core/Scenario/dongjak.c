/* ==========================================================================
 * dongjak.c - "동작"(2단계: 건조 -> 분쇄 -> 배출) 시나리오 상태머신 구현.
 * ★기준 원본: doc/R1/zerogeo_scenario.docx ([2단계] 동작, 2026-07-30). 최신·우선.
 * 설계/시퀀스/상수 개요는 dongjak.h 참조. 미검증 값은 DJ_* 매크로로 분리.
 *
 * 하드웨어 소유권:
 *   - 히터     : GPIO_OUT_HT_POWER (PA12). 히스테리시스 114 ON/117 OFF (210=HW 바이메탈).
 *   - 교반     : g_stir_ctrl  (M2/U16). 건조=CW3/정지2/CCW3 반복(기획서 4.2), 식힘/배출=CW3/1/CCW3.
 *   - 분쇄     : g_grind_ctrl (M1/U11). 80℃↑ 1500CW3/2 3분->1000CW연속, 110분↑ 2000CCW4/2.
 *   - 배수문   : WDoor (U5). 헹굼/잔수배수/최종 완전개방.
 *   - 배출문   : TDoor (U7). 배출(개방 후 2분간 DC 미구동, 이후 닫음).
 *   - 수증기   : 냄새관로 스텝(STEP1)+흡입 스텝(STEP2)+FAN_VAPOR. 100℃ ON/84℃ OFF.
 *   - 팬       : FAN_VAPOR(수증기 방수팬, THERM3 100/84℃) / FAN_EXHAUST(배기팬, 시나리오
 *               전체 15분ON·2분OFF 독립 duty) / BLDC_FAN(분쇄동작중 30/10).
 *   - 온도     : 처리통 g_therm_c_d10[DJ_TEMP_CH]=CH0(히터/분쇄/식힘) / 수증기는
 *               g_therm_c_d10[DJ_VAPOR_TEMP_CH]=CH2(THERM3,J23). 수거통: g_bin_fill_pct.
 *
 * 안전 락(HW, MCU 아님)은 dongjak.h 참조 - FW는 감시/구동만.
 * ========================================================================== */

#include "dongjak.h"

#include "gpio_ctrl.h"
#include "heater_hyst.h"
#include "wdoor.h"
#include "tdoor.h"
#include "bldc_ctrl.h"
#include "hallsensor.h"
#include "thermistor.h"
#include "step_motor.h"
#if DJ_TEST_FAST_TIMING
#include "lm4871.h"                 /* [TEST] 단계 비프용 스피커(전역 lm4871) */
#endif

/* defaultTask가 갱신하는 센서 스냅샷(정의: freertos.c). 같은 태스크에서 읽는다. */
extern volatile int16_t g_therm_c_d10[];
extern volatile uint8_t g_bin_fill_pct;

#define DJ_DOOR_DUTY_PCT    50U    /* 도어 PWM 힘(모음과 동일)               */

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
	dj_enter(c, DJ_ERROR, now);
}

#if DJ_TEST_FAST_TIMING
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
	WDoor_Enable(); WDoor_Open((uint8_t)DJ_DOOR_DUTY_PCT);
	dj_drain_valve(1U);
}

static void dj_wdoor_close(void)
{
	WDoor_Enable(); WDoor_Close((uint8_t)DJ_DOOR_DUTY_PCT);
	dj_drain_valve(0U);
}

/* 도어 리미트 게이트 판정. 정상 모드=리미트 도달만. DJ_DOOR_LIMIT_OPTIONAL=1
 * (벤치 육안)=리미트 미도달이라도 DJ_DOOR_BENCH_MS 경과 시 통과(센서 무시).
 * 모음(MOEUM_DOOR_LIMIT_OPTIONAL)과 동일 기조. el = 해당 도어 구동 경과(ms). */
static uint8_t dj_door_done(uint8_t at_limit, uint32_t el)
{
#if DJ_DOOR_LIMIT_OPTIONAL
	return (uint8_t)(at_limit || (el >= (uint32_t)DJ_DOOR_BENCH_MS));
#else
	(void)el;
	return at_limit;
#endif
}

/* 배출문(TDoor) 전용 리미트 게이트. THALL(PF2/PF5)이 벤치 미검증이라, OPTIONAL=1이면
 * 리미트 미도달이라도 el(구동 경과) >= DJ_DOOR_BENCH_MS 경과 시 통과시킨다. 근거:
 * 배출문 개방 자체로 HW 2분타이머가 시작되므로, 리미트 인식 실패가 교반/타이머
 * 시작을 막지 않도록 함(리미트 검증되면 DJ_TDOOR_LIMIT_OPTIONAL=0). */
static uint8_t dj_tdoor_done(uint8_t at_limit, uint32_t el)
{
#if DJ_TDOOR_LIMIT_OPTIONAL
	return (uint8_t)(at_limit || (el >= (uint32_t)DJ_DOOR_BENCH_MS));
#else
	(void)el;
	return at_limit;
#endif
}

static void dj_all_off(void)
{
	BldcCtrl_Stop(&g_grind_ctrl);
	BldcCtrl_Stop(&g_stir_ctrl);
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

static void dj_stir_spin(uint8_t reverse, uint16_t rpm)
{
	BldcCtrl_Stop(&g_stir_ctrl);
	BldcCtrl_Start(&g_stir_ctrl, reverse);
	g_stir_ctrl.target_out_rpm = rpm;
}

static void dj_grind_spin(uint8_t reverse, uint16_t rpm)
{
	BldcCtrl_Stop(&g_grind_ctrl);
	BldcCtrl_Start(&g_grind_ctrl, reverse);
	g_grind_ctrl.target_out_rpm = rpm;
}

static uint8_t dj_water_present(void)
{
	if (gpio_ctrl_exti_flag_get(GPIO_EXTI_WATER_SEN1) ||
	    gpio_ctrl_exti_flag_get(GPIO_EXTI_WATER_SEN2))
	{
		return 1U;
	}
#if DJ_WATER_USE_LEVEL
#if DJ_WATER_ACTIVE_LOW
	return (uint8_t)((gpio_ctrl_exti_read(GPIO_EXTI_WATER_SEN1) == 0U) ||
	                 (gpio_ctrl_exti_read(GPIO_EXTI_WATER_SEN2) == 0U));
#else
	return (uint8_t)((gpio_ctrl_exti_read(GPIO_EXTI_WATER_SEN1) == 1U) ||
	                 (gpio_ctrl_exti_read(GPIO_EXTI_WATER_SEN2) == 1U));
#endif
#else
	return 0U;
#endif
}

/* ---- 교반 패턴 A: 건조 (CW 3s/정지 2s ×5 → CCW 3s 반복, 신스펙 REPS=5) ---
 * RPM은 호출부가 지정(110분 전 20, 이후 27). 패턴은 동일. */
static void dj_stir_dry_begin(DongjakCtx *c, uint32_t now)
{
	c->stir_phase = (uint8_t)DJ_STIR_FWD;
	c->stir_reps  = 0U;
	c->stir_since = now;
	dj_stir_spin(0U, (uint16_t)DJ_STIR_RPM);   /* CW (시작=20RPM, sel=0)      */
}

static void dj_stir_dry_tick(DongjakCtx *c, uint32_t now, uint16_t rpm)
{
	uint32_t el = now - c->stir_since;
	switch ((DjStirPhase)c->stir_phase)
	{
	case DJ_STIR_FWD:
		if (el >= (uint32_t)DJ_STIR_FWD_MS)
		{
			BldcCtrl_Stop(&g_stir_ctrl);
			c->stir_reps++;
			c->stir_phase = (uint8_t)DJ_STIR_STOPPED;
			c->stir_since = now;
		}
		break;
	case DJ_STIR_STOPPED:
		if (el >= (uint32_t)DJ_STIR_STOP_MS)
		{
			if (c->stir_reps < (uint8_t)DJ_STIR_FWD_REPS)
			{
				dj_stir_spin(0U, rpm);                         /* CW           */
				c->stir_phase = (uint8_t)DJ_STIR_FWD;
			}
			else
			{
				dj_stir_spin(1U, rpm);                         /* CCW          */
				c->stir_phase = (uint8_t)DJ_STIR_REV;
			}
			c->stir_since = now;
		}
		break;
	case DJ_STIR_REV:
	default:
		if (el >= (uint32_t)DJ_STIR_REV_MS)
		{
			c->stir_reps = 0U;
			dj_stir_spin(0U, rpm);                             /* CW 재시작    */
			c->stir_phase = (uint8_t)DJ_STIR_FWD;
			c->stir_since = now;
		}
		break;
	}
}

/* ---- 교반 패턴 B: CW 3s/정지 1s/CCW 3s (헹굼·식힘·배출, docx 6·10.2) ---
 * stir_phase 0=CW, 1=정지, 2=CCW 재사용. rpm은 호출부가 지정. */
static void dj_stir313_begin(DongjakCtx *c, uint32_t now, uint16_t rpm)
{
	c->stir_phase = 0U;
	c->stir_since = now;
	dj_stir_spin(0U, rpm);                     /* CW                          */
}

static void dj_stir313_tick(DongjakCtx *c, uint32_t now, uint16_t rpm)
{
	uint32_t el = now - c->stir_since;
	switch (c->stir_phase)
	{
	case 0:  /* CW */
		if (el >= (uint32_t)DJ_S313_CW_MS)
		{
			BldcCtrl_Stop(&g_stir_ctrl);
			c->stir_phase = 1U; c->stir_since = now;
		}
		break;
	case 1:  /* 정지 */
		if (el >= (uint32_t)DJ_S313_STOP_MS)
		{
			dj_stir_spin(1U, rpm);             /* CCW                         */
			c->stir_phase = 2U; c->stir_since = now;
		}
		break;
	case 2:  /* CCW */
	default:
		if (el >= (uint32_t)DJ_S313_CCW_MS)
		{
			dj_stir_spin(0U, rpm);             /* CW 재시작                   */
			c->stir_phase = 0U; c->stir_since = now;
		}
		break;
	}
}

/* ---- 히터: 114↓ON / 117↑OFF 히스테리시스 (+210 SW 보조, 신스펙 항목5) ----
 * 판정은 tb_heat와 공유하는 순수함수 Heater_HystDecide()가 담당(임계 이원화 방지).
 * 정책: **센서 에러 시 안전 OFF**(과열 방지). 밴드 안(114~117)은 HOLD=핀 유지. */
static void dj_heater_tick(DongjakCtx *c)
{
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

	switch (Heater_HystDecide(&cfg, c->temp_d10, c->temp_valid))
	{
	case HEATER_CMD_ON:   gpio_ctrl_on(GPIO_OUT_HT_POWER);  break;
	case HEATER_CMD_OFF:  gpio_ctrl_off(GPIO_OUT_HT_POWER); break;
	case HEATER_CMD_HOLD:
	default:              /* 밴드 안(114~117): 핀 유지 */ break;
	}
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
	case DJ_GM_COARSE: dj_grind_spin(0U, (uint16_t)DJ_GRIND_COARSE_RPM); break; /* CW */
	case DJ_GM_FINE:   dj_grind_spin(0U, (uint16_t)DJ_GRIND_FINE_RPM);   break; /* CW 연속 */
	case DJ_GM_FINAL:  dj_grind_spin(1U, (uint16_t)DJ_GRIND_FINAL_RPM);  break; /* CCW */
	case DJ_GM_COOL:   dj_grind_spin(1U, (uint16_t)DJ_GRIND_FINE_RPM);   break; /* CCW 연속 */
	case DJ_GM_OFF:
	default:           BldcCtrl_Stop(&g_grind_ctrl);                     break;
	}
}

/* 토글 모드(COARSE/FINAL)만 run/stop 반복. 연속 모드(FINE/COOL)는 유지. */
static void dj_grind_toggle(DongjakCtx *c, uint32_t now, uint8_t rev, uint16_t rpm,
                            uint32_t run_ms, uint32_t stop_ms)
{
	uint32_t el = now - c->grind_since;
	if ((DjGrindPhase)c->grind_phase == DJ_GR_RUN)
	{
		if (el >= run_ms)
		{
			BldcCtrl_Stop(&g_grind_ctrl);
			c->grind_phase = (uint8_t)DJ_GR_STOP; c->grind_since = now;
		}
	}
	else
	{
		if (el >= stop_ms)
		{
			dj_grind_spin(rev, rpm);
			c->grind_phase = (uint8_t)DJ_GR_RUN; c->grind_since = now;
		}
	}
}

/* 건조 구간 분쇄 제어(80℃ 게이팅 + 시간 프로파일). sel = 시나리오 경과. */
static void dj_grind_heat_tick(DongjakCtx *c, uint32_t now, uint32_t sel)
{
	int16_t t = c->temp_d10;

	if (t < (int16_t)DJ_TEMP_GRIND_ON_D10)          /* 80℃ 미만: 분쇄 금지     */
	{
		dj_grind_apply_mode(c, DJ_GM_OFF, now);
		c->grind_start = 0U;                        /* 재개 시 1차부터          */
		return;
	}
	if (c->grind_start == 0U) { c->grind_start = now; } /* 80℃ 최초 도달 기록   */

	if (sel >= (uint32_t)DJ_T_HISPEED_MS)           /* 110분↑: 2000 CCW 4초/정지2초 */
	{
		/* 신스펙 "역회전 4초 / 정지 2초" 반복. 단, 최초 진입 시 급격한 CW->CCW 반전
		 * 인러시(전류 실패) 방지 위해: 현재 방향 유지한 채 먼저 감속(target=0, 슬루)
		 * -> DECEL_MS 후 역회전 개시 -> 슬루가 2000까지 상승 -> 이후 4/2초 토글.
		 * 토글의 정지->재기동도 dj_grind_spin이 슬루 0->2000 소프트스타트라 급전류 없음. */
		if (c->grind_mode != (uint8_t)DJ_GM_FINAL)  /* FINAL 최초 진입: 감속 시작 */
		{
			c->grind_mode     = (uint8_t)DJ_GM_FINAL;
			c->grind_final_ph = 0U;
			c->grind_since    = now;
			g_grind_ctrl.target_out_rpm = 0U;       /* 현재 회전 방향 유지한 채 감속 */
		}
		if (c->grind_final_ph == 0U)                /* 감속 대기 -> 역회전 개시 */
		{
			if ((now - c->grind_since) >= (uint32_t)DJ_GRIND_FINAL_DECEL_MS)
			{
				dj_grind_spin(1U, (uint16_t)DJ_GRIND_FINAL_RPM); /* CCW 개시, 슬루 0->2000 */
				c->grind_final_ph = 1U;
				c->grind_phase    = (uint8_t)DJ_GR_RUN;
				c->grind_since    = now;
			}
		}
		else                                        /* 역회전 확립: 4초 구동/2초 정지 토글 */
		{
			dj_grind_toggle(c, now, 1U, (uint16_t)DJ_GRIND_FINAL_RPM,
			                (uint32_t)DJ_GRIND_RUN_FINAL_MS, (uint32_t)DJ_GRIND_STOP_FINAL_MS);
		}
	}
	else if ((now - c->grind_start) < (uint32_t)DJ_GRIND_COARSE_MS) /* 1차 3분  */
	{
		dj_grind_apply_mode(c, DJ_GM_COARSE, now);
		dj_grind_toggle(c, now, 0U, (uint16_t)DJ_GRIND_COARSE_RPM,
		                (uint32_t)DJ_GRIND_RUN_MS, (uint32_t)DJ_GRIND_STOP_MS);
	}
	else                                            /* 이후: 1000 CW 연속       */
	{
		dj_grind_apply_mode(c, DJ_GM_FINE, now);
	}
}

/* ---- 수증기: 100℃ 개방(방수팬->관로->흡입), 84℃ 역순 폐쇄 (docx 7.1) --
 * 온도 소스는 수증기 전용 THERM3(J23, PC2) = c->vapor_temp_d10 (히터/분쇄와 별개).
 * FAN_VAPOR(방수팬)만 100℃에서 먼저 ON한 뒤 STEP1(관로)/STEP2(흡입)을 개방, 84℃에서
 * 스텝 역순 폐쇄 후 FAN_VAPOR OFF. (FAN_EXHAUST 배기팬은 THERM3 분리, fanx 15/2 독립) */
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

static void dj_vapor_tick(DongjakCtx *c, uint32_t now)
{
	int16_t  t  = c->vapor_temp_d10;
	/* STEP2(흡입) 위상(OPEN_AIR·CLOSE_AIR)은 느린 페이싱, STEP1(관로)은 기본. */
	uint8_t  air = (uint8_t)((c->vapor_phase == (uint8_t)DJ_VP_OPEN_AIR) ||
	                         (c->vapor_phase == (uint8_t)DJ_VP_CLOSE_AIR));
	uint8_t  due = (uint8_t)((now - c->vapor_step_since) >= dj_step_interval(c->vapor_step_cnt, air));

	switch ((DjVaporPhase)c->vapor_phase)
	{
	case DJ_VP_CLOSED:
		if (t >= (int16_t)DJ_TEMP_VAPOR_ON_D10)
		{
			gpio_ctrl_on(GPIO_OUT_FAN_VAPOR);        /* 1) 방수팬 ON (팬 먼저)  */
			/* FAN_EXHAUST는 THERM3 루프에서 분리됨(dj_fan_exhaust_tick가 15/2 duty로 독립 제어) */
			c->vapor_step_cnt = 0U; c->vapor_step_since = now;
			c->vapor_phase = (uint8_t)DJ_VP_OPEN_DUCT;
		}
		break;
	case DJ_VP_OPEN_DUCT:                             /* 2) 냄새 관로 OPEN       */
		if (due)
		{
			StepMotor_Step(&dj_duct, 1); c->vapor_step_cnt++; c->vapor_step_since = now;
			if (c->vapor_step_cnt >= (uint16_t)DJ_DUCT_STEPS)
			{
#if DJ_STEP_RELEASE_DUCT_ON_OPEN
				StepMotor_Release(&dj_duct);  /* STEP1 코일 해제 → STEP2에 레일 전류 양보 */
#endif
				c->vapor_step_cnt = 0U; c->vapor_step_since = now; c->vapor_phase = (uint8_t)DJ_VP_OPEN_AIR;
			}
		}
		break;
	case DJ_VP_OPEN_AIR:                              /* 3) 흡입 제어 OPEN       */
		if (due)
		{
			StepMotor_Step(&dj_air, 1); c->vapor_step_cnt++; c->vapor_step_since = now;
			if (c->vapor_step_cnt >= (uint16_t)DJ_DUCT_STEPS)
			{ c->vapor_phase = (uint8_t)DJ_VP_OPEN; }
		}
		break;
	case DJ_VP_OPEN:
		if (t <= (int16_t)DJ_TEMP_VAPOR_OFF_D10)      /* 84℃: 역순 폐쇄 시작     */
		{
			c->vapor_step_cnt = 0U; c->vapor_step_since = now;
			c->vapor_phase = (uint8_t)DJ_VP_CLOSE_AIR;
		}
		break;
	case DJ_VP_CLOSE_AIR:                             /* 1') 흡입 CLOSE          */
		if (due)
		{
			StepMotor_Step(&dj_air, -1); c->vapor_step_cnt++; c->vapor_step_since = now;
			if (c->vapor_step_cnt >= (uint16_t)DJ_DUCT_STEPS)
			{ StepMotor_Release(&dj_air); c->vapor_step_cnt = 0U; c->vapor_step_since = now; c->vapor_phase = (uint8_t)DJ_VP_CLOSE_DUCT; }
		}
		break;
	case DJ_VP_CLOSE_DUCT:                            /* 2') 관로 CLOSE          */
	default:
		if (due)
		{
			StepMotor_Step(&dj_duct, -1); c->vapor_step_cnt++; c->vapor_step_since = now;
			if (c->vapor_step_cnt >= (uint16_t)DJ_DUCT_STEPS)
			{
				StepMotor_Release(&dj_duct);
				gpio_ctrl_off(GPIO_OUT_FAN_VAPOR);   /* 3') 방수팬 OFF (배기팬은 fanx 독립) */
				c->vapor_phase = (uint8_t)DJ_VP_CLOSED;
			}
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

/* ---- BLDC 식힘팬: 분쇄모터 동작 중 30/10초 (신스펙 요구1) --------------
 * 분쇄는 온도게이팅으로 켜지므로(HEAT: 센서65℃↑, COOLDOWN: 80℃↑), 팬을 분쇄
 * 동작상태(grind_mode != OFF)에 연동한다. 분쇄가 서면 팬도 정지. */
static void dj_fan_bldc_tick(DongjakCtx *c, uint32_t now)
{
	if (c->grind_mode == (uint8_t)DJ_GM_OFF)            /* 분쇄 정지: 식힘팬 OFF */
	{
		if (c->fanb_on) { gpio_ctrl_off(GPIO_OUT_BLDC_FAN); c->fanb_on = 0U; c->fanb_since = now; }
		return;
	}
	uint32_t el = now - c->fanb_since;
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

/* ---- 배기팬(FAN_EXHAUST): 시나리오 전체 15분 ON / 2분 OFF 독립 duty ------
 * THERM3 수증기 루프에서 분리(2026-08-17). 시나리오 시작 시 begin으로 ON 개시,
 * 처리 상태(RINSE1_CLOSE~DISCHARGE) 동안 tick이 15/2로 토글. 종료(DONE/ERROR/
 * ABORTED)에서 dj_all_off가 OFF. (구 10초/5초 내부정화 duty → 15분/2분으로 변경) */
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

/* 식힘 "80℃ 미만 도달"(분쇄 OFF·교반313 전환) 판정:
 *   (THERM1 && THERM2 둘 다 유효 && <80℃)  ||  HW 80℃ 바이메탈(PF0) 하강엣지.
 * 보수적(A안): 어느 써미스터든 에러(미상)면 '아직 안 식음'으로 보아 정지하지 않되,
 * 바이메탈 하강엣지가 오면 그걸로 정지(안전 폴백). 엣지는 COOLDOWN 진입 시 stale
 * 클리어된 이후의 것만 유효. DJ_COOL_USE_THERM2=0이면 THERM1 단독(구동작). */
static uint8_t dj_cool_reached(DongjakCtx *c)
{
#if DJ_COOL_USE_BIMETAL80
	if (gpio_ctrl_exti_flag_get(GPIO_EXTI_BIMETAL_80)) { return 1U; }  /* 바이메탈: 안전 폴백 */
#endif
	/* THERM1 유효 && <80℃ */
	if (!(c->temp_valid && (c->temp_d10 < (int16_t)DJ_TEMP_COOL_GRIND_OFF_D10))) { return 0U; }
#if DJ_COOL_USE_THERM2
	/* THERM2도 유효 && <80℃ 라야 '식음' 확정(둘 중 하나라도 미달/에러면 계속 냉각) */
	if (!(c->temp2_valid && (c->temp2_d10 < (int16_t)DJ_TEMP_COOL_GRIND_OFF_D10))) { return 0U; }
#endif
	return 1U;
}

/* ---- 초기 헹굼 (docx 2단계 2번: 1단계 2~6번 x2) -----------------------
 * 세부단계를 DongjakState(DJ_RINSE1_x · DJ_RINSE2_x)로 노출(moeum MoeumState 방식).
 * 2회는 고정 상수라 통째로 언롤 - 카운터 변수 없음. 각 단계 진입동작은 dj_enter()에서
 * 수행하고 타이밍은 state_since 기준. 1·2차 차이는 배수 교반시간뿐이라 스텝 로직을
 * 헬퍼로 공유하고 '다음 상태'와 '배수 지속시간'만 인자로 넘긴다.
 * docx는 "1단계 2~6번(배수 제외)"이라 명시하나, 배수 없이 2회 급수 시 2회차
 * 수위가 이미 도달 상태 -> 에지 감지 불가(교착) + 물리적 오버플로 위험. 따라서
 * 헹굼 사이에 개방/배수를 넣어 매 회 신선한 급수·감지가 되게 한다(설계 판단). */
static void dj_rs_close(DongjakCtx *c, uint32_t now, DongjakState next)
{
	uint32_t el = now - c->state_since;
	if (dj_door_done(WDoor_AtClose(), el))
	{ WDoor_Stop(); WDoor_Disable(); dj_enter(c, next, now); }
	else if (el >= (uint32_t)DJ_DOOR_TIMEOUT_MS) { dj_fail(c, DJ_ERR_RINSE_CLOSE, now); }
}

static void dj_rs_fill(DongjakCtx *c, uint32_t now, DongjakState next)
{
	if (c->water_reached) { dj_enter(c, next, now); }
	else if ((now - c->state_since) >= (uint32_t)DJ_FILL_TIMEOUT_MS)
	{ dj_fill_off(); dj_fail(c, DJ_ERR_RINSE_FILL, now); }
}

static void dj_rs_fill_extra(DongjakCtx *c, uint32_t now, DongjakState next)
{
	if ((now - c->state_since) >= (uint32_t)DJ_FILL_EXTRA_MS)
	{ dj_fill_off(); dj_enter(c, next, now); }
}

static void dj_rs_stir(DongjakCtx *c, uint32_t now, DongjakState next)
{
	dj_stir313_tick(c, now, (uint16_t)DJ_RINSE_STIR_RPM);
	if ((now - c->state_since) >= (uint32_t)DJ_RINSE_STIR_MS)
	{ BldcCtrl_Stop(&g_stir_ctrl); dj_enter(c, next, now); }
}

static void dj_rs_open(DongjakCtx *c, uint32_t now, DongjakState next)
{
	uint32_t el = now - c->state_since;
	if (dj_door_done(WDoor_AtOpen(), el))
	{ WDoor_Stop(); WDoor_Disable(); dj_enter(c, next, now); }
	else if (el >= (uint32_t)DJ_DOOR_TIMEOUT_MS) { dj_fail(c, DJ_ERR_RINSE_OPEN, now); }
}

/* 배수: 문 열린 채 교반하며 물빼기(잔수 배수). 지속시간만 1·2차 다름. */
static void dj_rs_drain(DongjakCtx *c, uint32_t now, uint32_t dur, DongjakState next)
{
	dj_stir313_tick(c, now, (uint16_t)DJ_RINSE_STIR_RPM);
	if ((now - c->state_since) >= dur)
	{ BldcCtrl_Stop(&g_stir_ctrl); dj_enter(c, next, now); }
}

static void dj_rinse_tick(DongjakCtx *c, uint32_t now)
{
	switch ((DongjakState)c->state)
	{
	/* 1차 */
	case DJ_RINSE1_CLOSE:      dj_rs_close(c, now, DJ_RINSE1_FILL);                     break;
	case DJ_RINSE1_FILL:       dj_rs_fill(c, now, DJ_RINSE1_FILL_EXTRA);                break;
	case DJ_RINSE1_FILL_EXTRA: dj_rs_fill_extra(c, now, DJ_RINSE1_STIR);               break;
	case DJ_RINSE1_STIR:       dj_rs_stir(c, now, DJ_RINSE1_OPEN);                      break;
	case DJ_RINSE1_OPEN:       dj_rs_open(c, now, DJ_RINSE1_DRAIN);                     break;
	case DJ_RINSE1_DRAIN:      dj_rs_drain(c, now, (uint32_t)DJ_RINSE1_DRAIN_MS, DJ_RINSE2_CLOSE); break;
	/* 2차 */
	case DJ_RINSE2_CLOSE:      dj_rs_close(c, now, DJ_RINSE2_FILL);                     break;
	case DJ_RINSE2_FILL:       dj_rs_fill(c, now, DJ_RINSE2_FILL_EXTRA);                break;
	case DJ_RINSE2_FILL_EXTRA: dj_rs_fill_extra(c, now, DJ_RINSE2_STIR);               break;
	case DJ_RINSE2_STIR:       dj_rs_stir(c, now, DJ_RINSE2_OPEN);                      break;
	case DJ_RINSE2_OPEN:       dj_rs_open(c, now, DJ_RINSE2_DRAIN);                     break;
	case DJ_RINSE2_DRAIN:      dj_rs_drain(c, now, (uint32_t)DJ_RINSE2_DRAIN_MS, DJ_HEAT); break; /* 잔수 흡수 -> HEAT */
	default: break;
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

/* 배출문 닫기 개시 시점 도달? = 시나리오 절대 135분(DJ_T_LATCH_MS) AND 배출 진입
 * 후 최소 배출 창(DJ_DISCH_WINDOW_MS) 경과. 후자는 수거통 확인 대기 등으로 배출
 * 진입이 135분 이후로 밀렸을 때 "열자마자 닫힘"을 막는 가드. */
static uint8_t dj_disch_close_due(const DongjakCtx *c, uint32_t now)
{
	return (uint8_t)(((now - c->scn_start)   >= (uint32_t)DJ_T_LATCH_MS) &&
	                 ((now - c->state_since) >= (uint32_t)DJ_DISCH_WINDOW_MS));
}

/* 닫기 개시: 교반 정지 → 배출문 역방향(CCW) 구동. 방향 반전 전 코스트로 한 번
 * 끊어 DRV8871 급반전 인러시를 피한다. 이후 CLOSE_T가 리미트까지 계속 돌린다. */
static void dj_disch_begin_close(DongjakCtx *c, uint32_t now)
{
	BldcCtrl_Stop(&g_stir_ctrl);
	TDoor_Stop();
	c->disc_pulse_off = 0U;                        /* 간헐 구동 위상 리셋      */
	TDoor_Enable(); TDoor_Close((uint8_t)DJ_DOOR_DUTY_PCT);
	dj_disc_enter(c, DJ_DS_CLOSE_T, now);
}

/* 리미트 미인식으로 길어질 때의 간헐(펄스) 구동 유지 — DJ_DISCH_DOOR_PULSE.
 * DJ_DISCH_DOOR_PULSE_AFTER_MS까지는 손대지 않으므로(연속 구동) 정상 개폐
 * 시간대의 동작은 기존과 동일하고, 그 이후에만 ON/OFF를 반복해 스톨 열을 줄인다.
 * 휴지 구간엔 코스트 + VM(EN) OFF. 상태가 바뀌는 순간에만 드라이버를 건드린다.
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
		if (opening) { TDoor_Open ((uint8_t)DJ_DOOR_DUTY_PCT); }
		else         { TDoor_Close((uint8_t)DJ_DOOR_DUTY_PCT); }
	}
#else
	(void)c; (void)opening; (void)el;
#endif
}

static void dj_discharge_tick(DongjakCtx *c, uint32_t now)
{
	uint32_t el  = now - c->disc_since;
	switch ((DjDischPhase)c->disc_phase)
	{
	case DJ_DS_OPEN_T:                                /* 배출문 개방(CW)        */
		if (dj_tdoor_done(TDoor_AtOpen(), el))
		{
			c->disc_open_ms = el;                    /* 리미트 인식까지 실측 시간 기록 */
			TDoor_Stop(); TDoor_Disable();           /* 개방 후 DC 미구동(HW 2분타이머 구간) */
			/* 배출문 열림 = HW 2분 타이머 시작. 그 종료는 TIMER-OUT(PF8) 하강엣지로
			 * 인식(신스펙 항목8). 진입 직전 stale 엣지 플래그 제거. */
			gpio_ctrl_exti_flag_clear(GPIO_EXTI_TIMER_OUT);
			dj_stir313_begin(c, now, (uint16_t)DJ_STIR_RPM);
			dj_disc_enter(c, DJ_DS_EXPEL, now);
		}
		else if (dj_disch_close_due(c, now))
		{
			/* 135분 백스톱: 개방 리미트 미인식이어도 여기서 개방 구동을 접고
			 * 닫기(CCW)로 전환한다. disc_open_ms는 0(미도달)으로 남는다. */
			dj_disch_begin_close(c, now);
		}
#if !DJ_DISCH_DOOR_NO_TIMEOUT
		else if (el >= (uint32_t)DJ_DOOR_TIMEOUT_MS) { dj_fail(c, DJ_ERR_DISCH_OPEN, now); }
#endif
		else { dj_tdoor_keep_driving(c, 1U, el); }   /* 개방(CW) 유지 - 길어지면 간헐 */
		break;

	case DJ_DS_EXPEL:                                 /* 교반 배출(HW 2분타이머) */
		dj_stir313_tick(c, now, (uint16_t)DJ_STIR_RPM);
		/* 2분 완료 = TIMER-OUT 하강엣지(항목8), 미도달 대비 SW 2분 백업. */
		if (gpio_ctrl_exti_flag_get(GPIO_EXTI_TIMER_OUT)
#if DJ_DISCH_STIR_SW_BACKUP
		    || (el >= (uint32_t)DJ_DISCHARGE_STIR_MS)
#endif
		   )
		{
			gpio_ctrl_exti_flag_clear(GPIO_EXTI_TIMER_OUT);
			BldcCtrl_Stop(&g_stir_ctrl);             /* 2분 완료 -> 교반 정지    */
#if DJ_DISCH_CLOSE_AT_LATCH
			dj_disc_enter(c, DJ_DS_HOLD, now);       /* 문은 열어둔 채 135분 대기 */
#else
			dj_disch_begin_close(c, now);            /* (구 동작) 즉시 닫기      */
#endif
		}
		else if (dj_disch_close_due(c, now))         /* 135분: 2분 판정 실패 백스톱 */
		{
			gpio_ctrl_exti_flag_clear(GPIO_EXTI_TIMER_OUT);
			dj_disch_begin_close(c, now);
		}
		break;

	case DJ_DS_HOLD:                                  /* 교반 정지, 개방 유지    */
		/* 배출문은 열린 채(코스트) 135분까지 대기. HW 2분 래치는 이미 해제됨. */
		if (dj_disch_close_due(c, now)) { dj_disch_begin_close(c, now); }
		break;

	case DJ_DS_CLOSE_T:                               /* 배출문 닫음(CCW, 래치 해제) */
		if (dj_tdoor_done(TDoor_AtClose(), el))
		{
			c->disc_close_ms = el;                   /* 리미트 인식까지 실측 시간 기록 */
			TDoor_Stop(); TDoor_Disable();
			dj_wdoor_open();                             /* 배수부 완전개방(배수밸브 동반) */
			dj_disc_enter(c, DJ_DS_OPEN_W, now);
		}
#if !DJ_DISCH_DOOR_NO_TIMEOUT
		else if (el >= (uint32_t)DJ_DOOR_TIMEOUT_MS) { dj_fail(c, DJ_ERR_DISCH_CLOSE, now); }
#endif
		/* NO_TIMEOUT=1: 닫힐 때까지 계속 CCW 구동(시간 상한 없음). 15초를 넘기면
		 * 간헐 구동으로 전환해 스톨 열을 줄인다. 완전 탈출은 정지요청으로만. */
		else { dj_tdoor_keep_driving(c, 0U, el); }
		break;

	case DJ_DS_OPEN_W:
	default:
		if (dj_door_done(WDoor_AtOpen(), el))
		{
			WDoor_Stop(); WDoor_Disable();
			c->cycle_count++;
			dj_enter(c, DJ_DONE, now);
		}
		else if (el >= (uint32_t)DJ_DOOR_TIMEOUT_MS) { dj_fail(c, DJ_ERR_DISCH_WOPEN, now); }
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
	/* 헹굼 진입동작 (fanx 초기화는 시나리오 시작 시 1회만 - DJ_IDLE 시작부에서 수행).
	 * 1·2차 동일 동작은 case를 묶는다. DRAIN은 문 열린 채 교반하며 물빼기(잔수 배수). */
	case DJ_RINSE1_CLOSE:
	case DJ_RINSE2_CLOSE:
		dj_wdoor_close();
		break;

	case DJ_RINSE1_FILL:
	case DJ_RINSE2_FILL:
		c->water_reached = 0U;
		gpio_ctrl_exti_flag_clear(GPIO_EXTI_WATER_SEN1);
		gpio_ctrl_exti_flag_clear(GPIO_EXTI_WATER_SEN2);
		dj_fill_on();
		break;

	case DJ_RINSE1_STIR:
	case DJ_RINSE2_STIR:
		dj_stir313_begin(c, now, (uint16_t)DJ_RINSE_STIR_RPM);
		break;

	case DJ_RINSE1_OPEN:
	case DJ_RINSE2_OPEN:
		dj_wdoor_open();
		break;

	case DJ_RINSE1_DRAIN:
	case DJ_RINSE2_DRAIN:
		/* 문은 OPEN 단계에서 이미 열림(모터 정지 완료). 교반 시작해 물빼기. */
		dj_stir313_begin(c, now, (uint16_t)DJ_RINSE_STIR_RPM);
		break;

	case DJ_RINSE1_FILL_EXTRA:
	case DJ_RINSE2_FILL_EXTRA:
		break;

	case DJ_HEAT:
		c->heat_started = 0U;                        /* 먼저 배수문 닫힘 확인   */
#if DJ_TEST_FAST_TIMING
		c->test_beeped_hi = 0U;                      /* [TEST] HISPEED 비프 재무장 */
#endif
		dj_wdoor_close();
		break;

	case DJ_COOLDOWN:
		gpio_ctrl_off(GPIO_OUT_HT_POWER);            /* 히터 OFF               */
		/* 식힘 진입 = 아직 뜨거움: 교반은 "지속 CW"(1회만 기동, 토글 아님),
		 * 분쇄는 tick에서 1000 CCW. 80℃ 미만으로 식으면 cool_phase=1 전환. */
		c->cool_phase = 0U;
		gpio_ctrl_exti_flag_clear(GPIO_EXTI_BIMETAL_80); /* 80℃ 바이메탈 stale 엣지 제거 */
		dj_stir_spin(0U, (uint16_t)DJ_STIR_RPM);     /* 교반 지속 CW 개시        */
#if DJ_TEST_FAST_TIMING
		dj_test_beep(2U);                            /* [TEST] 8분대(120분) 삑삑 */
#endif
		break;

	case DJ_BIN_CHECK:
		BldcCtrl_Stop(&g_stir_ctrl);
		BldcCtrl_Stop(&g_grind_ctrl);
		gpio_ctrl_off(GPIO_OUT_BLDC_FAN);    c->fanb_on = 0U;
		/* 내부정화팬(fanx)은 배출 완료(래치 해제)까지 유지 - 끄지 않음 */
		break;

	case DJ_DISCHARGE:
#if DJ_TEST_FAST_TIMING
		dj_test_beep(3U);                            /* [TEST] 9분대(130분) 삑삑삑 */
#endif
		/* 배출문 개방 = 정방향(CW). 리미트(THALL_OPEN, PF5) 인식까지 계속 구동. */
		c->disc_open_ms = 0U; c->disc_close_ms = 0U; c->disc_pulse_off = 0U;
		TDoor_Enable(); TDoor_Open((uint8_t)DJ_DOOR_DUTY_PCT);
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
	case DJ_ERROR:
		dj_all_off();
		c->fanb_on = 0U; c->fanx_on = 0U;   /* 배기팬 duty 정지 */
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
#if DJ_TEST_FAST_TIMING
	c->dbg_beep = 0U;
#endif
	c->temp_d10 = 0; c->temp_valid = 0U; c->temp2_d10 = 0; c->temp2_valid = 0U; c->vapor_temp_d10 = 0;
	c->water_reached = 0U; c->bin_fill_pct = 0U; c->hs_prev = 0U;
	c->abort_req = 0U; c->dbg_force_stop = 0U; c->lid_guard = 0U; c->lid_low_cnt = 0U;
	c->err_code = (uint8_t)DJ_ERR_NONE; c->err_clear_req = 0U;
	c->cycle_count = 0U;
	c->heat_started = 0U; c->heat_since = 0U;
#if DJ_TEST_FAST_TIMING
	c->test_beeped_hi = 0U;                       /* [TEST] 비프 래치 리셋 */
#endif
	c->stir_phase = 0U; c->stir_since = 0U; c->stir_reps = 0U;
	c->grind_mode = (uint8_t)DJ_GM_OFF; c->grind_phase = 0U; c->grind_final_ph = 0U;
	c->grind_since = 0U; c->grind_start = 0U; c->cool_phase = 0U;
	c->vapor_phase = (uint8_t)DJ_VP_CLOSED; c->vapor_step_since = 0U; c->vapor_step_cnt = 0U;
	c->fanb_on = 0U; c->fanb_since = 0U;
	c->fanx_on = 0U; c->fanx_since = 0U;
	c->disc_phase = 0U; c->disc_since = 0U;
	c->disc_open_ms = 0U; c->disc_close_ms = 0U; c->disc_pulse_off = 0U;
	StepMotor_Init(&dj_duct);        /* 코일 해제 + 시퀀스 인덱스 리셋         */
	StepMotor_Init(&dj_air);
	dj_all_off();
}

void Dongjak_Start(void)
{
	if (g_dongjak.state == (uint8_t)DJ_IDLE) { g_dongjak.start_req = 1U; }
}

void Dongjak_Abort(void)
{
	dj_all_off();
	g_dongjak.start_req    = 0U;
	g_dongjak.dbg_enter_heat = 0U;
	g_dongjak.abort_req    = 0U;
	g_dongjak.dbg_force_stop = 0U;
	g_dongjak.lid_guard    = 0U;
	g_dongjak.lid_low_cnt  = 0U;
	g_dongjak.heat_started = 0U;
	g_dongjak.grind_mode   = (uint8_t)DJ_GM_OFF;
	g_dongjak.cool_phase   = 0U;
	g_dongjak.vapor_phase  = (uint8_t)DJ_VP_CLOSED;
	g_dongjak.fanb_on      = 0U;
	g_dongjak.fanx_on      = 0U;
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

	c->scn_start = now;                              /* 시나리오 경과 0부터        */
	dj_fan_exhaust_begin(c, now);                    /* 배기팬 15/2 duty 개시(벤치도 동일) */
	dj_enter(c, DJ_HEAT, now);                       /* heat_started=0 + 배수문 닫힘 */

	if (skip_door)
	{
		WDoor_Stop(); WDoor_Disable();               /* 도어 구동 즉시 정지        */
		c->heat_started = 1U; c->heat_since = now;   /* 가열중으로 강제            */
		dj_stir_dry_begin(c, now);                   /* 교반 CW 즉시 개시          */
		c->grind_mode  = (uint8_t)DJ_GM_OFF; c->grind_start = 0U;
		c->vapor_phase = (uint8_t)DJ_VP_CLOSED;
		c->fanb_on = 0U; c->fanb_since = now;
	}
}

/* 100ms, StartDefaultTask - 센서만. */
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
	/* HS5 시작 에지와 마개 이탈(HS_LOST) 감시는 중재자가 담당한다. 여기서 또
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

#if DJ_TEST_FAST_TIMING
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
	uint32_t    sel = now_ms - c->scn_start;   /* 시나리오 절대 경과           */

	/* 4.5 비상정지 요청 소비: 처리 중이면 안전정지(DJ_ABORTED)로 전이.
	 * DJ_ERROR에서 정지요청이 오면 에러를 해제하고 IDLE로 복구(정지=리셋 겸용). */
	if (c->abort_req)
	{
		c->abort_req = 0U;
		if ((DongjakState)c->state == DJ_ERROR)
		{
			c->err_code = (uint8_t)DJ_ERR_NONE;
			c->lid_guard = 0U; c->lid_low_cnt = 0U;
			dj_enter(c, DJ_IDLE, now_ms);
		}
		else if (Dongjak_IsBusy() && ((DongjakState)c->state != DJ_ABORTED))
		{
			dj_enter(c, DJ_ABORTED, now_ms);
		}
	}

	/* 배기팬(FAN_EXHAUST) 15/2 duty: 처리 상태(RINSE1_CLOSE~DISCHARGE) 동안 매 틱 갱신.
	 * IDLE/DONE/ERROR/ABORTED에서는 돌지 않는다(시작은 dj_fan_exhaust_begin). */
	if (((DongjakState)c->state >= DJ_RINSE1_CLOSE) &&
	    ((DongjakState)c->state <= DJ_DISCHARGE))
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
			dj_fan_exhaust_begin(c, now_ms);         /* 배기팬 15/2 duty 개시(시나리오 시작) */
			dj_enter(c, DJ_RINSE1_CLOSE, now_ms);
		}
		break;

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
		dj_rinse_tick(c, now_ms);
		break;

	case DJ_HEAT:
		if (!c->heat_started)
		{
			if (dj_door_done(WDoor_AtClose(), el))
			{
				WDoor_Stop(); WDoor_Disable();
				c->heat_started = 1U; c->heat_since = now_ms;
				dj_stir_dry_begin(c, now_ms);
				c->grind_mode = (uint8_t)DJ_GM_OFF; c->grind_start = 0U;
				c->vapor_phase = (uint8_t)DJ_VP_CLOSED;
				/* 배기팬(fanx)은 시나리오 시작 시 이미 개시됨 — 재초기화하지 않아 15분/2분
				 * duty를 시작부터 연속 유지(THERM3 무관 독립). */
				c->fanb_on = 0U; c->fanb_since = now_ms;
			}
			else if (el >= (uint32_t)DJ_DOOR_TIMEOUT_MS) { dj_fail(c, DJ_ERR_HEAT_DOOR, now_ms); }
			break;
		}
		/* 가열 진행: 히터/교반/분쇄/수증기/팬 동시 제어 */
		dj_heater_tick(c);
#if DJ_TEST_FAST_TIMING
		/* [TEST] 110분대(HISPEED) 진입 시 1회 삑 (상태전이 아니라 HEAT 내부라 래치 필요) */
		if ((sel >= (uint32_t)DJ_T_HISPEED_MS) && (c->test_beeped_hi == 0U))
		{
			c->test_beeped_hi = 1U;
			dj_test_beep(1U);
		}
#endif
		/* 교반: 건조 패턴(CW3/정지2 ×5 → CCW3) 유지, 110분↑ RPM만 20→27 (신스펙 항목3) */
		dj_stir_dry_tick(c, now_ms,
		    (sel >= (uint32_t)DJ_T_HISPEED_MS) ? (uint16_t)DJ_STIR_RPM_HISPEED
		                                       : (uint16_t)DJ_STIR_RPM);
		dj_grind_heat_tick(c, now_ms, sel);
		dj_vapor_tick(c, now_ms);          /* FAN_VAPOR+FAN_EXHAUST(THERM3) + 스텝 */
		dj_fan_bldc_tick(c, now_ms);       /* 분쇄 동작 중 식힘팬 30/10           */
		if (sel >= (uint32_t)DJ_T_COOLDOWN_MS)       /* 120분: 식힘             */
		{
			dj_vapor_off(c);
			dj_enter(c, DJ_COOLDOWN, now_ms);
		}
		break;

	case DJ_COOLDOWN:
		/* 히터 OFF(진입 시). 신스펙 식힘:
		 *  - 80℃ 이상(아직 뜨거움): 교반 = 지속 CW, 분쇄 = 1000 CCW 연속.
		 *  - 80℃ 미만(식음)      : 분쇄 OFF, 교반 = CW3/정지1/CCW3 반복. */
		if (c->cool_phase == 0U)
		{
			dj_grind_apply_mode(c, DJ_GM_COOL, now_ms);    /* 1000 CCW 연속       */
			/* 교반은 진입 시 지속 CW로 이미 기동됨 — 유지(재기동 안 함).           */
			if (dj_cool_reached(c))                        /* 센서<80℃ ‖ 바이메탈 하강엣지 */
			{
				dj_grind_apply_mode(c, DJ_GM_OFF, now_ms);           /* 분쇄 OFF     */
				dj_stir313_begin(c, now_ms, (uint16_t)DJ_STIR_RPM);  /* 교반 313 개시 */
				c->cool_phase = 1U;
			}
		}
		else                                                /* 식음: 분쇄 OFF + 교반 313 */
		{
			dj_grind_apply_mode(c, DJ_GM_OFF, now_ms);
			dj_stir313_tick(c, now_ms, (uint16_t)DJ_STIR_RPM);
		}
		dj_fan_bldc_tick(c, now_ms);                       /* 분쇄 동작 중 식힘팬(80℃↑ COOL 동안) */
		if (sel >= (uint32_t)DJ_T_DISCHARGE_MS)            /* 130분: 배출 전 확인 */
		{
			dj_enter(c, DJ_BIN_CHECK, now_ms);
		}
		break;

	case DJ_BIN_CHECK:
#if DJ_BIN_CHECK_ENABLE
		/* 수거통 유무 홀 채널 미확정(§1-2) -> 처리횟수 + 분말높이로 판정.
		 * 6회↑ 이거나 가득 차면 비움 안내(음성 미구현 TODO) 후 대기 -> 비우면 진행. */
		if ((c->cycle_count < (uint16_t)DJ_MAX_CYCLES) &&
		    (c->bin_fill_pct < (uint8_t)DJ_BIN_FULL_PCT))
		{
			dj_enter(c, DJ_DISCHARGE, now_ms);
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
		dj_enter(c, DJ_IDLE, now_ms);
		break;

	case DJ_ERROR:
		/* 에러 복구: err_clear_req(=Dongjak_ClearError) 세팅 시 IDLE(대기)로 복귀.
		 * 액추에이터는 진입 시 dj_all_off로 이미 정지. 이후 새 시작(HS5/강제)으로
		 * 처음부터 재개한다. err_code는 복구 직전까지 유지되어 원인 확인 가능. */
		if (c->err_clear_req)
		{
			c->err_clear_req = 0U;
			c->err_code      = (uint8_t)DJ_ERR_NONE;
			c->lid_guard     = 0U; c->lid_low_cnt = 0U;
			dj_enter(c, DJ_IDLE, now_ms);
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
