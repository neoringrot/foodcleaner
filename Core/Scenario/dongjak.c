/* ==========================================================================
 * dongjak.c - "동작"(2단계: 건조 -> 분쇄 -> 배출) 시나리오 상태머신 구현.
 * ★기준 원본: doc/R1/zerogeo_scenario.docx ([2단계] 동작, 2026-07-30). 최신·우선.
 * 설계/시퀀스/상수 개요는 dongjak.h 참조. 미검증 값은 DJ_* 매크로로 분리.
 *
 * 하드웨어 소유권:
 *   - 히터     : GPIO_OUT_HT_POWER (PA12). 히스테리시스 190/195 (210=HW 바이메탈).
 *   - 교반     : g_stir_ctrl  (M2/U16). 건조=CW3/정지2 x5+CCW3, 식힘/배출=CW3/1/CCW3.
 *   - 분쇄     : g_grind_ctrl (M1/U11). 80℃↑ 1500CW3/2 3분->1000CW연속, 110분↑ 2000CCW4/2.
 *   - 배수문   : WDoor (U5). 헹굼/잔수배수/최종 완전개방.
 *   - 배출문   : TDoor (U7). 배출(개방 후 2분간 DC 미구동, 이후 닫음).
 *   - 수증기   : 냄새관로 스텝(STEP1)+흡입 스텝(STEP2)+FAN_VAPOR. 100℃ ON/84℃ OFF.
 *   - 팬       : FAN_EXHAUST(내부정화 10/5), BLDC_FAN(80℃↑ 30/10), FAN_VAPOR(수증기).
 *   - 온도     : g_therm_c_d10[DJ_TEMP_CH]. 수거통: g_bin_fill_pct + cycle_count.
 *
 * 안전 락(HW, MCU 아님)은 dongjak.h 참조 - FW는 감시/구동만.
 * ========================================================================== */

#include "dongjak.h"

#include "gpio_ctrl.h"
#include "wdoor.h"
#include "tdoor.h"
#include "bldc_ctrl.h"
#include "hallsensor.h"
#include "thermistor.h"
#include "step_motor.h"

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

/* ---- 저수준 헬퍼 --------------------------------------------------------- */

static void dj_all_off(void)
{
	BldcCtrl_Stop(&g_grind_ctrl);
	BldcCtrl_Stop(&g_stir_ctrl);
	gpio_ctrl_off(GPIO_OUT_HT_POWER);
	gpio_ctrl_off(GPIO_OUT_VALVE_DRY_IN);
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

/* ---- 교반 패턴 A: 건조 (CW 3s/정지 2s x5 -> CCW 3s -> 반복, docx 5) ----- */
static void dj_stir_dry_begin(DongjakCtx *c, uint32_t now)
{
	c->stir_phase = (uint8_t)DJ_STIR_FWD;
	c->stir_reps  = 0U;
	c->stir_since = now;
	dj_stir_spin(0U, (uint16_t)DJ_STIR_RPM);   /* CW                          */
}

static void dj_stir_dry_tick(DongjakCtx *c, uint32_t now)
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
				dj_stir_spin(0U, (uint16_t)DJ_STIR_RPM);       /* CW           */
				c->stir_phase = (uint8_t)DJ_STIR_FWD;
			}
			else
			{
				dj_stir_spin(1U, (uint16_t)DJ_STIR_RPM);       /* CCW          */
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
			dj_stir_spin(0U, (uint16_t)DJ_STIR_RPM);           /* CW 재시작    */
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

/* ---- 히터: 190↓ON / 195↑OFF 히스테리시스 (+210 SW 보조) ---------------- */
static void dj_heater_tick(DongjakCtx *c)
{
	int16_t t = c->temp_d10;
	if (t == THERMISTOR_ERR_D10) { return; }
	if (t >= (int16_t)DJ_TEMP_SAFETY_D10) { gpio_ctrl_off(GPIO_OUT_HT_POWER); return; }
	if (t >= (int16_t)DJ_TEMP_HEATER_OFF_D10)      { gpio_ctrl_off(GPIO_OUT_HT_POWER); }
	else if (t <= (int16_t)DJ_TEMP_HEATER_ON_D10)  { gpio_ctrl_on(GPIO_OUT_HT_POWER); }
}

/* ---- 분쇄: 모드(시간·온도) 결정 후 토글/연속 구동 ---------------------- *
 * 80℃ 미만 = OFF. 110분↑ = 2000 CCW 토글. 그 외 개시 3분 = 1500 CW 토글,
 * 이후 = 1000 CW 연속. (식힘 구간은 DJ_GM_COOL: 1000 CCW 연속) */
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

	if (sel >= (uint32_t)DJ_T_HISPEED_MS)           /* 110분↑: 2000 CCW 4/2    */
	{
		dj_grind_apply_mode(c, DJ_GM_FINAL, now);
		dj_grind_toggle(c, now, 1U, (uint16_t)DJ_GRIND_FINAL_RPM,
		                (uint32_t)DJ_GRIND_RUN_FINAL_MS, (uint32_t)DJ_GRIND_STOP_FINAL_MS);
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

/* ---- 수증기: 100℃ 개방(방수팬->관로->흡입), 84℃ 역순 폐쇄 (docx 7.1) -- */
static void dj_vapor_tick(DongjakCtx *c, uint32_t now)
{
	int16_t  t  = c->temp_d10;
	uint8_t  due = (uint8_t)((now - c->vapor_step_since) >= (uint32_t)DJ_STEP_INTERVAL_MS);

	switch ((DjVaporPhase)c->vapor_phase)
	{
	case DJ_VP_CLOSED:
		if (t >= (int16_t)DJ_TEMP_VAPOR_ON_D10)
		{
			gpio_ctrl_on(GPIO_OUT_FAN_VAPOR);        /* 1) 방수팬 ON            */
			c->vapor_step_cnt = 0U; c->vapor_step_since = now;
			c->vapor_phase = (uint8_t)DJ_VP_OPEN_DUCT;
		}
		break;
	case DJ_VP_OPEN_DUCT:                             /* 2) 냄새 관로 OPEN       */
		if (due)
		{
			StepMotor_Step(&dj_duct, 1); c->vapor_step_cnt++; c->vapor_step_since = now;
			if (c->vapor_step_cnt >= (uint16_t)DJ_DUCT_STEPS)
			{ c->vapor_step_cnt = 0U; c->vapor_phase = (uint8_t)DJ_VP_OPEN_AIR; }
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
			{ StepMotor_Release(&dj_air); c->vapor_step_cnt = 0U; c->vapor_phase = (uint8_t)DJ_VP_CLOSE_DUCT; }
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
				gpio_ctrl_off(GPIO_OUT_FAN_VAPOR);   /* 3') 방수팬 OFF          */
				c->vapor_phase = (uint8_t)DJ_VP_CLOSED;
			}
		}
		break;
	}
}

static void dj_vapor_off(DongjakCtx *c)
{
	gpio_ctrl_off(GPIO_OUT_FAN_VAPOR);
	StepMotor_Release(&dj_duct);
	StepMotor_Release(&dj_air);
	c->vapor_phase = (uint8_t)DJ_VP_CLOSED;
}

/* ---- 팬: 내부정화(10/5 연속), BLDC식힘(80℃↑ 30/10) (docx 7) ----------- */
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

static void dj_fan_bldc_tick(DongjakCtx *c, uint32_t now)
{
	if (c->temp_d10 < (int16_t)DJ_TEMP_BLDCFAN_ON_D10)  /* 80℃ 미만: OFF        */
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

/* ---- 초기 헹굼 서브-FSM (docx 2단계 2번: 1단계 2~6번 x2) --------------- */
static void dj_rinse_enter(DongjakCtx *c, DjRinsePhase p, uint32_t now)
{
	c->rinse_phase = (uint8_t)p;
	c->rinse_since = now;
	switch (p)
	{
	case DJ_RS_CLOSE:
		WDoor_Enable(); WDoor_Close((uint8_t)DJ_DOOR_DUTY_PCT);
		break;
	case DJ_RS_FILL:
		c->water_reached = 0U;
		gpio_ctrl_exti_flag_clear(GPIO_EXTI_WATER_SEN1);
		gpio_ctrl_exti_flag_clear(GPIO_EXTI_WATER_SEN2);
		gpio_ctrl_on(GPIO_OUT_VALVE_DRY_IN);
		break;
	case DJ_RS_STIR:
		dj_stir313_begin(c, now, (uint16_t)DJ_RINSE_STIR_RPM);
		break;
	case DJ_RS_OPEN:
		WDoor_Enable(); WDoor_Open((uint8_t)DJ_DOOR_DUTY_PCT);
		break;
	case DJ_RS_FILL_EXTRA:
	case DJ_RS_DRAIN:
	default:
		break;
	}
}

/* 헹굼 1회 = 닫힘 -> 급수 -> 2초 -> 교반 2분30초 -> 개방 -> 짧은 배수.
 * docx는 "1단계 2~6번(배수 제외)"이라 명시하나, 배수 없이 2회 급수 시 2회차
 * 수위가 이미 도달 상태 -> 에지 감지 불가(교착) + 물리적 오버플로 위험. 따라서
 * 헹굼 사이에 개방/짧은 배수를 넣어 매 회 신선한 급수·감지가 되게 한다(설계 판단). */
static void dj_rinse_tick(DongjakCtx *c, uint32_t now)
{
	uint32_t el = now - c->rinse_since;
	switch ((DjRinsePhase)c->rinse_phase)
	{
	case DJ_RS_CLOSE:
		if (WDoor_AtClose())
		{
			WDoor_Stop(); WDoor_Disable();
			dj_rinse_enter(c, DJ_RS_FILL, now);
		}
		else if (el >= (uint32_t)DJ_DOOR_TIMEOUT_MS) { dj_enter(c, DJ_ERROR, now); }
		break;

	case DJ_RS_FILL:
		if (c->water_reached) { dj_rinse_enter(c, DJ_RS_FILL_EXTRA, now); }
		else if (el >= (uint32_t)DJ_FILL_TIMEOUT_MS)
		{ gpio_ctrl_off(GPIO_OUT_VALVE_DRY_IN); dj_enter(c, DJ_ERROR, now); }
		break;

	case DJ_RS_FILL_EXTRA:
		if (el >= (uint32_t)DJ_FILL_EXTRA_MS)
		{ gpio_ctrl_off(GPIO_OUT_VALVE_DRY_IN); dj_rinse_enter(c, DJ_RS_STIR, now); }
		break;

	case DJ_RS_STIR:
		dj_stir313_tick(c, now, (uint16_t)DJ_RINSE_STIR_RPM);
		if (el >= (uint32_t)DJ_RINSE_STIR_MS)
		{
			BldcCtrl_Stop(&g_stir_ctrl);
			dj_rinse_enter(c, DJ_RS_OPEN, now);
		}
		break;

	case DJ_RS_OPEN:
		if (WDoor_AtOpen())
		{
			WDoor_Stop(); WDoor_Disable();
			dj_rinse_enter(c, DJ_RS_DRAIN, now);
		}
		else if (el >= (uint32_t)DJ_DOOR_TIMEOUT_MS) { dj_enter(c, DJ_ERROR, now); }
		break;

	case DJ_RS_DRAIN:
	default:
		if (el >= (uint32_t)DJ_RINSE_DRAIN_MS)
		{
			c->rinse_iter++;
			if (c->rinse_iter < (uint8_t)DJ_RINSE_COUNT)
			{
				dj_rinse_enter(c, DJ_RS_CLOSE, now);  /* 다음 헹굼               */
			}
			else
			{
				dj_enter(c, DJ_DRAIN_RESIDUAL, now);  /* 2회 완료 -> 잔수 배수    */
			}
		}
		break;
	}
}

/* ---- 배출 서브-FSM (docx 10·11번) -------------------------------------- */
static void dj_disc_enter(DongjakCtx *c, DjDischPhase p, uint32_t now)
{
	c->disc_phase = (uint8_t)p;
	c->disc_since = now;
}

static void dj_discharge_tick(DongjakCtx *c, uint32_t now)
{
	uint32_t el = now - c->disc_since;
	switch ((DjDischPhase)c->disc_phase)
	{
	case DJ_DS_OPEN_T:                                /* 배출문 개방            */
		if (TDoor_AtOpen())
		{
			TDoor_Stop(); TDoor_Disable();           /* 이후 2분간 DC 미구동    */
			dj_stir313_begin(c, now, (uint16_t)DJ_STIR_RPM);
			dj_disc_enter(c, DJ_DS_EXPEL, now);
		}
		else if (el >= (uint32_t)DJ_DOOR_TIMEOUT_MS) { dj_enter(c, DJ_ERROR, now); }
		break;

	case DJ_DS_EXPEL:                                 /* 2분 교반 배출          */
		dj_stir313_tick(c, now, (uint16_t)DJ_STIR_RPM);
		if (el >= (uint32_t)DJ_DISCHARGE_STIR_MS)
		{
			BldcCtrl_Stop(&g_stir_ctrl);
			TDoor_Enable(); TDoor_Close((uint8_t)DJ_DOOR_DUTY_PCT);
			dj_disc_enter(c, DJ_DS_CLOSE_T, now);
		}
		break;

	case DJ_DS_CLOSE_T:                               /* 배출문 닫음(래치 해제)  */
		if (TDoor_AtClose())
		{
			TDoor_Stop(); TDoor_Disable();
			WDoor_Enable(); WDoor_Open((uint8_t)DJ_DOOR_DUTY_PCT); /* 배수부 완전개방 */
			dj_disc_enter(c, DJ_DS_OPEN_W, now);
		}
		else if (el >= (uint32_t)DJ_DOOR_TIMEOUT_MS) { dj_enter(c, DJ_ERROR, now); }
		break;

	case DJ_DS_OPEN_W:
	default:
		if (WDoor_AtOpen())
		{
			WDoor_Stop(); WDoor_Disable();
			c->cycle_count++;
			dj_enter(c, DJ_DONE, now);
		}
		else if (el >= (uint32_t)DJ_DOOR_TIMEOUT_MS) { dj_enter(c, DJ_ERROR, now); }
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
	case DJ_RINSE:
		c->rinse_iter = 0U;
		dj_rinse_enter(c, DJ_RS_CLOSE, now);
		break;

	case DJ_DRAIN_RESIDUAL:
		WDoor_Enable(); WDoor_Open((uint8_t)DJ_DOOR_DUTY_PCT); /* 문 개방        */
		dj_stir313_begin(c, now, (uint16_t)DJ_RINSE_STIR_RPM); /* 2분 교반 배수  */
		break;

	case DJ_HEAT:
		c->heat_started = 0U;                        /* 먼저 배수문 닫힘 확인   */
		WDoor_Enable(); WDoor_Close((uint8_t)DJ_DOOR_DUTY_PCT);
		break;

	case DJ_COOLDOWN:
		gpio_ctrl_off(GPIO_OUT_HT_POWER);            /* 히터 OFF               */
		dj_stir313_begin(c, now, (uint16_t)DJ_STIR_RPM);
		break;

	case DJ_BIN_CHECK:
		BldcCtrl_Stop(&g_stir_ctrl);
		BldcCtrl_Stop(&g_grind_ctrl);
		gpio_ctrl_off(GPIO_OUT_BLDC_FAN);    c->fanb_on = 0U;
		/* 내부정화팬(fanx)은 배출 완료(래치 해제)까지 유지 - 끄지 않음 */
		break;

	case DJ_DISCHARGE:
		TDoor_Enable(); TDoor_Open((uint8_t)DJ_DOOR_DUTY_PCT);
		dj_disc_enter(c, DJ_DS_OPEN_T, now);
		break;

	case DJ_DONE:
	case DJ_ERROR:
		dj_all_off();
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
	c->start_req = 0U; c->dbg_force_start = 0U; c->temp_d10 = 0;
	c->water_reached = 0U; c->bin_fill_pct = 0U; c->hs_prev = 0U;
	c->cycle_count = 0U;
	c->rinse_iter = 0U; c->rinse_phase = 0U; c->rinse_since = 0U;
	c->heat_started = 0U; c->heat_since = 0U;
	c->stir_phase = 0U; c->stir_since = 0U; c->stir_reps = 0U;
	c->grind_mode = (uint8_t)DJ_GM_OFF; c->grind_phase = 0U;
	c->grind_since = 0U; c->grind_start = 0U;
	c->vapor_phase = (uint8_t)DJ_VP_CLOSED; c->vapor_step_since = 0U; c->vapor_step_cnt = 0U;
	c->fanx_on = 0U; c->fanx_since = 0U; c->fanb_on = 0U; c->fanb_since = 0U;
	c->disc_phase = 0U; c->disc_since = 0U;
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
	g_dongjak.heat_started = 0U;
	g_dongjak.grind_mode   = (uint8_t)DJ_GM_OFF;
	g_dongjak.vapor_phase  = (uint8_t)DJ_VP_CLOSED;
	g_dongjak.fanx_on      = 0U;
	g_dongjak.fanb_on      = 0U;
	g_dongjak.state        = (uint8_t)DJ_IDLE;
	g_dongjak.state_since  = HAL_GetTick();
}

/* 100ms, StartDefaultTask - 센서만. */
void Dongjak_SenseTick(void)
{
	uint8_t hs      = HallSensor_Get((uint8_t)DJ_HS_START_IDX);
	uint8_t hs_edge = (uint8_t)((hs != 0U) && (g_dongjak.hs_prev == 0U));
	if (hs_edge || (g_dongjak.dbg_force_start != 0U))
	{
		g_dongjak.dbg_force_start = 0U;
		Dongjak_Start();
	}
	g_dongjak.hs_prev = hs;

	{
		int16_t t = g_therm_c_d10[DJ_TEMP_CH];
		if (t != THERMISTOR_ERR_D10) { g_dongjak.temp_d10 = t; }
	}
	if (dj_water_present()) { g_dongjak.water_reached = 1U; }
	g_dongjak.bin_fill_pct = g_bin_fill_pct;
}

/* 1ms, StartMotorTask - 상태머신 소유 + 모든 액추에이터 구동. */
void Dongjak_MotorTick(uint32_t now_ms)
{
	DongjakCtx *c  = &g_dongjak;
	uint32_t    el = now_ms - c->state_since;
	uint32_t    sel = now_ms - c->scn_start;   /* 시나리오 절대 경과           */

	switch ((DongjakState)c->state)
	{
	case DJ_IDLE:
		if (c->start_req)
		{
			c->start_req = 0U;
			c->scn_start = now_ms;
			dj_enter(c, DJ_RINSE, now_ms);
		}
		break;

	case DJ_RINSE:
		dj_rinse_tick(c, now_ms);
		break;

	case DJ_DRAIN_RESIDUAL:
		if (WDoor_AtOpen()) { WDoor_Stop(); WDoor_Disable(); }
		dj_stir313_tick(c, now_ms, (uint16_t)DJ_RINSE_STIR_RPM);
		if (el >= (uint32_t)DJ_DRAIN_RESIDUAL_MS)
		{
			BldcCtrl_Stop(&g_stir_ctrl);
			dj_enter(c, DJ_HEAT, now_ms);
		}
		break;

	case DJ_HEAT:
		if (!c->heat_started)
		{
			if (WDoor_AtClose())
			{
				WDoor_Stop(); WDoor_Disable();
				c->heat_started = 1U; c->heat_since = now_ms;
				dj_stir_dry_begin(c, now_ms);
				c->grind_mode = (uint8_t)DJ_GM_OFF; c->grind_start = 0U;
				c->vapor_phase = (uint8_t)DJ_VP_CLOSED;
				c->fanx_on = 1U; c->fanx_since = now_ms; gpio_ctrl_on(GPIO_OUT_FAN_EXHAUST);
				c->fanb_on = 0U; c->fanb_since = now_ms;
			}
			else if (el >= (uint32_t)DJ_DOOR_TIMEOUT_MS) { dj_enter(c, DJ_ERROR, now_ms); }
			break;
		}
		/* 가열 진행: 히터/교반/분쇄/수증기/팬 동시 제어 */
		dj_heater_tick(c);
		dj_stir_dry_tick(c, now_ms);
		dj_grind_heat_tick(c, now_ms, sel);
		dj_vapor_tick(c, now_ms);
		dj_fan_exhaust_tick(c, now_ms);
		dj_fan_bldc_tick(c, now_ms);
		if (sel >= (uint32_t)DJ_T_COOLDOWN_MS)       /* 120분: 식힘             */
		{
			dj_vapor_off(c);
			dj_enter(c, DJ_COOLDOWN, now_ms);
		}
		break;

	case DJ_COOLDOWN:
		/* 히터 OFF(진입 시). 교반 CW3/1/CCW3 유지, 분쇄 1000 CCW->80℃서 OFF. */
		dj_stir313_tick(c, now_ms, (uint16_t)DJ_STIR_RPM);
		if (c->temp_d10 >= (int16_t)DJ_TEMP_GRIND_ON_D10)
		{
			dj_grind_apply_mode(c, DJ_GM_COOL, now_ms);   /* 1000 CCW 연속       */
		}
		else
		{
			dj_grind_apply_mode(c, DJ_GM_OFF, now_ms);     /* 80℃ 미만: 분쇄 OFF */
		}
		dj_fan_bldc_tick(c, now_ms);                       /* 80℃↑ 식힘팬 유지   */
		dj_fan_exhaust_tick(c, now_ms);                    /* 내부정화팬 유지     */
		if (sel >= (uint32_t)DJ_T_DISCHARGE_MS)            /* 130분: 배출 전 확인 */
		{
			dj_enter(c, DJ_BIN_CHECK, now_ms);
		}
		break;

	case DJ_BIN_CHECK:
		dj_fan_exhaust_tick(c, now_ms);                    /* 내부정화팬 유지     */
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
		dj_fan_exhaust_tick(c, now_ms);                    /* 배출 완료까지 유지  */
		dj_discharge_tick(c, now_ms);
		break;

	case DJ_DONE:
		dj_enter(c, DJ_IDLE, now_ms);
		break;

	case DJ_ERROR:
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
