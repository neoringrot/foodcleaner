#include "tb_app.h"

#if ENABLE_TESTBENCH_APP

#include "protocol_r0.h"      /* PROTO_LEN_TB_* / proto_nak_t                */
#include "mode_arbiter.h"     /* g_app_mode / AppMode_IsBenchIdle / g_modearb */
#include "jungji.h"           /* Jungji_IsBraking / IsCooling                 */
#include "gpio_ctrl.h"        /* 분쇄 허가(PF0/PF3) 관측                      */
#include "keypad.h"           /* U31/U32 스냅샷                               */
#include "hallsensor.h"       /* HS1~8 마스크                                 */

/* 제어 대상 tb_* 변수들. 이 파일이 테스트벤치 헤더를 모아 쥐는 대신
 * protocol_r0.c 는 tb_app.h 하나만 본다(tb_app.h 머리주석 참조). */
#include "tb_tca9554.h"
#include "tb_drv8871.h"
#include "wdoor.h"           /* 도어 리밋 직접 디코드 (§0.42)                 */
#include "tdoor.h"
#include "tb_stepmotor.h"
#include "tb_lift.h"
#include "tb_gpioout.h"
#include "tb_water.h"
#include "tb_heat.h"
#include "tb_hallsensor.h"
#include "tb_doorhall.h"
#include "tb_distance.h"
#include "tb_thermistor.h"
#include "tb_speaker.h"
#include "tb_rinse.h"
#include "tb_rotation.h"   /* 회전수 운전 엔진 벤치 (§0.39) */
#include "tb_voice.h"      /* U21 음성 플래시 벤치 (§0.39)  */
#include "tb_protocol.h"   /* R0 코덱 자체검사 (§0.39)      */
#include "voice.h"         /* Voice_IsBusy                  */

TbAppCtx g_tb_app;

/* ---- 작은 헬퍼 ----------------------------------------------------------- */

static inline void put_u16(uint8_t *b, uint16_t i, uint16_t v)
{
	b[i]     = (uint8_t)(v & 0xFFU);
	b[i + 1] = (uint8_t)(v >> 8);
}

static inline void put_u32(uint8_t *b, uint16_t i, uint32_t v)
{
	b[i]     = (uint8_t)(v & 0xFFU);
	b[i + 1] = (uint8_t)((v >> 8)  & 0xFFU);
	b[i + 2] = (uint8_t)((v >> 16) & 0xFFU);
	b[i + 3] = (uint8_t)((v >> 24) & 0xFFU);
}

static inline uint8_t bit(uint8_t cond, uint8_t mask)
{
	return cond ? mask : 0U;
}

/* u32 를 65535 로 물려 u16 에 담는다(경과/런 길이 표시용). */
static uint16_t clamp_u16(uint32_t v)
{
	return (v > 0xFFFFUL) ? 0xFFFFU : (uint16_t)v;
}

/* 분쇄 허가 관측. dongjak.c 의 dj_grind_allowed() 와 **같은 식**이다 -
 * 회로가 U15/U26 으로 AND 하는 것을 MCU 가 되읽을 수 없어 PF0+PF3 재계산이
 * 유일한 관측 경로다(구현현황 §0.19.1). 허가 없이 분쇄를 명령하면 FG 가 오지
 * 않아 약 10초 뒤 BLDC_LOCKED 로 떨어지므로 앱은 이 비트를 먼저 본다. */
static uint8_t grind_allowed(void)
{
	return (uint8_t)((gpio_ctrl_exti_read(GPIO_EXTI_BIMETAL_70)  == 0U) &&
	                 (gpio_ctrl_exti_read(GPIO_EXTI_WHALL_CLOSE) == 0U));
}

/* ==========================================================================
 * 읽기
 * ========================================================================== */
uint32_t TbApp_Read(tb_item_t item)
{
	switch (item)
	{
	/* --- BLDC --- */
	case TB_IT_GRIND_EN:       return tb_grind_en;
	case TB_IT_GRIND_REV:      return tb_grind_rev;
	case TB_IT_GRIND_SPD:      return tb_grind_spd_req;
	case TB_IT_STIR_EN:        return tb_stir_en;
	case TB_IT_STIR_REV:       return tb_stir_rev;
	case TB_IT_STIR_SPD:       return tb_stir_spd_req;
	case TB_IT_KEYPAD_MOTOR:   return tb_keypad_motor_en;

	/* --- 도어 --- */
	case TB_IT_WDOOR_EN:       return tb_wdoor_enable;
	case TB_IT_WDOOR_REV:      return tb_wdoor_reverse;
	case TB_IT_WDOOR_DUTY:     return tb_wdoor_duty;
	case TB_IT_WDOOR_PROFILE:  return tb_wdoor_profile;
	case TB_IT_WDOOR_LIMIT:    return tb_wdoor_limit_stop;
	case TB_IT_TDOOR_EN:       return tb_tdoor_enable;
	case TB_IT_TDOOR_REV:      return tb_tdoor_reverse;
	case TB_IT_TDOOR_DUTY:     return tb_tdoor_duty;
	case TB_IT_TDOOR_PROFILE:  return tb_tdoor_profile;
	case TB_IT_TDOOR_LIMIT:    return tb_tdoor_limit_stop;

	/* --- 리프트 --- */
	case TB_IT_LIFT_EN:        return tb_lift_enable;
	case TB_IT_LIFT_REV:       return tb_lift_reverse;
	case TB_IT_LIFT_DUTY:      return tb_lift_duty;
	case TB_IT_LIFT_PROFILE:   return tb_lift_profile;
	case TB_IT_LIFT_LIMIT:     return tb_lift_limit_stop;

	/* --- 스테퍼 --- */
	case TB_IT_STEP1_EN:       return tb_step1_enable;
	case TB_IT_STEP1_DIR:      return tb_step1_dir;
	case TB_IT_STEP1_PERIOD:   return tb_step1_period_ms;
	case TB_IT_STEP1_RUN_MS:   return tb_step1_run_ms;
	case TB_IT_STEP1_HOLD:     return tb_step1_hold;
	case TB_IT_STEP2_EN:       return tb_step2_enable;
	case TB_IT_STEP2_DIR:      return tb_step2_dir;
	case TB_IT_STEP2_PERIOD:   return tb_step2_period_ms;
	case TB_IT_STEP2_OPEN_MS:  return tb_step2_run_open_ms;
	case TB_IT_STEP2_CLOSE_MS: return tb_step2_run_close_ms;
	case TB_IT_STEP2_HOLD:     return tb_step2_hold;

	/* --- 밸브/팬 --- */
	case TB_IT_VALVE_DRAIN:    return tb_valve_drain_en;
	case TB_IT_VALVE_DRY:      return tb_valve_dry_en;
	case TB_IT_FAN_VAPOR:      return tb_fan_vapor_en;
	case TB_IT_FAN_EXHAUST:    return tb_fan_exhaust_en;
	case TB_IT_FAN_BLDC:       return tb_bldc_fan_en;

	/* --- 급수/히터 --- */
	case TB_IT_WATER_EN:       return tb_water_enable;
	case TB_IT_HEAT_EN:        return tb_heat_enable;
	case TB_IT_HEAT_MODE:      return tb_heat_mode;
	case TB_IT_HEAT_MANUAL:    return tb_heat_manual_on;
	case TB_IT_HEAT_CH:        return tb_heat_ch;
	case TB_IT_HEAT_ON_D10:    return (uint32_t)(uint16_t)tb_heat_on_d10;
	case TB_IT_HEAT_OFF_D10:   return (uint32_t)(uint16_t)tb_heat_off_d10;
	case TB_IT_HEAT_CLR_FAULT: return tb_heat_clear_fault;

	/* --- 센서 모니터 --- */
	case TB_IT_HALL_EN:        return tb_hall_enable;
	case TB_IT_DOORHALL_EN:    return tb_doorhall_enable;
	case TB_IT_DIST_EN:        return tb_dist_enable;
	case TB_IT_THERM_EN:       return tb_therm_enable;

	/* --- 헹굼 벤치 --- */
	case TB_IT_RINSE_SPIN:     return tb_rinse_spin_once;
	case TB_IT_RINSE_STOP:     return tb_rinse_stop_once;
	case TB_IT_RINSE_MARK:     return tb_rinse_mark_once;
	case TB_IT_RINSE_RPM:      return tb_rinse_spin_rpm;
	case TB_IT_RINSE_REV:      return tb_rinse_spin_rev;
	case TB_IT_RINSE_MS:       return tb_rinse_spin_ms;

	/* --- 스피커 --- */
	case TB_IT_SPEAKER_EN:     return tb_speaker_enable;
	case TB_IT_SPEAKER_FREQ:   return tb_speaker_freq;

	/* --- 중재자 --- */
	case TB_IT_ARB_DISABLE:    return g_modearb.dbg_disable;

	/* --- §0.39 추가 --- */
	case TB_IT_RINSE_BOTH:       return tb_rinse_both_edges;
	case TB_IT_RINSE_PAT:        return tb_rinse_pat_once;
	case TB_IT_RINSE_PAT_PARK:   return tb_rinse_pat_park;
	case TB_IT_RINSE_PAT_STOP40: return tb_rinse_pat_stop_at40;
	case TB_IT_ROT_SELFTEST:     return tb_rotation_selftest_once;
	case TB_IT_ROT_PROFILE:      return tb_rotation_profile;
	case TB_IT_ROT_RPM:          return tb_rotation_rpm;
	case TB_IT_ROT_POS_SRC:      return tb_rotation_pos_src;
	case TB_IT_ROT_MAX_LEGS:     return tb_rotation_max_legs;
	case TB_IT_ROT_RUN:          return tb_rotation_once;
	case TB_IT_ROT_STOP:         return tb_rotation_stop_once;
	case TB_IT_VOICE_SLOT:       return tb_voice_slot;
	case TB_IT_VOICE_PROBE:      return tb_voice_probe_once;
	case TB_IT_VOICE_DIR:        return tb_voice_dir_once;
	case TB_IT_VOICE_HDR:        return tb_voice_hdr_once;
	case TB_IT_VOICE_PLAY:       return tb_voice_play_once;
	case TB_IT_VOICE_STOP:       return tb_voice_stop_once;
	case TB_IT_VOICE_SELFTEST:   return tb_voice_selftest_once;
	case TB_IT_PROTO_SELFTEST:   return tb_proto_run_once;

	case TB_IT_ALL_OFF:
	case TB_IT_NONE:
	default:                   return 0U;
	}
}

/* ==========================================================================
 * 일괄 정지 - 모든 벤치 enable 을 0 으로
 *
 * jungji 의 정지와 무엇이 다른가: 이쪽은 **플래그만** 내린다. 단락제동도,
 * 도어 Brake 도, 시나리오 Abort 도 하지 않는다. 앱의 "전부 끄기" 버튼용이고,
 * 진짜 비상정지는 0x31 CONTROL 의 PROTO_ACT_STOP(= Jungji_StopAll)이다.
 *
 * ★tb_grind_en/tb_stir_en 은 엣지 입력이라 여기서 0 으로 내리는 것이 곧
 *   "다음 1 쓰기가 기동 엣지가 된다"는 뜻이다(구현현황 §0.20 과 같은 이유).
 * ========================================================================== */
void TbApp_AllOff(void)
{
	tb_grind_en       = 0U;
	tb_stir_en        = 0U;

	tb_wdoor_enable   = 0U;
	tb_tdoor_enable   = 0U;
	tb_lift_enable    = 0U;
	tb_step1_enable   = 0U;
	tb_step2_enable   = 0U;

	tb_valve_drain_en = 0U;
	tb_valve_dry_en   = 0U;
	tb_fan_vapor_en   = 0U;
	tb_fan_exhaust_en = 0U;
	tb_bldc_fan_en    = 0U;

	tb_water_enable   = 0U;
	tb_heat_enable    = 0U;
	tb_heat_manual_on = 0U;
	tb_speaker_enable = 0U;

	tb_rinse_stop_once = 1U;   /* 원샷 교반·패턴(run E)이 돌고 있으면 세운다 */
	tb_rotation_stop_once = 1U;/* 회전 엔진 run F 도 세운다 (§0.39)          */
	tb_voice_stop_once = 1U;   /* 재생 중인 음성도 끊는다                     */
}

/* ==========================================================================
 * 쓰기
 * ========================================================================== */

/* 범위 검사. 0 = 위반. 항목별로 "앱이 실수로 보낼 수 있는 값"만 막는다 -
 * 물리적 한계(듀티 100 초과 등)는 여기서, 의미적 판단은 각 tb_* 가 한다. */
static uint8_t value_ok(tb_item_t item, uint32_t v)
{
	switch (item)
	{
	/* 0/1 불리언 */
	case TB_IT_GRIND_EN:  case TB_IT_GRIND_REV: case TB_IT_GRIND_SPD:
	case TB_IT_STIR_EN:   case TB_IT_STIR_REV:  case TB_IT_STIR_SPD:
	case TB_IT_KEYPAD_MOTOR:
	case TB_IT_WDOOR_EN:  case TB_IT_WDOOR_REV: case TB_IT_WDOOR_PROFILE:
	case TB_IT_WDOOR_LIMIT:
	case TB_IT_TDOOR_EN:  case TB_IT_TDOOR_REV: case TB_IT_TDOOR_PROFILE:
	case TB_IT_TDOOR_LIMIT:
	case TB_IT_LIFT_EN:   case TB_IT_LIFT_REV:  case TB_IT_LIFT_PROFILE:
	case TB_IT_LIFT_LIMIT:
	case TB_IT_STEP1_EN:  case TB_IT_STEP1_DIR: case TB_IT_STEP1_HOLD:
	case TB_IT_STEP2_EN:  case TB_IT_STEP2_DIR: case TB_IT_STEP2_HOLD:
	case TB_IT_VALVE_DRAIN: case TB_IT_VALVE_DRY:
	case TB_IT_FAN_VAPOR: case TB_IT_FAN_EXHAUST: case TB_IT_FAN_BLDC:
	case TB_IT_WATER_EN:  case TB_IT_HEAT_EN:   case TB_IT_HEAT_MODE:
	case TB_IT_HEAT_MANUAL: case TB_IT_HEAT_CLR_FAULT:
	case TB_IT_HALL_EN:   case TB_IT_DOORHALL_EN:
	case TB_IT_DIST_EN:   case TB_IT_THERM_EN:
	case TB_IT_RINSE_SPIN: case TB_IT_RINSE_STOP: case TB_IT_RINSE_MARK:
	case TB_IT_RINSE_REV: case TB_IT_SPEAKER_EN:
	case TB_IT_ARB_DISABLE: case TB_IT_ALL_OFF:
	case TB_IT_RINSE_BOTH: case TB_IT_RINSE_PAT:
	case TB_IT_RINSE_PAT_PARK: case TB_IT_RINSE_PAT_STOP40:
	case TB_IT_ROT_SELFTEST: case TB_IT_ROT_POS_SRC:
	case TB_IT_ROT_RUN: case TB_IT_ROT_STOP:
	case TB_IT_VOICE_PROBE: case TB_IT_VOICE_DIR: case TB_IT_VOICE_HDR:
	case TB_IT_VOICE_PLAY: case TB_IT_VOICE_STOP: case TB_IT_VOICE_SELFTEST:
	case TB_IT_PROTO_SELFTEST:
		return (uint8_t)(v <= 1UL);

	/* 회전 엔진 (tb_rotation.h) */
	case TB_IT_ROT_PROFILE:
		return (uint8_t)((v >= 1UL) && (v <= 3UL));      /* WASH/DRAIN/PROCESS */
	case TB_IT_ROT_RPM:
		return (uint8_t)((v >= 5UL) && (v <= 60UL));
	case TB_IT_ROT_MAX_LEGS:
		return (uint8_t)((v >= 1UL) && (v <= 60UL));

	/* 음성 슬롯 (U21 = 256KB x 32) */
	case TB_IT_VOICE_SLOT:
		return (uint8_t)(v <= 31UL);

	/* 듀티 [%] */
	case TB_IT_WDOOR_DUTY: case TB_IT_TDOOR_DUTY: case TB_IT_LIFT_DUTY:
		return (uint8_t)(v <= 100UL);

	/* 스텝 주기 - 0 은 무한루프가 되므로 막는다(step_motor.h) */
	case TB_IT_STEP1_PERIOD: case TB_IT_STEP2_PERIOD:
		return (uint8_t)((v >= 1UL) && (v <= 1000UL));

	/* 런 길이 [ms]. 상한 10분 - 벤치에서 잊고 방치하는 것을 막는다. */
	case TB_IT_STEP1_RUN_MS: case TB_IT_STEP2_OPEN_MS:
	case TB_IT_STEP2_CLOSE_MS: case TB_IT_RINSE_MS:
		return (uint8_t)((v >= 1UL) && (v <= 600000UL));

	/* NTC 채널 */
	case TB_IT_HEAT_CH:
		return (uint8_t)(v <= 2UL);

	/* 히터 임계 [0.1℃]. i16 범위 안 + 물리적으로 말이 되는 구간.
	 * 상한 200.0℃ - 안전 임계(tb_heat_safety_d10, 기본 210.0℃)를 넘겨
	 * 설정하면 히스테리시스가 폴트보다 늦게 걸린다. */
	case TB_IT_HEAT_ON_D10: case TB_IT_HEAT_OFF_D10:
		return (uint8_t)(v <= 2000UL);

	/* 교반 원샷 RPM - STIR_LADDER 범위를 넉넉히 감싼다 */
	case TB_IT_RINSE_RPM:
		return (uint8_t)((v >= 5UL) && (v <= 60UL));

	case TB_IT_SPEAKER_FREQ:
		return (uint8_t)((v >= 100UL) && (v <= 8000UL));

	case TB_IT_NONE:
	default:
		return 0U;
	}
}

/* 실제 대입. value_ok() 를 통과한 값만 들어온다. */
static void apply(tb_item_t item, uint32_t v)
{
	uint8_t  v8  = (uint8_t)v;
	uint16_t v16 = (uint16_t)v;

	switch (item)
	{
	case TB_IT_GRIND_EN:       tb_grind_en        = v8;  break;
	case TB_IT_GRIND_REV:      tb_grind_rev       = v8;  break;
	case TB_IT_GRIND_SPD:      tb_grind_spd_req   = v8;  break;
	case TB_IT_STIR_EN:        tb_stir_en         = v8;  break;
	case TB_IT_STIR_REV:       tb_stir_rev        = v8;  break;
	case TB_IT_STIR_SPD:       tb_stir_spd_req    = v8;  break;
	case TB_IT_KEYPAD_MOTOR:   tb_keypad_motor_en = v8;  break;

	case TB_IT_WDOOR_EN:       tb_wdoor_enable     = v8; break;
	case TB_IT_WDOOR_REV:      tb_wdoor_reverse    = v8; break;
	case TB_IT_WDOOR_DUTY:     tb_wdoor_duty       = v8; break;
	case TB_IT_WDOOR_PROFILE:  tb_wdoor_profile    = v8; break;
	case TB_IT_WDOOR_LIMIT:    tb_wdoor_limit_stop = v8; break;
	case TB_IT_TDOOR_EN:       tb_tdoor_enable     = v8; break;
	case TB_IT_TDOOR_REV:      tb_tdoor_reverse    = v8; break;
	case TB_IT_TDOOR_DUTY:     tb_tdoor_duty       = v8; break;
	case TB_IT_TDOOR_PROFILE:  tb_tdoor_profile    = v8; break;
	case TB_IT_TDOOR_LIMIT:    tb_tdoor_limit_stop = v8; break;

	case TB_IT_LIFT_EN:        tb_lift_enable     = v8;  break;
	case TB_IT_LIFT_REV:       tb_lift_reverse    = v8;  break;
	case TB_IT_LIFT_DUTY:      tb_lift_duty       = v8;  break;
	case TB_IT_LIFT_PROFILE:   tb_lift_profile    = v8;  break;
	case TB_IT_LIFT_LIMIT:     tb_lift_limit_stop = v8;  break;

	case TB_IT_STEP1_EN:       tb_step1_enable       = v8;  break;
	case TB_IT_STEP1_DIR:      tb_step1_dir          = v8;  break;
	case TB_IT_STEP1_PERIOD:   tb_step1_period_ms    = v16; break;
	case TB_IT_STEP1_RUN_MS:   tb_step1_run_ms       = v;   break;
	case TB_IT_STEP1_HOLD:     tb_step1_hold         = v8;  break;
	case TB_IT_STEP2_EN:       tb_step2_enable       = v8;  break;
	case TB_IT_STEP2_DIR:      tb_step2_dir          = v8;  break;
	case TB_IT_STEP2_PERIOD:   tb_step2_period_ms    = v16; break;
	case TB_IT_STEP2_OPEN_MS:  tb_step2_run_open_ms  = v;   break;
	case TB_IT_STEP2_CLOSE_MS: tb_step2_run_close_ms = v;   break;
	case TB_IT_STEP2_HOLD:     tb_step2_hold         = v8;  break;

	case TB_IT_VALVE_DRAIN:    tb_valve_drain_en = v8; break;
	case TB_IT_VALVE_DRY:      tb_valve_dry_en   = v8; break;
	case TB_IT_FAN_VAPOR:      tb_fan_vapor_en   = v8; break;
	case TB_IT_FAN_EXHAUST:    tb_fan_exhaust_en = v8; break;
	case TB_IT_FAN_BLDC:       tb_bldc_fan_en    = v8; break;

	case TB_IT_WATER_EN:       tb_water_enable   = v8; break;
	case TB_IT_HEAT_EN:        tb_heat_enable    = v8; break;
	case TB_IT_HEAT_MODE:      tb_heat_mode      = v8; break;
	case TB_IT_HEAT_MANUAL:    tb_heat_manual_on = v8; break;
	case TB_IT_HEAT_CH:        tb_heat_ch        = v8; break;
	case TB_IT_HEAT_ON_D10:    tb_heat_on_d10    = (int16_t)v16; break;
	case TB_IT_HEAT_OFF_D10:   tb_heat_off_d10   = (int16_t)v16; break;
	case TB_IT_HEAT_CLR_FAULT: tb_heat_clear_fault = v8; break;

	case TB_IT_HALL_EN:        tb_hall_enable     = v8; break;
	case TB_IT_DOORHALL_EN:    tb_doorhall_enable = v8; break;
	case TB_IT_DIST_EN:        tb_dist_enable     = v8; break;
	case TB_IT_THERM_EN:       tb_therm_enable    = v8; break;

	case TB_IT_RINSE_SPIN:     tb_rinse_spin_once = v8;  break;
	case TB_IT_RINSE_STOP:     tb_rinse_stop_once = v8;  break;
	case TB_IT_RINSE_MARK:     tb_rinse_mark_once = v8;  break;
	case TB_IT_RINSE_RPM:      tb_rinse_spin_rpm  = v16; break;
	case TB_IT_RINSE_REV:      tb_rinse_spin_rev  = v8;  break;
	case TB_IT_RINSE_MS:       tb_rinse_spin_ms   = v;   break;

	case TB_IT_SPEAKER_EN:     tb_speaker_enable = v8;  break;
	case TB_IT_SPEAKER_FREQ:   tb_speaker_freq   = v16; break;

	case TB_IT_ARB_DISABLE:    g_modearb.dbg_disable = v8; break;

	case TB_IT_RINSE_BOTH:       tb_rinse_both_edges    = v8;  break;
	case TB_IT_RINSE_PAT:        tb_rinse_pat_once      = v8;  break;
	case TB_IT_RINSE_PAT_PARK:   tb_rinse_pat_park      = v8;  break;
	case TB_IT_RINSE_PAT_STOP40: tb_rinse_pat_stop_at40 = v8;  break;
	case TB_IT_ROT_SELFTEST:     tb_rotation_selftest_once = v8; break;
	case TB_IT_ROT_PROFILE:      tb_rotation_profile    = v8;  break;
	case TB_IT_ROT_RPM:          tb_rotation_rpm        = v16; break;
	case TB_IT_ROT_POS_SRC:      tb_rotation_pos_src    = v8;  break;
	case TB_IT_ROT_MAX_LEGS:     tb_rotation_max_legs   = v8;  break;
	case TB_IT_ROT_RUN:          tb_rotation_once       = v8;  break;
	case TB_IT_ROT_STOP:         tb_rotation_stop_once  = v8;  break;
	case TB_IT_VOICE_SLOT:       tb_voice_slot          = v8;  break;
	case TB_IT_VOICE_PROBE:      tb_voice_probe_once    = v8;  break;
	case TB_IT_VOICE_DIR:        tb_voice_dir_once      = v8;  break;
	case TB_IT_VOICE_HDR:        tb_voice_hdr_once      = v8;  break;
	case TB_IT_VOICE_PLAY:       tb_voice_play_once     = v8;  break;
	case TB_IT_VOICE_STOP:       tb_voice_stop_once     = v8;  break;
	case TB_IT_VOICE_SELFTEST:   tb_voice_selftest_once = v8;  break;
	case TB_IT_PROTO_SELFTEST:   tb_proto_run_once      = v8;  break;

	case TB_IT_ALL_OFF:
		if (v8) { TbApp_AllOff(); }
		break;

	case TB_IT_NONE:
	default:
		break;
	}
}

uint8_t TbApp_Write(tb_item_t item, uint32_t value, uint8_t *nak)
{
	uint8_t reason = (uint8_t)PROTO_NAK_NONE;
	uint8_t ok     = 0U;

	g_tb_app.last_item  = (uint8_t)item;
	g_tb_app.last_value = value;

	/* 설계규칙 (2): 벤치 유휴에서만 쓴다. 시나리오가 교반/도어를 쥐고 있는 동안
	 * 앱이 끼어들면 소유자가 둘이 된다. TB_IT_ALL_OFF 와 중재자 스위치는
	 * 예외로 허용한다 - "끄는" 쪽과 "벤치로 돌아오는" 쪽은 언제나 안전하다. */
	if ((item != TB_IT_ALL_OFF) && (item != TB_IT_ARB_DISABLE) &&
	    !AppMode_IsBenchIdle(g_app_mode))
	{
		reason = (uint8_t)PROTO_NAK_BUSY;
	}
	else if (!value_ok(item, value))
	{
		/* 미지원 항목도 여기로 떨어진다(value_ok 의 default = 0). */
		reason = (uint8_t)PROTO_NAK_BAD_VALUE;
	}
	else
	{
		apply(item, value);

		/* 설계규칙 (3): 쓰고 되읽어 검증(R0 §2). 원샷 항목은 펌웨어가 이미
		 * 소비해 0 이 되어 있을 수 있으므로 검증을 건너뛴다(tb_app.h 참조). */
		if (TB_ITEM_IS_ONESHOT(item) || (TbApp_Read(item) == value))
		{
			ok = 1U;
		}
		else
		{
			reason = (uint8_t)PROTO_NAK_VERIFY_FAIL;
		}
	}

	g_tb_app.last_result = (uint8_t)(ok ? 0U : 1U);
	g_tb_app.last_nak    = reason;
	if (ok) { if (g_tb_app.write_count  < 0xFFFFU) g_tb_app.write_count++;  }
	else    { if (g_tb_app.reject_count < 0xFFFFU) g_tb_app.reject_count++; }

	if (nak != NULL)
		*nak = reason;
	return ok;
}

/* ==========================================================================
 * 0x27 TB_STATE 페이로드
 * 바이트 배치의 정본은 protocol_r0.h 의 주석이다. 여기와 그쪽을 함께 고칠 것.
 * ========================================================================== */
uint16_t TbApp_BuildState(uint8_t *b, uint16_t buf_sz)
{
	uint8_t en1, en2, en3, hit, sens, misc;

	if ((b == NULL) || (buf_sz < PROTO_LEN_TB_STATE))
		return 0U;

	en1 = (uint8_t)(bit(tb_grind_en,     0x01U) | bit(tb_grind_rev,    0x02U) |
	                bit(tb_stir_en,      0x04U) | bit(tb_stir_rev,     0x08U) |
	                bit(tb_wdoor_enable, 0x10U) | bit(tb_wdoor_reverse,0x20U) |
	                bit(tb_tdoor_enable, 0x40U) | bit(tb_tdoor_reverse,0x80U));

	en2 = (uint8_t)(bit(tb_lift_enable,  0x01U) | bit(tb_lift_reverse, 0x02U) |
	                bit(tb_step1_enable, 0x04U) | bit(tb_step1_dir,    0x08U) |
	                bit(tb_step2_enable, 0x10U) | bit(tb_step2_dir,    0x20U) |
	                bit(tb_water_enable, 0x40U) | bit(tb_heat_enable,  0x80U));

	en3 = (uint8_t)(bit(tb_valve_drain_en, 0x01U) | bit(tb_valve_dry_en,   0x02U) |
	                bit(tb_fan_vapor_en,   0x04U) | bit(tb_fan_exhaust_en, 0x08U) |
	                bit(tb_bldc_fan_en,    0x10U) | bit(tb_speaker_enable, 0x20U) |
	                bit(tb_hall_enable,    0x40U) | bit(tb_doorhall_enable,0x80U));

	/* "모터가 limit 나 시간으로 멈췄다"를 한 바이트로. 전부 래치라 앱이 런
	 * 종료 사유를 놓치지 않는다(다음 런 시작에서 각 tb_* 가 지운다). */
	hit = (uint8_t)(bit(tb_wdoor_limit_hit, 0x01U) | bit(tb_wdoor_time_hit, 0x02U) |
	                bit(tb_tdoor_limit_hit, 0x04U) | bit(tb_tdoor_time_hit, 0x08U) |
	                bit(tb_lift_limit_hit,  0x10U) | bit(tb_lift_time_hit,  0x20U) |
	                bit(tb_tdoor_overrun,   0x40U) | bit(tb_lift_at_bottom, 0x80U));

	/* "센서가 인식되었다"를 한 바이트로. 수위는 tb_water 의 판정값이다.
	 * ★도어 4종은 드라이버를 **매번 직접** 읽는다(§0.42). 종전에는 tb_doorhall 의
	 *   스냅샷(tb_*door_at_*)을 실었는데, 그 값은 tb_doorhall_enable=1 일 때만
	 *   갱신되고 기본값이 0 이라 앱 도어 카드의 리밋 LED 가 부팅 직후 값에 멈춰
	 *   있었다(사용자 제보 "배출문 열림 센서 인식이 올바르지 않다"). 이제
	 *   0x23 SENSOR 의 limit 비트와 같은 식이다 - 극성 보정 포함 순간 레벨. */
	sens = (uint8_t)(bit(WDoor_AtOpen(),    0x01U) | bit(WDoor_AtClose(),   0x02U) |
	                 bit(TDoor_AtOpen(),    0x04U) | bit(TDoor_AtClose(),   0x08U) |
	                 bit(tb_water_present,  0x10U) | bit(tb_water_sen1_level,0x20U) |
	                 bit(tb_rinse_level,    0x40U) | bit(tb_rinse_spinning, 0x80U));

	misc = (uint8_t)(bit(grind_allowed(),                 0x01U) |
	                 bit(AppMode_IsBenchIdle(g_app_mode), 0x02U) |
	                 bit(g_modearb.dbg_disable,           0x04U) |
	                 bit(Keypad_IsPresent(),              0x08U) |
	                 bit(Jungji_IsBraking(),              0x10U) |
	                 bit(Jungji_IsCooling(),              0x20U) |
	                 bit(tb_heat_out,                     0x40U) |
	                 bit(tb_rinse_settling,               0x80U));

	b[0]  = en1;
	b[1]  = en2;
	b[2]  = en3;
	b[3]  = hit;
	b[4]  = sens;
	b[5]  = misc;
	b[6]  = Keypad_GetMask();
	b[7]  = Keypad_GetLedMask();
	b[8]  = HallSensor_GetMask();
	b[9]  = tb_heat_fault;
	put_u16(b, 10, (uint16_t)tb_heat_temp_d10);
	b[12] = (uint8_t)g_app_mode;
	b[13] = g_modearb.pos_stable;
	put_u16(b, 14, clamp_u16(tb_lift_last_run_ms));
	put_u32(b, 16, tb_rinse_count);
	put_u32(b, 20, tb_rinse_res_epr_x100);
	put_u16(b, 24, tb_rinse_res_rpm);
	put_u16(b, 26, tb_rinse_spin_rpm);
	b[28] = g_tb_app.last_item;
	b[29] = g_tb_app.last_result;
	b[30] = g_tb_app.last_nak;
	/* 도어 리밋 "이번 이동 중 인식" 래치 (§0.42). Reached*() = 이동 시작(LimitArm)
	 * 이후 EXTI 하강엣지가 한 번이라도 잡혔거나 지금 레벨이 도달 - 시나리오·벤치가
	 * 정지 판정에 쓰는 바로 그 값이다. 순간 레벨(b[4])은 모니터링 주기(500 ms)
	 * 사이에 자석을 스치고 지나가면 놓치지만 이 래치는 놓치지 않는다.
	 * 읽기만 한다 - 플래그를 지우는 것은 다음 이동의 LimitArm 뿐이다. */
	b[31] = (uint8_t)(bit(WDoor_ReachedOpen(),  0x01U) | bit(WDoor_ReachedClose(), 0x02U) |
	                  bit(TDoor_ReachedOpen(),  0x04U) | bit(TDoor_ReachedClose(), 0x08U));
	put_u16(b, 32, g_tb_app.write_count);
	put_u16(b, 34, g_tb_app.reject_count);

	return PROTO_LEN_TB_STATE;
}

/* ==========================================================================
 * 0x28 TB_STATE2 페이로드 (§0.39)
 * 0x27 이 36바이트로 찼고 PROTO_MON 마스크 8비트도 다 써서, 새 벤치들은 이
 * 패킷으로 모았다. 주기 송신은 PROTO_MON_TB_STATE 비트를 **0x27 과 공유**한다.
 * 바이트 배치의 정본은 protocol_r0.h §9. 여기와 pv_proto.parse_tb_state2 가 한 쌍.
 * 긴 시간은 0.1초 단위 u16 으로 줄였다(최대 6553초) - 패턴 기본 길이가 6분이라
 * ms 를 u16 에 담으면 넘친다.
 * ========================================================================== */
static uint16_t ds(uint32_t ms)   /* ms -> 0.1 s, u16 클램프 */
{
	return clamp_u16(ms / 100UL);
}

uint16_t TbApp_BuildState2(uint8_t *b, uint16_t buf_sz)
{
	if ((b == NULL) || (buf_sz < PROTO_LEN_TB_STATE2))
		return 0U;

	/* --- 회전수 운전 엔진 (tb_rotation) 0..27 --- */
	b[0]  = tb_rotation_state;
	b[1]  = tb_rotation_result;
	b[2]  = tb_rotation_fail_where;
	b[3]  = (uint8_t)(bit(tb_rotation_selftest_done, 0x01U) |
	                  bit(tb_rotation_locked,        0x02U) |
	                  bit(tb_rotation_initial_done,  0x04U) |
	                  bit(tb_rotation_is_late,       0x08U) |
	                  bit(tb_rotation_stir_rest,     0x10U) |
	                  bit(tb_rotation_grind_rest,    0x20U) |
	                  bit(tb_rotation_pos_src,       0x40U) |
	                  bit(tb_rotation_dir_invert,    0x80U));
	b[4]  = tb_rotation_profile;
	b[5]  = tb_rotation_dir;
	put_u16(b, 6,  clamp_u16(tb_rotation_legs));
	put_u32(b, 8,  tb_rotation_dir_mask);
	put_u16(b, 12, clamp_u16(tb_rotation_edges));
	b[14] = (tb_rotation_leg_edges > 255UL) ? 255U : (uint8_t)tb_rotation_leg_edges;
	b[15] = tb_rotation_max_legs;
	put_u16(b, 16, clamp_u16(tb_rotation_leg_ms));
	put_u16(b, 18, clamp_u16(tb_rotation_rest_wait_ms));
	put_u16(b, 20, clamp_u16(tb_rotation_rest_wait_max_ms));
	put_u16(b, 22, clamp_u16(tb_rotation_selftest_asserts));
	put_u16(b, 24, clamp_u16(tb_rotation_selftest_fail_line));
	put_u16(b, 26, tb_rotation_rpm);

	/* --- 헹굼 패턴 run E (tb_rinse_pat_*) 28..45 --- */
	b[28] = tb_rinse_pat_state;
	b[29] = tb_rinse_pat_result;
	b[30] = tb_rinse_pat_phase;
	b[31] = (uint8_t)(bit(tb_rinse_pat_park,       0x01U) |
	                  bit(tb_rinse_pat_stop_at40,  0x02U) |
	                  bit(tb_rinse_pat_park_level, 0x04U) |
	                  bit(tb_rinse_both_edges,     0x08U));
	put_u16(b, 32, clamp_u16(tb_rinse_pat_edges));
	put_u16(b, 34, clamp_u16(tb_rinse_pat_cycles));
	put_u16(b, 36, clamp_u16(tb_rinse_pat_epc_x100));
	put_u16(b, 38, ds(tb_rinse_pat_t20_ms));
	put_u16(b, 40, ds(tb_rinse_pat_t40_ms));
	put_u16(b, 42, ds(tb_rinse_pat_max_gap_ms));
	put_u16(b, 44, ds(tb_rinse_pat_elapsed_ms));

	/* --- 음성 플래시 U21 (tb_voice) 46..57 --- */
	b[46] = (uint8_t)tb_voice_status;        /* int8 w25q_status_t, 0 = OK */
	b[47] = tb_voice_play_err;
	b[48] = (uint8_t)(bit(tb_voice_dir_valid,  0x01U) |
	                  bit(tb_voice_dir_crc_ok, 0x02U) |
	                  bit(tb_voice_dir_blank,  0x04U) |
	                  bit(tb_voice_hdr_valid,  0x08U) |
	                  bit(tb_voice_hdr_crc_ok, 0x10U) |
	                  bit(tb_voice_hdr_blank,  0x20U) |
	                  bit(Voice_IsBusy(),      0x40U));
	b[49] = tb_voice_slot;
	b[50] = tb_voice_dir_slots;
	b[51] = tb_voice_step;
	put_u32(b, 52, tb_voice_jedec_id);
	put_u16(b, 56, tb_voice_fails);

	/* --- R0 코덱 자체검사 (tb_protocol) 58..63 --- */
	b[58] = tb_proto_done ? 1U : 0U;
	b[59] = 0U;
	put_u16(b, 60, tb_proto_checks);
	put_u16(b, 62, tb_proto_fails);

	return PROTO_LEN_TB_STATE2;
}

/* ==========================================================================
 * 0x33 TB_CTRL 의 R 응답
 * ========================================================================== */
uint16_t TbApp_BuildCtrl(uint8_t *b, uint16_t buf_sz)
{
	if ((b == NULL) || (buf_sz < PROTO_LEN_TB_CTRL))
		return 0U;

	b[0] = g_tb_app.last_item;
	b[1] = g_tb_app.last_result;
	b[2] = g_tb_app.last_nak;
	b[3] = AppMode_IsBenchIdle(g_app_mode) ? 1U : 0U;
	put_u32(b, 4, g_tb_app.last_value);
	put_u32(b, 8, TbApp_Read((tb_item_t)g_tb_app.last_item));

	return PROTO_LEN_TB_CTRL;
}

/* ========================================================================== */
void TbApp_Init(void)
{
	g_tb_app.last_item    = (uint8_t)TB_IT_NONE;
	g_tb_app.last_result  = 0U;
	g_tb_app.last_nak     = (uint8_t)PROTO_NAK_NONE;
	g_tb_app.last_value   = 0U;
	g_tb_app.write_count  = 0U;
	g_tb_app.reject_count = 0U;
}

#endif /* ENABLE_TESTBENCH_APP */
