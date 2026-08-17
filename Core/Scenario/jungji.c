/* ==========================================================================
 * jungji.c - "정지" 단일 처리 모듈 구현. 목록/판단근거는 jungji.h §1~§3 참조.
 *
 * 여기서만 정지가 실행되므로 "무엇을 끄는지"가 한 파일에 모여 있다. 새 부하를
 * 추가하면 jungji.h §1 표와 Jungji_StopAll()의 호출 목록 양쪽을 갱신할 것.
 * ========================================================================== */

#include "jungji.h"

#include "gpio_ctrl.h"
#include "bldc_ctrl.h"
#include "wdoor.h"
#include "tdoor.h"
#include "lift_motor.h"
#include "thermistor.h"

#include "moeum.h"
#include "dongjak.h"
#include "kangeum.h"
#include "baesu.h"

/* 테스트벤치 enable 플래그(정지 시 전부 0으로 눌러 재구동을 막는다) */
#include "tb_drv8871.h"
#include "tb_stepmotor.h"
#include "tb_lift.h"
#include "tb_gpioout.h"
#include "tb_heat.h"
#include "tb_water.h"
#include "tb_speaker.h"

/* defaultTask가 갱신하는 온도 스냅샷(정의: freertos.c). */
extern volatile int16_t g_therm_c_d10[];

JungjiCtx g_jungji;

/* ---- 내부 헬퍼 ----------------------------------------------------------- */

/* 잔열 판정. 써미스터 오류값(THERMISTOR_ERR_D10 = -32768)은 "모름"이므로
 * 안전측(냉각 유지)으로 본다 - 히터를 쓴 직후 센서가 죽었을 수 있다. */
static uint8_t jungji_is_hot(void)
{
	int16_t t = g_therm_c_d10[JUNGJI_TEMP_CH];

	g_jungji.temp_d10 = t;
	if (t == THERMISTOR_ERR_D10)       /* 센서 오류 = "모름" -> 안전측(고온 취급) */
	{
		return 1U;
	}
	return (uint8_t)(t >= (int16_t)JUNGJI_COOL_TEMP_D10);
}

/* 시나리오 중 하나라도 진행 중인가. 진행 중이면 팬 소유권은 그 시나리오에
 * 있으므로 jungji는 냉각 관리에서 손을 뗀다. */
static uint8_t jungji_any_busy(void)
{
	return (uint8_t)(Moeum_IsBusy()   || Dongjak_IsBusy() ||
	                 Kangeum_IsBusy() || Baesu_IsBusy());
}

/* ---- #1,#2 분쇄/교반 BLDC ------------------------------------------------ */
/* EMERGENCY: nBRAKE 단락제동으로 즉시 세우고 JUNGJI_BRAKE_MS 뒤 해제(Tick).
 * NORMAL   : duty 0 + sleep = 코스트(관성으로 서서히 감쇠). */
void Jungji_Bldc(JungjiKind kind, uint32_t now_ms)
{
#if JUNGJI_BLDC_BRAKE_USE
	/* 제동을 다시 걸어야 하는 두 경우:
	 *   - 새 비상정지 요청(kind == EMERGENCY)
	 *   - 이미 제동 유지 중(braking) -> 그 사이 누가 코스트 정지를 걸었을 수
	 *     있으므로 무조건 재확립한다. 실제로 Jungji_Scenarios()의
	 *     Moeum_Abort()/Dongjak_Abort()는 내부에서 BldcCtrl_Stop()(= sleep)을
	 *     부르며, sleep은 nBRAKE 단락제동을 무효화한다. StopAll이 이 함수를
	 *     맨 마지막에 부르는 것과 짝을 이루는 방어다(멱등). */
	if ((kind == JUNGJI_KIND_EMERGENCY) || (g_jungji.braking != 0U))
	{
		BldcCtrl_BrakeStop(&g_grind_ctrl);
		BldcCtrl_BrakeStop(&g_stir_ctrl);
		if (kind == JUNGJI_KIND_EMERGENCY)
		{
			g_jungji.braking     = 1U;
			g_jungji.brake_until = now_ms + (uint32_t)JUNGJI_BRAKE_MS;
		}
		return;
	}
#else
	(void)now_ms;
#endif
	/* 일반 정지: duty 0 + ENABLE LOW(sleep) = 코스트. 관성으로 서서히 멎는다. */
	BldcCtrl_Stop(&g_grind_ctrl);
	BldcCtrl_Stop(&g_stir_ctrl);
}

/* ---- #3,#4 배수문 / 배출문 ----------------------------------------------- */
/* 실제로 문을 그 자리에 붙잡는 것은 DRV8871의 Brake가 아니라 웜기어의 자기유지
 * (비역구동)다 - VM만 내려도 되돌아가지 않는다(drv8871.h 참조). 그래도 Brake를
 * 먼저 거는 이유는 VM이 살아있는 마지막 순간에 저측 단락으로 잔여 회전을
 * 없애기 위함이고, Disable 뒤의 Stop은 유휴 상태에서 IN1/IN2를 아무것도
 * 어서트하지 않은 코스트 패턴으로 되돌려 놓기 위함이다. */
void Jungji_Doors(void)
{
	WDoor_Brake();      /* VM ON 상태에서 저측 단락(잔여 회전 제거)  */
	WDoor_Disable();    /* VM OFF - 여기서부터 웜기어가 위치를 유지   */
	WDoor_Stop();       /* IN1/IN2 코스트로 복귀(유휴 상태 정리)     */

	TDoor_Brake();
	TDoor_Disable();
	TDoor_Stop();
}

/* ---- #5 리프트 ------------------------------------------------------------ */
/* 리프트도 같은 웜기어 구조(U6 DRV8871). 순서/이유는 Jungji_Doors() 참조. */
void Jungji_Lift(void)
{
	Lift_Brake();
	Lift_Disable();
	Lift_Stop();
}

/* ---- #6,#7 스텝모터 STEP1/STEP2 ------------------------------------------ */
/* 4상 전부 LOW = 저측 FET 전부 OFF = 무여자. 홀딩토크가 사라지므로 즉시 정지
 * (동시에 코일 발열도 없다). 핸들 없이 핀만 내리면 되므로 어느 소유자의
 * StepMotor_HandleTypeDef 든 상관없이 안전하게 끌 수 있다. */
void Jungji_Steppers(void)
{
	gpio_ctrl_off(GPIO_OUT_STEP1_M1);
	gpio_ctrl_off(GPIO_OUT_STEP1_M2);
	gpio_ctrl_off(GPIO_OUT_STEP1_M3);
	gpio_ctrl_off(GPIO_OUT_STEP1_M4);

	gpio_ctrl_off(GPIO_OUT_STEP2_M1);
	gpio_ctrl_off(GPIO_OUT_STEP2_M2);
	gpio_ctrl_off(GPIO_OUT_STEP2_M3);
	gpio_ctrl_off(GPIO_OUT_STEP2_M4);
}

/* ---- #8 히터 -------------------------------------------------------------- */
/* 출력만 즉시 끊긴다. 발열체 잔열은 남으므로 냉각팬(#13,#14) 처리를 함께 볼 것.
 * tb_heat도 같이 눌러야 TESTBENCH 모드로 돌아갔을 때 HT-POWER를 다시 잡지 않는다. */
void Jungji_Heater(void)
{
	gpio_ctrl_off(GPIO_OUT_HT_POWER);
	tb_heat_enable    = 0U;
	tb_heat_manual_on = 0U;
}

/* ---- #9~#11 밸브 ---------------------------------------------------------- */
void Jungji_Valves(void)
{
	gpio_ctrl_off(GPIO_OUT_VALVE_DRY_IN);    /* 급수(건조통)      PB13 */
	gpio_ctrl_off(GPIO_OUT_WATER_ON);        /* 급수 메인/펌프    PE2  */
	gpio_ctrl_off(GPIO_OUT_VALVE_DRAIN_CLN); /* 배수구세척        PB14 */
}

/* ---- #12~#14 팬 ----------------------------------------------------------- */
/* keep_cooling = 1 이면 배기팬 + BLDC 냉각팬은 켠 채로 둔다(잔열 배출).
 * tb_* 플래그도 같은 값으로 맞춘다: 정지 후 TESTBENCH 모드에서는
 * TB_GpioOut_Poll()이 매 틱 플래그를 핀에 미러링하므로, 플래그를 0으로 두면
 * 켜 둔 냉각팬을 곧바로 다시 꺼 버린다. */
void Jungji_Fans(uint8_t keep_cooling)
{
	gpio_ctrl_off(GPIO_OUT_FAN_VAPOR);
	tb_fan_vapor_en = 0U;

	if (keep_cooling)
	{
		gpio_ctrl_on(GPIO_OUT_FAN_EXHAUST);
		gpio_ctrl_on(GPIO_OUT_BLDC_FAN);
		tb_fan_exhaust_en = 1U;
		tb_bldc_fan_en    = 1U;
	}
	else
	{
		gpio_ctrl_off(GPIO_OUT_FAN_EXHAUST);
		gpio_ctrl_off(GPIO_OUT_BLDC_FAN);
		tb_fan_exhaust_en = 0U;
		tb_bldc_fan_en    = 0U;
	}
}

/* ---- #15 스피커 ----------------------------------------------------------- */
/* TODO(음성): "처리 중단 / 추가 투입 불가" 안내 멘트는 미구현. 안내음이 생기면
 * 여기서 무음화 대신 안내 재생을 트리거하도록 바꾼다. */
void Jungji_Speaker(void)
{
	tb_speaker_enable = 0U;
	gpio_ctrl_off(GPIO_OUT_EN_SPK);
}

/* ---- #16 테스트벤치 ------------------------------------------------------- */
/* 정지 직후 모드가 TESTBENCH로 돌아가도 벤치 폴러가 부하를 되살리지 못하도록
 * enable 플래그를 전부 내린다. 냉각팬 플래그는 Jungji_Fans()가 뒤에서 다시
 * 세울 수 있으므로 여기서는 건드리되 순서상 Fans()를 나중에 호출한다. */
void Jungji_Testbench(void)
{
	tb_wdoor_enable   = 0U;
	tb_tdoor_enable   = 0U;
	tb_step1_enable   = 0U;
	tb_step2_enable   = 0U;
	tb_lift_enable    = 0U;
	tb_water_enable   = 0U;
	tb_speaker_enable = 0U;

	tb_valve_drain_en = 0U;
	tb_valve_dry_en   = 0U;
	tb_fan_vapor_en   = 0U;
	tb_fan_exhaust_en = 0U;
	tb_bldc_fan_en    = 0U;
}

/* ---- #17 시나리오 FSM ----------------------------------------------------- */
/* 전 시나리오를 즉시 IDLE로. Abort()는 각 시나리오의 abort_req까지 지우므로
 * RequestStop()을 같이 부를 필요가 없다(불러도 곧바로 덮여 무의미하다).
 *
 * 동작 시나리오의 DJ_ABORTED(고온 식힘 상태)를 경유하지 않고 바로 IDLE로
 * 내리는데, 그 대신 잔열 냉각은 jungji가 직접 이어받는다(Jungji_StopAll이
 * cooling 을 세우고 Jungji_Tick이 온도로 팬을 관리). 시나리오에 냉각을
 * 맡기면 모드가 바뀌는 순간 그 시나리오가 리셋되면서 냉각이 끊기기 때문. */
void Jungji_Scenarios(void)
{
	Moeum_Abort();
	Dongjak_Abort();
	Kangeum_Abort();
	Baesu_Abort();
}

/* ---- API ----------------------------------------------------------------- */

void Jungji_Init(void)
{
	g_jungji.req          = 0U;
	g_jungji.req_src      = (uint8_t)JUNGJI_SRC_NONE;
	g_jungji.req_kind     = (uint8_t)JUNGJI_KIND_NORMAL;
	g_jungji.dbg_stop_req = 0U;
	g_jungji.last_src     = (uint8_t)JUNGJI_SRC_NONE;
	g_jungji.last_kind    = (uint8_t)JUNGJI_KIND_NORMAL;
	g_jungji.last_tick    = 0U;
	g_jungji.stop_count   = 0U;
	g_jungji.braking      = 0U;
	g_jungji.brake_until  = 0U;
	g_jungji.cooling      = 0U;
	g_jungji.temp_d10     = 0;
}

void Jungji_Request(JungjiSrc src, JungjiKind kind)
{
	/* 대기 중 요청이 있으면 더 강한 등급이 이긴다(일반 요청이 비상 요청을
	 * 덮어써 제동을 코스트로 떨어뜨리지 않도록). */
	if (g_jungji.req && ((JungjiKind)g_jungji.req_kind == JUNGJI_KIND_EMERGENCY))
	{
		kind = JUNGJI_KIND_EMERGENCY;
	}
	g_jungji.req_src  = (uint8_t)src;
	g_jungji.req_kind = (uint8_t)kind;
	g_jungji.req      = 1U;
}

void Jungji_StopAll(JungjiSrc src, JungjiKind kind, uint32_t now_ms)
{
	uint8_t hot = jungji_is_hot();

	/* ---- 호출 순서가 곧 안전성이다 -------------------------------------
	 * (1) 시나리오 FSM을 먼저 정리한다. Moeum_Abort()/Dongjak_Abort()는 내부에서
	 *     자기 액추에이터를 끄는데, 그 안에 BldcCtrl_Stop()(= sleep, 코스트)이
	 *     들어 있다. 이걸 나중에 부르면 방금 건 단락제동을 풀어 버린다.
	 * (2) 벤치 enable 플래그를 내린다. Jungji_Fans()가 냉각팬 플래그를 다시
	 *     세울 수 있으므로 Fans보다 먼저.
	 * (3) 나머지 부하를 끈다.
	 * (4) 팬은 냉각 여부에 따라 최종값을 쓴다.
	 * (5) BLDC를 마지막에 세운다 -> 이후 어떤 Stop도 제동을 덮지 않는다. */
	Jungji_Scenarios();          /* #17    (반드시 먼저)                    */
	Jungji_Testbench();          /* #16                                     */
	Jungji_Doors();              /* #3,#4  */
	Jungji_Lift();               /* #5     */
	Jungji_Steppers();           /* #6,#7  */
	Jungji_Heater();             /* #8     */
	Jungji_Valves();             /* #9~#11 */
	Jungji_Speaker();            /* #15    */
	Jungji_Fans(hot);            /* #12~#14 (고온이면 냉각팬 유지)          */
	Jungji_Bldc(kind, now_ms);   /* #1,#2  (반드시 마지막)                  */

	g_jungji.cooling    = hot;
	g_jungji.last_src   = (uint8_t)src;
	g_jungji.last_kind  = (uint8_t)kind;
	g_jungji.last_tick  = now_ms;
	if (g_jungji.stop_count < 0xFFFFU) { g_jungji.stop_count++; }
}

void Jungji_Tick(uint32_t now_ms)
{
	/* 디버거 강제 정지(1회성) */
	if (g_jungji.dbg_stop_req != 0U)
	{
		g_jungji.dbg_stop_req = 0U;
		Jungji_Request(JUNGJI_SRC_DEBUG, JUNGJI_KIND_EMERGENCY);
	}

	/* 요청 소비 */
	if (g_jungji.req != 0U)
	{
		JungjiSrc  src  = (JungjiSrc)g_jungji.req_src;
		JungjiKind kind = (JungjiKind)g_jungji.req_kind;

		g_jungji.req = 0U;
		Jungji_StopAll(src, kind, now_ms);
	}

	/* 단락제동 홀드 해제: 유지 시간이 지나면 nBRAKE를 풀고 드라이버를 재운다.
	 * 이 구간이 끝나야 중재자가 새 시나리오를 시작한다(ModeArbiter_MotorTick). */
	if (g_jungji.braking != 0U)
	{
		if ((uint32_t)(now_ms - g_jungji.brake_until) < 0x80000000UL)
		{
			BldcCtrl_BrakeRelease(&g_grind_ctrl);
			BldcCtrl_BrakeRelease(&g_stir_ctrl);
			g_jungji.braking = 0U;
		}
	}

	/* 잔열 냉각: 정지 후 아무 시나리오도 돌지 않는 동안만 jungji가 팬을 쥔다.
	 * 새 시나리오가 시작되면 팬 소유권을 넘기고 손을 뗀다. */
	if (g_jungji.cooling != 0U)
	{
		if (jungji_any_busy())
		{
			g_jungji.cooling = 0U;       /* 소유권 이양(팬 상태는 시나리오가 결정) */
		}
		else if (!jungji_is_hot())
		{
			Jungji_Fans(0U);             /* 충분히 식음 -> 냉각팬 OFF              */
			g_jungji.cooling = 0U;
		}
		else
		{
			Jungji_Fans(1U);             /* 계속 유지(플래그 미러링 대비 매 틱 재확인) */
		}
	}
}

uint8_t Jungji_IsBraking(void)
{
	return g_jungji.braking;
}

uint8_t Jungji_IsCooling(void)
{
	return g_jungji.cooling;
}
