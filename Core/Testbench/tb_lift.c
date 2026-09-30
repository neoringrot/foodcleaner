#include "tb_lift.h"

#include "cmsis_os.h"
#include "lift_motor.h"

/* 런타임 스위치 -- 디버거에서 바꾼다. 기본값은 정지/안전.
 *
 * R1 반영: 예전 이 파일은 PG3/PG4 에 타이머 채널이 없다는 이유로 1 ms 폴 주기를
 * 잘라 쓰는 **소프트웨어 PWM** 을 돌렸다. R1 에서 리프트가 PB8/PB9(TIM4_CH3/CH4)
 * 로 옮겨져 도어와 같은 20 kHz HW PWM 을 쓰게 됐으므로 그 구조는 폐지했다.
 * (남겨두면 100 % duty HW PWM 위에 100 Hz 소프트 초핑이 겹치는 이중 변조가 된다.)
 * 이제 duty 는 Lift_UpSpeed()/Lift_DownSpeed() 로 그대로 내려간다.
 *
 * 수치 출처는 lift_motor.h 의 LIFT_* 뿐이다 -- 도어(wdoor.h/tdoor.h)와 같은 규약.
 * ⚠️ 단, 도어와 달리 LIFT_* 는 실측값이 아니라 잠정값(C071 미정의)이다. */
volatile uint8_t  tb_lift_enable  = 0;
volatile uint8_t  tb_lift_reverse = 0;   /* 0 = 상승, 1 = 하강 (확정 2026-09-20) */
volatile uint8_t  tb_lift_duty    = LIFT_DUTY;

volatile uint8_t  tb_lift_profile     = 1;
volatile uint8_t  tb_lift_run_duty    = LIFT_DUTY;
volatile uint16_t tb_lift_up_max_ms   = LIFT_UP_MAX_MS;
volatile uint16_t tb_lift_down_max_ms = LIFT_DOWN_MAX_MS;

/* 하단(HS8) 자동정지. 리프트에 있는 유일한 행정 센서라 기본 ON 이다.
 * tb_lift_down_reverse 는 "tb_lift_reverse 의 어느 값이 하강인가"다. 2026-09-20
 * 벤치에서 **1 = 하강** 으로 확정됐고(기본값과 일치), 하강 중 HS8 인식으로 정지하는
 * 것까지 확인했다. 방향을 바꿀 일이 생기면 이 변수가 아니라 lift_motor.c 를 고친다. */
volatile uint8_t  tb_lift_limit_stop   = 1;
volatile uint8_t  tb_lift_down_reverse = 1;

/* 상태(읽기전용) */
volatile uint8_t  tb_lift_limit_hit   = 0;
volatile uint8_t  tb_lift_time_hit    = 0;
volatile uint8_t  tb_lift_at_bottom   = 0;
volatile uint32_t tb_lift_last_run_ms = 0;

/* VM 을 전이에서만 토글하기 위한 이전 상태. */
static uint8_t lift_running = 0;

/* 현재 출력(0 = 코스트, 1 = Up, 2 = Down)과 그때의 duty. 변화가 있을 때만
 * CCR 을 다시 쓴다. */
static uint8_t lift_applied      = 0;
static uint8_t lift_applied_duty = 0;

/* 이번 런의 시작 tick 과 시작 시 래치된 방향. 런 중 방향을 뒤집으면 새 런이다. */
static uint32_t lift_start_ms = 0;
static uint8_t  lift_dir      = 0;

/* HS8 디바운스 누산기. Poll() 이 1 ms 이므로 ~5 ms 확인창이다 -- 도어(U5/U7)의
 * TB_DOOR_LIMIT_CONFIRM 과 같은 근거. */
#ifndef TB_LIFT_LIMIT_CONFIRM
#define TB_LIFT_LIMIT_CONFIRM 5U
#endif
static uint8_t lift_confirm = 0;

static uint8_t duty_clamp(uint8_t d)
{
	return (d > 100U) ? 100U : d;
}

/* 코스트 + VM off + 런 종료. 스테퍼 벤치처럼 enable 을 펌웨어가 0 으로 되돌린다. */
static void lift_finish(uint32_t now)
{
	Lift_Stop();
	Lift_Disable();
	tb_lift_last_run_ms = (uint32_t)(now - lift_start_ms);
	tb_lift_enable      = 0U;
	lift_running        = 0U;
	lift_applied        = 0U;
	lift_applied_duty   = 0U;
	lift_confirm        = 0U;
}

void TB_Lift_Init(void)
{
	/* Lift_Init()(VM off, 코스트)은 main.c USER CODE BEGIN 2 에서 스케줄러보다
	 * 먼저 수행된다. 여기서는 런 상태만 초기화한다. */
	lift_running        = 0;
	lift_applied        = 0;
	lift_applied_duty   = 0;
	lift_start_ms       = 0;
	lift_dir            = 0;
	lift_confirm        = 0;
	tb_lift_limit_hit   = 0;
	tb_lift_time_hit    = 0;
	tb_lift_at_bottom   = 0;
	tb_lift_last_run_ms = 0;
}

void TB_Lift_Poll(void)
{
	uint32_t now = HAL_GetTick();
	uint8_t  at_bottom = Lift_AtBottom();

	tb_lift_at_bottom = at_bottom;

	if (tb_lift_enable)
	{
		uint8_t  duty;
		uint8_t  desired;
		uint32_t elapsed;

		/* 런 시작(0->1) 또는 런 중 방향 반전 -> 새 런. */
		if (!lift_running || (tb_lift_reverse != lift_dir))
		{
			if (!lift_running) { Lift_Enable(); }   /* VM 먼저 */
			lift_start_ms     = now;
			lift_dir          = tb_lift_reverse;
			lift_applied      = 0U;   /* 첫 기록을 강제 */
			lift_applied_duty = 0U;
			lift_confirm      = 0U;
			lift_running      = 1U;
			tb_lift_limit_hit = 0U;
			tb_lift_time_hit  = 0U;
		}

		elapsed = (uint32_t)(now - lift_start_ms);

		/* 하단 리미트: 하강 방향에서만 본다. 상단에는 센서가 없다. */
		if (tb_lift_limit_stop && (lift_dir == tb_lift_down_reverse))
		{
			if (at_bottom)
			{
				if (lift_confirm < (uint8_t)TB_LIFT_LIMIT_CONFIRM) { lift_confirm++; }
			}
			else
			{
				lift_confirm = 0U;
			}

			if (lift_confirm >= (uint8_t)TB_LIFT_LIMIT_CONFIRM)
			{
				tb_lift_limit_hit = 1U;
				lift_finish(now);
				return;
			}
		}
		else
		{
			lift_confirm = 0U;
		}

		/* 시간 상한: 프로파일 ON 에서만. 상승은 이것이 유일한 종료 조건이다. */
		if (tb_lift_profile)
		{
			uint16_t cap = (lift_dir == tb_lift_down_reverse) ? tb_lift_down_max_ms
			                                                  : tb_lift_up_max_ms;
			if ((cap != 0U) && (elapsed >= (uint32_t)cap))
			{
				tb_lift_time_hit = 1U;
				lift_finish(now);
				return;
			}
			duty          = duty_clamp(tb_lift_run_duty);
			tb_lift_duty  = duty;   /* 적용 duty 를 표시용으로 되써준다 */
		}
		else
		{
			duty = duty_clamp(tb_lift_duty);
		}

		/* 출력 확정. duty 0 은 코스트다. */
		desired = (duty == 0U) ? 0U : (lift_dir ? 2U : 1U);
		if ((desired != lift_applied) || (duty != lift_applied_duty))
		{
			switch (desired)
			{
			case 1U:  Lift_UpSpeed(duty);   break;
			case 2U:  Lift_DownSpeed(duty); break;
			default:  Lift_Stop();          break;   /* 코스트 (VM 은 유지) */
			}
			lift_applied      = desired;
			lift_applied_duty = duty;
		}
	}
	else if (lift_running)
	{
		/* 사용자가 enable 을 내렸다(정지 포함, jungji 가 이 플래그를 0 으로 쓴다). */
		lift_finish(now);
	}
}

void TB_Lift_Loop(void)
{
	for (;;)
	{
		TB_Lift_Poll();
		osDelay(1);
	}
}
