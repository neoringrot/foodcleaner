#ifndef SRC_TB_LIFT_H_
#define SRC_TB_LIFT_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Testbed for the U6 Lift DRV8871 (schematic net "LIFT").
 *
 * R1 이후: 리프트 IN1/IN2 는 PG3/PG4(타이머 없음) 에서 **PB8/PB9 = TIM4_CH3/CH4**
 * 로 옮겨졌고, 드라이버(`lift_motor.c`)는 도어(U5/U7)와 같은 DRV8871 20 kHz
 * 하드웨어 PWM 엔진을 쓴다. 따라서 이 벤치의 **소프트웨어 PWM 은 폐지**됐다 --
 * 속도는 `Lift_UpSpeed()/Lift_DownSpeed()` 로 HW PWM duty 를 직접 지령한다.
 * (`tb_lift_pwm_period_ms` 는 그래서 사라졌다.)
 *
 * 구조는 도어 벤치(`tb_drv8871`)와 같은 규약을 따른다:
 *   - 구동 수치의 단일 출처는 드라이버 헤더 `lift_motor.h`(LIFT_*). 아래 tb_*
 *     변수는 그 값으로 초기화되는 **런타임 실험용 복사본**이다.
 *   - 프로파일 ON 이면 Poll() 이 적용 duty 를 tb_lift_duty 에 **써 넣는다**
 *     (읽기전용 표시). 수동 duty 를 쓰려면 tb_lift_profile = 0.
 *   - 런은 tb_lift_enable 의 0->1 엣지에서 시작하고, 방향은 시작 시점에
 *     래치된다. 런 중 tb_lift_reverse 를 뒤집으면 **새 런으로 재시작**한다.
 *   - 종료(리미트/시간상한)되면 코스트 + VM off 후 tb_lift_enable 을 스스로
 *     0 으로 되돌린다(스테퍼 벤치와 같은 원샷 성격).
 *
 * ★ 동작 범위 센서 (REV02 넷리스트 확인 결과):
 *     하단 = HS8 (U24 P7, J30) 하나뿐이다. **상단 리미트는 회로에 없다.**
 *     그래서 상승은 tb_lift_up_max_ms 시간 상한으로만 끝난다 -- 상한을 크게
 *     잡으면 기구 끝단에 물린 채 계속 밀게 되므로 주의할 것.
 *     (J16/PF7 `NEW-HALL-INT` 가 미배정 홀 입력으로 남아 있지만 용도는 N9
 *      회신 대기이고 가이드 홀로 추정된다 -- 리프트 상단으로 확정된 바 없다.)
 *
 * 사용법: MotorTask 가 1 ms 로 TB_Lift_Poll() 을 돌린다(벤치 모드에서만).
 *     TB_Lift_Init();
 *     for(;;) { TB_Lift_Poll(); osDelay(1); }
 * 또는 TB_Lift_Loop();
 *
 * ✅ 방향 라벨 **확정 [2026-09-20 벤치]**: tb_lift_reverse **0 = 상승 / 1 = 하강**.
 * 드라이버의 임시 매핑(Forward=상승)이 그대로 맞아 `lift_motor.c` 는 스왑하지
 * 않았다. 따라서 tb_lift_down_reverse 의 기본값 1 이 실제와 일치한다 -- 이 변수는
 * 이제 보정용이 아니라 "하강이 어느 쪽인지"의 기록으로 남는다(바꾸지 말 것). */

/* ---- 구동 스위치 ------------------------------------------------------ */
extern volatile uint8_t  tb_lift_enable;   /* 1 = 구동, 0 = 정지(코스트+VM off) */
extern volatile uint8_t  tb_lift_reverse;  /* 0 = 상승, 1 = 하강 (확정 2026-09-20) */
extern volatile uint8_t  tb_lift_duty;     /* 0..100 % (프로파일 ON 이면 표시용) */

/* ---- 프로파일(기본 ON) ------------------------------------------------
 * 값은 lift_motor.h 의 LIFT_* 에서 온다. profile = 0 으로 두면 duty 는 수동,
 * 시간 상한도 적용되지 않는다(자유 구동). HS8 자동정지는 프로파일과 무관하게
 * tb_lift_limit_stop 이 따로 관장한다. */
extern volatile uint8_t  tb_lift_profile;      /* 1 = 프로파일, 0 = 수동    */
extern volatile uint8_t  tb_lift_run_duty;     /* LIFT_DUTY (80 %)          */
extern volatile uint16_t tb_lift_up_max_ms;    /* 상승 상한 (센서 없음!)    */
extern volatile uint16_t tb_lift_down_max_ms;  /* 하강 상한 (HS8 백스톱)    */

/* ---- 하단 리미트 ------------------------------------------------------
 * 1 이면 **하강 방향에서만** HS8(리프트 하단)을 보고, TB_LIFT_LIMIT_CONFIRM 회
 * 연속으로 잡히면 코스트 + VM off + enable 0 + tb_lift_limit_hit = 1 로 끝낸다.
 * 상승 방향은 센서가 없으므로 게이트 대상이 아니다 -- 하단에 앉은 상태에서
 * 상승 명령은 그대로 나간다(도어와 같은 방향별 게이트).
 * 0 이면 자유 구동: 기구 끝단에 물려도 시간 상한까지 계속 민다. */
extern volatile uint8_t  tb_lift_limit_stop;    /* 기본 1                    */
extern volatile uint8_t  tb_lift_down_reverse;  /* 하강 = reverse 1 (확정)    */

/* ---- 상태 (읽기전용) --------------------------------------------------
 * limit_hit / time_hit 는 **래치**다 -- 다음 런이 시작될 때(enable 0->1 엣지,
 * 또는 런 중 방향 반전)에만 지워진다. 센서에서 벗어나도 1 을 유지하는 것이
 * 정상이다: 종료 원인을 나중에 읽으려고 남기는 값이기 때문이다.
 * (도어 벤치의 tb_*_limit_hit 과 같은 규약. 2026-09-20 벤치에서 확인됨.)
 * 반면 at_bottom 은 래치가 아니라 **매 poll 갱신되는 현재 레벨**이라 구동 중이
 * 아니어도 센서를 따라 0/1 로 움직인다 -- 정지 원인의 증거로 쓰면 안 된다.
 *
 * 하단에 앉은 채 다시 하강을 걸면: 새 런이 시작되며 limit_hit 가 0 으로 지워지고,
 * 확인창(5 ms) 뒤 다시 1 이 되며 멈춘다(짧게 움찔). 상승은 게이트 대상이 아니라
 * 그대로 빠져나온다. */
extern volatile uint8_t  tb_lift_limit_hit;  /* 1 = HS8 인식으로 종료(래치)  */
extern volatile uint8_t  tb_lift_time_hit;   /* 1 = 시간 상한으로 종료(래치) */
extern volatile uint8_t  tb_lift_at_bottom;  /* HS8 현재 레벨(래치 아님)     */
extern volatile uint32_t tb_lift_last_run_ms;/* 마지막 런 길이 [ms]          */

void TB_Lift_Init(void);
void TB_Lift_Poll(void);   /* apply current enable/direction/duty state once */
void TB_Lift_Loop(void);   /* Init already done; poll forever                */

#ifdef __cplusplus
}
#endif

#endif /* SRC_TB_LIFT_H_ */
