#ifndef SCENARIO_MOEUM_H_
#define SCENARIO_MOEUM_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * moeum - "모음"(1단계 헹굼 세척) 시나리오 상태머신.
 *
 * 근거: Core/Scenario/사용자_시나리오_검토정리.md
 *   - §4.1   모음 기능(헹굼 세척) 순서
 *   - §4.1.1 모음 펌웨어 제어 시퀀스(하드웨어 매핑, R1 넷리스트/260806 핀맵)
 * 관련 HW 미확정: Core/Scenario/하드웨어결정_잔여항목.md (§1-1 수위/전류, §1-2 홀,
 *                §2-1 도어 홀 극성). 아래 placeholder 상수는 벤치 확정 후 갱신.
 *
 * 시퀀스(락 미SET=정상 전제):
 *   1) HS2(모음) 눌림 감지 → 시작
 *   2) 배수문(U5, WDoor) 닫힘 구동             [MotorTick]
 *   3) 배수문 닫힘 인식(W-HALL-CLOSE, PF3) → 모터 정지
 *   4) 급수 솔밸브 ON (VALVE-DRY-IN, PB13, 최대)
 *   5) 수위센서(PF6/PF7) 감지 → 2초 추가급수 → 밸브 OFF
 *   6) 교반 BLDC(M2, U16) ~25RPM: CCW 3s / 정지 1s / CW 3s = 1cycle 7s x 22 (~2m30s)
 *   7) 교반 지속 중 배수문 열림 구동            [MotorTick]
 *   8) 배수문 열림 인식(W-HALL-OPEN, PF4) → 도어 모터 정지
 *   9) 배수문 열림 후 약 1분 뒤 교반 정지 → 배수문 열린 상태로 종료
 *
 * 태스크 배치(freertos.c):
 *   Moeum_SenseTick()  - StartDefaultTask(100ms). 센서만: 시작(HS2) 감지, 수위 감지.
 *   Moeum_MotorTick()  - StartMotorTask(1ms). 모터/밸브 구동 + 상태 전이 소유(single owner).
 * 교반 폐루프는 moeum이 BldcCtrl_Start/Stop로 지령만 내리고, 실제 PI 실행을 위해
 * 호출자가 BldcCtrl_Tick(&g_stir_ctrl, now)를 매 틱 함께 호출해야 한다.
 *
 * 테스트벤치와 g_stir_ctrl / WDoor 를 공유하므로 동시 실행 불가 -> freertos.c에서
 * 런타임 모드(g_app_mode)로 배타 선택한다.
 * ========================================================================== */

/* ---- 튜닝 상수 (벤치 확정 전 placeholder 포함) --------------------------- */
#ifndef MOEUM_DOOR_DUTY_PCT
#define MOEUM_DOOR_DUTY_PCT     50U      /* 배수문 PWM 힘(docx: 50%)          */
#endif
#ifndef MOEUM_STIR_OUT_RPM
#define MOEUM_STIR_OUT_RPM      25U      /* 교반 출력축 RPM(약 25)            */
#endif
#ifndef MOEUM_STIR_CCW_MS
#define MOEUM_STIR_CCW_MS       3000U    /* CCW 구간                          */
#endif
#ifndef MOEUM_STIR_DELAY_MS
#define MOEUM_STIR_DELAY_MS     1000U    /* 정지(딜레이) 구간                 */
#endif
#ifndef MOEUM_STIR_CW_MS
#define MOEUM_STIR_CW_MS        3000U    /* CW 구간                           */
#endif
#ifndef MOEUM_STIR_CYCLES
#define MOEUM_STIR_CYCLES       22U      /* 7s x 22 = 154s(~2m30s) 헹굼       */
#endif
#ifndef MOEUM_FILL_EXTRA_MS
#define MOEUM_FILL_EXTRA_MS     2000U    /* 수위 감지 후 2초 추가 급수        */
#endif
/* 급수 시 WATER_ON(PE2, 급수 펌프/메인 밸브 enable)을 VALVE_DRY_IN(PB13)과
 * 함께 ON. 근거: 수위센서 HW 검증이 PE2 ON 상태(tb_water)에서 통수·감지되었고,
 * PB13 단독 통수는 미확인. 벤치에서 PB13 단독으로 물이 나오면 0으로 두면 된다. */
#ifndef MOEUM_FILL_USE_WATER_ON
#define MOEUM_FILL_USE_WATER_ON 1
#endif
#ifndef MOEUM_DRAIN_STIR_MS
#define MOEUM_DRAIN_STIR_MS     60000U   /* 배수문 열림 후 1분 뒤 교반 정지   */
#endif
#ifndef MOEUM_DOOR_TIMEOUT_MS
#define MOEUM_DOOR_TIMEOUT_MS   15000U   /* 도어 리미트 미도달 안전 타임아웃  */
#endif
/* 도어 리미트(PF3/PF4) 처리 모드.
 *   0 = 정상/스펙: 리미트 인식으로만 다음 단계 진행, 미도달 시 MOEUM_DOOR_TIMEOUT_MS
 *       후 ERROR (W-HALL-CLOSE/OPEN 센서 게이팅). ← 실동작/검증 기본.
 *   1 = 벤치 육안 전용: 방향 미확정 등으로 리미트 미도달 시에도 ERROR 대신
 *       MOEUM_DOOR_BENCH_MS 만큼만 구동 후 진행(센서 무시). 모터 회전 관찰용 임시. */
#ifndef MOEUM_DOOR_LIMIT_OPTIONAL
#define MOEUM_DOOR_LIMIT_OPTIONAL 0
#endif
#ifndef MOEUM_DOOR_BENCH_MS
#define MOEUM_DOOR_BENCH_MS     4000U    /* (육안모드) 리미트 무시 시 도어 구동 관찰 시간 */
#endif
#ifndef MOEUM_FILL_TIMEOUT_MS
#define MOEUM_FILL_TIMEOUT_MS   120000U  /* 급수 안전 타임아웃(에러기준 미정) */
#endif
/* HS 채널 의미는 코드에 아직 미인코딩(하드웨어결정 §1-2). docx 가정: HS2=모음.
 * 0-based 인덱스이므로 HS2 = idx 1. 확정 후 값만 조정. */
#ifndef MOEUM_HS_START_IDX
#define MOEUM_HS_START_IDX      1U
#endif
/* 수위 감지 방식(폴리시 미확정, 하드웨어결정 §1-1).
 * 기본은 EXTI 하강에지(플래그)만 사용 - 유휴 극성이 확정 전이라 레벨 폴백은
 * 즉시 오검출(급수 스킵) 위험이 있어 opt-in으로 둔다. 벤치 확정 후 활성화.
 *   MOEUM_WATER_USE_LEVEL  1 -> 레벨 폴백도 함께 사용
 *   MOEUM_WATER_ACTIVE_LOW 1 -> (레벨 사용 시) LOW를 물 감지로 판정 */
#ifndef MOEUM_WATER_USE_LEVEL
#define MOEUM_WATER_USE_LEVEL   0
#endif
#ifndef MOEUM_WATER_ACTIVE_LOW
#define MOEUM_WATER_ACTIVE_LOW  1
#endif

/* ---- 상태 (디버거 관찰용) ----------------------------------------------- */
typedef enum
{
	MOEUM_IDLE = 0,     /* 대기: 시작 요청 기다림                            */
	MOEUM_DOOR_CLOSE,   /* 배수문 닫힘 구동 -> W-HALL-CLOSE 대기             */
	MOEUM_FILL,         /* 급수밸브 ON -> 수위 감지 대기                     */
	MOEUM_FILL_EXTRA,   /* 수위 감지 후 2초 추가급수 -> 밸브 OFF            */
	MOEUM_STIR,         /* 교반 CCW/정지/CW 22 cycle 헹굼                   */
	MOEUM_DRAIN_OPEN,   /* (교반 지속) 배수문 열림 구동 -> W-HALL-OPEN 대기 */
	MOEUM_DRAIN_WAIT,   /* (교반 지속) 열림 후 1분 대기 -> 교반 정지        */
	MOEUM_DONE,         /* 완료: 배수문 열린 상태로 정지                     */
	MOEUM_ERROR         /* 타임아웃 등 이상 -> 안전 정지                     */
} MoeumState;

/* 교반 서브-FSM 위상 */
typedef enum
{
	MOEUM_STIR_CCW = 0,
	MOEUM_STIR_DELAY,
	MOEUM_STIR_CW
} MoeumStirPhase;

typedef struct
{
	volatile MoeumState  state;         /* MoeumState (RO 관찰)                  */
	uint32_t          state_since;   /* 현 상태 진입 tick                     */

	volatile uint8_t  start_req;     /* SenseTick가 세팅, MotorTick가 소비    */
	volatile uint8_t  dbg_force_start;/* 벤치 강제 시작: 1 세팅 -> HS2 없이 시작(SenseTick가 1회 소비 후 0). HS2 상승에지와 OR. */
	volatile uint8_t  water_reached; /* SenseTick가 세팅, FILL진입 시 클리어  */
	uint8_t           hs_prev;       /* HS2 에지 검출용 이전값                */

	/* 교반 서브-FSM */
	uint8_t           stir_active;   /* 1 = 교반 사이클 구동 중               */
	uint8_t           stir_phase;    /* MoeumStirPhase                        */
	uint32_t          stir_phase_since;
	uint16_t          stir_cycles;   /* 완료한 CCW+delay+CW 사이클 수         */

	uint32_t          drain_open_since; /* W-HALL-OPEN 인식 tick              */
} MoeumCtx;

extern MoeumCtx g_moeum;             /* 디버거 관찰/제어용 단일 인스턴스      */

/* ---- API ----------------------------------------------------------------- */
void       Moeum_Init(void);                 /* 상태/모터/밸브 초기화(IDLE)   */
void       Moeum_Start(void);                /* 시작 요청(IDLE에서만 수락)    */
void       Moeum_Abort(void);                /* 즉시 안전정지 -> IDLE         */
void       Moeum_SenseTick(void);            /* 100ms, StartDefaultTask       */
void       Moeum_MotorTick(uint32_t now_ms); /* 1ms, StartMotorTask           */
MoeumState Moeum_GetState(void);
uint8_t    Moeum_IsBusy(void);               /* 1 = IDLE/DONE/ERROR 이 아님   */

#ifdef __cplusplus
}
#endif

#endif /* SCENARIO_MOEUM_H_ */
