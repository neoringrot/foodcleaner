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
 *   1) HS5(모음) 눌림 감지 → 시작
 *   2) 배수문(U5, WDoor) 닫힘 구동             [MotorTick]
 *   3) 배수문 닫힘 인식(W-HALL-CLOSE, PF3) → 모터 정지
 *   4) 급수 솔밸브 ON (VALVE-DRY-IN, PB13, 최대)
 *   5) 수위센서(PF6/PF7) 감지 → 2초 추가급수 → 밸브 OFF
 *   6) 교반 BLDC(M2, U16) ~30RPM: CW 6s / 정지 1s / CCW 6s = 1cycle 13s x 11 (143s)
 *   7) 교반 지속 중 배수문 열림 구동            [MotorTick]
 *   8) 배수문 열림 인식(W-HALL-OPEN, PF4) → 도어 모터 정지
 *   9) 배수문 열림 후 30초 뒤 교반 정지 → 배수문 열린 상태로 종료
 *
 * 태스크 배치(freertos.c):
 *   Moeum_SenseTick()  - StartDefaultTask(100ms). 센서만: 시작(HS5) 감지, 수위 감지.
 *   Moeum_MotorTick()  - StartMotorTask(1ms). 모터/밸브 구동 + 상태 전이 소유(single owner).
 * 교반 폐루프는 moeum이 BldcCtrl_Start/Stop로 지령만 내리고, 실제 PI 실행을 위해
 * 호출자가 BldcCtrl_Tick(&g_stir_ctrl, now)를 매 틱 함께 호출해야 한다.
 *
 * 테스트벤치와 g_stir_ctrl / WDoor 를 공유하므로 동시 실행 불가 -> freertos.c에서
 * 런타임 모드(g_app_mode)로 배타 선택한다.
 * ========================================================================== */

/* ---- 튜닝 상수 (벤치 확정 전 placeholder 포함) --------------------------- */
/* [삭제 2026-08-25] 배수문의 duty/구동시간/리미트 처리는 전부 wdoor.h의 WDOOR_*
 * 프로파일이 단일 출처다(테스트벤치 tb_drv8871과 동일 값·동일 규칙). 여기 있던
 * MOEUM_DOOR_DUTY_PCT / MOEUM_DOOR_TIMEOUT_MS / MOEUM_DOOR_LIMIT_OPTIONAL /
 * MOEUM_DOOR_BENCH_MS 는 참조처가 없는 채로 남아 "여기를 고치면 된다"고 오해를
 * 주기에 지웠다. 도어 동작을 바꾸려면 wdoor.h 를 고칠 것.
 *   닫힘 = Forward, WDOOR_CLOSE_DUTY(80%), WDOOR_CLOSE_MS(4.2s) 경과가 정상 종료
 *   열림 = Reverse, 킥 80%(2s) -> 65%, WHALL-OPEN 인식이 정상 종료(상한 6s)
 * 어느 쪽이든 리미트 미인식은 ERROR 가 아니라 시간으로 종료한다 - 미인식 자체는
 * 앱 검증표의 '배수문 닫힘/열림 인식' 항목이 잡는다. */
/* ★교반 패턴 이력: 25RPM CW3/정지1/CCW3(1cycle 7s)
 *   -> 2026-08-25 30RPM CW4/정지1/CCW4(1cycle 9s)
 *   -> 2026-08-26 30RPM CW6/정지1/CCW6(1cycle 13s)  ← 현재
 * 동작 시나리오의 1·2차 교반 헹굼(dongjak.h DJ_RINSE_*)도 같은 구간을 쓴다.
 * RPM 은 30 유지. */
#ifndef MOEUM_STIR_OUT_RPM
#define MOEUM_STIR_OUT_RPM      30U      /* 교반 출력축 RPM(약 30)            */
#endif
#ifndef MOEUM_STIR_CCW_MS
#define MOEUM_STIR_CCW_MS       6000U    /* CCW 구간 (사이클의 3번째)         */
#endif
#ifndef MOEUM_STIR_DELAY_MS
#define MOEUM_STIR_DELAY_MS     1000U    /* 정지(딜레이) 구간                 */
#endif
#ifndef MOEUM_STIR_CW_MS
#define MOEUM_STIR_CW_MS        6000U    /* CW 구간 (사이클의 1번째)          */
#endif
#ifndef MOEUM_STIR_CYCLES
#define MOEUM_STIR_CYCLES       11U      /* 1cycle = CW6s+정지1s+CCW6s = 13s.
                                          * ★2026-08-26 기준: 목표 총시간
                                          * 2분30초(150s)에 '사이클을 내려서'
                                          * 맞춘다 -> floor(150/13) = 11.
                                          * 13s x 11 = 143s(2m23s).
                                          * 12사이클이면 156s 로 150s 를 넘기므로
                                          * 내림이 맞다. 사이클 길이를 바꾸면
                                          * 이 값도 floor(150000/1cycle) 로
                                          * 다시 계산할 것. */
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
#define MOEUM_DRAIN_STIR_MS     30000U   /* 배수문 열림 후 30초 뒤 교반 정지  */
                                         /* ★2026-08-26: 1분 -> 30초 변경.  */
#endif
#ifndef MOEUM_FILL_TIMEOUT_MS
#define MOEUM_FILL_TIMEOUT_MS   1800000U /* 수위 감지 대기 타임아웃 30분.
                                          * 급수(MOEUM_FILL) 단계가 곧 '수위센서가 잡힐
                                          * 때까지 기다리는' 단계이므로 이 상수 하나가
                                          * 둘의 타임아웃이다.
                                          * 2분 -> 5분 -> 30분으로 늘려 왔다. 이 값은
                                          * '고장 판정' 기준이지 정상 급수시간이 아니다
                                          * (정상 종료는 water_reached 로 한다).
                                          * 넉넉해야 하는 이유: 유량·초기수위에 따라
                                          * 감지까지 걸리는 시간이 크게 달라지는데,
                                          * 짧으면 정상 급수 중에도 MOEUM_ERROR 로
                                          * 빠진다(실측). */
#endif
/* HS5 = 모음 (U24 P4, 0-based 이므로 idx 4). tb_hallsensor.h의 P0..P4 매핑과 동일.
 * ★2026-08-25: 마개 라벨 순서 재정의로 모음이 HS2 -> HS5 로 옮겨졌다(동작과 맞교환).
 *   배선은 그대로이고 라벨만 바뀐 것이라 이 인덱스와 mode_arbiter.c 디코드 표만
 *   반대로 잡으면 된다. */
#ifndef MOEUM_HS_START_IDX
#define MOEUM_HS_START_IDX      4U
#endif
/* 시작 트리거(HS5 상승에지)를 이 파일 안에서 볼지 여부.
 *   0 = 기본. 중재자(mode_arbiter.c)가 HS1~5를 항상 디코딩해 모드 전환과
 *       Moeum_Start()를 담당한다. SenseTick가 모드 게이팅되어 있어 hs_prev가
 *       얼어붙던 문제(모드 진입 시 가짜 상승에지 / stale HIGH로 에지 누락)가
 *       구조적으로 사라진다. 마개 이탈 감시(lid_guard)도 중재자의 HS_LOST가
 *       담당하므로 함께 비활성.
 *   1 = 구(舊) 동작. 중재자를 쓰지 않고 이 시나리오가 직접 HS5를 볼 때만.
 * 어느 쪽이든 dbg_force_start(벤치 강제 시작)는 항상 유효하다. */
#ifndef MOEUM_HS_TRIGGER_INTERNAL
#define MOEUM_HS_TRIGGER_INTERNAL 0
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

/* ---- 4.5 추가 투입 금지 / 비상 정지 (기획서 4.5, 동작·모음 공통) ------- *
 * 처리 중 투입구 마개가 모음 위치를 벗어나면(=시작 홀 HS5 이탈) 즉시 전 모터/밸브
 * 정지 후 IDLE(모음은 히터 없어 식힘 불필요). 벤치 강제시작은 무장하지 않음. */
#ifndef MOEUM_LID_OPEN_ABORT
#define MOEUM_LID_OPEN_ABORT     1        /* 1=투입구 개방 비상정지 사용        */
#endif
#ifndef MOEUM_LID_CONFIRM_SAMPLES
#define MOEUM_LID_CONFIRM_SAMPLES 3U      /* HS LOW 연속 N회(≈300ms) 후 정지    */
#endif

/* ---- 상태 (디버거 관찰용) ----------------------------------------------- */
typedef enum
{
	MOEUM_IDLE = 0,     /* 대기: 시작 요청 기다림                            */
	MOEUM_DOOR_CLOSE,   /* 배수문 닫힘 구동 -> W-HALL-CLOSE 대기             */
	MOEUM_FILL,         /* 급수밸브 ON -> 수위 감지 대기                     */
	MOEUM_FILL_EXTRA,   /* 수위 감지 후 2초 추가급수 -> 밸브 OFF            */
	MOEUM_STIR,         /* 교반 CW/정지/CCW 22 cycle 헹굼                   */
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
	volatile uint8_t  dbg_force_start;/* 벤치 강제 시작: 1 세팅 -> HS5 없이 시작(SenseTick가 1회 소비 후 0). HS5 상승에지와 OR. */
	volatile uint8_t  water_reached; /* SenseTick가 세팅, FILL진입 시 클리어  */
	uint8_t           hs_prev;       /* HS5 에지 검출용 이전값                */

	/* 4.5 비상정지 */
	volatile uint8_t  abort_req;     /* 정지 요청(SenseTick/RequestStop 세팅) */
	volatile uint8_t  dbg_force_stop;/* 벤치 강제 정지(1회성) = 정지버튼 모사  */
	uint8_t           lid_guard;     /* 1=실제 HS 시작 -> 투입구 개방 감시 무장 */
	uint8_t           lid_low_cnt;   /* HS LOW 연속 카운트(디바운스)          */

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
void       Moeum_RequestStop(void);          /* 4.5 비상정지 요청(정지버튼)   */
void       Moeum_SenseTick(void);            /* 100ms, StartDefaultTask       */
void       Moeum_MotorTick(uint32_t now_ms); /* 1ms, StartMotorTask           */
MoeumState Moeum_GetState(void);
uint8_t    Moeum_IsBusy(void);               /* 1 = IDLE/DONE/ERROR 이 아님   */

#ifdef __cplusplus
}
#endif

#endif /* SCENARIO_MOEUM_H_ */
