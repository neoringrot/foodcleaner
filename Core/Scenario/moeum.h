#ifndef SCENARIO_MOEUM_H_
#define SCENARIO_MOEUM_H_

#include "main.h"
#include <stdint.h>
#include "rotation_port.h"   /* ★R3 개정4 ④: 본교반·배수교반 = 회전수 운전 */
#include "rinse.h"           /* ★R3 5단계 [2026-09-21 §0.27]: 가이드 준비 탐색(R001~R006) */

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * moeum - "모음"(1단계 헹굼 세척) 시나리오 상태머신.
 *
 * 근거: Core/doc/사용자_시나리오_검토정리R3.md
 *   - §4.1   모음 기능(헹굼 세척) 순서
 *   - §4.1.1 모음 펌웨어 제어 시퀀스(하드웨어 매핑, R1 넷리스트/260806 핀맵)
 * 관련 HW 미확정: Core/doc/하드웨어결정_잔여항목R3.md (§1-1 수위/전류, §1-2 홀,
 *                §2-1 도어 홀 극성). 아래 placeholder 상수는 벤치 확정 후 갱신.
 *
 * 시퀀스(락 미SET=정상 전제):
 *   1) HS5(모음) 눌림 감지 → 시작
 *   2) 배수문(U5, WDoor) 닫힘 구동             [MotorTick]
 *   3) 배수문 닫힘 인식(W-HALL-CLOSE, PF3) → 모터 정지
 *   4) 급수 솔밸브 ON (VALVE-DRY-IN, PB13, 최대)
 *   5) 수위센서(PF6, REV02 SEN1 단독) 감지 → 추가급수 0초(R3 C017) → 밸브 OFF
 *   6) 교반 BLDC(M2, U16) 30RPM **WASH 회전수 운전**: 2회전+정지 2s × 10회
 *      (CW×5 → CW/CCW/CW/CCW/CW). ★R3 개정4 ④ [2026-09-21, §0.26]
 *   7) 배수문 열림 구동 + **DRAIN 회전수 운전** 시작(CW 2회전+정지 2s × 10회)  [MotorTick]
 *   8) 배수문 열림 인식(W-HALL-OPEN, PF4) → 도어 모터 정지 (교반은 계속)
 *   9) DRAIN 10회 완료 → 교반 정지 → 배수문 열린 상태로 종료
 *   ※ 종전 6)·9) 는 시간 종료(8s 패턴 × 18 / 열림 후 30초)였다 — MOEUM_STIR_CYCLES·
 *     MOEUM_DRAIN_STIR_MS 는 ZG_TRACKED_ONLY 로 추적용만 남는다.
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
/* ★★R3 개정4 ④ [2026-09-21, §0.26]: 아래 **시간 패턴은 제어에 쓰이지 않는다.** 교반은
 * rotation_port(WASH/DRAIN)가 구동한다. MOEUM_STIR_CW/CCW/DELAY_MS 는 앱 보고
 * (protocol_r0.c 패턴 필드) 호환 때문에만 남아 있다 — 앱 동기화(8단계)에서 정리.
 *
 * (이력) ★R3 C013(P09~P11, config.c `rinse_*`) — 2026-09-20 2단계.
 * 교반 패턴 이력: 25RPM CW3/정지1/CCW3(1cycle 7s)
 *   -> 2026-08-25 30RPM CW4/정지1/CCW4(1cycle 9s)
 *   -> 2026-08-26 30RPM CW6/정지1/CCW6(1cycle 13s)
 *   -> 2026-09-20 **30RPM 정3/정지1/역3/정지1(1cycle 8s)**  ← 현재
 * ★★ 4구간이다. 09.16 레퍼런스 `controller.c` 의 alternate() 가
 * "Each direction change has its own stop interval" 이라 **역→정 전환에도**
 * 정지가 들어간다(I06 해소). 그래서 MOEUM_STIR_DELAY2 phase 가 새로 생겼다.
 * 동작 시나리오의 1·2차 교반 헹굼(dongjak.h DJ_RINSE_*)도 같은 구간을 쓴다.
 * RPM 은 30 유지. */
#ifndef MOEUM_STIR_OUT_RPM
#define MOEUM_STIR_OUT_RPM      30U      /* 교반 날개 RPM — WASH/DRAIN 지령(P08) */
#endif
#ifndef MOEUM_STIR_CCW_MS
#define MOEUM_STIR_CCW_MS       3000U    /* R3 C013: 역회전 3초 (3번째 구간)  */
#endif
#ifndef MOEUM_STIR_DELAY_MS
#define MOEUM_STIR_DELAY_MS     1000U    /* 정지 1초 — 2·4번째 구간 공용      */
#endif
#ifndef MOEUM_STIR_CW_MS
#define MOEUM_STIR_CW_MS        3000U    /* R3 C013: 정회전 3초 (1번째 구간)  */
#endif
#if ZG_TRACKED_ONLY   /* ★개정4 ④: 추적용 — 본교반 종료는 WASH 10회 운전(C013·C014) */
#ifndef MOEUM_STIR_CYCLES
#define MOEUM_STIR_CYCLES       18U      /* 1cycle = 정3+정지1+역3+정지1 = 8s.
                                          * 목표 총시간 2분30초(150s)에
                                          * '사이클을 내려서' 맞춘다(2026-08-26
                                          * 규칙 유지) -> floor(150/8) = 18.
                                          * 8s x 18 = 144s(2m24s).
                                          * 19사이클이면 152s 로 150s 를 넘기므로
                                          * 내림이 맞다.
                                          * ★R3 C013 으로 1cycle 이 13s -> 8s 가
                                          * 되면서 11 -> 18 로 재계산(2026-09-20).
                                          * ※ R3 의 실제 헹굼 종료는 시간/사이클이
                                          *   아니라 HW 신호(가이드 엣지 cnt 20/40)
                                          *   다 — C014, 5단계에서 교체된다. */
#endif
#endif /* ZG_TRACKED_ONLY */
/* ★R3 C017(ST14 / P38 / TB-D01): 수위 감지 후 추가 급수 2초 -> **0초**.
 * 감지~밸브 닫기 ≤100ms 요구. 상태(MOEUM_FILL_EXTRA)는 남겨 두되 0ms라 진입
 * 즉시 통과한다(상태머신 구조 불변 — 상태 삭제는 5단계 헹굼 재작성 소관).
 * 동작 시나리오의 짝은 dongjak.h: DJ_FILL_EXTRA_MS. */
#ifndef MOEUM_FILL_EXTRA_MS
#define MOEUM_FILL_EXTRA_MS     0U       /* R3 C017: 추가급수 없음(0초)       */
#endif
/* ★§0.30: 헹굼이 rinse.c 로 옮겨 가면서 추가급수 단계 자체가 없어졌다 — 0 이 아니면 조용히
 * 무시되는 대신 빌드를 막는다. */
#if (MOEUM_FILL_EXTRA_MS != 0U)
#error "MOEUM_FILL_EXTRA_MS: R3 C017 로 추가급수는 0 초다. rinse.c 에는 추가급수 단계가 없다(§0.30)"
#endif
/* ★§0.30 헹굼 반복 횟수 N — 벤더 P01 rinse_repeats(0.3.0 config.c = 1).
 * ★R3 C004 [2026-09-22, §0.35] 사용자 지시로 **1회 고정**(모음·동작 공통).
 * TODO(C004, TBD: N18) N(1~3) 입력 경로·버튼 배정은 시나리오 미정 — 차후 질의(dongjak.h DJ_RINSE_COUNT 주석). */
#ifndef MOEUM_RINSE_REPEATS
#define MOEUM_RINSE_REPEATS     1U
#endif
/* 급수 시 WATER_ON(PE2, 급수 펌프/메인 밸브 enable)을 VALVE_DRY_IN(PB13)과
 * 함께 ON. 근거: 수위센서 HW 검증이 PE2 ON 상태(tb_water)에서 통수·감지되었고,
 * PB13 단독 통수는 미확인. 벤치에서 PB13 단독으로 물이 나오면 0으로 두면 된다. */
#ifndef MOEUM_FILL_USE_WATER_ON
#define MOEUM_FILL_USE_WATER_ON 1
#endif
#if ZG_TRACKED_ONLY   /* ★개정4 ④: 추적용 — 배수교반 종료는 DRAIN 10회 운전(C018) */
#ifndef MOEUM_DRAIN_STIR_MS
#define MOEUM_DRAIN_STIR_MS     30000U   /* 배수문 열림 후 30초 뒤 교반 정지  */
                                         /* ★2026-08-26: 1분 -> 30초 변경.  */
#endif
#endif /* ZG_TRACKED_ONLY */
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
/* ★2026-09-22 완료(MOEUM_DONE) 유지 시간. 종전에는 DONE 이 1 ms 만에 IDLE 로 넘어가 앱(1 s 주기 +
 * 상태변화 즉시 송신, 둘 다 100 ms defaultTask)이 state 7 을 한 번도 못 봤다 → 정상 완료가
 * "정지(중단)"·"DRAIN cnt 40 = 39 FAIL"로 찍혔다(report_20260922_063750_모음). 이 시간 동안
 * DONE 에 머문 뒤 IDLE. 운전 중이 아니므로(Moeum_IsBusy = 0) 정지·마개 로직과 무관하고,
 * 새 시작 요청은 DONE 에서도 받는다(곧바로 IDLE 로 넘어가 시작). */
#ifndef MOEUM_DONE_HOLD_MS
#define MOEUM_DONE_HOLD_MS       3000U    /* 앱 모니터 주기(1 s)의 3배 */
#endif
#ifndef MOEUM_LID_CONFIRM_SAMPLES
#define MOEUM_LID_CONFIRM_SAMPLES 3U      /* HS LOW 연속 N회(≈300ms) 후 정지    */
#endif

/* ---- 상태 (디버거 관찰용) ----------------------------------------------- */
typedef enum
{
	/* ★§0.30: 1~6·9 는 rinse.c 단계의 **미러**다(moeum_rinse_mirror). 제어는 g_moeum.rinse.step.
	 * MOEUM_FILL_EXTRA(3)는 더 이상 나타나지 않는다(C017). 번호 재배열은 8단계. */
	MOEUM_IDLE = 0,     /* 대기: 시작 요청 기다림                            */
	MOEUM_DOOR_CLOSE,   /* 배수문 닫힘 구동 -> W-HALL-CLOSE 대기             */
	MOEUM_FILL,         /* 급수밸브 ON -> 수위 감지 대기                     */
	MOEUM_FILL_EXTRA,   /* 추가급수(MOEUM_FILL_EXTRA_MS=0 -> 즉시 통과)    */
	MOEUM_STIR,         /* WASH 회전수 운전 10회 (개정4 ④)                   */
	MOEUM_DRAIN_OPEN,   /* DRAIN 운전 시작 + 배수문 열림 구동 -> W-HALL-OPEN */
	MOEUM_DRAIN_WAIT,   /* (DRAIN 계속) 10회 완료 -> 교반 정지             */
	MOEUM_DONE,         /* 완료: 배수문 열린 상태로 정지                     */
	MOEUM_ERROR,        /* 타임아웃 등 이상 -> 안전 정지                     */
	/* ★[2026-09-21 §0.27] 가이드 준비 탐색. 흐름상 IDLE 다음(배수문 닫기 전)이지만 **값은 끝에
	 * 붙였다** — 앱·watch 가 쓰는 기존 번호(1~8)를 바꾸지 않기 위해서다. 재배열은 8단계. */
	MOEUM_PREP          /* R001~R006 교반가이드 탐색 -> 성공 시 DOOR_CLOSE  */
} MoeumState;

/* 교반 위상 — ★개정4 ④부터 **앱 보고용 미러**다(rotation 엔진 반환에서 채운다:
 * 1=CW→MOEUM_STIR_CW, 2=CCW→MOEUM_STIR_CCW, 0=정지→MOEUM_STIR_DELAY). 제어에 쓰지 않는다. */
typedef enum
{
	MOEUM_STIR_CCW = 0,
	MOEUM_STIR_DELAY,     /* 정 -> 역 사이의 정지                             */
	MOEUM_STIR_CW,
	MOEUM_STIR_DELAY2     /* R3 C013(I06): 역 -> 정 사이의 정지. 1cycle 경계   */
} MoeumStirPhase;

/* 배수문 이동 종료 사유 (2026-09-21) — **진단 전용**, 제어에 쓰지 않는다.
 * 닫힘/열림 모두 "리미트 인식 OR 시간 상한"으로 끝나며 어느 쪽이든 정상 진행한다
 * (wdoor.h 프로파일 규칙). 그래서 리미트를 못 잡아도 에러가 나지 않는데, 그 사실을
 * 사후에라도 알 수 있도록 무엇으로 끝났는지만 남긴다. 앱에는 MOEUM 패킷 flags 의
 * PROTO_MO_WCLOSE_TMO / PROTO_MO_WOPEN_TMO 로 나간다. */
typedef enum
{
	MOEUM_DOOR_BY_NONE = 0,   /* 아직 끝나지 않음(이번 회차에서 해당 이동 전)   */
	MOEUM_DOOR_BY_LIMIT,      /* 리미트(W-HALL-CLOSE/OPEN) 인식으로 종료        */
	MOEUM_DOOR_BY_TIMEOUT     /* 리미트 미인식, 시간 상한(4.2s / 6s)으로 종료   */
} MoeumDoorBy;

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

	/* 교반 — ★개정4 ④: 회전수 운전(rotation_port). 아래 4개는 **앱 보고용 미러**. */
	RotStir_t         rot;           /* WASH → DRAIN 회전수 운전                */
	RinseCtx          rinse;         /* ★§0.30 공통 헹굼 전 구간. rinse.step·fault·done (rinse.h) */
	volatile uint8_t  rot_st;        /* 직전 RotStir_Tick 반환 ROT_ST_*        */
	uint8_t           stir_active;   /* [미러] 1 = 회전수 운전 활성            */
	uint8_t           stir_phase;    /* [미러] MoeumStirPhase                   */
	uint32_t          stir_phase_since; /* [미러] 방향/정지가 바뀐 tick         */
	uint16_t          stir_cycles;   /* [미러] 현재 프로파일 완료 운전 수(0~10) */

	uint32_t          drain_open_since; /* W-HALL-OPEN 인식 tick              */

	/* 배수문 종료 사유(진단). 회차 시작(DOOR_CLOSE 진입)마다 NONE/0 으로 리셋되고
	 * 완료 후에도 다음 시작 전까지 남는다. */
	volatile uint8_t  door_close_by; /* MoeumDoorBy: 닫힘 종료 사유            */
	volatile uint8_t  door_open_by;  /* MoeumDoorBy: 열림 종료 사유            */
	uint16_t          door_close_ms; /* 닫힘 구동 시작~종료 경과 ms            */
	uint16_t          door_open_ms;  /* 열림 구동 시작~종료 경과 ms            */

	/* ★R3 예외처리 G3·G4 [2026-09-22, 구현현황 §0.33] — 에러 코드. 값은 **DjErrCode 와 같은 표**
	 * (DJ_ERR_GUIDE 9 = E01, DJ_ERR_RINSE_FILL 2 = E04, DJ_ERR_ROTATION 8 = E09). 해제 분류도 같다
	 * (Dongjak_ErrIsLatched): E01 은 마개 이탈·정지로 풀리고, 나머지는 err_clear_req(앱 CLEAR_ERR)로만. */
	volatile uint8_t  err_code;      /* MOEUM_ERROR 원인(RO). 0 = 없음                         */
	volatile uint8_t  err_clear_req; /* 1 쓰면 MOEUM_ERROR -> IDLE(1회성, MotorTick 이 소비)   */
	volatile uint8_t  last_err;      /* 마지막 에러(RO). 해제 뒤에도 남고 새 시작에서 0         */
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
void       Moeum_ClearError(void);           /* ★G3: MOEUM_ERROR -> IDLE 요청  */
uint8_t    Moeum_IsLatched(void);            /* ★G3: 1 = MOEUM_ERROR + 래치형 */

#ifdef __cplusplus
}
#endif

#endif /* SCENARIO_MOEUM_H_ */
