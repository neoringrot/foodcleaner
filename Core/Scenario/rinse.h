#ifndef SCENARIO_RINSE_H_
#define SCENARIO_RINSE_H_

#include "main.h"
#include <stdint.h>
#include "rinse_lock.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * rinse - 공통 헹굼 모듈 (모음·동작·자가세척 공용). 설계서 `공통헹굼_모듈_구조R3.md` v2.
 *
 * ★1차 [2026-09-21, 구현현황 §0.27] — 가이드 준비 탐색(R001~R006, C006~C010).
 * ★2차 [2026-09-22, 구현현황 §0.30] — **회차 루프(R007~R019)** 를 moeum.c / dongjak.c 에서
 *   이 모듈로 옮겼다(설계서 §11-3~5). 자가세척(09.16 O016, 0.3.0 동일)이 세 번째 호출부다.
 * ★3차 [2026-09-22, 구현현황 §0.34] — **rinse_lock(cnt 20/40, UL1 상당)** 연결. 종료 판정이 엔진 10회 운전 완료에서
 *   **가이드 감지 cnt** 로 바뀌었다(벤더 R013~R017, C014·C015·C019·C020·C022):
 *     WASH  : cnt 20 → 배수(엔진 도중이어도). 엔진 10회가 먼저 끝나면 15 s 안에 cnt 20 → 아니면 E09.
 *     DRAIN : cnt 40 → 교반 정지(문 여는 중이어도, 문은 계속). 엔진 10회 후 15 s 안에 40 → 아니면 E09.
 *     완료  : cnt 40 && 이상 이력 없음(과속 t_cycle < 60 s 포함) → done++.
 *   ★§0.38 순이동 감시(C060·C094, B006) 추가 — 운전 중 가이드 소실 → E01. BLDC 과속 보조(C061)는 TODO. rinse_lock.h 참조.
 *
 * 흐름 (벤더 controller.c 0.3.0 ZG_R001~ZG_R019 대응, UE1 요청/수락 R007·R012 는 0 ms 통과):
 *
 *   prep_after_fill = 0 (모음·동작)           prep_after_fill = 1 (자가세척, 벤더 self_clean_guide_pending)
 *   ───────────────────────────────           ───────────────────────────────────────────────
 *   PREP(R001~R006)                           CLOSE(R008~R009)
 *    └→ CLOSE(R008~R009)                       └→ FILL(R010)
 *        └→ FILL(R010)                             └→ PREP(R011→R001~R006, 물 찬 상태에서 탐색)
 *            └→ WASH(R012~R013)                        └→ WASH(R012~R013)
 *   이후 공통: DRAIN_OPEN(R014~R015) → DRAIN(R016) → done++ (R018)
 *             → done < repeats 이면 CLOSE(R007~) / 아니면 FINISHED(R019)
 *   준비 탐색은 **운전당 1회**(회차 반복 때 다시 찾지 않는다 — 벤더와 같다).
 *
 * 준비 탐색 (벤더 ZG_R001~ZG_R006 — 09.21 개정 무관):
 *   R001 정회전 30rpm, 최대 5 s  ─┐ 가이드(HS6) 감지 → 즉시 정지 → R006 성공
 *   R002 정지 1 s                 │  (5.000 s 경계는 감지 우선 — 엣지 시각으로 판정, C008)
 *   R003 역회전 30rpm, 최대 5 s   │
 *   R004 정지 1 s                 │
 *   R005 정회전 30rpm, 최대 5 s  ─┘ 미감지 → E01(RINSE_PREP_FAULT). 4차 재시도 금지(C010)
 *   - 감지 = GuideEdge_Count() 가 baseline 과 다름. baseline 은 탐색 시작과 각 정지 끝에서 뜬다.
 *   - 준비 중 감지는 본 헹굼 cnt 에 넣지 않는다(C009).
 *   - 정회전 = **위에서 볼 때 시계방향**(0.3.0 정회전, rotation_port 와 같은 기준).
 *
 * 배수문 (wdoor.h 프로파일 — 벤치 확정 규칙 그대로):
 *   닫힘 = WHALL-CLOSE 인식 **또는** WDOOR_CLOSE_MS(4.2 s) 경과가 정상 종료.
 *   열림 = WHALL-OPEN  인식 **또는** WDOOR_OPEN_MAX_MS(6 s) 상한. duty 는 킥 80 % 2 s → 65 %.
 *   리미트 미인식은 에러가 아니라 이벤트(RINSE_EV_*_TMO)다 — 호출부가 안내 음성만 낸다.
 *
 * ★액추에이터를 만지지 않는다(설계서 §3). 한 tick 의 **명령 비트(cmd)** 와 **사건 비트(ev)** 를
 *   RinseOutputs 로 돌려주고, 호출부가 자기 문맥(MotorTask)에서 적용한다. 호출부마다 다른 것 —
 *   배수밸브 동반 여부(동작만), 음성, 에러 코드, 앱 보고용 상태 미러 — 은 호출부에 남는다.
 *
 * 호출 규약 (1 ms MotorTick):
 *   1) rot 가 활성이면 먼저 RotStir_Tick() → in.rot_st
 *   2) in.door_closed/open = WDoor_ReachedClose/Open(), in.water = 수위 래치
 *   3) Rinse_Tick(c, &in, now, &out)
 *   4) out.cmd 적용 (RINSE_CMD_FILL_ON 이면 호출부의 수위 래치를 먼저 지운다)
 *   5) out.ev / 반환 step 으로 음성·에러·다음 단계
 * ========================================================================== */

#ifndef RINSE_PREP_MS
#define RINSE_PREP_MS          5000U   /* ZG_PREPARE_MS — 탐색 스트로크 상한 (R001·R003·R005) */
#endif
#ifndef RINSE_PREP_STOP_MS
#define RINSE_PREP_STOP_MS     1000U   /* P10 rinse_stop_ms — R002·R004 정지. ★P53(2 s)로 대체되지 않는다 */
#endif
#ifndef RINSE_PREP_RPM
#define RINSE_PREP_RPM         30U     /* P08 rinse_rpm — 교반 날개 rpm                          */
#endif

typedef enum
{
	RINSE_IDLE = 0,
	/* --- 준비 탐색 (R001~R006). 값 1~7 은 §0.27 watch 문서와 같다 --- */
	RINSE_PREP_F1,      /* 1 R001 정회전 탐색, 최대 5 s          */
	RINSE_PREP_S1,      /* 2 R002 정지 1 s                       */
	RINSE_PREP_R,       /* 3 R003 역회전 탐색, 최대 5 s          */
	RINSE_PREP_S2,      /* 4 R004 정지 1 s                       */
	RINSE_PREP_F2,      /* 5 R005 정회전 탐색(마지막), 최대 5 s  */
	RINSE_PREP_OK,      /* 6 R006 감지 성공 — 같은 tick 에 다음 단계로 넘어가는 통과 상태 */
	RINSE_PREP_FAULT,   /* 7 E01 교반가이드 미확인 — 종료(교반 차단)                      */
	/* --- 회차 루프 (R007~R019) — ★§0.30 --- */
	RINSE_CLOSE,        /* 8  R007~R009 배수문 닫힘 → 리미트 또는 4.2 s                 */
	RINSE_FILL,         /* 9  R010 급수 → 수위 감지 (추가급수 0 s, C017)                */
	RINSE_WASH,         /* 10 R011~R013 WASH 회전수 운전 → **cnt 20**                   */
	RINSE_DRAIN_OPEN,   /* 11 R014~R015 DRAIN 전환 + 배수문 열림 (교반은 문 여는 중에도, cnt 40 이면 교반만 정지) */
	RINSE_DRAIN,        /* 12 R016~R017 DRAIN 계속 → **cnt 40**                         */
	RINSE_FINISHED,     /* 13 R018~R019 반복 완료 — 배수문 열린 채, 교반 정지           */
	RINSE_FAULT         /* 14 급수 타임아웃 / 회전수 실패 — 원인은 RinseCtx.fault      */
} RinseStep;

/* 종료 원인 (RinseCtx.fault) */
typedef enum
{
	RINSE_FAULT_NONE = 0,
	RINSE_FAULT_GUIDE,  /* E01 준비 3회 미감지  (step = RINSE_PREP_FAULT) */
	RINSE_FAULT_FILL,   /* E04 급수 타임아웃    (step = RINSE_FAULT)      */
	RINSE_FAULT_ROT,    /* E09 회전수 운전 실패 (step = RINSE_FAULT)      */
	RINSE_FAULT_COUNT,  /* E09 엔진 10회 완료 후 15 s 안에 cnt 20/40 미도달 (count_mismatch). lock.fault 로 WASH/DRAIN 구분 */
	RINSE_FAULT_OVERSPEED, /* C015 cnt 40 도달 시 t_cycle < 60 s (과속)                                               */
	RINSE_FAULT_GUIDE_LOST /* ★§0.38 C094·C060 운전 중 가이드 소실 — |순이동| ≥ 1.25 회전에 새 가이드 없음 (B006, E01)  */
} RinseFault;

/* 배수문 종료 사유 — 진단 전용. 값은 moeum.h MoeumDoorBy 와 같다. */
#define RINSE_DOOR_BY_NONE     0U
#define RINSE_DOOR_BY_LIMIT    1U
#define RINSE_DOOR_BY_TIMEOUT  2U

/* ---- 명령 비트 (RinseOutputs.cmd) — 이번 tick 에 호출부가 할 일 ---------- */
#define RINSE_CMD_STIR_MANUAL  0x0001U  /* 준비 탐색 교반: RotStir_Manual(stir_dir, stir_rpm) */
#define RINSE_CMD_STIR_STOP    0x0002U  /* 교반 정지·슬립: RotStir_Stop()                       */
#define RINSE_CMD_ROT_WASH     0x0004U  /* RotStir_Start(WASH, rot_rpm)                          */
#define RINSE_CMD_ROT_DRAIN    0x0008U  /* RotStir_Switch(DRAIN, rot_rpm) — 엔진 유지            */
#define RINSE_CMD_DOOR_CLOSE   0x0010U  /* 배수문 닫힘 개시 (LimitArm·Enable·Close 80 %)         */
#define RINSE_CMD_DOOR_OPEN    0x0020U  /* 배수문 열림 개시 (LimitArm·Enable·Open door_duty)     */
#define RINSE_CMD_DOOR_DUTY    0x0040U  /* 열림 중 duty 갱신: WDoor_Open(door_duty)              */
#define RINSE_CMD_DOOR_STOP    0x0080U  /* 배수문 정지 (Stop·Disable) — 위치 유지                */
#define RINSE_CMD_FILL_ON      0x0100U  /* 급수 ON — **먼저 수위 래치·EXTI 플래그를 지운다**     */
#define RINSE_CMD_FILL_OFF     0x0200U  /* 급수 OFF                                              */

/* ---- 사건 비트 (RinseOutputs.ev) — 음성·진단용. 제어에는 영향 없음 -------- */
#define RINSE_EV_CLOSE_TMO     0x01U    /* 닫힘이 리미트 없이 시간으로 끝남 (음성 05-04)   */
#define RINSE_EV_OPEN_TMO      0x02U    /* 열림이 리미트 없이 상한으로 끝남 (음성 05-05)   */
#define RINSE_EV_DRAIN_OPENED  0x04U    /* 배수문 열림 완료 — 배수 시작 (모음 음성 01-01)  */
#define RINSE_EV_CYCLE_DONE    0x08U    /* 한 회차 완료 (done 증가)                        */
#define RINSE_EV_PREP_OK       0x10U    /* 준비 탐색 성공                                   */

typedef struct
{
	uint8_t  repeats;          /* 반복 N (C004/C011, 벤더 P01). 1 이상                    */
	uint16_t rot_rpm;          /* WASH/DRAIN 교반 지령 (날개 rpm, P08 = 30)               */
	uint32_t fill_timeout_ms;  /* 수위 미감지 고장 판정 (moeum 30분 / dongjak 120 s)      */
	uint8_t  prep_after_fill;  /* 1 = 급수 뒤 준비 탐색 (자가세척). 0 = 시작 직후 (모음·동작) */
} RinseConfig;

typedef struct
{
	uint8_t  door_closed;      /* WDoor_ReachedClose() (arm 이후 엣지 OR 레벨) */
	uint8_t  door_open;        /* WDoor_ReachedOpen()                          */
	uint8_t  water;            /* 수위 감지 래치 (호출부 SenseTick)            */
	uint8_t  rot_st;           /* 이번 tick RotStir_Tick 반환 (비활성이면 ROT_ST_IDLE) */
	uint32_t stir_pos;         /* ★§0.38 BldcCtrl_Position(&g_stir_ctrl) — 순이동 감시(B006) */
	uint16_t stir_rpm;         /* ★§0.41 g_stir_ctrl.meas_out_rpm(날개) — 과속 보조 A(B007)   */
} RinseInputs;

typedef struct
{
	uint16_t cmd;              /* RINSE_CMD_*                                        */
	uint8_t  ev;               /* RINSE_EV_*                                         */
	uint8_t  stir_dir;         /* RINSE_CMD_STIR_MANUAL: 0 = 정(위에서 CW) 1 = 역   */
	uint16_t stir_rpm;         /* RINSE_CMD_STIR_MANUAL: 0 = 깨운 채 정지            */
	uint16_t rot_rpm;          /* RINSE_CMD_ROT_WASH/DRAIN                           */
	uint8_t  door_duty;        /* RINSE_CMD_DOOR_OPEN/DUTY                           */
} RinseOutputs;

typedef struct
{
	volatile uint8_t  step;        /* RinseStep (watch)                              */
	uint32_t          step_since;  /* 현 step 진입 tick                               */
	uint32_t          base;        /* 감지 baseline (GuideEdge_Count)                 */
	RinseConfig       cfg;
	uint8_t           prep_done;   /* 이번 운전에서 준비 탐색을 마쳤다                */
	volatile uint8_t  done;        /* 완료 회차 수 (R018)                             */
	volatile uint8_t  fault;       /* RinseFault                                      */
	/* 관측 (RO) */
	volatile uint8_t  prep_try;    /* 성공한 try 1~3 (0 = 미성공)                     */
	volatile uint16_t prep_ms;     /* 성공 try 시작 → 감지 엣지 (ms)                  */
	volatile uint32_t prep_total_ms;/* 탐색 시작 → 성공/실패 (ms)                     */
	uint32_t          begin_ms;    /* 탐색 시작 tick                                  */
	volatile uint8_t  door_close_by; /* RINSE_DOOR_BY_* (회차의 CLOSE 진입마다 리셋)   */
	volatile uint8_t  door_open_by;
	uint16_t          door_close_ms; /* 닫힘 구동 시작~종료 ms                          */
	uint16_t          door_open_ms;
	uint32_t          drain_open_since; /* 배수문 열림 완료 tick                        */
	/* ★2026-09-22 cnt 20/40 (UL1 상당) */
	RinseLock         lock;        /* cnt·t20/t40·이상 래치 (watch: rinse.lock.cnt)    */
	uint8_t           sw_done;     /* 1 = 이번 프로파일 엔진 10회 완료를 봤다          */
	uint32_t          sw_done_since;/* 그 tick — 여기서 15 s 안에 cnt 마일스톤이 와야 함 */
	uint8_t           stir_cut;    /* 1 = cnt 40 으로 교반을 이미 멈췄다               */
} RinseCtx;

/* 운전 시작. out 에 첫 명령을 담아 돌려준다(호출부가 곧바로 적용). */
void      Rinse_Begin  (RinseCtx *c, const RinseConfig *cfg, uint32_t now_ms, RinseOutputs *out);
/* 1 ms MotorTick. RINSE_FINISHED / RINSE_PREP_FAULT / RINSE_FAULT 에 도달하면 그대로 머문다. */
RinseStep Rinse_Tick   (RinseCtx *c, const RinseInputs *in, uint32_t now_ms, RinseOutputs *out);
void      Rinse_Reset  (RinseCtx *c);                 /* RINSE_IDLE */
RinseStep Rinse_GetStep(const RinseCtx *c);
uint8_t   Rinse_IsEnded(const RinseCtx *c);           /* FINISHED / PREP_FAULT / FAULT */

#ifdef __cplusplus
}
#endif

#endif /* SCENARIO_RINSE_H_ */
