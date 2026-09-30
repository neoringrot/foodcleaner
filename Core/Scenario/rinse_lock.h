#ifndef SCENARIO_RINSE_LOCK_H_
#define SCENARIO_RINSE_LOCK_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * rinse_lock - UL1 상당(헹굼 락 회로)의 **MCU 구현**. 설계서 `공통헹굼_모듈_구조R3.md` §6. 구현현황 §0.34.
 *
 * R3 문서는 헹굼 진행량을 회로(UL1)가 가이드 감지 횟수 cnt 로 판정한다고 쓴다:
 *     cnt = 20  → 급수 허가 상실(hw_water_allowed = 0) → 배수문 열기
 *     cnt = 40  → 회차 완료(hw_cycle_complete = 1)     → 교반 정지
 * REV02 보드에는 UL1 이 없으므로(검토서 §0 결론 1·2) 같은 판정을 여기서 한다.
 * `rinse.c` 는 이 모듈이 만든 관측값만 본다 — 훗날 UL1 이 생기면 이 파일만 갈아끼운다.
 *
 * ★2026-09-22 사용자 확정
 *   - 엣지 정의 = **상승엣지**(자석 도착 1회 = 가이드 1회전, I02 실측). guide_edge.h GUIDE_EDGE_BOTH 0.
 *   - cnt 미도달 대기 = **15 s** (RINSE_MILESTONE_TIMEOUT_MS). 벤더 port_contract
 *     "count_mismatch": 소프트웨어 20회전(엔진 10회 운전)이 먼저 끝나도 20/40 을 **만들어 내지
 *     않는다** — 그 뒤 request_timeout 안에 오지 않으면 E09. 벤더는 값을 보드에 맡겼다.
 *
 * 계수 규칙
 *   - 기준값(base)은 **WASH 진입**(= 급수 종료, 자가세척은 준비 탐색 뒤)에서 한 번 뜬다.
 *     준비 탐색 엣지는 들어가지 않는다(C009). WASH→DRAIN 사이에서 **리셋하지 않는다**(20→40 연속).
 *   - GuideEdge_Count() 는 단조 증가라 uint32 차이로 랩어라운드에 안전하다.
 *   - 회차시계 t_cycle 도 같은 순간 기산한다(검토서 §4.3 "급수 종료 기산").
 *
 * 이상(hw_stir_fault) — **래치**. 회차 반복으로 지워지지 않고 RinseLock_Reset(= 새 운전의
 * 준비 탐색 시작)에서만 풀린다(C022).
 *   - 과속(C015): cnt 40 도달 시 t_cycle < RINSE_MIN_CYCLE_MS(60 s) 이면 이상.
 *     (무부하 실측 WASH+DRAIN ≈140 s — 정상 운전에서 걸릴 값이 아니다.)
 *   - cnt 미도달(count_mismatch): 판정은 rinse.c 가 엔진 완료 시각과 함께 하고 여기 기록한다.
 *   - 순이동(C060·C094)은 ★§0.38 RinseLock_NetTick. 과속 보조(C061)는 ★§0.41 RinseLock_RpmTick(A, UL2 상당)·
 *     RinseLock_IntervalTick(B, 가이드 간격 — 순이동 기준 G-B 대신 시간 기준으로 채택). 아래 참조.
 *
 * 재급수 차단(C019): 락이 무장된 동안(cnt 계수 중) cnt ≥ 20 이면 급수 불가.
 *   회차가 새로 시작되면(RinseLock_Disarm) 다시 허가 — UL1 래치가 R007 에서 풀리는 것과 같다.
 * ========================================================================== */

#ifndef RINSE_WASH_EDGES
#define RINSE_WASH_EDGES            20U      /* ZG_HW_WASH_EDGES  — 급수 차단·배수 개시   */
#endif
#ifndef RINSE_TOTAL_EDGES
#define RINSE_TOTAL_EDGES           40U      /* ZG_HW_TOTAL_EDGES — 회차 완료·교반 정지   */
#endif
#ifndef RINSE_MIN_CYCLE_MS
#define RINSE_MIN_CYCLE_MS          60000U   /* ZG_HW_MIN_CYCLE_MS — 이보다 빠르면 과속    */
#endif
/* C015 시간 과속(cnt 40 도달 시 t_cycle < 60 s → E02) 판정 스위치. t40_ms 기록은 끈 상태에서도 남는다.
 * ★2026-09-22 사용자 지시 — **E02 헹굼 과속은 R4 에서 진행, 현재 미적용.** 모음 벤치(report_20260922_062811)에서 DRAIN 중 E02 로 정지 → 0 으로 끔(코드는 보존). R4 에서 1 로 되돌린다. */
#ifndef RINSE_MIN_CYCLE_GUARD
#define RINSE_MIN_CYCLE_GUARD       0        /* R4 이관 — 0 = 미적용 */
#endif
#ifndef RINSE_MILESTONE_TIMEOUT_MS
#define RINSE_MILESTONE_TIMEOUT_MS  15000U   /* 엔진 10회 완료 후 cnt 20/40 대기 → E09.
                                              * 2026-09-22 사용자 확정(운전 1회 ≈6.9 s 의 2회분). */
#endif

/* ★R3 C094·C060 [2026-09-22, 구현현황 §0.38] — **가이드 순이동 감시**(벤더 B006, I18 의 MCU 대체 방어 — N12).
 * "실제 정회전 이동은 더하고 역회전은 뺌. |순이동| ≥ 올림(1.25×4P)에 다음 가이드 없으면 이상. 저속·정지 시간만으로 판정하지 않음."
 * 이동 = M2 FG × DIR 부호 위치(BldcCtrl_Position, 호출부가 RinseInputs.stir_pos 로 준다). 가이드 엣지(HS6 상승)마다 기준 위치를
 * 새로 뜬다(순이동 0). 벤더 "4P"(가이드 1회전당 모터 홀 주기 × 4)를 우리 FG 단위로 옮기면 **P54 = 822(FG/날개 1회전)** 로 보고
 * 한도 = 올림(1.25 × P54) = 1028 — 이 해석은 N12 질의에 덧붙였다. 구간 = 락 무장(WASH 진입) ~ cnt 40 교반 정지.
 * 이상이면 교반만 즉시 정지 → E01(해제형, 벤더 X001). 0 = 감시 끔. */
#ifndef RINSE_NET_GUARD_ENABLE
#define RINSE_NET_GUARD_ENABLE      1
#endif
#ifndef RINSE_NET_GUIDE_TICKS
#define RINSE_NET_GUIDE_TICKS       ((ROT_M2_TICKS_PER_REV * 5U + 3U) / 4U)   /* 올림(1.25 × P54) — rinse_lock.c 에서만 전개 */
#endif

/* ★R3 C061 [2026-09-22, 구현현황 §0.41] — **교반 과속 보조 감시**(벤더 B007/BG7, UL2 LM2917 의 MCU 대체). 결과는 둘 다 E02.
 * A (벤더와 같음): 날개 rpm(M2 FG 주파수 ÷ 기어, g_stir_ctrl.meas_out_rpm — UL2 가 보는 모터 홀 주파수와 같은 신호)이
 *   P14 40rpm 초과가 RINSE_OVERSPEED_HOLD_MS 연속이면 이상. 40 이하에선 절대 차단 안 함(TB-B07). 구간 = 준비 회전 + 본 헹굼·배수.
 * B (벤더에 없음, 사용자 지시 2회 연속·스위치): 가이드(HS6) 엣지 간격 < 60000/P14 = 1.5 s 가 2회 연속이면 이상 —
 *   FG 와 날개가 따로 노는 경우(기어 이탈·고속 모터 오결선, I18)를 날개 쪽 신호로 잡는다. 방향 전환 재통과는 엔진 정지 2 s 가 끼어
 *   1.5 s 보다 길다. 구간 = 본 헹굼·배수(준비 탐색은 첫 엣지에서 멈춘다). 남는 위험 = 자석·홀 이중 감지. */
/* ★2026-09-22 사용자 지시 — **E02 헹굼 과속은 R4 에서 진행, 현재 미적용.** 모음 벤치(report_20260922_062811)에서 DRAIN 중 E02 로 정지 → 0 으로 끔(코드는 보존). R4 에서 1 로 되돌린다. */
#ifndef RINSE_RPM_GUARD_ENABLE
#define RINSE_RPM_GUARD_ENABLE      0        /* R4 이관 — A(날개 rpm > 40) 미적용. 종전 1 */
#endif
#ifndef RINSE_OVERSPEED_RPM
#define RINSE_OVERSPEED_RPM         40U      /* P14 명목 과속 기준(날개 rpm). 초과(>)만 이상          */
#endif
#ifndef RINSE_OVERSPEED_HOLD_MS
#define RINSE_OVERSPEED_HOLD_MS     300U     /* 측정 창(100 ms) 3개 — 기동 오버슈트·창 잡음 흡수       */
#endif
#ifndef RINSE_GUIDE_INTERVAL_GUARD
#define RINSE_GUIDE_INTERVAL_GUARD  0        /* R4 이관 — B(가이드 간격 < 1.5 s ×2) 미적용. 종전 1 */
#endif
#ifndef RINSE_GUIDE_MIN_INTERVAL_MS
#define RINSE_GUIDE_MIN_INTERVAL_MS (60000U / RINSE_OVERSPEED_RPM)   /* 1500 ms = 40rpm 1회전 */
#endif
#ifndef RINSE_GUIDE_SHORT_COUNT
#define RINSE_GUIDE_SHORT_COUNT     2U       /* 짧은 간격 연속 횟수 */
#endif

/* 이상 원인 (RinseLock.fault, 래치) */
typedef enum
{
	RINSE_LOCK_OK = 0,
	RINSE_LOCK_COUNT_WASH,    /* WASH 엔진 10회 완료 후 15 s 안에 cnt 20 미도달  (E09) */
	RINSE_LOCK_COUNT_DRAIN,   /* DRAIN 엔진 10회 완료 후 15 s 안에 cnt 40 미도달 (E09) */
	RINSE_LOCK_OVERSPEED,     /* cnt 40 도달 시 t_cycle < 60 s                   (C015) */
	RINSE_LOCK_GUIDE_MISSING, /* ★§0.38 |순이동| ≥ 1.25 회전인데 새 가이드 없음 (B006, E01) */
	RINSE_LOCK_OVERSPEED_RPM, /* ★§0.41 A 날개 rpm > 40 이 300 ms 연속 (B007/UL2 상당, E02)      */
	RINSE_LOCK_GUIDE_INTERVAL /* ★§0.41 B 가이드 간격 < 1.5 s 2회 연속 (기어 이탈·오결선, E02) */
} RinseLockFault;

typedef struct
{
	volatile uint8_t  armed;        /* 1 = 이번 회차 계수 중(WASH 진입 ~ 다음 회차 시작)   */
	uint32_t          base;         /* GuideEdge_Count() 기준값                           */
	volatile uint16_t cnt;          /* 이번 회차 가이드 감지 수 (watch)                   */
	uint32_t          cycle_since;  /* t_cycle 기산 tick (급수 종료 = WASH 진입)          */
	volatile uint32_t t20_ms;       /* 기산 → cnt 20 도달 (0 = 미도달)                    */
	volatile uint32_t t40_ms;       /* 기산 → cnt 40 도달 (0 = 미도달)                    */
	volatile uint8_t  fault;        /* RinseLockFault (래치, C022)                         */
	/* ★§0.38 순이동 감시 (B006) */
	uint8_t           net_armed;    /* 1 = 기준 위치를 떴다                                */
	uint32_t          net_base_pos; /* 마지막 가이드 엣지(또는 감시 시작) 때 stir_pos      */
	uint32_t          net_edges;    /* 그때의 GuideEdge_Count()                           */
	volatile int32_t  net;          /* 현재 순이동(FG, 부호) — watch                      */
	volatile uint16_t net_max;      /* 이번 회차 |순이동| 최대(여유 확인용, RO)           */
	/* ★§0.41 과속 보조 */
	uint32_t          rpm_over_since; /* A: rpm > 40 연속 시작 tick (0 = 아님)              */
	volatile uint16_t rpm_max;      /* A: 감시 구간 rpm 최대 (RO)                          */
	uint8_t           iv_armed;     /* B: 직전 엣지 기준을 갖고 있다                        */
	uint32_t          iv_edges;     /* B: 직전 GuideEdge_Count()                          */
	uint32_t          iv_last_ms;   /* B: 직전 엣지 시각(GuideEdge_LastEdgeMs)             */
	volatile uint32_t iv_min_ms;    /* B: 이번 운전 최소 간격 (RO, 0 = 없음)               */
	volatile uint8_t  iv_short;     /* B: 짧은 간격 연속 수                                 */
} RinseLock;

void    RinseLock_Reset     (RinseLock *l);                 /* 새 운전(준비 탐색) — 이상 해제 포함  */
void    RinseLock_BeginCycle(RinseLock *l, uint32_t now_ms); /* WASH 진입: base·t_cycle 기산        */
void    RinseLock_Disarm    (RinseLock *l);                 /* 다음 회차 시작(R007): 급수 재허가    */
void    RinseLock_Tick      (RinseLock *l, uint32_t now_ms); /* 1 ms: cnt 갱신·도달시각·과속        */
/* ★§0.38 1 ms: 순이동 감시. watch = 1 인 동안만(교반 중 — WASH/DRAIN, cnt 40 정지 전). 이상이면 1 을 돌려준다. */
uint8_t RinseLock_NetTick   (RinseLock *l, uint32_t stir_pos, uint8_t watch);
/* ★§0.41 1 ms: 과속 보조. A = rpm(날개) 감시, B = 가이드 간격 감시. 이상이면 1. armed 와 무관(A 는 준비 탐색도 본다). */
uint8_t RinseLock_RpmTick     (RinseLock *l, uint16_t stir_rpm, uint8_t watch, uint32_t now_ms);
uint8_t RinseLock_IntervalTick(RinseLock *l, uint8_t watch);
void    RinseLock_SetFault  (RinseLock *l, RinseLockFault f);/* 처음 원인만 남긴다                   */

/* 관측값 — 레퍼런스 zg_inputs 의 hw_* 3종 */
uint8_t RinseLock_WaterAllowed (const RinseLock *l);   /* hw_water_allowed  = !armed || cnt < 20 */
uint8_t RinseLock_WashDone     (const RinseLock *l);   /* armed && cnt >= 20                     */
uint8_t RinseLock_CycleComplete(const RinseLock *l);   /* hw_cycle_complete = armed && cnt >= 40 */
uint8_t RinseLock_StirFault    (const RinseLock *l);   /* hw_stir_fault     = fault != OK        */

#ifdef __cplusplus
}
#endif

#endif /* SCENARIO_RINSE_LOCK_H_ */
