#ifndef SRC_TB_RINSE_H_
#define SRC_TB_RINSE_H_

#include "main.h"
#include "guide_edge.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * tb_rinse - 5단계(공통 헹굼) 벤치. **1차 범위는 가이드 엣지 계수뿐이다.**
 *
 * 근거: 설계서 [공통헹굼_모듈_구조R3.md] §10 V1~V4 · §11-1 · 항목 C007·C009·C014.
 * 이 벤치의 목적은 딱 하나 — **I02(엣지 정의) 를 실측으로 확정하는 것**이다.
 *   cnt 20/40 의 "1" 이 자석 1회 통과(상승엣지)인가, 통과+이탈(양엣지)인가?
 *   교반 1회전에 엣지가 몇 개 나오는가? → 40이 몇 바퀴인지가 여기서 정해진다.
 *
 * 아직 만들지 않은 것: 회차 FSM(rinse.c) · cnt→hw_* 합성(rinse_lock.c) ·
 * 준비 탐색. 그것들은 I02·I03·N3·I15 회신 후다(설계서 §9).
 *
 * WHERE TO POLL: **StartMotorTask(1ms)** 의 테스트벤치 분기.
 *   - 엣지 계수(GuideEdge_Tick)는 1ms 가 아니면 ≤50ms 응답을 못 맞춘다(C007).
 *   - 교반 BLDC(g_stir_ctrl)는 MotorTask 소유다.
 *   GuideEdge_Tick() 자체는 freertos.c 가 **모드와 무관하게 매 사이클** 부른다
 *   (시나리오 중에도 계수가 돌아야 5단계에서 그대로 쓴다). 이 벤치는 그 위에서
 *   측정/교반 구동만 얹는다.
 *
 * 안전: 교반 구동은 **원샷**이고 tb_rinse_spin_ms 가 지나면 스스로 멈춘다.
 *       Jungji_IsBraking() 구간에서는 기동하지 않는다(재기동 금지).
 *
 * ── ★램프를 빼야 한다 ─────────────────────────────────────────────────
 * g_stir_ctrl 의 slew_rpm_per_s = 20(출력축)이라 0->30rpm 에 **1.5초**가 걸리고
 * PI 정착까지 더 든다. 그래서 "12초 돌렸으니 6회전"은 **틀리다** — 램프 구간은
 * 30rpm 이 아니다. 이 벤치는 원샷을 두 구간으로 쪼개 그 오차를 없앤다:
 *
 *   spin_once=1
 *     |<-- tb_rinse_spin_settle_ms (기본 3000) -->|<-- 측정 구간 -->|
 *     0                    램프+정착(버림)        자동 mark        spin_ms 종료
 *                                                                  ↓ 결과 래치
 *
 * 측정 구간만으로 계산해 원샷이 끝나는 순간 아래를 **래치**한다:
 *     tb_rinse_res_edges     측정 구간 엣지 수
 *     tb_rinse_res_ms        측정 구간 길이(ms)
 *     tb_rinse_res_epr_x100  **1회전당 엣지 수 × 100**   <- 읽을 값은 이것 하나
 *         epr = edges * 60000 / (res_ms * rpm)
 * 래치값은 다음 원샷 전까지 유지되므로 정지한 뒤 천천히 읽으면 된다.
 *
 * ── 벤치 절차 (설계서 §10 V1·V4 의 선행 실측) ─────────────────────────
 *  0) g_app_mode 가 벤치(대기/정지)인지 확인. **측정 중 키패드를 누르지 말 것**
 *     — tb_tca9554 가 버튼을 g_stir_ctrl Start/Stop 으로 매핑한다(충돌).
 *  1) tb_rinse_both_edges = 0   (상승엣지만 = 자석 도착)
 *  2) tb_rinse_spin_rpm=30 · tb_rinse_spin_rev=0 · tb_rinse_spin_ms=15000
 *     (기본 settle 3000 + 측정 12000 = 30rpm 에서 6회전)
 *  3) tb_rinse_spin_once = 1    -> 자동으로 돌고, settle 후 mark, 끝나면 래치
 *  4) tb_rinse_spinning 이 0 이 되면 **tb_rinse_res_epr_x100** 을 읽는다.
 *         100 -> 1회전당 1엣지 / 200 -> 2엣지 / 400 -> 4엣지
 *  5) tb_rinse_both_edges = 1 로 뒤집고 3~4 반복.
 *     양엣지는 상승엣지의 **정확히 2배**가 나와야 정상이다(아니면 채터 의심).
 *  6) tb_rinse_spin_rev = 1 로 역회전도 반복. 정/역이 다르면 C060(순이동)의
 *     전제가 깨진다.
 *
 * 1회전당 엣지 수가 나오면 ZG_HW_TOTAL_EDGES(40) 이 몇 바퀴인지 계산되고,
 * 그 값이 R3 의 "회차 1회"가 물리적으로 무엇인지를 확정한다 — I02 회신이
 * 늦어져도 이 실측이 있으면 rinse_lock 을 쓸 수 있다.
 *
 * tb_rinse_mark_once 는 **손으로 돌려서 재는 경우**에만 쓴다(모터 없이 baseline
 * 만 뜨는 용도). spin_once 는 자기가 알아서 mark 하므로 같이 쓸 필요 없다.
 *
 * ══ 패턴 모드 (run E, 2026-09-21 추가) ═══════════════════════════════════
 * 1차 실측(연속 회전)으로 가이드 주기 3.10 s 가 나왔다. 그런데 실제 헹굼 교반은
 * **정3 / 정지1 / 역3 / 정지1** — 한 스트로크(3 s, 램프 포함)가 가이드 1주기보다
 * 짧고 정·역이 같은 길이라 **순이동이 0** 이다. 게다가 준비 탐색은 가이드를 감지한
 * 즉시 세우므로 본 세척은 **자석 바로 위**에서 출발해 매 사이클 그 자리로 돌아온다.
 * → cnt 가 진행하는지는 **연속 회전으로는 보이지 않는다.** 이 모드가 그걸 잰다.
 * (기록지 I02_엣지실측_기록R3.md §7-④)
 *
 *   pat_once=1
 *     park(선택): 정회전으로 돌다 **감지 즉시 정지** (준비 탐색 C006/C007 흉내)
 *     hold      : 정지 pat_stop_ms(1 s) 유지
 *     pattern   : 4구간 반복. baseline 은 여기서 다시 뜬다(C009 - park 감지 제외)
 *     종료      : cnt 40 도달(pat_stop_at40=1) 또는 pat_ms 경과 또는 stop_once
 *
 * 교반 지령은 dongjak.c dj_stir_spin() 과 **같게** 스트로크마다 Stop->Start 한다.
 * 그래서 매 스트로크가 램프부터 다시 오른다 = 실제 헹굼과 같은 회전각.
 *
 * ── run E 절차 ──────────────────────────────────────────────────────────
 *  0) g_app_mode 0 또는 5. 키패드 금지. tb_rinse_both_edges = 0(상승엣지).
 *  1) 기본값이 그대로 run E 다: 30rpm · 3000/1000/3000 · group 1 · park 1 ·
 *     pat_ms 360000(6분) · stop_at40 1. **tb_rinse_pat_once = 1** 하나만 쓴다.
 *  2) 도는 동안 pat_edges · pat_cycles · pat_max_gap_ms · pat_zero_cycles 를 본다.
 *     pat_max_gap_ms 가 **계속 커지면** 정체다 — 그 자체가 답이므로 stop_once 로 끊어도
 *     된다(지표는 래치된다).
 *  3) pat_state == 4(DONE) 가 되면 pat_result 와 아래 값을 읽는다.
 *
 * ── 판독 ────────────────────────────────────────────────────────────────
 *     pat_epc_x100  1cycle 당 엣지 × 100
 *        ≈200 : 정·역 스트로크가 매번 자석을 지난다 -> cnt40 ≈ 20 cycle ≈ 160 s
 *        ≈100 : 한 방향만 지난다                   -> cnt40 ≈ 40 cycle ≈ 320 s
 *        → 0  : **정체** — 5단계 cnt 방식이 무부하에선 성립 안 함(N3 과 겹쳐 헹굼 무한)
 *     pat_t20_ms / pat_t40_ms   cnt 20 / 40 도달 시각(0 = 미도달)
 *     pat_zero_cycles / pat_max_zero_run   0엣지 cycle 수 / 최장 연속 — 불안정성 지표
 *     pat_park_ms / pat_park_level         park 에 걸린 시간 / 정지 직후 레벨(1=자석 위)
 *
 * pat_park = 0 으로 두면 **현재 위치에서 바로** 출발한다 — 자석 위 출발(1)과 비교해
 * 출발 위치가 결과를 바꾸는지 본다.
 * ---------------------------------------------------------------------- */

/* 패턴 모드 상태 / 결과 코드 */
#define TB_RINSE_PAT_IDLE          0U
#define TB_RINSE_PAT_PARK          1U   /* 자석 찾는 중 (정회전)          */
#define TB_RINSE_PAT_HOLD          2U   /* 찾은 뒤 정지 유지              */
#define TB_RINSE_PAT_RUN           3U   /* 4구간 패턴 구동 중             */
#define TB_RINSE_PAT_DONE          4U

#define TB_RINSE_PAT_RES_NONE      0U
#define TB_RINSE_PAT_RES_REACHED   1U   /* cnt 40 도달                    */
#define TB_RINSE_PAT_RES_TIMEOUT   2U   /* pat_ms 동안 40 미도달          */
#define TB_RINSE_PAT_RES_PARK_FAIL 3U   /* park_max_ms 안에 가이드 못 찾음 */
#define TB_RINSE_PAT_RES_ABORT     4U   /* stop_once / 벤치 이탈          */

/* R3 레퍼런스 generated/config.h 의 ZG_HW_WASH_EDGES / ZG_HW_TOTAL_EDGES 와 같은 값.
 * 벤치 판정용 사본이다 - 5단계 rinse_lock 이 생기면 그쪽 상수로 바꾼다. */
#define TB_RINSE_WASH_EDGES        20U
#define TB_RINSE_TOTAL_EDGES       40U

/* ---- 관측 (읽기 전용) --------------------------------------------------- */
extern volatile uint32_t tb_rinse_count;        /* GuideEdge_Count() 스냅샷   */
extern volatile uint32_t tb_rinse_since_mark;   /* mark 이후 엣지 수          */
extern volatile uint32_t tb_rinse_last_edge_ms; /* 마지막 엣지 tick           */
extern volatile uint32_t tb_rinse_interval_ms;  /* 직전 두 엣지 간격          */
extern volatile uint32_t tb_rinse_int_min_ms;   /* mark 이후 최소 간격        */
extern volatile uint32_t tb_rinse_int_max_ms;   /* mark 이후 최대 간격        */
extern volatile uint32_t tb_rinse_mark_ms;      /* mark 를 뜬 tick            */
extern volatile uint32_t tb_rinse_elapsed_ms;   /* mark 이후 경과(구동 중 갱신) */
extern volatile uint8_t  tb_rinse_level;        /* 현재 가이드 비트(1=감지)   */
extern volatile uint8_t  tb_rinse_spinning;     /* 1 = 원샷 교반 구동 중      */
extern volatile uint8_t  tb_rinse_settling;     /* 1 = 램프/정착 구간(측정 전)*/

/* ---- 원샷 결과 (구동이 끝나는 순간 래치. 다음 원샷 전까지 유지) --------- */
extern volatile uint32_t tb_rinse_res_edges;    /* 측정 구간 엣지 수          */
extern volatile uint32_t tb_rinse_res_ms;       /* 측정 구간 길이(ms)         */
extern volatile uint32_t tb_rinse_res_epr_x100; /* ★1회전당 엣지 수 × 100     */
extern volatile uint16_t tb_rinse_res_rpm;      /* 계산에 쓴 RPM(지령)        */
extern volatile uint8_t  tb_rinse_res_both;     /* 계산에 쓴 엣지 정의        */
/* ★2026-09-21 추가 (1차 실측에서 드러난 한계 보완) */
extern volatile uint32_t tb_rinse_res_period_ms;   /* 창 안 평균 엣지 주기(ms) - 정밀값 */
extern volatile uint16_t tb_rinse_res_meas_rpm;    /* BLDC 실측 RPM 평균(meas_out_rpm = 날개 rpm, 1:82×6/5) */
extern volatile uint32_t tb_rinse_res_outrev_x100; /* 엣지 1개당 출력축 회전수 × 100   */

/* ★P54 직접 실측 (2026-09-21, 기록지 §11) — **상승엣지(both=0) 에서만** 값이 나온다.
 * 첫~마지막 가이드 엣지 사이(= 정확히 revs 회전)의 M2 FG 증가량으로 계산한다. */
extern volatile uint32_t tb_rinse_res_fg_edges;       /* 그 구간 FG 하강엣지 증가량      */
extern volatile uint32_t tb_rinse_res_revs;           /* 그 구간 출력축 회전수(=edges-1) */
extern volatile uint32_t tb_rinse_res_fg_per_rev_x10; /* ★P54 × 10 = FG엣지 / 출력 1회전 */
extern volatile uint32_t tb_rinse_res_pp_x_gear_x10;  /* (위 ÷ 3) × 10 = 극쌍 × 감속비   */
extern volatile uint32_t tb_rinse_fg_now;             /* M2 FG 누적 실시간(돌 때 늘어야 함) */

/* ---- 설정 (디버거에서 쓴다) --------------------------------------------- */
/* 엣지 정의. 0=상승엣지만(자석 도착) / 1=양엣지. 쓰면 즉시 GuideEdge 에 반영.
 * ★TBD: I02 — 이 벤치로 확정한다. */
extern volatile uint8_t  tb_rinse_both_edges;

extern volatile uint16_t tb_rinse_spin_rpm;     /* 교반 출력축 RPM (기본 30)  */
extern volatile uint8_t  tb_rinse_spin_rev;     /* 0=정회전 1=역회전          */
extern volatile uint32_t tb_rinse_spin_ms;      /* 원샷 **총** 길이 (기본 15000) */
/* 램프/정착 구간. 이 시간이 지난 뒤부터 측정한다(그 전 엣지는 버린다).
 * slew 20rpm/s 라 30rpm 까지 1.5초 + PI 정착 - 3초면 넉넉하다. */
extern volatile uint32_t tb_rinse_spin_settle_ms;  /* 기본 3000 */

/* ---- 원샷 커맨드 (1 을 쓰면 실행 후 스스로 0) --------------------------- */
extern volatile uint8_t  tb_rinse_mark_once;    /* baseline·통계 리셋(수동측정용) */
extern volatile uint8_t  tb_rinse_spin_once;    /* 교반 원샷 구동 시작        */
extern volatile uint8_t  tb_rinse_stop_once;    /* 구동 즉시 중단             */

/* ---- 패턴 모드 (run E) — 설정 ------------------------------------------ */
extern volatile uint16_t tb_rinse_pat_rpm;         /* 기본 DJ_RINSE_STIR_RPM(30)   */
extern volatile uint32_t tb_rinse_pat_fwd_ms;      /* 기본 DJ_RINSE_CW_MS(3000)    */
extern volatile uint32_t tb_rinse_pat_stop_ms;     /* 기본 DJ_RINSE_STOP_MS(1000) - 정지 2곳 공용 */
extern volatile uint32_t tb_rinse_pat_rev_ms;      /* 기본 DJ_RINSE_CCW_MS(3000)   */
extern volatile uint8_t  tb_rinse_pat_group;       /* 정회전 묶음. 헹굼 1 / 건조 5  */
extern volatile uint32_t tb_rinse_pat_ms;          /* 패턴 최대 길이 (기본 360000) */
extern volatile uint8_t  tb_rinse_pat_park;        /* 1 = 자석 위에 먼저 세움      */
extern volatile uint32_t tb_rinse_pat_park_max_ms; /* 자석 찾기 상한 (기본 8000)   */
extern volatile uint8_t  tb_rinse_pat_stop_at40;   /* 1 = cnt 40 도달 시 종료      */
extern volatile uint8_t  tb_rinse_pat_once;        /* ★시작 원샷                   */

/* ---- 패턴 모드 — 관측 (종료 시 래치, 다음 시작 전까지 유지) ------------- */
extern volatile uint8_t  tb_rinse_pat_state;        /* TB_RINSE_PAT_*              */
extern volatile uint8_t  tb_rinse_pat_result;       /* TB_RINSE_PAT_RES_*          */
extern volatile uint8_t  tb_rinse_pat_phase;        /* 0=정 1=정지 2=역 3=정지     */
extern volatile uint32_t tb_rinse_pat_edges;        /* 패턴 시작 이후 엣지 (park 제외) */
extern volatile uint32_t tb_rinse_pat_cycles;       /* 완료한 4구간 cycle 수       */
extern volatile uint32_t tb_rinse_pat_elapsed_ms;   /* 패턴 경과                   */
extern volatile uint32_t tb_rinse_pat_t20_ms;       /* cnt 20 도달 시각 (0=미도달) */
extern volatile uint32_t tb_rinse_pat_t40_ms;       /* cnt 40 도달 시각 (0=미도달) */
extern volatile uint32_t tb_rinse_pat_max_gap_ms;   /* 최장 무엣지 구간(실시간 증가) */
extern volatile uint32_t tb_rinse_pat_zero_cycles;  /* 엣지 0개였던 cycle 수        */
extern volatile uint32_t tb_rinse_pat_max_zero_run; /* 0엣지 cycle 최장 연속        */
extern volatile uint32_t tb_rinse_pat_epc_x100;     /* ★1cycle 당 엣지 × 100       */
extern volatile uint32_t tb_rinse_pat_park_ms;      /* park 에 걸린 시간           */
extern volatile uint8_t  tb_rinse_pat_park_level;   /* park 정지 직후 레벨(1=자석 위) */

/* ---- API ---------------------------------------------------------------- */
void TB_Rinse_Init(void);

/* 1ms, StartMotorTask 의 테스트벤치 분기. bench_idle=0 이면 구동 커맨드를
 * 받지 않고 관측만 갱신한다(시나리오 중 교반을 가로채지 않기 위함). */
void TB_Rinse_Poll(uint8_t bench_idle);

#ifdef __cplusplus
}
#endif

#endif /* SRC_TB_RINSE_H_ */
