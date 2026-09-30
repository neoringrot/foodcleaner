#include "tb_rinse.h"
#include "bldc_ctrl.h"
#include "jungji.h"
#include "dongjak.h"   /* DJ_RINSE_* - 패턴 모드 기본값의 단일 출처(상수만 쓴다) */
#include "drv8306.h"   /* drv8306_m2.fg_edges (읽기만) · DRV8306_FG_EDGES_PER_ELEC_REV */
#include "tb_rotation.h" /* TB_Rotation_Busy() - run F 와 교반 배타 */

/* 5단계 벤치 - 가이드 엣지 계수 실측. 목적·절차는 tb_rinse.h 참조.
 * 계수 자체는 guide_edge.c 가 하고(모드와 무관하게 freertos.c 가 Tick 을 돈다),
 * 이 파일은 baseline/통계와 원샷 교반 구동만 얹는다. */

volatile uint32_t tb_rinse_count        = 0;
volatile uint32_t tb_rinse_since_mark   = 0;
volatile uint32_t tb_rinse_last_edge_ms = 0;
volatile uint32_t tb_rinse_interval_ms  = 0;
volatile uint32_t tb_rinse_int_min_ms   = 0;
volatile uint32_t tb_rinse_int_max_ms   = 0;
volatile uint32_t tb_rinse_mark_ms      = 0;
volatile uint32_t tb_rinse_elapsed_ms   = 0;
volatile uint8_t  tb_rinse_level        = 0;
volatile uint8_t  tb_rinse_spinning     = 0;
volatile uint8_t  tb_rinse_settling     = 0;

volatile uint32_t tb_rinse_res_edges    = 0;
volatile uint32_t tb_rinse_res_ms       = 0;
volatile uint32_t tb_rinse_res_epr_x100 = 0;
volatile uint16_t tb_rinse_res_rpm      = 0;
volatile uint8_t  tb_rinse_res_both     = 0;
volatile uint32_t tb_rinse_res_period_ms   = 0;
volatile uint16_t tb_rinse_res_meas_rpm    = 0;
volatile uint32_t tb_rinse_res_outrev_x100 = 0;
volatile uint32_t tb_rinse_res_fg_edges       = 0;
volatile uint32_t tb_rinse_res_revs           = 0;
volatile uint32_t tb_rinse_res_fg_per_rev_x10 = 0;
volatile uint32_t tb_rinse_res_pp_x_gear_x10  = 0;
volatile uint32_t tb_rinse_fg_now             = 0;

volatile uint8_t  tb_rinse_both_edges   = (uint8_t)GUIDE_EDGE_BOTH;
volatile uint16_t tb_rinse_spin_rpm     = 30U;
volatile uint8_t  tb_rinse_spin_rev     = 0;
volatile uint32_t tb_rinse_spin_ms         = 15000U;
volatile uint32_t tb_rinse_spin_settle_ms  = 3000U;

volatile uint8_t  tb_rinse_mark_once    = 0;
volatile uint8_t  tb_rinse_spin_once    = 0;
volatile uint8_t  tb_rinse_stop_once    = 0;

/* ---- 패턴 모드 (run E) - 설정 ------------------------------------------ */
volatile uint16_t tb_rinse_pat_rpm         = (uint16_t)DJ_RINSE_STIR_RPM;   /* 30   */
volatile uint32_t tb_rinse_pat_fwd_ms      = (uint32_t)DJ_RINSE_CW_MS;      /* 3000 */
volatile uint32_t tb_rinse_pat_stop_ms     = (uint32_t)DJ_RINSE_STOP_MS;    /* 1000 */
volatile uint32_t tb_rinse_pat_rev_ms      = (uint32_t)DJ_RINSE_CCW_MS;     /* 3000 */
volatile uint8_t  tb_rinse_pat_group       = 1U;
volatile uint32_t tb_rinse_pat_ms          = 360000U;
volatile uint8_t  tb_rinse_pat_park        = 1U;
volatile uint32_t tb_rinse_pat_park_max_ms = 8000U;
volatile uint8_t  tb_rinse_pat_stop_at40   = 1U;
volatile uint8_t  tb_rinse_pat_once        = 0;

/* ---- 패턴 모드 - 관측 ---------------------------------------------------- */
volatile uint8_t  tb_rinse_pat_state       = TB_RINSE_PAT_IDLE;
volatile uint8_t  tb_rinse_pat_result      = TB_RINSE_PAT_RES_NONE;
volatile uint8_t  tb_rinse_pat_phase       = 0;
volatile uint32_t tb_rinse_pat_edges       = 0;
volatile uint32_t tb_rinse_pat_cycles      = 0;
volatile uint32_t tb_rinse_pat_elapsed_ms  = 0;
volatile uint32_t tb_rinse_pat_t20_ms      = 0;
volatile uint32_t tb_rinse_pat_t40_ms      = 0;
volatile uint32_t tb_rinse_pat_max_gap_ms  = 0;
volatile uint32_t tb_rinse_pat_zero_cycles = 0;
volatile uint32_t tb_rinse_pat_max_zero_run = 0;
volatile uint32_t tb_rinse_pat_epc_x100    = 0;
volatile uint32_t tb_rinse_pat_park_ms     = 0;
volatile uint8_t  tb_rinse_pat_park_level  = 0;

static uint32_t s_mark_count;      /* mark 시점의 GuideEdge_Count()          */
static uint32_t s_seen_count;      /* 통계 갱신에 쓴 마지막 카운트           */
static uint32_t s_spin_since;      /* 원샷 구동 시작 tick                    */
static uint8_t  s_both_applied;    /* GuideEdge 에 반영한 마지막 엣지 정의   */
static uint16_t s_spin_rpm_used;   /* 이번 원샷의 RPM(계산에 쓴다)           */
static uint32_t s_first_edge_ms;   /* mark 이후 첫 엣지 tick                 */
static uint8_t  s_have_first;      /* 0 = mark 이후 엣지 아직 없음           */
static uint32_t s_rpm_sum;         /* 측정 구간 meas_out_rpm 누적            */
static uint32_t s_rpm_n;           /* 누적 샘플 수                           */
static uint32_t s_fg_at_first;     /* mark 이후 첫 가이드 엣지 순간의 M2 FG 카운트 */
static uint32_t s_fg_at_last;      /* 마지막 가이드 엣지 순간의 M2 FG 카운트  */

/* 패턴 모드 내부 상태 */
static uint32_t s_pat_since;       /* 현 단계(park/hold/pattern) 시작 tick   */
static uint32_t s_pat_start;       /* 패턴 구동 시작 tick (=회차시계 기산)   */
static uint32_t s_pat_base;        /* 패턴 시작 시 GuideEdge_Count() (C009)  */
static uint32_t s_pat_seen;        /* 통계에 반영한 마지막 패턴 엣지 수      */
static uint32_t s_pat_gap_ref;     /* 간격 기준: 직전 엣지 tick(첫 엣지 전엔 시작 tick) */
static uint32_t s_pat_cyc_edges;   /* 현 1cycle 시작 시점의 패턴 엣지 수     */
static uint32_t s_pat_zero_run;    /* 연속 0엣지 cycle 수                    */
static uint32_t s_alt_since;       /* 4구간 현 phase 시작 tick               */
static uint8_t  s_alt_reps;        /* 정회전 묶음 카운터                     */
static uint32_t s_last_poll;       /* 직전 Poll tick - 호출 공백(벤치 이탈) 검출용 */

/* Poll 이 이보다 오래 안 불렸으면 그 사이 벤치를 벗어났던 것으로 본다.
 * TB_Rinse_Poll() 은 freertos.c 의 **테스트벤치 분기에서만** 불린다 - 시나리오
 * 모드에선 아예 호출되지 않으므로 인자 bench_idle 은 늘 1 이다. 그래서 이탈은
 * "호출이 끊겼다" 로만 알 수 있다. 1ms 루프라 50ms 면 넉넉하다. */
#define TB_RINSE_POLL_GAP_MS   50U

static void tb_rinse_stir_stop(void)
{
	BldcCtrl_Stop(&g_stir_ctrl);
	tb_rinse_spinning = 0U;
	tb_rinse_settling = 0U;
}

/* ======================================================================
 * 패턴 모드 (run E) — 실제 헹굼 4구간 패턴에서 cnt 가 진행하는가
 * ====================================================================== */

/* 교반 지령은 dongjak.c 의 dj_stir_spin() 과 **똑같이** 낸다 — 스트로크마다
 * Stop -> Start -> target. 그래야 매 스트로크가 0 에서 램프(slew 20rpm/s)로
 * 다시 올라가는 실제 헹굼과 같은 회전각이 나온다. (연속 회전 모드와 다른 점.) */
static void tb_rinse_pat_apply(uint8_t phase, uint16_t rpm)
{
	if ((phase == 0U) || (phase == 2U))
	{
		BldcCtrl_Stop(&g_stir_ctrl);
		BldcCtrl_Start(&g_stir_ctrl, (phase == 2U) ? 1U : 0U);
		g_stir_ctrl.target_out_rpm = rpm;
	}
	else
	{
		BldcCtrl_Stop(&g_stir_ctrl);                /* 1·3 = 정지 */
	}
}

/* 패턴 종료 — 어떤 이유로 끝나든 지표는 래치한다(중단해도 그때까지의 값을 본다). */
static void tb_rinse_pat_finish(uint8_t result)
{
	BldcCtrl_Stop(&g_stir_ctrl);
	tb_rinse_pat_result   = result;
	tb_rinse_pat_state    = TB_RINSE_PAT_DONE;
	tb_rinse_pat_epc_x100 = (tb_rinse_pat_cycles != 0U)
	                      ? ((tb_rinse_pat_edges * 100U) / tb_rinse_pat_cycles)
	                      : 0U;
}

static void tb_rinse_pat_begin_pattern(uint32_t now_ms)
{
	/* C009: 준비(park) 구간의 감지는 cnt 에 넣지 않는다 - 여기서 baseline 을 다시 뜬다. */
	s_pat_base      = GuideEdge_Count();
	s_pat_seen      = 0U;
	s_pat_start     = now_ms;
	s_pat_gap_ref   = now_ms;     /* 첫 엣지까지의 대기도 '간격'으로 센다 */
	s_pat_cyc_edges = 0U;
	s_pat_zero_run  = 0U;
	s_alt_reps      = 0U;
	s_alt_since     = now_ms;
	tb_rinse_pat_phase = 0U;
	tb_rinse_pat_state = TB_RINSE_PAT_RUN;
	tb_rinse_pat_apply(0U, tb_rinse_pat_rpm);
}

static void tb_rinse_pat_start(uint32_t now_ms)
{
	tb_rinse_pat_result       = TB_RINSE_PAT_RES_NONE;
	tb_rinse_pat_edges        = 0U;
	tb_rinse_pat_cycles       = 0U;
	tb_rinse_pat_elapsed_ms   = 0U;
	tb_rinse_pat_t20_ms       = 0U;
	tb_rinse_pat_t40_ms       = 0U;
	tb_rinse_pat_max_gap_ms   = 0U;
	tb_rinse_pat_zero_cycles  = 0U;
	tb_rinse_pat_max_zero_run = 0U;
	tb_rinse_pat_epc_x100     = 0U;
	tb_rinse_pat_park_ms      = 0U;
	tb_rinse_pat_park_level   = 0U;
	s_pat_since               = now_ms;

	if (tb_rinse_pat_park)
	{
		/* 준비 탐색(C006/C007) 흉내: 정회전으로 돌다가 **감지 즉시 정지**.
		 * 실제 헹굼에서 본 세척은 이렇게 '자석 바로 위'에서 출발한다. */
		s_pat_base = GuideEdge_Count();
		tb_rinse_pat_state = TB_RINSE_PAT_PARK;
		tb_rinse_pat_apply(0U, tb_rinse_pat_rpm);
	}
	else
	{
		tb_rinse_pat_begin_pattern(now_ms);         /* 현재 위치에서 바로 출발 */
	}
}

/* 레퍼런스 alternate() / dongjak.c dj_stir_alt_tick() 과 같은 전이 규칙.
 *   0=정 -> 1=정지 -> (묶음 미달이면 0) -> 2=역 -> 3=정지 -> 0 (cycle++) */
static void tb_rinse_pat_alt_tick(uint32_t now_ms)
{
	uint8_t  ph  = tb_rinse_pat_phase;
	uint32_t dur = (ph == 0U) ? tb_rinse_pat_fwd_ms
	             : (ph == 2U) ? tb_rinse_pat_rev_ms
	                          : tb_rinse_pat_stop_ms;
	if ((now_ms - s_alt_since) < dur) { return; }

	s_alt_since = now_ms;
	if ((ph == 1U) && (++s_alt_reps < tb_rinse_pat_group))
	{
		ph = 0U;
	}
	else if (ph == 3U)
	{
		ph = 0U; s_alt_reps = 0U;
		/* 1cycle 완료 - 이번 cycle 에 엣지가 하나도 없었는지 본다(정체 검출). */
		tb_rinse_pat_cycles++;
		if (tb_rinse_pat_edges == s_pat_cyc_edges)
		{
			tb_rinse_pat_zero_cycles++;
			s_pat_zero_run++;
			if (s_pat_zero_run > tb_rinse_pat_max_zero_run)
			{ tb_rinse_pat_max_zero_run = s_pat_zero_run; }
		}
		else
		{
			s_pat_zero_run = 0U;
		}
		s_pat_cyc_edges = tb_rinse_pat_edges;
	}
	else
	{
		ph++;
	}
	tb_rinse_pat_phase = ph;
	tb_rinse_pat_apply(ph, tb_rinse_pat_rpm);
}

static void tb_rinse_pat_tick(uint32_t now_ms, uint8_t bench_idle)
{
	if ((tb_rinse_pat_state == TB_RINSE_PAT_IDLE) ||
	    (tb_rinse_pat_state == TB_RINSE_PAT_DONE)) { return; }

	/* 벤치를 벗어났다 왔으면(Poll 공백) 즉시 중단 - 그 사이 시나리오가 교반을 썼다. */
	if (!bench_idle) { tb_rinse_pat_finish(TB_RINSE_PAT_RES_ABORT); return; }

	switch (tb_rinse_pat_state)
	{
	case TB_RINSE_PAT_PARK:                         /* 자석 찾기 */
		if (GuideEdge_Count() != s_pat_base)
		{
			BldcCtrl_Stop(&g_stir_ctrl);            /* 감지 즉시 정지(C007) */
			tb_rinse_pat_park_ms    = now_ms - s_pat_since;
			tb_rinse_pat_park_level = GuideEdge_Level();
			tb_rinse_pat_state      = TB_RINSE_PAT_HOLD;
			s_pat_since             = now_ms;
		}
		else if ((now_ms - s_pat_since) >= tb_rinse_pat_park_max_ms)
		{
			tb_rinse_pat_finish(TB_RINSE_PAT_RES_PARK_FAIL);   /* 가이드 못 찾음 */
		}
		break;

	case TB_RINSE_PAT_HOLD:                         /* 정지 유지 후 본 세척 */
		if ((now_ms - s_pat_since) >= tb_rinse_pat_stop_ms)
		{
			tb_rinse_pat_begin_pattern(now_ms);
		}
		break;

	case TB_RINSE_PAT_RUN:
	default:
	{
		uint32_t edges = GuideEdge_Count() - s_pat_base;   /* uint32 modular */
		uint32_t el    = now_ms - s_pat_start;

		tb_rinse_pat_edges      = edges;
		tb_rinse_pat_elapsed_ms = el;

		if (edges != s_pat_seen)                    /* 새 엣지 */
		{
			uint32_t last = GuideEdge_LastEdgeMs();
			uint32_t gap  = last - s_pat_gap_ref;
			if (gap > tb_rinse_pat_max_gap_ms) { tb_rinse_pat_max_gap_ms = gap; }
			s_pat_gap_ref = last;
			s_pat_seen    = edges;
			if ((edges >= TB_RINSE_WASH_EDGES)  && (tb_rinse_pat_t20_ms == 0U))
			{ tb_rinse_pat_t20_ms = last - s_pat_start; }
			if ((edges >= TB_RINSE_TOTAL_EDGES) && (tb_rinse_pat_t40_ms == 0U))
			{ tb_rinse_pat_t40_ms = last - s_pat_start; }
		}
		else if ((now_ms - s_pat_gap_ref) > tb_rinse_pat_max_gap_ms)
		{
			/* 엣지가 안 오는 동안에도 최대 간격을 키운다 - 정체가 실시간으로 보이게. */
			tb_rinse_pat_max_gap_ms = now_ms - s_pat_gap_ref;
		}

		if (tb_rinse_pat_stop_at40 && (edges >= TB_RINSE_TOTAL_EDGES))
		{
			tb_rinse_pat_finish(TB_RINSE_PAT_RES_REACHED);
		}
		else if (el >= tb_rinse_pat_ms)
		{
			tb_rinse_pat_finish((edges >= TB_RINSE_TOTAL_EDGES)
			                    ? TB_RINSE_PAT_RES_REACHED : TB_RINSE_PAT_RES_TIMEOUT);
		}
		else
		{
			tb_rinse_pat_alt_tick(now_ms);
		}
		break;
	}
	}
}

/* 원샷 측정 구간을 닫고 1회전당 엣지 수를 래치한다.
 *   rev  = res_ms * rpm / 60000
 *   epr  = edges / rev = edges * 60000 / (res_ms * rpm)
 * 정수 나눗셈 오차를 피하려고 64bit 로 한 번에 계산한다(벤치라 비용 무관). */
static void tb_rinse_latch(uint32_t now_ms)
{
	uint32_t edges = GuideEdge_Count() - s_mark_count;
	uint32_t ms    = now_ms - tb_rinse_mark_ms;
	uint64_t den   = (uint64_t)ms * (uint64_t)s_spin_rpm_used;

	tb_rinse_res_edges    = edges;
	tb_rinse_res_ms       = ms;
	tb_rinse_res_rpm      = s_spin_rpm_used;
	tb_rinse_res_both     = s_both_applied;
	tb_rinse_res_epr_x100 = (den != 0ULL)
	                      ? (uint32_t)(((uint64_t)edges * 6000000ULL) / den)
	                      : 0U;

	/* ★엣지 **주기** — 개수(정수)로 나눈 epr 은 12초 창에서 4엣지면 ±25% 로 거칠다
	 * (2026-09-21 1차 실측: 66 vs 간격 기준 64.5). 창 안의 첫/끝 엣지 사이를
	 * (엣지-1) 로 나누면 창 길이와 무관하게 정밀하다. 양엣지면 펄스폭/간격이 번갈아
	 * 섞인 평균(=반주기)이 나온다. */
	tb_rinse_res_period_ms = (s_have_first && (edges >= 2U))
	                       ? ((GuideEdge_LastEdgeMs() - s_first_edge_ms) / (edges - 1U))
	                       : 0U;

	/* ★BLDC 가 스스로 잰 날개 RPM 평균(FG x 6 / 410, §0.22). 지령과 비교해 PI 가
	 * 목표를 따라갔는지 본다. §0.22 전(1:49 가정)에는 이 값이 날개를 1.43배 크게
	 * 보여 run D 에서 26 으로 읽혔다(실제 날개 20). */
	tb_rinse_res_meas_rpm = (s_rpm_n != 0U) ? (uint16_t)(s_rpm_sum / s_rpm_n) : 0U;

	/* 가이드 엣지 1개당 (BLDC 가 보는) 날개 회전수 × 100 = meas_rpm × period / 600.
	 * ★§0.22 이후 **≈100** 이어야 한다 — BLDC 환산이 실제 날개와 맞는다는 자가검증.
	 * (§0.22 전: 131 — 1:49 환산 오차가 그대로 드러났던 값.) 상승엣지 기준. */
	tb_rinse_res_outrev_x100 = (uint32_t)(((uint64_t)tb_rinse_res_meas_rpm *
	                                       (uint64_t)tb_rinse_res_period_ms) / 600ULL);

	/* ★P54 직접 실측 (2026-09-21) — 출력축 1회전당 M2 FG 하강엣지 수.
	 * run D 로 가이드 상승엣지 1개 = 출력축 정확히 1회전임이 확인됐다(기록지 §10).
	 * 그래서 **첫 가이드 엣지 ~ 마지막 가이드 엣지** 사이는 정확히 (edges-1) 회전이고,
	 * 그 사이의 FG 증가량을 회전수로 나누면 FG엣지/회전이 나온다. 창 시작·끝(임의
	 * 각도)을 쓰지 않으므로 반바퀴 오차가 없다.
	 *   순수 기어열이면 이 값은 **정수** = 3 × pole_pairs × gear_ratio.
	 *   ×10 으로 보여 주는 이유: 7650 처럼 끝자리가 0 이면 정수가 맞다는 확인이 된다.
	 * 양엣지 모드면 엣지 사이가 반바퀴가 아니라 펄스폭/간격이라 회전수가 성립하지
	 * 않는다 → **상승엣지(both=0)일 때만** 계산하고 아니면 0. */
	if (s_have_first && (edges >= 2U) && (s_both_applied == 0U))
	{
		uint32_t revs = edges - 1U;
		uint32_t fg   = s_fg_at_last - s_fg_at_first;          /* uint32 modular */
		tb_rinse_res_fg_edges       = fg;
		tb_rinse_res_revs           = revs;
		tb_rinse_res_fg_per_rev_x10 = (uint32_t)(((uint64_t)fg * 10ULL) / revs);
		tb_rinse_res_pp_x_gear_x10  = (uint32_t)(((uint64_t)fg * 10ULL) /
		                              ((uint64_t)revs * (uint64_t)DRV8306_FG_EDGES_PER_ELEC_REV));
	}
	else
	{
		tb_rinse_res_fg_edges       = 0U;
		tb_rinse_res_revs           = 0U;
		tb_rinse_res_fg_per_rev_x10 = 0U;
		tb_rinse_res_pp_x_gear_x10  = 0U;
	}
}

static void tb_rinse_stir_start(uint8_t rev, uint16_t rpm, uint32_t now_ms)
{
	BldcCtrl_Stop(&g_stir_ctrl);
	BldcCtrl_Start(&g_stir_ctrl, rev);
	g_stir_ctrl.target_out_rpm = rpm;
	s_spin_since      = now_ms;
	s_spin_rpm_used   = rpm;
	tb_rinse_spinning = 1U;
	tb_rinse_settling = 1U;    /* 램프/정착 구간 - 아직 측정 안 한다 */
}

static void tb_rinse_mark(uint32_t now_ms)
{
	s_mark_count          = GuideEdge_Count();
	s_seen_count          = s_mark_count;
	tb_rinse_since_mark   = 0U;
	tb_rinse_int_min_ms   = 0U;
	tb_rinse_int_max_ms   = 0U;
	tb_rinse_mark_ms      = now_ms;
	tb_rinse_elapsed_ms   = 0U;
	s_have_first          = 0U;
	s_first_edge_ms       = 0U;
	s_rpm_sum             = 0U;
	s_rpm_n               = 0U;
	s_fg_at_first         = 0U;
	s_fg_at_last          = 0U;
}

void TB_Rinse_Init(void)
{
	s_mark_count   = 0U;
	s_seen_count   = 0U;
	s_spin_since    = 0U;
	s_both_applied  = (uint8_t)GUIDE_EDGE_BOTH;
	s_spin_rpm_used = 0U;
	s_last_poll     = 0U;
	tb_rinse_pat_state  = TB_RINSE_PAT_IDLE;
	tb_rinse_pat_result = TB_RINSE_PAT_RES_NONE;
	tb_rinse_stir_stop();
}

void TB_Rinse_Poll(uint8_t bench_idle)
{
	uint32_t now = HAL_GetTick();
	uint32_t cnt = GuideEdge_Count();

	/* ★2026-09-21: 벤치 이탈 검출. 측정 중 마개가 모음/동작 위치로 가면 이 함수가
	 * 불리지 않다가, 돌아오는 순간 오래된 s_spin_since / s_pat_start 로 '엄청 긴
	 * 측정'을 래치했다(엉터리 epr). 모터는 모드 전환 때 Jungji_StopAll 이 이미 세웠으므로
	 * 여기서는 **측정만 무효화**한다 - 결과는 ABORT 로 남기고 연속 모드는 래치하지 않는다. */
	if ((s_last_poll != 0U) && ((now - s_last_poll) > TB_RINSE_POLL_GAP_MS))
	{
		bench_idle = 0U;
	}
	s_last_poll = now;

	/* ---- 관측 갱신 (모드 무관) ------------------------------------- */
	tb_rinse_count        = cnt;
	tb_rinse_level        = GuideEdge_Level();
	tb_rinse_last_edge_ms = GuideEdge_LastEdgeMs();
	tb_rinse_interval_ms  = GuideEdge_LastIntervalMs();
	tb_rinse_since_mark   = cnt - s_mark_count;      /* uint32 modular */
	tb_rinse_fg_now       = drv8306_m2.fg_edges;     /* M2 FG 누적(ISR) - 살아 있는지 확인용 */
	if (tb_rinse_mark_ms != 0U) { tb_rinse_elapsed_ms = now - tb_rinse_mark_ms; }

	/* 새 엣지가 들어온 사이클에만 간격 min/max 를 갱신한다.
	 * ★2026-09-21 수정: **직전 엣지가 mark 이후인 간격만** 센다. 전에는 mark 뒤 첫
	 * 엣지의 간격이 "이전 run 의 마지막 엣지 ~ 지금" 이라 run 사이 대기시간이 섞였다
	 * (1차 실측 run B int_max 116776 / run C 57425 가 그것 - 측정값이 아니다). */
	if (cnt != s_seen_count)
	{
		uint32_t iv   = GuideEdge_LastIntervalMs();
		uint32_t last = GuideEdge_LastEdgeMs();
		s_seen_count = cnt;
		/* 가이드 엣지 순간의 FG 카운트 스냅샷. GuideEdge_Tick 이 같은 1ms 루프 앞에서
		 * 엣지를 잡으므로 지연은 ≤1~2ms - 실제 ≈19.8rpm 에서 FG ≈0.25엣지/ms 라
		 * 스냅샷당 ±1엣지, 3회전(≈2300엣지) 기준 0.1% 미만이다. */
		s_fg_at_last = drv8306_m2.fg_edges;
		if (!s_have_first) { s_first_edge_ms = last; s_have_first = 1U; s_fg_at_first = s_fg_at_last; }
		if ((iv != 0U) && ((int32_t)((last - iv) - tb_rinse_mark_ms) >= 0))
		{
			if ((tb_rinse_int_min_ms == 0U) || (iv < tb_rinse_int_min_ms))
			{ tb_rinse_int_min_ms = iv; }
			if (iv > tb_rinse_int_max_ms) { tb_rinse_int_max_ms = iv; }
		}
	}

	/* ---- 엣지 정의 전환 (I02 실측) --------------------------------- */
	if (tb_rinse_both_edges != s_both_applied)
	{
		s_both_applied = (uint8_t)(tb_rinse_both_edges ? 1U : 0U);
		tb_rinse_both_edges = s_both_applied;
		GuideEdge_SetBothEdges(s_both_applied);
	}

	/* ---- 원샷 커맨드 ------------------------------------------------ */
	if (tb_rinse_mark_once) { tb_rinse_mark_once = 0U; tb_rinse_mark(now); }

	if (tb_rinse_stop_once)
	{
		tb_rinse_stop_once = 0U;
		tb_rinse_stir_stop();
		if ((tb_rinse_pat_state != TB_RINSE_PAT_IDLE) &&
		    (tb_rinse_pat_state != TB_RINSE_PAT_DONE))
		{
			tb_rinse_pat_finish(TB_RINSE_PAT_RES_ABORT);   /* 그때까지 지표는 래치 */
		}
	}

	/* 두 모드는 배타다 - 하나가 도는 중이면 다른 쪽 시작 커맨드는 버린다. */
	uint8_t pat_busy = (uint8_t)((tb_rinse_pat_state != TB_RINSE_PAT_IDLE) &&
	                             (tb_rinse_pat_state != TB_RINSE_PAT_DONE));

	if (tb_rinse_pat_once)
	{
		tb_rinse_pat_once = 0U;
		if (bench_idle && !Jungji_IsBraking() && !tb_rinse_spinning && !pat_busy &&
		    !TB_Rotation_Busy())
		{
			tb_rinse_pat_start(now);
			pat_busy = 1U;
		}
	}

	/* 구동 커맨드는 벤치 유휴에서만 받는다 - 시나리오가 교반을 쓰는 중에
	 * 가로채면 안 된다. 제동 홀드 구간에서도 기동하지 않는다. */
	if (tb_rinse_spin_once)
	{
		tb_rinse_spin_once = 0U;
		if (bench_idle && !Jungji_IsBraking() && !pat_busy && !TB_Rotation_Busy())
		{
			/* mark 는 여기서 뜨지 않는다 - 램프 구간을 재면 RPM 이 30 이 아니라
			 * 1회전당 엣지 수가 실제보다 작게 나온다. settle 이 지난 뒤 자동으로
			 * 뜬다(아래). 그전까지의 엣지는 버린다. */
			tb_rinse_mark(now);                      /* 관측용 baseline(임시) */
			tb_rinse_stir_start(tb_rinse_spin_rev, tb_rinse_spin_rpm, now);
		}
	}

	/* ---- 원샷 진행: 정착 -> 측정 -> 래치 ---------------------------- */
	if (tb_rinse_spinning)
	{
		uint32_t el = now - s_spin_since;

		if (tb_rinse_settling && (el >= tb_rinse_spin_settle_ms))
		{
			tb_rinse_settling = 0U;
			tb_rinse_mark(now);                      /* 여기서부터가 측정 구간 */
		}
		if (!tb_rinse_settling)                      /* 측정 구간: 실측 RPM 누적 */
		{
			s_rpm_sum += g_stir_ctrl.meas_out_rpm;
			s_rpm_n++;
		}
		/* 벤치를 벗어났다 왔으면(Poll 공백) 측정은 버리고 정지한다 - 래치하지 않는다. */
		if (!bench_idle)
		{
			tb_rinse_stir_stop();
		}
		else if (el >= tb_rinse_spin_ms)
		{
			if (!tb_rinse_settling) { tb_rinse_latch(now); }  /* 정착 전에 끝나면 결과 없음 */
			tb_rinse_stir_stop();
		}
	}

	/* ---- 패턴 모드 진행 -------------------------------------------- */
	tb_rinse_pat_tick(now, bench_idle);
}
