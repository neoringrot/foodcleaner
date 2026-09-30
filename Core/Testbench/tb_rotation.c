#include "tb_rotation.h"
#include <stdbool.h>
#include "guide_edge.h"
#include "bldc_ctrl.h"
#include "drv8306.h"    /* drv8306_m2.fg_edges (읽기만) — 운전당 FG 교차검증(P54) */
#include "jungji.h"
#include "tb_rinse.h"   /* 교반 배타 */

/* 설계·절차는 tb_rotation.h 머리말. 엔진 원본은 Core/Scenario/rotation.c (벤더 0.3.0). */

/* ---- A. 자가시험 결과 ---- */
volatile uint8_t  tb_rotation_selftest_once;
volatile uint8_t  tb_rotation_selftest_done;
volatile uint32_t tb_rotation_selftest_asserts;
volatile uint32_t tb_rotation_selftest_fail_line;

/* ==== A. 자가시험 — 벤더 tests/test_rotation.c 기계 변환 ==================
 * 원본 sha256 c750f193407d2f04fff222a1fb3ba73ba4efc17f6a9993f661cca0c0ac844f99 (0.3.0).
 * 바꾼 것은 셋뿐이다: ① 식별자에 st_ 접두(r/in/now/cfg/run/reset/assertions)
 * ② REQUIRE 가 exit() 대신 첫 실패 줄을 남기고 return ③ `return assertions` 제거.
 * 본문 로직·숫자는 한 글자도 손대지 않았다 — scratchpad/gen_selftest.py 로 생성. */
#define REQUIRE(x) do { ++st_asserts; if (!(x)) { st_fail_line = (uint32_t)__LINE__; return; } } while (0)
static unsigned st_asserts;
static uint32_t st_fail_line;
static zg_rotation st_r;
static const zg_rotation_config st_cfg = {100U,2U,2000U,15000U,30U,10U,2U,5U};
static zg_rotation_input st_in;
static uint32_t st_now;
static uint8_t st_run(zg_rotation_profile p, uint32_t ms) {
    st_now += ms; return zg_rotation_tick(&st_r,p,&st_cfg,&st_in,st_now);
}
static void st_reset(void) {
    zg_rotation_reset(&st_r); st_in=(zg_rotation_input){0U,true,true,true,true,false}; st_now=0U;
}
static void tb_rotation_selftest_body(void) {
    st_asserts=0U;
    for (unsigned speed=0U;speed<3U;speed++) {
        st_reset();uint8_t dir=st_run(ZG_ROT_WASH,0U);
        for (unsigned cycle=0U;cycle<10U;cycle++) {
            uint8_t wanted=cycle==6U||cycle==8U?2U:1U;
            REQUIRE(dir==wanted);
            st_in.position+=wanted==1U?200U:(uint32_t)-200;
            REQUIRE(st_run(ZG_ROT_WASH,3200U+speed*800U)==0U);
            REQUIRE(st_run(ZG_ROT_WASH,1999U)==0U);
            dir=st_run(ZG_ROT_WASH,1U);
        }
        REQUIRE(dir==0U&&st_r.cycle==10U);
        REQUIRE(st_run(ZG_ROT_DRAIN,0U)==0U);
        dir=st_run(ZG_ROT_DRAIN,2000U);
        for (unsigned cycle=0U;cycle<10U;cycle++) {
            REQUIRE(dir==1U);st_in.position+=200U;
            REQUIRE(st_run(ZG_ROT_DRAIN,4800U)==0U);dir=st_run(ZG_ROT_DRAIN,2000U);
        }
        REQUIRE(dir==0U&&st_r.cycle==10U);
    }
    st_reset();st_now=UINT32_MAX-3000U;st_in.position=UINT32_MAX-99U;
    REQUIRE(st_run(ZG_ROT_PROCESS,0U)==1U);st_in.position+=200U;
    REQUIRE(st_run(ZG_ROT_PROCESS,4000U)==0U);REQUIRE(st_run(ZG_ROT_PROCESS,2000U)==1U);
    st_reset();REQUIRE(st_run(ZG_ROT_PROCESS,0U)==1U);
    st_in.position=100U;REQUIRE(st_run(ZG_ROT_PROCESS,4000U)==1U);
    st_r.initial_done=true;st_r.phase=2U;st_in.late_requested=true;
    REQUIRE(st_run(ZG_ROT_PROCESS,1U)==0U);REQUIRE(st_r.late&&st_r.origin==0U);
    st_in.grind_at_rest=false;REQUIRE(st_run(ZG_ROT_PROCESS,2000U)==0U);
    st_in.grind_at_rest=true;REQUIRE(st_run(ZG_ROT_PROCESS,1U)==1U);
    st_in.position=199U;REQUIRE(st_run(ZG_ROT_PROCESS,2000U)==1U);
    st_in.position=200U;REQUIRE(st_run(ZG_ROT_PROCESS,1U)==0U);
    st_reset();REQUIRE(st_run(ZG_ROT_PROCESS,0U)==1U);
    st_in.permitted=false;st_in.position=50U;REQUIRE(st_run(ZG_ROT_PROCESS,1000U)==0U);
    st_in.position=75U;REQUIRE(st_run(ZG_ROT_PROCESS,3000U)==0U);
    st_in.permitted=true;REQUIRE(st_run(ZG_ROT_PROCESS,1U)==1U);
    st_in.position=200U;REQUIRE(st_run(ZG_ROT_PROCESS,3000U)==0U);
    st_reset();REQUIRE(st_run(ZG_ROT_PROCESS,0U)==1U);
    st_in.position=UINT32_MAX;REQUIRE(st_run(ZG_ROT_PROCESS,1U)==0U&&st_r.failed);
    st_reset();REQUIRE(st_run(ZG_ROT_PROCESS,0U)==1U);
    REQUIRE(st_run(ZG_ROT_PROCESS,14999U)==1U);REQUIRE(st_run(ZG_ROT_PROCESS,1U)==0U&&st_r.failed);
    st_reset();REQUIRE(st_run(ZG_ROT_PROCESS,0U)==1U);st_in.position=200U;
    REQUIRE(st_run(ZG_ROT_PROCESS,4000U)==0U);st_in.grind_at_rest=false;
    REQUIRE(st_run(ZG_ROT_PROCESS,15000U)==0U&&st_r.failed);
    st_reset();st_in.valid=false;REQUIRE(st_run(ZG_ROT_WASH,0U)==0U&&st_r.failed);
    st_reset();REQUIRE(st_run(ZG_ROT_PROCESS,0U)==1U);st_in.late_requested=true;
    REQUIRE(st_run(ZG_ROT_PROCESS,1000U)==1U&&!st_r.late); /* Initial CW is not bypassed. */
}
#undef REQUIRE

/* ==== B. run F — 실모터 "2회전 + 정지 2초 × N회" ============================= */

#define TB_ROT_POLL_GAP_MS    50U     /* 이보다 오래 안 불렸으면 벤치 이탈로 본다   */
#define TB_ROT_TIMEOUT_MIN_MS 2001U   /* 엔진 조건: feedback_timeout_ms > stop_ms    */
#define TB_ROT_P54_FG_PER_REV 822U    /* run G 실측 FG엣지/날개 1회전 (§0.22)       */

/* 설정 */
volatile uint8_t  tb_rotation_profile       = 1U;
volatile uint16_t tb_rotation_rpm           = 30U;
volatile uint8_t  tb_rotation_pos_src       = 0U;
volatile uint32_t tb_rotation_ticks_per_rev = 1U;
volatile uint32_t tb_rotation_timeout_ms    = 8000U;
volatile uint8_t  tb_rotation_max_legs      = 36U;
volatile uint8_t  tb_rotation_late          = 0U;
volatile uint8_t  tb_rotation_dir_invert    = 1U;   /* ★N14 확정(2026-09-21): DIR_CW = 위에서 본 반시계 */
volatile uint8_t  tb_rotation_once          = 0U;
volatile uint8_t  tb_rotation_stop_once     = 0U;

/* 결과 */
volatile uint8_t  tb_rotation_state;
volatile uint8_t  tb_rotation_result;
volatile uint8_t  tb_rotation_fail_where;
volatile uint32_t tb_rotation_legs;
volatile uint32_t tb_rotation_dir_mask;
volatile uint32_t tb_rotation_edges;
volatile uint32_t tb_rotation_leg_edges;
volatile uint32_t tb_rotation_leg_ms;
volatile uint32_t tb_rotation_leg_min_ms;
volatile uint32_t tb_rotation_leg_max_ms;
volatile uint32_t tb_rotation_leg_fg;
volatile uint32_t tb_rotation_rest_wait_ms;
volatile uint32_t tb_rotation_rest_wait_max_ms;
volatile uint32_t tb_rotation_elapsed_ms;
volatile uint8_t  tb_rotation_locked;
volatile uint32_t tb_rotation_position;
volatile uint8_t  tb_rotation_dir;
volatile uint32_t tb_rotation_cycle;
volatile uint32_t tb_rotation_phase;
volatile uint8_t  tb_rotation_initial_done;
volatile uint8_t  tb_rotation_is_late;
volatile uint8_t  tb_rotation_stir_rest;
volatile uint8_t  tb_rotation_grind_rest;

/* 엔진·어댑터 상태 */
static zg_rotation         s_rot;
static zg_rotation_config  s_cfg;
static zg_rotation_profile s_prof;
static uint32_t s_last_poll;
static uint32_t s_start_ms;
static uint32_t s_hs6_pos;        /* pos_src=0 전용: HS6 엣지 × 엔진 방향 부호 (벤치 단순안) */
static int8_t   s_sign;           /* 마지막으로 **명령한** 엔진 방향 부호 — 관성 회전도 같은 쪽 */
static uint32_t s_edge_prev;      /* GuideEdge_Count() 직전값                     */
static uint32_t s_edge_base;      /* run 시작 시 GuideEdge_Count()                */
static uint32_t s_pos;            /* 엔진에 넣은 position (미러용)                */
static uint8_t  s_out;            /* 직전 엔진 반환 (0/1/2)                       */
static uint8_t  s_prev_active;    /* 직전 tick 의 s_rot.active_leg               */
static uint8_t  s_prev_complete;  /* 직전 tick 의 s_rot.complete_leg             */
static uint32_t s_leg_start_ms, s_leg_edge0, s_leg_fg0;
static uint32_t s_stop_cmd_ms;    /* 정지 명령 시각 (rest_wait 측정)              */
static uint8_t  s_rest_pending;   /* 1 = 정지 명령 후 아직 정지 미확인            */

static uint8_t tb_rot_rinse_busy(void)
{
	return (uint8_t)(tb_rinse_spinning ||
	                 ((tb_rinse_pat_state != TB_RINSE_PAT_IDLE) &&
	                  (tb_rinse_pat_state != TB_RINSE_PAT_DONE)));
}

uint8_t TB_Rotation_Busy(void)
{
	return (uint8_t)(tb_rotation_state == TB_ROT_ST_RUN);
}

/* 정지 관측 — 드라이버 BldcCtrl_IsAtRest() (C089). 판정 시간은 g_stir_ctrl.rest_ms. */
static void tb_rot_rest_track(uint32_t now)
{
	tb_rotation_stir_rest  = BldcCtrl_IsAtRest(&g_stir_ctrl,  now);
	tb_rotation_grind_rest = BldcCtrl_IsAtRest(&g_grind_ctrl, now);
}

/* 엔진에 넣을 position — 계약의 "위에서 볼 때 CW = +".
 *  pos_src 1: 드라이버 BldcCtrl_Position()(C088, **DIR핀 기준 부호**)을 dir_invert 로
 *             "위에서 본" 부호로 바꾼다. dir_invert 가 N14 실측 결과 그 자체다.
 *  pos_src 0: HS6 엣지를 엔진 방향 부호로 누적(§17.6 단순안, 계약상 **벤치 전용**). */
static uint32_t tb_rot_position(void)
{
	uint32_t e = GuideEdge_Count();
	uint32_t d = e - s_edge_prev;
	s_edge_prev = e;
	if (s_sign >= 0) { s_hs6_pos += d; } else { s_hs6_pos -= d; }

	if (tb_rotation_pos_src == 0U) { return s_hs6_pos; }
	{
		uint32_t p = BldcCtrl_Position(&g_stir_ctrl);
		return tb_rotation_dir_invert ? (0U - p) : p;
	}
}

static void tb_rot_drive(uint8_t out)
{
	uint8_t reverse;

	if (out == 0U)
	{
		/* ★§0.25: 운전 사이 정지는 **깨운 채** — 슬립하면 FG(DVDD 풀업)가 죽어 at_rest 가
		 * 가짜가 된다. 벤치 종료(tb_rot_finish)만 Stop() 으로 재운다. */
		BldcCtrl_CoastAwake(&g_stir_ctrl);
		return;
	}
	reverse = (uint8_t)((out == 2U) ? 1U : 0U);
	if (tb_rotation_dir_invert) { reverse = (uint8_t)!reverse; }
	s_sign = (out == 2U) ? (int8_t)-1 : (int8_t)1;
	BldcCtrl_Stop(&g_stir_ctrl);
	BldcCtrl_Start(&g_stir_ctrl, reverse);
	g_stir_ctrl.target_out_rpm = tb_rotation_rpm;
}

static void tb_rot_finish(uint8_t result)
{
	BldcCtrl_Stop(&g_stir_ctrl);
	s_out = 0U;
	tb_rotation_dir    = 0U;
	tb_rotation_result = result;
	tb_rotation_state  = TB_ROT_ST_DONE;
}

static void tb_rot_start(uint32_t now)
{
	uint8_t p = tb_rotation_profile;
	s_prof = (p == 2U) ? ZG_ROT_DRAIN : (p == 3U) ? ZG_ROT_PROCESS : ZG_ROT_WASH;

	/* P48·P49~P53 은 벤더 generated/config.c 값 고정. P54·P55 만 벤치에서 바꾼다. */
	/* P54: HS6 단순안은 엣지 1개 = 1 tick. FG안은 run G 실측 822(기록지 §11.7). */
	if ((tb_rotation_pos_src != 0U) && (tb_rotation_ticks_per_rev <= 1U))
	{
		tb_rotation_ticks_per_rev = TB_ROT_P54_FG_PER_REV;
	}
	s_cfg.ticks_per_revolution = (tb_rotation_pos_src == 0U) ? 1U : tb_rotation_ticks_per_rev;
	s_cfg.revolutions          = 2U;      /* P48 */
	s_cfg.stop_ms              = 2000U;   /* P53 */
	s_cfg.feedback_timeout_ms  = tb_rotation_timeout_ms;   /* P55 */
	s_cfg.initial_cycles       = 30U;     /* P49 */
	s_cfg.forward_cycles       = 10U;     /* P51 */
	s_cfg.reverse_cycles       = 2U;      /* P50 */
	s_cfg.wash_initial_cycles  = 5U;      /* P52 */

	zg_rotation_reset(&s_rot);
	s_hs6_pos = 0U; s_sign = 1;
	s_edge_prev = GuideEdge_Count(); s_edge_base = s_edge_prev;
	s_out = 0U; s_prev_active = 0U; s_prev_complete = 0U;
	s_rest_pending = 0U;
	s_start_ms = now;

	tb_rotation_result = TB_ROT_RES_NONE;
	tb_rotation_fail_where = TB_ROT_FAIL_NONE;
	tb_rotation_legs = 0U; tb_rotation_dir_mask = 0U; tb_rotation_edges = 0U;
	tb_rotation_leg_edges = 0U; tb_rotation_leg_ms = 0U; tb_rotation_leg_fg = 0U;
	tb_rotation_leg_min_ms = 0xFFFFFFFFU; tb_rotation_leg_max_ms = 0U;
	tb_rotation_rest_wait_ms = 0U; tb_rotation_rest_wait_max_ms = 0U;
	tb_rotation_locked = 0U; tb_rotation_elapsed_ms = 0U;
	tb_rotation_state = TB_ROT_ST_RUN;
}

static void tb_rot_run_tick(uint32_t now)
{
	zg_rotation_input in;
	uint8_t  out;
	uint8_t  was_active = (uint8_t)s_rot.active_leg;
	uint32_t origin     = s_rot.origin;
	uint8_t  dir        = s_rot.direction;

	s_pos = tb_rot_position();

	in.position       = s_pos;
	in.valid          = true;   /* HS6·FG 배선 고장 판정은 아직 없다 — 벤치는 늘 유효 */
	in.stir_at_rest   = (tb_rotation_stir_rest  != 0U);
	in.grind_at_rest  = (tb_rotation_grind_rest != 0U);
	in.permitted      = true;   /* 교반 전용 벤치 — 분쇄 허가(§0.19)와 무관           */
	in.late_requested = (tb_rotation_late != 0U);

	out = zg_rotation_tick(&s_rot, s_prof, &s_cfg, &in, now);

	/* 엔진 미러 */
	tb_rotation_position     = s_pos;
	tb_rotation_dir          = out;
	tb_rotation_cycle        = s_rot.cycle;
	tb_rotation_phase        = s_rot.phase;
	tb_rotation_initial_done = (uint8_t)s_rot.initial_done;
	tb_rotation_is_late      = (uint8_t)s_rot.late;
	tb_rotation_edges        = GuideEdge_Count() - s_edge_base;
	tb_rotation_elapsed_ms   = now - s_start_ms;

	if (s_rot.failed)
	{
		/* 엔진은 실패 사유를 주지 않는다 — 직전 상태로 가른다. */
		if ((s_cfg.feedback_timeout_ms <= s_cfg.stop_ms) || (s_cfg.ticks_per_revolution == 0U))
		{
			tb_rotation_fail_where = TB_ROT_FAIL_CONFIG;
		}
		else if (was_active)
		{
			uint32_t travel = (dir == 1U) ? (s_pos - origin) : (origin - s_pos);
			tb_rotation_fail_where = (travel >= 0x80000000U) ? TB_ROT_FAIL_TRAVEL
			                                                  : TB_ROT_FAIL_PROGRESS;
		}
		else
		{
			tb_rotation_fail_where = TB_ROT_FAIL_REST;
		}
		tb_rot_finish(TB_ROT_RES_FAILED);
		return;
	}

	/* 운전 시작(active_leg 0→1) */
	if (s_rot.active_leg && !s_prev_active)
	{
		s_leg_start_ms = now;
		s_leg_edge0    = GuideEdge_Count();
		s_leg_fg0      = drv8306_m2.fg_edges;
	}
	/* 운전 완료(complete_leg 0→1) — 엔진 기준. late 로 끊긴 정지는 완료가 아니다. */
	if (s_rot.complete_leg && !s_prev_complete)
	{
		uint32_t ms = now - s_leg_start_ms;
		if ((tb_rotation_legs < 32U) && (s_rot.direction == 2U))
		{
			tb_rotation_dir_mask |= (1UL << tb_rotation_legs);
		}
		tb_rotation_legs++;
		tb_rotation_leg_edges = GuideEdge_Count() - s_leg_edge0;
		tb_rotation_leg_fg    = drv8306_m2.fg_edges - s_leg_fg0;
		tb_rotation_leg_ms    = ms;
		if (ms < tb_rotation_leg_min_ms) { tb_rotation_leg_min_ms = ms; }
		if (ms > tb_rotation_leg_max_ms) { tb_rotation_leg_max_ms = ms; }
	}
	s_prev_active   = (uint8_t)s_rot.active_leg;
	s_prev_complete = (uint8_t)s_rot.complete_leg;

	/* 구동 — 반환이 바뀐 사이클에만 명령한다(Stop+Start 는 램프를 다시 건다). */
	if (out != s_out)
	{
		tb_rot_drive(out);
		if (out == 0U) { s_stop_cmd_ms = now; s_rest_pending = 1U; }
		s_out = out;
	}
	/* ★N15: 정지 명령 → 실제 정지 확인까지 */
	if (s_rest_pending && tb_rotation_stir_rest)
	{
		uint32_t w = now - s_stop_cmd_ms;
		tb_rotation_rest_wait_ms = w;
		if (w > tb_rotation_rest_wait_max_ms) { tb_rotation_rest_wait_max_ms = w; }
		s_rest_pending = 0U;
	}

	/* 종료 */
	if ((s_prof != ZG_ROT_PROCESS) && (s_rot.cycle >= 10U))
	{
		tb_rot_finish(TB_ROT_RES_COMPLETE);
	}
	else if ((s_prof == ZG_ROT_PROCESS) && (tb_rotation_legs >= tb_rotation_max_legs))
	{
		tb_rot_finish(TB_ROT_RES_COMPLETE);
	}
	else if (g_stir_ctrl.state == (uint8_t)BLDC_LOCKED)
	{
		tb_rotation_locked = 1U;
		tb_rot_finish(TB_ROT_RES_ABORT);
	}
}

void TB_Rotation_Init(void)
{
	tb_rotation_state  = TB_ROT_ST_IDLE;
	tb_rotation_result = TB_ROT_RES_NONE;
	tb_rotation_selftest_done = 0U;
	s_last_poll  = 0U;
}

void TB_Rotation_Poll(uint8_t bench_idle)
{
	uint32_t now = HAL_GetTick();

	/* 벤치 이탈 — 이 함수가 한동안 안 불렸으면(시나리오 모드였다) 무효 처리.
	 * 모터는 모드 전환 때 Jungji 가 이미 세웠다. */
	if ((s_last_poll != 0U) && ((now - s_last_poll) > TB_ROT_POLL_GAP_MS)) { bench_idle = 0U; }
	s_last_poll = now;

	tb_rot_rest_track(now);

	/* A. 자가시험 — 모터 무관, 즉시 끝난다 */
	if (tb_rotation_selftest_once)
	{
		tb_rotation_selftest_once = 0U;
		st_fail_line = 0U;
		tb_rotation_selftest_body();
		tb_rotation_selftest_asserts   = st_asserts;
		tb_rotation_selftest_fail_line = st_fail_line;
		tb_rotation_selftest_done      = 1U;
	}

	/* B. run F */
	if (tb_rotation_state == TB_ROT_ST_RUN)
	{
		if (tb_rotation_stop_once || !bench_idle)
		{
			tb_rotation_stop_once = 0U;
			tb_rot_finish(TB_ROT_RES_ABORT);
			return;
		}
		tb_rot_run_tick(now);
		return;
	}
	tb_rotation_stop_once = 0U;

	if (tb_rotation_once)
	{
		tb_rotation_once = 0U;
		if (tb_rot_rinse_busy())
		{
			tb_rotation_result = TB_ROT_RES_BUSY;
			tb_rotation_state  = TB_ROT_ST_DONE;
		}
		else if (bench_idle && !Jungji_IsBraking())
		{
			if (tb_rotation_timeout_ms < TB_ROT_TIMEOUT_MIN_MS)
			{
				tb_rotation_timeout_ms = TB_ROT_TIMEOUT_MIN_MS;   /* 그 이하면 엔진이 즉시 실패 */
			}
			tb_rot_start(now);
		}
	}
}
