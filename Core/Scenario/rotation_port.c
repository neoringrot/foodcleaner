/* ==========================================================================
 * rotation_port.c - 회전수 엔진(rotation.c, 벤더 0.3.0) ↔ 교반 BLDC 어댑터.
 * 설계·계약은 rotation_port.h. 구동 규칙은 벤치 검증본(tb_rotation, HW 1-30)과 같다:
 *   - 엔진 반환이 **바뀐 tick 에만** 모터에 명령한다(Stop+Start 는 램프를 다시 건다).
 *   - 운전 사이 정지(반환 0)는 BldcCtrl_CoastAwake() — 슬립하면 FG 풀업(DVDD)이 꺼져
 *     정지 판정이 가짜가 된다(§0.25).
 *   - 엔진 실패 시에만 BldcCtrl_Stop()(슬립)으로 확실히 끈다.
 * ========================================================================== */

#include "rotation_port.h"
#include "bldc_ctrl.h"
#include "guide_edge.h"

/* P48~P53 은 벤더 generated/config.c 0.3.0 값. P54·P55 는 rotation_port.h. */
static const zg_rotation_config s_cfg =
{
	.ticks_per_revolution = ROT_M2_TICKS_PER_REV,     /* P54 */
	.revolutions          = 2U,                       /* P48 (엔진이 2 외에는 실패시킨다) */
	.stop_ms              = 2000U,                    /* P53 */
	.feedback_timeout_ms  = ROT_FEEDBACK_TIMEOUT_MS,  /* P55 */
	.initial_cycles       = 30U,                      /* P49 (§0.46 테스트 축소는 §0.47 로 폐지) */
	.forward_cycles       = 10U,                      /* P51 */
	.reverse_cycles       = 2U,                       /* P50 */
	.wash_initial_cycles  = 5U,                       /* P52 */
};

/* 드라이버 위치(DIR핀 기준 부호) → 계약 position("위에서 본 CW = +"). */
static uint32_t rs_position(void)
{
	uint32_t p = BldcCtrl_Position(&g_stir_ctrl);
	return ROT_M2_CW_IS_REVERSE ? (0U - p) : p;
}

/* 위에서 본 방향(1 = CW, 2 = CCW) → DRV8306 reverse 비트. 극성은 모터마다 다르다(N14). */
static uint8_t rs_rev(uint8_t out, uint8_t cw_is_reverse)
{
	return (out == 1U) ? cw_is_reverse : (uint8_t)!cw_is_reverse;
}

static void rs_drive(RotStir_t *r, uint8_t out)
{
	if (out == 0U)
	{
		BldcCtrl_CoastAwake(&g_stir_ctrl);            /* 깨운 채 정지 — FG 로 정지 관측 */
		if (r->paired) { BldcCtrl_CoastAwake(&g_grind_ctrl); }
		return;
	}
	BldcCtrl_Stop(&g_stir_ctrl);                       /* tb_rotation 검증본과 동일 순서 */
	BldcCtrl_Start(&g_stir_ctrl, rs_rev(out, ROT_M2_CW_IS_REVERSE ? 1U : 0U));
	g_stir_ctrl.target_out_rpm = r->rpm;

	if (r->paired)
	{
		/* 벤더 ZG_D008~D010: 분쇄 = 교반과 같은 방향, late 면 반대 방향. 같은 가동 구간. */
		uint8_t gdir = out;
		r->late_on = (uint8_t)(r->eng.late ? 1U : 0U);
		if (r->late_on) { gdir = (out == 1U) ? 2U : 1U; }
		BldcCtrl_Stop(&g_grind_ctrl);
		BldcCtrl_Start(&g_grind_ctrl, rs_rev(gdir, ROT_M1_CW_IS_REVERSE ? 1U : 0U));
		g_grind_ctrl.target_out_rpm = r->late_on ? r->late_rpm : r->grind_rpm;
	}
}

void RotStir_Manual(uint8_t ccw, uint16_t rpm)
{
	uint8_t cw_rev = ROT_M2_CW_IS_REVERSE ? 1U : 0U;

	if (rpm == 0U)
	{
		BldcCtrl_CoastAwake(&g_stir_ctrl);
		return;
	}
	BldcCtrl_Stop(&g_stir_ctrl);
	BldcCtrl_Start(&g_stir_ctrl, (ccw != 0U) ? (uint8_t)!cw_rev : cw_rev);
	g_stir_ctrl.target_out_rpm = rpm;
}

void RotStir_Start(RotStir_t *r, zg_rotation_profile p, uint16_t rpm)
{
	/* 다른 제어가 교반을 돌리던 중이면 먼저 깨운 채 세운다 — 엔진은 첫 운전 전에
	 * 정지를 확인하는데, 아무도 모터를 세우지 않으면 그 확인이 영영 안 온다(→ P55 실패). */
	if (BldcCtrl_IsRunning(&g_stir_ctrl)) { BldcCtrl_CoastAwake(&g_stir_ctrl); }

	zg_rotation_reset(&r->eng);
	r->paired    = 0U;
	r->late_on   = 0U;
	r->prof      = p;
	r->rpm       = rpm;
	r->out       = 0U;
	r->out_since = HAL_GetTick();
	r->edge_base = GuideEdge_Count();
	r->edges     = 0U;
	r->failed    = 0U;
}

void RotStir_StartProcess(RotStir_t *r, uint16_t stir_rpm, uint16_t grind_rpm, uint16_t late_rpm)
{
	/* 돌던 분쇄가 있으면 깨운 채 세운다 — paired 정지 확인(grind_at_rest)이 와야 첫 운전이 나간다. */
	if (BldcCtrl_IsRunning(&g_grind_ctrl)) { BldcCtrl_CoastAwake(&g_grind_ctrl); }
	RotStir_Start(r, ZG_ROT_PROCESS, stir_rpm);
	r->paired    = 1U;
	r->grind_rpm = grind_rpm;
	r->late_rpm  = late_rpm;
}

void RotStir_Switch(RotStir_t *r, zg_rotation_profile p, uint16_t rpm)
{
	/* 엔진은 유지 — 다음 tick 에 프로파일 변경을 보고 스스로 리셋 + 정지 대기(hold)한다. */
	r->prof      = p;
	r->rpm       = rpm;
	r->edge_base = GuideEdge_Count();
	r->edges     = 0U;
}

uint8_t RotStir_Tick(RotStir_t *r, bool permitted, uint32_t now_ms)
{
	return RotStir_TickEx(r, permitted, false, now_ms);
}

uint8_t RotStir_TickEx(RotStir_t *r, bool permitted, bool late_requested, uint32_t now_ms)
{
	zg_rotation_input in;
	uint8_t out;

	if (r->prof == ZG_ROT_NONE) { return ROT_ST_IDLE; }
	if (r->failed)              { return ROT_ST_FAILED; }

	in.position       = rs_position();
	in.valid          = true;   /* FG 배선 고장 검출은 없다 — 무진행은 P55 타임아웃이 잡는다 */
	in.stir_at_rest   = (BldcCtrl_IsAtRest(&g_stir_ctrl,  now_ms) != 0U);
	in.grind_at_rest  = (BldcCtrl_IsAtRest(&g_grind_ctrl, now_ms) != 0U);
	in.permitted      = permitted;
	in.late_requested = late_requested;  /* PROCESS 만 의미 있다(엔진이 paired 에서만 본다) */

	out = zg_rotation_tick(&r->eng, r->prof, &s_cfg, &in, now_ms);
	r->edges = GuideEdge_Count() - r->edge_base;

	if (r->eng.failed)
	{
		BldcCtrl_Stop(&g_stir_ctrl);                   /* 실패: 확실히 끈다(슬립) */
		if (r->paired) { BldcCtrl_Stop(&g_grind_ctrl); }
		r->failed    = 1U;
		r->out       = 0U;
		r->out_since = now_ms;
		return ROT_ST_FAILED;
	}
	if (out != r->out)
	{
		rs_drive(r, out);
		r->out       = out;
		r->out_since = now_ms;
	}
	if ((r->prof != ZG_ROT_PROCESS) && (r->eng.cycle >= ROT_WASH_DRAIN_LEGS))
	{
		return ROT_ST_DONE;
	}
	return ROT_ST_RUNNING;
}

void RotStir_Stop(RotStir_t *r)
{
	r->prof = ZG_ROT_NONE;
	r->out  = 0U;
	BldcCtrl_Stop(&g_stir_ctrl);
	if (r->paired) { BldcCtrl_Stop(&g_grind_ctrl); }
	r->paired  = 0U;
	r->late_on = 0U;
}

uint8_t RotStir_InitialDone(const RotStir_t *r)
{
	return (uint8_t)(r->eng.initial_done ? 1U : 0U);
}

uint8_t RotStir_Late(const RotStir_t *r)
{
	return (uint8_t)(r->eng.late ? 1U : 0U);
}

uint8_t RotStir_IsActive(const RotStir_t *r)
{
	return (uint8_t)((r->prof != ZG_ROT_NONE) && !r->failed);
}

uint32_t RotStir_Legs(const RotStir_t *r)
{
	return r->eng.cycle;
}
