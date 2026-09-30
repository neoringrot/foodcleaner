/* ==========================================================================
 * rinse.c - 공통 헹굼 모듈. 설계·계약은 rinse.h / 공통헹굼_모듈_구조R3.md.
 *
 * 준비 탐색(R001~R006)은 벤더 controller.c ZG_R001~ZG_R006 과 같은 판정:
 *   case R001/R003/R005:
 *     if (guide_detected && (last_edge - step_since) <= ZG_PREPARE_MS)  → R006
 *     else if (expired(step_since, ZG_PREPARE_MS))                      → R002/R004, R005 면 E01
 *     else 교반(R003 = 역, 나머지 = 정, rinse_rpm)
 *   case R002/R004:
 *     if (expired(step_since, rinse_stop_ms)) { baseline = count; → R003/R005 }
 *
 * 회차 루프(R007~R019, ★§0.30)는 종전 moeum.c MOEUM_DOOR_CLOSE~DRAIN_WAIT /
 * dongjak.c dj_rs_* 의 규칙을 그대로 옮겼다(배수문 = wdoor.h 프로파일, 교반 = WASH/DRAIN 회전수 운전).
 * ★2026-09-22 종료 판정 = rinse_lock cnt 20/40(벤더 R013~R017). 엔진 10회는 교반 운전 단위일 뿐
 *   종료 신호가 아니다 — 먼저 끝나면 cnt 를 15 s 기다리고, 안 오면 E09(count_mismatch).
 * 이 파일은 액추에이터 API 를 부르지 않는다 — 명령 비트만 낸다(rinse.h).
 * ========================================================================== */

#include "rinse.h"
#include "guide_edge.h"
#include "rotation_port.h"   /* ROT_ST_* (값만) */
#include "wdoor.h"           /* WDOOR_* 프로파일 · WDoor_OpenDutyAt() (inline 계산) */

static void rinse_go(RinseCtx *c, RinseStep s, uint32_t now)
{
	c->step       = (uint8_t)s;
	c->step_since = now;
}

static void rinse_stir(RinseOutputs *out, uint8_t dir, uint16_t rpm)
{
	out->cmd     |= RINSE_CMD_STIR_MANUAL;
	out->stir_dir = dir;
	out->stir_rpm = rpm;
}

static uint16_t rinse_ms16(uint32_t el)
{
	return (uint16_t)((el > 0xFFFFU) ? 0xFFFFU : el);
}

/* ---- 단계 진입 (진입 명령을 out 에 싣는다) ------------------------------ */

static void rinse_prep_begin(RinseCtx *c, uint32_t now, RinseOutputs *out)
{
	c->begin_ms = now;
	c->base     = GuideEdge_Count();          /* ZG_S005 / ZG_R011: baseline 후 R001 */
	rinse_go(c, RINSE_PREP_F1, now);
	rinse_stir(out, 0U, (uint16_t)RINSE_PREP_RPM);
}

static void rinse_close_begin(RinseCtx *c, uint32_t now, RinseOutputs *out)
{
	RinseLock_Disarm(&c->lock);                       /* R007: 새 회차 — 급수 재허가(C019) */
	c->door_close_by = (uint8_t)RINSE_DOOR_BY_NONE;   /* 새 회차: 진단값 리셋 */
	c->door_open_by  = (uint8_t)RINSE_DOOR_BY_NONE;
	c->door_close_ms = 0U;
	c->door_open_ms  = 0U;
	rinse_go(c, RINSE_CLOSE, now);
	out->cmd |= RINSE_CMD_DOOR_CLOSE;
}

static void rinse_wash_begin(RinseCtx *c, uint32_t now, RinseOutputs *out)
{
	/* 개정4 ④ C013: 본교반 = WASH 회전수 운전(CW×5 → CW/CCW/CW/CCW/CW). 매 회차 새 운전. */
	rinse_go(c, RINSE_WASH, now);
	out->cmd    |= RINSE_CMD_ROT_WASH;
	out->rot_rpm = c->cfg.rot_rpm;
	RinseLock_BeginCycle(&c->lock, now);              /* R012: cnt 기준값·t_cycle 기산 */
	c->sw_done  = 0U;
	c->stir_cut = 0U;
}

/* 엔진(소프트웨어 20회전)이 끝났는데 cnt 마일스톤이 오지 않으면 RINSE_MILESTONE_TIMEOUT_MS(15 s)
 * 뒤 1 을 돌려준다 — 벤더 counted() 의 "cycle >= 10 && expired(progress_since, request_timeout)".
 * 마일스톤을 만들어 내지 않는다(port_contract count_mismatch). */
static uint8_t rinse_milestone_late(RinseCtx *c, const RinseInputs *in, uint32_t now)
{
	if (in->rot_st != ROT_ST_DONE) { c->sw_done = 0U; return 0U; }
	if (!c->sw_done) { c->sw_done = 1U; c->sw_done_since = now; return 0U; }
	return (uint8_t)((now - c->sw_done_since) >= (uint32_t)RINSE_MILESTONE_TIMEOUT_MS);
}

static void rinse_fault(RinseCtx *c, RinseFault f, uint32_t now)
{
	c->fault = (uint8_t)f;
	rinse_go(c, (f == RINSE_FAULT_GUIDE) ? RINSE_PREP_FAULT : RINSE_FAULT, now);
}

/* ---- 준비 탐색 한 tick. RINSE_PREP_OK / RINSE_PREP_FAULT 로 끝난다 ------ */
static void rinse_prep_tick(RinseCtx *c, uint32_t now, RinseOutputs *out)
{
	RinseStep s  = (RinseStep)c->step;
	uint32_t  el = now - c->step_since;

	switch (s)
	{
	case RINSE_PREP_F1:
	case RINSE_PREP_R:
	case RINSE_PREP_F2:
		/* 감지를 시간보다 먼저 본다 — 5.000 s 에 들어온 엣지는 감지로 친다(C008).
		 * 판정 시각은 이 tick 이 아니라 엣지를 센 시각이다(벤더 guide_last_edge_ms). */
		if ((GuideEdge_Count() != c->base) &&
		    ((uint32_t)(GuideEdge_LastEdgeMs() - c->step_since) <= (uint32_t)RINSE_PREP_MS))
		{
			uint32_t d = GuideEdge_LastEdgeMs() - c->step_since;
			c->prep_try      = (uint8_t)((s == RINSE_PREP_F1) ? 1U : (s == RINSE_PREP_R) ? 2U : 3U);
			c->prep_ms       = (uint16_t)d;
			c->prep_total_ms = now - c->begin_ms;
			rinse_go(c, RINSE_PREP_OK, now);
			rinse_stir(out, 0U, 0U);              /* 감지 즉시 정지 (≤50 ms, C007) */
		}
		else if (el >= (uint32_t)RINSE_PREP_MS)
		{
			if (s == RINSE_PREP_F2)
			{
				c->prep_total_ms = now - c->begin_ms;
				rinse_fault(c, RINSE_FAULT_GUIDE, now);   /* E01. 4차 재시도 없음(C010) */
			}
			else
			{
				rinse_go(c, (s == RINSE_PREP_F1) ? RINSE_PREP_S1 : RINSE_PREP_S2, now);
			}
			rinse_stir(out, 0U, 0U);                   /* rpm 0 = 깨운 채 정지(방향 무관) */
		}
		break;

	case RINSE_PREP_S1:
	case RINSE_PREP_S2:
		if (el >= (uint32_t)RINSE_PREP_STOP_MS)
		{
			c->base = GuideEdge_Count();          /* 정지 중 관성 엣지는 버린다 */
			if (s == RINSE_PREP_S1)
			{
				rinse_go(c, RINSE_PREP_R, now);
				rinse_stir(out, 1U, (uint16_t)RINSE_PREP_RPM);
			}
			else
			{
				rinse_go(c, RINSE_PREP_F2, now);
				rinse_stir(out, 0U, (uint16_t)RINSE_PREP_RPM);
			}
		}
		break;

	default:
		break;
	}
}

/* ---- API ----------------------------------------------------------------- */

void Rinse_Reset(RinseCtx *c)
{
	c->step             = (uint8_t)RINSE_IDLE;
	c->step_since       = 0U;
	c->base             = 0U;
	c->prep_done        = 0U;
	c->done             = 0U;
	c->fault            = (uint8_t)RINSE_FAULT_NONE;
	c->prep_try         = 0U;
	c->prep_ms          = 0U;
	c->prep_total_ms    = 0U;
	c->begin_ms         = 0U;
	c->door_close_by    = (uint8_t)RINSE_DOOR_BY_NONE;
	c->door_open_by     = (uint8_t)RINSE_DOOR_BY_NONE;
	c->door_close_ms    = 0U;
	c->door_open_ms     = 0U;
	c->drain_open_since = 0U;
	RinseLock_Reset(&c->lock);                        /* 새 운전: 이상 이력 해제(C022) */
	c->sw_done          = 0U;
	c->sw_done_since    = 0U;
	c->stir_cut         = 0U;
}

void Rinse_Begin(RinseCtx *c, const RinseConfig *cfg, uint32_t now_ms, RinseOutputs *out)
{
	Rinse_Reset(c);
	c->cfg = *cfg;
	if (c->cfg.repeats == 0U) { c->cfg.repeats = 1U; }   /* 벤더 S005: 1~3 밖이면 E10 — 여기선 1 로 */

	out->cmd = 0U;
	out->ev  = 0U;
	if (c->cfg.prep_after_fill) { rinse_close_begin(c, now_ms, out); }   /* 자가세척: R007 부터 */
	else                        { rinse_prep_begin(c, now_ms, out);  }   /* 모음·동작: R001 부터 */
}

RinseStep Rinse_Tick(RinseCtx *c, const RinseInputs *in, uint32_t now_ms, RinseOutputs *out)
{
	uint32_t el = now_ms - c->step_since;

	out->cmd = 0U;
	out->ev  = 0U;

	RinseLock_Tick(&c->lock, now_ms);                 /* cnt 갱신 — 판정보다 먼저 */

	/* ★§0.38 C094·C060 — 가이드 순이동 감시(B006). 교반 중(WASH·DRAIN_OPEN·DRAIN, cnt 40 정지 전)에만.
	 * 새 가이드 없이 1.25 회전을 넘게 움직이면 교반만 즉시 멈추고 E01(호출부). 문 여는 중이면 문도 세운다(에러 = 전부 정지). */
	{
		RinseStep st    = (RinseStep)c->step;
		uint8_t   watch = (uint8_t)(((st == RINSE_WASH) || (st == RINSE_DRAIN_OPEN) || (st == RINSE_DRAIN)) &&
		                            !c->stir_cut);
		if (RinseLock_NetTick(&c->lock, in->stir_pos, watch))
		{
			out->cmd |= RINSE_CMD_STIR_STOP;
			if (st == RINSE_DRAIN_OPEN) { out->cmd |= RINSE_CMD_DOOR_STOP; }
			rinse_fault(c, RINSE_FAULT_GUIDE_LOST, now_ms);
			return (RinseStep)c->step;
		}
		/* ★§0.41 C061 과속 보조(B007). A = 날개 rpm > 40 300 ms(준비 회전 포함), B = 가이드 간격 < 1.5 s 2회 연속(본 헹굼·배수).
		 * 둘 다 E02(RINSE_FAULT_OVERSPEED) — 원인은 lock.fault 5/6 으로 가른다(C015 시간 과속은 3).
		 * ★2026-09-22 사용자 지시: E02 헹굼 과속은 R4 — 현재 미적용(rinse_lock.h 스위치 0). 스위치가 0 이면 블록째 빠진다. */
#if (RINSE_RPM_GUARD_ENABLE || RINSE_GUIDE_INTERVAL_GUARD)
		{
			uint8_t prep_run = (uint8_t)((st == RINSE_PREP_F1) || (st == RINSE_PREP_R) || (st == RINSE_PREP_F2));
			uint8_t a = RinseLock_RpmTick(&c->lock, in->stir_rpm, (uint8_t)(prep_run || watch), now_ms);
			uint8_t b = RinseLock_IntervalTick(&c->lock, watch);
			if (a || b)
			{
				out->cmd |= RINSE_CMD_STIR_STOP;
				if (st == RINSE_DRAIN_OPEN) { out->cmd |= RINSE_CMD_DOOR_STOP; }
				rinse_fault(c, RINSE_FAULT_OVERSPEED, now_ms);
				return (RinseStep)c->step;
			}
		}
#endif
	}

	switch ((RinseStep)c->step)
	{
	case RINSE_PREP_F1:
	case RINSE_PREP_S1:
	case RINSE_PREP_R:
	case RINSE_PREP_S2:
	case RINSE_PREP_F2:
		rinse_prep_tick(c, now_ms, out);
		if (c->step != (uint8_t)RINSE_PREP_OK) { break; }
		/* R006 성공 — 같은 tick 에 다음 단계로 */
		c->prep_done = 1U;
		out->ev     |= RINSE_EV_PREP_OK;
		if (c->cfg.prep_after_fill)
		{
			/* 자가세척(R006 → R012): 물 찬 상태에서 곧바로 WASH. 교반을 슬립시키지 않는다 —
			 * 감지 tick 에 이미 깨운 채 정지(CoastAwake)했고, 엔진이 FG 로 정지를 확인한 뒤
			 * 첫 운전을 시작한다(슬립하면 FG 가 죽어 정지 판정이 가짜가 된다, §0.25). */
			rinse_wash_begin(c, now_ms, out);
		}
		else
		{
			out->cmd |= RINSE_CMD_STIR_STOP;       /* 탐색 교반 끝 — 슬립(배수문·급수 동안) */
			rinse_close_begin(c, now_ms, out);
		}
		break;

	case RINSE_CLOSE:
		/* 닫힘은 시간이 정상 종료 조건(WDOOR_CLOSE_MS 4.2 s). 그 전에 WHALL-CLOSE 가
		 * 인식되면 거기서 멈춘다. 리미트는 한 번만 읽은 값(in)으로 판정·기록한다. */
		if (in->door_closed || (el >= (uint32_t)WDOOR_CLOSE_MS))
		{
			out->cmd        |= RINSE_CMD_DOOR_STOP;
			c->door_close_by = (uint8_t)(in->door_closed ? RINSE_DOOR_BY_LIMIT : RINSE_DOOR_BY_TIMEOUT);
			c->door_close_ms = rinse_ms16(el);
			if (!in->door_closed) { out->ev |= RINSE_EV_CLOSE_TMO; }
			rinse_go(c, RINSE_FILL, now_ms);
			/* 호출부가 수위 래치를 먼저 지운다. 급수는 락이 허가할 때만(C019, 벤더 R010
			 * out->water = hw_water_allowed) — 회차 시작에서 해제되므로 정상 흐름에선 항상 허가. */
			if (RinseLock_WaterAllowed(&c->lock)) { out->cmd |= RINSE_CMD_FILL_ON; }
		}
		break;

	case RINSE_FILL:
		/* R3 C017: 수위 감지 → 추가급수 없이 즉시 OFF(구 *_FILL_EXTRA 상태 소멸). */
		if (in->water)
		{
			out->cmd |= RINSE_CMD_FILL_OFF;
			if (c->cfg.prep_after_fill && !c->prep_done)
			{
				rinse_prep_begin(c, now_ms, out);      /* 자가세척 R011 → R001 */
			}
			else
			{
				rinse_wash_begin(c, now_ms, out);
			}
		}
		else if (el >= c->cfg.fill_timeout_ms)
		{
			out->cmd |= RINSE_CMD_FILL_OFF;
			rinse_fault(c, RINSE_FAULT_FILL, now_ms);
		}
		break;

	case RINSE_WASH:
		/* ★C014 (2026-09-22): 종료 = **cnt 20**(벤더 R013 `!hw_water_allowed`). 엔진이 10회를 다 돌기
		 * 전이어도 넘어간다 — RotStir_Switch 가 정지 확인 + 2 s 뒤 DRAIN 을 시작하므로 돌던 모터를
		 * 뒤집지 않는다. 엔진이 먼저 끝나면 15 s 안에 cnt 20 이 와야 한다. */
		if (in->rot_st == ROT_ST_FAILED)
		{
			rinse_fault(c, RINSE_FAULT_ROT, now_ms);
		}
		else if (RinseLock_WashDone(&c->lock))
		{
			c->sw_done = 0U;                                    /* DRAIN 엔진은 새로 센다 */
			/* C018: DRAIN(CW 만)으로 전환 — 문 여는 동안에도 돌고 센다(R015). 엔진이 정지 확인 +
			 * 2 s 뒤에 시작하므로 WASH 마지막 운전과 방향이 부딪히지 않는다. */
			rinse_go(c, RINSE_DRAIN_OPEN, now_ms);
			out->cmd      |= RINSE_CMD_ROT_DRAIN | RINSE_CMD_DOOR_OPEN;
			out->rot_rpm   = c->cfg.rot_rpm;
			out->door_duty = WDoor_OpenDutyAt(0U);     /* 킥 80 % */
		}
		else if (rinse_milestone_late(c, in, now_ms))
		{
			RinseLock_SetFault(&c->lock, RINSE_LOCK_COUNT_WASH);
			out->cmd |= RINSE_CMD_STIR_STOP;
			rinse_fault(c, RINSE_FAULT_COUNT, now_ms);          /* E09 count_mismatch */
		}
		break;

	case RINSE_DRAIN_OPEN:
		/* C020: 문 여는 중에 cnt 40 이면 **교반만** 멈춘다(문 구동은 계속). 벤더 R015
		 * `if (!hw_cycle_complete) out->stir = counted(...)`. */
		if (!c->stir_cut)
		{
			if (RinseLock_CycleComplete(&c->lock))
			{
				out->cmd   |= RINSE_CMD_STIR_STOP;
				c->stir_cut = 1U;
			}
			else if (in->rot_st == ROT_ST_FAILED)
			{
				rinse_fault(c, RINSE_FAULT_ROT, now_ms);
				break;
			}
			else if (rinse_milestone_late(c, in, now_ms))
			{
				RinseLock_SetFault(&c->lock, RINSE_LOCK_COUNT_DRAIN);
				out->cmd |= RINSE_CMD_STIR_STOP | RINSE_CMD_DOOR_STOP;
				rinse_fault(c, RINSE_FAULT_COUNT, now_ms);      /* E09 */
				break;
			}
		}
		/* 열림은 WHALL-OPEN 인식이 정상 종료, WDOOR_OPEN_MAX_MS(6 s)는 미인식 대비 상한.
		 * duty 는 킥(80 %, 2 s) → 유지(65 %)로 매 tick 재지령한다. */
		if (in->door_open || (el >= (uint32_t)WDOOR_OPEN_MAX_MS))
		{
			out->cmd          |= RINSE_CMD_DOOR_STOP;
			c->door_open_by    = (uint8_t)(in->door_open ? RINSE_DOOR_BY_LIMIT : RINSE_DOOR_BY_TIMEOUT);
			c->door_open_ms    = rinse_ms16(el);
			c->drain_open_since = now_ms;
			out->ev           |= RINSE_EV_DRAIN_OPENED;
			if (!in->door_open) { out->ev |= RINSE_EV_OPEN_TMO; }
			rinse_go(c, RINSE_DRAIN, now_ms);          /* DRAIN 이 문 여는 중에 끝났어도 다음 tick 에 빠진다 */
		}
		else
		{
			out->cmd      |= RINSE_CMD_DOOR_DUTY;
			out->door_duty = WDoor_OpenDutyAt(el);
		}
		break;

	case RINSE_DRAIN:
		/* ★C014·C020 (2026-09-22): 종료 = **cnt 40** → 교반 정지(≤50 ms, 1 ms tick). 완료는 이상
		 * 이력이 없을 때만(벤더 R017 `hw_cycle_complete && !hw_stir_fault`). 과속(t_cycle < 60 s)은
		 * 이상으로 래치된다(C015) — 벤더는 R017 에 머물지만 여기선 에러로 끝낸다(멈춘 채 대기 방지).
		 * 호출부가 RINSE_FAULT_OVERSPEED 를 E02(DJ_ERR_OVERSPEED 13, 해제형)로 보낸다(§0.33). */
		if (RinseLock_CycleComplete(&c->lock))
		{
			if (!c->stir_cut) { out->cmd |= RINSE_CMD_STIR_STOP; c->stir_cut = 1U; }
#if RINSE_MIN_CYCLE_GUARD                           /* ★2026-09-22 E02 과속 R4 이관 — 현재 미적용 */
			if (RinseLock_StirFault(&c->lock))
			{
				rinse_fault(c, RINSE_FAULT_OVERSPEED, now_ms);
				break;
			}
#endif
			if (c->done < 0xFFU) { c->done++; }        /* R018 — 이 전이에서 1회만(C021) */
			out->ev  |= RINSE_EV_CYCLE_DONE;
			if (c->done < c->cfg.repeats) { rinse_close_begin(c, now_ms, out); }   /* R019 → R007 */
			else                          { rinse_go(c, RINSE_FINISHED, now_ms); }
		}
		else if (in->rot_st == ROT_ST_FAILED)
		{
			rinse_fault(c, RINSE_FAULT_ROT, now_ms);
		}
		else if (rinse_milestone_late(c, in, now_ms))
		{
			RinseLock_SetFault(&c->lock, RINSE_LOCK_COUNT_DRAIN);
			out->cmd |= RINSE_CMD_STIR_STOP;
			rinse_fault(c, RINSE_FAULT_COUNT, now_ms);          /* E09 count_mismatch */
		}
		break;

	case RINSE_IDLE:
	case RINSE_PREP_OK:          /* 통과 상태 — 여기 머무르지 않는다 */
	case RINSE_PREP_FAULT:
	case RINSE_FINISHED:
	case RINSE_FAULT:
	default:
		break;
	}
	return (RinseStep)c->step;
}

RinseStep Rinse_GetStep(const RinseCtx *c)
{
	return (RinseStep)c->step;
}

uint8_t Rinse_IsEnded(const RinseCtx *c)
{
	RinseStep s = (RinseStep)c->step;
	return (uint8_t)((s == RINSE_FINISHED) || (s == RINSE_PREP_FAULT) || (s == RINSE_FAULT));
}
