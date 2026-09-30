/* ==========================================================================
 * rinse_lock.c - UL1 상당 cnt 20/40 판정(MCU 구현). 계약은 rinse_lock.h.
 * 입력은 guide_edge(HS6 상승엣지 카운터) 하나뿐이고 액추에이터는 만지지 않는다.
 * ========================================================================== */

#include "rinse_lock.h"
#include "guide_edge.h"
#include "rotation_port.h"          /* ROT_M2_TICKS_PER_REV (P54) — RINSE_NET_GUIDE_TICKS */

void RinseLock_Reset(RinseLock *l)
{
	l->armed       = 0U;
	l->base        = 0U;
	l->cnt         = 0U;
	l->cycle_since = 0U;
	l->t20_ms      = 0U;
	l->t40_ms      = 0U;
	l->fault       = (uint8_t)RINSE_LOCK_OK;   /* C022: 이상은 새 운전에서만 풀린다 */
	l->net_armed   = 0U;
	l->net_base_pos = 0U;
	l->net_edges   = 0U;
	l->net         = 0;
	l->net_max     = 0U;
	l->rpm_over_since = 0U;
	l->rpm_max     = 0U;
	l->iv_armed    = 0U;
	l->iv_edges    = 0U;
	l->iv_last_ms  = 0U;
	l->iv_min_ms   = 0U;
	l->iv_short    = 0U;
}

void RinseLock_BeginCycle(RinseLock *l, uint32_t now_ms)
{
	l->base        = GuideEdge_Count();         /* 준비 탐색 엣지 제외(C009) */
	l->cnt         = 0U;
	l->cycle_since = now_ms;
	l->t20_ms      = 0U;
	l->t40_ms      = 0U;
	l->armed       = 1U;
	l->net_armed   = 0U;                        /* §0.38: 첫 NetTick 에서 기준 위치를 뜬다 */
	l->net         = 0;
	l->net_max     = 0U;
	/* fault 는 건드리지 않는다 — 회차 반복으로 소거되지 않는다(C022). */
}

void RinseLock_Disarm(RinseLock *l)
{
	l->armed = 0U;                              /* 다음 회차 급수 재허가(C019) */
}

void RinseLock_Tick(RinseLock *l, uint32_t now_ms)
{
	uint32_t n;

	if (!l->armed) { return; }

	n = GuideEdge_Count() - l->base;            /* uint32 모듈러 — 랩어라운드 안전 */
	l->cnt = (uint16_t)((n > 0xFFFFU) ? 0xFFFFU : n);

	if ((l->t20_ms == 0U) && (l->cnt >= (uint16_t)RINSE_WASH_EDGES))
	{
		l->t20_ms = (now_ms - l->cycle_since) | 1U;   /* 0 = 미도달과 구분 */
	}
	if ((l->t40_ms == 0U) && (l->cnt >= (uint16_t)RINSE_TOTAL_EDGES))
	{
		uint32_t t = now_ms - l->cycle_since;
		l->t40_ms = t | 1U;
#if RINSE_MIN_CYCLE_GUARD                           /* ★2026-09-22 E02 과속 R4 이관 — 현재 미적용 */
		if (t < (uint32_t)RINSE_MIN_CYCLE_MS)
		{
			RinseLock_SetFault(l, RINSE_LOCK_OVERSPEED);   /* C015 과속 */
		}
#endif
	}
}

uint8_t RinseLock_NetTick(RinseLock *l, uint32_t stir_pos, uint8_t watch)
{
#if RINSE_NET_GUARD_ENABLE
	uint32_t e;
	uint32_t a;

	if (!watch || !l->armed) { l->net_armed = 0U; return 0U; }

	e = GuideEdge_Count();
	if (!l->net_armed || (e != l->net_edges))
	{
		/* 감시 시작 또는 새 가이드 → 순이동 0 (B006 "새 감지면 순이동 0") */
		l->net_armed    = 1U;
		l->net_edges    = e;
		l->net_base_pos = stir_pos;
		l->net          = 0;
		return 0U;
	}
	l->net = (int32_t)(stir_pos - l->net_base_pos);           /* uint32 모듈러 차 → 부호 */
	a = (l->net < 0) ? (uint32_t)(-l->net) : (uint32_t)l->net;
	if (a > l->net_max) { l->net_max = (uint16_t)((a > 0xFFFFU) ? 0xFFFFU : a); }
	if (a >= (uint32_t)RINSE_NET_GUIDE_TICKS)
	{
		RinseLock_SetFault(l, RINSE_LOCK_GUIDE_MISSING);
		return 1U;
	}
	return 0U;
#else
	(void)l; (void)stir_pos; (void)watch;
	return 0U;
#endif
}

uint8_t RinseLock_RpmTick(RinseLock *l, uint16_t stir_rpm, uint8_t watch, uint32_t now_ms)
{
#if RINSE_RPM_GUARD_ENABLE
	if (!watch) { l->rpm_over_since = 0U; return 0U; }
	if (stir_rpm > l->rpm_max) { l->rpm_max = stir_rpm; }
	if (stir_rpm > (uint16_t)RINSE_OVERSPEED_RPM)                   /* 40 초과만 — 40 이하는 절대 차단 안 함 */
	{
		if (l->rpm_over_since == 0U) { l->rpm_over_since = now_ms | 1U; }
		if ((uint32_t)(now_ms - l->rpm_over_since) >= (uint32_t)RINSE_OVERSPEED_HOLD_MS)
		{
			RinseLock_SetFault(l, RINSE_LOCK_OVERSPEED_RPM);
			return 1U;
		}
	}
	else { l->rpm_over_since = 0U; }
	return 0U;
#else
	(void)l; (void)stir_rpm; (void)watch; (void)now_ms;
	return 0U;
#endif
}

uint8_t RinseLock_IntervalTick(RinseLock *l, uint8_t watch)
{
#if RINSE_GUIDE_INTERVAL_GUARD
	uint32_t e;

	if (!watch) { l->iv_armed = 0U; l->iv_short = 0U; return 0U; }
	e = GuideEdge_Count();
	if (!l->iv_armed)
	{
		/* 감시 시작: 기준만. 첫 간격은 감시 밖에서 시작됐을 수 있어 재지 않는다. */
		l->iv_armed = 1U; l->iv_edges = e; l->iv_last_ms = 0U; l->iv_short = 0U;
		return 0U;
	}
	if (e != l->iv_edges)
	{
		uint32_t t     = GuideEdge_LastEdgeMs();
		uint8_t  one   = (uint8_t)((e - l->iv_edges) == 1U);
		uint8_t  have  = (uint8_t)(l->iv_last_ms != 0U);
		uint32_t iv    = t - l->iv_last_ms;
		l->iv_edges    = e;
		l->iv_last_ms  = t | 1U;                                     /* 0 = 아직 없음 과 구분 */
		if (!one || !have) { l->iv_short = 0U; return 0U; }          /* 첫 엣지 / 한 tick 에 여러 개(간격 불명) */
		if ((l->iv_min_ms == 0U) || (iv < l->iv_min_ms)) { l->iv_min_ms = iv; }
		if (iv < (uint32_t)RINSE_GUIDE_MIN_INTERVAL_MS)
		{
			if (l->iv_short < 0xFFU) { l->iv_short++; }
			if (l->iv_short >= (uint8_t)RINSE_GUIDE_SHORT_COUNT)
			{
				RinseLock_SetFault(l, RINSE_LOCK_GUIDE_INTERVAL);
				return 1U;
			}
		}
		else { l->iv_short = 0U; }
	}
	return 0U;
#else
	(void)l; (void)watch;
	return 0U;
#endif
}

void RinseLock_SetFault(RinseLock *l, RinseLockFault f)
{
	if (l->fault == (uint8_t)RINSE_LOCK_OK) { l->fault = (uint8_t)f; }
}

uint8_t RinseLock_WaterAllowed(const RinseLock *l)
{
	return (uint8_t)(!l->armed || (l->cnt < (uint16_t)RINSE_WASH_EDGES));
}

uint8_t RinseLock_WashDone(const RinseLock *l)
{
	return (uint8_t)(l->armed && (l->cnt >= (uint16_t)RINSE_WASH_EDGES));
}

uint8_t RinseLock_CycleComplete(const RinseLock *l)
{
	return (uint8_t)(l->armed && (l->cnt >= (uint16_t)RINSE_TOTAL_EDGES));
}

uint8_t RinseLock_StirFault(const RinseLock *l)
{
	return (uint8_t)(l->fault != (uint8_t)RINSE_LOCK_OK);
}
