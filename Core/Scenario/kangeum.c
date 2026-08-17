/* ==========================================================================
 * kangeum.c - "강음"(찰음/강한 음식물 보정) 시나리오. ★미구현 스텁★
 * 배경/미확정 사유는 kangeum.h 참조. 지금은 중재자와의 연결부만 존재하며
 * 어떤 액추에이터도 구동하지 않는다.
 * ========================================================================== */

#include "kangeum.h"

KangeumCtx g_kangeum;

void Kangeum_Init(void)
{
	g_kangeum.state           = (uint8_t)KANGEUM_IDLE;
	g_kangeum.state_since     = 0U;
	g_kangeum.start_req       = 0U;
	g_kangeum.dbg_force_start = 0U;
	g_kangeum.abort_req       = 0U;
	g_kangeum.select_count    = 0U;
	/* TODO(강음): 구현 시 여기서 전용 액추에이터 초기 상태를 잡는다.
	 * (현재는 소유 액추에이터가 없으므로 아무것도 끄지 않는다 - 다른 모드가
	 *  쥐고 있는 부하를 건드리면 안 되기 때문.) */
}

void Kangeum_Start(void)
{
	if (g_kangeum.state == (uint8_t)KANGEUM_IDLE)
	{
		g_kangeum.start_req = 1U;
	}
}

void Kangeum_Abort(void)
{
	g_kangeum.start_req = 0U;
	g_kangeum.abort_req = 0U;
	g_kangeum.state     = (uint8_t)KANGEUM_IDLE;
	g_kangeum.state_since = HAL_GetTick();
	/* TODO(강음): 구현 시 전 액추에이터 OFF. 공통 정지는 jungji.c가 담당하므로
	 * 여기서는 이 시나리오 고유 자원만 정리하면 된다. */
}

void Kangeum_RequestStop(void)
{
	g_kangeum.abort_req = 1U;
}

/* 100ms, StartDefaultTask - 센서만. 시작 트리거(HS1)는 중재자가 소유하므로
 * 여기서는 벤치 강제 시작만 받는다(다른 시나리오와 동일 기조). */
void Kangeum_SenseTick(void)
{
	if (g_kangeum.dbg_force_start != 0U)
	{
		g_kangeum.dbg_force_start = 0U;
		Kangeum_Start();
	}
	/* TODO(강음): 온도/전류/수위 등 이 모드가 볼 센서 스냅샷을 여기서 갱신. */
}

/* 1ms, StartMotorTask - 상태머신. 현재는 "선택됨" 표시만 하고 대기한다. */
void Kangeum_MotorTick(uint32_t now_ms)
{
	KangeumCtx *c = &g_kangeum;

	if (c->abort_req)
	{
		c->abort_req = 0U;
		c->state       = (uint8_t)KANGEUM_IDLE;
		c->state_since = now_ms;
	}

	if (c->start_req)
	{
		c->start_req   = 0U;
		c->state       = (uint8_t)KANGEUM_TODO;
		c->state_since = now_ms;
		if (c->select_count < 0xFFFFU) { c->select_count++; }
		/* TODO(강음): 여기서 실제 보정 시퀀스를 시작한다. 파라미터 확정 전까지
		 * 의도적으로 아무 모터도 돌리지 않는다(오동작 방지).
		 * TODO(음성): "강음 모드 선택" 안내 멘트 (미구현) */
	}

	/* TODO(강음): KANGEUM_TODO 상태의 실제 시퀀스 tick. */
}

KangeumState Kangeum_GetState(void)
{
	return (KangeumState)g_kangeum.state;
}

uint8_t Kangeum_IsBusy(void)
{
	return (uint8_t)(g_kangeum.state != (uint8_t)KANGEUM_IDLE);
}
