/* ==========================================================================
 * baesu.c - "배수"(설거지 보조) 시나리오. ★미구현 스텁★
 * 배경/미확정 사유는 baesu.h 참조. 지금은 중재자와의 연결부만 존재하며
 * 어떤 액추에이터도 구동하지 않는다.
 * ========================================================================== */

#include "baesu.h"

BaesuCtx g_baesu;

void Baesu_Init(void)
{
	g_baesu.state           = (uint8_t)BAESU_IDLE;
	g_baesu.state_since     = 0U;
	g_baesu.start_req       = 0U;
	g_baesu.dbg_force_start = 0U;
	g_baesu.abort_req       = 0U;
	g_baesu.select_count    = 0U;
	/* TODO(배수): 구현 시 배수문/배수밸브 초기 상태를 잡는다. 현재는 소유
	 * 액추에이터가 없으므로 아무것도 건드리지 않는다. */
}

void Baesu_Start(void)
{
	if (g_baesu.state == (uint8_t)BAESU_IDLE)
	{
		g_baesu.start_req = 1U;
	}
}

void Baesu_Abort(void)
{
	g_baesu.start_req = 0U;
	g_baesu.abort_req = 0U;
	g_baesu.state     = (uint8_t)BAESU_IDLE;
	g_baesu.state_since = HAL_GetTick();
	/* TODO(배수): 구현 시 이 시나리오 고유 자원 정리. 배수문을 닫을지 열어둘지는
	 * 정책 미확정(baesu.h 참조). 공통 정지는 jungji.c가 담당한다. */
}

void Baesu_RequestStop(void)
{
	g_baesu.abort_req = 1U;
}

/* 100ms, StartDefaultTask - 센서만. 시작 트리거(HS4)는 중재자가 소유하므로
 * 여기서는 벤치 강제 시작만 받는다(다른 시나리오와 동일 기조). */
void Baesu_SenseTick(void)
{
	if (g_baesu.dbg_force_start != 0U)
	{
		g_baesu.dbg_force_start = 0U;
		Baesu_Start();
	}
	/* TODO(배수): 수위/배수문 리미트 등 이 모드가 볼 센서 스냅샷 갱신. */
}

/* 1ms, StartMotorTask - 상태머신. 현재는 "선택됨" 표시만 하고 대기한다. */
void Baesu_MotorTick(uint32_t now_ms)
{
	BaesuCtx *c = &g_baesu;

	if (c->abort_req)
	{
		c->abort_req   = 0U;
		c->state       = (uint8_t)BAESU_IDLE;
		c->state_since = now_ms;
	}

	if (c->start_req)
	{
		c->start_req   = 0U;
		c->state       = (uint8_t)BAESU_TODO;
		c->state_since = now_ms;
		if (c->select_count < 0xFFFFU) { c->select_count++; }
		/* TODO(배수): 배수문 개방 + 배수밸브 개방 유지 시퀀스를 여기서 시작.
		 * 종료는 시간이 아니라 마개 위치 이탈(중재자)이 담당한다.
		 * TODO(음성): "배수 모드 선택" 안내 멘트 (미구현) */
	}

	/* TODO(배수): BAESU_TODO 상태의 실제 유지 tick. */
}

BaesuState Baesu_GetState(void)
{
	return (BaesuState)g_baesu.state;
}

uint8_t Baesu_IsBusy(void)
{
	return (uint8_t)(g_baesu.state != (uint8_t)BAESU_IDLE);
}
