#include "guide_edge.h"
#include "hallsensor.h"
#include "gpio_ctrl.h"

/* 가이드(HS6 = U24 P5 = J28-2) 엣지 카운터. 설계 근거·규약은 guide_edge.h 참조.
 * 요약: PF9(HALL-INT1) 플래그가 섰을 때만 I2C 1회를 읽고, 자기 prev 와 비교해
 * 센다. 카운터는 리셋하지 않고 baseline 차이로 쓴다. */

static volatile uint32_t s_count;          /* 단조 증가 엣지 수              */
static volatile uint32_t s_last_edge_ms;   /* 마지막으로 센 엣지 tick        */
static volatile uint32_t s_last_interval;  /* 직전 두 엣지 간격(ms)          */
static volatile uint8_t  s_level;          /* 현재 가이드 비트               */
static volatile uint8_t  s_both;           /* 0=상승만 1=양엣지 (I02)        */
static uint8_t           s_prev;           /* 마지막으로 본 레벨             */
static uint8_t           s_have_prev;      /* 0 = 아직 seed 전               */
static uint8_t           s_seeded_once;    /* 첫 엣지 전에는 interval 무의미 */

static uint8_t guide_bit_now(void)
{
	return (uint8_t)(HallSensor_Get((uint8_t)GUIDE_EDGE_HS_BIT) ? 1U : 0U);
}

void GuideEdge_Init(void)
{
	s_count         = 0U;
	s_last_edge_ms  = 0U;
	s_last_interval = 0U;
	s_level         = 0U;
	s_both          = (uint8_t)GUIDE_EDGE_BOTH;
	s_prev          = 0U;
	s_have_prev     = 0U;   /* 첫 Tick 이 현재 레벨을 seed - 부팅 레벨은 엣지가 아니다 */
	s_seeded_once   = 0U;
}

void GuideEdge_Tick(uint32_t now_ms)
{
	uint8_t lvl;

	/* U24 INT 가 떠 있을 때만 버스를 건드린다. ServiceInt() 는 플래그가 없으면
	 * 그대로 0 을 돌려주는 no-op 이라, 평상시 1ms 루프에 I2C 부하가 없다.
	 * (플래그가 섰는데 defaultTask 가 먼저 소비했더라도 아래 캐시 비교로
	 *  변화는 여전히 한 번 잡힌다 - prev 를 이 모듈이 따로 갖는 이유다.) */
	(void)HallSensor_ServiceInt();

	lvl     = guide_bit_now();
	s_level = lvl;

	if (!s_have_prev)                      /* 첫 호출: 현재 레벨을 기준으로만 */
	{
		s_prev      = lvl;
		s_have_prev = 1U;
		return;
	}
	if (lvl == s_prev) { return; }         /* 변화 없음                       */

	/* 채터 억제. 디바운스 구간 안의 변화는 prev 도 갱신하지 않는다 - 갱신하면
	 * 되돌아온 파형을 다음 tick 에서 '새 엣지'로 세게 된다. */
	if ((GUIDE_EDGE_DEBOUNCE_MS != 0U) && s_seeded_once &&
	    ((now_ms - s_last_edge_ms) < (uint32_t)GUIDE_EDGE_DEBOUNCE_MS))
	{
		return;
	}

	s_prev = lvl;

	/* 상승엣지만 셀지 양엣지를 셀지는 I02 회신 전까지 런타임 전환(벤치 실측용). */
	if ((s_both == 0U) && (lvl == 0U)) { return; }   /* 이탈은 세지 않음 */

	if (s_seeded_once) { s_last_interval = now_ms - s_last_edge_ms; }
	s_last_edge_ms = now_ms;
	s_seeded_once  = 1U;
	s_count++;
}

uint32_t GuideEdge_Count(void)          { return s_count; }
uint32_t GuideEdge_LastEdgeMs(void)     { return s_last_edge_ms; }
uint8_t  GuideEdge_Level(void)          { return s_level; }
uint8_t  GuideEdge_HasPrev(void)        { return s_have_prev; }
uint32_t GuideEdge_LastIntervalMs(void) { return s_last_interval; }

void GuideEdge_SetBothEdges(uint8_t both)
{
	s_both = (uint8_t)(both ? 1U : 0U);
}

uint8_t GuideEdge_GetBothEdges(void)    { return s_both; }
