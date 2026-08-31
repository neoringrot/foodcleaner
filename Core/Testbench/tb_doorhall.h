#ifndef SRC_TB_DOORHALL_H_
#define SRC_TB_DOORHALL_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * tb_doorhall - testbed for the four door-limit Hall sensors that gate the
 * 배수문(WDoor/U5) and 배출문(TDoor/U7) open/close transitions in the 동작/모음
 * scenarios. These are EXTI inputs (GPIO_MODE_IT_FALLING), read as levels by the
 * door drivers (WDoor_AtOpen/AtClose, TDoor_AtOpen/AtClose):
 *
 *   exti3_WHALL_CLOSE (PF3) : 배수문 닫힘 리밋   -> WDoor_AtClose()
 *   exti4_WHALL_OPEN  (PF4) : 배수문 열림 리밋   -> WDoor_AtOpen()
 *   exti2_THALL_CLOSE (PF2) : 배출문 닫힘 리밋   -> TDoor_AtClose()
 *   exti5_THALL_OPEN  (PF5) : 배출문 열림 리밋   -> TDoor_AtOpen()
 *
 * 센서 모니터라서 다른 EXTI 모니터(tb_water)와 동일하게 액추에이터를 구동하지
 * 않고 관찰만 한다:
 *   - **레벨/at-limit은 StartDefaultTask(~100 ms)에서 폴링** - 런타임에서 핀
 *     전압과 리밋 도달 상태를 그대로 보기 위해 폴링 스냅샷을 유지한다. MotorTask가
 *     아니므로 g_app_mode(테스트벤치/모음/동작)와 무관하게 항상 폴링되지만,
 *     tb_doorhall_enable=0이면 아무 것도 안 한다.
 *   - **falling edge는 EXTI 인터럽트에서 직접 카운트** (TB_DoorHall_OnEXTI).
 *   - 프로덕션 드라이버(WDoor_/TDoor_AtOpen/AtClose)를 그대로 재사용 -> at-limit
 *     판정이 시나리오와 100 % 일치. 이 tb는 raw 레벨과 엣지 카운트를 덧붙일 뿐.
 *
 * at-limit 판정 극성은 wdoor.h/tdoor.h의 DOOR_LIMIT_ACTIVE_HIGH(기본 0=active-low)
 * 를 그대로 따른다(드라이버 함수 재사용). raw 레벨(tb_*_level)은 배선/자석 위치
 * 추적용 진단 뷰다.
 *
 * 엣지 카운트는 **인터럽트 직결**이다. PF2~PF5는 GPIO_MODE_IT_FALLING이고
 * main.c의 중앙 EXTI 라우터(HAL_GPIO_EXTI_Callback)가 falling edge마다
 * TB_DoorHall_OnEXTI(GPIO_Pin)을 호출한다 -> 카운터/타임스탬프를 ISR 문맥에서
 * 바로 갱신하므로 100 ms 폴링 주기 안에 몰린 엣지도 합쳐지지 않는다.
 * (예전엔 gpio_ctrl_exti_flag_*(1비트 래치)를 Poll에서 소비했다 -> 주기당 최대
 * 1엣지, 시각 정보 없음. 이제 이 tb는 그 공유 플래그를 쓰지 않는다.)
 *
 * ISR은 tb_doorhall_enable=0이면 즉시 반환하므로 disable 구간의 엣지는 아예
 * 세지 않는다 -> 예전의 "enable 시 stale 플래그 1회 클리어"가 필요 없다.
 * 리밋 홀/자석의 채터링은 TB_DOORHALL_DEBOUNCE_MS(기본 5 ms) 라인별 소프트
 * 디바운스로 걸러낸다.
 *
 * Usage: 디버거에서 tb_doorhall_enable = 1. 문을 손으로 여닫거나 tb_drv8871의
 * tb_wdoor_/tb_tdoor_ 변수로 구동하면서 at-limit/level/events 필드를 관찰한다.
 * 0으로 되돌리면 모니터링 정지(마지막 스냅샷 동결).
 *
 * TESTBENCH ONLY와 무관: 순수 입력 관찰이라 시나리오와 충돌하지 않는다(액추에이터
 * 미구동). 이제 gpio_ctrl_exti_flag_*를 소비/클리어하지 않으므로 tb_water 등 다른
 * 모니터와 플래그를 두고 다투지도 않는다.
 * ---------------------------------------------------------------------- */

/* Runtime switch -- set from the debugger while running. Default off. */
extern volatile uint8_t tb_doorhall_enable;   /* 1 = monitor, 0 = idle */

/* Raw pin levels (0/1), diagnostic. Updated only while enabled. */
extern volatile uint8_t tb_whall_close_level; /* PF3 raw level */
extern volatile uint8_t tb_whall_open_level;  /* PF4 raw level */
extern volatile uint8_t tb_thall_close_level; /* PF2 raw level */
extern volatile uint8_t tb_thall_open_level;  /* PF5 raw level */

/* at-limit decode (polarity applied, matches the door drivers exactly). */
extern volatile uint8_t tb_wdoor_at_close;    /* 1 = 배수문 닫힘 리밋 도달 */
extern volatile uint8_t tb_wdoor_at_open;     /* 1 = 배수문 열림 리밋 도달 */
extern volatile uint8_t tb_tdoor_at_close;    /* 1 = 배출문 닫힘 리밋 도달 */
extern volatile uint8_t tb_tdoor_at_open;     /* 1 = 배출문 열림 리밋 도달 */

/* Falling-edge counts per line. Incremented in EXTI ISR context. */
extern volatile uint32_t tb_whall_close_events;
extern volatile uint32_t tb_whall_open_events;
extern volatile uint32_t tb_thall_close_events;
extern volatile uint32_t tb_thall_open_events;

/* HAL_GetTick() of the last accepted falling edge per line (0 = none yet). */
extern volatile uint32_t tb_whall_close_tick;
extern volatile uint32_t tb_whall_open_tick;
extern volatile uint32_t tb_thall_close_tick;
extern volatile uint32_t tb_thall_open_tick;

extern volatile uint32_t tb_doorhall_isr_count; /* accepted edges, all 4 lines */
extern volatile uint32_t tb_doorhall_samples;   /* poll count while enabled */

/* Per-line software debounce for the edge counters (ms). Edges closer than this
 * to the previous accepted edge on the SAME line are dropped as chatter. Set to
 * 0 to count every interrupt. */
#ifndef TB_DOORHALL_DEBOUNCE_MS
#define TB_DOORHALL_DEBOUNCE_MS 5U
#endif

/* Clear all state/counters. Call once, after MX_GPIO_Init(). */
void TB_DoorHall_Init(void);

/* Refresh raw levels + at-limit decode while enabled. Call every poll
 * (StartDefaultTask, ~100 ms). Does NOT touch the edge counters. */
void TB_DoorHall_Poll(void);

/* Falling-edge hook. Call from HAL_GPIO_EXTI_Callback() with its GPIO_Pin for
 * every EXTI; ignores pins other than the four door-limit Halls, and does
 * nothing while tb_doorhall_enable == 0. ISR context: no blocking calls. */
void TB_DoorHall_OnEXTI(uint16_t gpio_pin);

#ifdef __cplusplus
}
#endif

#endif /* SRC_TB_DOORHALL_H_ */
