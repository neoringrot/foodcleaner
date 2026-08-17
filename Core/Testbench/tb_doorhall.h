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
 *   - **StartDefaultTask(~100 ms)에서 폴링** - 이 태스크가 EXTI 플래그를
 *     폴링/클리어하는 소유자다. MotorTask가 아니므로 g_app_mode(테스트벤치/모음/
 *     동작)와 무관하게 항상 폴링되지만, tb_doorhall_enable=0이면 아무 것도 안 한다.
 *   - 프로덕션 드라이버(WDoor_/TDoor_AtOpen/AtClose)를 그대로 재사용 -> at-limit
 *     판정이 시나리오와 100 % 일치. 이 tb는 raw 레벨과 엣지 카운트를 덧붙일 뿐.
 *
 * at-limit 판정 극성은 wdoor.h/tdoor.h의 DOOR_LIMIT_ACTIVE_HIGH(기본 0=active-low)
 * 를 그대로 따른다(드라이버 함수 재사용). raw 레벨(tb_*_level)은 배선/자석 위치
 * 추적용 진단 뷰다.
 *
 * 엣지 카운트: PF2~PF5는 GPIO_MODE_IT_FALLING이고 main.c의 중앙 EXTI 라우터가
 * 모든 핀을 gpio_ctrl_exti_dispatch()로 래치하므로, 여기서 falling edge를 센다.
 * enable 엣지에서 stale 플래그를 1회 클리어해 시작 시점의 레벨을 가짜 엣지로
 * 세지 않는다.
 *
 * Usage: 디버거에서 tb_doorhall_enable = 1. 문을 손으로 여닫거나 tb_drv8871의
 * tb_wdoor_/tb_tdoor_ 변수로 구동하면서 at-limit/level/events 필드를 관찰한다.
 * 0으로 되돌리면 모니터링 정지(마지막 스냅샷 동결).
 *
 * TESTBENCH ONLY와 무관: 순수 입력 관찰이라 시나리오와 충돌하지 않는다(액추에이터
 * 미구동, EXTI 플래그만 소비). 다만 tb_water와 같은 100 ms 태스크에서 EXTI
 * 플래그를 클리어하므로, 엣지 카운트가 필요한 다른 모니터와 동일 핀을 공유하지는
 * 않는다(도어홀 4핀 전용).
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

/* Falling-edge counts per line. */
extern volatile uint32_t tb_whall_close_events;
extern volatile uint32_t tb_whall_open_events;
extern volatile uint32_t tb_thall_close_events;
extern volatile uint32_t tb_thall_open_events;

extern volatile uint32_t tb_doorhall_samples; /* poll count while enabled */

/* Clear all state/flags. Call once, after MX_GPIO_Init(). */
void TB_DoorHall_Init(void);

/* Refresh levels/at-limit/edge counts while enabled. Call every poll. */
void TB_DoorHall_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* SRC_TB_DOORHALL_H_ */
