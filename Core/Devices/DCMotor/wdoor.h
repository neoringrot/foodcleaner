#ifndef DEVICES_WDOOR_H_
#define DEVICES_WDOOR_H_

#include "drv8871.h"

#ifdef __cplusplus
extern "C" {
#endif

/* U5 -- Water Door DRV8871 (schematic net "W-DOOR").
 *   IN1 = TIM3_CH1 / PC6 (tim3_WDOOR_IN1, net W-DOOR-IN1)
 *   IN2 = TIM3_CH2 / PC7 (tim3_WDDOR_IN2, net W-DOOR-IN2)   [main.h spells it WDDOR]
 *   EN  = PE4 o_EN_DOOR_WATER (net WATER-DOOR-EN)
 *
 * Open/Close map onto DRV8871 Forward/Reverse. CONFIRMED on the bench
 * (2026-08-25): **Forward = 닫힘, Reverse = 열림** -- so WDoor_Open() drives
 * DRV8871_Reverse and WDoor_Close() drives DRV8871_Forward. In the testbench
 * this shows up as tb_wdoor_reverse 0 = 닫힘, 1 = 열림 (opposite of TDoor,
 * where 0 = 열림 -- the two doors are wired the other way round).
 *
 * Limit sensors (R1): two Hall switches report the door end positions, wired
 * straight to the MCU as EXTI inputs:
 *   W-HALL-OPEN  = PF4 exti4_WHALL_OPEN  (GPIO_EXTI_WHALL_OPEN)  -> fully open
 *   W-HALL-CLOSE = PF3 exti3_WHALL_CLOSE (GPIO_EXTI_WHALL_CLOSE) -> fully closed
 * Polarity ACTIVE-LOW (confirmed from the R1 net-list): 3-wire open-drain Hall
 * (connector VCC/OUT/GND) with a 10k pull-up to 3V3 -- pin HIGH when clear,
 * LOW when magnet present. EXTI is IT_FALLING (edge fires at the limit).
 * (Note: W-HALL-CLOSE also feeds HW NOR gate U13 with Bimetal-80 -- a separate
 * interlock, independent of this MCU read.)
 * WDoor_AtOpen()/AtClose() return 1 when the door has reached that limit. A
 * motion controller should stop/brake the motor once the target limit reads 1.
 *
 * 레벨 폴링만으로는 리미트를 "스쳐 지나가는" 경우(관성 오버슈트, 자석 아래를
 * 통과, 채터링)를 놓칠 수 있다. 그래서 도달 판정은 시나리오의 다른 EXTI 센서
 * (WATER_SEN/BIMETAL/TIMER_OUT)와 **동일한 관용구**로 통일한다:
 *
 *   이동 시작 시 WDoor_LimitArm()  -> 이전 이동의 stale 하강엣지 플래그 제거
 *   판정은 WDoor_ReachedOpen/Close() -> "arm 이후의 EXTI 하강엣지" OR 현재 레벨
 *
 * 엣지는 EXTI ISR(main.c 라우터 -> gpio_ctrl_exti_dispatch)에서 래치되므로 폴링
 * 주기와 무관하게 순간 통과도 잡힌다. 레벨 OR를 유지하는 이유는 arm 시점에 이미
 * 리미트에 걸려 있어 새 엣지가 영영 오지 않는 경우(제자리 재구동)를 위해서다.
 * 원래의 순수 레벨 리드가 필요하면 WDoor_AtOpen/AtClose()를 그대로 쓰면 된다. */

/* Asserted (magnet-present) pin level for the door-limit Halls. 0 = active-low
 * (default). Keep this equal to tdoor.h's definition. */
#ifndef DOOR_LIMIT_ACTIVE_HIGH
#define DOOR_LIMIT_ACTIVE_HIGH 0U
#endif

/* 1 = Reached*()가 EXTI 하강엣지 래치도 함께 본다(기본). 0 = 순수 레벨 판정.
 * 핀은 GPIO_MODE_IT_FALLING이므로 "하강엣지 = 리미트 도달"은 active-low일 때만
 * 성립한다 -> DOOR_LIMIT_ACTIVE_HIGH=1이면 엣지는 자동으로 무시된다. */
#ifndef DOOR_LIMIT_USE_EDGE
#define DOOR_LIMIT_USE_EDGE 1U
#endif

/* ---- 도어 구동 프로파일 (2026-08-25 벤치 확정) ------------------------
 * 테스트벤치(tb_drv8871)와 시나리오(모음/동작)가 **같은 값·같은 규칙**으로
 * 배수문을 움직이도록 여기 한 곳에만 둔다. 두 방향의 동작이 다르다:
 *
 *   열림 : WDOOR_OPEN_KICK_DUTY(80 %)로 WDOOR_OPEN_KICK_MS(2 s) 기동 후
 *          WDOOR_OPEN_RUN_DUTY(65 %) 유지. WHALL-OPEN 인식으로 정지하고,
 *          미인식 대비 WDOOR_OPEN_MAX_MS(6 s)가 상한.
 *   닫힘 : WDOOR_CLOSE_DUTY(80 %) 고정, WDOOR_CLOSE_MS(4.2 s) 경과가 정상
 *          종료 조건. 그 전에 WHALL-CLOSE가 인식되면 거기서 정지.
 *
 * 경과시간 el_ms 는 그 방향으로 구동을 시작한 시점부터 잰다. */
#ifndef WDOOR_OPEN_KICK_DUTY
#define WDOOR_OPEN_KICK_DUTY   80U
#endif
#ifndef WDOOR_OPEN_KICK_MS
#define WDOOR_OPEN_KICK_MS     2000U
#endif
#ifndef WDOOR_OPEN_RUN_DUTY
#define WDOOR_OPEN_RUN_DUTY    65U
#endif
#ifndef WDOOR_OPEN_MAX_MS
#define WDOOR_OPEN_MAX_MS      6000U
#endif
#ifndef WDOOR_CLOSE_DUTY
#define WDOOR_CLOSE_DUTY       80U
#endif
#ifndef WDOOR_CLOSE_MS
#define WDOOR_CLOSE_MS         4200U
#endif

/* 열림 구동 duty: 기동 구간이면 킥, 지나면 유지값. 매 tick 호출해 재지령한다. */
static inline uint8_t WDoor_OpenDutyAt(uint32_t el_ms)
{
	return (el_ms < (uint32_t)WDOOR_OPEN_KICK_MS) ? (uint8_t)WDOOR_OPEN_KICK_DUTY
	                                              : (uint8_t)WDOOR_OPEN_RUN_DUTY;
}

void WDoor_Init(void);
void WDoor_Enable(void);          /* switch 24V onto VM */
void WDoor_Disable(void);         /* remove VM          */
void WDoor_Open(uint8_t duty_pct);  /* 0-100 */
void WDoor_Close(uint8_t duty_pct); /* 0-100 */
void WDoor_Brake(void);
void WDoor_Stop(void);            /* coast */

/* Limit-switch level reads: 1 = door is at that end position (magnet present).
 * Raw level only - can miss a limit that is merely passed through. */
uint8_t WDoor_AtOpen(void);
uint8_t WDoor_AtClose(void);

/* Drop any latched WHALL falling edge. Call once when a door move STARTS (not
 * on every re-drive) so the arrival latch below only reports this move. */
void WDoor_LimitArm(void);

/* Arrival test used by the scenarios: latched falling edge since the last
 * WDoor_LimitArm() OR the limit level right now. */
uint8_t WDoor_ReachedOpen(void);
uint8_t WDoor_ReachedClose(void);


/* ---- 모니터링 read-only (protocol_r0 0x26 OUTPUT) ---------------------
 * drive = drv8871_drive_t (0 코스트 / 1 정회전 / 2 역회전 / 3 제동),
 * duty  = 마지막으로 지령한 PWM [%]. 피드백이 아니라 "마지막 지령"이다. */
uint8_t WDoor_GetDrive(void);
uint8_t WDoor_GetDuty(void);

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_WDOOR_H_ */
