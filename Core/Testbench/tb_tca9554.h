#ifndef SRC_TB_TCA9554_H_
#define SRC_TB_TCA9554_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * tb_tca9554 - front-panel keypad testbed: turns button presses into DRV8306
 * BLDC commands (M1 grinder, M2 stirrer).
 *
 * OWNERSHIP (REV02): this file no longer touches I2C. Devices/ExtGpio/keypad.*
 * is the single owner of U31 (0x39, DIS-SW1..8) and U32 (0x38, DIS-LED1..8) and
 * does the debouncing, the press/long-press events and the LED latch; this
 * testbed only calls Keypad_TakePress(). Wiring, addresses and polarity live in
 * keypad.h. (Before REV02 this file owned the two expanders itself - see the
 * deleted membrane.* / R2 notes.)
 *
 * LED behaviour comes from keypad.c: TCA9554 has no PWM, so the panel is dark
 * and only a HELD key's LED lights until release (KEYPAD_LED_FOLLOW_PRESS).
 * SWn <-> LEDn, 1:1 for all 8 channels. The LEDs are ACTIVE-HIGH - confirmed on
 * the REV02 bench 2026-09-20; the old "active-low + lit by default" pair in this
 * file cancelled out to the same waveform, and keypad.h now states it correctly.
 *
 * Button -> motor map (buttons are 1-based on the panel; index below is 0-based):
 *   Odd  buttons SW1/3/5/7 -> M1 (grinder, drv8306_m1 / U11)
 *   Even buttons SW2/4/6/8 -> M2 (stirrer, drv8306_m2 / U16)
 *
 *   SW1 / SW2 : start FORWARD (CW)   - only reacts while that motor is STOPPED
 *   SW3 / SW4 : STOP
 *   SW5 / SW6 : start REVERSE (CCW)  - only reacts while that motor is STOPPED
 *   SW7 (M1)  : speed up one rung each press. M1 (direct drive) speed is a
 *                 MOTOR-RPM ladder; every start begins at 800 rpm, each press
 *                 steps up, and a press at the top wraps back:
 *                 -> 800 -> 1200 -> 1600 -> 2000 -> 2500 -> 800 -> ... [rpm]
 *   SW8 (M2)  : speed up one rung each press on the STIRRER's OUTPUT-shaft
 *                 ladder (blade rpm, §0.22): 20 -> 25 -> 30 -> 35 -> 40 -> 20 [rpm]
 *
 * BOTH motors are commanded CLOSED-LOOP through bldc_ctrl: M1 = g_grind_ctrl,
 * M2 = g_stir_ctrl (BldcCtrl_Start/Stop/SpeedStep), and the "stopped?" gate is
 * BldcCtrl_IsRunning(). This testbed sits ON TOP of both controllers and must be
 * polled in the same task (StartMotorTask) as their BldcCtrl_Tick() calls, so
 * each controller is touched from one context only.
 *
 * Call model (StartMotorTask, freertos.c):
 *     Keypad_Init(); TB_DRV8306_Init(); TB_TCA9554_Init();
 *     for (;;) { Keypad_Tick(now); ...; TB_TCA9554_Poll(); ...; osDelay(1); }
 * TB_TCA9554_Poll() itself does no bus traffic at all.
 * ---------------------------------------------------------------------- */

#define TB_TCA9554_BTN_COUNT   8U    /* SW1..8 / LED1..8 */

/* Debounce, switch polarity and LED polarity now live in keypad.h
 * (KEYPAD_DEBOUNCE_MS / KEYPAD_SW_ACTIVE_HIGH / KEYPAD_LED_ACTIVE_HIGH). */

/* The SW7 (M1) and SW8 (M2) speed ladders are defined in bldc_ctrl.c as
 * GRIND_LADDER[] (motor RPM) and STIR_LADDER[] (output RPM). Edit those arrays
 * to change the rungs. */

/* ---- Current button state (per request: organised as an array) -----------
 * One entry per SWn, index 0 = SW1 .. 7 = SW8. Debounced; snapshot the debugger
 * can watch. `edge` is the rising (fresh-press) flag consumed by Poll() to run
 * the action once per press; `press_cnt` is a diagnostic press tally. */
typedef struct
{
	uint8_t  pressed;    /* 1 while the key is held (debounced)            */
	uint8_t  edge;       /* 1 on the poll a fresh press was accepted (RO)  */
	uint32_t press_cnt;  /* total accepted presses since Init (diagnostic) */
} tb_tca9554_btn_t;

extern volatile tb_tca9554_btn_t tb_btn[TB_TCA9554_BTN_COUNT];

/* Debounced pressed mask (bit i = SW(i+1) held) and the current LED latch mask
 * (bit i = LED(i+1) lit). Handy single-word views for the debugger. */
extern volatile uint8_t tb_btn_mask;   /* pressed keys  */
extern volatile uint8_t tb_led_mask;   /* lit LEDs      */

/* ---- Debug enable (TESTBENCH ONLY) ---------------------------------------
 * Drive the grinder (M1) / stirrer (M2) WITHOUT the panel keypad: write these
 * from the debugger (or code) and TB_TCA9554_Poll() applies them exactly as if
 * the matching button had been pressed. They are ignored outside testbench mode
 * because Poll() only runs in MotorTask_RunTestbench().
 *
 *   tb_*_en     : level. 0 -> 1 edge = START (like SW1/SW2, dir from tb_*_rev),
 *                        1 -> 0 edge = STOP  (like SW3/SW4). Reflects intent, not
 *                        live running state (a jam soft-lock can stop the motor
 *                        while this stays 1; clear to 0 then back to 1 to retry).
 *   tb_*_rev    : direction sampled at the START edge. 0 = forward/CW, 1 = rev/CCW.
 *   tb_*_spd_req: write 1 to advance one speed rung (like SW7/SW8); auto-clears. */
/* 키패드 -> 모터 결선 차단(기본 1 = 종전대로 동작).
 * 0 으로 쓰면 버튼을 눌러도 BldcCtrl_Start/Stop/SpeedStep 을 호출하지 않는다.
 * J27 8채널 대응 확인처럼 "버튼이 어느 핀인지"만 볼 때, SW1 한 번에 분쇄
 * BLDC 가 도는 것을 막기 위한 것이다. tb_btn[]/press_cnt 관측은 그대로 된다.
 * 디버거 enable 변수(tb_grind_en 등)는 이 플래그와 무관하게 계속 듣는다. */
extern volatile uint8_t tb_keypad_motor_en;

extern volatile uint8_t tb_grind_en;       /* M1 grinder: 1 = run, 0 = stop     */
extern volatile uint8_t tb_grind_rev;      /* M1 start direction (0 fwd, 1 rev)  */
extern volatile uint8_t tb_grind_spd_req;  /* M1 speed-step request (self-clears)*/
extern volatile uint8_t tb_stir_en;        /* M2 stirrer: 1 = run, 0 = stop      */
extern volatile uint8_t tb_stir_rev;       /* M2 start direction (0 fwd, 1 rev)  */
extern volatile uint8_t tb_stir_spd_req;   /* M2 speed-step request (self-clears)*/

/* ---- Lifecycle ------------------------------------------------------------ */
/* Seed the debugger views and the debug-enable edge state. No bus traffic -
 * call once, AFTER Keypad_Init() and TB_DRV8306_Init(). */
void TB_TCA9554_Init(void);

/* One service cycle: consume this cycle's keypad press events and run the bound
 * motor action, then refresh the debugger views. Call every poll (~1 ms), after
 * Keypad_Tick(). */
void TB_TCA9554_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* SRC_TB_TCA9554_H_ */
