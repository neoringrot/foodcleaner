#ifndef SRC_TB_TCA9554_H_
#define SRC_TB_TCA9554_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * tb_tca9554 - front-panel keypad (U9) + LED (U8) testbed that drives the two
 * DRV8306 BLDC motors (M1 grinder, M2 stirrer) from button presses.
 *
 * Hardware (I2C1) - ADDRESSING CONFIRMED ON THE BENCH, and it is the REVERSE of
 * the netlist labels in Devices/ExtGpio/tca9554.h: on this board the keypad and
 * LED expanders are swapped, so this testbed uses the addresses below (verified
 * working - keypad reads and LEDs light correctly):
 *   0x39 (A0=1, netlist "U8")  P0..P7 = DIS-SW1..8   -> buttons, read as INPUTS
 *   0x38 (A0=0, netlist "U9")  P0..P7 = DIS-LED1..8  -> panel LEDs, OUTPUTS
 * Hence TB_TCA9554_Init() binds  s_sw -> TCA9554_U8_ADDR(0x39),
 *                                s_led-> TCA9554_U9_ADDR(0x38).
 * Both lines are ACTIVE-LOW on this board (press = pin LOW, LED lit = pin LOW),
 * so TB_TCA9554_SW_ACTIVE_HIGH / _LED_ACTIVE_HIGH are both 0. See the netlist
 * analysis doc (doc/회로도_넷리스트_분석.md, section 8) for the full note.
 *
 * IMPORTANT - single owner of U8/U9:
 *   The generic Membrane_Init()/Membrane_Poll() driver ALSO owns U8/U9 and
 *   implements a different behaviour (press = latch/toggle the LED). It must be
 *   DISABLED while this testbed runs, or the two will fight over the LED port
 *   and the I2C bus. Membrane_Init() (main.c) and Membrane_Poll() (freertos.c)
 *   are commented out for this reason; re-enable them if you drop this testbed.
 *
 * LED behaviour (TCA9554 has NO PWM/dimming - it is a pure digital I/O
 * expander, registers are only Input/Output/Polarity/Config). So instead of
 * dimming on press, every DIS-LED is driven HIGH (lit) by default, and while a
 * button is HELD its matching LED is turned OFF; releasing lights it again.
 * SWn <-> LEDn, 1:1 for all 8 channels.
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
 *                 ladder (1:49 geared): 20 -> 25 -> 30 -> 35 -> 40 -> 20 [rpm]
 *
 * BOTH motors are commanded CLOSED-LOOP through bldc_ctrl: M1 = g_grind_ctrl,
 * M2 = g_stir_ctrl (BldcCtrl_Start/Stop/SpeedStep), and the "stopped?" gate is
 * BldcCtrl_IsRunning(). This testbed sits ON TOP of both controllers and must be
 * polled in the same task (StartMotorTask) as their BldcCtrl_Tick() calls, so
 * each controller is touched from one context only.
 *
 * Call model (StartMotorTask, freertos.c):
 *     TB_DRV8306_Init();  TB_TCA9554_Init();
 *     for (;;) { TB_TCA9554_Poll(); TB_DRV8306_Poll(); ...; osDelay(1); }
 * TB_TCA9554_Poll() does one U9 read + at most one U8 write per call.
 * ---------------------------------------------------------------------- */

#define TB_TCA9554_BTN_COUNT   8U    /* SW1..8 / LED1..8 */

/* Debounce depth in poll samples. StartMotorTask polls at ~1 ms, so 8 samples
 * ~= 8 ms of stable contact - above membrane bounce, still responsive. */
#ifndef TB_TCA9554_DEBOUNCE_SAMPLES
#define TB_TCA9554_DEBOUNCE_SAMPLES  8U
#endif

/* Electrical sense of a *pressed* key on the U9 input. This board is ACTIVE-LOW
 * (a press pulls the pin to GND), so this is 0: TB_TCA9554_Init() then programs
 * U9's polarity-inversion register (0xFF) so a press still reads as logical 1
 * and edge detection stays polarity-agnostic. Set 1 if a press drives HIGH. */
#ifndef TB_TCA9554_SW_ACTIVE_HIGH
#define TB_TCA9554_SW_ACTIVE_HIGH    0U
#endif

/* Drive level that LIGHTS a DIS-LED. This board wires the LEDs ACTIVE-LOW
 * (common-anode / sinking), so this is 0: the port value is inverted, a LOW pin
 * lights the LED. Set 1 for active-high (HIGH lights it). */
#ifndef TB_TCA9554_LED_ACTIVE_HIGH
#define TB_TCA9554_LED_ACTIVE_HIGH   0U
#endif

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

/* ---- Lifecycle ------------------------------------------------------------ */
/* Bring up U9 (all inputs) and U8 (all outputs, all LEDs lit), seed debounce,
 * and reset both motors' speed ladder to the start duty. Call once, after
 * MX_I2C1_Init() and TB_DRV8306_Init(). */
void TB_TCA9554_Init(void);

/* One service cycle: read U9, debounce, run button actions on fresh presses,
 * reflect held keys onto U8 (held -> its LED off). Call every poll (~1 ms). */
void TB_TCA9554_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* SRC_TB_TCA9554_H_ */
