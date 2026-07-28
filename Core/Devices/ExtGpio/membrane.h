#ifndef DEVICES_MEMBRANE_H_
#define DEVICES_MEMBRANE_H_

#include "main.h"
#include "tca9554.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * membrane - front-panel membrane keypad + LED latch built on two TCA9554A.
 *
 *   U9 (0x39)  P0..P7 = DIS-SW1..8    -> membrane keypad, read as inputs
 *   U8 (0x38)  P0..P7 = DIS-LED1..8   -> panel LEDs, driven as outputs
 *
 * Behaviour (per request): each key press is a RISING EDGE that TOGGLES the
 * matching LED like a latch. Press DIS-SW1 -> DIS-LED1 lights; press it again
 * -> DIS-LED1 goes dark. SWn maps to LEDn 1:1 for all 8 channels.
 *
 * Why polling, not a hardware interrupt:
 *   The TCA9554 INT output (pin 13, open-drain, active-low) is NOT wired to
 *   the MCU on this board, so an MCU EXTI line cannot be triggered by the
 *   expander. Even if it were wired, INT asserts on ANY input change (both
 *   press and release) and self-clears on an input-port read, so "rising edge
 *   only" still requires a software compare. We therefore poll U9 over I2C on
 *   a fixed cadence and detect edges in firmware. Membrane switches bounce, so
 *   a sample must be stable for MEMBRANE_DEBOUNCE_SAMPLES polls before it is
 *   accepted; the LED toggles on the debounced 0->1 (release->press) edge.
 *
 * Call model:
 *   Membrane_Init()  once, after MX_I2C1_Init().
 *   Membrane_Poll()  periodically (every ~MEMBRANE_POLL_MS ms) from a task or
 *                    timer. It does one U9 read + at most one U8 write.
 * ---------------------------------------------------------------------- */

#define MEMBRANE_KEY_COUNT        8U    /* DIS-SW1..8 / DIS-LED1..8            */

/* Poll cadence the caller should honour, and the debounce depth. 5 ms x 3
 * samples = ~15 ms of stable contact required, comfortably above membrane
 * bounce while staying responsive. */
#ifndef MEMBRANE_POLL_MS
#define MEMBRANE_POLL_MS          5U
#endif
#ifndef MEMBRANE_DEBOUNCE_SAMPLES
#define MEMBRANE_DEBOUNCE_SAMPLES 3U
#endif

/* Electrical sense of a *pressed* key on the U9 input pin. Membrane keypads
 * are commonly active-low (press pulls the line to GND). Set this to 0 for
 * active-low: the driver then programs U9's polarity-inversion register so a
 * press still reads as logical 1, giving a clean rising edge. Set to 1 if a
 * press drives the pin HIGH. */
#ifndef MEMBRANE_SW_ACTIVE_HIGH
#define MEMBRANE_SW_ACTIVE_HIGH   1U
#endif

/* Drive level that lights a DIS-LED. TCA9554 push-pull outputs can source or
 * sink; set to 0 if the LEDs are wired active-low (cathode-common / sinking). */
#ifndef MEMBRANE_LED_ACTIVE_HIGH
#define MEMBRANE_LED_ACTIVE_HIGH  1U
#endif

/* Optional hook: called from Membrane_Poll() context whenever a key toggles.
 * key = 0..7 (SW1..8), led_on = new latch state. May be NULL. */
typedef void (*membrane_key_cb_t)(uint8_t key, uint8_t led_on);

/* ---- Lifecycle ------------------------------------------------------- */
/* Bring up both expanders (U9 all-inputs, U8 all-outputs, all LEDs off) and
 * seed the debounce state from the current keypad reading. Returns HAL_OK only
 * if both devices initialise on the bus. */
HAL_StatusTypeDef Membrane_Init(membrane_key_cb_t cb);

/* One service cycle: read U9, debounce, toggle latch(es) on rising edges,
 * write U8 if anything changed. Call every ~MEMBRANE_POLL_MS ms. Returns
 * HAL_OK on a clean bus cycle. */
HAL_StatusTypeDef Membrane_Poll(void);

/* ---- LED latch access ------------------------------------------------ */
uint8_t           Membrane_GetLed(uint8_t key);            /* 0..7 -> 0/1 latch */
uint8_t           Membrane_GetLedMask(void);               /* bit i = LEDi+1    */
HAL_StatusTypeDef Membrane_SetLed(uint8_t key, uint8_t on);/* force one latch   */
HAL_StatusTypeDef Membrane_SetLedMask(uint8_t mask);       /* force all latches */

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_MEMBRANE_H_ */
