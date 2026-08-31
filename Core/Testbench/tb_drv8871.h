#ifndef SRC_TB_DRV8871_H_
#define SRC_TB_DRV8871_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Testbed for the DRV8871 door motors (U5 Water Door, U7 Trash Door on TIM3).
 *
 * Usage (e.g. from StartDefaultTask in freertos.c):
 *     TB_DRV8871_Init();
 *     for(;;) { TB_DRV8871_Poll(); osDelay(10); }
 * or just call the ready-made loop:
 *     TB_DRV8871_Init();
 *     TB_DRV8871_Loop();     // never returns
 *
 * The tb_*_enable flags are the runtime switches: set one to 1 in the debugger
 * (live watch / expression) and that motor spins; set it back to 0 and it
 * stops. Duty and direction are also tweakable live. */

extern volatile uint8_t tb_wdoor_enable;   /* 1 = run U5 water door, 0 = stop */
extern volatile uint8_t tb_wdoor_duty;     /* 0-100 % PWM                     */
extern volatile uint8_t tb_wdoor_reverse;  /* 0 = Open dir, 1 = Close dir     */

extern volatile uint8_t tb_tdoor_enable;   /* 1 = run U7 trash door, 0 = stop */
extern volatile uint8_t tb_tdoor_duty;     /* 0-100 % PWM                     */
extern volatile uint8_t tb_tdoor_reverse;  /* 0 = Open dir, 1 = Close dir     */

/* Limit auto-stop (default ON). With this set, Poll() reads the door-limit Hall
 * for the direction being driven -- WDoor_AtOpen()/AtClose(), the same decode
 * the scenarios use -- and once it stays asserted for TB_DOOR_LIMIT_CONFIRM
 * consecutive polls it coasts the motor, drops VM, clears tb_*_enable back to 0
 * and raises tb_*_limit_hit. Only the limit in the travel direction blocks, so
 * flipping tb_*_reverse and re-enabling backs the door off the end stop.
 * Set to 0 for free-run (old behaviour) -- e.g. while the open/close direction
 * is still unconfirmed or a Hall is disconnected. Beware: with it off the motor
 * stalls against the end stop until you clear tb_*_enable by hand. */
extern volatile uint8_t tb_wdoor_limit_stop;
extern volatile uint8_t tb_tdoor_limit_stop;

/* Status, read-only: 1 = that door was auto-stopped by its limit. Cleared when
 * the motor is next started away from the limit. */
extern volatile uint8_t tb_wdoor_limit_hit;
extern volatile uint8_t tb_tdoor_limit_hit;

/* WDoor direction profile (default ON). Direction is confirmed on the bench:
 * tb_wdoor_reverse = 0 -> Open, 1 -> Close. The two directions behave
 * differently, both timed from the start of the run:
 *   Open  : tb_wdoor_open_kick_duty (80 %) for tb_wdoor_open_kick_ms (2000 ms)
 *           to break away, then tb_wdoor_open_run_duty (65 %) held. Normally
 *           ends on the WHALL-OPEN limit; tb_wdoor_open_ms (6000 ms) is the
 *           backstop for a Hall that never fires.
 *   Close : tb_wdoor_close_duty (80 %) flat, TIME-driven -- the run ends after
 *           tb_wdoor_close_ms (4200 ms) whether or not WHALL-CLOSE ever
 *           asserts. The limit auto-stop still applies and wins if the Hall
 *           fires first.
 * Either way, a run that ends on its time limit raises tb_wdoor_time_hit; one
 * that ends on a Hall raises tb_wdoor_limit_hit.
 * A run starts on each 0->1 edge of tb_wdoor_enable and restarts if
 * tb_wdoor_reverse is flipped mid-run (the door has to break away again).
 * While the profile is on, Poll() WRITES the applied duty into tb_wdoor_duty
 * every poll -- watch that variable to see the profile, but a duty you type
 * there by hand is overwritten on the next poll. Set tb_wdoor_profile = 0 to
 * hand tb_wdoor_duty back to manual control (its value is then left alone and
 * the Close time limit is not applied either). */
extern volatile uint8_t  tb_wdoor_profile;         /* 1 = profile, 0 = manual  */
extern volatile uint8_t  tb_wdoor_open_kick_duty;  /* 0-100 %, Open breakaway  */
extern volatile uint16_t tb_wdoor_open_kick_ms;    /* Open breakaway window ms */
extern volatile uint8_t  tb_wdoor_open_run_duty;   /* 0-100 %, Open hold duty  */
extern volatile uint16_t tb_wdoor_open_ms;         /* Open run cap [ms]        */
extern volatile uint8_t  tb_wdoor_close_duty;      /* 0-100 %, Close duty flat */
extern volatile uint16_t tb_wdoor_close_ms;        /* Close run length [ms]    */

/* Status, read-only: 1 = the run ended on its time limit rather than on a Hall
 * limit. Cleared when the next run starts. */
extern volatile uint8_t tb_wdoor_time_hit;

/* TDoor profile (default ON). The door is driven at tb_tdoor_run_duty (80 %),
 * and its Hall limit does NOT stop the motor dead: the first confirmed hit
 * latches an overrun window and the door keeps turning for
 * tb_tdoor_overrun_ms (1000 ms) so it travels clear of the sensor, then coasts
 * and clears tb_tdoor_enable. The window is latched, so the level going away
 * as the door passes the magnet does not cut it short; flipping
 * tb_tdoor_reverse mid-run drops it (that is a new run).
 * While the profile is on, Poll() WRITES tb_tdoor_run_duty into tb_tdoor_duty
 * every poll -- set tb_tdoor_profile = 0 to hand tb_tdoor_duty back to manual
 * control, which also makes the limit stop the motor immediately (overrun 0).
 * Each direction also has its own hard run-time cap (14300 ms both, kept as
 * separate variables so they can be tuned apart): the backstop for a Hall that
 * never fires. It bounds the WHOLE run, so it also truncates an overrun that
 * was latched too close to the cap, and it raises tb_tdoor_time_hit instead of
 * tb_tdoor_limit_hit. tb_tdoor_profile = 0 removes the caps as well. */
extern volatile uint8_t  tb_tdoor_profile;       /* 1 = profile, 0 = manual  */
extern volatile uint8_t  tb_tdoor_run_duty;      /* 0-100 %, flat drive duty */
extern volatile uint16_t tb_tdoor_overrun_ms;    /* extra run past Hall [ms] */
extern volatile uint16_t tb_tdoor_open_max_ms;   /* Open run cap [ms]        */
extern volatile uint16_t tb_tdoor_close_max_ms;  /* Close run cap [ms]       */

/* Status, read-only: 1 = Hall seen, door inside the extra-run window. */
extern volatile uint8_t tb_tdoor_overrun;

/* Status, read-only: 1 = the run ended on its time cap, not on the Hall.
 * Cleared when the next run starts. */
extern volatile uint8_t tb_tdoor_time_hit;

void TB_DRV8871_Init(void);
void TB_DRV8871_Poll(void);   /* apply current enable/duty state once */
void TB_DRV8871_Loop(void);   /* Init already done; poll forever      */

#ifdef __cplusplus
}
#endif

#endif /* SRC_TB_DRV8871_H_ */
