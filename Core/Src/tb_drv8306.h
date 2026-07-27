#ifndef SRC_TB_DRV8306_H_
#define SRC_TB_DRV8306_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Testbed for the DRV8306 BLDC gate drivers (U11 = M1, U16 = M2 on TIM1).
 *
 * Usage (from StartDefaultTask in freertos.c):
 *     TB_DRV8306_Init();
 *     for (;;) { TB_DRV8306_Poll(); osDelay(10); }   // 10 ms cadence
 * or the ready-made loop:
 *     TB_DRV8306_Init();
 *     TB_DRV8306_Loop();                              // never returns
 *
 * Live control from the debugger (watch / expression window):
 *   tb_m1_enable = 1   -> motor M1 spins;  = 0 -> M1 coasts to a stop
 *   tb_m1_rpm          -> target speed in RPM, editable while running
 *   tb_m1_reverse      -> 0 = CW, 1 = CCW (set before/at enable)
 * Read-back (do not write):
 *   tb_m1_meas_rpm     -> speed measured from FGOUT (for calibration)
 *   tb_m1_fault        -> 1 if the DRV8306 latched a fault (nFAULT)
 * M2 has the identical set (tb_m2_*).
 *
 * SPEED / RPM NOTE: the DRV8306 commutates internally from Hall sensors; the
 * MCU only sets PWM DUTY, which is open-loop on speed. tb_mX_rpm is converted
 * to a duty by a *linear approximation* (DRV8306_RpmToDuty). Compare tb_mX_rpm
 * against tb_mX_meas_rpm on the bench and adjust DRV8306_MIN_RPM /
 * DRV8306_MAX_RPM / DRV8306_MIN_DUTY_PCT (and the motor's pole_pairs) in
 * drv8306.h until they agree. Until calibrated, treat RPM as approximate and
 * DRV8306_SetDuty() as the exact control.
 *
 * Default: tb_mX_rpm = DRV8306_MIN_RPM, i.e. the LOWEST speed, and both motors
 * disabled, so nothing spins until you set an enable flag. */

extern volatile uint8_t  tb_m1_enable;    /* 1 = run U11 (M1), 0 = stop      */
extern volatile uint16_t tb_m1_rpm;       /* target speed [MIN_RPM..MAX_RPM] */
extern volatile uint8_t  tb_m1_reverse;   /* 0 = CW, 1 = CCW                 */
extern volatile uint16_t tb_m1_meas_rpm;  /* measured speed (FGOUT), RO      */
extern volatile uint8_t  tb_m1_fault;     /* latched fault flag, RO          */

extern volatile uint8_t  tb_m2_enable;    /* 1 = run U16 (M2), 0 = stop      */
extern volatile uint16_t tb_m2_rpm;       /* target speed [MIN_RPM..MAX_RPM] */
extern volatile uint8_t  tb_m2_reverse;   /* 0 = CW, 1 = CCW                 */
extern volatile uint16_t tb_m2_meas_rpm;  /* measured speed (FGOUT), RO      */
extern volatile uint8_t  tb_m2_fault;     /* latched fault flag, RO          */

void TB_DRV8306_Init(void);
void TB_DRV8306_Poll(void);   /* apply current enable/rpm state once  */
void TB_DRV8306_Loop(void);   /* Init already done; poll forever      */

#ifdef __cplusplus
}
#endif

#endif /* SRC_TB_DRV8306_H_ */
