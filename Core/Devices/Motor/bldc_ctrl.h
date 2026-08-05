#ifndef DEVICES_BLDC_CTRL_H_
#define DEVICES_BLDC_CTRL_H_

#include "main.h"
#include "drv8306.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * bldc_ctrl - generic closed-loop controller for the two DRV8306 BLDC motors.
 *
 * Both motors are commanded in RPM and held with a FGOUT closed-loop PI (the
 * per-mille duty / feed-forward / anti-windup engine in drv8306.h), plus a
 * soft-lock jam protector. The only per-motor differences (which DRV8306, the
 * speed ladder, PI gains, stall thresholds) live in a BldcCtrl_Cfg_t, so the
 * exact same control code drives:
 *
 *   g_grind_ctrl : M1 / U11, JK60BLS03 grinder, DIRECT drive (gear_ratio = 1),
 *                  so "output RPM" == motor RPM. Ladder in MOTOR RPM.
 *   g_stir_ctrl  : M2 / U16, JK42BLS02 stirrer behind a 1:49 planetary gearbox
 *                  (rated 4.4 N-m / 65 RPM output). Ladder in OUTPUT RPM; the
 *                  gearbox conversion is handled inside the DRV8306 layer.
 *
 * UNITS GOTCHA: the PI error is in whatever unit the ladder/target uses -- for
 * M1 that is MOTOR RPM (0..2500), for M2 OUTPUT RPM (0..40). That is why the two
 * configs use DIFFERENT PI gains (a gain that is gentle on M2's small numbers
 * would be violent on M1's large ones). See bldc_ctrl.c.
 *
 * Current/torque: there is NO firmware current loop; the DRV8306 stage current
 * limit is set in HARDWARE (CSA gain 3, Rsense 0.018 ohm -> ~4.6 A). That
 * protects the FETs, so the SOFT-LOCK here is the motors' effective protection
 * against a sustained mechanical jam (rice-cake / crab-shell on the grinder,
 * debris on the stirrer). Keep it enabled.
 *
 * Ownership: g_grind_ctrl owns drv8306_m1 and g_stir_ctrl owns drv8306_m2,
 * EXCLUSIVELY -- nothing else may drive those handles (this replaces the old
 * open-loop tb_drv8306 bench).
 *
 * Call model (StartMotorTask, freertos.c), after DRV8306_InitAll():
 *     BldcCtrl_Init(&g_grind_ctrl);
 *     BldcCtrl_Init(&g_stir_ctrl);
 *     for (;;) {
 *         ...; TB_TCA9554_Poll();
 *         BldcCtrl_Tick(&g_grind_ctrl, HAL_GetTick());
 *         BldcCtrl_Tick(&g_stir_ctrl,  HAL_GetTick());
 *         osDelay(1);
 *     }
 * BldcCtrl_Tick() self-times to cfg->period_ms (100 ms), so calling it every
 * ~1 ms is fine; the PI, the FGOUT window and the stall timing all run at that
 * cadence.
 * ========================================================================== */

/* Control-loop state (also the debugger view of BldcCtrl_t.state). */
typedef enum
{
	BLDC_IDLE   = 0,   /* stopped, armed                                    */
	BLDC_RUN    = 1,   /* PI holding target RPM                             */
	BLDC_UNJAM  = 2,   /* reverse push to clear a detected jam              */
	BLDC_LOCKED = 3    /* soft-locked (jam retries exhausted or nFAULT);    */
	                   /* stays here until BldcCtrl_Stop() re-arms it.      */
} bldc_state_t;

/* Immutable per-motor configuration (one static instance per motor). */
typedef struct
{
	DRV8306_HandleTypeDef *h;          /* the motor this controller owns     */
	const uint16_t        *ladder;     /* speed rungs [RPM] (motor or output)*/
	uint8_t                ladder_len; /* number of rungs                    */

	/* PI gains (integer fractions; see DRV8306_PI_Compute). err is in the
	 * ladder's RPM unit; duty is per-mille. */
	int32_t  kp_num, kp_den;
	int32_t  ki_num, ki_den;
	int16_t  duty_min_pm;              /* duty clamp low  [per-mille]         */
	int16_t  duty_max_pm;             /* duty clamp high [per-mille]         */

	/* Soft-lock (jam) protection. */
	uint16_t stall_pct;               /* stall if meas < target*pct/100 ...  */
	int16_t  duty_sat_pm;             /* ...AND duty >= this (saturated) ...  */
	uint32_t stall_ms;                /* ...sustained this long -> jam.       */
	uint32_t unjam_ms;                /* reverse-push duration to clear jam.  */
	int16_t  unjam_pm;                /* fixed duty during unjam (HW I-lim).  */
	uint8_t  retry_max;               /* unjam attempts before latching.      */

	/* Setpoint slew rate [RPM/s]: the PI setpoint accelerates/decelerates
	 * toward the commanded target at this rate instead of stepping, so a
	 * button/target change does not produce a PI error step (current spike). */
	uint16_t slew_rpm_per_s;

	uint32_t period_ms;               /* control cadence (PI + window + FSM). */
	uint32_t wake_mask_ms;            /* tWAKE nFAULT spurious-blip mask.     */
} BldcCtrl_Cfg_t;

/* Live controller state. The `volatile` members are the debugger view:
 * target_out_rpm / reverse are the command (writable), the rest are read-only
 * status. For M1 (gear 1) *_out_rpm are motor RPM; for M2 they are output RPM. */
typedef struct
{
	const BldcCtrl_Cfg_t *cfg;        /* bound at definition time            */
	DRV8306_PI_t          pi;         /* PI state                            */

	uint8_t  spd_idx;                 /* current ladder rung                 */
	uint32_t last_tick;               /* HAL_GetTick() of the last step      */
	uint32_t wake_tick;               /* start of the wake-fault mask window */
	uint32_t stall_since;             /* tick the stall condition first held */
	uint32_t unjam_until;             /* tick the unjam push should end      */
	uint8_t  stall_active;            /* 1 while the stall condition is timed */

	volatile uint16_t target_out_rpm; /* commanded RPM (the goal; writable)  */
	volatile uint16_t sp_out_rpm;     /* PI setpoint, slewed toward target(RO)*/
	volatile uint16_t meas_out_rpm;   /* measured RPM (RO)                   */
	volatile int16_t  duty_pm;        /* applied duty [per-mille] (RO)       */
	volatile uint8_t  running;        /* 1 = spinning (RO)                   */
	volatile uint8_t  reverse;        /* base dir: 0 = CW, 1 = CCW           */
	volatile uint8_t  state;          /* bldc_state_t (RO)                   */
	volatile uint8_t  fault;          /* latched nFAULT (RO)                 */
	volatile uint8_t  retry_cnt;      /* unjam attempts so far (RO)          */
} BldcCtrl_t;

/* The two board motors as closed-loop controllers (defined in bldc_ctrl.c). */
extern BldcCtrl_t g_grind_ctrl;       /* M1 / U11 grinder (direct drive)     */
extern BldcCtrl_t g_stir_ctrl;        /* M2 / U16 stirrer (1:49 geared)      */

/* ---- API ------------------------------------------------------------------
 * The motor handle (c->cfg->h) must already be brought up by DRV8306_InitAll();
 * BldcCtrl_Init() leaves it parked/disabled. */
void    BldcCtrl_Init(BldcCtrl_t *c);

/* Run the control law when a cfg->period_ms window has elapsed. Pass
 * HAL_GetTick(); safe to call every poll (it self-times). */
void    BldcCtrl_Tick(BldcCtrl_t *c, uint32_t now_ms);

/* Panel actions (called from tb_tca9554 on a fresh SWn press). */
void    BldcCtrl_Start(BldcCtrl_t *c, uint8_t reverse); /* start; resumes last speed rung */
void    BldcCtrl_Stop(BldcCtrl_t *c);                   /* stop + re-arm (clears lock; keeps rung) */
void    BldcCtrl_SpeedStep(BldcCtrl_t *c);              /* next ladder rung (cycles)   */

/* 1 = motor currently spinning (RUN or UNJAM); mirrors c->running. */
uint8_t BldcCtrl_IsRunning(const BldcCtrl_t *c);

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_BLDC_CTRL_H_ */
