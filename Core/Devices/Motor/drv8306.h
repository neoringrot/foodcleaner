#ifndef DEVICES_DRV8306_H_
#define DEVICES_DRV8306_H_

#include "main.h"
#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif

/* TI DRV8306 3-phase BLDC gate driver (SLVSE38A), instance-based driver.
 *
 * The board has two DRV8306 gate drivers, each with its own IPT015N10N5
 * 3-phase power stage, driving one large BLDC motor (netlist: U11 = M1,
 * U16 = M2, both VM = 24V):
 *
 *   U11 (M1)  PWM = PE9  (TIM1_CH1)   DIR = PE10   ENABLE = PE7
 *             nBRAKE = PE13           nFAULT = PF14 (EXTI)  FGOUT = PF15 (EXTI)
 *   U16 (M2)  PWM = PE11 (TIM1_CH2)   DIR = PE12   ENABLE = PE8
 *             nBRAKE = PE14           nFAULT = PF13 (EXTI)  FGOUT = PF12 (EXTI)
 *
 * Control model (datasheet 7.3.1.1, "1x PWM Control Mode"):
 * The DRV8306 does the 120-degree, 6-step trapezoidal commutation INTERNALLY
 * from three on-chip Hall comparators. The MCU therefore does NOT generate the
 * six gate signals -- it only provides:
 *
 *   PWM     : one PWM signal whose DUTY CYCLE sets the phase voltage, i.e. the
 *             motor speed (0 % = stopped, 100 % = full voltage, 100 % allowed).
 *             The PWM *frequency* is the phase switching frequency.
 *   DIR     : rotation direction (0 / 1). Swaps the commutation table.
 *   nBRAKE  : active-LOW brake. LOW  -> all high-side off, all low-side on
 *             (short-brake), independent of PWM/DIR. HIGH -> normal run.
 *   ENABLE  : active-HIGH. HIGH -> operating; LOW -> low-power sleep (all
 *             MOSFETs off, charge pump + DVDD LDO off). A >=15..40us LOW pulse
 *             also resets latched faults (datasheet 7.4.1.1). tWAKE (~100us)
 *             must elapse after a rising edge before inputs are accepted.
 *   nFAULT  : open-drain fault, active-LOW input (wired to EXTI falling edge).
 *   FGOUT   : open-drain tacho, one pulse train derived from the Hall states
 *             (datasheet 7.3.5). Falling edges are counted on EXTI to measure
 *             actual motor speed for closed-loop / verification.
 *
 * Because commutation is internal, there is no true torque/current loop here;
 * speed is open-loop on PWM duty. To let the caller command speed in RPM, the
 * driver maps a target RPM to a duty (DRV8306_SetSpeedRPM). That mapping is a
 * linear approximation that MUST be calibrated to the physical motor using the
 * FGOUT-measured RPM (DRV8306_MeasuredRPM) -- see the calibration constants
 * below, which are placeholders until measured on the bench. */

/* ---- PWM ---------------------------------------------------------------- *
 * Phase switching frequency. 20 kHz is inaudible and well under the DRV8306
 * 200-kHz gate-drive limit. TIM1 is on APB2 (APB2CLKDivider = DIV1), so
 * TIM1CLK = HCLK = SystemCoreClock and ARR = SystemCoreClock/freq - 1, the
 * same relation the DRV8871/TIM3 driver relies on. Both motors share TIM1, so
 * this frequency (the ARR) is timer-wide -- both instances use it. */
#define DRV8306_PWM_FREQ_HZ            20000U

/* ---- Speed / RPM calibration (PLACEHOLDERS -- calibrate on the bench) ---- *
 * These describe the physical motor, which the netlist does not specify.
 * Verify each against the actual motor + a scope on FGOUT before trusting the
 * RPM numbers; only the raw duty-cycle path (DRV8306_SetDuty) is exact. */
#define DRV8306_MIN_RPM               500U    /* lowest usable / start speed  */
#define DRV8306_MAX_RPM               3000U   /* approx. speed at 100 % duty  */
#define DRV8306_MIN_DUTY_PCT          10U     /* duty that maps to MIN_RPM    */

/* Motor electrical constants used to convert FGOUT frequency -> mechanical
 * RPM.  mech_RPM = fg_falling_edges/s * 60 / (FG_EDGES_PER_ELEC_REV * poles).
 * pole_pairs is per-instance (struct field); confirm FG_EDGES from Figure 21
 * of the datasheet for this motor's Hall arrangement. */
#define DRV8306_FG_EDGES_PER_ELEC_REV 3U      /* falling edges per elec. rev  */
#define DRV8306_DEFAULT_POLE_PAIRS    4U      /* motor pole pairs (calibrate) */

typedef enum
{
	DRV8306_DIR_CW  = 0,
	DRV8306_DIR_CCW = 1
} DRV8306_Direction;

typedef struct
{
	/* PWM (speed) */
	TIM_HandleTypeDef *htim;        /* PWM timer   (e.g. &htim1)             */
	uint32_t           pwm_ch;      /* TIM_CHANNEL_x driving PWM             */

	/* Control GPIOs */
	GPIO_TypeDef      *en_port;     /* ENABLE  (active high; low = sleep)    */
	uint16_t           en_pin;
	GPIO_TypeDef      *dir_port;    /* DIR                                   */
	uint16_t           dir_pin;
	GPIO_TypeDef      *nbrake_port; /* nBRAKE  (active low)                  */
	uint16_t           nbrake_pin;

	/* Status inputs (EXTI) */
	GPIO_TypeDef      *nfault_port; /* nFAULT  (active low, EXTI)            */
	uint16_t           nfault_pin;
	GPIO_TypeDef      *fgout_port;  /* FGOUT   (tacho, EXTI)                 */
	uint16_t           fgout_pin;

	/* Motor parameter */
	uint16_t           pole_pairs;  /* for FGOUT -> RPM conversion           */

	/* State (filled/maintained by the driver) */
	uint32_t           arr;         /* PWM auto-reload (from Init)           */
	volatile uint32_t  fg_edges;    /* FGOUT falling-edge count (ISR)        */
	uint32_t           fg_last;     /* snapshot for MeasuredRPM window       */
	volatile uint8_t   fault;       /* 1 if nFAULT fell since last clear     */
	uint8_t            enabled;     /* shadow of the ENABLE pin              */
	uint16_t           meas_rpm;    /* last computed measured RPM            */
} DRV8306_HandleTypeDef;

/* Board instances wired per the netlist (defined in drv8306.c). */
extern DRV8306_HandleTypeDef drv8306_m1;   /* U11 */
extern DRV8306_HandleTypeDef drv8306_m2;   /* U16 */

/* ---- Core API ----------------------------------------------------------- */
void     DRV8306_Init(DRV8306_HandleTypeDef *h);
void     DRV8306_InitAll(void);                 /* init drv8306_m1 + drv8306_m2 */

void     DRV8306_Enable(DRV8306_HandleTypeDef *h);   /* ENABLE high (wake)   */
void     DRV8306_Disable(DRV8306_HandleTypeDef *h);  /* ENABLE low  (sleep)  */

void     DRV8306_SetDirection(DRV8306_HandleTypeDef *h, DRV8306_Direction dir);
void     DRV8306_SetDuty(DRV8306_HandleTypeDef *h, uint8_t duty_pct); /* 0-100, exact */

void     DRV8306_Brake(DRV8306_HandleTypeDef *h);    /* nBRAKE low  (short brake) */
void     DRV8306_ReleaseBrake(DRV8306_HandleTypeDef *h);/* nBRAKE high (run mode) */
void     DRV8306_Stop(DRV8306_HandleTypeDef *h);     /* duty 0 + sleep (coast) */

/* RPM helpers (open-loop map + FGOUT feedback). rpm is clamped to
 * [DRV8306_MIN_RPM, DRV8306_MAX_RPM]. */
void     DRV8306_SetSpeedRPM(DRV8306_HandleTypeDef *h, uint16_t rpm);
uint8_t  DRV8306_RpmToDuty(uint16_t rpm);            /* the mapping, exposed  */

/* Recompute measured RPM from FGOUT edges accumulated over window_ms.
 * Call periodically (e.g. every 200 ms) with the elapsed time; returns RPM
 * and also stores it in h->meas_rpm. */
uint16_t DRV8306_MeasuredRPM(DRV8306_HandleTypeDef *h, uint32_t window_ms);

/* Fault helpers. IsFault reads the latched flag; ClearFault pulses the device
 * through sleep (ENABLE low->high) which resets latched faults. */
uint8_t  DRV8306_IsFault(DRV8306_HandleTypeDef *h);
void     DRV8306_ClearFault(DRV8306_HandleTypeDef *h);

/* EXTI hook: call from HAL_GPIO_EXTI_Callback for each edge. Updates fg_edges
 * (FGOUT) and fault (nFAULT) for whichever instance owns GPIO_Pin. */
void     DRV8306_OnEXTI(DRV8306_HandleTypeDef *h, uint16_t GPIO_Pin);

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_DRV8306_H_ */
