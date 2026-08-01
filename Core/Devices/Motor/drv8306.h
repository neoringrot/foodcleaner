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
 * linear approximation now anchored to the JK60BLS03-XX nameplate (24 V bus,
 * Ke = 6 V/kRPM, 4000 RPM no-load) -- see the calibration constants below.
 * It is still a NO-LOAD relation, so where the RPM must be accurate under load,
 * trim against the FGOUT-measured RPM (DRV8306_MeasuredRPM). */

/* ---- PWM ---------------------------------------------------------------- *
 * Phase switching frequency. 20 kHz is inaudible and well under the DRV8306
 * 200-kHz gate-drive limit. TIM1 is on APB2 (APB2CLKDivider = DIV1), so
 * TIM1CLK = HCLK = SystemCoreClock and ARR = SystemCoreClock/freq - 1, the
 * same relation the DRV8871/TIM3 driver relies on. Both motors share TIM1, so
 * this frequency (the ARR) is timer-wide -- both instances use it. */
#define DRV8306_PWM_FREQ_HZ            20000U

/* ---- Motor nameplate (JK60BLS03-XX, 300 W) ------------------------------ *
 * From datasheet/JK60BLS03-XX-300W.pdf (JKONGMOTOR, rev 2026.5.6). These are
 * the physical motor parameters the netlist does not carry. Kept as named
 * constants so the RPM calibration below is self-documenting and so any future
 * closed-loop / current-limit work has the real numbers to hand.
 *
 *   Poles                  8            -> 4 pole pairs
 *   Rated voltage          24 VDC       (= PWM bus voltage VM)
 *   No-load speed          4000 RPM     (at 24 V, i.e. 100 % duty, unloaded)
 *   Rated speed            3000 RPM     (at rated torque)
 *   Rated torque           0.95 N-m
 *   No-load current        2.4 A max
 *   Rated current          ~15.8 A      (= rated torque / Kt)
 *   Phase-phase resistance 0.13 ohm
 *   Back-EMF constant Ke   6  V/kRPM    (line-to-line)
 *   Torque constant   Kt   0.06 N-m/A
 *   Hall sensors           3 (HU/HV/HW), power +5..+20 VDC
 *   Rotation               CW viewed from the output shaft (DIR = CW default) */
#define DRV8306_MOTOR_VBUS_MV         24000U  /* rated / bus voltage   [mV]   */
#define DRV8306_MOTOR_KE_MV_PER_KRPM  6000U   /* back-EMF constant     [mV/kRPM] */
#define DRV8306_MOTOR_NOLOAD_RPM      4000U   /* no-load speed at 24 V        */
#define DRV8306_MOTOR_RATED_RPM       3000U   /* speed at rated 0.95 N-m load */

/* ---- Speed / RPM calibration -------------------------------------------- *
 * Open-loop map from commanded RPM to PWM duty. The 100 %-duty anchor is the
 * datasheet no-load speed (4000 RPM at 24 V); this is cross-checked by the
 * back-EMF constant, since Ke * no-load_RPM = 6 V/kRPM * 4000 RPM = 24 V = VM.
 * That makes the (unloaded) relation essentially linear through the origin:
 *
 *     duty[%] ~= 100 * Ke_V * RPM / VM = RPM / 40      (e.g. 3000 RPM -> ~75 %)
 *
 * MIN_DUTY_PCT rounds 500 RPM (500/40 = 12.5 %) up to 13 % for start margin.
 * NOTE: this is the NO-LOAD relation. Under mechanical load the actual speed
 * sags below the commanded value (rated 3000 RPM needs full 24 V at 0.95 N-m),
 * so treat the RPM path as approximate and trim with FGOUT (DRV8306_MeasuredRPM)
 * where accuracy matters. The raw duty path (DRV8306_SetDuty) is always exact. */
#define DRV8306_MIN_RPM               500U    /* lowest usable / start speed  */
#define DRV8306_MAX_RPM               4000U   /* no-load speed at 100 % duty  */
#define DRV8306_MIN_DUTY_PCT          13U     /* duty that maps to MIN_RPM    */

/* FGOUT -> mechanical RPM conversion.
 *   mech_RPM = fg_falling_edges/s * 60 / (FG_EDGES_PER_ELEC_REV * pole_pairs)
 * FGOUT carries all three Hall inputs (datasheet 7.3.5, Figure 20/21): the
 * 3 Hall signals give 6 commutation edges per electrical revolution, i.e.
 * 3 FALLING edges per electrical revolution -- consistent with this motor's
 * 3-Hall (HU/HV/HW) arrangement. pole_pairs is per-instance (struct field);
 * default 4 = 8 poles / 2, confirmed by the nameplate above. */
#define DRV8306_FG_EDGES_PER_ELEC_REV 3U      /* falling edges per elec. rev  */
#define DRV8306_DEFAULT_POLE_PAIRS    4U      /* 8 poles / 2 (datasheet)      */

/* ---- Per-instance mechanics (gearbox + no-load speed) ------------------- *
 * The two motors are NOT the same part, so these live per-instance (handle
 * fields) instead of as shared #defines:
 *   M1 (U11, grinder) : direct drive, JK60BLS03 nameplate (4000 RPM no-load).
 *   M2 (U16, stirrer) : JK42BLS02-39JXE49 (datasheet/JK42BLS02-...-41.8W.pdf),
 *                       5000 RPM no-load motor behind a 1:49 planetary gearbox
 *                       (rated 4.4 N-m / 65 RPM at the OUTPUT shaft).
 * gear_ratio lets the closed-loop stirrer control (stir_ctrl.c) command the
 * OUTPUT shaft in RPM while FGOUT still measures the motor shaft; noload_rpm
 * anchors the open-loop feed-forward (DRV8306_FeedForwardPerMille). */
#define DRV8306_M1_GEAR_RATIO         1U      /* direct drive (no gearbox)    */
#define DRV8306_M2_GEAR_RATIO         49U     /* 1:49 planetary reduction     */
#define DRV8306_M2_NOLOAD_RPM         5000U   /* JK42BLS02 no-load at 24 V    */

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
	uint16_t           gear_ratio;  /* output-shaft reduction (1 = direct)   */
	uint16_t           noload_rpm;  /* motor no-load RPM at 100 % duty / VM  */

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

/* ---- Closed-loop speed control (FGOUT PI) ------------------------------- *
 * A small fixed-point PI regulator that holds a commanded OUTPUT-shaft RPM
 * using the FGOUT tacho as feedback. It is motor-agnostic: M2 (geared) uses it
 * via stir_ctrl.c today; M1 can reuse it unchanged with gear_ratio = 1 (then
 * "output RPM" == motor RPM). See stir_ctrl.c for the M1 note.
 *
 * Duty is handled in PER-MILLE (0..1000) for resolution: TIM1 ARR is 3199
 * (64 MHz / 20 kHz), ~3.2 ticks per per-mille -- far finer than the old 1 %
 * path and smooth enough for a PI. Gains are integer fractions (num/den), so
 * no float is used:
 *     p_pm    = kp_num * err / kp_den                       [per-mille]
 *     integ  += ki_num * err * dt_ms / ki_den               [per-mille]
 *     duty_pm = ff_pm + p_pm + integ   (clamped to [min,max])
 * Anti-windup is conditional integration: when the output is on a rail the
 * integrator only moves in the unwinding direction. err is in OUTPUT RPM. */
typedef struct
{
	int32_t  kp_num, kp_den;     /* proportional gain fraction              */
	int32_t  ki_num, ki_den;     /* integral gain fraction (per ms)         */
	int32_t  integ_pm;           /* integral accumulator [per-mille]        */
	int16_t  out_min_pm;         /* duty clamp low  [per-mille]             */
	int16_t  out_max_pm;         /* duty clamp high [per-mille]             */
	int16_t  duty_pm;            /* last computed duty (read-back)          */
} DRV8306_PI_t;

/* Reset the PI to the given gains/clamp and zero the integrator. */
void     DRV8306_PI_Init(DRV8306_PI_t *pi,
                         int32_t kp_num, int32_t kp_den,
                         int32_t ki_num, int32_t ki_den,
                         int16_t out_min_pm, int16_t out_max_pm);

/* One PI step. err = target_out_rpm - meas_out_rpm; ff_pm is the open-loop
 * feed-forward duty for the target. Returns the clamped duty [per-mille] and
 * also stores it in pi->duty_pm. */
int16_t  DRV8306_PI_Compute(DRV8306_PI_t *pi, int32_t err,
                            int16_t ff_pm, uint32_t dt_ms);

/* Set duty in per-mille (0..1000). Exact; the closed-loop path uses this. */
void     DRV8306_SetDutyPerMille(DRV8306_HandleTypeDef *h, uint16_t duty_pm);

/* Open-loop feed-forward: duty [per-mille] to spin the OUTPUT shaft at out_rpm,
 * from the no-load relation (motor_rpm = out_rpm*gear_ratio; duty = motor_rpm /
 * noload_rpm). Used as the PI baseline so the integrator only trims the error. */
int16_t  DRV8306_FeedForwardPerMille(const DRV8306_HandleTypeDef *h,
                                     uint16_t out_rpm);

/* Measured OUTPUT-shaft RPM = DRV8306_MeasuredRPM()/gear_ratio. Advances the
 * FGOUT window exactly like DRV8306_MeasuredRPM(), so call it once per window. */
uint16_t DRV8306_MeasuredOutputRPM(DRV8306_HandleTypeDef *h, uint32_t window_ms);

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
