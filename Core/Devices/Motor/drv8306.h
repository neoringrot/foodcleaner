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

/* ---- Per-instance mechanics (gearbox + final stage + no-load speed) ------ *
 * The two motors are NOT the same part, so these live per-instance (handle
 * fields) instead of as shared #defines:
 *   M1 (U11, grinder) : direct drive, JK60BLS03 nameplate (4000 RPM no-load).
 *   M2 (U16, stirrer) : JK42BLS02 8-pole, 5000 RPM no-load, **1:82 gearbox**
 *                       -> **x1.2 step-up gear** -> stirrer blade (HS6 guide).
 *
 * ★2026-09-21 (구현현황 §0.22, 기록지 I02_엣지실측_기록R3.md §11.7~§11.11) —
 *   사용자 지시: "gear_ratio 82로 바꾸고 날개 기준으로 맞춰줘".
 *   - 실측(run G): 날개(가이드) 1회전당 M2 FG 822.2개 = 모터 68.52회전
 *     (FG 12개/모터회전 = 3 x 4극쌍). 정/역·15~40rpm 에서 ±0.08%.
 *   - 업체: 모터 사양 출력 39 rpm, 마지막 기어 x1.2 증속. 68.52 x 1.2 = 82.2 =
 *     감속기, 82.2 x 39 = 3207 ~= JK42BLS02 정격 3200 rpm -> 서로 일치.
 *   - 그래서 R2 의 1:49 는 틀렸다. datasheet/ 의 `-39JXE49` 파일(1:49, 65 rpm)은
 *     **설치된 모터와 다른 감속기 사양**이다. 1:49 로 계산하던 "출력 rpm" 은
 *     감속기 출력을 1.67배, 날개를 1.43배 크게 보고 있었다(지령 30 -> 날개 20).
 *
 * "OUTPUT" = **날개(blade)** 다 (M2). 환산은 두 단을 모두 거친다:
 *     out_rpm = motor_rpm x stage_num / (gear_ratio x stage_den)
 *     M2 : motor / 82 x 6/5 = motor / 68.33   (실측 68.52 과 0.28% 차 - 허용)
 *   gear_ratio 는 uint16 이라 1.2 를 담을 수 없어 증속 단을 num/den 으로 분리했다.
 *   M1 은 1/1 (직결, 변화 없음).
 * noload_rpm anchors the open-loop feed-forward (DRV8306_FeedForwardPerMille). */
#define DRV8306_M1_GEAR_RATIO         1U      /* direct drive (no gearbox)    */
#define DRV8306_M1_STAGE_NUM          1U      /* no final stage               */
#define DRV8306_M1_STAGE_DEN          1U
/* ★2026-09-21 §0.22: 49 -> 82. 실측 68.52 x 업체 증속 1.2 = 82.2 (정격 3200/39 = 82.05). */
#define DRV8306_M2_GEAR_RATIO         82U     /* 1:82 planetary reduction     */
/* ★2026-09-21 §0.22 신설: 감속기 출력 -> 날개 x1.2 증속 (업체 회신). 6/5 = 1.2 */
#define DRV8306_M2_STAGE_NUM          6U      /* blade = gearbox_out x 6/5    */
#define DRV8306_M2_STAGE_DEN          5U
#define DRV8306_M2_NOLOAD_RPM         5000U   /* JK42BLS02 no-load at 24 V    */

/* ---- Feed-forward offset (static friction / no-load current) ------------- *
 * ★2026-09-21 (구현현황 §0.22.5) — 사용자 지시 "(가) 피드포워드 오프셋 추가해줘".
 * 피드포워드 duty = motor_rpm / noload_rpm 은 **0 rpm 에서 0‰** 인 직선이다. 실제 모터는
 * 마찰·무부하 전류 때문에 돌기 시작하는 데 일정 duty 가 더 든다. §0.22 벤치(무부하,
 * 기록지 §11.14)에서 지령 20/30/40 모두 날개가 **≈2 rpm 일정하게 모자랐다**(90/93/95%) —
 * 속도에 비례하지 않으므로 PI 비례오차가 아니라 이 상수 누락이다.
 * 값: 정상상태에서 PI 의 P항(kp 5 × 오차 ≈2)이 이미 ≈10‰ 를 보태고 있으므로,
 *     필요한 상수 = FF 기울기분(13.67‰/rpm × ≈2.1 ≈ 28) + P분(≈10) = **36~40‰**
 *     (지령 20: 36.0 / 30: 39.6 / 40: 40.1). 가운데 값 38.
 * 목표 0 rpm(정지·램프 시작점)에서는 0 을 그대로 준다 — 소프트스타트 유지.
 * M1(분쇄)은 측정 근거가 없어 0(변화 없음). */
#ifndef DRV8306_M1_FF_OFFSET_PM
#define DRV8306_M1_FF_OFFSET_PM       0U      /* grinder: unchanged           */
#endif
#ifndef DRV8306_M2_FF_OFFSET_PM
#define DRV8306_M2_FF_OFFSET_PM       38U     /* stirrer: §0.22.5 bench 36~40 */
#endif

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
	uint16_t           gear_ratio;  /* gearbox reduction (1 = direct)        */
	uint16_t           stage_num;   /* final stage: out = gearbox_out x num/den */
	uint16_t           stage_den;   /*   (M2 6/5 = x1.2 step-up to blade)     */
	uint16_t           noload_rpm;  /* motor no-load RPM at 100 % duty / VM  */
	uint16_t           ff_offset_pm;/* FF constant added when out_rpm > 0 [‰] */

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

/* Open-loop feed-forward: duty [per-mille] to spin the OUTPUT (M2: blade) at
 * out_rpm, from the no-load relation
 *   motor_rpm = out_rpm x gear_ratio x stage_den / stage_num ;
 *   duty = motor_rpm / noload_rpm + ff_offset_pm   (out_rpm == 0 -> 0)
 * Used as the PI baseline so the integrator only trims the error. */
int16_t  DRV8306_FeedForwardPerMille(const DRV8306_HandleTypeDef *h,
                                     uint16_t out_rpm);

/* Measured OUTPUT RPM (M2: blade) =
 *   DRV8306_MeasuredRPM() x stage_num / (gear_ratio x stage_den).
 * Advances the FGOUT window exactly like DRV8306_MeasuredRPM(), so call it once
 * per window. */
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
