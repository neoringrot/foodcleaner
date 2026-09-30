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
 *   g_stir_ctrl  : M2 / U16, JK42BLS02 stirrer: 1:82 gearbox -> x1.2 gear ->
 *                  **blade**. Ladder / target / measured are in **BLADE RPM**
 *                  (★2026-09-21 §0.22 — 전에는 1:49 가정의 "출력 rpm" 이었고
 *                  지령 30 이 실제 날개 20 이었다). Conversion is in DRV8306.
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
 * status. For M1 (gear 1) *_out_rpm are motor RPM; for M2 they are BLADE RPM. */
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

	/* ★R3 개정4 C088~C090 [2026-09-21] — 회전수 운전(rotation.c) 입력용 관측.
	 * BldcCtrl_Tick() 이 **매 호출**(주기 게이트 앞) 갱신한다. 설명은 아래 API 주석. */
	volatile uint32_t position;       /* FG엣지 × DIR핀 부호 누적, uint32 모듈러 (RO) */
	volatile uint16_t rest_ms;        /* at_rest 판정: FG 무펄스 연속 시간 (기본 BLDC_REST_MS) */
	uint32_t pos_fg_last;             /* position 누적용 FG 직전값           */
	uint32_t fg_move_ms;              /* FG 가 마지막으로 바뀐 tick          */
	uint8_t  dir_ccw;                 /* 지금 DIR 핀에 걸린 방향(UNJAM 포함) */
} BldcCtrl_t;

/* The two board motors as closed-loop controllers (defined in bldc_ctrl.c). */
extern BldcCtrl_t g_grind_ctrl;       /* M1 / U11 grinder (direct drive)     */
extern BldcCtrl_t g_stir_ctrl;        /* M2 / U16 stirrer (blade RPM, §0.22) */

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

/* ---- Emergency stop (jungji.c) ------------------------------------------
 * BldcCtrl_Stop() is a COAST stop: DRV8306_Stop() drops duty to 0 and puts the
 * gate driver to sleep, so a loaded rotor (grinder blade, stirrer paddle) free-
 * wheels down over seconds. That is fine for a normal end-of-step stop, but not
 * for the 4.5 emergency path (lid opened -> a hand can reach the blade).
 *
 * BldcCtrl_BrakeStop() short-brakes instead: nBRAKE LOW shorts all three phases
 * through the low-side FETs, which dumps the rotor's kinetic energy as heat in
 * the windings and stops it in a fraction of the coast time. The brake only
 * works while the gate driver is AWAKE, so ENABLE is deliberately kept HIGH and
 * the controller is merely marked idle (running=0) so BldcCtrl_Tick() cannot
 * re-drive it. The caller must therefore call BldcCtrl_BrakeRelease() after a
 * hold time (JUNGJI_BRAKE_MS) to release nBRAKE and drop the driver to sleep --
 * leaving the phases shorted forever would keep the FETs conducting.
 * Pairing is enforced by Jungji_Tick(); do not call these directly elsewhere. */
void    BldcCtrl_BrakeStop(BldcCtrl_t *c);    /* short brake, driver stays awake */
void    BldcCtrl_BrakeRelease(BldcCtrl_t *c); /* release brake -> normal Stop()  */

/* 1 = motor currently spinning (RUN or UNJAM); mirrors c->running. */
uint8_t BldcCtrl_IsRunning(const BldcCtrl_t *c);

/* ---- ★R3 개정4 회전수 운전 관측 (C088~C090, 검토서 §17.5·§17.6) -----------
 * rotation.c 의 zg_rotation_input 중 stir_position_ticks · stir_at_rest ·
 * grind_at_rest 의 **원천**이다. 벤더 port_contract rotation_contract 를 따른다.
 *
 * BldcCtrl_Position() — FG 하강엣지를 **지금 DIR 핀에 걸린 방향** 부호로 누적한다.
 *   uint32 모듈러(계약: "uint32_t modulo counter"). UNJAM 역회전도 실제 방향으로 센다.
 *   정지 명령 뒤 관성 회전은 마지막으로 건 방향으로 센다.
 *   ⚠ **부호는 DIR 핀 기준이다**: DRV8306_DIR_CW 일 때 +. 이것이 계약의
 *     "위에서 볼 때 시계방향 = +" 와 같은지는 **N14 미확인**이다 — 그 변환은
 *     호출부(rotation 어댑터) 한 곳에서 한다. 여기서 부호를 뒤집지 말 것.
 *   M2 는 날개 1회전 = FG 822 (P54, run G 실측, 구현현황 §0.22).
 *
 * BldcCtrl_IsAtRest() — 계약: "STOP 명령이나 고정 true 가 아니라 **실제 정지 관측**".
 *   running == 0 이고 FG 가 rest_ms 동안 한 번도 안 바뀌었으면 1.
 *   rest_ms 300 에서 FG 공백 = 모터 ≈17rpm 이하(날개 ≈0.25rpm) — 사실상 정지.
 *   ⚠ **드라이버가 깨어 있을 때만 관측이다** [2026-09-21 run F 에서 발견, 구현현황 §0.25].
 *     FGOUT 풀업(M1 R43 / M2 R62 10k)이 DRV8306 내부 LDO **DVDD** 에 물려 있어, 슬립
 *     (BldcCtrl_Stop 의 ENABLE LOW)에서는 DVDD 가 꺼져 **관성 회전 중에도 FG 가 안 나온다**.
 *     그 상태의 IsAtRest 는 "정지 명령 후 rest_ms 지남" 과 같다(계약 위반). position 도
 *     관성분을 놓친다. → 회전수 운전의 정지는 **BldcCtrl_CoastAwake()** 로 할 것.
 *
 * BldcCtrl_Tick() 이 불리지 않는 모드(강음·배수 스텁 등)에서는 갱신되지 않는다.
 * 재개 첫 호출에서 FG 가 바뀌어 있으면 "방금 움직임" 으로 보므로 at_rest 는 보수적
 * (늦게 1)이다. */
#ifndef BLDC_REST_MS
#define BLDC_REST_MS   300U   /* TBD: N15 — tb_rotation_rest_wait_ms 실측 후 확정 */
#endif
uint32_t BldcCtrl_Position(const BldcCtrl_t *c);
uint8_t  BldcCtrl_IsAtRest(const BldcCtrl_t *c, uint32_t now_ms);

/* ★R3 개정4 C089 [2026-09-21, §0.25] — 회전수 운전 **운전 사이 정지 전용**.
 * duty 0 으로 구동만 끊고 **드라이버는 깨운 채**(ENABLE HIGH, nBRAKE HIGH) 둔다 → DVDD 가
 * 살아 FG 가 관성 회전을 계속 보고하므로 IsAtRest()·Position() 이 **실제 관측**이 된다.
 * 상태는 Stop() 과 같다(running 0, IDLE, 소프트락 해제, PI 리셋) — 다음 Start() 로 재기동.
 * 1x PWM 모드에서 duty 0 이 순수 관성인지 동기정류 제동에 가까운지는 DRV8306 MODE 스트랩에
 * 따른다(어느 쪽이든 정지로 간다). **운전을 끝낼 때는 반드시 Stop() 으로 재워라** —
 * 이 함수는 드라이버를 깨운 채 남긴다. 정지(jungji)·비상정지는 이 함수를 쓰지 않는다. */
void     BldcCtrl_CoastAwake(BldcCtrl_t *c);

/* ---- 폴트 래치 해제 -------------------------------------------------------
 * nFAULT 는 DRV8306 쪽에서 래치된다(h->fault, EXTI 하강에지). BldcCtrl_Stop() 은
 * 소프트락(LOCKED)만 풀고 이 래치는 건드리지 않으므로, 한 번 폴트가 뜨면 다음
 * Start 까지 c->fault 가 1 로 남아 앱 화면에 계속 표시된다.
 * 이 함수는 DRV8306_ClearFault()(ENABLE 펄스로 래치 리셋)를 호출해 표시를 실제로
 * 지운다. 앱의 '에러 해제' / '정지' 명령이 이 경로를 쓴다.
 *
 * ⚠ 회전 중에는 아무것도 하지 않는다. ClearFault 는 ENABLE 을 내렸다 올려 드라이버를
 *   깨우는 동작이라 구동 중에 부르면 출력이 끊긴다. 폴트를 지우려면 먼저 정지해야
 *   한다(정지 경로에서 부르므로 실제로는 문제되지 않는다).
 * ⚠ MotorTask 에서만 호출할 것(ENABLE 핀을 만진다). 프로토콜 태스크는 jungji 에
 *   요청만 남긴다 - Jungji_RequestFaultClear() 참조. */
void    BldcCtrl_ClearFault(BldcCtrl_t *c);

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_BLDC_CTRL_H_ */
