#ifndef SRC_TB_HEAT_H_
#define SRC_TB_HEAT_H_

#include "main.h"
#include "thermistor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * tb_heat - testbed for the 220V heater on HT-POWER (PA12 = o_HT_POWER,
 * gpio_ctrl GPIO_OUT_HT_POWER). The pin drives a MOC3063 zero-cross opto-
 * triac (Q31/Q32) into the AC heater; a HW bimetal (60/80C EXTI + 210C) is
 * the real over-temp safety, the FW only supervises/switches.
 *
 * WHY IT NEEDS THE THERMISTOR: a heater cannot be bench-tested blind. This
 * testbed reads one NTC probe (thermistor.c, default CH0 = DJ_TEMP_CH, the
 * 처리통 probe the 동작 scenario uses) and closes the same 190/195C
 * hysteresis loop the 동작 scenario runs, so the SENSOR IS IN THE LOOP -- you
 * watch temperature track the heater. The thermistor's EMA-smoothed value is
 * reused via Thermistor_GetCelsius_d10(), NOT a fresh ADC read, so this testbed
 * does no ADC of its own.
 *
 * SHARED DECISION: the HYST-mode on/off/over-temp choice is made by the pure
 * Heater_HystDecide()/Heater_IsOverTemp() in interface/heater_hyst.h, the SAME
 * functions dongjak.c dj_heater_tick() uses -- so the testbed and the scenario
 * cannot drift apart. MANUAL mode and the idle-guard watchdog below are
 * TESTBENCH-ONLY extras; the 동작 scenario has neither.
 *
 * WHERE TO POLL - ADC1 THREAD-SAFETY + ACTUATOR OWNERSHIP:
 *   1) The temperature it consumes is produced by Thermistor_Tick() on
 *      StartDefaultTask (the sole ADC1 owner). So TB_Heat_Poll() must run on
 *      that same task, AFTER Thermistor_Tick(), never on StartMotorTask.
 *   2) HT-POWER is a shared actuator: in a SCENARIO (동작) the MotorTask owns
 *      it via dj_heater_tick(). To avoid two tasks fighting the pin, pass
 *      `testbench_active` = (g_app_mode == APP_MODE_TESTBENCH) into Poll(); it
 *      only drives the pin when active and otherwise keeps its hands off so
 *      the scenario is the sole writer. It also self-disables (forces OFF)
 *      whenever tb_heat_enable == 0.
 *
 * Call model (StartDefaultTask, freertos.c, ~100 ms -- plenty for a thermal
 * load):
 *     AdcCtrl_Init(); Thermistor_Init(); TB_Heat_Init();
 *     for(;;){ ...; Thermistor_Tick(); ...;
 *              TB_Heat_Poll(g_app_mode == APP_MODE_TESTBENCH); osDelay(100); }
 *
 * ------------------------------------------------------------------------
 * SAFETY (this switches mains-voltage heat -- treat every default as armed):
 *   - Starts DISABLED and OFF. Nothing happens until you set tb_heat_enable=1
 *     from the debugger while the app is in TESTBENCH mode.
 *   - Over-temp cut at tb_heat_safety_d10 (210.0C, matches DJ_TEMP_SAFETY):
 *     LATCHED fault, heater forced OFF until cleared. Applies in BOTH modes --
 *     MANUAL cannot override a real over-temp (when a valid probe reads it).
 *   - Optional idle-guard watchdog (tb_heat_max_on_ms): if the heater has been
 *     ON continuously that long, LATCH -> OFF. DISABLED by default (0), because
 *     a real run outlasts any short cap; set e.g. 600000 (10 min) if you want a
 *     "forgot to turn it off" backstop. TESTBENCH-ONLY.
 *   - HYST mode needs a valid reading: probe open/short (ERR) blocks the loop
 *     (forced OFF, TB_HEAT_FAULT_SENSOR set, auto-recovers). MANUAL mode is NOT
 *     sensor-gated, so an electrical pin test works with no probe attached.
 *   - Clear a latched fault by toggling tb_heat_enable 0->1, or set
 *     tb_heat_clear_fault=1 (auto-clears back to 0).
 * ---------------------------------------------------------------------- */

/* ---- Modes -------------------------------------------------------------- */
#define TB_HEAT_MODE_MANUAL   0u   /* drive HT-POWER = tb_heat_manual_on      */
#define TB_HEAT_MODE_HYST     1u   /* closed-loop hysteresis on the NTC probe */

/* ---- Fault bits (tb_heat_fault) ---------------------------------------- */
#define TB_HEAT_FAULT_NONE     0x00u
#define TB_HEAT_FAULT_SENSOR   0x01u  /* probe ERR (live, auto-recovers)      */
#define TB_HEAT_FAULT_OVERTEMP 0x02u  /* >= safety threshold  (LATCHED)       */
#define TB_HEAT_FAULT_WATCHDOG 0x04u  /* max ON-time exceeded (LATCHED)       */

/* ---- Live inputs (set from the debugger) ------------------------------- */
extern volatile uint8_t  tb_heat_enable;      /* 0 = OFF/idle, 1 = run testbed */
extern volatile uint8_t  tb_heat_mode;        /* TB_HEAT_MODE_MANUAL / _HYST   */
extern volatile uint8_t  tb_heat_ch;          /* NTC probe idx 0..2 (def 0)    */
extern volatile uint8_t  tb_heat_manual_on;   /* MANUAL mode: 1 = pin HIGH     */
extern volatile uint8_t  tb_heat_clear_fault; /* write 1 to clear latch (self-clears) */

/* Hysteresis band + safety, in 0.1C units.
 * ⚠️ STALE vs the scenario: these defaults (1900/1950) mirror the OLD 190/195C
 * spec. The 동작 scenario was re-specified on 2026-08-15 to DJ_TEMP_HEATER_ON/
 * OFF_D10 = 1140/1170 (114C ON / 117C OFF); only SAFETY (2100) still matches.
 * Set tb_heat_on_d10/off_d10 = 1140/1170 in the debugger to test the band the
 * product actually uses. To exercise the loop at bench temperature, lower them
 * to e.g. 300/350 (30/35C) and warm the probe by hand/hot-air; the safety stays
 * a hard backstop. */
extern volatile int16_t  tb_heat_on_d10;      /* <= this -> heater ON  (1900)  */
extern volatile int16_t  tb_heat_off_d10;     /* >= this -> heater OFF (1950)  */
extern volatile int16_t  tb_heat_safety_d10;  /* >= this -> LATCH OFF  (2100)  */
extern volatile uint32_t tb_heat_max_on_ms;   /* idle-guard watchdog [ms]; 0 = OFF (default) */

/* ---- Live status (watch in the debugger) ------------------------------- */
extern volatile uint8_t  tb_heat_out;         /* level commanded onto HT-POWER 0/1 */
extern volatile int16_t  tb_heat_temp_d10;    /* probe temp used this cycle (ERR=-32768) */
extern volatile uint8_t  tb_heat_fault;       /* TB_HEAT_FAULT_* bitmask       */
extern volatile uint32_t tb_heat_on_ms;       /* current continuous ON-time [ms] */
extern volatile uint32_t tb_heat_samples;     /* liveness: ++ each active poll  */

/* ---- Timed logging session --------------------------------------------- *
 * When tb_heat_enable rises 0->1 a bounded 5-minute session starts: it runs
 * the heater as usual AND snapshots the state into tb_heat_log[] every 30 s so
 * the thermal response can be read back at runtime (debugger array watch). The
 * session ENDS on either edge:
 *   - 5 minutes elapse  -> auto-terminate: tb_heat_enable is cleared to 0 and
 *                          the heater is forced OFF.
 *   - tb_heat_enable <-0 -> operator stop: session closes, heater OFF.
 * Samples land at t = 0, 30, 60, ... 300 s (11 slots). Re-arming (enable
 * 0->1 again) clears the log and starts a fresh session. */
#define TB_HEAT_LOG_PERIOD_MS   30000u   /* one snapshot every 30 s            */
#define TB_HEAT_SESSION_MS      300000u  /* total session length = 5 min       */
#define TB_HEAT_LOG_COUNT       11u      /* slots for t=0,30,...,300 s          */

typedef struct
{
	uint32_t t_ms;      /* elapsed since session start at capture [ms]        */
	int16_t  temp_d10;  /* probe temp 0.1C (THERMISTOR_ERR_D10 on fault)      */
	uint8_t  out;       /* heater level driven onto HT-POWER at capture 0/1   */
	uint8_t  fault;     /* TB_HEAT_FAULT_* bitmask at capture                 */
} TbHeatLog_t;

/* Ring-free flat log, filled front-to-back. Read tb_heat_log[0..count-1]. */
extern volatile TbHeatLog_t tb_heat_log[TB_HEAT_LOG_COUNT];
extern volatile uint8_t     tb_heat_log_count;      /* valid entries 0..COUNT  */
extern volatile uint8_t     tb_heat_session_active; /* 1 = 5-min session live  */
extern volatile uint32_t    tb_heat_session_ms;     /* elapsed session time[ms]*/

/* ---- Lifecycle ---------------------------------------------------------- */
/* Force HT-POWER OFF, clear flags/faults, load safe defaults. Call once after
 * MX_GPIO_Init() / Thermistor_Init(). */
void TB_Heat_Init(void);

/* One service cycle. Pass testbench_active = (g_app_mode==APP_MODE_TESTBENCH):
 *   active   -> run the enabled/mode/safety logic and drive HT-POWER.
 *   inactive -> keep hands off the pin (scenario owns it), reset internal
 *               timers so a later activation starts clean.
 * Call from StartDefaultTask after Thermistor_Tick(). */
void TB_Heat_Poll(uint8_t testbench_active);

#ifdef __cplusplus
}
#endif

#endif /* SRC_TB_HEAT_H_ */
