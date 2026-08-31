/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tb_drv8871.h"
#include "tb_stepmotor.h"
#include "drv8306.h"
#include "bldc_ctrl.h"
#include "tb_tca9554.h"
#include "tb_lift.h"
#include "tb_gpioout.h"
#include "distance.h"
#include "tb_distance.h"
#include "hallsensor.h"
#include "tb_hallsensor.h"
#include "adc_ctrl.h"
#include "thermistor.h"
#include "tb_thermistor.h"
#include "tb_heat.h"
#include "tb_speaker.h"
#include "tb_water.h"
#include "tb_doorhall.h"
#include "moeum.h"
#include "dongjak.h"
#include "kangeum.h"
#include "baesu.h"
#include "jungji.h"
#include "mode_arbiter.h"
#include "protocol_r0.h"
#include "tb_protocol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* GP2Y0A41SK distance sensor (수거통 용량 감지) -- latest readings, kept as
 * globals so they can be inspected in the debugger / consumed by other tasks. */
volatile uint16_t g_distance_mm   = 0;   /* measured distance [mm]  */
volatile uint8_t  g_bin_fill_pct  = 0;   /* bin fill level  [0..100]*/
/* NTC thermistors x3 (HCET-103F3950). Temperature in 0.1C units, so e.g.
 * 253 -> 25.3 C. THERMISTOR_ERR_D10 (-32768) means the read failed. */
volatile int16_t  g_therm_c_d10[THERMISTOR_COUNT] = {0};
/* Hall sensors HS1..HS8 via U24 TCA9554A (I2C1, INT on PF9). bit i = HS(i+1)
 * magnet present. Refreshed by StartDefaultTask; inspect in the debugger or via
 * HallSensor_Get(). */
volatile uint8_t  g_hall_mask     = 0;   /* HS1..8 detected mask [bit0..7] */

/* Runtime app mode: which of the mutually exclusive halves the two RTOS tasks
 * run -- the debug TESTBENCH (keypad/debugger-driven individual motors) or one
 * of the scenarios (모음 / 동작 / 강음 / 배수). They cannot run at once because
 * they share g_stir_ctrl (M2), the grinder and the doors, so each task branches
 * on this flag.
 *
 * OWNERSHIP MOVED (R1): app_mode_t and g_app_mode now live in
 * Core/Scenario/mode_arbiter.c. The arbiter decodes the lid-position Hall
 * sensors HS1..HS5 every 100 ms REGARDLESS of the current mode and is the only
 * writer of g_app_mode, so turning the lid cap actually starts/stops a scenario
 * (previously the trigger was gated behind the very mode it was supposed to
 * select, so only a debugger write could switch modes). The mode-transition
 * cleanup that used to live in StartMotorTask below now runs inside
 * ModeArbiter_MotorTick() -> Jungji_StopAll(), which also fixes the ordering
 * bug where the transition Abort() wiped a freshly latched start_req. */

/* Motor control service thread. Owns all motor testbeds -- DRV8871 doors (U5/
 * U7), steppers STEP1/STEP2, DRV8306 BLDC (U11/U16) and the U6 lift -- so motor
 * timing stays independent of the 100 ms sensor loop in defaultTask. */
osThreadId_t Motor_TaskHandle;
const osThreadAttr_t Motor_Task_attributes = {
  .name = "Motor_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for i2c1_mutex */
osMutexId_t i2c1_mutexHandle;
const osMutexAttr_t i2c1_mutex_attributes = {
  .name = "i2c1_mutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartMotorTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of i2c1_mutex */
  i2c1_mutexHandle = osMutexNew(&i2c1_mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  Motor_TaskHandle = osThreadNew(StartMotorTask, NULL, &Motor_Task_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Analog sensors on ADC1, all read through the shared adc_ctrl layer:
   *   - GP2Y0A41SK distance sensor (PA0)      -> bin fill level
   *   - 3x NTC thermistors (PC0/PC1/PC2)      -> temperatures [0.1 C]
   * AdcCtrl_Init() calibrates ADC1 and puts it in single-conversion mode; it
   * must run before any Distance_/Thermistor_ read. The distance sensor
   * refreshes every 16.5 ms, so 100 ms polling is plenty. */
  AdcCtrl_Init();
  Distance_Init();
  TB_Distance_Init();     /* enable-gated Distance-SEN monitor (defaultTask owns ADC1) */
  Thermistor_Init();
  TB_Thermistor_Init();   /* enable-gated thermistor monitor (defaultTask owns ADC1) */
  /* Heater (HT-POWER PA12) testbed. Reads the smoothed thermistor value cached
   * by Thermistor_Tick() and closes the 동작 190/195C hysteresis loop, so the
   * NTC probe is in the loop. Starts disabled/off; only drives the pin while in
   * TESTBENCH mode so it never fights the 동작 scenario over HT-POWER. */
  TB_Heat_Init();
  /* Hall sensors HS1..8 on the U24 TCA9554A (I2C1). MX_I2C1_Init() already ran
   * in main() before the scheduler, so the bus is up. U24's INT is wired to PF9
   * (HALL-INT1): the loop services that edge via HallSensor_ServiceInt() and
   * also polls periodically as a safety net. */
  HallSensor_Init();
  TB_HallSensor_Init();   /* enable-gated U24 Hall-sensor monitor (defaultTask owns the HS reads) */
  /* U15 LM4871 speaker (SPK-EN PA3, SPK-DAC PA4). Enable-gated low-volume test
   * tone; no ADC1/I2C involvement so it is safe on this task. Idempotent init. */
  TB_Speaker_Init();      /* enable-gated speaker test tone (set tb_speaker_enable=1) */
  /* Clean-water fill path: WATER-ON supply (PE2) + WATER-SEN1/SEN2 level sensors
   * (PF6/PF7 EXTI). Enable-gated; plain GPIO + EXTI-flag reads, so it is safe on
   * this task, which already polls the EXTI flags. */
  TB_Water_Init();        /* enable-gated water supply + level-sensor monitor (set tb_water_enable=1) */
  /* Door-limit Hall monitor: WHALL/THALL open/close (PF3/PF4/PF2/PF5 EXTI) that
   * gate the WDoor/TDoor transitions in the 모음/동작 scenarios. Enable-gated,
   * input-only (reuses WDoor_/TDoor_At* decode), so it is safe on this task and
   * never fights a scenario. */
  TB_DoorHall_Init();     /* enable-gated door-limit Hall monitor (set tb_doorhall_enable=1) */
  /* R0 protocol self-test. Pure in-memory encode/decode round trip -- it never
   * touches UART5 or the motors, so it is safe to run with the app connected
   * and a scenario in progress. Set tb_proto_run_once=1 in the debugger for a
   * single pass and read tb_proto_fails (0 = OK). See tb_protocol.h. */
  TB_Protocol_Init();
  /* NOTE: the front-panel keypad (U8/U9 TCA9554A on I2C1) is NOT serviced from
   * this task. The generic membrane driver was removed on 2026-08-18 (the
   * product controls nothing from those buttons); the only U8/U9 owner left is
   * the tb_tca9554 testbed on StartMotorTask, kept for bench motor driving in
   * APP_MODE_TESTBENCH. */
  for(;;)
  {
    g_distance_mm  = Distance_ReadMm();
    g_bin_fill_pct = Distance_FillFromMm(g_distance_mm);

    /* One ADC read-set + EMA update for all 3 probes (single owner of the
     * thermistor filter), then publish the SMOOTHED value. Thermistor_Tick()
     * must be called from here only, once per 100 ms cycle. */
    Thermistor_Tick();
    for (uint8_t i = 0; i < THERMISTOR_COUNT; i++)
    {
      g_therm_c_d10[i] = Thermistor_GetCelsius_d10(i);
    }

    /* Enable-gated Distance-SEN monitor (testbed): set tb_dist_enable=1 in the
     * debugger to refresh tb_dist_raw/volt/cm/mm/fill, 0 to stop. Safe here
     * because this task is the sole ADC1 owner. */
    TB_Distance_Poll();

    /* Enable-gated thermistor monitor (testbed): set tb_therm_enable=1 in the
     * debugger to refresh tb_therm_raw/tb_therm_c/tb_therm_c_d10, 0 to stop.
     * Safe here because this task is the sole ADC1 owner. */
    TB_Thermistor_Poll();

    /* Enable-gated heater testbed (HT-POWER PA12): set tb_heat_enable=1 in the
     * debugger to close the hysteresis loop (or MANUAL-force the pin) using the
     * smoothed temperature published just above. Passing testbench-active gates
     * it so it only drives HT-POWER outside the 동작 scenario, which otherwise
     * owns the pin from StartMotorTask. */
    TB_Heat_Poll(AppMode_IsBenchIdle(g_app_mode));

    /* Refresh the HS1..8 snapshot (bit i = HS(i+1) magnet present). U24 pulls
     * HALL-INT1 (PF9) low on any change; service that edge first for low
     * latency, then do the periodic read as a safety net for missed edges. */
    (void)HallSensor_ServiceInt();
    if (HallSensor_Update() == HAL_OK)
    {
      g_hall_mask = HallSensor_GetMask();
    }

    /* Enable-gated U24 Hall-sensor monitor (testbed): set tb_hall_enable=1 in
     * the debugger to split the P0..P7 mask into the four named channels
     * (trigger group P0..P4, 교반원점 P5, 수거통 P6, 리프트하단 P7) with
     * edge counts, 0 to stop. Safe here: this task is the sole U24 reader. */
    TB_HallSensor_Poll();

    /* Enable-gated speaker testbed: set tb_speaker_enable=1 in the debugger to
     * play a repeating low-volume tone (SPK-EN active, SPK-DAC bit-banged), 0 to
     * mute. A burst blocks this task for tb_speaker_on_ms; harmless at 100 ms. */
    TB_Speaker_Poll();

    /* Enable-gated water testbed: set tb_water_enable=1 in the debugger to open
     * WATER-ON and refresh tb_water_sen1/2_level/_present/_events, 0 to shut the
     * supply off. Reads the shared EXTI flags polled/cleared on this task.
     * Gated like TB_Heat_Poll: WATER-ON (PE2) is also driven by the Moeum/Dongjak
     * scenarios from StartMotorTask, so the bench only touches it when idle. */
    TB_Water_Poll(AppMode_IsBenchIdle(g_app_mode));

    /* Enable-gated door-limit Hall monitor (testbed): set tb_doorhall_enable=1 in
     * the debugger to refresh tb_wdoor_at_open/close, tb_tdoor_at_open/close and
     * the raw pin levels, 0 to stop. Input-only. Falling edges are NOT counted
     * here - TB_DoorHall_OnEXTI() does that from the EXTI ISR (main.c router),
     * so this poll only owns the level/at-limit snapshot. */
    TB_DoorHall_Poll();

    /* Mode arbiter. Runs on EVERY cycle, in every mode: it decodes the lid
     * position from the HS1..HS5 snapshot refreshed just above and decides the
     * mode (HS5 모음 / HS2 동작 / HS1 강음 / HS4 배수 / HS3 정지=APP_MODE_JUNGJI)
     * or requests a stop (HS3, or the lid leaving every position). It only latches
     * requests here; the switch itself is executed by ModeArbiter_MotorTick() on the
     * motor task. Must come AFTER HallSensor_Update() so it sees this cycle's
     * mask. */
    ModeArbiter_SenseTick();

    /* Scenario sensing half. Sensor-only (water level, temperature, bin fill,
     * debug force-start), so it belongs on this 100 ms cadence and never
     * touches a motor. The start trigger itself is no longer read here -- the
     * arbiter above owns it. */
    switch (g_app_mode)
    {
      case APP_MODE_MOEUM:   Moeum_SenseTick();   break;
      case APP_MODE_DONGJAK: Dongjak_SenseTick(); break;
      case APP_MODE_KANGEUM: Kangeum_SenseTick(); break;   /* 미구현 스텁 */
      case APP_MODE_BAESU:   Baesu_SenseTick();   break;   /* 미구현 스텁 */
      case APP_MODE_JUNGJI:                                /* 정지(HS3): 대기와 동일 */
      case APP_MODE_TESTBENCH:
      default:                                    break;
    }

    /* App-facing R0 protocol service (UART5 via the BLE module in BYPASS).
     * Drains the uart_ctrl RX ring, answers R/W frames, and pushes the M
     * (monitoring) packets for 모음/동작. Must come AFTER the sensor reads and
     * the scenario SenseTick above so a packet carries THIS cycle's snapshot.
     * Sensor-only + queued TX (never blocks on the UART), so it belongs on this
     * task; scenario-control writes must latch a request for MotorTask instead
     * of driving anything here. */
    Proto_Tick(HAL_GetTick());

    /* Enable-gated R0 protocol self-test: set tb_proto_run_once=1 (single pass)
     * or tb_proto_enable=1 (repeating) in the debugger, then read tb_proto_fails
     * / tb_proto_first_fail. ~1-2 ms per pass; uses only its own buffers, so it
     * cannot disturb Proto_Tick above. */
    TB_Protocol_Poll();

    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief  Function implementing the Motor_Task thread. Owns every motor
  *         testbed. Set the matching enable flag in the debugger to run a
  *         motor, clear it to stop:
  *           tb_wdoor_enable / tb_tdoor_enable -> DRV8871 doors (U5 / U7);
  *                                stop themselves at the Hall limit for the
  *                                direction driven (tb_*_limit_stop=1 default,
  *                                result in tb_*_limit_hit)
  *           tb_step1_enable / tb_step2_enable -> steppers STEP1 / STEP2; both
  *                                are TIMED one-shots that self-clear the flag:
  *                                STEP1 runs 15 s, STEP2 runs 2 s for dir=1
  *                                (닫힘) and 1 s for dir=0 (열림)
  *           g_grind_ctrl (M1/U11) & g_stir_ctrl (M2/U16) -> DRV8306 BLDC, both
  *                                                CLOSED-LOOP: drive from the
  *                                                keypad (odd SW=M1, even SW=M2)
  *                                                or set g_*_ctrl.target_out_rpm;
  *                                                status in .meas_out_rpm/.state
  *           tb_lift_enable                    -> U6 lift (dir: tb_lift_reverse)
  *         Poll all at 1 ms so the stepper pacing (tb_stepN_period_ms) is
  *         accurate; the DRV8306 testbed self-times its FGOUT window off
  *         HAL_GetTick().
  * @param  argument: Not used
  * @retval None
  */
/* TESTBENCH half (original behaviour): debugger/keypad-driven individual motors.
 * TB_TCA9554_Poll() reads the keypad and translates presses into BldcCtrl
 * commands BEFORE the control ticks apply them, so a press acts on the same
 * cycle. */
static void MotorTask_RunTestbench(uint32_t now_ms)
{
  TB_DRV8871_Poll();
  TB_StepMotor_Poll();
  /* Keypad -> BldcCtrl_Start(). Suppressed while the emergency short brake is
   * held: BldcCtrl_Start() releases nBRAKE and wakes the driver, so a keypress
   * landing inside the JUNGJI_BRAKE_MS window would spin the grinder back up
   * right after a 정지. */
  if (!Jungji_IsBraking())
  {
    TB_TCA9554_Poll();
  }
  BldcCtrl_Tick(&g_grind_ctrl, now_ms);   /* M1 closed-loop PI (100 ms)        */
  BldcCtrl_Tick(&g_stir_ctrl,  now_ms);   /* M2 closed-loop PI (100 ms)        */
  TB_Lift_Poll();
  TB_GpioOut_Poll();                      /* drain valve + 3 fans (on/off)     */
}

/* SCENARIO half - "모음" rinse. Moeum_MotorTick() owns the state machine and
 * commands the WDoor motor + valve + stir BLDC; BldcCtrl_Tick(&g_stir_ctrl)
 * still has to run every cycle to execute the stir closed-loop PI that moeum
 * only issues Start/Stop/target to. The grinder (M1) is unused here and stays
 * parked. No TB_*_Poll runs, so nothing else touches the shared motors. */
static void MotorTask_RunMoeum(uint32_t now_ms)
{
  Moeum_MotorTick(now_ms);
  BldcCtrl_Tick(&g_stir_ctrl, now_ms);
}

/* SCENARIO half - "동작"(건조/분쇄/배출). Dongjak_MotorTick() owns the state
 * machine and commands heater/doors/steppers/fans + both BLDC via Start/Stop/
 * target; BldcCtrl_Tick(M1 grinder, M2 stirrer) must run every cycle to execute
 * their closed-loop PI. No TB_*_Poll runs, so nothing else touches the motors. */
static void MotorTask_RunDongjak(uint32_t now_ms)
{
  Dongjak_MotorTick(now_ms);
  BldcCtrl_Tick(&g_grind_ctrl, now_ms);
  BldcCtrl_Tick(&g_stir_ctrl,  now_ms);
}

/* SCENARIO half - "강음"(HS1) / "배수"(HS4). Both are UNIMPLEMENTED stubs: the
 * state machines only record that the mode was selected and drive nothing, so
 * no BldcCtrl_Tick is needed yet. Add it when the sequences are filled in. */
static void MotorTask_RunKangeum(uint32_t now_ms)
{
  Kangeum_MotorTick(now_ms);
}

static void MotorTask_RunBaesu(uint32_t now_ms)
{
  Baesu_MotorTick(now_ms);
}

void StartMotorTask(void *argument)
{
  TB_DRV8871_Init();               /* also WDoor_Init/TDoor_Init (doors)       */
  TB_StepMotor_Init();
  DRV8306_InitAll();               /* bring up drv8306_m1 + drv8306_m2         */
  BldcCtrl_Init(&g_grind_ctrl);    /* M1 (U11) grinder closed loop            */
  BldcCtrl_Init(&g_stir_ctrl);     /* M2 (U16) stirrer closed loop            */
  TB_TCA9554_Init();               /* keypad(U9); after the controllers above */
  TB_Lift_Init();
  TB_GpioOut_Init();               /* drain valve + 3 fans, all forced off     */
  Moeum_Init();                    /* scenario armed but idle until selected   */
  Dongjak_Init();                  /* scenario armed but idle until selected   */
  Kangeum_Init();                  /* HS1 강음 - unimplemented stub            */
  Baesu_Init();                    /* HS4 배수 - unimplemented stub            */
  Jungji_Init();                   /* common stop handler (must precede arbiter)*/
  ModeArbiter_Init();              /* owns g_app_mode; starts in TESTBENCH      */

  for(;;)
  {
    uint32_t now = HAL_GetTick();

    /* Common stop handler. Consumes any pending stop request (HS3 정지, lid
     * lost, mode switch, scenario fault, debugger), releases the BLDC short
     * brake after JUNGJI_BRAKE_MS and holds the cooling fans while the pot is
     * still hot. Runs first so the arbiter below sees the settled brake state. */
    Jungji_Tick(now);

    /* Mode arbiter, motor half: the only place g_app_mode is written. Applies a
     * pending switch as stop -> switch -> start, waiting for the brake hold to
     * expire before issuing the start so a braked BLDC is never commanded to
     * spin. */
    ModeArbiter_MotorTick(now);

    switch (g_app_mode)
    {
      case APP_MODE_MOEUM:   MotorTask_RunMoeum(now);   break;
      case APP_MODE_DONGJAK: MotorTask_RunDongjak(now); break;
      case APP_MODE_KANGEUM: MotorTask_RunKangeum(now); break;
      case APP_MODE_BAESU:   MotorTask_RunBaesu(now);   break;
      /* 정지(HS3)는 대기와 같은 벤치 분기로 간다. HS3 이후 대기로 돌아가던
       * 종전 동작과 동일하게 두기 위함이다 - JUNGJI 는 '보이는 상태'일 뿐
       * 실행 성격을 바꾸지 않는다(mode_arbiter.h app_mode_t 주석 참조). */
      case APP_MODE_JUNGJI:
      case APP_MODE_TESTBENCH:
      default:               MotorTask_RunTestbench(now); break;
    }
    osDelay(1);
  }
}

/* USER CODE END Application */

