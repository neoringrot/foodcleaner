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
#include "distance.h"
#include "membrane.h"
#include "hallsensor.h"
#include "adc_ctrl.h"
#include "thermistor.h"
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
/* Hall sensors HS1..HS7 via U10 TCA9554A (I2C1). bit i = HS(i+1) magnet
 * present. Refreshed by StartDefaultTask; inspect in the debugger or via
 * HallSensor_Get(). */
volatile uint8_t  g_hall_mask     = 0;   /* HS1..7 detected mask [bit0..6] */

/* Motor control service thread. Owns all motor testbeds -- DRV8871 doors (U5/
 * U7), steppers STEP1/STEP2, DRV8306 BLDC (U11/U16) and the U6 lift -- so motor
 * timing stays independent of the 100 ms sensor loop in defaultTask. */
osThreadId_t Motor_TaskHandle;
const osThreadAttr_t Motor_Task_attributes = {
  .name = "Motor_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
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
  Thermistor_Init();
  /* Hall sensors HS1..7 on the U10 TCA9554A (I2C1). MX_I2C1_Init() already ran
   * in main() before the scheduler, so the bus is up. No INT line is wired, so
   * the sensors are monitored by polling in this loop. */
  HallSensor_Init();
  /* Membrane keypad (U8/U9 TCA9554A on I2C1). Membrane_Init() already ran in
   * main() after MX_I2C1_Init, so this loop only polls for debounced edges.
   * NOTE: polling now shares the 100 ms sensor cadence instead of the former
   * ~MEMBRANE_POLL_MS thread; keypad response is coarser but code is simpler. */
  for(;;)
  {
    g_distance_mm  = Distance_ReadMm();
    g_bin_fill_pct = Distance_FillFromMm(g_distance_mm);

    for (uint8_t i = 0; i < THERMISTOR_COUNT; i++)
    {
      g_therm_c_d10[i] = Thermistor_ReadCelsius_d10(i);
    }

    /* Refresh the HS1..7 snapshot (bit i = HS(i+1) magnet present). */
    if (HallSensor_Update() == HAL_OK)
    {
      g_hall_mask = HallSensor_GetMask();
    }

    /* Membrane_Poll() is DISABLED while the tb_tca9554 keypad testbed owns
     * U8/U9 (it drives the LEDs and BLDC motors from the same expanders and
     * would fight this driver over the LED port / I2C bus). Re-enable this and
     * Membrane_Init() in main.c if you drop that testbed. */
    /* (void)Membrane_Poll(); */
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
  *           tb_wdoor_enable / tb_tdoor_enable -> DRV8871 doors (U5 / U7)
  *           tb_step1_enable / tb_step2_enable -> steppers STEP1 / STEP2
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
void StartMotorTask(void *argument)
{
  TB_DRV8871_Init();
  TB_StepMotor_Init();
  DRV8306_InitAll();               /* bring up drv8306_m1 + drv8306_m2         */
  BldcCtrl_Init(&g_grind_ctrl);    /* M1 (U11) grinder closed loop            */
  BldcCtrl_Init(&g_stir_ctrl);     /* M2 (U16) stirrer closed loop            */
  TB_TCA9554_Init();               /* keypad(U9); after the controllers above */
  TB_Lift_Init();
  for(;;)
  {
    TB_DRV8871_Poll();
    TB_StepMotor_Poll();
    /* Read the keypad and translate presses into BldcCtrl commands BEFORE the
     * control ticks apply them, so a press acts on the same cycle. */
    TB_TCA9554_Poll();
    BldcCtrl_Tick(&g_grind_ctrl, HAL_GetTick());  /* M1 closed-loop PI (100 ms) */
    BldcCtrl_Tick(&g_stir_ctrl,  HAL_GetTick());  /* M2 closed-loop PI (100 ms) */
    TB_Lift_Poll();
    osDelay(1);
  }
}

/* USER CODE END Application */

