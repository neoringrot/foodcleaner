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
#include "tb_drv8306.h"
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

    (void)Membrane_Poll();
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
  *           tb_m1_enable    / tb_m2_enable    -> DRV8306 BLDC (U11 / U16),
  *                                                speed via tb_mX_rpm
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
  TB_DRV8306_Init();
  TB_Lift_Init();
  for(;;)
  {
    TB_DRV8871_Poll();
    TB_StepMotor_Poll();
    TB_DRV8306_Poll();
    TB_Lift_Poll();
    osDelay(1);
  }
}

/* USER CODE END Application */

