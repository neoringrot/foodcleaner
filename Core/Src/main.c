/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dac.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gpio_ctrl.h"
#include "drv8306.h"
#include "lift_motor.h"
#include "wifi.h"
#include "ble.h"
#include "membrane.h"
#include "lm4871.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* U6 Lift DRV8871 -- IN1/IN2 on PG3/PG4 are plain GPIO (no timer), so unlike
   * the TIM3 doors (U5/U7) this device is a GPIO-only driver. Bring it to a
   * safe state (VM off, coast) before the scheduler starts. */
  Lift_Init();

  /* Comms modules: arm the UART RX interrupts before the scheduler starts.
   * WiFi_Init (ESP-AT on UART4) and BLE_Init (BoT-nLE521 on UART5) must run
   * after MX_UART4_Init()/MX_UART5_Init() above. BLE_Init also drives
   * o_BLE_MODE LOW to select the default BYPASS mode. */
  WiFi_Init();
  BLE_Init();

  /* Front-panel membrane keypad + LED latch on I2C1 (U8=DIS-LED @0x39,
   * U9=DIS-SW @0x38). DISABLED: the tb_tca9554 testbed (Core/Testbench) now
   * owns U8/U9 - it configures both expanders itself in TB_TCA9554_Init()
   * (StartMotorTask, freertos.c) and would collide with this driver on the LED
   * port and I2C bus. Re-enable this line and Membrane_Poll() in freertos.c to
   * restore the generic press-to-toggle keypad behaviour. */
  /* Membrane_Init(NULL); */

  /* U15 LM4871 speaker amplifier driven by the DAC: PA4 (DAC_OUT1) -> Ci/Ri ->
   * -IN, shutdown on PA3 (o_EN_SPK, active-low). Must run after MX_DAC_Init()
   * and MX_GPIO_Init() above. Init parks the DAC at mid-scale and leaves the
   * amp muted; the short power-on beep confirms the audio path end-to-end.
   * (Blocking ~120ms via the DWT delay -- fine here, before the scheduler.) */
  LM4871_BoardInit();
  LM4871_Beep(&lm4871, 2000u, 120u);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  GPIO EXTI line detection callback (single override for the build).
  * @note   Called from HAL_GPIO_EXTI_IRQHandler() (see stm32f1xx_it.c) for every
  *         configured EXTI line on a falling edge. There can be only one strong
  *         HAL_GPIO_EXTI_Callback in the image, so this is the central router:
  *           1) latch the per-pin gpio_ctrl flag for ALL EXTI pins (not consumed
  *              yet - reserved for future edge handling);
  *           2) forward the M1/M2 FGOUT and nFAULT lines to their DRV8306
  *              instance so tacho counting / fault latching keep working.
  * @param  GPIO_Pin : the pin (GPIO_PIN_x) that triggered the interrupt
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  gpio_ctrl_exti_dispatch(GPIO_Pin);

  switch (GPIO_Pin)
  {
  case exti15_M1_FGOT_Pin:      /* PF15 */
  case exti14_M1_nFAULT_Pin:    /* PF14 */
    DRV8306_OnEXTI(&drv8306_m1, GPIO_Pin);
    break;
  case exti12_M2_FGOT_Pin:      /* PF12 */
  case exti13_M2_nFAULT_Pin:    /* PF13 */
    DRV8306_OnEXTI(&drv8306_m2, GPIO_Pin);
    break;
  default:
    break;
  }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
