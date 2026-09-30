/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* ==========================================================================
 * ENABLE_TESTBENCH_APP - 테스트벤치 원격검증(PC 앱) 코드 일괄 스위치
 *
 *   1 = R0 프로토콜에 테스트벤치 제어/관측 명령(0x27 TB_STATE / 0x33 TB_CTRL)을
 *       얹고 Testbench/tb_app.* 를 빌드한다. PC 앱 src/parts_verification 이
 *       이 경로로 개별기능 검증을 돈다.
 *   0 = 위 전부를 컴파일에서 제외한다. 양산 바이너리는 0 으로 둔다 -
 *       시리얼로 액추에이터를 직접 돌릴 수 있는 경로이기 때문이다.
 *
 * 이 매크로 하나만 0 으로 내리면 되도록 모든 추가분을 #if 로 묶어 두었다.
 * 걸려 있는 곳: Devices/Comm/protocol_r0.{h,c} · Testbench/tb_app.{h,c} ·
 *               Src/freertos.c(TbApp_Init). tb_app.c 는 파일 전체가 이 가드 안이다.
 * ========================================================================== */
#ifndef ENABLE_TESTBENCH_APP
#define ENABLE_TESTBENCH_APP   1
#endif

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define o_WATER_ON_Pin GPIO_PIN_2
#define o_WATER_ON_GPIO_Port GPIOE
#define o_EN_DOOR_TRASH_Pin GPIO_PIN_3
#define o_EN_DOOR_TRASH_GPIO_Port GPIOE
#define o_EN_DOOR_WATER_Pin GPIO_PIN_4
#define o_EN_DOOR_WATER_GPIO_Port GPIOE
#define o_NEW_SENSOR_EN_Pin GPIO_PIN_5
#define o_NEW_SENSOR_EN_GPIO_Port GPIOE
#define exti0_BIMETAL_70_Pin GPIO_PIN_0
#define exti0_BIMETAL_70_GPIO_Port GPIOF
#define exti0_BIMETAL_70_EXTI_IRQn EXTI0_IRQn
#define exti1_BIMETAL_50_Pin GPIO_PIN_1
#define exti1_BIMETAL_50_GPIO_Port GPIOF
#define exti1_BIMETAL_50_EXTI_IRQn EXTI1_IRQn
#define exti2_THALL_CLOSE_Pin GPIO_PIN_2
#define exti2_THALL_CLOSE_GPIO_Port GPIOF
#define exti2_THALL_CLOSE_EXTI_IRQn EXTI2_IRQn
#define exti3_WHALL_CLOSE_Pin GPIO_PIN_3
#define exti3_WHALL_CLOSE_GPIO_Port GPIOF
#define exti3_WHALL_CLOSE_EXTI_IRQn EXTI3_IRQn
#define exti4_WHALL_OPEN_Pin GPIO_PIN_4
#define exti4_WHALL_OPEN_GPIO_Port GPIOF
#define exti4_WHALL_OPEN_EXTI_IRQn EXTI4_IRQn
#define exti5_THALL_OPEN_Pin GPIO_PIN_5
#define exti5_THALL_OPEN_GPIO_Port GPIOF
#define exti5_THALL_OPEN_EXTI_IRQn EXTI9_5_IRQn
#define exti6_WATER_SEN1_Pin GPIO_PIN_6
#define exti6_WATER_SEN1_GPIO_Port GPIOF
#define exti6_WATER_SEN1_EXTI_IRQn EXTI9_5_IRQn
#define exti7_NEW_HALL_INT_Pin GPIO_PIN_7
#define exti7_NEW_HALL_INT_GPIO_Port GPIOF
#define exti7_NEW_HALL_INT_EXTI_IRQn EXTI9_5_IRQn
#define exti8_TIMER_OUT_Pin GPIO_PIN_8
#define exti8_TIMER_OUT_GPIO_Port GPIOF
#define exti8_TIMER_OUT_EXTI_IRQn EXTI9_5_IRQn
#define exti9_HALL_INT1_Pin GPIO_PIN_9
#define exti9_HALL_INT1_GPIO_Port GPIOF
#define exti9_HALL_INT1_EXTI_IRQn EXTI9_5_IRQn
#define exti10_HALL_INT2_Pin GPIO_PIN_10
#define exti10_HALL_INT2_GPIO_Port GPIOF
#define exti10_HALL_INT2_EXTI_IRQn EXTI15_10_IRQn
#define adc1_THERMISTOR1_Pin GPIO_PIN_0
#define adc1_THERMISTOR1_GPIO_Port GPIOC
#define adc1_THERMISTOR2_Pin GPIO_PIN_1
#define adc1_THERMISTOR2_GPIO_Port GPIOC
#define adc1_THERMISTOR3_Pin GPIO_PIN_2
#define adc1_THERMISTOR3_GPIO_Port GPIOC
#define adc1_WEIGHT_SENS_Pin GPIO_PIN_3
#define adc1_WEIGHT_SENS_GPIO_Port GPIOC
#define adc1_DISTANCE_SENS_Pin GPIO_PIN_0
#define adc1_DISTANCE_SENS_GPIO_Port GPIOA
#define o_EN_SPK_Pin GPIO_PIN_3
#define o_EN_SPK_GPIO_Port GPIOA
#define dac_SPK_OUT_Pin GPIO_PIN_4
#define dac_SPK_OUT_GPIO_Port GPIOA
#define spi1_EEPROM_SCK_Pin GPIO_PIN_5
#define spi1_EEPROM_SCK_GPIO_Port GPIOA
#define spi1_EEPROM_MISO_Pin GPIO_PIN_6
#define spi1_EEPROM_MISO_GPIO_Port GPIOA
#define spi1_EEPROM_MOSI_Pin GPIO_PIN_7
#define spi1_EEPROM_MOSI_GPIO_Port GPIOA
#define o_SPI1_EEPROM_CS_Pin GPIO_PIN_4
#define o_SPI1_EEPROM_CS_GPIO_Port GPIOC
#define adc1_DC_CURR1_Pin GPIO_PIN_5
#define adc1_DC_CURR1_GPIO_Port GPIOC
#define adc1_DC_CURR2_Pin GPIO_PIN_0
#define adc1_DC_CURR2_GPIO_Port GPIOB
#define adc1_DC_CURR3_Pin GPIO_PIN_1
#define adc1_DC_CURR3_GPIO_Port GPIOB
#define exti11_HALL_INT3_Pin GPIO_PIN_11
#define exti11_HALL_INT3_GPIO_Port GPIOF
#define exti11_HALL_INT3_EXTI_IRQn EXTI15_10_IRQn
#define exti12_M2_FGOT_Pin GPIO_PIN_12
#define exti12_M2_FGOT_GPIO_Port GPIOF
#define exti12_M2_FGOT_EXTI_IRQn EXTI15_10_IRQn
#define exti13_M2_nFAULT_Pin GPIO_PIN_13
#define exti13_M2_nFAULT_GPIO_Port GPIOF
#define exti13_M2_nFAULT_EXTI_IRQn EXTI15_10_IRQn
#define exti14_M1_nFAULT_Pin GPIO_PIN_14
#define exti14_M1_nFAULT_GPIO_Port GPIOF
#define exti14_M1_nFAULT_EXTI_IRQn EXTI15_10_IRQn
#define exti15_M1_FGOT_Pin GPIO_PIN_15
#define exti15_M1_FGOT_GPIO_Port GPIOF
#define exti15_M1_FGOT_EXTI_IRQn EXTI15_10_IRQn
#define o_BLDC_FAN_Pin GPIO_PIN_1
#define o_BLDC_FAN_GPIO_Port GPIOG
#define o_M1_ENABLE_Pin GPIO_PIN_7
#define o_M1_ENABLE_GPIO_Port GPIOE
#define o_M2_ENABLE_Pin GPIO_PIN_8
#define o_M2_ENABLE_GPIO_Port GPIOE
#define tim1_M1_PWM_Pin GPIO_PIN_9
#define tim1_M1_PWM_GPIO_Port GPIOE
#define o_M1_DIR_Pin GPIO_PIN_10
#define o_M1_DIR_GPIO_Port GPIOE
#define tim1_M2_PWM_Pin GPIO_PIN_11
#define tim1_M2_PWM_GPIO_Port GPIOE
#define o_M2_DIR_Pin GPIO_PIN_12
#define o_M2_DIR_GPIO_Port GPIOE
#define o_M1_nBRAKE_Pin GPIO_PIN_13
#define o_M1_nBRAKE_GPIO_Port GPIOE
#define o_M2_nBRAKE_Pin GPIO_PIN_14
#define o_M2_nBRAKE_GPIO_Port GPIOE
#define o_MTR_DC_LIFT_Pin GPIO_PIN_12
#define o_MTR_DC_LIFT_GPIO_Port GPIOB
#define o_VALVE_DRY_IN_Pin GPIO_PIN_13
#define o_VALVE_DRY_IN_GPIO_Port GPIOB
#define o_VALVE_DRAIN_CLN_Pin GPIO_PIN_14
#define o_VALVE_DRAIN_CLN_GPIO_Port GPIOB
#define o_FAN_VAPOR_Pin GPIO_PIN_15
#define o_FAN_VAPOR_GPIO_Port GPIOB
#define o_STEP1_M1_Pin GPIO_PIN_8
#define o_STEP1_M1_GPIO_Port GPIOD
#define o_STEP1_M2_Pin GPIO_PIN_9
#define o_STEP1_M2_GPIO_Port GPIOD
#define o_STEP1_M3_Pin GPIO_PIN_10
#define o_STEP1_M3_GPIO_Port GPIOD
#define o_STEP1_M4_Pin GPIO_PIN_11
#define o_STEP1_M4_GPIO_Port GPIOD
#define o_STEP2_M1_Pin GPIO_PIN_12
#define o_STEP2_M1_GPIO_Port GPIOD
#define o_STEP2_M2_Pin GPIO_PIN_13
#define o_STEP2_M2_GPIO_Port GPIOD
#define o_STEP2_M3_Pin GPIO_PIN_14
#define o_STEP2_M3_GPIO_Port GPIOD
#define o_STEP2_M4_Pin GPIO_PIN_15
#define o_STEP2_M4_GPIO_Port GPIOD
#define o_FAN_EXHAUST_Pin GPIO_PIN_2
#define o_FAN_EXHAUST_GPIO_Port GPIOG
#define tim3_WDOOR_IN1_Pin GPIO_PIN_6
#define tim3_WDOOR_IN1_GPIO_Port GPIOC
#define tim3_WDDOR_IN2_Pin GPIO_PIN_7
#define tim3_WDDOR_IN2_GPIO_Port GPIOC
#define tim3_TDOOR_IN1_Pin GPIO_PIN_8
#define tim3_TDOOR_IN1_GPIO_Port GPIOC
#define tim3_TDOOR_IN2_Pin GPIO_PIN_9
#define tim3_TDOOR_IN2_GPIO_Port GPIOC
#define o_HT_POWER_Pin GPIO_PIN_12
#define o_HT_POWER_GPIO_Port GPIOA
#define uart4_WIFI_TX_Pin GPIO_PIN_10
#define uart4_WIFI_TX_GPIO_Port GPIOC
#define uart4_WIF_RX_Pin GPIO_PIN_11
#define uart4_WIF_RX_GPIO_Port GPIOC
#define uart5_BLE_TX_Pin GPIO_PIN_12
#define uart5_BLE_TX_GPIO_Port GPIOC
#define uart5_BLE_RX_Pin GPIO_PIN_2
#define uart5_BLE_RX_GPIO_Port GPIOD
#define i_BLE_STATUS_Pin GPIO_PIN_3
#define i_BLE_STATUS_GPIO_Port GPIOD
#define o_BLE_MODE_Pin GPIO_PIN_4
#define o_BLE_MODE_GPIO_Port GPIOD
#define tim4_LIFT_IN1_Pin GPIO_PIN_8
#define tim4_LIFT_IN1_GPIO_Port GPIOB
#define tim4_LIFT_IN2_Pin GPIO_PIN_9
#define tim4_LIFT_IN2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
