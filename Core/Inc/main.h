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

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define exti0_BIMETAL1_Pin GPIO_PIN_0
#define exti0_BIMETAL1_GPIO_Port GPIOF
#define exti1_BIMETAL2_Pin GPIO_PIN_1
#define exti1_BIMETAL2_GPIO_Port GPIOF
#define exti2_BIMETAL3_Pin GPIO_PIN_2
#define exti2_BIMETAL3_GPIO_Port GPIOF
#define exti3_BIMETAL4_Pin GPIO_PIN_3
#define exti3_BIMETAL4_GPIO_Port GPIOF
#define exti4_BIMETAL5_Pin GPIO_PIN_4
#define exti4_BIMETAL5_GPIO_Port GPIOF
#define exti5_LEAD_SW_Pin GPIO_PIN_5
#define exti5_LEAD_SW_GPIO_Port GPIOF
#define exti6_WATER_SEN1_Pin GPIO_PIN_6
#define exti6_WATER_SEN1_GPIO_Port GPIOF
#define exti7_WATER_SEN2_Pin GPIO_PIN_7
#define exti7_WATER_SEN2_GPIO_Port GPIOF
#define o_WATER_ON_Pin GPIO_PIN_8
#define o_WATER_ON_GPIO_Port GPIOF
#define o_EN_DOOR_TRASH_Pin GPIO_PIN_9
#define o_EN_DOOR_TRASH_GPIO_Port GPIOF
#define o_EN_DOOR_WATER_Pin GPIO_PIN_10
#define o_EN_DOOR_WATER_GPIO_Port GPIOF
#define adc1_THERMISTOR1_Pin GPIO_PIN_0
#define adc1_THERMISTOR1_GPIO_Port GPIOC
#define adc1_THERMISTOR2_Pin GPIO_PIN_1
#define adc1_THERMISTOR2_GPIO_Port GPIOC
#define adc1_THERMISTOR3_Pin GPIO_PIN_2
#define adc1_THERMISTOR3_GPIO_Port GPIOC
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
#define o_FRAME_WP_Pin GPIO_PIN_15
#define o_FRAME_WP_GPIO_Port GPIOE
#define i2c2_FRAME_SCL_Pin GPIO_PIN_10
#define i2c2_FRAME_SCL_GPIO_Port GPIOB
#define i2c2_FRAME_SDA_Pin GPIO_PIN_11
#define i2c2_FRAME_SDA_GPIO_Port GPIOB
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
#define i_LIFT_IN1_Pin GPIO_PIN_3
#define i_LIFT_IN1_GPIO_Port GPIOG
#define i_LIFT_IN2_Pin GPIO_PIN_4
#define i_LIFT_IN2_GPIO_Port GPIOG
#define i_WDOOR_IN1_Pin GPIO_PIN_6
#define i_WDOOR_IN1_GPIO_Port GPIOC
#define i_WDDOR_IN2_Pin GPIO_PIN_7
#define i_WDDOR_IN2_GPIO_Port GPIOC
#define exti8_TDOOR_IN1_Pin GPIO_PIN_8
#define exti8_TDOOR_IN1_GPIO_Port GPIOC
#define exti9_TDOOR_IN2_Pin GPIO_PIN_9
#define exti9_TDOOR_IN2_GPIO_Port GPIOC
#define exti11_TIMER_OUT_Pin GPIO_PIN_11
#define exti11_TIMER_OUT_GPIO_Port GPIOA
#define exti11_TIMER_OUT_EXTI_IRQn EXTI15_10_IRQn
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

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
