/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, o_WATER_ON_Pin|o_EN_DOOR_TRASH_Pin|o_EN_DOOR_WATER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, o_EN_SPK_Pin|o_HT_POWER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(o_SPI1_EEPROM_CS_GPIO_Port, o_SPI1_EEPROM_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, o_M1_ENABLE_Pin|o_M2_ENABLE_Pin|o_M1_DIR_Pin|o_M2_DIR_Pin
                          |o_FRAME_WP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, o_M1_nBRAKE_Pin|o_M2_nBRAKE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, o_MTR_DC_LIFT_Pin|o_VALVE_DRY_IN_Pin|o_VALVE_DRAIN_CLN_Pin|o_FAN_VAPOR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, o_STEP1_M1_Pin|o_STEP1_M2_Pin|o_STEP1_M3_Pin|o_STEP1_M4_Pin
                          |o_STEP2_M1_Pin|o_STEP2_M2_Pin|o_STEP2_M3_Pin|o_STEP2_M4_Pin
                          |o_BLE_MODE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(o_FAN_EXHAUST_GPIO_Port, o_FAN_EXHAUST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(i_WDDOR_IN2_GPIO_Port, i_WDDOR_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : exti0_BIMETAL1_Pin exti1_BIMETAL2_Pin exti2_BIMETAL3_Pin exti3_BIMETAL4_Pin
                           exti4_BIMETAL5_Pin exti5_LEAD_SW_Pin exti6_WATER_SEN1_Pin exti7_WATER_SEN2_Pin
                           exti13_M2_nFAULT_Pin exti14_M1_nFAULT_Pin */
  GPIO_InitStruct.Pin = exti0_BIMETAL1_Pin|exti1_BIMETAL2_Pin|exti2_BIMETAL3_Pin|exti3_BIMETAL4_Pin
                          |exti4_BIMETAL5_Pin|exti5_LEAD_SW_Pin|exti6_WATER_SEN1_Pin|exti7_WATER_SEN2_Pin
                          |exti13_M2_nFAULT_Pin|exti14_M1_nFAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : o_WATER_ON_Pin o_EN_DOOR_TRASH_Pin o_EN_DOOR_WATER_Pin */
  GPIO_InitStruct.Pin = o_WATER_ON_Pin|o_EN_DOOR_TRASH_Pin|o_EN_DOOR_WATER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : o_EN_SPK_Pin o_HT_POWER_Pin */
  GPIO_InitStruct.Pin = o_EN_SPK_Pin|o_HT_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : o_SPI1_EEPROM_CS_Pin i_WDDOR_IN2_Pin */
  GPIO_InitStruct.Pin = o_SPI1_EEPROM_CS_Pin|i_WDDOR_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : exti12_M2_FGOT_Pin exti15_M1_FGOT_Pin */
  GPIO_InitStruct.Pin = exti12_M2_FGOT_Pin|exti15_M1_FGOT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : o_M1_ENABLE_Pin o_M2_ENABLE_Pin o_M1_DIR_Pin o_M2_DIR_Pin
                           o_M1_nBRAKE_Pin o_M2_nBRAKE_Pin o_FRAME_WP_Pin */
  GPIO_InitStruct.Pin = o_M1_ENABLE_Pin|o_M2_ENABLE_Pin|o_M1_DIR_Pin|o_M2_DIR_Pin
                          |o_M1_nBRAKE_Pin|o_M2_nBRAKE_Pin|o_FRAME_WP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : o_MTR_DC_LIFT_Pin o_VALVE_DRY_IN_Pin o_VALVE_DRAIN_CLN_Pin o_FAN_VAPOR_Pin */
  GPIO_InitStruct.Pin = o_MTR_DC_LIFT_Pin|o_VALVE_DRY_IN_Pin|o_VALVE_DRAIN_CLN_Pin|o_FAN_VAPOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : o_STEP1_M1_Pin o_STEP1_M2_Pin o_STEP1_M3_Pin o_STEP1_M4_Pin
                           o_STEP2_M1_Pin o_STEP2_M2_Pin o_STEP2_M3_Pin o_STEP2_M4_Pin
                           o_BLE_MODE_Pin */
  GPIO_InitStruct.Pin = o_STEP1_M1_Pin|o_STEP1_M2_Pin|o_STEP1_M3_Pin|o_STEP1_M4_Pin
                          |o_STEP2_M1_Pin|o_STEP2_M2_Pin|o_STEP2_M3_Pin|o_STEP2_M4_Pin
                          |o_BLE_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : o_FAN_EXHAUST_Pin */
  GPIO_InitStruct.Pin = o_FAN_EXHAUST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(o_FAN_EXHAUST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : i_LIFT_IN1_Pin i_LIFT_IN2_Pin */
  GPIO_InitStruct.Pin = i_LIFT_IN1_Pin|i_LIFT_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : i_WDOOR_IN1_Pin */
  GPIO_InitStruct.Pin = i_WDOOR_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(i_WDOOR_IN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : exti8_TDOOR_IN1_Pin exti9_TDOOR_IN2_Pin */
  GPIO_InitStruct.Pin = exti8_TDOOR_IN1_Pin|exti9_TDOOR_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : exti11_TIMER_OUT_Pin */
  GPIO_InitStruct.Pin = exti11_TIMER_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(exti11_TIMER_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : i_BLE_STATUS_Pin */
  GPIO_InitStruct.Pin = i_BLE_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(i_BLE_STATUS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
