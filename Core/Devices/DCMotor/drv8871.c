#include "drv8871.h"

/* Timer clock assumption: TIM3 prescaler is 0 (see MX_TIM3_Init in tim.c). TIM3
 * is on APB1, and SystemClock_Config sets APB1CLKDivider = RCC_HCLK_DIV2. On
 * STM32F1 a non-unity APB prescaler multiplies the timer clock by 2, so
 * TIM3CLK = 2 * (HCLK/2) = HCLK = SystemCoreClock -- i.e. the same relation the
 * hasudo1 TIM1 driver relies on. ARR is therefore SystemCoreClock/freq - 1. */

static uint32_t DRV8871_DutyTicks(const DRV8871_HandleTypeDef *h, uint8_t duty_pct)
{
	if (duty_pct > 100U)
	{
		duty_pct = 100U;
	}
	return (h->arr + 1U) * duty_pct / 100U;
}

void DRV8871_Init(DRV8871_HandleTypeDef *h)
{
	h->arr = (SystemCoreClock / DRV8871_PWM_FREQ_HZ) - 1U;

	/* ARR is timer-wide: every DRV8871 on this timer sets the same value
	 * (all use DRV8871_PWM_FREQ_HZ), so repeated Init calls are harmless. */
	__HAL_TIM_SET_AUTORELOAD(h->htim, h->arr);
	__HAL_TIM_SET_COMPARE(h->htim, h->in1_ch, 0);
	__HAL_TIM_SET_COMPARE(h->htim, h->in2_ch, 0);

	/* Both channels are started here and never stopped again; a channel
	 * compare of 0 holds its pin continuously low (see DRV8871_Coast()),
	 * keeping IN1/IN2 actively driven rather than floating between direction
	 * changes. */
	HAL_TIM_PWM_Start(h->htim, h->in1_ch);
	HAL_TIM_PWM_Start(h->htim, h->in2_ch);

	/* Keep VM off until a run is actually commanded (gpio.c already leaves the
	 * enable pins low out of reset; make it explicit here too). */
	DRV8871_Disable(h);
}

void DRV8871_Enable(DRV8871_HandleTypeDef *h)
{
	HAL_GPIO_WritePin(h->en_port, h->en_pin, GPIO_PIN_SET);
}

void DRV8871_Disable(DRV8871_HandleTypeDef *h)
{
	HAL_GPIO_WritePin(h->en_port, h->en_pin, GPIO_PIN_RESET);
}

/* Speed control via coast-decay (datasheet section 7.4.2): the driven input is
 * PWM'd for speed while the other input is held low the whole time. */
void DRV8871_Forward(DRV8871_HandleTypeDef *h, uint8_t duty_pct)
{
	__HAL_TIM_SET_COMPARE(h->htim, h->in2_ch, 0);
	__HAL_TIM_SET_COMPARE(h->htim, h->in1_ch, DRV8871_DutyTicks(h, duty_pct));
}

void DRV8871_Reverse(DRV8871_HandleTypeDef *h, uint8_t duty_pct)
{
	__HAL_TIM_SET_COMPARE(h->htim, h->in1_ch, 0);
	__HAL_TIM_SET_COMPARE(h->htim, h->in2_ch, DRV8871_DutyTicks(h, duty_pct));
}

/* A compare value past ARR keeps a channel's output active for the whole period
 * (the counter wraps before ever reaching CCR), i.e. constant logic high on
 * both IN1 and IN2 -- Table 1's brake state. */
void DRV8871_Brake(DRV8871_HandleTypeDef *h)
{
	__HAL_TIM_SET_COMPARE(h->htim, h->in1_ch, h->arr + 1U);
	__HAL_TIM_SET_COMPARE(h->htim, h->in2_ch, h->arr + 1U);
}

void DRV8871_Coast(DRV8871_HandleTypeDef *h)
{
	__HAL_TIM_SET_COMPARE(h->htim, h->in1_ch, 0);
	__HAL_TIM_SET_COMPARE(h->htim, h->in2_ch, 0);
}
