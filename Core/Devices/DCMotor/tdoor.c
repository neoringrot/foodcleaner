#include "tdoor.h"
#include "gpio_ctrl.h"

/* U7 Trash Door: TIM3 CH3/CH4, VM enable on o_EN_DOOR_TRASH (PE3). */
static DRV8871_HandleTypeDef tdoor =
{
	.htim    = &htim3,
	.in1_ch  = TIM_CHANNEL_3,             /* PC8 tim3_TDOOR_IN1 */
	.in2_ch  = TIM_CHANNEL_4,             /* PC9 tim3_TDOOR_IN2 */
	.en_port = o_EN_DOOR_TRASH_GPIO_Port, /* PE3                */
	.en_pin  = o_EN_DOOR_TRASH_Pin,
};

void TDoor_Init(void)
{
	DRV8871_Init(&tdoor);
}

void TDoor_Enable(void)
{
	DRV8871_Enable(&tdoor);
}

void TDoor_Disable(void)
{
	DRV8871_Disable(&tdoor);
}

/* Direction mapping is provisional (see tdoor.h) -- swap Forward/Reverse here
 * if the door opens the wrong way on the bench. */
void TDoor_Open(uint8_t duty_pct)
{
	DRV8871_Forward(&tdoor, duty_pct);
}

void TDoor_Close(uint8_t duty_pct)
{
	DRV8871_Reverse(&tdoor, duty_pct);
}

void TDoor_Brake(void)
{
	DRV8871_Brake(&tdoor);
}

void TDoor_Stop(void)
{
	DRV8871_Coast(&tdoor);
}

/* Door has reached the limit when the Hall pin sits at its asserted level
 * (DOOR_LIMIT_ACTIVE_HIGH; default 0 = active-low, see tdoor.h). */
uint8_t TDoor_AtOpen(void)
{
	return (gpio_ctrl_exti_read(GPIO_EXTI_THALL_OPEN) == DOOR_LIMIT_ACTIVE_HIGH) ? 1U : 0U;
}

uint8_t TDoor_AtClose(void)
{
	return (gpio_ctrl_exti_read(GPIO_EXTI_THALL_CLOSE) == DOOR_LIMIT_ACTIVE_HIGH) ? 1U : 0U;
}
