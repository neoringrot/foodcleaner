#include "wdoor.h"
#include "gpio_ctrl.h"

/* U5 Water Door: TIM3 CH1/CH2, VM enable on o_EN_DOOR_WATER (PE4). */
static DRV8871_HandleTypeDef wdoor =
{
	.htim    = &htim3,
	.in1_ch  = TIM_CHANNEL_1,             /* PC6 tim3_WDOOR_IN1 */
	.in2_ch  = TIM_CHANNEL_2,             /* PC7 tim3_WDDOR_IN2 */
	.en_port = o_EN_DOOR_WATER_GPIO_Port, /* PE4                */
	.en_pin  = o_EN_DOOR_WATER_Pin,
};

void WDoor_Init(void)
{
	DRV8871_Init(&wdoor);
}

void WDoor_Enable(void)
{
	DRV8871_Enable(&wdoor);
}

void WDoor_Disable(void)
{
	DRV8871_Disable(&wdoor);
}

/* Direction mapping is provisional (see wdoor.h) -- swap Forward/Reverse here
 * if the door opens the wrong way on the bench. */
void WDoor_Open(uint8_t duty_pct)
{
	DRV8871_Forward(&wdoor, duty_pct);
}

void WDoor_Close(uint8_t duty_pct)
{
	DRV8871_Reverse(&wdoor, duty_pct);
}

void WDoor_Brake(void)
{
	DRV8871_Brake(&wdoor);
}

void WDoor_Stop(void)
{
	DRV8871_Coast(&wdoor);
}

/* Door has reached the limit when the Hall pin sits at its asserted level
 * (DOOR_LIMIT_ACTIVE_HIGH; default 0 = active-low, see wdoor.h). */
uint8_t WDoor_AtOpen(void)
{
	return (gpio_ctrl_exti_read(GPIO_EXTI_WHALL_OPEN) == DOOR_LIMIT_ACTIVE_HIGH) ? 1U : 0U;
}

uint8_t WDoor_AtClose(void)
{
	return (gpio_ctrl_exti_read(GPIO_EXTI_WHALL_CLOSE) == DOOR_LIMIT_ACTIVE_HIGH) ? 1U : 0U;
}

/* ---- 모니터링 read-only (protocol_r0 0x26 OUTPUT) -------------------- */
uint8_t WDoor_GetDrive(void)
{
	return wdoor.drive;
}

uint8_t WDoor_GetDuty(void)
{
	return wdoor.duty_pct;
}
