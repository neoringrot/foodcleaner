#include "wdoor.h"

/* U5 Water Door: TIM3 CH1/CH2, VM enable on o_EN_DOOR_WATER (PF10). */
static DRV8871_HandleTypeDef wdoor =
{
	.htim    = &htim3,
	.in1_ch  = TIM_CHANNEL_1,             /* PC6 tim3_WDOOR_IN1 */
	.in2_ch  = TIM_CHANNEL_2,             /* PC7 tim3_WDDOR_IN2 */
	.en_port = o_EN_DOOR_WATER_GPIO_Port, /* PF10               */
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
