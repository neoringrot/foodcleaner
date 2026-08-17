#include "lift_motor.h"

/* U6 Lift: IN1/IN2 driven by TIM4 CH3/CH4 PWM (PB8/PB9), VM enable on
 * o_MTR_DC_LIFT (PB12). See lift_motor.h for the pin map and the DRV8871
 * IN1/IN2 truth table. Structurally identical to the TIM3 doors (U5/U7). */
static DRV8871_HandleTypeDef lift =
{
	.htim    = &htim4,
	.in1_ch  = TIM_CHANNEL_3,           /* PB8 tim4_LIFT_IN1 */
	.in2_ch  = TIM_CHANNEL_4,           /* PB9 tim4_LIFT_IN2 */
	.en_port = o_MTR_DC_LIFT_GPIO_Port, /* PB12              */
	.en_pin  = o_MTR_DC_LIFT_Pin,
};

void Lift_Init(void)
{
	/* Starts TIM4 PWM on both channels, sets the shared 20 kHz ARR, parks both
	 * inputs at 0 (coast). Then force a known-safe state: VM off, coast. */
	DRV8871_Init(&lift);
	Lift_Disable();
	Lift_Stop();
}

void Lift_Enable(void)
{
	DRV8871_Enable(&lift);
}

void Lift_Disable(void)
{
	DRV8871_Disable(&lift);
}

/* Direction mapping is provisional (see lift_motor.h) -- swap the Up/Down bodies
 * if the lift runs the wrong way on the bench. */
void Lift_Up(void)
{
	DRV8871_Forward(&lift, 100U);   /* IN1 PWM 100%, IN2 low -> forward */
}

void Lift_Down(void)
{
	DRV8871_Reverse(&lift, 100U);   /* IN2 PWM 100%, IN1 low -> reverse */
}

void Lift_UpSpeed(uint8_t duty_pct)
{
	DRV8871_Forward(&lift, duty_pct);
}

void Lift_DownSpeed(uint8_t duty_pct)
{
	DRV8871_Reverse(&lift, duty_pct);
}

void Lift_Brake(void)
{
	DRV8871_Brake(&lift);           /* IN1=IN2 high -> brake */
}

void Lift_Stop(void)
{
	DRV8871_Coast(&lift);           /* IN1=IN2 low -> coast */
}

/* ---- 모니터링 read-only (protocol_r0 0x26 OUTPUT) -------------------- */
uint8_t Lift_GetDrive(void)
{
	return lift.drive;
}

uint8_t Lift_GetDuty(void)
{
	return lift.duty_pct;
}
