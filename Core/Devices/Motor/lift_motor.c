#include "lift_motor.h"

/* U6 Lift: IN1/IN2 driven as plain GPIO (PG3/PG4), VM enable on o_MTR_DC_LIFT
 * (PB12). See lift_motor.h for the pin map and the DRV8871 IN1/IN2 truth table. */

void Lift_Init(void)
{
	/* Pins are already configured GPIO_MODE_OUTPUT_PP in MX_GPIO_Init();
	 * force a known-safe state: VM off, both inputs low (coast). */
	Lift_Disable();
	Lift_Stop();
}

void Lift_Enable(void)
{
	gpio_ctrl_on(GPIO_OUT_MTR_DC_LIFT);
}

void Lift_Disable(void)
{
	gpio_ctrl_off(GPIO_OUT_MTR_DC_LIFT);
}

/* Direction mapping is provisional (see lift_motor.h) -- swap the Up/Down bodies if
 * the lift runs the wrong way on the bench. */
void Lift_Up(void)
{
	gpio_ctrl_off(GPIO_OUT_LIFT_IN2);
	gpio_ctrl_on(GPIO_OUT_LIFT_IN1);   /* IN1=1, IN2=0 -> forward */
}

void Lift_Down(void)
{
	gpio_ctrl_off(GPIO_OUT_LIFT_IN1);
	gpio_ctrl_on(GPIO_OUT_LIFT_IN2);   /* IN1=0, IN2=1 -> reverse */
}

void Lift_Brake(void)
{
	gpio_ctrl_on(GPIO_OUT_LIFT_IN1);
	gpio_ctrl_on(GPIO_OUT_LIFT_IN2);   /* IN1=1, IN2=1 -> brake */
}

void Lift_Stop(void)
{
	gpio_ctrl_off(GPIO_OUT_LIFT_IN1);
	gpio_ctrl_off(GPIO_OUT_LIFT_IN2);  /* IN1=0, IN2=0 -> coast */
}
