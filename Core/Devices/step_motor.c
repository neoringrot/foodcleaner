#include "step_motor.h"

/* Phase active level. GPIO HIGH is assumed to turn the transistor ON and
 * energize the coil. If the switching stage is inverting, swap these two. */
#define STEP_MOTOR_ON_STATE   GPIO_PIN_SET
#define STEP_MOTOR_OFF_STATE  GPIO_PIN_RESET

/* Full-step, 2-phase-on sequence. Bit i corresponds to phase Mi+1:
 *   bit0=M1, bit1=M2, bit2=M3, bit3=M4.
 *   idx0 M1+M2 (0b0011), idx1 M2+M3 (0b0110), idx2 M3+M4 (0b1100), idx3 M4+M1 (0b1001).
 * Stepping forward walks 0->1->2->3->0; reverse walks the other way. If the
 * motor only vibrates instead of turning, the physical M1..M4 wiring order does
 * not match this sequence -- reorder the handle's port[]/pin[] arrays. */
static const uint8_t k_step_seq[STEP_MOTOR_PHASES] =
{
	0x3U, /* M1 + M2 */
	0x6U, /* M2 + M3 */
	0xCU, /* M3 + M4 */
	0x9U, /* M4 + M1 */
};

/* Drive the four phase pins from a bitmask (bit i -> phase Mi+1). */
static void StepMotor_Apply(StepMotor_HandleTypeDef *h, uint8_t mask)
{
	uint8_t i;

	for (i = 0U; i < STEP_MOTOR_PHASES; i++)
	{
		HAL_GPIO_WritePin(h->port[i], h->pin[i],
		                  (mask & (1U << i)) ? STEP_MOTOR_ON_STATE : STEP_MOTOR_OFF_STATE);
	}
	h->energized = 1U;
}

void StepMotor_Init(StepMotor_HandleTypeDef *h)
{
	h->phase = 0U;
	StepMotor_Release(h);
}

void StepMotor_Step(StepMotor_HandleTypeDef *h, int8_t dir)
{
	/* +1 / -1 modulo 4 (adding 3 == subtracting 1). */
	h->phase = (uint8_t)((h->phase + ((dir < 0) ? 3U : 1U)) & 0x3U);
	StepMotor_Apply(h, k_step_seq[h->phase]);
}

void StepMotor_Hold(StepMotor_HandleTypeDef *h)
{
	StepMotor_Apply(h, k_step_seq[h->phase]);
}

void StepMotor_Release(StepMotor_HandleTypeDef *h)
{
	uint8_t i;

	for (i = 0U; i < STEP_MOTOR_PHASES; i++)
	{
		HAL_GPIO_WritePin(h->port[i], h->pin[i], STEP_MOTOR_OFF_STATE);
	}
	h->energized = 0U;
}
