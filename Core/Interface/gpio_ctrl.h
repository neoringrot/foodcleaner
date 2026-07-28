#ifndef INTERFACE_GPIO_CTRL_H_
#define INTERFACE_GPIO_CTRL_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * gpio_ctrl - named, table-driven access to the foodcleaner board GPIO.
 *
 * Every digital output/input pin defined in main.h is wrapped behind an enum
 * so callers use a symbol (GPIO_OUT_FAN_VAPOR) instead of repeating the
 * (port, pin) pair at every call site. A lookup table in gpio_ctrl.c maps
 * each enum to its GPIO_TypeDef* / pin mask, so this stays in sync with the
 * CubeMX-generated defines by name.
 *
 * Level convention: these helpers work on the RAW pin voltage level.
 *   on  / enable  -> pin HIGH (GPIO_PIN_SET)
 *   off / disable -> pin LOW  (GPIO_PIN_RESET)
 * Most o_* nets on this board are active-high (see drv8871.h: the VM-enable
 * gate is active-high / non-inverting). For the few active-low nets
 * (o_M1_nBRAKE, o_M2_nBRAKE, and the nFAULT inputs) "on/HIGH" is the raw
 * level, not the logical asserted state - the caller owns that inversion.
 *
 * Reads return the raw voltage level: 0 = pin LOW, 1 = pin HIGH.
 *
 * Pins are split into three groups by how they are configured in MX_GPIO_Init:
 *   gpio_out_t  - o_*    nets, GPIO_MODE_OUTPUT_PP
 *   gpio_in_t   - i_*    nets, GPIO_MODE_INPUT (polled level only)
 *   gpio_exti_t - exti_* nets, GPIO_MODE_IT_FALLING (interrupt sources)
 *
 * EXTI pins carry a per-pin interrupt flag (gpio_exti_flag[]). The flag is set
 * from HAL_GPIO_EXTI_Callback() -> gpio_ctrl_exti_dispatch() on each falling
 * edge and is meant to be polled/cleared by application code later. The flags
 * are wired up but not consumed anywhere yet - storage + accessors only.
 * ---------------------------------------------------------------------- */

/* ---- Output pins (o_* nets, GPIO_MODE_OUTPUT_PP) --------------------- */
typedef enum
{
	GPIO_OUT_WATER_ON = 0,   /* PF8  - o_WATER_ON        */
	GPIO_OUT_EN_DOOR_TRASH,  /* PF9  - o_EN_DOOR_TRASH   */
	GPIO_OUT_EN_DOOR_WATER,  /* PF10 - o_EN_DOOR_WATER   */
	GPIO_OUT_EN_SPK,         /* PA3  - o_EN_SPK          */
	GPIO_OUT_HT_POWER,       /* PA12 - o_HT_POWER        */
	GPIO_OUT_SPI1_EEPROM_CS, /* PC4  - o_SPI1_EEPROM_CS  */
	GPIO_OUT_M1_ENABLE,      /* PE7  - o_M1_ENABLE       */
	GPIO_OUT_M2_ENABLE,      /* PE8  - o_M2_ENABLE       */
	GPIO_OUT_M1_DIR,         /* PE10 - o_M1_DIR          */
	GPIO_OUT_M2_DIR,         /* PE12 - o_M2_DIR          */
	GPIO_OUT_M1_nBRAKE,      /* PE13 - o_M1_nBRAKE (act.low) */
	GPIO_OUT_M2_nBRAKE,      /* PE14 - o_M2_nBRAKE (act.low) */
	GPIO_OUT_FRAME_WP,       /* PE15 - o_FRAME_WP        */
	GPIO_OUT_MTR_DC_LIFT,    /* PB12 - o_MTR_DC_LIFT     */
	GPIO_OUT_VALVE_DRY_IN,   /* PB13 - o_VALVE_DRY_IN    */
	GPIO_OUT_VALVE_DRAIN_CLN,/* PB14 - o_VALVE_DRAIN_CLN */
	GPIO_OUT_FAN_VAPOR,      /* PB15 - o_FAN_VAPOR       */
	GPIO_OUT_STEP1_M1,       /* PD8  - o_STEP1_M1        */
	GPIO_OUT_STEP1_M2,       /* PD9  - o_STEP1_M2        */
	GPIO_OUT_STEP1_M3,       /* PD10 - o_STEP1_M3        */
	GPIO_OUT_STEP1_M4,       /* PD11 - o_STEP1_M4        */
	GPIO_OUT_STEP2_M1,       /* PD12 - o_STEP2_M1        */
	GPIO_OUT_STEP2_M2,       /* PD13 - o_STEP2_M2        */
	GPIO_OUT_STEP2_M3,       /* PD14 - o_STEP2_M3        */
	GPIO_OUT_STEP2_M4,       /* PD15 - o_STEP2_M4        */
	GPIO_OUT_BLE_MODE,       /* PD4  - o_BLE_MODE        */
	GPIO_OUT_FAN_EXHAUST,    /* PG2  - o_FAN_EXHAUST     */
	GPIO_OUT_LIFT_IN1,       /* PG3  - o_LIFT_IN1        */
	GPIO_OUT_LIFT_IN2,       /* PG4  - o_LIFT_IN2        */
	GPIO_OUT_COUNT           /* keep last */
} gpio_out_t;

/* ---- Input pins (i_* nets, GPIO_MODE_INPUT, polled level) ------------ */
typedef enum
{
	GPIO_IN_BLE_STATUS = 0,  /* PD3  - i_BLE_STATUS         */
	GPIO_IN_COUNT            /* keep last */
} gpio_in_t;

/* ---- EXTI pins (exti_* nets, GPIO_MODE_IT_FALLING) ------------------- */
typedef enum
{
	GPIO_EXTI_BIMETAL1 = 0,  /* PF0  - exti0_BIMETAL1       */
	GPIO_EXTI_BIMETAL2,      /* PF1  - exti1_BIMETAL2       */
	GPIO_EXTI_BIMETAL3,      /* PF2  - exti2_BIMETAL3       */
	GPIO_EXTI_BIMETAL4,      /* PF3  - exti3_BIMETAL4       */
	GPIO_EXTI_BIMETAL5,      /* PF4  - exti4_BIMETAL5       */
	GPIO_EXTI_LEAD_SW,       /* PF5  - exti5_LEAD_SW        */
	GPIO_EXTI_WATER_SEN1,    /* PF6  - exti6_WATER_SEN1     */
	GPIO_EXTI_WATER_SEN2,    /* PF7  - exti7_WATER_SEN2     */
	GPIO_EXTI_TIMER_OUT,     /* PA11 - exti11_TIMER_OUT     */
	GPIO_EXTI_M2_FGOT,       /* PF12 - exti12_M2_FGOT       */
	GPIO_EXTI_M2_nFAULT,     /* PF13 - exti13_M2_nFAULT     */
	GPIO_EXTI_M1_nFAULT,     /* PF14 - exti14_M1_nFAULT     */
	GPIO_EXTI_M1_FGOT,       /* PF15 - exti15_M1_FGOT       */
	GPIO_EXTI_COUNT          /* keep last */
} gpio_exti_t;

/* ---- Output control -------------------------------------------------- */
void    gpio_ctrl_on(gpio_out_t out);      /* drive pin HIGH              */
void    gpio_ctrl_off(gpio_out_t out);     /* drive pin LOW               */
void    gpio_ctrl_enable(gpio_out_t out);  /* alias of gpio_ctrl_on()     */
void    gpio_ctrl_disable(gpio_out_t out); /* alias of gpio_ctrl_off()    */
void    gpio_ctrl_set(gpio_out_t out, uint8_t on); /* on!=0 -> HIGH        */
void    gpio_ctrl_toggle(gpio_out_t out);  /* flip current level          */
uint8_t gpio_ctrl_is_on(gpio_out_t out);   /* read-back output latch: 0/1 */

/* ---- Input read ------------------------------------------------------- */
uint8_t gpio_ctrl_read(gpio_in_t in);      /* voltage level: 0 = LOW, 1 = HIGH */

/* ---- EXTI pins: level read + interrupt flags ------------------------- */
uint8_t gpio_ctrl_exti_read(gpio_exti_t e);       /* voltage level: 0 / 1  */

/* Set the matching pin's flag from an EXTI ISR context. Pass the GPIO_Pin
 * argument of HAL_GPIO_EXTI_Callback(); pins are unique across all EXTI
 * lines so the pin number alone identifies the source. No-op if unknown. */
void    gpio_ctrl_exti_dispatch(uint16_t gpio_pin);

uint8_t gpio_ctrl_exti_flag_get(gpio_exti_t e);   /* 1 = edge seen, else 0 */
void    gpio_ctrl_exti_flag_clear(gpio_exti_t e); /* clear one flag        */
void    gpio_ctrl_exti_flag_clear_all(void);      /* clear every flag      */

#ifdef __cplusplus
}
#endif

#endif /* INTERFACE_GPIO_CTRL_H_ */
