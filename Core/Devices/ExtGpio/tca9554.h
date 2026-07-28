#ifndef DEVICES_TCA9554_H_
#define DEVICES_TCA9554_H_

#include "main.h"
#include "i2c.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * tca9554 - TI TCA9554 / TCA9554A 8-bit I2C & SMBus I/O expander driver.
 *
 * Transport (this board): I2C1 (hi2c1), PB6 = I2C1_SCL, PB7 = I2C1_SDA, with
 * 4.7k pull-ups (R53/R54). Three expanders share the bus: U8, U9, U10.
 *   U8  P0..P7 -> DIS-LED1..8   (outputs, front-panel LEDs)
 *   U9  P0..P7 -> DIS-SW1..8    (inputs,  membrane keypad switches)
 *   U10 P0..P7 -> other I/O     (not handled here)
 *
 * IMPORTANT - part variant / address:
 *   The parts fitted are TCA9554A (marking "TCA9554APWR" in the netlist), whose
 *   FIXED address field is 0111b, so the 7-bit address is:
 *        0 1 1 1 A2 A1 A0   ->  base 0x38 .. 0x3F
 *   The plain TCA9554 (the part described in doc/../tca9554.pdf) instead uses
 *   0100b -> base 0x20 .. 0x27. If a plain TCA9554 is ever fitted, change
 *   TCA9554_ADDR_BASE to 0x20.
 *
 * IMPORTANT - address pins on the schematic:
 *   A0/A1/A2 of U8, U9 and U10 are ALL strapped to DGND in the current
 *   netlist, i.e. hardware address 000 for every device -> they would all
 *   answer at 0x38 and collide. Per the design intent (and the request), U9 is
 *   treated as if its A0 pin were pulled HIGH, giving U9 = 0x39. U8 keeps the
 *   000 strap = 0x38. Fix the strapping (or solder bridges) on the PCB to make
 *   this match; the firmware addresses below assume the corrected wiring.
 *
 * INT pin (pin 13, open-drain active-low): NOT routed to the MCU on this board
 * for U8/U9/U10. A hardware interrupt from the expander is therefore not
 * possible - the keypad is serviced by periodic polling (see membrane.h).
 *
 * Register model (command byte selects the register; auto-increment is not
 * used - each access carries its own command byte):
 *   REG 0 Input Port      (RO)  - live pin levels
 *   REG 1 Output Port     (RW)  - output latch (POR = 0xFF)
 *   REG 2 Polarity Invert (RW)  - invert INPUT readings per-bit (POR = 0x00)
 *   REG 3 Configuration   (RW)  - direction: 1 = input(Hi-Z), 0 = output
 *                                 (POR = 0xFF, all inputs)
 * ---------------------------------------------------------------------- */

/* ---- Registers (command byte) --------------------------------------- */
#define TCA9554_REG_INPUT     0x00U
#define TCA9554_REG_OUTPUT    0x01U
#define TCA9554_REG_POLARITY  0x02U
#define TCA9554_REG_CONFIG    0x03U

/* ---- Slave addresses (7-bit; HAL calls shift them left by 1) --------- */
#define TCA9554_ADDR_BASE     0x38U   /* TCA9554A fixed field 0111b (plain part: 0x20) */

#define TCA9554_ADDR(a2, a1, a0) \
	((uint8_t)(TCA9554_ADDR_BASE | (((a2) & 1U) << 2) | (((a1) & 1U) << 1) | ((a0) & 1U)))

#define TCA9554_U8_ADDR   TCA9554_ADDR(0, 0, 0)   /* 0x38 - DIS-LED driver (outputs) */
#define TCA9554_U9_ADDR   TCA9554_ADDR(0, 0, 1)   /* 0x39 - DIS-SW keypad (inputs), A0 assumed HIGH */
#define TCA9554_U10_ADDR  TCA9554_ADDR(0, 1, 0)   /* 0x3A - reserved for U10          */

/* ---- Direction masks for TCA9554_Init(config) ----------------------- */
#define TCA9554_ALL_OUTPUTS   0x00U   /* every pin driven      */
#define TCA9554_ALL_INPUTS    0xFFU   /* every pin Hi-Z input  */

/* Per-device state. out_shadow / cfg_shadow mirror the write-only latches so
 * single-pin updates are read-modify-write without a bus read-back. */
typedef struct
{
	I2C_HandleTypeDef *hi2c;       /* owning I2C bus (hi2c1 on this board) */
	uint8_t            addr7;      /* 7-bit slave address                 */
	uint8_t            out_shadow; /* cached Output Port register          */
	uint8_t            cfg_shadow; /* cached Configuration register        */
} tca9554_t;

/* ---- Lifecycle ------------------------------------------------------- */
/* Push out_init to the output latch, clear polarity inversion, then apply the
 * direction mask (config). Order avoids glitching outputs through the 0xFF POR
 * default. Returns HAL_OK only if every bus transfer succeeds. */
HAL_StatusTypeDef TCA9554_Init(tca9554_t *dev, I2C_HandleTypeDef *hi2c,
                               uint8_t addr7, uint8_t config, uint8_t out_init);
uint8_t           TCA9554_IsPresent(tca9554_t *dev);   /* 1 = ACKs on the bus */

/* ---- Direction / polarity ------------------------------------------- */
HAL_StatusTypeDef TCA9554_SetConfig(tca9554_t *dev, uint8_t config);     /* dir mask */
HAL_StatusTypeDef TCA9554_SetPolarity(tca9554_t *dev, uint8_t polarity); /* invert inputs */

/* ---- Output ---------------------------------------------------------- */
HAL_StatusTypeDef TCA9554_WriteOutput(tca9554_t *dev, uint8_t val); /* whole port      */
uint8_t           TCA9554_GetOutput(const tca9554_t *dev);          /* cached latch    */
HAL_StatusTypeDef TCA9554_WritePin(tca9554_t *dev, uint8_t pin, uint8_t level); /* pin 0..7 */
HAL_StatusTypeDef TCA9554_TogglePin(tca9554_t *dev, uint8_t pin);   /* flip one pin    */

/* ---- Input ----------------------------------------------------------- */
HAL_StatusTypeDef TCA9554_ReadInput(tca9554_t *dev, uint8_t *val);  /* live pin levels */

/* ---- Raw register access (rarely needed) ---------------------------- */
HAL_StatusTypeDef TCA9554_WriteReg(tca9554_t *dev, uint8_t reg, uint8_t val);
HAL_StatusTypeDef TCA9554_ReadReg(tca9554_t *dev, uint8_t reg, uint8_t *val);

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_TCA9554_H_ */
