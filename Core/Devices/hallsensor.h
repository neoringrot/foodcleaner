#ifndef DEVICES_HALLSENSOR_H_
#define DEVICES_HALLSENSOR_H_

#include "main.h"
#include "tca9554.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * hallsensor - 7 Hall-effect sensors read through the U10 TCA9554A expander.
 *
 *   U10 (0x3A)  P0..P6 = HS1..HS7   -> Hall sensor inputs (GPIO input)
 *               P7               = unused (not routed in the netlist)
 *
 * Transport: I2C1 (hi2c1), shared with U8/U9 (see tca9554.h). U10's INT pin is
 * likewise not wired to the MCU, so the sensors are monitored by periodic
 * polling (StartDefaultTask), not a hardware interrupt.
 *
 * I2C address: per request, U10 is addressed as if its A1 pin were strapped
 * HIGH (A2 A1 A0 = 0 1 0), giving 0x3A on the TCA9554A base 0x38. This ignores
 * the current netlist strap (all address pins to DGND) - fix the PCB strap to
 * match. The address lives in tca9554.h as TCA9554_U10_ADDR.
 *
 * Levels: HS1..HS7 map to bit0..bit6 of every mask this module returns.
 * "Raw" masks are the pin voltages; "detected" masks are polarity-normalised
 * so a set bit always means "magnet present", regardless of the sensor's
 * electrical sense (see HALLSENSOR_ACTIVE_HIGH).
 * ---------------------------------------------------------------------- */

#define HALLSENSOR_COUNT   7U   /* HS1..HS7 on U10 P0..P6 */

/* Electrical sense of an *active* (magnet-present) Hall output on the U10 pin.
 * Open-drain Hall switches usually pull the line LOW when active, so the
 * default is active-low: the driver programs U10's polarity-inversion register
 * so a detected magnet still reads as logical 1. Set to 1 if the sensors drive
 * the pin HIGH when active. */
#ifndef HALLSENSOR_ACTIVE_HIGH
#define HALLSENSOR_ACTIVE_HIGH   0U
#endif

/* ---- Lifecycle ------------------------------------------------------- */
/* Configure U10 as all-inputs (HS1..7 + the unused P7) and, for active-low
 * sensors, set the polarity-inversion register so reads are normalised.
 * Returns HAL_OK only if U10 initialises on the bus. */
HAL_StatusTypeDef HallSensor_Init(void);
uint8_t           HallSensor_IsPresent(void);   /* 1 = U10 ACKs on the bus */

/* ---- Read ------------------------------------------------------------ */
/* Detected mask (bit i = HSi+1 magnet present), polarity already applied.
 * Returns HAL_OK on a clean bus read; *mask untouched on error. */
HAL_StatusTypeDef HallSensor_Read(uint8_t *mask);
HAL_StatusTypeDef HallSensor_ReadRaw(uint8_t *mask);  /* raw U10 input port */

/* ---- Cached accessors (updated by the monitor routine) --------------- */
/* HallSensor_Update() does one bus read and latches the result; the getters
 * below then return that snapshot without touching the bus. Call Update() from
 * the polling loop; call the getters from anywhere. */
HAL_StatusTypeDef HallSensor_Update(void);
uint8_t           HallSensor_GetMask(void);        /* last detected mask     */
uint8_t           HallSensor_Get(uint8_t idx);     /* idx 0..6 -> 0/1        */

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_HALLSENSOR_H_ */
