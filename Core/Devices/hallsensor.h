#ifndef DEVICES_HALLSENSOR_H_
#define DEVICES_HALLSENSOR_H_

#include "main.h"
#include "tca9554.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * hallsensor - 8 Hall-effect sensors read through the U24 TCA9554A expander.
 *
 *   U24 (0x3B)  P0..P7 = HS1..HS8   -> Hall sensor inputs (GPIO input)
 *
 * R1 change: the expander (was U10, 7 sensors HS1..HS7 with P7 unused) now
 * carries 8 sensors HS1..HS8 (P7 = HS8), and its INT pin is wired to the MCU on
 * PF9 = HALL-INT1 (exti9_HALL_INT1, active-low open-drain). So besides the
 * periodic poll in StartDefaultTask, an INT falling edge can trigger an
 * immediate HallSensor_Update() (see the GPIO_EXTI_HALL_INT1 flag). HALL-INT2
 * (PF10) is routed only to the J15 external test header (U25/U26) and is not
 * populated on this board.
 *
 * Transport: I2C1 (hi2c1), shared with U8/U9 (see tca9554.h).
 *
 * I2C address: U24 straps A0=A1=HIGH, A2=LOW (A2 A1 A0 = 0 1 1) -> 0x3B on the
 * TCA9554A base 0x38. The address lives in tca9554.h as TCA9554_U10_ADDR.
 *
 * Levels: HS1..HS8 map to bit0..bit7 of every mask this module returns.
 * "Raw" masks are the pin voltages; "detected" masks are polarity-normalised
 * so a set bit always means "magnet present", regardless of the sensor's
 * electrical sense (see HALLSENSOR_ACTIVE_HIGH).
 * ---------------------------------------------------------------------- */

#define HALLSENSOR_COUNT   8U   /* HS1..HS8 on U24 P0..P7 */

/* Electrical sense of an *active* (magnet-present) Hall output on the U10 pin.
 * Open-drain Hall switches usually pull the line LOW when active, so the
 * default is active-low: the driver programs U24's polarity-inversion register
 * so a detected magnet still reads as logical 1. Set to 1 if the sensors drive
 * the pin HIGH when active. */
#ifndef HALLSENSOR_ACTIVE_HIGH
#define HALLSENSOR_ACTIVE_HIGH   0U
#endif

/* ---- Lifecycle ------------------------------------------------------- */
/* Configure U24 as all-inputs (HS1..8) and, for active-low
 * sensors, set the polarity-inversion register so reads are normalised.
 * Returns HAL_OK only if U24 initialises on the bus. */
HAL_StatusTypeDef HallSensor_Init(void);
uint8_t           HallSensor_IsPresent(void);   /* 1 = U24 ACKs on the bus */

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
uint8_t           HallSensor_Get(uint8_t idx);     /* idx 0..7 -> 0/1        */

/* INT-driven refresh: if the U24 INT flag (GPIO_EXTI_HALL_INT1, PF9) is set,
 * do one HallSensor_Update() and clear the flag. Returns 1 if a refresh ran.
 * Safe to call every loop; it is a no-op until the next INT edge. */
uint8_t           HallSensor_ServiceInt(void);

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_HALLSENSOR_H_ */
