#ifndef DEVICES_DISTANCE_H_
#define DEVICES_DISTANCE_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Sharp GP2Y0A41SK0F IR distance sensor -- 수거통 용량 감지 (bin fill level).
 *
 * Hardware (schematic net "Distance-SEN", U21 pin 34 / PA0-WKUP = ADC1_IN0):
 *   J20 : 3-pin sensor connector  pin1 = +5V, pin2 = GND, pin3 = Vo (analog)
 *   R51 : 1.0k in series from J20.3 (Vo) to the PA0 node
 *   R52 : 2.7k from PA0 node to GND      -> resistor divider, ratio 2.7/3.7
 *   C36 : 100pF from PA0 node to GND     -> anti-alias / noise filter
 *
 * The sensor is 5V powered and swings Vo ~0.4V (30cm) .. ~2.65V (4cm). The
 * divider scales that down to ~0.29V .. ~1.93V at PA0, so the firmware must
 * multiply the measured pin voltage by (R51+R52)/R52 to recover Vo.
 *
 * Datasheet (OP1300 8EN):
 *   - Measuring range        : 4 .. 30 cm
 *   - Vo @ L=30cm            : 0.40 V typ
 *   - Vo change 30cm -> 4cm  : 2.25 V typ  (=> Vo @ 4cm ~ 2.65 V)
 *   - Measuring cycle        : 16.5 ms
 *
 * Vo vs L is linear in 1/L. Fitting the two datasheet points gives
 *   L[cm] = 10.385 / (Vo - 0.0539)
 * which is what Distance_ReadCm() applies (clamped to 4..30 cm).
 *
 * ADC access goes through the shared adc_ctrl layer (Core/Interface), which
 * runs ADC1 in single-conversion mode and reads channel 0 on demand. Call
 * AdcCtrl_Init() once at startup before Distance_Read*(). See adc_ctrl.h.
 */

/* Bin geometry for fill-level (%). Distance measured from the sensor (mounted
 * at the top, looking down) to the top of the waste. Tune to the real bin:
 *   EMPTY -> largest distance (waste far away)   FULL -> smallest distance. */
#ifndef DISTANCE_BIN_EMPTY_MM
#define DISTANCE_BIN_EMPTY_MM   300u   /* empty bin: ~30 cm (sensor max range) */
#endif
#ifndef DISTANCE_BIN_FULL_MM
#define DISTANCE_BIN_FULL_MM     40u   /* full bin : ~4 cm  (sensor min range) */
#endif

#define DISTANCE_ERR_MM          0u    /* Distance_ReadMm() error sentinel      */

void     Distance_Init(void);              /* calibrate ADC1 (call once, post-MX) */
uint16_t Distance_ReadRaw(void);           /* median 12-bit ADC counts, 0 on error */
float    Distance_ReadVoltage(void);       /* recovered sensor Vo [V], <0 on error */
float    Distance_ReadCm(void);            /* distance [cm], clamped 4..30, <0 err  */
uint16_t Distance_ReadMm(void);            /* distance [mm], DISTANCE_ERR_MM on err */
uint8_t  Distance_ReadFillPercent(void);   /* bin fill 0..100 (%)                   */
uint8_t  Distance_FillFromMm(uint16_t mm); /* pure mm -> fill%, no ADC access        */

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_DISTANCE_H_ */
