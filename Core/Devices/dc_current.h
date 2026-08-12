#ifndef DEVICES_DC_CURRENT_H_
#define DEVICES_DC_CURRENT_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * dc_current - 3 DRV8871 DC-motor current-sense channels (R1).
 *
 * Sense chain (identical on all 3, confirmed from R1 net-list):
 *   motor GND -> 10 mohm shunt (WSHP2818R0100) -> DGND
 *   shunt drop -> LM321LV non-inverting amp, gain = 1 + 1k/1k = 2
 *   amp out -> 470 ohm + 1 nF RC -> ADC1 input
 *
 *   ch          ADC pin        op-amp  DRV8871  motor
 *   DC_CURR_LIFT  PC5 ADC1_IN15  U4      U6       Lift        (net LIFT-*)
 *   DC_CURR_TDOOR PB0 ADC1_IN8   U5      U9       Trash door  (net T-DOOR-*)
 *   DC_CURR_WDOOR PB1 ADC1_IN9   U25     U11      Water door  (net W-DOOR-*)
 *
 * Scale: Vadc = I * 10 mohm * 2 = 20 mV/A. With Vref 3.3 V / 12-bit that is
 * ~24.8 codes/A (~40.3 mA/code). Small vs. full scale, but enough for stall
 * detection.
 *
 * Motor JK30ZYT-5840-577 (24 VDC, ratio 577): no-load 0.2 A, rated 1 A,
 * stall 4.5 A. The overcurrent trip below defaults to 3.0 A -- above rated +
 * inrush, below hard stall -- for jam detection (scenario 사용자_시나리오 §5.2).
 *
 * Reads go through adc_ctrl (single-conversion); AdcCtrl_Init() must have run
 * (StartDefaultTask does this). adc_ctrl is NOT reentrant - read from one task.
 * ---------------------------------------------------------------------- */

typedef enum
{
	DC_CURR_LIFT  = 0,   /* PC5  U4->U6  lift        */
	DC_CURR_TDOOR = 1,   /* PB0  U5->U9  trash door  */
	DC_CURR_WDOOR = 2,   /* PB1  U25->U11 water door */
	DC_CURRENT_COUNT
} DcCurrent_Ch_t;

/* Sense-chain constants (edit here if the shunt or gain changes). */
#define DC_CURRENT_SHUNT_MOHM   10U    /* WSHP2818R0100 = 10 mohm            */
#define DC_CURRENT_GAIN         2U     /* LM321LV non-inverting 1 + 1k/1k    */
#define DC_CURRENT_VREF_MV      3300U  /* ADC reference                      */

/* Motor JK30ZYT-5840-577 reference points [mA]. */
#define DC_CURRENT_RATED_MA     1000U  /* rated load current                */
#define DC_CURRENT_STALL_MA     4500U  /* stall current                     */

/* Overcurrent / jam trip point [mA]. Default 3 A (between rated and stall). */
#ifndef DC_CURRENT_TRIP_MA
#define DC_CURRENT_TRIP_MA      3000U
#endif

/* Median samples per read (passed to AdcCtrl_ReadMedian, clamped 1..15). */
#ifndef DC_CURRENT_SAMPLES
#define DC_CURRENT_SAMPLES      5U
#endif

/* One median-filtered read of channel ch. raw 12-bit code (0 if ch invalid). */
uint16_t DcCurrent_ReadRaw(DcCurrent_Ch_t ch);

/* Read expressed as motor current in milliamps. */
uint16_t DcCurrent_ReadMilliAmp(DcCurrent_Ch_t ch);

/* 1 if channel ch is at/above DC_CURRENT_TRIP_MA on this read, else 0. */
uint8_t  DcCurrent_IsOver(DcCurrent_Ch_t ch);

/* Read all channels into out[DC_CURRENT_COUNT] (mA) and return a bitmask
 * (bit ch) of channels over the trip point. out may be NULL. */
uint8_t  DcCurrent_UpdateAll(uint16_t out_ma[DC_CURRENT_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_DC_CURRENT_H_ */
