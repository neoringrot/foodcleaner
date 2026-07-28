#ifndef DEVICES_LM4871_H_
#define DEVICES_LM4871_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* TI/National LM4871 mono bridged (BTL) 3W audio power amplifier with shutdown
 * (SNAS038, "3W Audio Power Amplifier with Shutdown Mode"), driven by the STM32
 * on-chip DAC. One instance on this board: U15.
 *
 * Schematic nets (sch/netlist final.NET) and datasheet mapping:
 *
 *   [SPK-DAC, net 00290]  MCU PA4 = DAC_OUT1  ->  Ci ---- Ri ---- U15.4 (-IN)
 *                         Ci = C65 0.39uF (AC-couple + high-pass),
 *                         Ri = R67 20k (input/gain resistor).
 *   [feedback, net 00205] U15.5 (VO1) -- Rf(R65 20k) --> U15.4 (-IN).
 *   [+IN/BYPASS, net 00116] U15.3 (+IN) tied to U15.2 (BYPASS) with
 *                         CB = C67 1uF to GND -> internal VDD/2 reference.
 *   [SPK-EN, net 00291]   MCU PA3 (o_EN_SPK) -> U15.1 (SD, shutdown),
 *                         R66 4.7k pull-up to 3V3-SPK.
 *   [outputs]             U15.5 VO1 / U15.8 VO2 -> speaker connector J17 (BTL).
 *   [power]               U15.6 VDD = 3V3-SPK (3.3V, LDO U14); U15.7 GND.
 *
 * Gain / bandwidth from the passives above (datasheet Eq. for BTL):
 *   First-stage gain  = Rf/Ri = 20k/20k = 1 (inverting).
 *   Differential gain = AVD = 2*(Rf/Ri) = 2  (+6 dB).
 *   Input high-pass   = 1/(2*pi*Ri*Ci) = 1/(2*pi*20k*0.39uF) ~= 20 Hz.
 * With VDD = 3.3V the maximum undistorted differential swing is a few hundred
 * mW into 8ohm -- this is a beeper/voice-prompt path, not a 3W speaker.
 *
 * SHUTDOWN polarity (datasheet, Shutdown Function): the amplifier is turned OFF
 * by a logic HIGH on SD and is ACTIVE (playing) with SD LOW. R66 pulls SD high,
 * so the part powers up muted until the MCU actively drives PA3 low. The pin is
 * therefore an ACTIVE-LOW enable even though its net is named o_EN_SPK / SPK-EN;
 * cfg.en_active_high captures this so the polarity is never guessed in code.
 * CB = 1uF (datasheet's recommended value) gives a slow output ramp on
 * enable/disable, which is the primary click-and-pop suppression here.
 *
 * DAC path: MX_DAC_Init() (dac.c) configures DAC_CHANNEL_1 on PA4 with output
 * buffer enabled and trigger = NONE, i.e. software-written (HAL_DAC_SetValue).
 * There is no timer/DMA waveform engine wired up, so tones are produced by this
 * driver bit-banging the DAC. LM4871_Beep() is therefore BLOCKING and timed off
 * the DWT cycle counter; call it from a task/context where a short busy-wait is
 * acceptable, or use LM4871_SetLevel() to feed your own sample stream.
 *
 * Idle behaviour: the DAC output is DC-blocked by Ci, so only its AC swing
 * reaches the speaker. The driver parks the DAC at cfg.dac_mid (mid-scale) when
 * not playing and leaves the amplifier disabled (SD high) to save the quiescent
 * supply current and avoid idle hiss.
 */

#define LM4871_DAC_MAX      4095u   /* 12-bit right-aligned full scale        */
#define LM4871_DAC_MID      2048u   /* mid-scale idle / waveform centre        */
#define LM4871_DEF_AMP      1200u   /* default beep peak amplitude around mid  */
#define LM4871_SETTLE_MS       5u   /* post-enable settle for CB ramp (pop)    */

/* Board/wiring settings -- everything the driver needs to talk to one LM4871.
 * Fill with LM4871_LoadDefaults() for U15 on this board, then override fields
 * as needed before LM4871_Init(). */
typedef struct
{
	DAC_HandleTypeDef *hdac;          /* DAC peripheral handle (&hdac)          */
	uint32_t           dac_channel;   /* DAC_CHANNEL_1 (PA4 = DAC_OUT1)         */
	uint32_t           dac_align;     /* DAC_ALIGN_12B_R                        */

	GPIO_TypeDef      *sd_port;       /* SD/enable GPIO port (o_EN_SPK_GPIO_Port)*/
	uint16_t           sd_pin;        /* SD/enable GPIO pin  (o_EN_SPK_Pin)     */
	uint8_t            en_active_high;/* 0 = SD active-low enable (LM4871 native)*/

	uint16_t           dac_mid;       /* idle/centre code (LM4871_DAC_MID)      */
	uint16_t           dac_amp;       /* default beep amplitude around dac_mid  */
	uint16_t           settle_ms;     /* pop-suppression settle after enabling  */
} LM4871_Config;

typedef struct
{
	LM4871_Config cfg;
	uint8_t       enabled;      /* current SD state: 1 = active/playing         */
	uint8_t       initialized;  /* set by LM4871_Init()                         */
} LM4871_HandleTypeDef;

/* Populate cfg with the fixed U15 wiring for this board (PA3 SD active-low,
 * DAC_CHANNEL_1 on PA4, mid-scale idle). Requires dac.h internally. */
void    LM4871_LoadDefaults(LM4871_Config *cfg);

/* Copy cfg into h, start the DAC channel, park it at mid-scale, init the DWT
 * delay, and leave the amplifier DISABLED (muted). Safe to call more than once. */
void    LM4871_Init(LM4871_HandleTypeDef *h, const LM4871_Config *cfg);

/* Un-mute (drive SD to the active level) and wait cfg.settle_ms for the CB ramp.
 * Disable() shuts the amplifier down (SD to the idle level) -- micropower. */
void    LM4871_Enable(LM4871_HandleTypeDef *h);
void    LM4871_Disable(LM4871_HandleTypeDef *h);
uint8_t LM4871_IsEnabled(const LM4871_HandleTypeDef *h);

/* Raw 12-bit DAC write, clamped to [0, LM4871_DAC_MAX]. Use to feed an external
 * sample stream; the amplifier must be enabled to hear it. */
void    LM4871_SetLevel(LM4871_HandleTypeDef *h, uint16_t code12);

/* Blocking square-wave beep at cfg.dac_amp: enables the amp if needed, plays for
 * duration_ms, returns the DAC to mid-scale, and restores the prior enable state.
 * freq_hz == 0 or duration_ms == 0 is treated as silence. */
void    LM4871_Beep(LM4871_HandleTypeDef *h, uint32_t freq_hz, uint32_t duration_ms);

/* As LM4871_Beep() with an explicit peak amplitude (clamped so mid +/- amp stays
 * in range) and control over whether the amplifier is left enabled afterwards. */
void    LM4871_BeepEx(LM4871_HandleTypeDef *h, uint32_t freq_hz,
                      uint32_t duration_ms, uint16_t amplitude, uint8_t leave_enabled);

/* ---- Board singleton (U15) -------------------------------------------------
 * The board carries exactly one LM4871, so a ready-made handle and a no-arg
 * init are provided for main.c / freertos.c, mirroring Lift_Init()/WiFi_Init().
 * LM4871_BoardInit() must run after MX_DAC_Init() and MX_GPIO_Init(). Tasks
 * then play audio with, e.g., LM4871_Beep(&lm4871, 2000, 100). */
extern LM4871_HandleTypeDef lm4871;
void    LM4871_BoardInit(void);

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_LM4871_H_ */
