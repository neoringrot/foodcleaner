#ifndef DEVICES_LIFT_H_
#define DEVICES_LIFT_H_

#include "drv8871.h"

#ifdef __cplusplus
extern "C" {
#endif

/* U6 -- Lift DRV8871 (schematic net "LIFT").
 *
 * R1 change: the lift's IN1/IN2 moved from PG3/PG4 (which had NO timer channel,
 * so the old build drove them as plain GPIO -- full-speed only) to PB8/PB9 =
 * TIM4_CH3/CH4. The lift is now a TIM4 PWM driver, identical in structure to the
 * TIM3 doors (U5 wdoor, U7 tdoor), so it shares the DRV8871 20 kHz PWM engine
 * and gains open-loop speed control.
 *
 *   IN1 = PB8  tim4_LIFT_IN1  = TIM4_CH3   (net LIFT-IN1)
 *   IN2 = PB9  tim4_LIFT_IN2  = TIM4_CH4   (net LIFT-IN2)
 *   EN  = PB12 o_MTR_DC_LIFT              (net MTR-DC-LIFT)
 *
 * DRV8871 IN1/IN2 truth table (datasheet Table 1):
 *   IN1  IN2  OUT1  OUT2  Mode
 *    0    0   Hi-Z  Hi-Z  Coast
 *    0    1    L     H    Reverse (Down)
 *    1    0    H     L    Forward (Up)
 *    1    1    L     L    Brake
 *
 * Enable: the DRV8871 has no logic enable pin; o_MTR_DC_LIFT gates the motor
 * supply onto VM through Q4 (NPN) -> Q2 (P-ch FET). It is active-high /
 * non-inverting: EN HIGH switches 24V onto VM, EN LOW removes it. Drive VM on
 * before commanding a direction and off after stopping.
 *
 * The public API keeps the original no-argument shape (full-speed Up/Down); it
 * now routes through the TIM4 PWM channels at 100% duty, behaviourally identical
 * to the old GPIO drive. Use Lift_UpSpeed()/Lift_DownSpeed() for open-loop speed
 * control (0-100%).
 *
 * Up/Down direction: **CONFIRMED on the bench 2026-09-20** -- the provisional
 * mapping turned out to be correct, so nothing was swapped.
 *   Lift_Up()   = DRV8871_Forward (IN1 PWM) -> 상승
 *   Lift_Down() = DRV8871_Reverse (IN2 PWM) -> 하강
 * 벤치 대응: tb_lift_reverse 0 = 상승 / 1 = 하강. 하강 중 HS8 인식으로 정지하는
 * 것까지 확인됐다(§0.18.5, HW미검증 1-25). */

/* ---- 구동 프로파일 (단일 출처) ---------------------------------------
 * 도어(wdoor.h / tdoor.h)와 같은 규약: 수치는 이 헤더 한 곳에만 두고, 벤치
 * `tb_lift_*` 변수와 (차후) 시나리오가 여기서 초기화된다. 값을 바꿀 때는 헤더를
 * 고친다 -- tb_* 는 런타임 실험용 복사본이다.
 *
 * ⚠️ 도어와 달리 아래 값은 **현장 실측이 아니라 잠정값**이다. 리프트 기능 자체가
 * 아직 정의되지 않았고(C071), 승강 행정시간도 측정된 적이 없다. 벤치에서
 * tb_lift 로 실측한 뒤 이 매크로를 갱신하고, 이 경고를 지울 것.
 *
 *   LIFT_DUTY          : 구동 duty [%]. 도어와 같은 웜기어 DC 라 80 %에서 출발.
 *   LIFT_UP_MAX_MS     : 상승 런 시간 상한 [ms]. 상단에는 리미트가 없으므로
 *                        이 상한이 유일한 종료 조건이다.
 *   LIFT_DOWN_MAX_MS   : 하강 런 시간 상한 [ms]. 정상 종료는 HS8 인식이고,
 *                        이 값은 HS8 미인식 대비 백스톱이다. */
#ifndef LIFT_DUTY
#define LIFT_DUTY            80U      /* TBD: C071 (리프트 기능 미정의) */
#endif
#ifndef LIFT_UP_MAX_MS
#define LIFT_UP_MAX_MS       5000U    /* TBD: C071 - 벤치 실측 전 잠정 */
#endif
#ifndef LIFT_DOWN_MAX_MS
#define LIFT_DOWN_MAX_MS     5000U    /* TBD: C071 - 벤치 실측 전 잠정 */
#endif

/* 하단 리미트: HS8 = U24 P7 (hallsensor.c 의 detected 마스크 bit7). 상단에는
 * 센서가 없다 -- 상승은 시간 상한으로만 끝난다. */
#ifndef LIFT_HS_BOTTOM_IDX
#define LIFT_HS_BOTTOM_IDX   7U
#endif

void Lift_Init(void);
void Lift_Enable(void);   /* switch 24V onto VM */
void Lift_Disable(void);  /* remove VM          */
void Lift_Up(void);       /* forward, full speed */
void Lift_Down(void);     /* reverse, full speed */
void Lift_UpSpeed(uint8_t duty_pct);   /* forward at 0-100% */
void Lift_DownSpeed(uint8_t duty_pct); /* reverse at 0-100% */
void Lift_Brake(void);
void Lift_Stop(void);     /* coast */


/* 하단 도달 판정(제품 디코드). HallSensor 캐시를 읽으므로 I2C 를 직접 건드리지
 * 않는다 -- 캐시는 U24 INT(PF9) 서비스와 100 ms 폴로 갱신된다. */
uint8_t Lift_AtBottom(void);

/* ---- 모니터링 read-only (protocol_r0 0x26 OUTPUT) ---------------------
 * drive = drv8871_drive_t (0 코스트 / 1 정회전 / 2 역회전 / 3 제동),
 * duty  = 마지막으로 지령한 PWM [%]. 피드백이 아니라 "마지막 지령"이다. */
uint8_t Lift_GetDrive(void);
uint8_t Lift_GetDuty(void);

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_LIFT_H_ */
