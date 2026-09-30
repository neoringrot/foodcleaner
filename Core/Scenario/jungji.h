#ifndef SCENARIO_JUNGJI_H_
#define SCENARIO_JUNGJI_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * jungji - "정지" 단일 처리 모듈. 어떤 경로로 정지가 요청되든(마개 HS3 정지
 * 위치, 마개 열림/이탈, 모드 전환, 시나리오 이상, 디버거) 여기 한 곳에서
 * 같은 순서로 전 액추에이터를 멈춘다.
 *
 * 근거: 기획서 4.5 "추가 투입 금지 / 비상 정지",
 *       Core/doc/사용자_시나리오_검토정리R3.md §2·§6(마개 위치 홀센서 5종).
 *
 * 호출 구조:
 *   Jungji_Request()  - 어느 태스크에서든 호출 가능(요청 래치만; 모터를 만지지
 *                       않는다). 중재자(mode_arbiter)와 시나리오가 사용.
 *   Jungji_Tick()     - StartMotorTask(1ms)에서 매 틱. 요청 소비 -> 실제 정지,
 *                       단락제동 홀드 해제, 고온 냉각팬 유지 관리.
 * 정지 실행 자체(Jungji_StopAll)는 MotorTask 문맥에서만 일어난다 = 모터 소유
 * 태스크와 동일하므로 시나리오 MotorTick과 경합하지 않는다.
 *
 * ------------------------------------------------------------------------
 * §1. 정지해야 하는 기능 목록 (전수). "즉시" = 전기적으로 곧바로 토크/출력이
 *     0이 되는 것, "감쇠" = 지령만 끊고 관성/기구로 서서히 멎는 것.
 *
 *  #  기능(부하)          드라이버/핀                  정지 방법            즉시?
 * --- ------------------- ---------------------------- -------------------- -----
 *  1  분쇄 BLDC M1        U11 DRV8306 / PE9,PE10,PE13  비상: nBRAKE 단락제동  즉시(제동)
 *                                                      일반: duty0+sleep     감쇠(코스트)
 *  2  교반 BLDC M2        U16 DRV8306 / PE11,PE12,PE14 상동                  상동
 *  3  배수문 WDoor        U5 DRV8871 / PC6,PC7,PE4     Brake -> Disable      즉시(웜기어 자기유지)
 *  4  배출문 TDoor        U7 DRV8871 / PA6,PA7,PE3     Brake -> Disable      즉시(상동)
 *  5  리프트 Lift         U6 DRV8871 / PB8,PB9,PB12    Brake -> Disable      즉시(상동)
 *  6  냄새관로 STEP1      Q30~33 / PD8..PD11           4상 무여자(Release)   즉시
 *  7  흡입제어 STEP2      Q38~41 / PD12..PD15          4상 무여자(Release)   즉시
 *  8  히터               HT-POWER PA12                 OFF                   즉시(발열체는 잔열)
 *  9  급수밸브           VALVE-DRY-IN PB13             OFF                   즉시
 * 10  급수 메인          WATER-ON PE2                  OFF                   즉시
 * 11  배수구세척 밸브    VALVE-DRAIN-CLN PB14          OFF                   즉시
 * 12  수증기팬          FAN-VAPOR PB15                 OFF                   즉시
 * 13  배기(내부정화)팬  FAN-EXHAUST PG2                OFF (고온이면 유지)   조건부
 * 14  BLDC 냉각팬       BLDC-FAN PG1                   OFF (고온이면 유지)   조건부
 * 15  스피커            SPK-EN PA3                     OFF                   즉시
 * 16  테스트벤치 전체   tb_*_enable 플래그들           전부 0                즉시
 *                      tb_grind_en / tb_stir_en       0 (엣지 재무장)       즉시
 * 17  시나리오 FSM      모음/동작/강음/배수            RequestStop + Abort    즉시
 *
 * §2. "즉시 정지 vs 서서히 정지" 판단 근거
 *   - BLDC 2대만이 유일하게 "서서히" 멎는 부하다. BldcCtrl_Stop()은 duty 0 +
 *     ENABLE LOW(sleep) = 코스트이므로 관성이 큰 분쇄날은 수 초간 계속 돈다.
 *     4.5 비상정지(마개 개방 = 손 접근 가능)에서는 이것으로 부족하므로
 *     JUNGJI_KIND_EMERGENCY는 nBRAKE 단락제동(BldcCtrl_BrakeStop)을 쓴다.
 *     단락제동은 게이트 드라이버가 깨어 있어야 유효하므로 JUNGJI_BRAKE_MS 동안
 *     유지한 뒤 Jungji_Tick()이 해제(BrakeRelease)한다.
 *   - 도어/리프트는 웜기어가 자기유지(비역구동)이므로 VM만 내리면 그 자리에
 *     멈춘다. 그래도 진행 중 관성을 없애려 Brake를 한 틱 걸고 Disable한다.
 *   - 스텝모터는 무여자 즉시 정지(홀딩토크 없음 = 발열 없음).
 *   - 히터는 출력만 즉시 끊기고 발열체 잔열은 남는다. 그래서 고온일 때는
 *     냉각팬(#13,#14)을 끄지 않고 JUNGJI_COOL_TEMP_D10 미만이 될 때까지 유지한다.
 *
 * §3. 정지 후 상태: 모든 시나리오 FSM은 IDLE. 재시작은 마개를 다시 동작/모음
 *     위치로 돌려야만(중재자 경유) 가능하다 = 정지 상태에서 저절로 재기동하지
 *     않는다.
 * ========================================================================== */

/* ---- 튜닝 상수 ----------------------------------------------------------- */
/* BLDC 단락제동 유지 시간. 이 시간 뒤 nBRAKE 해제 + 드라이버 sleep.
 * 짧으면 완전히 서기 전에 코스트로 넘어가고, 길면 권선/FET 발열이 늘어난다. */
#ifndef JUNGJI_BRAKE_MS
#define JUNGJI_BRAKE_MS         500U
#endif
/* 0 = 비상정지도 코스트(제동 미사용). 벤치에서 제동 동작을 빼고 볼 때만 0. */
#ifndef JUNGJI_BLDC_BRAKE_USE
#define JUNGJI_BLDC_BRAKE_USE   1
#endif
/* 정지 후 냉각팬을 계속 돌릴 온도 임계(0.1℃). DJ_TEMP_BLDCFAN_ON_D10와 동일. */
#ifndef JUNGJI_COOL_TEMP_D10
#define JUNGJI_COOL_TEMP_D10    800
#endif
/* 냉각 판정에 쓰는 NTC 채널(DJ_TEMP_CH와 동일하게 유지). */
#ifndef JUNGJI_TEMP_CH
#define JUNGJI_TEMP_CH          0U
#endif

/* ---- 정지 사유 ----------------------------------------------------------- */
typedef enum
{
	JUNGJI_SRC_NONE = 0,
	JUNGJI_SRC_HS_STOP,      /* HS3 = 마개가 "정지" 위치로 확정              */
	JUNGJI_SRC_HS_LOST,      /* HS1~5 전부 미인식 = 마개 열림/이탈           */
	JUNGJI_SRC_HS_INVALID,   /* 2채널 이상 동시 인식 = 홀 이상/과도구간      */
	JUNGJI_SRC_MODE_SWITCH,  /* 모드 전환 전 이전 모드 정리                  */
	JUNGJI_SRC_SCENARIO,     /* 시나리오 내부 이상(도어 타임아웃 등)         */
	JUNGJI_SRC_DEBUG,        /* 디버거 강제(g_jungji.dbg_stop_req)           */
	JUNGJI_SRC_APP           /* 앱 W 명령(protocol_r0 PROTO_ACT_STOP)        */
} JungjiSrc;

/* ---- 정지 등급 ----------------------------------------------------------- */
typedef enum
{
	JUNGJI_KIND_NORMAL = 0,  /* 지령 정지: BLDC는 코스트(감쇠) 허용          */
	JUNGJI_KIND_EMERGENCY    /* 비상 정지: BLDC 단락제동(즉시)               */
} JungjiKind;

/* ---- 관찰/제어 컨텍스트 (디버거 watch) ----------------------------------- */
typedef struct
{
	volatile uint8_t  req;          /* 1 = 미처리 정지 요청 있음              */
	volatile uint8_t  req_src;      /* JungjiSrc  (요청 시점)                 */
	volatile uint8_t  req_kind;     /* JungjiKind (요청 시점, 강한 쪽 우선)   */
	volatile uint8_t  dbg_stop_req; /* 디버거에서 1 -> 비상정지(1회성)        */

	volatile uint8_t  last_src;     /* 마지막으로 실행된 정지 사유            */
	volatile uint8_t  last_kind;    /* 마지막으로 실행된 정지 등급            */
	volatile uint32_t last_tick;    /* 마지막 정지 실행 tick                  */
	volatile uint16_t stop_count;   /* 누적 정지 실행 횟수                    */

	volatile uint8_t  braking;      /* 1 = BLDC 단락제동 유지 중              */
	uint32_t          brake_until;  /* 제동 해제 예정 tick                    */

	volatile uint8_t  cooling;      /* 1 = 잔열 냉각으로 팬 유지 중           */
	volatile int16_t  temp_d10;     /* 냉각 판정에 쓴 온도(0.1℃)              */
} JungjiCtx;

extern JungjiCtx g_jungji;

/* ---- API ----------------------------------------------------------------- */
void    Jungji_Init(void);

/* 정지 요청(래치만). 어느 태스크에서든 안전. 이미 대기 중인 요청이 있으면
 * 더 강한 등급(EMERGENCY)이 이긴다. */
void    Jungji_Request(JungjiSrc src, JungjiKind kind);

/* 1ms, StartMotorTask. 요청 소비 + 제동 홀드 해제 + 냉각팬 유지 관리. */
void    Jungji_Tick(uint32_t now_ms);

/* 즉시 전체 정지(요청 래치를 거치지 않음). MotorTask 문맥에서만 호출할 것. */
void    Jungji_StopAll(JungjiSrc src, JungjiKind kind, uint32_t now_ms);

uint8_t Jungji_IsBraking(void);   /* 1 = 단락제동 홀드 중(재기동 금지 구간)  */
uint8_t Jungji_IsCooling(void);   /* 1 = 잔열 냉각팬 유지 중                 */

/* ---- 개별 처리함수 (§1 목록과 1:1). Jungji_StopAll이 순서대로 호출한다.
 * 개별 검증/벤치에서 한 계통만 떼어 부를 수 있도록 공개해 둔다. -------------- */
void    Jungji_Bldc(JungjiKind kind, uint32_t now_ms); /* #1,#2  */
void    Jungji_Doors(void);                            /* #3,#4  */
void    Jungji_Lift(void);                             /* #5     */
void    Jungji_Steppers(void);                         /* #6,#7  */
void    Jungji_Heater(void);                           /* #8     */
void    Jungji_Valves(void);                           /* #9~#11 */
void    Jungji_Fans(uint8_t keep_cooling);             /* #12~#14*/
void    Jungji_Speaker(void);                          /* #15    */
void    Jungji_Testbench(void);                        /* #16    */
void    Jungji_Scenarios(void);                        /* #17    */

#ifdef __cplusplus
}
#endif

#endif /* SCENARIO_JUNGJI_H_ */
