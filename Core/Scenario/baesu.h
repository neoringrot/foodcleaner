#ifndef SCENARIO_BAESU_H_
#define SCENARIO_BAESU_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * baesu - "배수"(설거지 보조) 시나리오. ★현재 미구현 스텁★
 *
 * 트리거: 마개 위치 홀센서 HS4 (U24 P3). 중재자(mode_arbiter.c)가 HS4 확정 시
 *         APP_MODE_BAESU로 전환하고 Baesu_Start()를 호출한다.
 *
 * 근거 / 미확정 사유:
 *   - 사용자_시나리오_검토정리.md §2·§6: 조작부 마개 홀센서 5종 중 하나가 "배수",
 *     7개 선택 모드(정지·모음·동작·찰음/강음·배수·자가세척·살균세척)의 하나.
 *   - 구현현황_및_미구현_점검.md §부가 시나리오: "배수(설거지 보조)" 미구현.
 *   - 배수 상태의 유지 조건(배수문을 연 채 대기하는 시간, 급수 동반 여부,
 *     교반 동반 여부)이 문서에 확정되어 있지 않다.
 *
 * TODO(배수): 확정 시 최소 구성은 "배수문(WDoor) 개방 + 배수구세척 밸브
 *             (VALVE-DRAIN-CLN, PB14) 개방 상태 유지, 마개가 배수 위치를 벗어날
 *             때까지". 개방 유지형이므로 종료 조건이 시간이 아니라 마개 위치라는
 *             점이 다른 시나리오와 다르다 - 중재자가 위치 이탈을 이미 잡아주므로
 *             Baesu_Abort()에서 문을 닫을지(닫음) 열어둘지(유지) 정책을 정해야 함.
 *
 * 태스크 배치(freertos.c) - 다른 시나리오와 동일:
 *   Baesu_SenseTick()  - StartDefaultTask(100ms), 센서만
 *   Baesu_MotorTick()  - StartMotorTask(1ms), 상태머신 + 구동
 * ========================================================================== */

/* ---- 트리거 채널 ---------------------------------------------------------- */
#ifndef BAESU_HS_IDX
#define BAESU_HS_IDX            3U      /* 배수 = HS4 (0-based, U24 P3)       */
#endif

typedef enum
{
	BAESU_IDLE = 0,   /* 대기                                                */
	BAESU_TODO        /* 선택됨 but 미구현: 아무 액추에이터도 구동 안 함      */
} BaesuState;

typedef struct
{
	volatile uint8_t  state;        /* BaesuState (RO)                       */
	uint32_t          state_since;
	volatile uint8_t  start_req;    /* 중재자/디버거가 세팅, MotorTick가 소비 */
	volatile uint8_t  dbg_force_start; /* 벤치 강제 시작(1회성)              */
	volatile uint8_t  abort_req;
	volatile uint16_t select_count; /* 선택된 횟수(미구현 관찰용)            */
} BaesuCtx;

extern BaesuCtx g_baesu;

/* ---- API (다른 시나리오와 동일 시그니처; 내부는 TODO) --------------------- */
void       Baesu_Init(void);
void       Baesu_Start(void);                /* 시작 요청(IDLE에서만)         */
void       Baesu_Abort(void);                /* 즉시 IDLE                     */
void       Baesu_RequestStop(void);          /* 4.5 정지 요청                 */
void       Baesu_SenseTick(void);            /* 100ms, StartDefaultTask       */
void       Baesu_MotorTick(uint32_t now_ms); /* 1ms,  StartMotorTask          */
BaesuState Baesu_GetState(void);
uint8_t    Baesu_IsBusy(void);

#ifdef __cplusplus
}
#endif

#endif /* SCENARIO_BAESU_H_ */
