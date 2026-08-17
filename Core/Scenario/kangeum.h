#ifndef SCENARIO_KANGEUM_H_
#define SCENARIO_KANGEUM_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * kangeum - "강음"(찰음/강한 음식물 보정) 시나리오. ★현재 미구현 스텁★
 *
 * 트리거: 마개 위치 홀센서 HS1 (U24 P0). 중재자(mode_arbiter.c)가 HS1 확정 시
 *         APP_MODE_KANGEUM으로 전환하고 Kangeum_Start()를 호출한다.
 *
 * 근거 / 미확정 사유:
 *   - 사용자_시나리오_검토정리.md §12 "찰음/강한음식 모드": 밥 과다·떡·게껍데기
 *     등 강한 음식 대응 **별도 보정 프로그램**. 문서에 "강음(반복 2회)" 언급만
 *     있고 세부 파라미터(분쇄 RPM/토크, 반복 횟수, 온도 프로파일)는 미정.
 *   - 구현현황_및_미구현_점검.md: "1차 기본 프로그램 테스트 후 업그레이드 예정".
 * 따라서 지금은 **연결부만** 만들어 두고 액추에이터는 전혀 구동하지 않는다.
 * 잘못 동작하는 것보다 아무것도 안 하는 편이 안전하기 때문이다.
 *
 * TODO(강음): 파라미터 확정 후 dongjak.c 와 같은 구조(SenseTick=센서,
 *             MotorTick=상태머신+구동)로 채운다. 동작 시나리오의 분쇄 구간을
 *             재사용하고 RPM/반복만 바꾸는 형태가 유력.
 *
 * 태스크 배치(freertos.c) - 다른 시나리오와 동일:
 *   Kangeum_SenseTick()  - StartDefaultTask(100ms), 센서만
 *   Kangeum_MotorTick()  - StartMotorTask(1ms), 상태머신 + 구동
 * ========================================================================== */

/* ---- 트리거 채널 ---------------------------------------------------------- */
#ifndef KANGEUM_HS_IDX
#define KANGEUM_HS_IDX          0U      /* 강음 = HS1 (0-based, U24 P0)       */
#endif

typedef enum
{
	KANGEUM_IDLE = 0,   /* 대기                                              */
	KANGEUM_TODO        /* 선택됨 but 미구현: 아무 액추에이터도 구동 안 함    */
} KangeumState;

typedef struct
{
	volatile uint8_t  state;        /* KangeumState (RO)                     */
	uint32_t          state_since;
	volatile uint8_t  start_req;    /* 중재자/디버거가 세팅, MotorTick가 소비 */
	volatile uint8_t  dbg_force_start; /* 벤치 강제 시작(1회성)              */
	volatile uint8_t  abort_req;
	volatile uint16_t select_count; /* 선택된 횟수(미구현 관찰용)            */
} KangeumCtx;

extern KangeumCtx g_kangeum;

/* ---- API (다른 시나리오와 동일 시그니처; 내부는 TODO) --------------------- */
void         Kangeum_Init(void);
void         Kangeum_Start(void);                /* 시작 요청(IDLE에서만)     */
void         Kangeum_Abort(void);                /* 즉시 IDLE                 */
void         Kangeum_RequestStop(void);          /* 4.5 정지 요청             */
void         Kangeum_SenseTick(void);            /* 100ms, StartDefaultTask   */
void         Kangeum_MotorTick(uint32_t now_ms); /* 1ms,  StartMotorTask      */
KangeumState Kangeum_GetState(void);
uint8_t      Kangeum_IsBusy(void);

#ifdef __cplusplus
}
#endif

#endif /* SCENARIO_KANGEUM_H_ */
