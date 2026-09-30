#ifndef SCENARIO_MODE_ARBITER_H_
#define SCENARIO_MODE_ARBITER_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * mode_arbiter - 마개 위치 홀센서(HS1~HS5)를 항상 읽어 동작 모드를 결정하는
 * 중재자. g_app_mode 의 유일한 소유자.
 *
 * 왜 필요한가(변경 전 문제):
 *   변경 전에는 Moeum_SenseTick()/Dongjak_SenseTick() 이 `g_app_mode` 로 게이팅
 *   되어 있었고, 시작 홀(모음/동작) 에지를 그 SenseTick 안에서만 읽었다. 즉
 *   "이미 그 모드에 들어가 있어야만 그 모드의 시작 홀을 읽는" 순환 구조라,
 *   기본값 APP_MODE_TESTBENCH 상태에서는 마개를 아무리 돌려도 시작되지 않았다
 *   (모드 변경 수단은 디버거뿐). 또 SenseTick가 멈춰 있는 동안 hs_prev 가
 *   얼어붙어, 모드 진입 시 정지 상태의 HIGH 레벨이 가짜 상승에지로 읽히거나
 *   (부팅 즉시 자동시작) 반대로 stale HIGH 때문에 에지를 놓치는 문제가 있었다.
 *
 * 변경 후 구조:
 *   HallSensor_Update() 는 원래부터 모드와 무관하게 defaultTask 에서 돌고 있다.
 *   그 스냅샷을 ModeArbiter_SenseTick() 이 항상 디코딩해 위치를 확정하고,
 *   위치가 바뀐 "확정 에지"에서만 모드 전환/정지를 요청한다. 시나리오 쪽의
 *   HS 시작 에지 검출은 꺼진다(MOEUM_HS_TRIGGER_INTERNAL / DJ_HS_TRIGGER_INTERNAL
 *   기본 0). 따라서 에지 기준선 문제도 사라진다.
 *
 * 마개 위치 매핑 (U24 TCA9554A P0..P4 = HS1..HS5, tb_hallsensor.h와 동일):
 *   HS1 P0 = 강음   -> APP_MODE_KANGEUM (미구현 스텁 kangeum.c)
 *   HS2 P1 = 동작   -> APP_MODE_DONGJAK
 *   HS3 P2 = 정지   -> 비상정지 후 APP_MODE_JUNGJI (2026-08-25 신설)
 *   HS4 P3 = 배수   -> APP_MODE_BAESU   (미구현 스텁 baesu.c)
 *   HS5 P4 = 모음   -> APP_MODE_MOEUM
 *   ★2026-08-25 변경: 마개 라벨 순서 재정의로 모음/동작이 맞바뀌었다
 *     (구: HS2=모음 / HS5=동작). 홀 채널 배선은 그대로이고 라벨만 바뀐 것이라
 *     펌웨어에서는 이 디코드 표와 *_HS_START_IDX 두 곳만 반대로 잡으면 된다.
 *   (HS6~HS8 = 교반원점/수거통/리프트하단. 마개 위치가 아니므로 여기서 무시.
 *    TODO: 이 3채널을 쓰는 기능은 별도 - 중재자 대상이 아님.)
 *
 * 정지 규칙(§ 기획서 4.5):
 *   - HS3 확정      -> Jungji_Request(HS_STOP, EMERGENCY) + APP_MODE_JUNGJI
 *   - 전 채널 미인식 -> 마개 열림/이탈 = Jungji_Request(HS_LOST, EMERGENCY).
 *                      모드는 바꾸지 않는다(마개가 없는 벤치에서 디버거 조작을
 *                      계속 쓸 수 있도록).
 *   - 2채널 이상 동시 -> 홀 이상/과도구간 = Jungji_Request(HS_INVALID, EMERGENCY).
 *
 * 순서 보장(중요):
 *   모드 전환은 반드시 "정지 -> 모드 변경 -> 시작" 순서다. 전환 판단은 100ms
 *   SenseTick 에서 하지만 실행은 1ms MotorTick 에서 하며, Jungji_StopAll() 이
 *   각 시나리오 Abort()(= start_req 클리어)를 부르므로 Start() 는 그 뒤에,
 *   그리고 BLDC 단락제동 홀드가 끝난 뒤에 호출한다.
 *
 * 부팅 시 동작:
 *   최초로 확정된 위치는 "기준선"으로만 삼고 어떤 액션도 하지 않는다. 전원을
 *   넣었을 때 마개가 이미 모음/동작 위치에 있다고 해서 저절로 돌기 시작하면
 *   안 되기 때문. 사용자가 마개를 한 번 움직여야 시작된다.
 *
 * 태스크 배치(freertos.c):
 *   ModeArbiter_SenseTick()  - StartDefaultTask(100ms). 모드와 무관하게 항상.
 *   ModeArbiter_MotorTick()  - StartMotorTask(1ms).   모드와 무관하게 항상.
 * ========================================================================== */

/* ---- 동작 모드 (g_app_mode) ----------------------------------------------
 * ★APP_MODE_JUNGJI(5) 신설 이유(2026-08-25): 그 전에는 HS3(정지)를 인식해도
 *   g_app_mode 가 APP_MODE_TESTBENCH 로 돌아갔다. 그러면 "마개가 정지 위치에
 *   있다"와 "부팅 직후 대기/벤치"가 같은 값 0 으로 보여, 디버거에서도 앱
 *   STATUS 에서도 정지 상태를 구분할 수 없었다. 별도 값을 주어 모음/동작 등
 *   실행 모드와도, 대기와도 구분되게 한다.
 *   실행 성격은 대기와 같다(시나리오 tick 없음, 벤치 폴링은 유지) - 아래
 *   AppMode_IsBenchIdle() 참조. 값만 다른 '보이는 정지 상태'다. */
typedef enum
{
	APP_MODE_TESTBENCH = 0,   /* 대기/벤치: TB_*_Poll + 키패드 BLDC          */
	APP_MODE_MOEUM     = 1,   /* 모음(헹굼 세척)   - HS5                     */
	APP_MODE_DONGJAK   = 2,   /* 동작(건조/분쇄/배출) - HS2                  */
	APP_MODE_KANGEUM   = 3,   /* 강음(보정)  - HS1, 미구현 스텁              */
	APP_MODE_BAESU     = 4,   /* 배수(설거지 보조) - HS4, 미구현 스텁        */
	APP_MODE_JUNGJI    = 5    /* 정지 - HS3. 비상정지 후 머무는 표시용 상태  */
} app_mode_t;

/* 시나리오가 액추에이터를 쥐고 있지 않은 모드인가(= 벤치가 공유 핀을 만져도
 * 되는가). 대기(TESTBENCH)와 정지(JUNGJI) 둘 다 해당한다.
 * TB_Heat_Poll / TB_Water_Poll 게이팅과 MotorTask 의 벤치 분기가 이 하나를
 * 쓰도록 모아 두었다 - JUNGJI 신설로 세 곳이 각자 == APP_MODE_TESTBENCH 를
 * 유지했다면 정지 상태에서 벤치 폴링만 조용히 죽었을 것이다. */
static inline uint8_t AppMode_IsBenchIdle(app_mode_t m)
{
	return (uint8_t)((m == APP_MODE_TESTBENCH) || (m == APP_MODE_JUNGJI));
}

/* 중재자만 쓴다(읽기는 자유). 디버거에서 수동으로 바꾸려면 g_modearb.dbg_disable
 * 을 1로 두어 중재자가 되돌리지 않도록 할 것. */
extern volatile app_mode_t g_app_mode;

/* ---- 마개 위치 ------------------------------------------------------------ */
typedef enum
{
	LID_POS_NONE = 0,   /* HS1~5 전부 미인식(마개 열림/이동 중)              */
	LID_POS_KANGEUM,    /* HS1                                               */
	LID_POS_MOEUM,      /* HS5                                               */
	LID_POS_JUNGJI,     /* HS3                                               */
	LID_POS_BAESU,      /* HS4                                               */
	LID_POS_DONGJAK,    /* HS2                                               */
	LID_POS_MULTI       /* 2채널 이상 동시(홀 이상/과도구간)                 */
} LidPos;

/* ---- 튜닝 상수 ----------------------------------------------------------- */
/* 위치 확정에 필요한 연속 동일 샘플 수(100ms 주기). 마개가 이웃 위치를 스쳐
 * 지나갈 때 그 위치가 확정되어 엉뚱한 모드가 뜨는 것을 막는다.
 * ★2026-09-21: 3(300ms) -> 10(1.0s). 마개를 천천히 돌리면 중간 위치(정지·배수)에
 *   0.3초만 머물러도 확정되어 엉뚱한 모드/정지가 떴다. 늘리는 대상은 **실제 위치**
 *   (강음/동작/정지/배수/모음)뿐이다 - 시작/모드 전환이 약 1초 늦어진다.
 *   이탈(NONE)·이중인식(MULTI)은 4.5 "추가 투입 금지" 안전정지라 늦추면 안 되므로
 *   MODEARB_LOST_CONFIRM_SAMPLES 로 따로 둔다(종전 3 = 300ms 유지). */
#ifndef MODEARB_CONFIRM_SAMPLES
#define MODEARB_CONFIRM_SAMPLES  10U     /* ≈1.0s : 실제 위치 확정            */
#endif
#ifndef MODEARB_LOST_CONFIRM_SAMPLES
#define MODEARB_LOST_CONFIRM_SAMPLES 3U  /* ≈300ms: NONE/MULTI(안전정지) 확정 */
#endif
/* 마개 위치로 쓰는 홀 채널 마스크(HS1~HS5 = bit0..bit4). */
#ifndef MODEARB_HS_MASK
#define MODEARB_HS_MASK          0x1FU
#endif

/* ---- 관찰/제어 컨텍스트 (디버거 watch) ----------------------------------- */
typedef struct
{
	volatile uint8_t  raw_mask;     /* 이번 샘플의 HS1~5 마스크(bit0..4)      */
	volatile uint8_t  pos_raw;      /* LidPos: 디바운스 전 디코딩             */
	volatile uint8_t  pos_stable;   /* LidPos: 확정된 위치                    */
	uint8_t           cand;         /* 디바운스 후보                          */
	uint8_t           cand_cnt;     /* 후보 연속 카운트                       */
	volatile uint8_t  primed;       /* 1 = 부팅 후 첫 확정 완료(기준선 잡힘)  */

	volatile uint8_t  pend_mode;    /* app_mode_t: 적용 대기 모드             */
	volatile uint8_t  pend_valid;   /* 1 = 모드 전환 요청 대기                */
	uint8_t           pend_start;   /* 1 = 전환 후 Start() 필요               */
	volatile uint8_t  start_wait;   /* 1 = 제동 해제 대기 중(곧 Start)        */
	/* ★DJ_HEAT 직행 요청(헹굼 생략). 1=도어 닫힘 대기부터 / 2=도어 대기 생략.
	 * start_wait 와 같은 지점에서 소비한다 - 모드 전환 정리(Jungji_StopAll ->
	 * Dongjak_Abort)가 g_dongjak.dbg_enter_heat 를 지우므로, 요청을 시나리오
	 * 변수에 미리 세워 두면 전환 과정에서 사라진다. 여기에 담아 전환·제동이
	 * 끝난 뒤에 세운다. */
	uint8_t           pend_heat;
	volatile uint8_t  heat_wait;
	/* ★식힘(DJ_COOLDOWN, cool_phase=1) 직행 요청. pend_heat 와 같은 규칙·같은
	 * 소비 지점이다. 둘은 배타적이라 하나를 세우면 다른 하나를 지운다. */
	uint8_t           pend_cool;
	volatile uint8_t  cool_wait;
	/* ★자가세척(DJ_SELFCLEAN) 직행 요청 [2026-09-22 앱 동기화]. pend_heat·pend_cool 과 같은 규칙·
	 * 같은 소비 지점이며 셋은 서로 배타적이다. */
	uint8_t           pend_selfclean;
	volatile uint8_t  selfclean_wait;

	volatile uint8_t  dbg_disable;  /* 1 = 중재자 무력화(디버거 수동 모드)    */
	volatile uint8_t  dbg_pos_force;/* LidPos 강제 주입(0=미사용, 홀 없이 시험) */
	volatile uint16_t transitions;  /* 확정 위치가 바뀐 횟수                  */
	volatile uint16_t mode_changes; /* g_app_mode 가 실제로 바뀐 횟수         */
} ModeArbCtx;

extern ModeArbCtx g_modearb;

/* ---- API ----------------------------------------------------------------- */
void   ModeArbiter_Init(void);
/* 동작 시나리오를 헹굼 없이 DJ_HEAT 부터 시작한다(앱 명령 / 벤치 전용).
 *   skip_door=0 : 배수문 닫힘 + W-HALL-CLOSE 대기부터(사양에 가깝다)
 *   skip_door=1 : 도어 대기까지 생략하고 '가열중'으로 즉시 진입
 * 모드가 동작이 아니면 먼저 전환한다. 성공 1 / 거절 0(중재자 무력화 시). */
uint8_t ModeArbiter_RequestDongjakHeat(uint8_t skip_door);
/* 동작 시나리오를 헹굼·건조 없이 식힘 교반(80℃ 미만)부터 시작한다(앱/벤치 전용).
 * DJ_HEAT 직행과 같은 래치 규칙을 쓴다 - 모드 전환 정리(Jungji_StopAll ->
 * Dongjak_Abort)가 g_dongjak.dbg_enter_cool 을 지우므로, 요청은 여기 pend_cool 에
 * 담아 두었다가 전환·제동이 끝난 뒤에 세운다. 성공 1 / 거절 0(중재자 무력화 시). */
uint8_t ModeArbiter_RequestDongjakCool(void);
/* 동작 시나리오를 처리·배출 없이 **자가세척(DJ_SELFCLEAN)** 부터 시작한다(앱 '동작 (자가세척부터)').
 * 같은 래치 규칙 — 전환·제동이 끝난 뒤 g_dongjak.dbg_enter_selfclean 을 세운다. 성공 1 / 거절 0. */
uint8_t ModeArbiter_RequestDongjakSelfclean(void);
void   ModeArbiter_SenseTick(void);            /* 100ms, StartDefaultTask     */
void   ModeArbiter_MotorTick(uint32_t now_ms); /* 1ms,  StartMotorTask        */
LidPos ModeArbiter_GetPos(void);                /* 확정 위치                  */
uint8_t ModeArbiter_PosIsRunnable(LidPos p);    /* 1 = 시나리오가 붙는 위치   */

/* ---- 외부(앱) 모드 요청 --------------------------------------------------
 * 동작 트리거는 두 가지다:
 *   (1) 장치 자체 - 마개 홀센서 확정 에지 (modearb_on_pos_change, 위 §)
 *   (2) 앱 - R0 프로토콜 W 명령(protocol_r0.h PROTO_CMD_CONTROL) -> 이 함수
 * 두 경로 모두 여기 pend_* 래치로 모이고, 실제 정지->전환->시작은
 * ModeArbiter_MotorTick() 이 수행한다. 따라서 앱 경로도 "이전 모드 정리 후 전환,
 * 제동 홀드 해제 뒤 시작" 순서 보장을 그대로 물려받는다.
 *
 * 어느 태스크에서 불러도 안전하다(래치만 세운다. 모터를 만지지 않는다).
 * 반환 1 = 접수, 0 = 거절(미구현 모드 / 중재자 무력화 중).
 *
 * ⚠ 마개 위치와의 관계: 중재자는 "확정 위치가 바뀐 순간"에만 개입하므로, 마개가
 *   가만히 있으면 앱이 지정한 모드가 유지된다(벤치에서 마개 없이 시작 가능).
 *   다만 실행 중 마개가 움직여 위치가 바뀌면 그때는 중재자 규칙이 이긴다
 *   (마개 이탈 -> HS_LOST 비상정지). 안전상 의도된 우선순위다.
 *
 * ⚠ 강음(APP_MODE_KANGEUM)/배수(APP_MODE_BAESU)는 시나리오가 미구현 스텁이므로
 *   allow_stub=0 이면 거절한다. 앱 버튼은 배치해 두고 동작하지 않게 하려면
 *   여기서 막는 것이 맞다(스텁이 조용히 "시작됨"으로 보이면 오해를 준다). */
uint8_t ModeArbiter_RequestMode(app_mode_t mode, uint8_t start, uint8_t allow_stub);

#ifdef __cplusplus
}
#endif

#endif /* SCENARIO_MODE_ARBITER_H_ */
