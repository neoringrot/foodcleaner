#ifndef TESTBENCH_TB_APP_H_
#define TESTBENCH_TB_APP_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if ENABLE_TESTBENCH_APP

/* ==========================================================================
 * tb_app - 테스트벤치 원격검증 어댑터 (PC 앱 <-> tb_* 변수)
 *
 * 무엇인가:
 *   지금까지 tb_* 변수는 **디버거 live watch** 로만 만질 수 있었다. 이 파일은
 *   같은 변수들을 R0 프로토콜(UART5 115200) 위로 열어, PC 앱
 *   `src/parts_verification` 이 개별기능 검증을 돌릴 수 있게 한다.
 *   디버거 경로는 그대로 살아 있다 - 이 파일은 그 위에 얹은 또 하나의 소비자다.
 *
 * 왜 별도 파일인가:
 *   protocol_r0.c 가 테스트벤치 헤더 12개를 끌어안으면 양산 바이너리에서
 *   떼어내기 어려워진다. 여기로 모아 두면 `ENABLE_TESTBENCH_APP = 0` 한 줄로
 *   이 파일 전체 + protocol_r0 의 훅 몇 줄만 사라진다(main.h 참조).
 *
 * --------------------------------------------------------------------------
 * 설계 규칙 3가지
 *
 * (1) **액추에이터를 직접 만지지 않는다.** TbApp_Write() 는 tb_* 플래그에
 *     값을 쓸 뿐이고, 실제 구동은 평소대로 MotorTask 의 TB_*_Poll() 이 한다.
 *     protocol_r0 의 W 처리는 defaultTask 문맥이므로 이 규칙을 어기면 모터
 *     소유권이 두 태스크로 갈라진다(protocol_r0.h §6 주의문과 같은 취지).
 *
 * (2) **벤치 유휴에서만 쓴다.** `AppMode_IsBenchIdle()` 이 아니면 쓰기를
 *     NAK 한다. 시나리오가 교반/도어를 쓰는 중에 앱이 끼어들면 안 된다.
 *     관측(TB_STATE)은 모드와 무관하게 언제나 읽을 수 있다.
 *
 * (3) **쓰고 되읽어 검증한다.** R0 §2 의 W 규칙 그대로다. TbApp_Write() 가
 *     쓴 뒤 TbApp_Read() 로 같은 값이 나오는지 확인하고 결과를 돌려준다.
 *     되읽기가 다른 항목(원샷 커맨드 등)은 아래 표에 명시했다.
 *
 * --------------------------------------------------------------------------
 * 원샷(one-shot) 항목의 되읽기
 *   `*_once` 계열(RINSE_SPIN_ONCE 등)과 STEP1/2 enable 은 펌웨어가 소비하면서
 *   스스로 0 으로 내린다. 쓴 직후 되읽으면 이미 0 일 수 있으므로 **검증을
 *   건너뛰고 항상 ACK** 한다(TB_ITEM_IS_ONESHOT). 실제로 돌았는지는 TB_STATE 의
 *   running/hit 비트로 확인하는 것이 맞다.
 * ========================================================================== */

/* ---- 0x33 TB_CTRL 항목 ID ------------------------------------------------
 * 값은 앱(src/parts_verification/pv_proto.py TbItem)과 **한 쌍**이다.
 * 새 항목은 뒤에 붙이고 기존 번호를 재사용하지 않는다. */
typedef enum
{
	TB_IT_NONE            = 0x00,

	/* --- BLDC 2대 (tb_tca9554) ------------------------------------------
	 * ★분쇄(M1)는 하드웨어 인터록에 묶여 있다: ENABLE 이 실제로 서려면
	 *   PF0(70℃ 바이메탈) LOW + PF3(배수문 닫힘) LOW 가 동시에 성립해야 한다
	 *   (구현현황 §0.19). 허가 없이 EN 을 1 로 쓰면 FG 가 오지 않아 약 10초 뒤
	 *   BLDC_LOCKED 로 떨어진다 - 앱은 TB_STATE 의 grind_allow 를 먼저 본다. */
	TB_IT_GRIND_EN        = 0x01,   /* tb_grind_en      0/1  (엣지: 0->1 기동) */
	TB_IT_GRIND_REV       = 0x02,   /* tb_grind_rev     0 CW / 1 CCW           */
	TB_IT_GRIND_SPD       = 0x03,   /* tb_grind_spd_req 1 = 한 칸 증속(원샷)   */
	TB_IT_STIR_EN         = 0x04,   /* tb_stir_en       0/1                    */
	TB_IT_STIR_REV        = 0x05,   /* tb_stir_rev      0 CW / 1 CCW           */
	TB_IT_STIR_SPD        = 0x06,   /* tb_stir_spd_req  1 = 한 칸 증속(원샷)   */
	TB_IT_KEYPAD_MOTOR    = 0x07,   /* tb_keypad_motor_en 0 = 버튼 눌러도 무시 */

	/* --- 배수문 WDoor (U5) ---------------------------------------------- */
	TB_IT_WDOOR_EN        = 0x10,
	TB_IT_WDOOR_REV       = 0x11,   /* 0 = 닫힘 (도어 방향 확정 2026-08-25)    */
	TB_IT_WDOOR_DUTY      = 0x12,   /* 0..100 %                                */
	TB_IT_WDOOR_PROFILE   = 0x13,   /* 1 = 시간 프로파일, 0 = 수동             */
	TB_IT_WDOOR_LIMIT     = 0x14,   /* 1 = 리미트 도달 시 자동 정지            */

	/* --- 배출문 TDoor (U7) ---------------------------------------------- */
	TB_IT_TDOOR_EN        = 0x18,
	TB_IT_TDOOR_REV       = 0x19,   /* 0 = 열림 (배수문과 방향이 반대다)       */
	TB_IT_TDOOR_DUTY      = 0x1A,
	TB_IT_TDOOR_PROFILE   = 0x1B,
	TB_IT_TDOOR_LIMIT     = 0x1C,

	/* --- 리프트 (U6) ----------------------------------------------------- */
	TB_IT_LIFT_EN         = 0x20,
	TB_IT_LIFT_REV        = 0x21,   /* 0 = 상승 / 1 = 하강 (확정 2026-09-20)   */
	TB_IT_LIFT_DUTY       = 0x22,
	TB_IT_LIFT_PROFILE    = 0x23,
	TB_IT_LIFT_LIMIT      = 0x24,   /* 하강 시 HS8 자동정지                    */

	/* --- 스테퍼 STEP1 (냄새관로) ----------------------------------------- */
	TB_IT_STEP1_EN        = 0x28,   /* ★원샷: 타이머 런, 끝나면 자동 0         */
	TB_IT_STEP1_DIR       = 0x29,   /* 0 = 열림 / 1 = 닫힘                     */
	TB_IT_STEP1_PERIOD    = 0x2A,   /* ms/step                                 */
	TB_IT_STEP1_RUN_MS    = 0x2B,   /* 런 길이 (기본 15000)                    */
	TB_IT_STEP1_HOLD      = 0x2C,   /* 1 = 정지 후 여자 유지                   */

	/* --- 스테퍼 STEP2 (흡입제어) ----------------------------------------- */
	TB_IT_STEP2_EN        = 0x30,   /* ★원샷                                   */
	TB_IT_STEP2_DIR       = 0x31,
	TB_IT_STEP2_PERIOD    = 0x32,
	TB_IT_STEP2_OPEN_MS   = 0x33,   /* dir=0 열림 런 (기본 1000)               */
	TB_IT_STEP2_CLOSE_MS  = 0x34,   /* dir=1 닫힘 런 (기본 2000)               */
	TB_IT_STEP2_HOLD      = 0x35,

	/* --- 밸브 / 팬 (tb_gpioout) ------------------------------------------ */
	TB_IT_VALVE_DRAIN     = 0x38,   /* VALVE-DRAIN-CLN PB14                    */
	TB_IT_VALVE_DRY       = 0x39,   /* VALVE-DRY-IN    PB13                    */
	TB_IT_FAN_VAPOR       = 0x3A,   /* FAN-VAPOR       PB15                    */
	TB_IT_FAN_EXHAUST     = 0x3B,   /* FAN-EXHAUST     PG2                     */
	TB_IT_FAN_BLDC        = 0x3C,   /* BLDC-FAN        PG1                     */

	/* --- 급수 (tb_water) -------------------------------------------------- */
	TB_IT_WATER_EN        = 0x40,   /* WATER-ON PE2 + 수위 모니터              */

	/* --- 히터 (tb_heat) --------------------------------------------------- */
	TB_IT_HEAT_EN         = 0x48,
	TB_IT_HEAT_MODE       = 0x49,   /* 0 MANUAL / 1 HYST                       */
	TB_IT_HEAT_MANUAL     = 0x4A,   /* MANUAL 에서 1 = HT-POWER HIGH           */
	TB_IT_HEAT_CH         = 0x4B,   /* NTC 채널 0..2                           */
	TB_IT_HEAT_ON_D10     = 0x4C,   /* 히스테리시스 ON  임계 [0.1℃]            */
	TB_IT_HEAT_OFF_D10    = 0x4D,   /* 히스테리시스 OFF 임계 [0.1℃]            */
	TB_IT_HEAT_CLR_FAULT  = 0x4E,   /* ★원샷: 래치 폴트 해제                   */

	/* --- 센서 모니터 enable ------------------------------------------------ */
	TB_IT_HALL_EN         = 0x50,   /* U24 8채널 모니터                         */
	TB_IT_DOORHALL_EN     = 0x51,   /* 도어 리밋홀 4채널 모니터                 */
	TB_IT_DIST_EN         = 0x52,   /* 거리센서(수거통 채움)                    */
	TB_IT_THERM_EN        = 0x53,   /* NTC 3채널                                */

	/* --- 헹굼 벤치 (tb_rinse, I02 실측) ------------------------------------ */
	TB_IT_RINSE_SPIN      = 0x58,   /* ★원샷: 교반 원샷 구동 시작              */
	TB_IT_RINSE_STOP      = 0x59,   /* ★원샷: 즉시 중단                        */
	TB_IT_RINSE_MARK      = 0x5A,   /* ★원샷: baseline/통계 리셋               */
	TB_IT_RINSE_RPM       = 0x5B,   /* 원샷 교반 출력축 RPM (기본 30)          */
	TB_IT_RINSE_REV       = 0x5C,
	TB_IT_RINSE_MS        = 0x5D,   /* 원샷 총 길이 (기본 15000)               */

	/* --- 스피커 ------------------------------------------------------------ */
	TB_IT_SPEAKER_EN      = 0x60,
	TB_IT_SPEAKER_FREQ    = 0x61,

	/* --- 중재자 (벤치 편의) ------------------------------------------------ */
	/* 1 = 마개 홀 중재를 멈춘다. 마개가 HS1/2/4/5 에 얹혀 있어도 벤치 모드가
	 * 유지된다. 벤치를 떠날 때 0 으로 되돌릴 것. */
	TB_IT_ARB_DISABLE     = 0x70,

	/* --- 일괄 -------------------------------------------------------------- */
	/* ★원샷: 모든 벤치 enable 을 0 으로. 앱의 비상정지 버튼이 쓴다.
	 * jungji 의 정지와는 다르다 - 이쪽은 플래그만 내리고 제동을 걸지 않는다.
	 * 진짜 비상정지는 0x31 CONTROL 의 PROTO_ACT_STOP 이다. */
	TB_IT_ALL_OFF         = 0x7F,

	/* ==== 2026-09-22 추가 (구현현황 §0.39) — 0x80 이후. 위 번호는 재사용 금지 ====
	 * 관측값은 0x28 TB_STATE2 에 실린다(0x27 은 36바이트로 이미 찼다). */

	/* --- 헹굼 벤치 확장 (tb_rinse: I02 엣지정의 + run E 패턴) --------------- */
	TB_IT_RINSE_BOTH      = 0x80,   /* tb_rinse_both_edges 0 상승 / 1 양엣지    */
	TB_IT_RINSE_PAT       = 0x81,   /* ★원샷: 4구간 패턴(run E) 시작            */
	TB_IT_RINSE_PAT_PARK  = 0x82,   /* 1 = 자석 위에 먼저 세우고 출발           */
	TB_IT_RINSE_PAT_STOP40= 0x83,   /* 1 = cnt 40 도달 시 종료                  */

	/* --- 회전수 운전 엔진 (tb_rotation, rotation.c 이식 검증) ---------------- */
	TB_IT_ROT_SELFTEST    = 0x88,   /* ★원샷: 벤더 test_rotation 을 타깃에서    */
	TB_IT_ROT_PROFILE     = 0x89,   /* 1 WASH / 2 DRAIN / 3 PROCESS             */
	TB_IT_ROT_RPM         = 0x8A,   /* 교반 지령 (날개 rpm, 기본 30)            */
	TB_IT_ROT_POS_SRC     = 0x8B,   /* 0 HS6 단순안 / 1 FG(P54=822)             */
	TB_IT_ROT_MAX_LEGS    = 0x8C,   /* PROCESS 종료 운전 수 (기본 36)           */
	TB_IT_ROT_RUN         = 0x8D,   /* ★원샷: run F 시작 (교반 M2만 구동)       */
	TB_IT_ROT_STOP        = 0x8E,   /* ★원샷: 즉시 중단                         */

	/* --- 음성 플래시 U21 (tb_voice) ----------------------------------------- */
	TB_IT_VOICE_SLOT      = 0x90,   /* 0..31: hdr/play 대상 슬롯                */
	TB_IT_VOICE_PROBE     = 0x91,   /* ★원샷: JEDEC ID + W25Q_Init              */
	TB_IT_VOICE_DIR       = 0x92,   /* ★원샷: 디렉터리 읽기·검증                */
	TB_IT_VOICE_HDR       = 0x93,   /* ★원샷: 슬롯 헤더 읽기·검증               */
	TB_IT_VOICE_PLAY      = 0x94,   /* ★원샷: 슬롯 재생                         */
	TB_IT_VOICE_STOP      = 0x95,   /* ★원샷: 재생 중지                         */
	/* ★원샷: 테스트 섹터(화이트리스트 0x001000~0x00FFFF) 소거/기록 왕복.
	 * defaultTask 를 약 0.9초 묶는다 - 그동안 모니터링 M 이 한 번 빈다. */
	TB_IT_VOICE_SELFTEST  = 0x96,

	/* --- R0 코덱 자체검사 (tb_protocol) -------------------------------------- */
	TB_IT_PROTO_SELFTEST  = 0x98    /* ★원샷: 인코드·디코드 왕복 (회선 무관)    */
} tb_item_t;

/* 원샷 항목인가(되읽기 검증을 건너뛴다). 위 주석 참조. */
#define TB_ITEM_IS_ONESHOT(it)                                            \
	(((it) == TB_IT_GRIND_SPD)   || ((it) == TB_IT_STIR_SPD)   ||         \
	 ((it) == TB_IT_STEP1_EN)    || ((it) == TB_IT_STEP2_EN)   ||         \
	 ((it) == TB_IT_HEAT_CLR_FAULT) ||                                    \
	 ((it) == TB_IT_RINSE_SPIN)  || ((it) == TB_IT_RINSE_STOP) ||         \
	 ((it) == TB_IT_RINSE_MARK)  || ((it) == TB_IT_ALL_OFF)    ||         \
	 ((it) == TB_IT_RINSE_PAT)   ||                                       \
	 ((it) == TB_IT_ROT_SELFTEST)|| ((it) == TB_IT_ROT_RUN)    ||         \
	 ((it) == TB_IT_ROT_STOP)    ||                                       \
	 ((it) == TB_IT_VOICE_PROBE) || ((it) == TB_IT_VOICE_DIR)  ||         \
	 ((it) == TB_IT_VOICE_HDR)   || ((it) == TB_IT_VOICE_PLAY) ||         \
	 ((it) == TB_IT_VOICE_STOP)  || ((it) == TB_IT_VOICE_SELFTEST) ||     \
	 ((it) == TB_IT_PROTO_SELFTEST))

/* ---- 관찰 컨텍스트 (디버거 watch + TB_STATE 꼬리에 실린다) --------------- */
typedef struct
{
	volatile uint8_t  last_item;    /* 마지막으로 받은 항목 ID               */
	volatile uint8_t  last_result;  /* 0 = ACK, 1 = NAK                      */
	volatile uint8_t  last_nak;     /* proto_nak_t                           */
	volatile uint32_t last_value;   /* 마지막으로 쓴 값                      */
	volatile uint16_t write_count;  /* 접수(ACK)된 쓰기 누적                 */
	volatile uint16_t reject_count; /* NAK 누적                              */
} TbAppCtx;

extern TbAppCtx g_tb_app;

/* ---- API ----------------------------------------------------------------
 * 전부 defaultTask(프로토콜 문맥)에서만 불린다. 모터를 만지지 않는다. */
void     TbApp_Init(void);

/* 항목의 현재 값. 미지원 항목이면 0. */
uint32_t TbApp_Read(tb_item_t item);

/* 항목에 값을 쓴다. 1 = ACK, 0 = NAK(사유는 *nak 에).
 * 벤치 유휴가 아니면 무조건 0(PROTO_NAK_BUSY). */
uint8_t  TbApp_Write(tb_item_t item, uint32_t value, uint8_t *nak);

/* 0x27 TB_STATE 페이로드를 채운다. 반환 = 쓴 바이트 수(= PROTO_LEN_TB_STATE).
 * 모드와 무관하게 언제나 유효하다(관측 전용). */
uint16_t TbApp_BuildState(uint8_t *b, uint16_t buf_sz);

/* 0x28 TB_STATE2 페이로드(회전 엔진 · 헹굼 패턴 · 음성 · 코덱 자체검사).
 * 반환 = PROTO_LEN_TB_STATE2. 모드와 무관하게 언제나 유효하다. */
uint16_t TbApp_BuildState2(uint8_t *b, uint16_t buf_sz);

/* 0x33 TB_CTRL 의 R 응답을 채운다. 반환 = PROTO_LEN_TB_CTRL. */
uint16_t TbApp_BuildCtrl(uint8_t *b, uint16_t buf_sz);

/* 모든 벤치 enable 을 0 으로(TB_IT_ALL_OFF 본체). jungji 와 달리 제동은
 * 걸지 않는다 - 플래그만 내린다. */
void     TbApp_AllOff(void);

#endif /* ENABLE_TESTBENCH_APP */

#ifdef __cplusplus
}
#endif

#endif /* TESTBENCH_TB_APP_H_ */
