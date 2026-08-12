#ifndef SCENARIO_DONGJAK_H_
#define SCENARIO_DONGJAK_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * dongjak - "동작"(2단계: 건조 -> 분쇄 -> 배출) 시나리오 상태머신.
 *
 * ★기준 원본: doc/R1/zerogeo_scenario.docx ([2단계] 동작 기능, 2026-07-30 김수빈)
 *   (사용자_시나리오_검토정리.md와 타이밍이 다르며, docx가 최신·우선.)
 * 관련 HW 미확정: Core/Scenario/하드웨어결정_잔여항목.md.
 *
 * 시간 기준: 모든 분(min) 마커는 "동작 홀센서 인식(=시나리오 시작)"부터의 절대시간.
 *   110분 → 분쇄 2000RPM 역회전 / 120분 → 식힘 / 130분 → 배출 / 135분 → 래치 해제(HW)
 *
 * 안전 락(HW, MCU 제어 아님):
 *   - 배수문 락: (배수문닫힘 홀 LOW) & (80℃ 바이메탈 LOW) → 래치 SET (docx 4.2)
 *   - 급수밸브 차단 락: (60℃ 바이메탈 HIGH) → 래치 SET (docx 4.3)
 *   - 분쇄 게이팅: (MCU HIGH) & (80℃ 바이메탈 LOW) 동시 (docx 6.1)
 *   - 210℃ 히터 차단: 바이메탈(직렬 4개) HW (docx 9.1)
 *   래치 해제는 배출문 리드/홀 스위치로 HW 즉시 처리. FW는 감시/구동만 한다.
 *
 * 태스크 배치(freertos.c):
 *   Dongjak_SenseTick()  - StartDefaultTask(100ms). 센서: 시작(HS)·온도·수위·fill%.
 *   Dongjak_MotorTick()  - StartMotorTask(1ms). 히터/교반/분쇄/도어/스테퍼/팬 구동 +
 *                          상태 전이 소유. 호출자가 BldcCtrl_Tick(M1,M2)를 함께 호출.
 * 온도 단위: 0.1C(d10). 예) 800 = 80.0C. 써미스터 에러면 직전 유효값 유지.
 * ========================================================================== */

/* ---- 온도 임계 (0.1C 단위) --------------------------------------------- */
#ifndef DJ_TEMP_GRIND_ON_D10
#define DJ_TEMP_GRIND_ON_D10    800   /* 80.0C: 분쇄 허용 / 식힘 분쇄 OFF 기준 */
#endif
#ifndef DJ_TEMP_BLDCFAN_ON_D10
#define DJ_TEMP_BLDCFAN_ON_D10  800   /* 80.0C: BLDC 식힘팬 동작 기준          */
#endif
#ifndef DJ_TEMP_VAPOR_ON_D10
#define DJ_TEMP_VAPOR_ON_D10    1000  /* 100.0C: 수증기 ALL ON                 */
#endif
#ifndef DJ_TEMP_VAPOR_OFF_D10
#define DJ_TEMP_VAPOR_OFF_D10   840   /* 84.0C: 수증기 ALL OFF                 */
#endif
#ifndef DJ_TEMP_HEATER_ON_D10
#define DJ_TEMP_HEATER_ON_D10   1900  /* 190.0C: 히터 복귀 ON                  */
#endif
#ifndef DJ_TEMP_HEATER_OFF_D10
#define DJ_TEMP_HEATER_OFF_D10  1950  /* 195.0C: 히터 OFF                      */
#endif
#ifndef DJ_TEMP_SAFETY_D10
#define DJ_TEMP_SAFETY_D10      2100  /* 210.0C: 과열 안전(주체는 HW 바이메탈) */
#endif

/* ---- 절대 경과시간 마커 (ms, 시나리오 시작 기준) ----------------------- */
#ifndef DJ_T_HISPEED_MS
#define DJ_T_HISPEED_MS         (110UL * 60000UL) /* 110분: 분쇄 2000 역회전  */
#endif
#ifndef DJ_T_COOLDOWN_MS
#define DJ_T_COOLDOWN_MS        (120UL * 60000UL) /* 120분: 식힘 시작          */
#endif
#ifndef DJ_T_DISCHARGE_MS
#define DJ_T_DISCHARGE_MS       (130UL * 60000UL) /* 130분: 배출 시작          */
#endif
#ifndef DJ_T_LATCH_MS
#define DJ_T_LATCH_MS           (135UL * 60000UL) /* 135분: 래치 해제(HW,참고) */
#endif
#ifndef DJ_GRIND_COARSE_MS
#define DJ_GRIND_COARSE_MS      (3UL * 60000UL)   /* 1차 깍두기 분쇄 ~3분      */
#endif

/* ---- 초기 헹굼/배수 (docx 2단계 2·3번) --------------------------------- */
#ifndef DJ_RINSE_COUNT
#define DJ_RINSE_COUNT          2U        /* 헹굼 2회 (1단계 2~6번)           */
#endif
#ifndef DJ_RINSE_STIR_MS
#define DJ_RINSE_STIR_MS        150000UL  /* 헹굼 교반 2분30초(22 cycle)      */
#endif
#ifndef DJ_RINSE_DRAIN_MS
#define DJ_RINSE_DRAIN_MS       20000UL   /* 헹굼 간 배수 대기(짧게)          */
#endif
#ifndef DJ_DRAIN_RESIDUAL_MS
#define DJ_DRAIN_RESIDUAL_MS    120000UL  /* 2회 후 배수문 열고 2분 교반 배수 */
#endif
#ifndef DJ_FILL_EXTRA_MS
#define DJ_FILL_EXTRA_MS        2000U     /* 수위 감지 후 2초 추가급수        */
#endif

/* ---- 배출 (docx 10·11번) ----------------------------------------------- */
#ifndef DJ_DISCHARGE_STIR_MS
#define DJ_DISCHARGE_STIR_MS    120000UL  /* 배출문 열고 2분 교반 배출        */
#endif

/* ---- RPM (분쇄=모터RPM, 교반=출력RPM) ---------------------------------- */
#ifndef DJ_STIR_RPM
#define DJ_STIR_RPM             20U       /* 건조 교반 ~20RPM                 */
#endif
#ifndef DJ_RINSE_STIR_RPM
#define DJ_RINSE_STIR_RPM       25U       /* 헹굼 교반 ~25RPM(1단계 동일)     */
#endif
#ifndef DJ_GRIND_COARSE_RPM
#define DJ_GRIND_COARSE_RPM     1500U     /* 1차 분쇄 CW                      */
#endif
#ifndef DJ_GRIND_FINE_RPM
#define DJ_GRIND_FINE_RPM       1000U     /* 이후 분쇄 CW 연속                */
#endif
#ifndef DJ_GRIND_FINAL_RPM
#define DJ_GRIND_FINAL_RPM      2000U     /* 110분↑ 역회전 고속               */
#endif

/* ---- 서브-패턴 구간(ms) ------------------------------------------------ */
#define DJ_STIR_FWD_MS          3000U     /* 건조 교반 CW 구간                */
#define DJ_STIR_STOP_MS         2000U     /* 건조 교반 정지 구간              */
#define DJ_STIR_REV_MS          3000U     /* 건조 교반 CCW 구간               */
#define DJ_STIR_FWD_REPS        5U        /* CW 반복 후 CCW                   */
#define DJ_S313_CW_MS           3000U     /* 식힘/배출 교반 CW 구간           */
#define DJ_S313_STOP_MS         1000U     /* 식힘/배출 교반 정지 구간         */
#define DJ_S313_CCW_MS          3000U     /* 식힘/배출 교반 CCW 구간          */
#define DJ_GRIND_RUN_MS         3000U     /* 1차 분쇄 구동 구간(CW)           */
#define DJ_GRIND_STOP_MS        2000U     /* 1차 분쇄 정지 구간               */
#define DJ_GRIND_RUN_FINAL_MS   4000U     /* 마무리 분쇄 구동(역회전)         */
#define DJ_GRIND_STOP_FINAL_MS  2000U     /* 마무리 분쇄 정지                 */
/* 헹굼 교반은 식힘/배출과 동일한 313 패턴(DJ_S313_*)을 재사용한다. */

/* ---- 팬 (docx 7번) ----------------------------------------------------- */
#define DJ_FANX_ON_MS           10000U    /* 내부 공기정화팬 ON 구간(10s)     */
#define DJ_FANX_OFF_MS          5000U     /* 내부 공기정화팬 OFF 구간(5s)     */
#define DJ_FANB_ON_MS           30000U    /* BLDC 식힘팬 ON 구간(30s)         */
#define DJ_FANB_OFF_MS          10000U    /* BLDC 식힘팬 OFF 구간(10s)        */

/* ---- 스테퍼 (냄새 관로=STEP1, 흡입제어=STEP2 / 역할 TBD) --------------- */
#ifndef DJ_DUCT_STEPS
#define DJ_DUCT_STEPS           512U      /* 관로 완전개폐 스텝수(TBD)        */
#endif
#ifndef DJ_STEP_INTERVAL_MS
#define DJ_STEP_INTERVAL_MS     2U        /* 스텝 간격(500PPS, <=1000PPS)     */
#endif

/* ---- 수거통 확인 (docx 배출 전 수거통 확인) --------------------------- */
#ifndef DJ_BIN_CHECK_ENABLE
#define DJ_BIN_CHECK_ENABLE     1
#endif
#ifndef DJ_MAX_CYCLES
#define DJ_MAX_CYCLES           6U
#endif
#ifndef DJ_BIN_FULL_PCT
#define DJ_BIN_FULL_PCT         90U       /* 분말 높이 임계(%) (TBD)          */
#endif

/* ---- 시작 홀 채널 / 온도 채널 / 수위 정책 (TBD, 모음과 동일 기조) ------ */
#ifndef DJ_HS_START_IDX
#define DJ_HS_START_IDX         4U        /* 동작=HS5 가정(0-based)           */
#endif
#ifndef DJ_TEMP_CH
#define DJ_TEMP_CH              0U        /* 처리통 내부 써미스터(0..2, TBD)  */
#endif
#ifndef DJ_WATER_USE_LEVEL
#define DJ_WATER_USE_LEVEL      0
#endif
#ifndef DJ_WATER_ACTIVE_LOW
#define DJ_WATER_ACTIVE_LOW     1
#endif

/* ---- 안전 타임아웃 (스펙 외 placeholder) ------------------------------- */
#ifndef DJ_DOOR_TIMEOUT_MS
#define DJ_DOOR_TIMEOUT_MS      15000U
#endif
#ifndef DJ_FILL_TIMEOUT_MS
#define DJ_FILL_TIMEOUT_MS      120000UL
#endif

/* ---- 메인 phase (디버거 관찰용) ---------------------------------------- */
typedef enum
{
	DJ_IDLE = 0,
	DJ_RINSE,          /* 초기 헹굼 2회 (닫힘/급수/교반)                      */
	DJ_DRAIN_RESIDUAL, /* 배수문 열고 2분 교반(잔수 배수)                     */
	DJ_HEAT,           /* 건조: 히터+교반+분쇄+수증기+팬 동시(온도/시간 게이팅)*/
	DJ_COOLDOWN,       /* 120분~ 식힘: 히터OFF, 교반 CW3/1/CCW3, 분쇄 80℃서OFF*/
	DJ_BIN_CHECK,      /* 배출 전 수거통 유무/횟수/높이 확인                  */
	DJ_DISCHARGE,      /* 130분~ 배출문 2분 개방+교반 배출 후 닫고 배수부 개방*/
	DJ_DONE,
	DJ_ERROR
} DongjakState;

/* 서브-FSM 위상 */
typedef enum { DJ_RS_CLOSE=0, DJ_RS_FILL, DJ_RS_FILL_EXTRA, DJ_RS_STIR,
               DJ_RS_OPEN, DJ_RS_DRAIN } DjRinsePhase;
typedef enum { DJ_STIR_FWD=0, DJ_STIR_STOPPED, DJ_STIR_REV } DjStirPhase;
typedef enum { DJ_GM_OFF=0, DJ_GM_COARSE, DJ_GM_FINE, DJ_GM_FINAL, DJ_GM_COOL } DjGrindMode;
typedef enum { DJ_GR_RUN=0, DJ_GR_STOP } DjGrindPhase;
typedef enum { DJ_VP_CLOSED=0, DJ_VP_OPEN_DUCT, DJ_VP_OPEN_AIR, DJ_VP_OPEN,
               DJ_VP_CLOSE_AIR, DJ_VP_CLOSE_DUCT } DjVaporPhase;
typedef enum { DJ_DS_OPEN_T=0, DJ_DS_EXPEL, DJ_DS_CLOSE_T, DJ_DS_OPEN_W } DjDischPhase;

typedef struct
{
	volatile uint8_t  state;          /* DongjakState (RO)                    */
	uint32_t          state_since;
	uint32_t          scn_start;      /* 시나리오 시작 tick(절대시간 기준)    */

	volatile uint8_t  start_req;
	volatile uint8_t  dbg_force_start;/* 벤치 강제 시작(1회성, HS와 OR)       */
	volatile int16_t  temp_d10;       /* 처리통 온도(SenseTick 갱신)          */
	volatile uint8_t  water_reached;
	volatile uint8_t  bin_fill_pct;
	uint8_t           hs_prev;

	uint16_t          cycle_count;    /* 처리 완료 횟수                       */

	/* 초기 헹굼 */
	uint8_t           rinse_iter;
	uint8_t           rinse_phase;    /* DjRinsePhase                         */
	uint32_t          rinse_since;

	/* 건조 */
	uint8_t           heat_started;   /* 0 = 배수문 닫는 중                    */
	uint32_t          heat_since;     /* 히터 구간 시작 tick(참고)            */

	/* 교반(건조=FWD/STOP/REV, 313=0/1/2 재사용) */
	uint8_t           stir_phase;
	uint32_t          stir_since;
	uint8_t           stir_reps;

	/* 분쇄 */
	uint8_t           grind_mode;     /* DjGrindMode                          */
	uint8_t           grind_phase;    /* DjGrindPhase(토글 모드)              */
	uint32_t          grind_since;
	uint32_t          grind_start;    /* 분쇄 개시(1차 3분 판정용)            */

	/* 수증기 */
	uint8_t           vapor_phase;    /* DjVaporPhase                         */
	uint32_t          vapor_step_since;
	uint16_t          vapor_step_cnt;

	/* 팬 */
	uint8_t           fanx_on;        /* 내부정화팬 현재 ON?                  */
	uint32_t          fanx_since;
	uint8_t           fanb_on;        /* BLDC 식힘팬 현재 ON?                 */
	uint32_t          fanb_since;

	/* 배출 */
	uint8_t           disc_phase;     /* DjDischPhase                         */
	uint32_t          disc_since;
} DongjakCtx;

extern DongjakCtx g_dongjak;

/* ---- API ----------------------------------------------------------------- */
void         Dongjak_Init(void);
void         Dongjak_Start(void);
void         Dongjak_Abort(void);
void         Dongjak_SenseTick(void);            /* 100ms, StartDefaultTask   */
void         Dongjak_MotorTick(uint32_t now_ms); /* 1ms,  StartMotorTask      */
DongjakState Dongjak_GetState(void);
uint8_t      Dongjak_IsBusy(void);

#ifdef __cplusplus
}
#endif

#endif /* SCENARIO_DONGJAK_H_ */
