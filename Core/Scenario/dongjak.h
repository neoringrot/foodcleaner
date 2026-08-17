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

/* ==========================================================================
 * [테스트 전용 — 현재 OFF]  타임라인 압축 + 단계 비프
 * --------------------------------------------------------------------------
 * 135분 시나리오를 벤치에서 ~12분에 관찰하기 위한 임시 기능. ★2026-08-18부로
 * 0(비활성) = 아래 양산 타임라인(110/120/130/135분)이 그대로 적용된다.
 *  (1) DJ_HEAT~배출 절대/상대 시간상수를 선점 정의해 원래 값을 무력화
 *      (아래 원래 #ifndef 정의가 양산값).
 *      110→7 / 120→8 / 130→9 / 135→12 분, 1차 분쇄 3분→30초.
 *      (135→12분: 배출 창 3분 > HW 배출문 2분 타이머. 아래 DJ_T_LATCH_MS 주석 참조)
 *  (2) 스피커 비프: 7분대 삑 / 8분대 삑삑 / 9분대 삑삑삑 (압축된 테스트 시각 기준).
 * 다시 벤치 압축이 필요하면 1로 두면 된다(dongjak.c의 [TEST] 코드가 함께 살아난다).
 * 완전 제거 시: 이 블록 + dongjak.c의 [TEST] 표시 코드(dj_test_beep·호출부·
 *       test_beeped_hi·lm4871 include)를 함께 삭제.
 * ========================================================================== */
#ifndef DJ_TEST_FAST_TIMING
#define DJ_TEST_FAST_TIMING     0        /* 0 = 양산 타임라인 / 1 = 벤치 압축  */
#endif
#if DJ_TEST_FAST_TIMING
#define DJ_T_HISPEED_MS         (7UL  * 60000UL)  /* [TEST] 110분 → 7분  */
#define DJ_T_COOLDOWN_MS        (8UL  * 60000UL)  /* [TEST] 120분 → 8분  */
#define DJ_T_DISCHARGE_MS       (9UL  * 60000UL)  /* [TEST] 130분 → 9분  */
/* [TEST] 135분 → 12분. ★9분이 아니라 12분인 이유: 배출 창(LATCH-DISCHARGE)이
 * HW 배출문 2분 타이머보다 길어야 한다. 10분(창 1분)이면 TIMER-OUT↓이 오기 전에
 * 135분 백스톱이 먼저 닫기를 지시하고, 그 시점엔 HW 래치가 아직 문 제어를 막고
 * 있어 배출문 닫힘을 검증할 수 없다. 12분 = 창 3분 > HW 2분. */
#define DJ_T_LATCH_MS           (12UL * 60000UL)  /* [TEST] 135분 → 12분 */
#define DJ_GRIND_COARSE_MS      (30UL * 1000UL)   /* [TEST] 1차 분쇄 3분 → 30초 */
#define DJ_FANX_ON_MS           (15UL * 1000UL)   /* [TEST] 배기팬 15분 → 15초 */
#define DJ_FANX_OFF_MS          (2UL  * 1000UL)   /* [TEST] 배기팬 2분 → 2초   */
#define DJ_TEST_BEEP_HZ         2000U             /* [TEST] 비프 주파수(Hz)   */
#define DJ_TEST_BEEP_MS         80U               /* [TEST] 비프 1회 길이(ms) */
#define DJ_TEST_BEEP_GAP_MS     80U               /* [TEST] 비프 간 간격(ms)  */
#endif

/* ---- 온도 임계 (0.1C 단위) --------------------------------------------- */
/* 분쇄 허용(BLDC 활성) 기준. 스펙: 실제 80℃(바이메탈)에서 활성인데, 그 시점
 * 온도센서 측정값은 약 65℃ -> MCU는 센서 65℃를 기준으로 분쇄 구동(2026-08-15 신스펙).
 * (이전 기획서 센서 80℃에서 정정) */
#ifndef DJ_TEMP_GRIND_ON_D10
#define DJ_TEMP_GRIND_ON_D10    650   /* 65.0C(센서)=실제 80C: 건조 분쇄 허용   */
#endif
/* 식힘(COOLDOWN) 분쇄 OFF 기준. 신스펙: "80도부터 분쇄 꺼짐"(센서 80℃).
 * (이전 기획서 100℃에서 80℃로 재정정, 2026-08-15 신스펙) */
#ifndef DJ_TEMP_COOL_GRIND_OFF_D10
#define DJ_TEMP_COOL_GRIND_OFF_D10  800   /* 80.0C: 식힘 중 분쇄 OFF 기준      */
#endif
/* 식힘 분쇄 OFF(80℃) 판정에 HW 80℃ 바이메탈(PF0/exti0_BIMETAL_80)의 하강엣지를
 * OR로 추가. 센서(65℃≈실제80℃ 오프셋) 단독보다 실제 80℃ 도달을 정확히 잡는다.
 * COOLDOWN 진입 시 stale 엣지 1회 클리어 후 "센서<80℃ || 바이메탈 하강엣지" 중
 * 먼저 성립하면 분쇄 OFF·교반313으로 전환(2026-08-16 요구). 0=바이메탈 미사용. */
#ifndef DJ_COOL_USE_BIMETAL80
#define DJ_COOL_USE_BIMETAL80   1
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
/* 건조 히터 히스테리시스. 신스펙 "건조 말기 히터 온도 유지: 114℃ ON / 117℃ OFF"를
 * 건조 전 구간에 적용(항목5 A안, 2026-08-15). 초기 급승온은 60/80℃ 바이메탈이 보조.
 * (이전 190/195℃에서 정정) */
#ifndef DJ_TEMP_HEATER_ON_D10
#define DJ_TEMP_HEATER_ON_D10   1140  /* 114.0C: 히터 복귀 ON                  */
#endif
#ifndef DJ_TEMP_HEATER_OFF_D10
#define DJ_TEMP_HEATER_OFF_D10  1170  /* 117.0C: 히터 OFF                      */
#endif
#ifndef DJ_TEMP_SAFETY_D10
#define DJ_TEMP_SAFETY_D10      2100  /* 210.0C: 과열 안전(주체는 HW 바이메탈) */
#endif

/* ---- 절대 경과시간 마커 (ms, 시나리오 시작 기준) ----------------------- */
#ifndef DJ_T_HISPEED_MS
#define DJ_T_HISPEED_MS         (110UL * 60000UL) /* 110분: 분쇄 2000 역회전 + 교반 27RPM */
#endif
#ifndef DJ_T_COOLDOWN_MS
#define DJ_T_COOLDOWN_MS        (120UL * 60000UL) /* 120분: 식힘 시작          */
#endif
#ifndef DJ_T_DISCHARGE_MS
#define DJ_T_DISCHARGE_MS       (130UL * 60000UL) /* 130분: 배출 시작          */
#endif
/* 135분: 전체 타이머 만료 -> 배출문 닫힘 백스톱. 정상은 배출 HW 2분타이머의
 * TIMER-OUT(PF8) 하강엣지로 종료하고, 미도달 시 이 시각에 강제 종료(신스펙 항목9). */
#ifndef DJ_T_LATCH_MS
#define DJ_T_LATCH_MS           (135UL * 60000UL) /* 135분: 배출 종료 백스톱   */
#endif
#ifndef DJ_GRIND_COARSE_MS
#define DJ_GRIND_COARSE_MS      (3UL * 60000UL)   /* 1차 깍두기 분쇄 ~3분      */
#endif

/* ---- 초기 헹굼/배수 (docx 2단계 2·3번) ---------------------------------
 * 헹굼 2회는 고정 상수(rinse_iter 변수 폐지) - 1차/2차를 별도 상태로 언롤.
 * docx/모음은 교반 2분30초지만 동작 헹굼은 2분. 배수는 교반하며 진행:
 *   1차 30초(물빼기), 2차 90초(물빼기 30초 + 잔수 60초) -> 별도 DJ_DRAIN_RESIDUAL 폐지. */
#ifndef DJ_RINSE_COUNT
#define DJ_RINSE_COUNT          2U        /* 헹굼 2회 (고정; 상태 언롤로 관리) */
#endif
#ifndef DJ_RINSE_STIR_MS
#define DJ_RINSE_STIR_MS        120000UL  /* 헹굼 교반 2분(동작 전용, 모음=2분30초) */
#endif
#ifndef DJ_RINSE1_DRAIN_MS
#define DJ_RINSE1_DRAIN_MS      30000UL   /* 1차 배수 교반 30초(물빼기)       */
#endif
#ifndef DJ_RINSE2_DRAIN_MS
#define DJ_RINSE2_DRAIN_MS      90000UL   /* 2차 배수 교반 90초(30초+잔수60초) */
#endif
#ifndef DJ_FILL_EXTRA_MS
#define DJ_FILL_EXTRA_MS        2000U     /* 수위 감지 후 2초 추가급수        */
#endif
/* 급수 시 WATER_ON(PE2, 급수 펌프/메인 밸브 enable)을 VALVE_DRY_IN(PB13)과
 * 함께 ON. 근거: 수위센서 HW 검증이 PE2 ON 상태(tb_water)에서 통수·감지되었고,
 * PB13 단독 통수는 미확인. 모음(MOEUM_FILL_USE_WATER_ON)과 동일 기조 — 이 처리가
 * 없으면 헹굼 첫 FILL에서 수위 감지 실패로 FILL 타임아웃→DJ_ERROR 낙하 위험.
 * 벤치에서 PB13 단독으로 물이 나오면 0으로 두면 된다. */
#ifndef DJ_FILL_USE_WATER_ON
#define DJ_FILL_USE_WATER_ON    1
#endif
/* 배수 시 VALVE_DRAIN_CLN(PB14, 배수구세척솔)을 배수문(WDoor) 개방과 함께 연다.
 * 근거: 실제 배수는 배수문(모터)만이 아니라 배수밸브를 함께 여는 구조(README §5).
 * WDoor를 여는(=배수) 모든 지점에서 밸브 ON, 닫는(=밀폐) 지점에서 밸브 OFF.
 * PB14는 tb_gpioout(tb_valve_drain_en)로 HW 검증완료(개별기능검증R1 항목5).
 * 벤치에서 배수밸브 없이 문만으로 배수되면 0으로 두면 된다. */
#ifndef DJ_DRAIN_VALVE_USE
#define DJ_DRAIN_VALVE_USE      1
#endif

/* ---- 배출 (docx 10·11번) -----------------------------------------------
 * 배출문 열림 = HW 2분 타이머 시작(그동안 HW가 배출문 제어를 차단). 교반 배출은
 * TIMER-OUT(PF8) 하강엣지 = 2분 종료로 멈추고, 배출문 "닫기"는 135분
 * (DJ_T_LATCH_MS)에 개시한다.
 * ★2026-08-17 재정의: 이전에는 TIMER-OUT 하강엣지에서 곧바로 문을 닫았으나,
 *   "130분부터 2분간 배출 / 135분이 되면 배출문이 다시 닫힌다" 지시에 맞춰
 *   2분 완료 시엔 교반만 정지하고 문은 열어둔 채(DJ_DS_HOLD) 135분을 기다린다.
 *   구 동작으로 되돌리려면 DJ_DISCH_CLOSE_AT_LATCH=0. */
#ifndef DJ_DISCHARGE_STIR_MS
#define DJ_DISCHARGE_STIR_MS    120000UL  /* 배출 교반 2분(TIMER-OUT SW 백업)  */
#endif
/* TIMER-OUT(PF8) 하강엣지가 오지 않아도 SW 2분(DJ_DISCHARGE_STIR_MS)으로 교반을
 * 멈출지. TIMER-OUT 극성/플로팅(NOPULL)이 HW 미검증이라 기본 1(백업 사용).
 * 0이면 TIMER-OUT만 신뢰 -> 미도달 시 교반이 135분까지 계속 돈다. */
#ifndef DJ_DISCH_STIR_SW_BACKUP
#define DJ_DISCH_STIR_SW_BACKUP 1
#endif
/* 배출문 닫기 개시 시점.
 *   1 = 135분(DJ_T_LATCH_MS) 도달 시 닫기 시작(신지시, 기본). 교반은 2분에 정지.
 *   0 = 교반 2분 종료(TIMER-OUT↓) 즉시 닫기 시작(구 동작). */
#ifndef DJ_DISCH_CLOSE_AT_LATCH
#define DJ_DISCH_CLOSE_AT_LATCH 1
#endif
/* 배출문(TDoor) 개폐 중 THALL 리미트 미인식 시 처리.
 *   1 = 타임아웃 없이 리미트 인식까지 계속 구동(신지시 - 배출에서 DJ_ERROR로
 *       전체 프로세스를 멈추지 않는다). 기본.
 *   0 = DJ_DOOR_TIMEOUT_MS(15s) 후 DJ_ERROR(DJ_ERR_DISCH_OPEN/CLOSE).
 * ⚠ 1일 때: 리미트/자석 고장이면 DC 모터가 스톨 상태로 계속 구동된다. 개방은
 *   135분 백스톱이 회수하지만(최대 DJ_DISCH_WINDOW_MS=5분), 닫기는 시간 상한이
 *   없어 DRV8871 전류제한/열보호와 정지버튼(Dongjak_RequestStop)에만 의존한다.
 *   실제 리미트 도달 시간은 disc_open_ms / disc_close_ms로 확인할 것. */
#ifndef DJ_DISCH_DOOR_NO_TIMEOUT
#define DJ_DISCH_DOOR_NO_TIMEOUT 1
#endif
/* 위 무한 구동의 스톨 열 대책 — 간헐(펄스) 구동.
 *   1 = DJ_DISCH_DOOR_PULSE_AFTER_MS까지는 연속 구동(정상 개폐 시간대는 기존과
 *       완전히 동일), 그 이후부터 ON/OFF 반복하며 리미트 인식까지 계속 시도.
 *       쉬는 구간엔 코스트 + VM(EN) OFF까지 내려 모터/드라이버를 식힌다.
 *   0 = 리미트 인식까지 100% 연속 구동.
 * 포기하지 않는다는 요구(타임아웃 없음)는 그대로고, 듀티만 낮추는 것이다. */
#ifndef DJ_DISCH_DOOR_PULSE
#define DJ_DISCH_DOOR_PULSE      1
#endif
#ifndef DJ_DISCH_DOOR_PULSE_AFTER_MS
#define DJ_DISCH_DOOR_PULSE_AFTER_MS 15000U /* 이 시간까지는 연속 구동(정상 구간) */
#endif
#ifndef DJ_DISCH_DOOR_PULSE_ON_MS
#define DJ_DISCH_DOOR_PULSE_ON_MS    10000U /* 이후 구동 10초 …                  */
#endif
#ifndef DJ_DISCH_DOOR_PULSE_OFF_MS
#define DJ_DISCH_DOOR_PULSE_OFF_MS    5000U /* … 휴지 5초 반복                   */
#endif
/* 배출 창(문 개방 개시 ~ 닫기 개시): 정상은 130분->135분. 수거통 확인 대기 등으로
 * 배출 진입이 135분 이후로 밀려도 최소 이 시간은 열어둔 뒤 닫는다(진입 직후 즉시
 * 닫힘 방지). 절대 135분과 AND 조건. */
#ifndef DJ_DISCH_WINDOW_MS
#define DJ_DISCH_WINDOW_MS      ((uint32_t)DJ_T_LATCH_MS - (uint32_t)DJ_T_DISCHARGE_MS)
#endif

/* ---- RPM (분쇄=모터RPM, 교반=출력RPM) ---------------------------------- */
#ifndef DJ_STIR_RPM
#define DJ_STIR_RPM             20U       /* 건조 교반 ~20RPM (110분 전)       */
#endif
/* 신스펙 항목3: 110분부터 교반 속도만 27RPM으로 변경(패턴은 건조와 동일). */
#ifndef DJ_STIR_RPM_HISPEED
#define DJ_STIR_RPM_HISPEED     27U       /* 110분↑ 교반 27RPM                 */
#endif
/* 기획서 4.2: "부하가 크면 약 30RPM 검토". 부하감지(4.6)는 별도 미구현 서브시스템
 * 이라 현재 스위칭 소스가 없어 대기 상태(값만 기록). 부하감지 구현 시 교반 목표를
 * 이 값으로 올리도록 연동. */
#ifndef DJ_STIR_RPM_HILOAD
#define DJ_STIR_RPM_HILOAD      30U       /* (대기) 부하 큼 감지 시 교반 RPM   */
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
#define DJ_STIR_FWD_MS          3000U     /* 건조 교반 CW 구간(정회전 3초)     */
#define DJ_STIR_STOP_MS         2000U     /* 건조 교반 정지 구간(정지 2초)     */
#define DJ_STIR_REV_MS          3000U     /* 건조 교반 CCW 구간(역회전 3초)    */
/* 신스펙: 건조 교반 = "CW 3초 + 정지 2초 ×5회 → CCW 3초, 반복"(=REPS 5).
 * (이전 기획서 단순 3-phase REPS 1에서 재정정, 2026-08-15 신스펙 항목1) */
#define DJ_STIR_FWD_REPS        5U        /* CW/정지 5회 후 CCW (신스펙)       */
#define DJ_S313_CW_MS           3000U     /* 식힘/배출 교반 CW 구간           */
#define DJ_S313_STOP_MS         1000U     /* 식힘/배출 교반 정지 구간         */
#define DJ_S313_CCW_MS          3000U     /* 식힘/배출 교반 CCW 구간          */
#define DJ_GRIND_RUN_MS         3000U     /* 1차 분쇄 구동 구간(CW)           */
#define DJ_GRIND_STOP_MS        2000U     /* 1차 분쇄 정지 구간               */
/* 110분↑ 고속(2000 CCW): 신스펙 "역회전 4초 / 정지 2초" 반복. 단, 최초 진입 시
 * 급격한 CW(1000)->CCW(2000) 반전 인러시(전류 실패)를 막기 위해 먼저 현재 방향으로
 * 감속(DECEL_MS) -> 역회전 개시 -> BLDC 슬루로 2000까지 상승시킨 뒤부터 4/2초 토글.
 * 정지 후 재기동도 슬루가 0->2000 소프트스타트하므로 급전류 없이 토글이 성립한다. */
#define DJ_GRIND_FINAL_DECEL_MS 1500U     /* 최초 역회전 전 감속 대기(1.5초)  */
#define DJ_GRIND_RUN_FINAL_MS   4000U     /* 2000 CCW 구동 4초                */
#define DJ_GRIND_STOP_FINAL_MS  2000U     /* 2000 CCW 정지 2초                */
/* 헹굼 교반은 식힘/배출과 동일한 313 패턴(DJ_S313_*)을 재사용한다. */

/* ---- 팬 (docx 7번) -----------------------------------------------------
 * 팬 3개 역할(2026-08-17 재정의):
 *   - FAN_VAPOR(방수팬)  : 수증기 루프 단독 제어(THERM3 100℃ ON / 84℃ OFF, duty 없음,
 *                          팬 먼저→STEP1/STEP2). HEAT 구간 한정.
 *   - FAN_EXHAUST(배기팬): THERM3 루프에서 분리. 시나리오 전체(헹굼~배출) 동안
 *                          15분 ON / 2분 OFF 독립 duty(구 10/5s 내부정화 → 15/2로 변경).
 *   - BLDC_FAN(식힘팬)   : 분쇄 동작 중 30/10초 duty. */
#define DJ_FANB_ON_MS           30000U    /* BLDC 식힘팬 ON 구간(30s)         */
#define DJ_FANB_OFF_MS          10000U    /* BLDC 식힘팬 OFF 구간(10s)        */
#ifndef DJ_FANX_ON_MS
#define DJ_FANX_ON_MS           (15UL * 60000UL)  /* 배기팬 ON 15분           */
#endif
#ifndef DJ_FANX_OFF_MS
#define DJ_FANX_OFF_MS          (2UL  * 60000UL)  /* 배기팬 OFF 2분           */
#endif

/* ---- 스테퍼 (냄새 관로=STEP1, 흡입제어=STEP2 / 역할 TBD) --------------- */
#ifndef DJ_DUCT_STEPS
#define DJ_DUCT_STEPS           512U      /* 관로 완전개폐 스텝수(TBD)        */
#endif
/* 스텝 간격: 2ms(500PPS)는 STEP2(35BYJ46) run 한계(~400PPS)를 넘겨 탈조 → 회전 안 함.
 * 테스트벤치 검증 3ms(333PPS)로 맞춤(STEP_MOTOR_RUN_PPS_MAX=400 이내). 실부하에서
 * 더 탈조하면 4~5ms로 낮출 것(무부하 기준값이라 부하 시 디레이팅 필요). */
#ifndef DJ_STEP_INTERVAL_MS
#define DJ_STEP_INTERVAL_MS     3U        /* 스텝 간격(333PPS, STEP2 안전)    */
#endif
/* 스텝 시작 램프: 정지 상태에서 곧바로 run PPS(333)로 밟으면 35BYJ46 자기기동
 * 한계(~100PPS=10ms)를 넘겨 실부하(수증기 흡입)에서 탈조("STEP2 원활치 않음").
 * 각 개폐 동작의 첫 RAMP_STEPS 스텝을 START_MS(100PPS)→RUN(DJ_STEP_INTERVAL_MS)로
 * 선형 가속한다. testbench는 무부하라 램프 없이 3ms로 돌았으나 실부하 STEP2 대비. */
#ifndef DJ_STEP_START_MS
#define DJ_STEP_START_MS        10U       /* 스텝 시작 간격(100PPS, 자기기동)  */
#endif
#ifndef DJ_STEP_RAMP_STEPS
#define DJ_STEP_RAMP_STEPS      40U       /* 가속 구간 스텝수(→333PPS)        */
#endif
/* STEP2(흡입, 35BYJ46)는 STEP1(24BYJ48)보다 기동토크 여유가 작고, DJ_HEAT 구간엔
 * 분쇄·교반 BLDC가 같은 24V 레일을 함께 쓴다. 무부하 testbench(3ms)에선 돌지만
 * 실부하+동시통전에선 탈조("진동만")하므로 STEP2 전용으로 더 보수적 페이싱을 쓴다.
 * (run 6ms=167PPS / start 20ms=50PPS. STEP1은 위 3/10ms 유지.) */
#ifndef DJ_STEP2_INTERVAL_MS
#define DJ_STEP2_INTERVAL_MS    6U        /* STEP2 run 간격(167PPS)           */
#endif
#ifndef DJ_STEP2_START_MS
#define DJ_STEP2_START_MS       20U       /* STEP2 시작 간격(50PPS)           */
#endif
/* 관로(STEP1) 개방 완료 후 STEP1 코일 해제: 홀딩전류를 없애 STEP2 구동에 레일
 * 전류를 양보(동시 2모터 통전 시 STEP2 탈조 방지). 기어드 스텝모터라 무통전에도
 * 기어 마찰/디텐트로 개방 위치를 대체로 유지. 드리프트가 문제면 0으로. */
#ifndef DJ_STEP_RELEASE_DUCT_ON_OPEN
#define DJ_STEP_RELEASE_DUCT_ON_OPEN 1
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
#define DJ_HS_START_IDX         4U        /* 동작 = HS5 (U24 P4, 0-based)     */
#endif
/* 시작 트리거(HS5 상승에지)를 이 파일 안에서 볼지 여부. 기본 0 = 중재자
 * (mode_arbiter.c)가 HS1~5를 항상 디코딩해 모드 전환/Dongjak_Start()/마개 이탈
 * 정지를 담당한다. 자세한 배경은 moeum.h MOEUM_HS_TRIGGER_INTERNAL 주석 참조.
 * 어느 쪽이든 dbg_force_start(벤치 강제 시작)는 항상 유효하다. */
#ifndef DJ_HS_TRIGGER_INTERNAL
#define DJ_HS_TRIGGER_INTERNAL  0
#endif
#ifndef DJ_TEMP_CH
#define DJ_TEMP_CH              0U        /* 처리통 써미스터1 CH0(히터/분쇄/식힘 주기준) */
#endif
/* 식힘 분쇄 정지 판정용 보조 써미스터(THERMISTOR2, PC1 = g_therm_c_d10[1]). 식힘은
 * THERM1·THERM2 둘 다 <80℃ 라야 분쇄 OFF(보수적, 신스펙 2026-08-17). 0=미사용
 * (THERM1 단독). 히터/분쇄 게이팅은 여전히 THERM1(DJ_TEMP_CH) 단독. */
#ifndef DJ_TEMP2_CH
#define DJ_TEMP2_CH             1U        /* THERMISTOR2 = g_therm_c_d10[1] (식힘 보조) */
#endif
#ifndef DJ_COOL_USE_THERM2
#define DJ_COOL_USE_THERM2      1U        /* 1 = 식힘 정지에 THERM2 AND 조건 추가 */
#endif
/* 수증기 배출 제어 전용 온도 채널. HW: [수증기 배출]용 팬은 J23에 연결된
 * Thermistor 3번(THERMISTOR3, PC2, ADC_CH_THERM3) -> g_therm_c_d10[2]. */
#ifndef DJ_VAPOR_TEMP_CH
#define DJ_VAPOR_TEMP_CH        2U        /* J23 = THERMISTOR3 = g_therm_c_d10[2] */
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

/* 도어 리미트(WHALL/THALL, PF3/PF4/PF2/PF5) 처리 모드. 모음(MOEUM_DOOR_LIMIT_
 * OPTIONAL)과 동일 기조.
 *   0 = 정상/스펙: 리미트 인식으로만 다음 단계 진행, 미도달 시 DJ_DOOR_TIMEOUT_MS
 *       후 DJ_ERROR (리미트 홀 게이팅). ← 실동작/검증 기본.
 *   1 = 벤치 육안 전용: 리미트 미도달 시에도 ERROR 대신 DJ_DOOR_BENCH_MS 만큼만
 *       구동 후 진행(센서 무시). 리밋홀 HW 검증 전 문 동작 관찰용 임시. */
#ifndef DJ_DOOR_LIMIT_OPTIONAL
#define DJ_DOOR_LIMIT_OPTIONAL  0
#endif
#ifndef DJ_DOOR_BENCH_MS
#define DJ_DOOR_BENCH_MS        4000U    /* (육안모드) 리미트 무시 시 도어 구동 관찰 시간 */
#endif
/* 배출문(TDoor) 전용 리미트 게이트 모드. 배출문 리미트는 THALL_OPEN(PF5)/
 * THALL_CLOSE(PF2). ★2026-08-16 THALL 리미트 있음 확인 -> 기본 0(리미트 게이팅).
 *   0 = 정상: THALL 인식으로만 다음 단계 진행, 미도달 시 DJ_DOOR_TIMEOUT_MS 후
 *       DJ_ERROR(배출 OPEN/CLOSE 코드). ← 실동작/검증 기본.
 *   1 = (벤치, THALL 미배선 시) 리미트 미도달이라도 DJ_DOOR_BENCH_MS 후 진행.
 * 배출문 개폐 방향/THALL 극성이 틀리면 0에서 리미트 미도달→타임아웃→DJ_ERROR 낙하
 * 하므로, 방향(TDoor_Open/Close)·DOOR_LIMIT_ACTIVE_HIGH를 벤치에서 확인할 것. */
#ifndef DJ_TDOOR_LIMIT_OPTIONAL
#define DJ_TDOOR_LIMIT_OPTIONAL 0
#endif

/* ---- 4.5 추가 투입 금지 / 비상 정지 (기획서 4.5) ----------------------- *
 * 처리 중 투입구 마개가 동작 위치를 벗어나면(=시작 홀 HS5 이탈) 즉시 전 액추에이터
 * 정지 후 안전 상태로. 고온이면 냉각팬만 유지하며 식힘(DJ_ABORTED).
 *   - 벤치 강제시작(dbg_force_start)은 HS 없이 돌 수 있어 lid 감시를 무장하지 않음.
 *   - 단발 글리치 오정지 방지: HS LOW를 DJ_LID_CONFIRM_SAMPLES회(100ms 주기) 연속
 *     확인해야 트리거. */
#ifndef DJ_LID_OPEN_ABORT
#define DJ_LID_OPEN_ABORT       1        /* 1=투입구 개방 비상정지 사용         */
#endif
#ifndef DJ_LID_CONFIRM_SAMPLES
#define DJ_LID_CONFIRM_SAMPLES  3U       /* HS LOW 연속 N회(≈300ms) 후 정지     */
#endif

/* ---- 메인 phase (디버거 관찰용) ---------------------------------------- */
typedef enum
{
	DJ_IDLE = 0,
	/* 초기 헹굼 2회 — moeum의 MoeumState처럼 세부단계를 최상위 상태로 노출.
	 * 2회는 고정 상수라 1차/2차를 통째로 언롤(변수 rinse_iter 폐지). 차이는 배수
	 * 교반시간뿐: 1차=DJ_RINSE1_DRAIN_MS(30초), 2차=DJ_RINSE2_DRAIN_MS(90초, 잔수 흡수).
	 * 2차 배수 완료 -> 곧바로 DJ_HEAT(별도 잔수배수 상태 없음). */
	DJ_RINSE1_CLOSE,      /* 1차 배수문 닫힘 구동 -> W-HALL-CLOSE 대기          */
	DJ_RINSE1_FILL,       /* 1차 급수밸브 ON -> 수위 감지 대기                  */
	DJ_RINSE1_FILL_EXTRA, /* 1차 수위 후 2초 추가급수 -> 밸브 OFF              */
	DJ_RINSE1_STIR,       /* 1차 교반 CW3/정지1/CCW3 2분 헹굼(패턴B)            */
	DJ_RINSE1_OPEN,       /* 1차 배수문 열림 구동 -> W-HALL-OPEN 대기           */
	DJ_RINSE1_DRAIN,      /* 1차 배수 교반 30초 -> 2차 시작                     */
	DJ_RINSE2_CLOSE,      /* 2차 배수문 닫힘 구동 -> W-HALL-CLOSE 대기          */
	DJ_RINSE2_FILL,       /* 2차 급수밸브 ON -> 수위 감지 대기                  */
	DJ_RINSE2_FILL_EXTRA, /* 2차 수위 후 2초 추가급수 -> 밸브 OFF              */
	DJ_RINSE2_STIR,       /* 2차 교반 CW3/정지1/CCW3 2분 헹굼(패턴B)            */
	DJ_RINSE2_OPEN,       /* 2차 배수문 열림 구동 -> W-HALL-OPEN 대기           */
	DJ_RINSE2_DRAIN,      /* 2차 배수 교반 90초(잔수 포함) -> HEAT              */
	DJ_HEAT,           /* 건조: 히터+교반+분쇄+수증기+팬 동시(온도/시간 게이팅)*/
	DJ_COOLDOWN,       /* 120분~ 식힘: 히터OFF, 교반 CW3/1/CCW3, 분쇄 80℃서OFF*/
	DJ_BIN_CHECK,      /* 배출 전 수거통 유무/횟수/높이 확인                  */
	DJ_DISCHARGE,      /* 130분~ 배출문 2분 개방+교반 배출 후 닫고 배수부 개방*/
	DJ_DONE,
	DJ_ERROR,
	DJ_ABORTED         /* 4.5 비상정지(투입구 개방/정지요청): 안전 식힘 후 IDLE */
} DongjakState;

/* DJ_ERROR 원인 코드(g_dongjak.err_code, RO). 복구는 Dongjak_ClearError()로 IDLE. */
typedef enum
{
	DJ_ERR_NONE        = 0,
	DJ_ERR_RINSE_CLOSE = 1,  /* 헹굼 배수문 닫힘 리미트 타임아웃 */
	DJ_ERR_RINSE_FILL  = 2,  /* 헹굼 급수 수위 미도달 타임아웃   */
	DJ_ERR_RINSE_OPEN  = 3,  /* 헹굼 배수문 열림 리미트 타임아웃 */
	DJ_ERR_HEAT_DOOR   = 4,  /* 건조 진입 배수문 닫힘 타임아웃   */
	DJ_ERR_DISCH_OPEN  = 5,  /* 배출문 열림 리미트 타임아웃      */
	DJ_ERR_DISCH_CLOSE = 6,  /* 배출문 닫힘 리미트 타임아웃      */
	DJ_ERR_DISCH_WOPEN = 7   /* 배출 후 배수부 개방 타임아웃     */
} DjErrCode;

/* 서브-FSM 위상 (헹굼은 DongjakState로 승격됨 - DjRinsePhase 폐지) */
typedef enum { DJ_STIR_FWD=0, DJ_STIR_STOPPED, DJ_STIR_REV } DjStirPhase;
typedef enum { DJ_GM_OFF=0, DJ_GM_COARSE, DJ_GM_FINE, DJ_GM_FINAL, DJ_GM_COOL } DjGrindMode;
typedef enum { DJ_GR_RUN=0, DJ_GR_STOP } DjGrindPhase;
typedef enum { DJ_VP_CLOSED=0, DJ_VP_OPEN_DUCT, DJ_VP_OPEN_AIR, DJ_VP_OPEN,
               DJ_VP_CLOSE_AIR, DJ_VP_CLOSE_DUCT } DjVaporPhase;
/* 배출 서브-FSM: 개방(CW) -> 교반 배출(HW 2분) -> 개방 유지 -> 닫기(CCW) -> 배수부.
 * ★DJ_DS_HOLD 추가로 CLOSE_T/OPEN_W 값이 2/3 -> 3/4로 밀렸다(디버거 watch 주의). */
typedef enum { DJ_DS_OPEN_T=0, DJ_DS_EXPEL, DJ_DS_HOLD, DJ_DS_CLOSE_T, DJ_DS_OPEN_W } DjDischPhase;

typedef struct
{
	volatile DongjakState  state;          /* DongjakState (RO)                    */
	uint32_t          state_since;
	uint32_t          scn_start;      /* 시나리오 시작 tick(절대시간 기준)    */

	volatile uint8_t  start_req;
	volatile uint8_t  dbg_force_start;/* 벤치 강제 시작(1회성, HS와 OR)       */
	volatile uint8_t  dbg_enter_heat; /* [디버그] DJ_HEAT 점프(1=도어대기부터/2=도어생략) 1회성 */
	volatile uint8_t  dbg_beep;       /* [TEST] N 쓰면 즉시 N회 비프(오디오 경로 확인) 1회성 */
	volatile int16_t  temp_d10;       /* 처리통 온도 CH0=THERM1(히터/분쇄/식힘, 에러 시 직전값 유지) */
	volatile uint8_t  temp_valid;     /* 0 = CH0 써미스터 에러(디바운스됨) -> 히터 강제 OFF */
	volatile int16_t  temp2_d10;      /* 처리통 온도 CH1=THERM2(식힘 분쇄정지 보조, 에러 시 직전값 유지) */
	volatile uint8_t  temp2_valid;    /* 0 = CH1 써미스터 에러 -> 식힘 판정에서 '안 식음' 취급 */
	volatile int16_t  vapor_temp_d10; /* 수증기 제어 온도 CH2=THERM3/J23(SenseTick) */
	volatile uint8_t  water_reached;
	volatile uint8_t  bin_fill_pct;
	uint8_t           hs_prev;

	/* 4.5 비상정지 */
	volatile uint8_t  abort_req;      /* 정지 요청(SenseTick/RequestStop 세팅) */
	volatile uint8_t  dbg_force_stop; /* 벤치 강제 정지(1회성) = 정지버튼 모사  */
	uint8_t           lid_guard;      /* 1=실제 HS 시작 -> 투입구 개방 감시 무장 */
	uint8_t           lid_low_cnt;    /* HS LOW 연속 카운트(디바운스)          */

	/* 에러 */
	volatile uint8_t  err_code;       /* DjErrCode: DJ_ERROR 원인(RO)         */
	volatile uint8_t  err_clear_req;  /* 1 쓰면 DJ_ERROR->IDLE 복구(1회성)    */

	uint16_t          cycle_count;    /* 처리 완료 횟수                       */

	/* 초기 헹굼: 세부단계·회차 모두 DongjakState(DJ_RINSE1_x · DJ_RINSE2_x)로 노출.
	 * 2회 고정 언롤이라 별도 카운터 없음. 타이밍=state_since. */

	/* 건조 */
	uint8_t           heat_started;   /* 0 = 배수문 닫는 중                    */
	uint32_t          heat_since;     /* 히터 구간 시작 tick(참고)            */
	uint8_t           test_beeped_hi; /* [TEST] 110분대(HISPEED) 비프 1회 래치 */

	/* 교반(건조=FWD/STOP/REV, 313=0/1/2 재사용). RPM은 110분↑ 27로 전환(패턴 동일) */
	uint8_t           stir_phase;
	uint32_t          stir_since;
	uint8_t           stir_reps;

	/* 분쇄 */
	uint8_t           grind_mode;     /* DjGrindMode                          */
	uint8_t           grind_phase;    /* DjGrindPhase(토글 모드)              */
	uint8_t           grind_final_ph; /* 110분 고속전환: 0=감속 대기, 1=역회전 확립(4/2 토글) */
	uint32_t          grind_since;
	uint32_t          grind_start;    /* 분쇄 개시(1차 3분 판정용)            */
	uint8_t           cool_phase;     /* 식힘: 0=뜨거움(교반 지속CW+분쇄1000CCW), 1=식음(교반313+분쇄OFF) */

	/* 수증기 */
	uint8_t           vapor_phase;    /* DjVaporPhase                         */
	uint32_t          vapor_step_since;
	uint16_t          vapor_step_cnt;

	/* 팬 */
	/* FAN_VAPOR는 vapor_phase로 관리(별도 상태 불필요). FAN_EXHAUST는 아래 fanx로 독립. */
	uint8_t           fanb_on;        /* BLDC 식힘팬 현재 ON?                 */
	uint32_t          fanb_since;
	uint8_t           fanx_on;        /* FAN_EXHAUST(배기팬) 현재 ON?(15/2 duty) */
	uint32_t          fanx_since;     /* 배기팬 현 구간 시작 tick             */

	/* 배출 */
	uint8_t           disc_phase;     /* DjDischPhase                         */
	uint32_t          disc_since;
	/* THALL 리미트 인식까지 실제로 걸린 시간(ms). 배출 진입 시 0으로 리셋되며,
	 * 0인 채로 다음 위상으로 넘어갔다면 = 리미트 미인식(135분 백스톱으로 전환).
	 * 타임아웃 값(DJ_DOOR_TIMEOUT_MS) 재산정용 실측치 — 디버거에서 관찰. */
	uint32_t          disc_open_ms;   /* 개방(CW) 리미트 도달까지 ms (0=미도달) */
	uint32_t          disc_close_ms;  /* 닫힘(CCW) 리미트 도달까지 ms (0=미도달)*/
	uint8_t           disc_pulse_off; /* 간헐 구동 현재 휴지구간?(1=코스트 중)  */
} DongjakCtx;

extern DongjakCtx g_dongjak;

/* ---- API ----------------------------------------------------------------- */
void         Dongjak_Init(void);
void         Dongjak_Start(void);
void         Dongjak_Abort(void);
void         Dongjak_RequestStop(void);          /* 4.5 비상정지 요청(정지버튼)  */
void         Dongjak_ClearError(void);           /* DJ_ERROR -> IDLE 복구        */
void         Dongjak_SenseTick(void);            /* 100ms, StartDefaultTask   */
void         Dongjak_MotorTick(uint32_t now_ms); /* 1ms,  StartMotorTask      */
DongjakState Dongjak_GetState(void);
uint8_t      Dongjak_IsBusy(void);

/* [디버그/벤치 전용] 헹굼을 건너뛰고 DJ_HEAT부터 시작.
 *   skip_door=0 : 정상 진입(배수문 닫힘+WHALL 대기부터). 리미트를 직접 조작할 때.
 *   skip_door=1 : 도어 닫힘 대기(WHALL)까지 생략, 교반/분쇄/수증기/팬을 정상
 *                 초기화한 '가열중' 상태로 즉시 진입.
 * 전제: g_app_mode=DONGJAK 유지(동작 마개 HS5 시작위치). scn_start=now로 리셋되어
 * 시나리오 경과(105/110/120분 분기)가 0부터 카운트된다.
 * ★런타임에서는 함수 직접호출 대신 변수 트리거를 권장(링커 제거·GDB call 불필요):
 *     g_dongjak.dbg_enter_heat = 1(도어대기부터) 또는 2(도어생략)
 *   -> MotorTick가 1회 소비하여 이 함수를 호출한다. */
void         Dongjak_DebugEnterHeat(uint8_t skip_door);

#ifdef __cplusplus
}
#endif

#endif /* SCENARIO_DONGJAK_H_ */
