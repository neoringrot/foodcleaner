#ifndef SCENARIO_DONGJAK_H_
#define SCENARIO_DONGJAK_H_

#include "main.h"
#include <stdint.h>
#include "dev_test.h"        /* ★[개발 검증] DJ_TEST_SHORT 단축 시험 스위치(§0.47) */
#include "rotation_port.h"   /* ★R3 개정4 ④: 헹굼 본교반·배수교반 = 회전수 운전 */
#include "rinse.h"           /* ★R3 5단계 [2026-09-21 §0.27]: 가이드 준비 탐색(R001~R006) */

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * dongjak - "동작"(2단계: 건조 -> 분쇄 -> 배출) 시나리오 상태머신.
 *
 * ★기준 원본: doc/R1/zerogeo_scenario.docx ([2단계] 동작 기능, 2026-07-30 김수빈)
 *   (사용자_시나리오_검토정리R3.md와 타이밍이 다르며, docx가 최신·우선.)
 * 관련 HW 미확정: Core/doc/하드웨어결정_잔여항목R3.md.
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
 * [개발 검증 전용 — 임시]  단축 시험은 dev_test.h 의 DJ_TEST_SHORT (§0.47).
 * 공정 시계 마커(아래 110/120/130/135분)는 **압축하지 않는다** — 단축 시험은 시간이 아니라
 * 조건(50/70℃ 확인 → 30 s 동시 운전 → 80℃ 미만)으로 건조~식힘을 끝낸다. dongjak.c DJ_HEAT/DJ_COOLDOWN 참조.
 * 단계 비프(구 [TEST] 기능)는 DJ_TEST_BEEP(기본 0) — LM4871_Beep 블로킹이 1 ms MotorTick 을 멈춘다.
 * ========================================================================== */

#ifndef DJ_TEST_BEEP
#define DJ_TEST_BEEP            0        /* [TEST] 1 = 110/120/130분 전환점 비프 + dbg_beep (블로킹) */
#endif
#if DJ_TEST_BEEP
#define DJ_TEST_BEEP_HZ         2000U             /* [TEST] 비프 주파수(Hz)   */
#define DJ_TEST_BEEP_MS         80U               /* [TEST] 비프 1회 길이(ms) */
#define DJ_TEST_BEEP_GAP_MS     80U               /* [TEST] 비프 간 간격(ms)  */
#endif

/* ---- 온도 임계 (0.1C 단위) --------------------------------------------- */
/* ★R3 C031 [2026-09-20] — **게이트 역할 폐기, 참조값으로만 잔존**.
 * 종전: 센서 65℃(=실제 80℃) 미만이면 dj_grind_heat_tick() 이 분쇄를 금지했다.
 * R3 원칙(`generated/config.h` "Hardware-owned thresholds: references only",
 * README "소프트웨어로 바이메탈이나 K2를 대체하지 않는다")에 어긋나므로,
 * 허가 판정은 **회로 관측**(dj_grind_allowed(), N2 해소)으로 옮겼다.
 * 이 상수는 앱 표시·`fakedev.py` 대조용 참조값으로만 남는다 — **구동을 막지 않는다.** */
#ifndef DJ_TEMP_GRIND_ON_D10
#define DJ_TEMP_GRIND_ON_D10    650   /* [참조] 65.0C(센서)=실제 80C           */
#endif
/* ★[벤치 전용 완화] 분쇄 허가 관측을 무시하고 바로 개시한다.
 * 실기에서 1 로 두면 **허가 없이 분쇄를 명령**하게 되고, ENABLE 이 LOW 라 FG 가
 * 오지 않아 bldc_ctrl 이 약 10초 만에 잼으로 오판해 BLDC_LOCKED(소프트락) 된다.
 * 70℃ 바이메탈을 붙일 수 없는 벤치에서 건조 패턴만 볼 때에만 1. **양산은 0.** */
#ifndef DJ_DRY_GATE_BYPASS
#define DJ_DRY_GATE_BYPASS      0
#endif
/* 식힘(COOLDOWN) 분쇄 OFF 기준. 신스펙: "80도부터 분쇄 꺼짐"(센서 80℃).
 * (이전 기획서 100℃에서 80℃로 재정정, 2026-08-15 신스펙) */
#ifndef DJ_TEMP_COOL_GRIND_OFF_D10
#define DJ_TEMP_COOL_GRIND_OFF_D10  800   /* 80.0C: 식힘 중 분쇄 OFF 기준      */
#endif
/* 식힘 분쇄 OFF(80℃) 판정의 바이메탈 OR 폴백.
 * ★REV02(R3 §4.6)에서 PF0 접점이 80℃ → **70℃** 로 교체되어 **근거를 잃었다** -
 * 80℃ 판정에 70℃ 접점을 OR 하면 기준이 어긋난다. 그래서 **기본값 0(센서 단독)** 으로
 * 내린다. 80℃ 전용 접점이 다시 생기거나 채널이 재배정되면 1 로 올리고
 * dj_cool_reached() 의 GPIO_EXTI_BIMETAL_70 을 그 채널로 바꾼다.
 * ⚠ PF0 는 70℃ 초과에서 LOW(active-low, 사용자 확인 2026-09-22) — EXTI 하강엣지는 가열 방향이다.
 *   식음 판정에 쓰려면 상승엣지(또는 레벨 HIGH 복귀)로 바꿔야 한다.
 * (이전: 2026-08-16 요구로 1. 센서 65℃≈실제80℃ 오프셋 보정 목적이었다.) */
#ifndef DJ_COOL_USE_BIMETAL80
#define DJ_COOL_USE_BIMETAL80   0
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

/* ---- 절대 경과시간 마커 (ms) — ★기산점 = heater_since (R3 C005·C041) ------
 * ★2026-09-21 (구현현황 §0.24): 종전 기산점은 **시나리오 시작(scn_start)** 이었다.
 * R3 개정4 C212/C216(검토서 §17.2 "시간 시작점")은 110/120분을 **"최초 히터 ON 명령이
 * 기판에 성공 전달된 시점(heater_since)"** 부터 세라고 확정했다(I07 해소). 온도 조절로
 * 히터가 꺼졌다 다시 켜져도, Jungji_Heater() 로 꺼져도 **다시 잡지 않는다**.
 * 벤더 레퍼런스: controller.c:492 `if (out->heater && !heater_started) heater_since = now`.
 *
 * 130분(배출 진입)·135분(배출 종료 백스톱)은 R3 에서 "안내 참고 시점"·"폐기"로 분류돼
 * 6단계에서 식힘 완료 이벤트로 대체될 값이지만, **그때까지는 같은 시계**로 센다 —
 * 110/120 만 옮기면 식힘 구간(120→130분)이 헹굼 시간(≈7분)만큼 줄어들기 때문이다.
 * 히터가 한 번도 켜지지 않으면 이 시계는 0 에 머문다(벤더와 동일 — 120분 판정이 오지 않는다).
 * scn_start 는 **앱 경과시간 표시용**으로만 남는다(protocol_r0.c). */
#ifndef DJ_T_HISPEED_MS
#define DJ_T_HISPEED_MS         (110UL * 60000UL) /* 110분: 분쇄 2000 역회전 (교반 RPM 전환은 C036 폐기) */
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
/* ★R3 C034(ST36 / P25, 09.16 config.c `initial_grind_ms`=120000): 초기 동시회전
 * 구간을 3분 -> 120초로 단축. 같은 항목의 분쇄 ON/OFF 구간은 DJ_GRIND_RUN_MS /
 * DJ_GRIND_STOP_MS(아래). 동시 교반(20RPM·역회전 없음)은 건조 패턴 소관이라
 * C035·C036(2단계)에서 처리한다 — 검토서 §16.1 1단계는 분쇄측만 지정. */
/* ★개정4 ⑤ [§0.31]: PROCESS 회전수 운전으로 대체 — 추적용(ZG_TRACKED_ONLY). 제어 미사용 */
#if ZG_TRACKED_ONLY
#ifndef DJ_GRIND_COARSE_MS
#define DJ_GRIND_COARSE_MS      (120UL * 1000UL)  /* R3 C034: 1차 분쇄 120초 → C093 초기 = 30회 운전 */
#endif
#endif

/* ---- 초기 헹굼/배수 (docx 2단계 2·3번) ---------------------------------
 * [R2 기록] 헹굼 2회는 고정 상수(rinse_iter 변수 폐지) - 1차/2차를 별도 상태로 언롤. ★R3 는 1회(아래 DJ_RINSE_COUNT, §0.35).
 * 모음 헹굼 교반은 사이클수 기준(11사이클 x 13s = 143초)이고 동작 헹굼은 시간 기준
 * (2분 이하의 정수 사이클 = 117초)이다. 배수는 교반하며 진행:
 *   1차 30초(물빼기), 2차 90초(물빼기 30초 + 잔수 60초) -> 별도 DJ_DRAIN_RESIDUAL 폐지. */
/* ★R3 C004 [2026-09-22, 구현현황 §0.35] — 사용자 지시: *"R3에서는 C004 에 대해 모음,동작시나리오에서
 *   헹굼 1회로 고정하고 버튼배정은 현재 시나리오가 정해지지 않음"*. 벤더 P01 rinse_repeats(0.3.0 config.c) = 1 과 같다.
 *   종전 R2 의 동작 헹굼 2회(1차 30 s / 2차 90 s 배수)는 폐지. 2 로 빌드하면 종전처럼 2회 돈다(미러 DJ_RINSE2_x).
 * TODO(C004, TBD: N18) 반복 설정 N(1~3)의 입력 경로 — 8버튼 중 어느 것·길게 누르기·앱 설정인지, 시작 전 래치,
 *   '11'·범위 밖이면 E10(C066). 버튼 기능 배정(I10·I14)이 정해지면 질의 후 이 상수를 래치값으로 교체한다. */
#ifndef DJ_RINSE_COUNT
#define DJ_RINSE_COUNT          1U        /* 헹굼 1회 — rinse.c RinseConfig.repeats(§0.30).
                                           * 앱 미러는 DJ_RINSE1_x/DJ_RINSE2_x 둘뿐이라 3 이상이면
                                           * 3회차부터 RINSE2 로 표시된다(제어는 정상) */
#endif
/* ★R3 C013 후속(2026-09-20): 1cycle 이 13초 -> 8초가 되면서 재계산.
 * 동작 헹굼 기준은 여전히 '2분'(2026-08-15 사용자 지정)이고, 2분을 넘지 않는
 * 정수 사이클로 내린다: floor(120/8) = 15 -> **15 x 8s = 120초 정확히**.
 * 13초 시절의 "9.2사이클째 중간에 멎는" 문제가 8초에서는 나눠떨어져 사라졌다.
 * 모음은 사이클수 기준(MOEUM_STIR_CYCLES)이라 방식이 다르다.
 * ※ R3 의 실제 헹굼 종료는 시간이 아니라 HW 신호(hw_water_allowed /
 *   hw_cycle_complete = 가이드 엣지 cnt 20/40)다 - C014, **5단계**에서 교체된다.
 *   이 상수는 그때까지의 과도기 값이다. */
/* ★★R3 개정4 ④ [2026-09-21, 구현현황 §0.26]: 아래 세 시간 종료 상수는 **추적용**이다.
 * 본교반 종료 = WASH 10회 운전(C013·C014), 배수교반 종료 = DRAIN 10회 운전(C018, 잔수배수
 * 개념 소멸). ZG_TRACKED_ONLY=0(기본)이면 정의가 빠진다 — 다시 쓰면 빌드가 깨지도록. */
#if ZG_TRACKED_ONLY
#ifndef DJ_RINSE_STIR_MS
#define DJ_RINSE_STIR_MS        120000UL  /* R3 C013: 헹굼 교반 120초 = 8s x 15 */
#endif
#ifndef DJ_RINSE1_DRAIN_MS
#define DJ_RINSE1_DRAIN_MS      30000UL   /* 1차 배수 교반 30초(물빼기)       */
#endif
#ifndef DJ_RINSE2_DRAIN_MS
#define DJ_RINSE2_DRAIN_MS      90000UL   /* 2차 배수 교반 90초(30초+잔수60초) */
#endif
#endif /* ZG_TRACKED_ONLY */
/* ★R3 C017(ST14 / P38 / TB-D01): 수위 감지 후 추가 급수 2초 -> **0초**.
 * 감지~밸브 닫기 ≤100ms 요구. 상태(DJ_RINSE*_FILL_EXTRA)는 남겨 두되 0ms라
 * 진입 즉시 통과한다(상태머신 구조 불변 — 상태 삭제는 5단계 헹굼 재작성 소관). */
#ifndef DJ_FILL_EXTRA_MS
#define DJ_FILL_EXTRA_MS        0U        /* R3 C017: 추가급수 없음(0초)      */
#endif
/* ★§0.30: 헹굼이 rinse.c 로 옮겨 가면서 추가급수 단계 자체가 없어졌다 — 0 이 아니면 빌드를 막는다. */
#if (DJ_FILL_EXTRA_MS != 0U)
#error "DJ_FILL_EXTRA_MS: R3 C017 로 추가급수는 0 초다. rinse.c 에는 추가급수 단계가 없다(§0.30)"
#endif

/* ---- ★§0.30 자동 자가세척 (09.16 O016 → R001~R019 → M001, 0.2.3·0.3.0 동일) ----------
 * 검토서 §12.1·§17.4. **매 정상 처리·배출 후**(R1 "5번째 처리 후"는 폐기 — 사용자 확정
 * 2026-09-22 "R3 가 업체의 최신 요청이므로 이를 따른다").
 *   진입: 배출문 닫힘 완료(DJ_DS_CLOSE_T) — 종전 "배수부 개방"(DJ_DS_OPEN_W) 자리.
 *   내용: 음식물 없이 헹굼 DJ_SELFCLEAN_REPEATS 회. 순서는 벤더 self_clean_guide_pending —
 *         배수문 닫힘 → 급수 → 준비 탐색(R001~R006) → WASH → 배수문 열림 + DRAIN.
 *   금지: 히터·분쇄·캠(C090). 다시 가열하지 않는다.
 *   진입 조건(★2026-09-22 재확인, 벤더 O009·O010·O016): **이번 배출의 공통 해제(K2·K3)** 가 성립해야 한다.
 *         REV02 해제 = U26-1 `DONE`(배출 120 s 타이머) AND `T-HALL-PULSE`(배출문 닫힘). MCU 는 K3 를 읽을 핀이
 *         없으므로 같은 두 입력으로 추정한다 — TIMER-OUT(PF8) 완료 수신 AND 배출문 THALL 실제 인식(20 s 상한
 *         종료가 아님). 불성립 → **E08**(DJ_ERR_RELEASE, 음성 05-06), 정상 완료로 세지 않는다(cycle_count 불변).
 *   급수: 공통 해제 확인 뒤 **온도 대기 없이** 급수한다. K3 는 50℃ 로 올라갈 때 서는 래치라, 해제 뒤 식는
 *         중에는 다시 서지 않는다(사용자 HW 확인 2026-09-22 — 50℃ 바이메탈 복귀 대기는 넣었다가 철회).
 *   종료: 배수문 열린 채 DJ_DONE → IDLE. 음성 04-01(시작)·04-02(끝).
 *   생략(REV02): 핀 캠 복귀(O012~O015, E06) — 캠 없음. I17 준비 요청/수락 — UE1 없음.
 * 0 이면 종전 동작(배출 뒤 배수부 개방으로 끝)으로 빌드된다. */
#ifndef DJ_SELFCLEAN_ENABLE
#define DJ_SELFCLEAN_ENABLE     1
#endif
#ifndef DJ_SELFCLEAN_REPEATS
#define DJ_SELFCLEAN_REPEATS    1U        /* 벤더 P01 rinse_repeats "저장 반복값" (0.3.0 = 1) */
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
 * ★R3 [2026-09-22, 구현현황 §0.37] 요약: 진입 = 130분 AND 식힘 완료 → 배출문 열기(무게 감시 시작) → 교반은 **회로**(U30 AUX)
 * → TIMER-OUT(PF8) 하강엣지 = 타이머 완료 → **즉시 닫기** → 해제 추정(닫기 전 TIMER-OUT AND 닫힘 THALL) → 자가세척 / E08.
 * 아래는 R2 기록.
 * 배출문 열림 = HW 2분 타이머 시작(그동안 HW가 배출문 제어를 차단). 교반 배출은
 * TIMER-OUT(PF8) 하강엣지 = 2분 종료로 멈추고, 배출문 "닫기"는 135분
 * (DJ_T_LATCH_MS)에 개시한다.
 * ★2026-08-17 재정의: 이전에는 TIMER-OUT 하강엣지에서 곧바로 문을 닫았으나,
 *   "130분부터 2분간 배출 / 135분이 되면 배출문이 다시 닫힌다" 지시에 맞춰
 *   2분 완료 시엔 교반만 정지하고 문은 열어둔 채(DJ_DS_HOLD) 135분을 기다린다.
 *   구 동작으로 되돌리려면 DJ_DISCH_CLOSE_AT_LATCH=0. */
/* ★R3 C049 D2 [2026-09-22, 구현현황 §0.37] — SW 백업은 **HW 타이머 허용 상한(P36 120 s, 108~132 s) 뒤**에만 온다.
 * 종전 120 s 는 정상인데 느린(120~132 s) 타이머보다 먼저 닫기를 시작해, 닫힘 원샷(T-HALL-PULSE)이 DONE 보다
 * 앞서 지나가 **Init-RST 가 안 나는데도** 뒤늦은 PF8 엣지로 "해제됨"을 추정했다. 백업으로 닫은 배출은 해제 불성립(E08). */
#ifndef DJ_DISCHARGE_STIR_MS
#define DJ_DISCHARGE_STIR_MS    140000UL  /* TIMER-OUT SW 백업 = P36 상한 132 s + 8 s */
#endif
/* TIMER-OUT(PF8) 하강엣지가 오지 않아도 SW 2분(DJ_DISCHARGE_STIR_MS)으로 교반을
 * 멈출지. TIMER-OUT 극성/플로팅(NOPULL)이 HW 미검증이라 기본 1(백업 사용).
 * 0이면 TIMER-OUT만 신뢰 -> 미도달 시 교반이 135분까지 계속 돈다. */
#ifndef DJ_DISCH_STIR_SW_BACKUP
#define DJ_DISCH_STIR_SW_BACKUP 1
#endif
/* 배출문 닫기 개시 시점.
 *   0 = 타이머 완료(TIMER-OUT↓) **즉시** 닫기 — ★R3 C051 D3 [§0.37] 기본(벤더 O008 "타이머 완료 후 닫기, 135분 개념 없음").
 *   1 = (R2 2026-08-17) 135분(DJ_T_LATCH_MS)까지 열어 두었다가 닫기. DJ_DS_HOLD 경유. */
#ifndef DJ_DISCH_CLOSE_AT_LATCH
#define DJ_DISCH_CLOSE_AT_LATCH 0
#endif
/* ★R3 C045·C046 D4 [§0.37] — 배출 진입 = 130분(DJ_T_DISCHARGE_MS) **AND 식힘 완료**(cool_phase 1 = THERM1·2 < 80℃,
 * dj_cool_reached). R3: "130분은 안내 참고 시점일 뿐 자동 문 개방 허가가 아니다"(C045), 벤더 O001
 * `request_discharge && cooling_complete`. 식힘 완료 기준 자체는 I08(기존 펌웨어 값). 0 = 종전(130분만).
 * TODO(C046, TBD: I10) 배출 **요청** 방식(자동/버튼) 미배정 — 지금은 조건이 되면 자동 요청으로 본다. */
#ifndef DJ_DISCH_REQUIRE_COOLED
#define DJ_DISCH_REQUIRE_COOLED 1
#endif
/* ★R3 C048·C082 D1 [§0.37] — 배출 중 교반 제어권은 **회로**다. 배출문이 닫힘 위치를 떠나는 순간 U33 FF2 → U30 SEL=B:
 * M2 EN=H(R181)·DIR=H(R183) 고정·nBRAKE=/DONE·PWM=AUX(20 kHz AND RUN-EN), 공통 해제(Init-RST→Q48)에서 SEL=A 복귀.
 * 그동안 MCU 의 M2 명령은 무시된다 — 종전처럼 313 패턴·폐루프를 돌리면 AUX 지연·제동 구간(FG 없음)에서 잼 판정 →
 * 소프트락 → §0.29 err 11 로 **배출문이 열린 채** 멈출 수 있었다. 1 = 배출 전 구간 M2 무명령(FG 관측만, disc_aux_fg).
 * 0 = 종전 313 구동(참고용 — 회로가 무시한다). */
#ifndef DJ_DISCH_STIR_BY_CIRCUIT
#define DJ_DISCH_STIR_BY_CIRCUIT 1
#endif
/* ★R3 C050·C053·C078 D5 [§0.37] — 무게(PC3 weight-ADC, J40) 유효 변화 **연속 P37 2.108 s** → f_weight 래치(벤더 O005).
 * 감시 구간 = 배출문 열기 시작(DJ_DISCHARGE 진입, O002) ~ 타이머 완료(O007). 기준값 = 구간 첫 샘플, 변화 = |raw - 기준|.
 * 짧은 신호 합산 없음(끊기면 0 부터, TB-J05), 래치 후 신호가 내려가도 유지(TB-J06), 다음 배출 시작에서 초기화(TB-J07).
 * TBD: N20 — 센서 종류·"유효 변화" 임계(ADC count)·방향·전원(PE5 NEW-SENSOR, N9)이 원본에 없다. 임계는 자리값이다.
 * DJ_RELEASE_REQUIRE_WEIGHT 0 = 공통 해제 추정에 **넣지 않는다**(관측만) — REV02 해제 AND 에 무게가 없어(N7) 넣으면
 * 회로는 풀었는데 MCU 만 E08 을 내게 된다. N7·N20 회신 뒤 1. */
#ifndef DJ_WEIGHT_ENABLE
#define DJ_WEIGHT_ENABLE        1
#endif
#ifndef DJ_WEIGHT_DELTA_COUNTS
#define DJ_WEIGHT_DELTA_COUNTS  100U      /* TBD: N20 — 유효 변화 임계(12-bit count). 자리값 */
#endif
#ifndef DJ_WEIGHT_CONTINUOUS_MS
#define DJ_WEIGHT_CONTINUOUS_MS 2108U     /* P37 — 회로(UW6) 명목값을 SW 로 */
#endif
#ifndef DJ_RELEASE_REQUIRE_WEIGHT
#define DJ_RELEASE_REQUIRE_WEIGHT 0
#endif
/* 배출문(TDoor) 개폐 중 THALL 리미트 미인식 시 처리.
 *   1 = 타임아웃 없이 리미트 인식까지 계속 구동(신지시 - 배출에서 DJ_ERROR로
 *       전체 프로세스를 멈추지 않는다). 기본.
 *   0 = (구 동작) 고정 타임아웃 후 DJ_ERROR(DJ_ERR_DISCH_OPEN/CLOSE). 도어 완료
 *       판정이 tdoor.h TDOOR_*_MAX_MS(20s) 상한으로 옮겨져 지금은 쓰이지 않는다.
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
/* ★R3 2단계(2026-09-20)로 교반 RPM 지형이 다시 갈렸다. 한눈에:
 *   - 헹굼             = DJ_RINSE_STIR_RPM    (30)  <- config.c `rinse_rpm`
 *   - 건조(초기·본)    = DJ_DRY_STIR_RPM      (20)  <- config.c `dry_rpm`  [C034·C035]
 *   - 식힘 80℃↑ 지속CW = DJ_COOL_HOT_STIR_RPM (20)
 *   - 식힘 80℃↓ 패턴   = DJ_COOL_STIR_RPM     (20)  [C043]
 *   - 배출             = DJ_STIR_RPM          (30)  <- R3 미확정(P46), R2값 유지
 * 2026-08-26의 "전 구간 30RPM 통일"은 R3 기준(09.16)에서 건조·식힘이 20으로
 * 내려가면서 사실상 해체됐다. DJ_STIR_RPM 은 이제 배출 전용이다. */
#ifndef DJ_STIR_RPM
#define DJ_STIR_RPM             30U       /* 배출 교반 30RPM (R3 미확정 P46)   */
#endif
/* ★R3 C036(ST38, 검토서 §4.7 · §7 Q6): 110분↑ 교반 고속 전환을 **폐기**.
 * 09.16 `config.c` 에 해당 상수가 아예 없고, 레퍼런스 `controller.c` 의 D009·D010
 * 이 둘 다 `dry_rpm`(20) 으로 교반한다 - R3 는 이 기능을 모른다. 그래서
 * DJ_STIR_RPM_HISPEED(40) 와 DJ_HEAT 의 RPM 분기를 함께 제거했다.
 * (40RPM 은 실기 검증도 없었다 - 확인된 최고 지령은 27, HW미검증 1-13 ①.)
 * ※ 110분 **분쇄** 고속(2000 역 4s/2s)은 C037 '존치'라 그대로다. 시각 기준
 *   DJ_T_HISPEED_MS 도 분쇄가 계속 쓰므로 남긴다. */
/* 식힘 진입 직후(온도 80℃ 이상, cool_phase=0) 의 "지속 CW" 전용 RPM.
 * 2026-08-26 지시는 '식힘 교반(80℃ 미만)'을 명시했으므로 이 구간은 20RPM 유지다.
 * 그전에는 DJ_STIR_RPM 을 같이 썼는데, 그 값이 30 으로 오르면서 분리했다. */
#ifndef DJ_COOL_HOT_STIR_RPM
#define DJ_COOL_HOT_STIR_RPM    20U       /* 식힘 80℃↑ 지속CW 20RPM(미변경)   */
#endif
/* ★R3 C034·C035(ST36~ST37 / P17·P22, config.c `dry_rpm`=20): 건조 구간 교반은
 * 초기 동시회전(120초)과 본 건조 모두 **20RPM**. R2는 30(110분↑ 40)이었다. */
/* ★개정4 ⑤ [§0.31]: PROCESS 회전수 운전으로 대체 — 추적용(ZG_TRACKED_ONLY). 제어 미사용 */
#if ZG_TRACKED_ONLY
#ifndef DJ_DRY_STIR_RPM
#define DJ_DRY_STIR_RPM         20U       /* R3 C034·C035: 건조 교반 20RPM → ⑤ DJ_PROCESS_STIR_RPM */
#endif
#endif
/* ★개정4 ⑤ [2026-09-22 §0.31] C092 / P47 `process_stir_rpm`=30 (25~30, 벤더 config_valid 가 범위 검사):
 * 건조 PROCESS 회전수 운전(D008~D010)의 교반 지령. 날개 rpm(§0.22). */
#ifndef DJ_PROCESS_STIR_RPM
#define DJ_PROCESS_STIR_RPM     30U       /* P47: 건조 PROCESS 교반 30rpm      */
#endif
/* ★⑤ P22 `initial_grind_rpm`=1200: PROCESS 분쇄(교반과 같은 방향·같은 가동 구간). 초기 30회와 이후 반복 공통.
 * N16 ◐ — 1200rpm 무부하 상승만 확인(2026-09-22). 110분 late 는 DJ_GRIND_FINAL_RPM(P28 2000). */
#ifndef DJ_GRIND_PROCESS_RPM
#define DJ_GRIND_PROCESS_RPM    1200U     /* P22: 건조 PROCESS 분쇄 1200rpm    */
#endif
/* ★⑤ 벤더 ZG_D005~D008 의 허가 조건은 `grind_allowed && outlet_closed`. outlet = 배출문(T-HALL-CLOSE, PF2).
 * 1(기본) = 배출문 닫힘도 허가 조건에 넣는다(벤더와 같음). 0 = 종전(분쇄 허가만). */
#ifndef DJ_DRY_REQUIRE_OUTLET
#define DJ_DRY_REQUIRE_OUTLET   1
#endif
/* ★R3 C026 H1 [2026-09-22, 구현현황 §0.36] — 히터 명령 = 온도·히스테리시스 **AND 배수문 닫힘**(PF3 레벨, WDoor_AtClose).
 * REV02 U35 가 이미 `W-HALL-CLOSE-1 AND HT-POWER` 로 히터 전원을 끊는다. W-HALL-CLOSE-1 은 PF3 와 **같은 센서(J6)**를
 * U34 로 반전한 것이라 소프트웨어 AND 가 회로와 어긋나지 않는다. 벤더 controller.c 464(`drain_closed` AND)·492(실제
 * 출력이 참일 때만 heater_since 기산)와 같다. 종전엔 배수문이 안 닫혀 회로가 막아도 PA12 가 ON 이라 공정 시계가 흘렀다.
 * 핀 조건(C025)은 REV02 에 캠이 없어 없다. 0 = 종전(배수문 미확인). */
#ifndef DJ_HEATER_REQUIRE_DRAIN_CLOSED
#define DJ_HEATER_REQUIRE_DRAIN_CLOSED 1
#endif
/* ★R3 C033 H2 [2026-09-22, §0.36] — "허가만으로 구동 금지 / 허가와 명령 분리". 벤더 controller.c 465 는 **모든 단계**에서
 * `!grind_allowed || !outlet_closed` 면 분쇄를 세운다. 종전 식힘 80℃↑ 분쇄(1000 CCW)는 허가를 보지 않아, 허가가 빠지면
 * ENABLE LOW → FG 없음 → ≈10 s 뒤 소프트락 → §0.29 `err_code 10` 로 공정 전체가 멈췄다.
 * 1 = 식힘 분쇄도 dj_dry_permitted()(분쇄 허가 && 배출문 닫힘) 동안만 — 빠지면 분쇄만 멈추고(에러 아님) 돌아오면 다시.
 * 0 = 종전. */
#ifndef DJ_COOL_GRIND_REQUIRE_PERMIT
#define DJ_COOL_GRIND_REQUIRE_PERMIT 1
#endif
/* ★R3 C043(ST41 / TB-H02, 검토서 §4.8): 식힘 80℃ **미만** 교반을
 * 30RPM CW6/정지1/CCW6 -> **20RPM 정3/정지1/역3/정지1**(헹굼과 같은 구간)으로.
 * RPM 은 `dry_rpm`(20), 구간은 `rinse_*`(3/1/3) - 레퍼런스 `controller.c` ZG_D013 이
 * `alternate(dry_rpm, rinse_forward_ms, rinse_stop_ms, rinse_reverse_ms, 1)` 라
 * 혼합 인용이 의도였음이 확인됐다(2026-09-20, §0.12.4 ③ 해소).
 * ★2026-09-20 2단계 정정: **4구간**이다(I06 해소 - 아래 DJ_RINSE_* 주석 참조).
 * 1단계에서 3구간(7초)으로 넣었던 것을 8초 cycle 로 고쳤다. */
#ifndef DJ_COOL_STIR_RPM
#define DJ_COOL_STIR_RPM        20U       /* R3 C043: 식힘 80℃↓ 교반 20RPM    */
#endif
/* 기획서 4.2: "부하가 크면 약 30RPM 검토". 부하감지(4.6)는 별도 미구현 서브시스템
 * 이라 현재 스위칭 소스가 없어 대기 상태(값만 기록).
 * ⚠️ 2026-08-26 기준 상시 RPM 이 이미 30 이라 이 값은 더 이상 '증속'이 아니다.
 * 부하감지를 구현할 때 목표값부터 다시 정할 것. */
#ifndef DJ_STIR_RPM_HILOAD
#define DJ_STIR_RPM_HILOAD      30U       /* (대기) 부하 큼 감지 시 교반 RPM   */
#endif
/* ★헹굼 교반 이력: 25RPM CW3/정지1/CCW3(1cycle 7s)
 *   -> 2026-08-25 30RPM CW4/정지1/CCW4(1cycle 9s)
 *   -> 2026-08-26 30RPM CW6/정지1/CCW6(1cycle 13s)  ← 현재. 모음과 같은 구간.
 *   식힘/배출 교반(DJ_S313_*)은 바뀌지 않으므로 구간 상수를 분리해 두었다. */
#ifndef DJ_RINSE_STIR_RPM
#define DJ_RINSE_STIR_RPM       30U       /* 헹굼 교반 ~30RPM(1단계 동일)     */
#endif
/* ★개정4 ⑤ [§0.31]: PROCESS 회전수 운전으로 대체 — 추적용(ZG_TRACKED_ONLY). 제어 미사용 */
#if ZG_TRACKED_ONLY
#ifndef DJ_GRIND_COARSE_RPM
#define DJ_GRIND_COARSE_RPM     1500U     /* 1차 분쇄 CW → ⑤ DJ_GRIND_PROCESS_RPM(1200) */
#endif
#endif
#ifndef DJ_GRIND_FINE_RPM
#define DJ_GRIND_FINE_RPM       1000U     /* P26 연속 분쇄 — ⑤ 이후 식힘 80℃↑(DJ_GM_COOL) 전용 */
#endif
#ifndef DJ_GRIND_FINAL_RPM
#define DJ_GRIND_FINAL_RPM      2000U     /* P28 late: 110분↑ 교반 반대 방향 (⑤ PROCESS late_rpm) */
#endif

/* ---- 서브-패턴 구간(ms) ------------------------------------------------ */
/* ★★ I06 해소 (2026-09-20) - 교반 패턴은 전부 **4구간**이다 ★★
 * 검토서 §4.x가 "역→정 전환에도 정지 1s를 넣을지 I06 미정"으로 남겼던 항목.
 * 09.16 레퍼런스 `src/controller.c` 의 `alternate()` 가 답을 갖고 있다:
 *     phase 0=정(fwd) -> 1=정지(stop) -> 2=역(rev) -> 3=정지(stop) -> 0
 *     "Each direction change has its own stop interval" (원문 주석)
 * 즉 문서가 "정3/정지1/역3"이라 3구간처럼 적었어도 1cycle 은 3+1+3+1 = **8초**다.
 * group N 을 주면 (정+정지)×N -> 역 -> 정지 로 묶인다(건조 = 5묶음).
 * R2의 3구간 FSM(dj_stir313_tick: CW->정지->CCW->CW)은 배출에만 남는다.
 * 구현: dongjak.c `dj_stir_alt_tick()` - 레퍼런스 alternate() 를 그대로 옮긴 것. */
#define DJ_S313_CW_MS           6000U     /* 배출 교반 CW 구간(6초, R3 미확정) */
#define DJ_S313_STOP_MS         1000U     /* 배출 교반 정지 구간               */
#define DJ_S313_CCW_MS          6000U     /* 배출 교반 CCW 구간(6초)           */
/* ★R3 C035(ST37 / P17~P21, config.c `dry_*`): 건조 교반을 **패턴 A 로 복원**.
 * §0.9.2(2026-08-26)에서 "CW3/정지2 ×5회 -> CCW3" 를 폐지하고 313 으로 통일했는데,
 * R3 는 그 이전 패턴을 쓴다 - 정회전만 3초 -> **5초**로 늘었다. 09.16 README:
 * "교반 정회전도 5초로 맞추며 기존 2초 정지·5회 묶음·역회전 3초는 유지합니다."
 * 1cycle = (5+2)×5 + 3 + 2 = **40초**(정 25s / 역 3s). */
/* ★개정4 ⑤ [§0.31]: PROCESS 회전수 운전으로 대체 — 추적용(ZG_TRACKED_ONLY). 제어 미사용 */
#if ZG_TRACKED_ONLY
#define DJ_DRY_FWD_MS           5000U     /* R3 C035: 건조 교반 정회전 5초    */
#define DJ_DRY_STOP_MS          2000U     /* R3 C035: 건조 교반 정지 2초      */
#define DJ_DRY_REV_MS           3000U     /* R3 C035: 건조 교반 역회전 3초    */
#define DJ_DRY_FWD_GROUP        5U        /* R3 C035: 정회전 5회 묶음         */
#endif
/* ★R3 C043: 식힘 80℃ 미만 전용 구간(정3/정지1/역3). 건조·배출의 DJ_S313_* 와
 * 분리한 이유는 위 DJ_COOL_STIR_RPM 주석 참조. dongjak.c: dj_stir_cool_tick(). */
#define DJ_COOL_S313_CW_MS      3000U     /* R3 C043: 식힘 교반 CW 3초        */
#define DJ_COOL_S313_STOP_MS    1000U     /* R3 C043: 식힘 교반 정지 1초      */
#define DJ_COOL_S313_CCW_MS     3000U     /* R3 C043: 식힘 교반 CCW 3초       */
/* ★R3 C013(P09~P11 / TB-A01, config.c `rinse_forward/stop/reverse_ms`):
 * 헹굼 교반을 30RPM CW6/정지1/CCW6(13초) -> **30RPM 정3/정지1/역3/정지1(8초)** 로.
 * 이력: 25RPM 3/1/3(7s) -> 2026-08-25 30RPM 4/1/4(9s) -> 2026-08-26 30RPM 6/1/6(13s)
 *       -> 2026-09-20 R3 30RPM 3/1/3+정지1(8s). 구간값은 첫 값으로 돌아왔고
 *       RPM 30 과 **역→정 정지(I06)** 가 더해진 형태다.
 * 정지 구간은 정→역·역→정 양쪽에 같은 값(DJ_RINSE_STOP_MS)이 쓰인다.
 * 모음(moeum.h MOEUM_STIR_*)도 같은 구간으로 맞췄다. */
#define DJ_RINSE_CW_MS          3000U     /* R3 C013: 헹굼 교반 정회전 3초    */
#define DJ_RINSE_STOP_MS        1000U     /* R3 C013: 헹굼 교반 정지 1초(양쪽)*/
#define DJ_RINSE_CCW_MS         3000U     /* R3 C013: 헹굼 교반 역회전 3초    */
/* ★R3 C034(P23·P24, config.c `initial_grind_on_ms`=5000 / `off_ms`=2000):
 * 1차 분쇄 구동 3초 -> **5초**, 정지 2초는 불변. RPM 1500도 불변. */
/* ★개정4 ⑤ [§0.31]: PROCESS 회전수 운전으로 대체 — 추적용(ZG_TRACKED_ONLY). 제어 미사용 */
#if ZG_TRACKED_ONLY
#define DJ_GRIND_RUN_MS         5000U     /* R3 C034: 1차 분쇄 구동 5초(CW)   */
#define DJ_GRIND_STOP_MS        2000U     /* 1차 분쇄 정지 구간(2초, 불변)    */
#endif
/* 110분↑ 고속(2000 CCW): 신스펙 "역회전 4초 / 정지 2초" 반복. 단, 최초 진입 시
 * 급격한 CW(1000)->CCW(2000) 반전 인러시(전류 실패)를 막기 위해 먼저 현재 방향으로
 * 감속(DECEL_MS) -> 역회전 개시 -> BLDC 슬루로 2000까지 상승시킨 뒤부터 4/2초 토글.
 * 정지 후 재기동도 슬루가 0->2000 소프트스타트하므로 급전류 없이 토글이 성립한다. */
/* ★개정4 ⑤ [§0.31]: PROCESS 회전수 운전으로 대체 — 추적용(ZG_TRACKED_ONLY). 제어 미사용 */
#if ZG_TRACKED_ONLY
#define DJ_GRIND_FINAL_DECEL_MS 1500U     /* 최초 역회전 전 감속 대기(1.5초)  */
#define DJ_GRIND_RUN_FINAL_MS   4000U     /* 2000 CCW 구동 4초                */
#define DJ_GRIND_STOP_FINAL_MS  2000U     /* 2000 CCW 정지 2초                */
#endif
/* ↑ late 는 벤더 0.3.0 에서 "역4s/정지2s" 가 폐기되고 교반과 같은 가동 구간(2회전+정지 2 s)이 됐다.
 *   감속 대기도 필요 없다 — 엔진이 late 적용 전에 두 모터 정지 확인 + 2 s 를 기다린다(C037·C090). */
/* 헹굼 교반은 ★개정4 ④ 부터 회전수 운전(WASH/DRAIN) — DJ_RINSE_* 는 앱 보고·tb_rinse 용. */

/* ---- 팬 (docx 7번) -----------------------------------------------------
 * 팬 3개 역할(2026-08-17 재정의):
 *   - FAN_VAPOR(방수팬)  : 수증기 루프 단독 제어(THERM3 100℃ ON / 84℃ OFF, duty 없음,
 *                          팬 먼저→STEP1/STEP2). HEAT 구간 한정.
 *   - FAN_EXHAUST(배기팬): THERM3 루프에서 분리. 시나리오 전체(헹굼~배출) 동안
 *                          독립 duty. 2026-08-17 에 10/5s → 15분/2분으로 올렸다가
 *                          R3 C056 으로 10초/5초 복귀(아래 DJ_FANX_*).
 *   - BLDC_FAN(식힘팬)   : 30/10초 duty. 구동 조건은 R3 C057 로 "분쇄 시작 OR ≥80℃"(아래). */
/* ★R3 C057(BG2 / P41·P42): duty 30s/10s 는 09.16 config.c `motor_fan_*` 와
 * 이미 같아 **값 불변**. 바뀌는 것은 구동 조건 — "분쇄 시작 OR 온도 ≥ 80℃"이고
 * 분쇄 간헐 정지 구간에도 duty 주기를 유지한다(기산점 리셋 금지).
 * 구현은 dongjak.c: dj_fan_bldc_tick() / ctx.fanb_active. 임계는 아래
 * DJ_TEMP_BLDCFAN_ON_D10(800) 재사용. */
#define DJ_FANB_ON_MS           30000U    /* BLDC 식힘팬 ON 구간(30s, 불변)   */
#define DJ_FANB_OFF_MS          10000U    /* BLDC 식힘팬 OFF 구간(10s, 불변)  */
/* ★R3 C056(BG1 / P39·P40, config.c `purification_on_ms`=10000 / `off_ms`=5000):
 * 공기정화팬(=FAN_EXHAUST PG2) duty 를 15분/2분 -> **10초/5초**로. R2가 2026-08-17
 * 에 10/5초에서 15분/2분으로 올렸던 것을 R3 기준(09.16 최종)으로 되돌린다 — N5 는
 * "09.16 이 최종"으로 확정(사용자 지시 2026-09-20). 구간은 동작 T0~배출문 닫힘,
 * 모음 미적용(moeum.c 에 팬 출력 없음 — 이미 충족). 종료를 '배출문 실제 닫힘'으로
 * 옮기는 것은 6단계(배출) 소관이라 여기서는 duty 만 바꾼다. */
#ifndef DJ_FANX_ON_MS
#define DJ_FANX_ON_MS           (10UL * 1000UL)   /* R3 C056: 배기팬 ON 10초  */
#endif
#ifndef DJ_FANX_OFF_MS
#define DJ_FANX_OFF_MS          (5UL  * 1000UL)   /* R3 C056: 배기팬 OFF 5초  */
#endif

/* ---- 스테퍼 (냄새 관로=STEP1, 흡입제어=STEP2 / 역할 TBD) --------------- */
/* ★2026-08-25 개폐 종료 조건을 "스텝수" -> "구동시간"으로 변경.
 * 벤치(tb_stepmotor)에서 실기구로 확정·빌드검증한 값과 동일하게 맞춘다:
 *   STEP1(관로) : 방향 무관 15초        (tb_step1_run_ms)
 *   STEP2(흡입) : 열림 1초 / 닫힘 2초   (tb_step2_run_open_ms / _close_ms)
 * 기구에 스토퍼가 있어 "시간"이 스펙이고 스텝수는 페이싱에 딸린 부산물이므로,
 * 아래 DJ_STEP*_MS 가 1차 기준이다. 페이싱(DJ_STEP*_INTERVAL_MS)을 바꾸면 같은
 * 시간 안에 밟는 스텝수(=이동량)가 함께 바뀌니 페이싱 변경 시 재확인할 것. */
#ifndef DJ_STEP1_RUN_MS
#define DJ_STEP1_RUN_MS         15000U    /* STEP1 관로 개/폐 각 15초         */
#endif
#ifndef DJ_STEP2_OPEN_MS
#define DJ_STEP2_OPEN_MS        1000U     /* STEP2 흡입 열림 1초              */
#endif
#ifndef DJ_STEP2_CLOSE_MS
#define DJ_STEP2_CLOSE_MS       2000U     /* STEP2 흡입 닫힘 2초              */
#endif
/* [폐지 2026-08-25] 개폐 종료가 시간 기준으로 바뀌어 더 이상 참조되지 않는다.
 * 값을 바꿔도 동작이 바뀌지 않으니 위 DJ_STEP*_MS 를 고칠 것. */
#ifndef DJ_DUCT_STEPS
#define DJ_DUCT_STEPS           512U      /* (구) 관로 완전개폐 스텝수        */
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
 * 분쇄·교반 BLDC가 같은 24V 레일을 함께 쓴다. 그래서 한때 run 6ms(167PPS)로
 * 디레이팅했으나, ★2026-08-25 개폐 종료가 "시간"(열림 1s/닫힘 2s) 기준으로 바뀌면서
 * 느린 페이싱이 곧 이동량 부족을 뜻하게 됐다 → 벤치 확정값과 같은 run 3ms(333PPS)로
 * 되돌린다(사용자 지시). 시작 간격 20ms(50PPS) 램프는 자기기동 여유로 유지.
 * 실부하에서 다시 탈조("진동만")하면 6ms로 되돌리고 대신 DJ_STEP2_OPEN_MS/_CLOSE_MS를
 * 늘려 이동량을 보전할 것. (STEP1은 위 3/10ms 그대로.) */
#ifndef DJ_STEP2_INTERVAL_MS
#define DJ_STEP2_INTERVAL_MS    3U        /* STEP2 run 간격(333PPS, 벤치와 동일) */
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
/* ★2026-09-22: 수거통 유무 = HS7(U24 P6, 센서 HW ✅ 2026-08-15). 없으면 DJ_BIN_CHECK 에서
 * 배출문을 열지 않고 기다리며 03-04 "수거통을 위치에 넣어 주세요" 를 **없어질 때마다 1회** 안내.
 * HallSensor_Get() 은 극성 정규화된 "자석 감지 = 1" 이다. 수거통에 자석이 있어 **감지 = 수거통 있음**
 * 으로 가정했다 — TBD: 실기 확인(HW미검증 1-35 D1). 반대면 DJ_BIN_PRESENT_LEVEL 을 0 으로.
 * 0 이면 종전대로 HS7 을 보지 않는다(처리횟수·분말높이만). */
#ifndef DJ_BIN_USE_HS7
#define DJ_BIN_USE_HS7          1
#endif
#ifndef DJ_HS_BIN_IDX
#define DJ_HS_BIN_IDX           6U        /* HS7 = U24 P6 (0-based)            */
#endif
#ifndef DJ_BIN_PRESENT_LEVEL
#define DJ_BIN_PRESENT_LEVEL    1U        /* TBD: HallSensor_Get 이 값이면 수거통 있음 */
#endif
/* ★2026-09-22: 도어 리미트 미인식 **안내만**(정지 안 함 — 시간 종료는 정상 진행이라는 2026-08-25
 * 설계 유지). 이동이 리미트 인식 없이 시간/상한으로 끝나면 1회 안내:
 *   배수문 닫힘 → 05-04 · 열림 → 05-05 / 배출문(열림·닫힘) → 06-03 "배출부 모터 상태를 확인해 주세요".
 * ⚠ 배수문 닫힘은 4.2 s **시간**이 정상 종료 조건이다. 실기에서 리미트가 4.2 s 안에 안 잡히는 것이
 *   평소 모습이면 매 헹굼마다 05-04 가 나온다 — 그때는 0 으로 끄고 WDOOR_CLOSE_MS 부터 볼 것. */
/* ★2026-09-22 완료(DJ_DONE) 유지 시간 — moeum.h MOEUM_DONE_HOLD_MS 와 같은 이유(구현현황 §0.49·§0.50).
 * 종전에는 DJ_DONE 이 1 ms 만에 IDLE 로 넘어가 앱이 state 17 을 못 봤다 → 자가세척 cnt 40 이 "39/40 FAIL",
 * 종료 사유 "정지(중단)"(report_20260922_090832_동작). 이 시간 동안 DONE 에 머문 뒤 IDLE.
 * 운전 중이 아니므로(Dongjak_IsBusy = 0) 정지·마개 로직과 무관, 새 시작 요청은 DONE 에서도 받는다. */
#ifndef DJ_DONE_HOLD_MS
#define DJ_DONE_HOLD_MS          3000U    /* 앱 모니터 주기(1 s)의 3배 */
#endif
#ifndef DJ_WDOOR_MISS_VOICE
#define DJ_WDOOR_MISS_VOICE     1
#endif
#ifndef DJ_TDOOR_MISS_VOICE
#define DJ_TDOOR_MISS_VOICE     1
#endif

/* ---- 시작 홀 채널 / 온도 채널 / 수위 정책 (TBD, 모음과 동일 기조) ------ */
/* ★2026-08-25: 마개 라벨 순서 재정의로 동작이 HS5 -> HS2 로 옮겨졌다(모음과 맞교환).
 * 배선은 그대로이고 라벨만 바뀐 것이라 이 인덱스와 mode_arbiter.c 디코드 표만
 * 반대로 잡으면 된다. */
#ifndef DJ_HS_START_IDX
#define DJ_HS_START_IDX         1U        /* 동작 = HS2 (U24 P1, 0-based)     */
#endif
/* 시작 트리거(HS2 상승에지)를 이 파일 안에서 볼지 여부. 기본 0 = 중재자
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

/* ---- 안전 타임아웃 ------------------------------------------------------ */
/* [삭제 2026-08-25] 도어의 duty/구동시간/리미트 처리는 전부 wdoor.h 의 WDOOR_* 와
 * tdoor.h 의 TDOOR_* 프로파일이 단일 출처다(테스트벤치 tb_drv8871과 동일 값·동일
 * 규칙). 여기 있던 DJ_DOOR_TIMEOUT_MS / DJ_DOOR_LIMIT_OPTIONAL / DJ_DOOR_BENCH_MS /
 * DJ_TDOOR_LIMIT_OPTIONAL 은 참조처가 없는 채로 남아 "여기를 고치면 된다"고 오해를
 * 주기에 지웠다. 도어 동작을 바꾸려면 wdoor.h / tdoor.h 를 고칠 것.
 *   배수문 : 닫힘 Forward 80% / 4.2s 종료, 열림 Reverse 킥80%(2s)->65% / 상한 6s
 *   배출문 : 열림 Forward / 닫힘 Reverse, 80% 고정, 열림은 THALL 인식 후 200ms 뒤 정지 / 닫힘은 인식 후 2.8s 추가회전,
 *            방향별 상한 20s
 * ★리미트 미인식은 더 이상 DJ_ERROR 가 아니다 - 시간으로 정상 종료한다. 그래서
 *   DJ_ERR_RINSE_CLOSE/OPEN, DJ_ERR_HEAT_DOOR, DJ_ERR_DISCH_* 는 현재 발생 지점이
 *   없다(DjErrCode 값은 앱과 공유하므로 남겨 둔다). 미인식 자체는 앱 검증표의
 *   '도어 열림/닫힘 인식' 항목이 유일한 감지 수단이다. */
#ifndef DJ_FILL_TIMEOUT_MS
#define DJ_FILL_TIMEOUT_MS      120000UL
#endif

/* ---- 4.5 추가 투입 금지 / 비상 정지 (기획서 4.5) ----------------------- *
 * 처리 중 투입구 마개가 동작 위치를 벗어나면(=시작 홀 HS2 이탈) 즉시 전 액추에이터
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
	 * 2회는 고정 상수라 1차/2차를 통째로 언롤(변수 rinse_iter 폐지).
	 * ★개정4 ④: 교반은 회전수 운전 — STIR = WASH 10회, OPEN+DRAIN = DRAIN 10회(문 여는 중에도
	 * 돌고 센다). 1·2차 차이(배수 30/90초)는 사라졌다(C018). 2차 배수 완료 -> 곧바로 DJ_HEAT. */
	DJ_RINSE1_CLOSE,      /* 1차 배수문 닫힘 구동 -> W-HALL-CLOSE 대기          */
	DJ_RINSE1_FILL,       /* 1차 급수밸브 ON -> 수위 감지 대기                  */
	DJ_RINSE1_FILL_EXTRA, /* 1차 추가급수(DJ_FILL_EXTRA_MS=0 -> 즉시 통과)     */
	DJ_RINSE1_STIR,       /* 1차 WASH 회전수 운전 10회 (개정4 ④)                */
	DJ_RINSE1_OPEN,       /* 1차 DRAIN 운전 시작 + 배수문 열림 -> W-HALL-OPEN   */
	DJ_RINSE1_DRAIN,      /* 1차 DRAIN 계속 -> 10회 완료 -> 2차 시작            */
	DJ_RINSE2_CLOSE,      /* 2차 배수문 닫힘 구동 -> W-HALL-CLOSE 대기          */
	DJ_RINSE2_FILL,       /* 2차 급수밸브 ON -> 수위 감지 대기                  */
	DJ_RINSE2_FILL_EXTRA, /* 2차 추가급수(DJ_FILL_EXTRA_MS=0 -> 즉시 통과)     */
	DJ_RINSE2_STIR,       /* 2차 WASH 회전수 운전 10회 (개정4 ④)                */
	DJ_RINSE2_OPEN,       /* 2차 DRAIN 운전 시작 + 배수문 열림 -> W-HALL-OPEN   */
	DJ_RINSE2_DRAIN,      /* 2차 DRAIN 계속 -> 10회 완료 -> HEAT                */
	DJ_HEAT,           /* 건조: 히터+교반+분쇄+수증기+팬 동시(온도/시간 게이팅)*/
	DJ_COOLDOWN,       /* 120분~ 식힘: 히터OFF, 교반 CW3/1/CCW3, 분쇄 80℃서OFF*/
	DJ_BIN_CHECK,      /* 배출 전 수거통 유무/횟수/높이 확인                  */
	DJ_DISCHARGE,      /* 130분~ 배출문 2분 개방+교반 배출 후 닫고 배수부 개방*/
	DJ_DONE,
	DJ_ERROR,
	DJ_ABORTED,        /* 4.5 비상정지(투입구 개방/정지요청): 안전 식힘 후 IDLE */
	/* ★[2026-09-21 §0.27] 가이드 준비 탐색. 흐름상 IDLE 다음(1차 배수문 닫기 전)이지만 **값은
	 * 끝(20)에 붙였다** — 앱·watch 가 쓰는 기존 번호를 바꾸지 않기 위해서다. 재배열은 8단계. */
	DJ_PREP,           /* R001~R006 교반가이드 탐색 -> 성공 시 DJ_RINSE1_CLOSE */
	/* ★[2026-09-22 §0.30] 자동 자가세척(21). 배출 완료 -> 헹굼 1회 -> DJ_DONE. 같은 이유로 끝에 붙였다.
	 * 세부 단계는 g_dongjak.rinse.step(RinseStep 8 CLOSE / 9 FILL / 1~5 PREP / 10 WASH / 11 DRAIN_OPEN / 12 DRAIN). */
	DJ_SELFCLEAN
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
	DJ_ERR_DISCH_WOPEN = 7,  /* 배출 후 배수부 개방 타임아웃     */
	DJ_ERR_ROTATION    = 8,  /* ★개정4 ④: 회전수 운전 실패 = 벤더 E09
	                          *   (위치 무진행·정지 미확인 > P55, 역방향 이동). 앱 코드표는 8단계 */
	DJ_ERR_GUIDE       = 9,  /* ★§0.27: 준비 탐색 3회 미감지 = 벤더 E01(교반가이드 미확인). 앱 코드표는 8단계 */
	DJ_ERR_GRIND_MOTOR = 10, /* ★2026-09-22: 분쇄 M1 소프트락(nFAULT/잼 재시도 소진) = 벤더 E09-분쇄. 음성 06-04 */
	DJ_ERR_STIR_MOTOR  = 11, /* ★2026-09-22: 교반 M2 소프트락(nFAULT/잼 재시도 소진) = 벤더 E09-교반. 음성 06-01 */
	DJ_ERR_RELEASE     = 12, /* ★§0.30: 공통 해제 미확인(타이머 완료 또는 배출문 THALL 없음) = 벤더 E08. 음성 05-06 */
	DJ_ERR_OVERSPEED   = 13  /* ★§0.33 G3: 헹굼 과속(cnt 40 도달 시 t_cycle < 60 s, RINSE_FAULT_OVERSPEED) = 벤더 E02(X002).
	                          *   종전(rinse_lock 1차)엔 E09(8)로 합쳐 보냈다 — 벤더는 E02 를 마개 이탈로 풀리는 쪽에 둔다. 음성 05-02 */
} DjErrCode;

/* ★R3 예외처리 G3 [2026-09-22, 구현현황 §0.33] — 에러 **해제 규칙**을 벤더 0.3.0 에 맞춘다.
 * 벤더 controller.c: X001/X002(가이드·과속)·X005(모드 이탈)는 **실제 마개 이탈**(physical_mode NONE)로
 * 풀리고, X003/X004/X006/X008~X012 는 "기존 오류 대기" — 어떤 입력으로도 빠져나오지 않는다(A/S, I15).
 *   해제형(마개 이탈·정지 요청로 IDLE) : DJ_ERR_GUIDE(E01) · DJ_ERR_OVERSPEED(E02) — E05 는 코드 없음(jungji 경로)
 *   래치형(Dongjak_ClearError 로만)    : 그 외 전부 — E03(1·3~7) · E04(2) · E09(8·10·11, cnt 미도달 포함) · E08(12)
 * 래치형 에러가 남아 있으면 **모음·동작 모두 새로 시작하지 않는다**(Dongjak_Start/Moeum_Start 가 거른다).
 * 모음(g_moeum.err_code)도 같은 코드값·같은 분류를 쓴다.
 * 0 = 종전 동작(모든 에러가 마개 이탈·정지 요청으로 풀림) — 벤치에서 매번 err_clear_req 쓰기가 번거로우면. */
#ifndef DJ_ERR_LATCH_ENABLE
#define DJ_ERR_LATCH_ENABLE     1
#endif

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
	volatile uint8_t  dbg_enter_cool; /* [디버그] DJ_COOLDOWN(식힘 80℃미만) 점프 1회성 */
	volatile uint8_t  dbg_enter_selfclean; /* [디버그] DJ_SELFCLEAN(자가세척) 점프 1회성 — 앱 PROTO_ACT_DJ_SELFCLEAN */
	volatile uint8_t  dbg_beep;       /* [TEST] N 쓰면 즉시 N회 비프(오디오 경로 확인) 1회성 */
	volatile int16_t  temp_d10;       /* 처리통 온도 CH0=THERM1(히터/분쇄/식힘, 에러 시 직전값 유지) */
	volatile uint8_t  temp_valid;     /* 0 = CH0 써미스터 에러(디바운스됨) -> 히터 강제 OFF */
	volatile int16_t  temp2_d10;      /* 처리통 온도 CH1=THERM2(식힘 분쇄정지 보조, 에러 시 직전값 유지) */
	volatile uint8_t  temp2_valid;    /* 0 = CH1 써미스터 에러 -> 식힘 판정에서 '안 식음' 취급 */
	volatile int16_t  vapor_temp_d10; /* 수증기 제어 온도 CH2=THERM3/J23(SenseTick) */
	volatile uint8_t  water_reached;
	volatile uint8_t  bin_fill_pct;
	volatile uint8_t  bin_present;    /* HS7 수거통 있음(1). SenseTick 갱신. DJ_BIN_USE_HS7=0 이면 항상 1 */
	uint8_t           bin_warned;     /* 03-04 안내 래치: 이번 '없음' 구간에 이미 말함 */
	uint8_t           hs_prev;

	/* 4.5 비상정지 */
	volatile uint8_t  abort_req;      /* 정지 요청(SenseTick/RequestStop 세팅) */
	volatile uint8_t  dbg_force_stop; /* 벤치 강제 정지(1회성) = 정지버튼 모사  */
	uint8_t           lid_guard;      /* 1=실제 HS 시작 -> 투입구 개방 감시 무장 */
	uint8_t           lid_low_cnt;    /* HS LOW 연속 카운트(디바운스)          */

	/* 에러 */
	volatile uint8_t  err_code;       /* DjErrCode: DJ_ERROR 원인(RO)         */
	volatile uint8_t  err_clear_req;  /* 1 쓰면 DJ_ERROR->IDLE 복구(1회성). 고온이면 DJ_ABORTED(식힘) 경유 */
	volatile uint8_t  last_err;       /* ★G4: 마지막 에러 코드(RO). 해제·Abort 뒤에도 남고 새 시작에서 0 */
	uint8_t           err_from;       /* ★G2: 에러 직전 상태(DongjakState, RO) — 에러 중 배기팬 유지 판정 */

	uint16_t          cycle_count;    /* 처리 완료 횟수                       */

	/* 초기 헹굼: ★§0.30 부터 rinse.c(아래 rinse)가 진행하고 DongjakState(DJ_PREP · DJ_RINSE1_x ·
	 * DJ_RINSE2_x)는 앱·watch 용 **미러**다. 회차 = rinse.done. 자가세척은 DJ_SELFCLEAN 하나. */

	/* 건조 */
	uint8_t           heat_started;   /* 0 = 배수문 닫는 중                    */
	uint32_t          heat_since;     /* DJ_HEAT 배수문 닫힘 확인 tick(참고) — heater_since 와 다르다 */
	uint8_t           heater_started; /* R3 C005: 1 = 이번 운전에서 히터가 한 번이라도 켜짐 */
	uint32_t          heater_since;   /* R3 C005: 최초 HT_POWER ON 성공 tick — 110/120/130/135분의 기산점 */
	uint8_t           heat_off_latch; /* R3 C041: 120분 히터 종료 래치(재가열 금지). 시나리오 시작에서만 해제 */
	uint8_t           dry_gate;       /* R3 C028: 0 = 분쇄허가 대기(교반·분쇄 정지) / 1 = 동시 개시됨 */
	/* ★[TEST] DJ_TEST_SHORT 단축 시험 관측(§0.47). 양산 빌드에서도 필드는 남는다(앱 watch 호환). */
	uint8_t           t_low_seen;     /* 1 = 이번 가열에서 50℃ 접점(PF1 HIGH)을 봤다(래치)  */
	uint8_t           t_high_seen;    /* 1 = 이번 가열에서 70℃ 접점(PF0 LOW)을 봤다(래치)   */
	uint8_t           pair_started;   /* 1 = 두 모터 실제 동시 회전(FG rpm>0 둘 다) 시작함   */
	uint32_t          pair_since;     /* 동시 회전 시작 tick — +DJ_TEST_PAIR_MS 에 식힘으로  */
	volatile uint8_t  cool_exit_by;   /* [관측] 식힘 종료 사유 0 = 식힘 중·미진입 / 1 = 온도(<80℃) / 2 = 시간(§0.48 30 s) */
	uint8_t           grind_allow;    /* [관측 RO] R3 C031: (PF0==0)&&(PF3==0) 재계산값 */
	volatile uint8_t  heat_door_ok;   /* [관측 RO] ★C026 H1(§0.36): 히터 게이트 = 배수문 닫힘(PF3) — 0 이면 히터 명령 안 함 */
	uint8_t           test_beeped_hi; /* [TEST] 110분대(HISPEED) 비프 1회 래치 */

	/* 교반 패턴 FSM. 4구간(R3): 0=정 1=정지 2=역 3=정지 (dj_stir_alt_tick).
	 * 배출만 R2의 3구간(dj_stir313_tick: 0=CW 1=정지 2=CCW)을 계속 쓴다. */
	uint8_t           stir_phase;
	uint32_t          stir_since;
	/* ★개정4 ④: 헹굼 교반 = 회전수 운전. 헹굼 중 stir_phase/stir_since 는 **앱 보고용 미러** */
	RotStir_t         rot;
	volatile uint8_t  rot_st;         /* 직전 RotStir_Tick 반환 ROT_ST_* */
	RinseCtx          rinse;          /* ★§0.30 헹굼·자가세척 전 구간. rinse.step·fault·done (rinse.h) */
	uint8_t           stir_reps;      /* R3 C035: 정회전 묶음 카운터(group) - §0.9.2에서 지웠다가 복원 */
	uint8_t           stir_mode;      /* R3 C034: 0=패턴 교반 / 1=초기 동시회전(분쇄 토글 동기) */

	/* 분쇄 */
	uint8_t           grind_mode;     /* DjGrindMode                          */
	uint8_t           grind_phase;    /* DjGrindPhase(토글 모드)              */
	uint8_t           grind_final_ph; /* 110분 고속전환: 0=감속 대기, 1=역회전 확립(4/2 토글) */
	uint32_t          grind_since;
	uint32_t          grind_start;    /* 분쇄 개시(1차 3분 판정용)            */
	uint8_t           cool_phase;     /* 식힘: 0=뜨거움(교반 지속CW+분쇄1000CCW), 1=식음(교반313+분쇄OFF) */

	/* 수증기 */
	uint8_t           vapor_phase;    /* DjVaporPhase                         */
	uint32_t          vapor_step_since; /* 직전 스텝 tick (페이싱용)          */
	uint32_t          vapor_move_since; /* 현재 개/폐 동작 시작 tick (종료판정) */
	uint16_t          vapor_step_cnt;   /* 동작 내 스텝 순번 (시작램프 인덱스) */
	uint8_t           vapor_close_done; /* R3 C058: 84℃ 1차 폐쇄 완료 래치(운전당 1회) */

	/* 팬 */
	/* FAN_VAPOR는 vapor_phase로 관리(별도 상태 불필요). FAN_EXHAUST는 아래 fanx로 독립. */
	uint8_t           fanb_on;        /* BLDC 식힘팬 현재 ON?                 */
	uint8_t           fanb_active;    /* R3 C057: 구동조건 성립 중?(duty 기산 유지) */
	uint32_t          fanb_since;
	uint8_t           fanx_on;        /* FAN_EXHAUST(배기팬) 현재 ON?(10s/5s duty) */
	uint32_t          fanx_since;     /* 배기팬 현 구간 시작 tick             */

	/* 배출 */
	uint8_t           disc_phase;     /* DjDischPhase                         */
	uint32_t          disc_since;
	/* THALL 리미트 인식까지 실제로 걸린 시간(ms). 배출 진입 시 0으로 리셋되며,
	 * 0인 채로 다음 위상으로 넘어갔다면 = 리미트 미인식(135분 백스톱으로 전환).
	 * tdoor.h TDOOR_OPEN_MAX_MS/CLOSE_MAX_MS 재산정용 실측치 — 디버거에서 관찰. */
	uint32_t          disc_open_ms;   /* 개방(CW) 리미트 도달까지 ms (0=미도달) */
	uint32_t          disc_close_ms;  /* 닫힘(CCW) 리미트 도달까지 ms (0=미도달)*/
	uint8_t           disc_pulse_off; /* 간헐 구동 현재 휴지구간?(1=코스트 중)  */
	/* ★§0.30 공통 해제 추정(REV02 U26-1 두 입력). 배출 진입 시 0. 자가세척 진입 판정에만 쓴다. */
	uint8_t           disc_timer_done; /* 1 = 이번 배출에서 TIMER-OUT(PF8) 완료 엣지 수신(SW 백업 아님) */
	uint8_t           disc_close_hall; /* 1 = 배출문 닫힘을 THALL 로 실제 인식(20 s 상한 종료 아님)  */
	uint8_t           disc_by_backup;  /* ★§0.37 D2: 1 = TIMER-OUT 없이 SW 백업으로 닫았다 → 해제 불성립 */
	uint32_t          disc_fg_base;    /* ★§0.37 D1: 배출 시작 시 M2 FG 엣지 수                       */
	volatile uint32_t disc_aux_fg;     /* ★§0.37 D1: 배출 중 M2 FG 엣지(회로 AUX 교반이 실제로 돌았나, RO) */
	/* ★§0.37 D5 무게 관측 (SenseTick 100 ms, RO) */
	volatile uint16_t weight_raw;      /* PC3 원시값(12-bit)                                          */
	volatile uint8_t  weight_mon;      /* 1 = 감시 구간(배출 시작 ~ 타이머 완료). MotorTick 이 세우고 내린다 */
	uint8_t           weight_armed;    /* SenseTick: 기준값을 떴다                                     */
	volatile uint16_t weight_base;     /* 감시 구간 첫 샘플                                            */
	volatile uint16_t weight_delta;    /* |raw - base|                                                 */
	uint32_t          weight_run_since;/* 변화 연속 시작 tick (0 = 연속 아님)                          */
	volatile uint16_t weight_run_ms;   /* 현재 연속 길이 ms                                            */
	volatile uint8_t  f_weight;        /* 1 = 이번 배출 무게 정상 이력(연속 ≥ 2.108 s) 래치            */
} DongjakCtx;

extern DongjakCtx g_dongjak;

/* ---- API ----------------------------------------------------------------- */
void         Dongjak_Init(void);
void         Dongjak_Start(void);
void         Dongjak_Abort(void);
void         Dongjak_RequestStop(void);          /* 4.5 비상정지 요청(정지버튼)  */
void         Dongjak_ClearError(void);           /* DJ_ERROR -> IDLE 복구        */
uint8_t      Dongjak_ErrIsLatched(uint8_t code); /* ★G3: 1 = 래치형(ClearError 로만 해제) */
uint8_t      Dongjak_IsLatched(void);            /* ★G3: 1 = DJ_ERROR + 래치형 에러 중    */
void         Dongjak_SenseTick(void);            /* 100ms, StartDefaultTask   */
void         Dongjak_MotorTick(uint32_t now_ms); /* 1ms,  StartMotorTask      */
DongjakState Dongjak_GetState(void);
uint8_t      Dongjak_IsBusy(void);

/* [디버그/벤치 전용] 헹굼을 건너뛰고 DJ_HEAT부터 시작.
 *   skip_door=0 : 정상 진입(배수문 닫힘+WHALL 대기부터). 리미트를 직접 조작할 때.
 *   skip_door=1 : 도어 닫힘 대기(WHALL)까지 생략, 교반/분쇄/수증기/팬을 정상
 *                 초기화한 '가열중' 상태로 즉시 진입.
 * 전제: g_app_mode=DONGJAK 유지(동작 마개 HS2 시작위치). scn_start=now로 리셋되어
 * 시나리오 경과(105/110/120분 분기)가 0부터 카운트된다.
 * ★런타임에서는 함수 직접호출 대신 변수 트리거를 권장(링커 제거·GDB call 불필요):
 *     g_dongjak.dbg_enter_heat = 1(도어대기부터) 또는 2(도어생략)
 *   -> MotorTick가 1회 소비하여 이 함수를 호출한다. */
void         Dongjak_DebugEnterHeat(uint8_t skip_door);
/* 헹굼·건조를 건너뛰고 식힘 교반(80℃ 미만)부터. 앱 '동작 (식힘부터)' 버튼.
 * 경과를 120분 지점으로 맞춰 넣으므로 10분 뒤 배출로 자연히 이어진다.
 *   g_dongjak.dbg_enter_cool = 1  (MotorTick 이 1회 소비) */
void         Dongjak_DebugEnterCool(void);
/* 처리·배출을 건너뛰고 **자가세척(DJ_SELFCLEAN)** 부터. 앱 '동작 (자가세척부터)' 버튼 [2026-09-22].
 * IDLE·DONE·해제형 에러에서만 받는다(처리 중·래치형 에러면 무시 — 시작 규칙과 같다).
 * 공통 해제 추정(DJ_DS_CLOSE_T)은 건너뛴다 — 실제 배출이 없었다. cycle_count 는 늘리지 않는다.
 *   g_dongjak.dbg_enter_selfclean = 1  (MotorTick 이 1회 소비) */
void         Dongjak_DebugEnterSelfclean(void);

#ifdef __cplusplus
}
#endif

#endif /* SCENARIO_DONGJAK_H_ */
