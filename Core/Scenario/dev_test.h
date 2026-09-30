#ifndef SCENARIO_DEV_TEST_H_
#define SCENARIO_DEV_TEST_H_

/* ==========================================================================
 * dev_test - [개발 검증 전용 — 임시] 동작 시나리오 **단축 시험** 스위치 (단일 출처)
 * --------------------------------------------------------------------------
 * ★2026-09-22 사용자 지시 "업체 방식으로 하고 cooling_safe 는 우리 기준, 그 외는 우리 기준 유지" (§0.47)
 * 원본: doc/R3/zerozio_legacy_short_test.c (업체 U 1.0) — 흐름만 따르고 코드는 이식하지 않았다.
 * (§0.45·§0.46 의 "공정 시계 1/1/1분 압축 + PROCESS 3/2/3회"는 이 방식으로 **대체·폐지**했다.)
 *
 * DJ_TEST_SHORT = 1 이면 동작 시나리오의 건조~식힘만 바뀐다:
 *   가열   : 히터만. 교반·분쇄 정지. **50℃ 접점(PF1 HIGH)과 70℃ 접점(PF0 LOW)을 둘 다 실제로 본 뒤**
 *            (각각 한 번 보면 래치), 우리 분쇄 허가(dj_dry_permitted = PF0 && PF3 && 배출문 닫힘)가 성립하면
 *            PROCESS 개시.                                             ← 업체 HEAT
 *   동시 운전: PROCESS 초기 정회전 패턴(2회전 + 정지 2 s, 교반 30 rpm · 분쇄 1200 rpm, 양산 P49 30회 그대로).
 *            **두 모터가 실제로 함께 돈 순간**(FG 측정 rpm > 0 둘 다)부터 DJ_TEST_PAIR_MS 뒤 식힘으로.
 *            30 s 는 초기 30회 안에서 끝나므로 역회전 반복은 오지 않는다.
 *            후반 분쇄(2000 rpm 반대방향)는 **요청하지 않는다**.        ← 업체 PAIR
 *   식힘   : 히터·분쇄 OFF, 교반은 기존 식힘 교반 유지. 끝 = **우리 기준**(THERM1·THERM2 둘 다 < 80℃,
 *            dj_cool_reached → cool_phase 1) **또는 식힘 진입 후 DJ_TEST_COOL_MAX_MS(30 s)** 중 먼저 오는 쪽
 *            (★§0.48 사용자 지시). 130분 시각 조건은 보지 않는다. 어느 쪽으로 끝났는지 = g_dongjak.cool_exit_by.
 *            ⚠ 30 s 로 끝나면 **80℃ 이상인 채로 배출**한다 — 업체 원본("안전 냉각 확인 전 배출 금지")과 다르다. 벤치 전용.
 *   그 외(준비 탐색·헹굼 1회·배출 HW 120 s·즉시 닫기·해제 추정·자가세척·에러·음성)는 양산과 같다.
 *   110/120/130/135분 상수(dongjak.h)는 양산값 그대로다 — 50/70℃ 가 끝내 안 오면 120분 백스톱이 빼낸다.
 *
 * ⚠ pythonapp `zerogeo/build_config.py` 의 FW_TEST_SHORT / TEST_PAIR_S / TEST_COOL_MAX_S 와 **같이** 바꿀 것(앱은 빌드를 판별할 수 없다).
 * ⚠ 이 상태로 양산 빌드 금지 — 켜져 있으면 빌드 로그에 #warning 이 뜬다. 끝나면 0 으로.
 * ========================================================================== */
#ifndef DJ_TEST_SHORT
#define DJ_TEST_SHORT           1        /* ★임시 1 = 업체식 단축 시험 / 0 = 양산 */
#endif

#if DJ_TEST_SHORT
#ifndef DJ_TEST_PAIR_MS
#define DJ_TEST_PAIR_MS         30000UL  /* [TEST] 두 모터 실제 동시 회전부터 30 s (업체 PAIR) */
#endif
#ifndef DJ_TEST_COOL_MAX_MS
#define DJ_TEST_COOL_MAX_MS     30000UL  /* [TEST] 식힘 최대 30 s — 온도(<80℃) 또는 이 시간 (§0.48) */
#endif
#warning "DJ_TEST_SHORT=1: 동작 단축 시험(가열 50/70℃ 확인 -> 30초 동시 운전 -> 80℃ 미만 또는 30초 식힘). 양산 빌드 전 0 으로 되돌릴 것"
#endif /* DJ_TEST_SHORT */

#endif /* SCENARIO_DEV_TEST_H_ */
