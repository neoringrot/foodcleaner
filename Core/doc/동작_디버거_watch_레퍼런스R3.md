# 동작(dongjak) 시나리오 — 디버거 Watch 레퍼런스

> STM32CubeIDE **Live Expressions**(실행 중 갱신) / **Expressions** 뷰에 등록해 동작 시나리오의
> **단계·시간·온도·서브동작·안전**을 런타임 추적하기 위한 참조표.
> 대상 코드: [dongjak.c](../Scenario/dongjak.c) / [dongjak.h](../Scenario/dongjak.h). 온도는 0.1℃ 단위(d10, 예 800 = 80.0℃).
> `foodcleaner.launch`에 `enable_live_expr=true` 확인됨(Live Expressions 사용 가능).
>
> **★2026-08-15 갱신**: 시작 트리거와 정지가 dongjak 밖으로 나갔다. 모드 선택은 중재자
> [mode_arbiter.c](../Scenario/mode_arbiter.c)(`g_modearb`), 정지는 [jungji.c](../Scenario/jungji.c)(`g_jungji`)가 담당한다.
> **`g_app_mode`를 디버거로 직접 쓰던 방식은 더 이상 그대로 동작하지 않는다** — 중재자가 되돌릴 수
> 있으므로 수동 고정이 필요하면 `g_modearb.dbg_disable = 1`을 먼저 세팅할 것(§6·§8).
>
> **★2026-08-18 갱신**: `DJ_TEST_FAST_TIMING=0`(양산 타임라인 135분) + **단계 전환 비프 OFF**(테스트 기능).
> 멤브레인 버튼 제어 폐지로 물리 정지버튼 경로는 없다 — 정지는 **HS3 / 마개이탈 / 앱 `PROTO_ACT_STOP` /
> `g_jungji.dbg_stop_req`**. 관찰 중 만나는 임시 완화 플래그의 배경은 [HW미검증_항목R3.md](HW미검증_항목R3.md) 참조.
>
> **★2026-09-21 갱신(R3)**: 시작 직후 **가이드 준비 탐색 `DJ_PREP`(값 20)** 이 추가됐고, 헹굼 교반은 **회전수 운전**
> (WASH/DRAIN 각 10회, `g_dongjak.rot`)으로 바뀌어 시간 종료 상수(117초·30/90초)가 빠졌다. 110/120/130/135분은
> **최초 히터 ON(`heater_since`) 기준**이다. 건조 교반 20rpm 패턴 A·식힘 20rpm 4구간·배기팬 10초/5초.

---

## 0. STM32CubeIDE 등록 방법

Expressions 목록은 **워크스페이스 UI 상태**라 프로젝트 파일로 주입 불가 → 아래처럼 등록:

1. 디버그 세션 시작 후 **Window ▸ Show View ▸ Live Expressions**(실행 중에도 값 갱신) 열기.
   (일시정지 상태에서만 보면 되는 값은 **Expressions** 뷰도 가능)
2. "**Add new expression**" 행에 §7 목록의 식을 한 줄씩 붙여넣고 Enter. (버전에 따라 여러 줄 붙여넣기 시 자동 다중 추가되기도 함)
3. 열거형(`state`, `grind_mode` 등)은 값에 커서를 두면 이름(예 `DJ_HEAT`)으로 표시됨. 숫자로만 보이면 §1·§4의 매핑 참조.
4. 경과시간은 §5의 `uwTick - …` 식을 그대로 추가.

> 팁: `g_dongjak` 구조체 자체를 하나 추가하면 하위 필드를 트리로 모두 펼쳐 볼 수 있다.

---

## 1. 메인 상태머신 — `g_dongjak.state`

★**§0.35(2026-09-22): R3 C004 로 동작 헹굼은 1회 고정(`DJ_RINSE_COUNT 1`)** — 기본 빌드에서는 1~6 만 나오고 1차 배수 후 곧바로 `DJ_HEAT`. 반복 N 입력은 TODO(N18). 아래는 `DJ_RINSE_COUNT=2U` 빌드 기준 설명: 헹굼 2회를 **고정 상수로 언롤** — moeum `MoeumState`처럼 1차(1~6)·2차(7~12) 세부단계를 모두 최상위 `state`로 노출(카운터 변수 `rinse_iter` 폐지). 1·2차 로직 **완전 동일**(★개정4 ④: 배수 30/90초 차이 소멸, C018) — 본교반 = WASH 10회, 열림+배수 = DRAIN 10회 회전수 운전. 2차 배수 후 곧바로 `DJ_HEAT`(별도 잔수배수 상태 없음). 시작 직후(1차 배수문 닫기 전) **`DJ_PREP`(20)** 에서 가이드 준비 탐색을 1회 한다 — 흐름상 IDLE 다음이지만 값은 enum 끝. 타이밍은 `el = uwTick - state_since`.

| 값 | 상태 | 하는 일 | 다음 단계 전이 조건 |
|---|---|---|---|
| 0 | `DJ_IDLE` | 대기 | `start_req`(**중재자가 HS2 확정 시 호출** / `dbg_force_start`) → `scn_start` 후 **DJ_PREP**(20) |
| 1 | `DJ_RINSE1_CLOSE` | 1차 배수문 닫힘 구동(80 % 고정) | `WDoor_ReachedClose()` **또는** `el ≥ WDOOR_CLOSE_MS`(4.2초) → RINSE1_FILL. **ERROR 없음** |
| 2 | `DJ_RINSE1_FILL` | 1차 급수밸브 ON, 수위 대기 | `water_reached` → RINSE1_FILL_EXTRA / `el ≥ DJ_FILL_TIMEOUT_MS` → ERROR |
| 3 | `DJ_RINSE1_FILL_EXTRA` | 1차 추가급수(R3 C017로 **0초** — 통과 상태) | 진입 즉시(`DJ_FILL_EXTRA_MS`=0) → 밸브OFF·RINSE1_STIR |
| 4 | `DJ_RINSE1_STIR` | 1차 **WASH 회전수 운전**(30RPM, 2회전+정지2s × 10회: CW×5 → CW/CCW/CW/CCW/CW) | `rot_st == 2`(ROT_ST_DONE) → RINSE1_OPEN / `rot_st == 3`(실패) → ERROR(8) |
| 5 | `DJ_RINSE1_OPEN` | 1차 **DRAIN 운전 전환**(`RotStir_Switch`, CW만) + 배수문 열림 구동(80 %×2초→65 %) — 문 여는 중에도 교반 | `WDoor_ReachedOpen()` **또는** `el ≥ WDOOR_OPEN_MAX_MS`(6초) → RINSE1_DRAIN. 도어 ERROR 없음(회전수 실패만 ERROR(8)) |
| 6 | `DJ_RINSE1_DRAIN` | 1차 배수 교반(DRAIN 운전 계속) | `rot_st == 2`(DRAIN 10회 완료) → RINSE2_CLOSE |
| 7 | `DJ_RINSE2_CLOSE` | 2차 배수문 닫힘 구동 | 1과 동일(`WDoor_ReachedClose()` 또는 4.2초) → RINSE2_FILL. **ERROR 없음** |
| 8 | `DJ_RINSE2_FILL` | 2차 급수밸브 ON, 수위 대기 | `water_reached` → RINSE2_FILL_EXTRA / 타임아웃 → ERROR |
| 9 | `DJ_RINSE2_FILL_EXTRA` | 2차 추가급수(**0초** — 통과 상태) | 진입 즉시 → 밸브OFF·RINSE2_STIR |
| 10 | `DJ_RINSE2_STIR` | 2차 WASH 회전수 운전(4와 동일, 새 운전) | `rot_st == 2` → RINSE2_OPEN / 실패 → ERROR(8) |
| 11 | `DJ_RINSE2_OPEN` | 2차 DRAIN 전환 + 배수문 열림 구동 | 5와 동일(`WDoor_ReachedOpen()` 또는 6초) → RINSE2_DRAIN |
| 12 | `DJ_RINSE2_DRAIN` | 2차 배수 교반(DRAIN 운전 계속) | `rot_st == 2`(DRAIN 10회 완료) → HEAT |
| 13 | `DJ_HEAT` | 건조(히터+교반+분쇄+수증기+팬). 교반·분쇄는 **`grind_allow`=1(70℃+배수문닫힘) 성립 시 동시 개시**(C028), 교반 20rpm (정5+정지2)×5→역3→정지2, 초기 120초는 분쇄 토글 동기. 110분↑ 교반 전환은 **폐기**(C036) | `sel ≥ DJ_T_COOLDOWN_MS`(120분, heater_since 기준) → COOLDOWN |
| 14 | `DJ_COOLDOWN` | 식힘(히터OFF). `cool_phase` 0=뜨거움(교반 지속CW 20rpm+분쇄1000CCW) / 1=식음(80℃미만: 분쇄OFF+교반 20rpm 정3/정지1/역3/정지1) | `sel ≥ DJ_T_DISCHARGE_MS`(130분) → BIN_CHECK |
| 15 | `DJ_BIN_CHECK` | 수거통 확인 | `cycle_count < 6` **&&** `bin_fill_pct < 90` → DISCHARGE (아니면 대기) |
| 16 | `DJ_DISCHARGE` | 배출문 개방(CW, THALL 인식 후 200ms 뒤 정지 / 닫힘은 +2.8s 추가회전, **상한 `TDOOR_OPEN_MAX_MS` 20초**)+교반313 2분(TIMER-OUT↓)→교반정지·개방유지→**135분에 닫기(CCW, 상한 20초)**→배수부 개방 | `disc_phase` 배출 FSM 완료 → `cycle_count++` → DONE |
| 17 | `DJ_DONE` | 완료 | 즉시 → IDLE |
| 18 | `DJ_ERROR` | 에러 정지(`err_code`=원인). 히터·교반·분쇄·급수·문 OFF, ★§0.33: **모터 냉각팬(80℃↑ 30/10, 온도 무효면 연속)·배기팬(에러 직전이 배기 구간이면 10/5) 유지** | ★§0.33 G3 — **해제형**(9 GUIDE·13 OVERSPEED): 마개 이탈·`dbg_force_stop=1`·`g_jungji.dbg_stop_req=1`·`err_clear_req=1`. **래치형**(그 외 전부): **`err_clear_req=1`(앱 CLEAR_ERR)만** — 동작 모드 tick 필요(마개 HS2). 해제 시 고온이면 19 ABORTED 경유. 래치 중에는 모음·동작 모두 시작 안 함 |
| 19 | `DJ_ABORTED` | 4.5 비상정지(안전 식힘) | `temp_d10 < 800`(80℃) 냉각 후 → IDLE |
| 20 | `DJ_PREP` | ★가이드 준비 탐색 R001~R006(`rinse.c`): 정5s→정지1s→역5s→정지1s→정5s, 30rpm, HS6 감지 즉시 정지 | `rinse.step == 6`(RINSE_PREP_OK) → RINSE1_CLOSE / `rinse.step == 7`(RINSE_PREP_FAULT, 3회 미감지) → ERROR(9) |
| 21 | `DJ_SELFCLEAN` | ★자동 자가세척(§0.30): 배출문 닫힘 직후 진입. 히터·분쇄·팬 OFF, `rinse.c` 헹굼 1회(배수문 닫힘 → 급수 → 준비 탐색 → WASH → 배수문 열림+DRAIN). 세부는 `rinse.step`(8 CLOSE/9 FILL/1~5 PREP/10 WASH/11 DRAIN_OPEN/12 DRAIN) | 진입: 공통 해제 추정(`disc_timer_done`·`disc_close_hall`) 불성립이면 21 대신 ERROR(12, E08). 급수는 온도 대기 없음. `rinse.step == 13`(FINISHED) → DONE / 급수·가이드·회전 실패 → ERROR(2/9/8) |

> ⚠️ **`DJ_ABORTED`는 이제 잘 보이지 않는다.** 마개 이탈/HS3 정지는 중재자 → `jungji`가 처리하며
> `Dongjak_Abort()`로 **곧바로 `DJ_IDLE`** 로 내린다(잔열 냉각은 `g_jungji.cooling`이 이어받음 — §8).
> `DJ_ABORTED`에 들어가는 경로는 `Dongjak_RequestStop()` / `g_dongjak.dbg_force_stop = 1` 뿐이다.

---

## 2. 시간 마커 매크로 — `sel`(시나리오 절대경과) 기준

> `sel` = `uwTick - g_dongjak.heater_since` (ms, `heater_started`=1 이후. 히터가 한 번도 안 켜졌으면 0). 분 = ÷60000.
> ★R3 C005(2026-09-21 §0.24): 기산점이 `scn_start` → **최초 HT_POWER ON(`heater_since`)** 으로 바뀌었다. `scn_start` 는 앱 경과시간 표시용으로만 남는다.
>
> ✅ **현재 `DJ_TEST_FAST_TIMING=0`(2026-08-18)**: 아래 표의 **양산 타임라인이 그대로 적용**되며 단계 비프도 없다(전 구간 1회 관찰 = 약 135분 + 헹굼 ~6분).
>
> ★2026-09-22 재정의(§0.47 — §0.45·§0.46 대체): 스위치 **`Scenario/dev_test.h` `DJ_TEST_SHORT`**(현재 **1 — 업체식 단축 시험**, 원본 `doc/R3/zerozio_legacy_short_test.c`). 가열(모터 정지) → **50℃(PF1)·70℃(PF0) 접점 둘 다 실제 확인** + 분쇄 허가 → 두 모터 실제 동시 회전(FG>0)부터 **30 s**(`DJ_TEST_PAIR_MS`, 초기 정회전만·후반 분쇄 생략) → 식힘(분쇄 OFF, **THERM1·2 < 80℃**(우리 기준) **또는 30 s**(`DJ_TEST_COOL_MAX_MS`, §0.48) 중 먼저, 130분 무시, 사유 = `g_dongjak.cool_exit_by` 1 온도/2 시간) → 배출 이후 양산과 같음. 110/120/130/135분은 압축하지 않음(120분은 백스톱). 켜져 있으면 `#warning`. 앱 **`zerogeo/build_config.py` `FW_TEST_SHORT`/`TEST_PAIR_S`/`TEST_COOL_MAX_S`** 와 같이 바꿀 것. ⚠ 30 s 로 끝나면 80℃ 이상인 채 배출(벤치 전용). **양산 전 0**.

| 매크로 | 값 | 게이팅 전이 |
|---|---|---|
| `DJ_GRIND_COARSE_MS` | **120초**(R3 C034) | 1차 분쇄(1500 CW 5/2초) 지속시간 — 기준점은 `grind_start`(분쇄 허가 최초 성립) |
| `DJ_T_HISPEED_MS` | **110분** | 분쇄 → 2000 CCW 4/2초 (교반 RPM 전환은 R3 C036 으로 **폐기**) |
| `DJ_T_COOLDOWN_MS` | **120분** | HEAT → COOLDOWN |
| `DJ_T_DISCHARGE_MS` | **130분** | COOLDOWN → BIN_CHECK |
| `DJ_T_LATCH_MS` | **135분** | **배출문 닫기 개시**(교반은 2분에 정지, 문은 이때까지 개방 유지). 배출 진입이 늦었으면 `DJ_DISCH_WINDOW_MS`(5분) 확보 후 |

---

## 3. 온도 임계 매크로 — `g_dongjak.temp_d10` 기준

| 매크로 | d10 / ℃ | 게이팅 |
|---|---|---|
| `DJ_TEMP_GRIND_ON_D10` | **650 / 65.0** | ★**R3 C031 로 게이트 역할 폐기(2026-09-20)** — 참조값일 뿐 구동을 막지 않는다. 실제 허가는 `g_dongjak.grind_allow`(=PF0·PF3 AND 관측) |
| `g_dongjak.grind_allow` | 0/1 | ★**분쇄 허가 관측**. 승온 중 0, 70℃+배수문닫힘에서 1 |
| `g_dongjak.heat_door_ok` | ★§0.36 C026 H1 — 히터 게이트(RO). 1 = 배수문 닫힘(PF3, U35 입력과 같은 센서) → 히터 명령 가능. 0 이면 PA12 OFF·공정 시계 기산 안 함 |
| `g_dongjak.disc_timer_done` / `.disc_by_backup` / `.disc_close_hall` | ★§0.30·§0.37 공통 해제 추정 — **닫기 전** TIMER-OUT 수신 / SW 백업(140 s)·135분 백스톱으로 닫음(→ 해제 불성립) / 닫힘 THALL 실제 인식. 해제 = 1·0·1 |
| `g_dongjak.disc_aux_fg` | ★§0.37 D1 — 배출 중 M2 FG 엣지 수(RO). MCU 는 명령하지 않는다 — 회로(U30 AUX) 교반이 실제로 돌았는지 |
| `g_dongjak.rinse.lock.net` / `.net_max` | ★§0.38 C094·C060 — 헹굼 중 마지막 가이드 이후 순이동(FG, 부호) / 이번 회차 최대. 1028(1.25 회전) 이상이면 E01(`rinse.fault` 6 GUIDE_LOST) |
| `g_dongjak.rinse.lock.rpm_max` / `.iv_min_ms` / `.iv_short` | ★§0.41 C061 — 과속 보조: 감시 구간 날개 rpm 최대(A, 40 초과 300 ms → E02) / 최소 가이드 간격(B, < 1500 ms 2회 연속 → E02) / 짧은 간격 연속 수. `lock.fault` 3 시간 과속 · 5 rpm 과속 · 6 가이드 간격 |
| `g_dongjak.weight_raw` / `.weight_mon` / `.weight_delta` / `.weight_run_ms` / `.f_weight` | ★§0.37 D5 — PC3 무게 원시값 / 감시 구간 / \|raw−기준\| / 연속 길이 / 연속 ≥ 2.108 s 래치. 임계 `DJ_WEIGHT_DELTA_COUNTS` 는 TBD(N20). 해제 판정 미사용 |
| `g_dongjak.dry_gate` | 0/1 | ★**C028 개시 게이트**. 0 = 교반·분쇄 대기, 1 = 동시 개시됨 |
| `DJ_TEMP_COOL_GRIND_OFF_D10` | **800 / 80.0** | 식힘 중 분쇄 OFF 하한 (**THERM1 AND THERM2 둘 다 <80℃**. 바이메탈 OR 는 `DJ_COOL_USE_BIMETAL80=0` 으로 비활성) |
| `DJ_TEMP_BLDCFAN_ON_D10` | 800 / 80.0 | BLDC 식힘팬 ON / ABORTED 냉각 하한 |
| `DJ_TEMP_VAPOR_ON_D10` | 1000 / 100.0 | 수증기 ALL ON (**소스=`vapor_temp_d10`, THERM3/J23**) |
| `DJ_TEMP_VAPOR_OFF_D10` | 840 / 84.0 | 수증기 ALL OFF (**소스=`vapor_temp_d10`, THERM3/J23**) |
| `DJ_TEMP_HEATER_ON_D10` | **1140 / 114.0** | 히터 복귀 ON |
| `DJ_TEMP_HEATER_OFF_D10` | **1170 / 117.0** | 히터 OFF |
| `DJ_TEMP_SAFETY_D10` | 2100 / 210.0 | 과열 SW 차단(주체=HW) |

---

## 4. 런타임 상태 변수 — `g_dongjak.*`

### 4.1 핵심 관찰 / 센서 입력
| 식 | 타입 | 의미 |
|---|---|---|
| `g_dongjak.state` | u8 | 메인 상태(§1) |
| `g_dongjak.state_since` | u32 | 현 상태 진입 tick |
| `g_dongjak.scn_start` | u32 | 시나리오 시작 tick(절대 0점) |
| `g_dongjak.temp_d10` | i16 | 처리통 온도 CH0/THERM1(히터·분쇄·식힘 주기준, 0.1℃; 에러 시 직전값 유지) |
| `g_dongjak.temp_valid` | u8 | 0 = CH0 써미스터 에러(5회 디바운스) → **히터 강제 OFF** |
| `g_dongjak.temp2_d10` | i16 | 처리통 온도 CH1/THERM2(**식힘 분쇄정지 보조**, 0.1℃; 에러 시 직전값 유지) |
| `g_dongjak.temp2_valid` | u8 | 0 = CH1 써미스터 에러 → 식힘 판정서 '안 식음' 취급(보수적) |
| `g_dongjak.vapor_temp_d10` | i16 | 수증기 제어 온도 CH2/THERM3/J23(0.1℃) |
| `g_dongjak.water_reached` | u8 | 수위 도달(1) |
| `g_dongjak.bin_fill_pct` | u8 | 수거통 분말높이 % |
| `g_dongjak.cycle_count` | u16 | 처리 완료 횟수 |

### 4.2 서브-FSM 위상
| 식 | 값/의미 |
|---|---|
| `g_dongjak.heat_started` | HEAT: 0=배수문 닫힘 대기, 1=가열중 |
| `g_dongjak.stir_phase` | 교반 위상. 건조·식힘 4구간 0 정/1 정지/2 역/3 정지 · 배출 313 0 CW/1 STOP/2 CCW · 초기 120초 0 구동/1 정지 · **헹굼 중엔 `rot.out` 미러**(0 FWD/1 STOPPED/2 REV) |
| `g_dongjak.rot_st` | ★헹굼 회전수 운전 `RotStir_Tick` 반환: 0 IDLE / 1 RUNNING / 2 DONE(10회 완료) / 3 FAILED(→`err_code` 8) |
| `g_dongjak.rot.out` / `rot.eng.cycle` / `rot.edges` | 엔진 지령 0 정지/1 CW/2 CCW · 현 프로파일 완료 운전 수(10 = 완료) · Start 이후 HS6 엣지(관측만, HW 마일스톤 아님) |
| `g_dongjak.rinse.step` | ★`RinseStep` 0 IDLE/1 F1/2 S1/3 R/4 S2/5 F2/6 OK(통과)/**7 FAULT(E01)** · ★§0.30 8 CLOSE/9 FILL/10 WASH/11 DRAIN_OPEN/12 DRAIN/13 FINISHED/14 FAULT(`rinse.fault` 2 급수/3 회전) · `rinse.done` 완료 회차 |
| `g_dongjak.rinse.prep_try` / `prep_ms` | 성공한 try 1~3(0=미성공) · 성공 try 시작→감지 엣지 ms |
| `g_dongjak.grind_mode` | `DjGrindMode` 0 OFF/1 COARSE/2 FINE/3 FINAL/4 COOL |
| `g_dongjak.grind_phase` | 토글 0 RUN/1 STOP (COARSE·110분↑ FINAL 공용) |
| `g_dongjak.grind_final_ph` | 110분 고속전환 0=감속대기(급반전 방지) / 1=역회전 확립→**4초구동/2초정지 토글** |
| `g_dongjak.cool_phase` | 식힘 0=뜨거움(교반 지속CW 20rpm+분쇄1000CCW) / 1=식음(분쇄OFF+교반 20rpm 정3/정지1/역3/정지1) |
| `g_dongjak.grind_start` | 분쇄 허가(`grind_allow`) 최초 성립 tick(초기 120초 판정). 허가 상실 시 0 으로 리셋 |
| `g_dongjak.heater_started` / `heater_since` | ★R3 C005 공정 시계: 1 = 이번 운전에서 HT_POWER 가 켜짐 / 최초 ON tick(110/120/130/135분 기산점) |
| `g_dongjak.vapor_phase` | `DjVaporPhase` 0 CLOSED/1 OPEN_DUCT(STEP1)/2 OPEN_AIR(STEP2)/3 OPEN/4 CLOSE_AIR/5 CLOSE_DUCT |
| `g_dongjak.vapor_step_cnt` | 현재 개/폐 동작 내 스텝 순번 — **시작 램프 인덱스 전용**(종료 판정에는 안 쓰임) |
| `g_dongjak.vapor_move_since` | 현재 개/폐 동작 시작 tick. **종료는 `now-vapor_move_since >= 구동시간`**(STEP1 15s / STEP2 열림1s·닫힘2s) |
| `g_dongjak.fanb_on` | BLDC 식힘팬(BLDC_FAN) 현재 ON?(**분쇄 동작 중 OR THERM1 ≥80℃** 동안 30/10, R3 C057 — 조건은 `fanb_active`). FAN_VAPOR(방수팬)은 `vapor_phase`로 확인(0=OFF, 1~5=ON) |
| `g_dongjak.fanx_on` | **FAN_EXHAUST(배기팬) 현재 ON?** — 시나리오 전체 **10초ON/5초OFF**(R3 C056) 독립 duty(THERM3 무관). `fanx_since`=현 구간 시작 tick |
| `g_dongjak.disc_phase` | `DjDischPhase` 0 OPEN_T(개방CW)/1 EXPEL(교반2분)/**2 HOLD(교반정지·개방유지, 135분 대기)**/3 CLOSE_T(닫힘CCW)/4 OPEN_W ← ★2026-08-17 HOLD 추가로 CLOSE_T·OPEN_W 값이 2/3→3/4로 밀림 |
| `g_dongjak.disc_open_ms` | 배출문 **개방** 종료까지 실측 ms(THALL 인식 후 200ms 뒤 정지, 닫힘 `disc_close_ms` 는 +2.8s 추가회전). **≈20000 = 상한(`TDOOR_OPEN_MAX_MS`) 종료 = 미인식 의심**, 0 = 135분 백스톱으로 닫기 전환. 타임아웃 값 재산정용 |
| `g_dongjak.disc_close_ms` | 배출문 **닫힘** 종료까지 실측 ms(0 = 아직 닫는 중). **≈20000 = 상한(`TDOOR_CLOSE_MAX_MS`) 종료 = 미인식 의심** |
| `g_dongjak.disc_pulse_off` | ~~배출문 간헐 구동 휴지구간~~ — **2026-08-25부터 항상 0**. 펄스 개시(15초)가 배출문 상한(20초)보다 늦어 도달하지 않는다 |

### 4.3 4.5 비상정지 / 시작·정지 제어
| 식 | 의미 |
|---|---|
| `g_dongjak.start_req` | 시작 래치(**중재자**/`dbg_force_start` → MotorTick 소비) |
| `g_dongjak.dbg_force_start` | **[쓰기]** 1 = HS 없이 강제 시작(1회성). **중재자 도입 후에도 그대로 유효** |
| `g_dongjak.dbg_force_stop` | **[쓰기]** 1 = 정지버튼 모사 → `DJ_ABORTED`(시나리오 자체 안전 식힘 경로) |
| `g_dongjak.abort_req` | 정지 요청 래치 |
| `g_dongjak.err_code` | `DjErrCode` DJ_ERROR 원인: ~~1 RINSE_CLOSE~~ / 2 RINSE_FILL(E04) / ~~3 RINSE_OPEN~~ / ~~4 HEAT_DOOR~~ / ~~5 DISCH_OPEN~~ / ~~6 DISCH_CLOSE~~ / ~~7 DISCH_WOPEN~~ / **8 ROTATION**(회전수 운전 실패·cnt 미도달 = E09) / **9 GUIDE**(준비 탐색 3회 미감지 = E01) / **10 GRIND_MOTOR**·**11 STIR_MOTOR**(소프트락 = E09) / **12 RELEASE**(공통 해제 미확인 = E08) / ★**13 OVERSPEED**(헹굼 과속 = E02, §0.33). 해제형 = 9·13, 나머지 래치형. 도어 코드(1·3~7)는 발생 지점 없음. 해제되면 0 — 원인은 `last_err` |
| `g_dongjak.err_clear_req` | **[쓰기]** 1 = DJ_ERROR 해제(1회성, `Dongjak_ClearError()`·앱 CLEAR_ERR 와 동일). **래치형 에러를 푸는 유일한 길.** 에러 진입 때 묵은 값은 버린다 |
| `g_dongjak.last_err` | ★§0.33 G4 — 마지막 에러 코드(RO). 해제·Abort 뒤에도 남고 **새 시작에서만 0** |
| `g_dongjak.err_from` | ★§0.33 G2 — 에러 직전 `DongjakState`(RO). 배기 구간(1~16·20)이면 에러 중 배기팬 10/5 유지 |
| `g_moeum.err_code` / `.last_err` / `.err_clear_req` | ★§0.33 — 모음 에러(값은 `DjErrCode` 표 공유: 9 E01 / 2 E04 / 8 E09 / 13 E02). `err_clear_req = 1` 은 모음 모드(HS5)에서 소비 |
| ~~`g_dongjak.lid_guard`~~ | **미사용**(`DJ_HS_TRIGGER_INTERNAL=0`). 마개 이탈 감시는 중재자의 `HS_LOST`가 담당 |
| ~~`g_dongjak.lid_low_cnt`~~ | **미사용**(상동). 디바운스는 `g_modearb.cand_cnt` |
| ~~`g_dongjak.hs_prev`~~ | **미사용**(상동). 위치 판정은 `g_modearb.pos_stable` |

> 위 3개는 `DJ_HS_TRIGGER_INTERNAL=1`(구 동작 복원)일 때만 갱신된다. 기본값 0에서는 0으로 고정이니
> 이 값들을 보고 마개 상태를 판단하지 말 것 — §8의 `g_modearb.*`를 볼 것.

---

## 5. 경과시간 Watch 식 (uwTick = HAL 틱 카운터 전역)

```text
(uwTick - g_dongjak.scn_start) / 60000      # 시나리오 경과(분, 앱 표시용)
(uwTick - g_dongjak.heater_since) / 60000   # ★공정 시계(분) = §2 sel. heater_started=1 일 때만 유효
(uwTick - g_dongjak.scn_start)              # 시나리오 경과(ms)
(uwTick - g_dongjak.state_since) / 1000     # 현 상태 경과(초)
```

> ⚠️ **Live Expressions** 뷰는 버전에 따라 산술식(`-`,`/`)을 못 받을 수 있다. 그 경우
> `uwTick`·`g_dongjak.scn_start`를 각각 등록해 손으로 빼거나, **Expressions** 뷰(일시정지 시)를 쓰면 산술식이 항상 동작한다.

---

## 6. 벤치 트리거 / 오버라이드

| 조작(쓰기) | 효과 |
|---|---|
| `g_modearb.dbg_disable = 1` | **중재자 정지** → `g_app_mode`를 디버거로 수동 고정 가능(구 방식 복원). **아래 강제시작 전에 먼저 할 것** |
| `g_app_mode = 2` | 수동으로 동작 모드 진입(`dbg_disable=1` 상태에서만 유지됨) |
| `g_dongjak.dbg_force_start = 1` | HS2 없이 시작(벤치, IDLE→**DJ_PREP**→RINSE1_CLOSE). 1회성. ※DJ_PREP 은 HS6(가이드) 감지가 필요 — 3회(5+1+5+1+5 = 약 17초) 안에 못 찾으면 `err_code`=9 |
| `g_dongjak.dbg_enter_cool = 1` | **헹굼·건조 생략, 식힘 교반(80℃ 미만)부터.** 경과를 120분 지점으로 맞춰 넣어 10분 뒤 배출로 이어진다. 앱 `동작 (식힘부터)` 버튼 / `PROTO_ACT_DJ_COOL`(0x09) / `send_action.py dj-cool`. 1회성 |
| `g_dongjak.dbg_enter_heat = 1` | **헹굼 건너뛰고 DJ_HEAT부터 시작**(1=도어 닫힘·WHALL 대기부터, 리미트 직접 조작 시). MotorTick가 1회 소비. `scn_start`=now 리셋 + `heater_started`=0 → 공정 시계는 첫 히터 ON 에서 0부터. **먼저 동작 모드 진입**(위 dbg_disable+app_mode) 필요 |
| `g_dongjak.dbg_enter_heat = 2` | 위와 동일하되 **도어 대기까지 생략**(바로 가열중). ★R3 C028: 교반·분쇄는 여기서도 켜지 않고 `grind_allow` 허가를 기다린다 — 70℃ 접점이 없는 벤치는 `DJ_DRY_GATE_BYPASS=1` |
| `g_dongjak.dbg_beep = N` | **[TEST] 즉시 N회 비프**(스피커/오디오 경로 확인용). ※`DJ_TEST_FAST_TIMING=1`일 때만 컴파일됨 — 현재 0이라 무효 |
| `g_dongjak.err_clear_req = 1` | **DJ_ERROR 해제**(원인은 `err_code`로 먼저 확인). 해제형·래치형 모두. ※동작 모드에서 tick이 돌아야 소비됨 — 래치형은 이것만 먹는다 |
| `g_jungji.dbg_stop_req = 1` | 전역 정지 → `Dongjak_Abort()`. ★§0.33 부터 **해제형(9·13)만 IDLE 로 풀린다** — 래치형은 DJ_ERROR·`err_code` 유지(`err_clear_req` 필요). `DJ_ERR_LATCH_ENABLE=0` 빌드면 종전처럼 모두 풀림 |
| `g_dongjak.dbg_force_stop = 1` | 처리 중 즉시 비상정지(`DJ_ABORTED`). DJ_ERROR 에서는 **해제형만** 해제(래치형은 무시, §0.33) |
| `g_jungji.dbg_stop_req = 1` | **전역 비상정지**(중재자·모드 무관, BLDC 단락제동 포함). 1회성 |
| `g_modearb.dbg_pos_force = 5` | 홀 없이 마개 위치 주입(1 강음/2 모음/3 정지/4 배수/5 동작). 0 = 미사용 |
| ~~매크로 `DJ_DOOR_LIMIT_OPTIONAL`/`DJ_TDOOR_LIMIT_OPTIONAL`~~ | **2026-08-25 폐지** — 육안모드 분기가 코드에서 제거됨. 도어 시간은 `wdoor.h`의 `WDOOR_CLOSE_MS`/`WDOOR_OPEN_MAX_MS`, `tdoor.h`의 `TDOOR_*_MAX_MS`로 조정 |
| 매크로 `DJ_HS_TRIGGER_INTERNAL=1` | 구 동작 복원 — dongjak이 직접 HS2 에지/lid 감시(중재자와 **이중 처리되니** 중재자를 끄고 쓸 것) |
| 매크로 `DJ_LID_OPEN_ABORT=0` | 4.5 투입구 개방 감시 비활성(`DJ_HS_TRIGGER_INTERNAL=1`일 때만 의미 있음) |
| 매크로 `DJ_BIN_CHECK_ENABLE=0` | 수거통 확인 생략(즉시 배출) |
| 매크로 `JUNGJI_BLDC_BRAKE_USE=0` | BLDC 비상정지를 단락제동 없이 코스트로(제동 거동을 빼고 관찰할 때) |

### 벤치에서 "홀 없이 동작 시나리오만 돌려보기" 절차
```text
1) g_modearb.dbg_disable = 1     # 중재자가 모드를 되돌리지 않게
2) g_app_mode = 2                # APP_MODE_DONGJAK
3) g_dongjak.dbg_force_start = 1 # 시작
```
> 2번을 건너뛰면 `Dongjak_MotorTick()` 자체가 돌지 않아 `start_req`만 쌓이고 아무 일도 안 난다.
> 1번을 건너뛰면 마개가 없는(=`LID_POS_NONE`) 상태가 확정되는 순간 중재자가 정지를 걸 수 있다.

---

## 7. 복사용 Live Expressions 목록

```text
g_dongjak.state
g_dongjak.temp_d10
g_dongjak.temp_valid
g_dongjak.temp2_d10
g_dongjak.temp2_valid
g_dongjak.vapor_temp_d10
g_dongjak.grind_mode
g_dongjak.grind_final_ph
g_dongjak.cool_phase
g_dongjak.stir_phase
g_dongjak.vapor_phase
g_dongjak.vapor_step_cnt
g_dongjak.disc_phase
g_dongjak.heat_started
g_dongjak.grind_allow
g_dongjak.dry_gate
g_dongjak.rot_st
g_dongjak.rinse.step
g_dongjak.heater_since
g_dongjak.err_code
g_dongjak.last_err
g_dongjak.cycle_count
g_dongjak.water_reached
g_dongjak.bin_fill_pct
g_dongjak.fanb_on
g_dongjak.fanx_on
g_dongjak.abort_req
g_dongjak.dbg_force_start
g_dongjak.dbg_force_stop
g_dongjak.scn_start
g_dongjak.state_since
g_app_mode
g_modearb.pos_stable
g_modearb.pos_raw
g_modearb.raw_mask
g_modearb.transitions
g_modearb.mode_changes
g_jungji.last_src
g_jungji.last_kind
g_jungji.stop_count
g_jungji.braking
g_jungji.cooling
uwTick
(uwTick - g_dongjak.scn_start) / 60000
(uwTick - g_dongjak.state_since) / 1000
```

---

## 8. 중재자 · 정지 Watch (★2026-08-15 신규)

> 마개를 돌렸는데 시작이 안 되거나, 갑자기 멈췄을 때 **가장 먼저 볼 곳**.
> 대상 코드: [mode_arbiter.c](../Scenario/mode_arbiter.c) / [jungji.c](../Scenario/jungji.c).

### 8.1 모드 — `g_app_mode`
| 값 | 모드 | 트리거(마개 위치) |
|---|---|---|
| 0 | `APP_MODE_TESTBENCH` | 대기/벤치(부팅 기본). 앱 정지 명령(`ACT_STOP`)도 여기로 |
| 1 | `APP_MODE_MOEUM` | HS5 모음 |
| 2 | `APP_MODE_DONGJAK` | HS2 동작 |
| 3 | `APP_MODE_KANGEUM` | HS1 강음 (스텁 — 구동 없음) |
| 4 | `APP_MODE_BAESU` | HS4 배수 (스텁 — 구동 없음) |
| 5 | `APP_MODE_JUNGJI` | **HS3 정지** (★2026-08-25 신설). 마개가 정지 위치에 있는 동안 유지. 시나리오 tick 없음 + 벤치 폴링 유지 = 대기와 실행 성격 동일, **표시만 구분**. 나가는 길: 마개를 다른 위치로 / 앱 `ACT_STOP`(→0) |

### 8.2 마개 위치 — `g_modearb.*`
| 식 | 의미 |
|---|---|
| `g_modearb.raw_mask` | 이번 샘플의 HS1~5 비트마스크(bit0=HS1 … bit4=HS5). **2비트 이상이면 홀 이상** |
| `g_modearb.pos_raw` | 디바운스 전 디코딩(`LidPos`) |
| `g_modearb.pos_stable` | **확정 위치**(`LidPos`). 0 NONE / 1 강음 / 2 모음 / 3 정지 / 4 배수 / 5 동작 / 6 MULTI |
| `g_modearb.cand` / `cand_cnt` | 디바운스 후보와 연속 카운트(`MODEARB_CONFIRM_SAMPLES`=10(≈1s)에 도달하면 확정. NONE/MULTI 는 `MODEARB_LOST_CONFIRM_SAMPLES`=3) |
| `g_modearb.primed` | 0 = 부팅 후 첫 확정 전(이때 확정되는 위치는 **기준선만 잡고 기동하지 않는다**) |
| `g_modearb.pend_valid` / `pend_mode` | 적용 대기 중인 모드 전환 |
| `g_modearb.start_wait` | 1 = 전환 완료, **BLDC 제동 해제를 기다리는 중**(곧 `*_Start()`) |
| `g_modearb.transitions` | 확정 위치가 바뀐 누적 횟수 |
| `g_modearb.mode_changes` | `g_app_mode`가 실제로 바뀐 누적 횟수 |
| `g_modearb.dbg_disable` | **[쓰기]** 1 = 중재자 정지(수동 모드) |
| `g_modearb.dbg_pos_force` | **[쓰기]** 위치 강제 주입(0=미사용) |

### 8.3 정지 — `g_jungji.*`
| 식 | 의미 |
|---|---|
| `g_jungji.last_src` | 마지막 정지 사유(`JungjiSrc`): 0 NONE / 1 **HS3 정지** / 2 **마개 이탈** / 3 홀 다중인식 / 4 모드전환 / 5 시나리오 / 6 디버거 / 7 앱(`PROTO_ACT_STOP`) |
| `g_jungji.last_kind` | 0 NORMAL(코스트) / 1 **EMERGENCY(단락제동)** |
| `g_jungji.last_tick` | 마지막 정지 실행 tick |
| `g_jungji.stop_count` | 누적 정지 실행 횟수 |
| `g_jungji.braking` | 1 = BLDC 단락제동 유지 중(`JUNGJI_BRAKE_MS`=500ms). **이 구간엔 키패드/시작 지령이 막힌다** |
| `g_jungji.cooling` | 1 = 잔열 냉각으로 배기팬+BLDC팬 유지 중 |
| `g_jungji.temp_d10` | 냉각 판정에 쓴 온도(`JUNGJI_COOL_TEMP_D10`=800 = 80.0℃ 기준) |
| `g_jungji.req` / `req_src` / `req_kind` | 아직 소비되지 않은 정지 요청(1ms `Jungji_Tick`이 즉시 소비하므로 보통 0) |
| `g_jungji.dbg_stop_req` | **[쓰기]** 1 = 전역 비상정지(1회성) |

### 8.4 증상별 진단
| 증상 | 확인 순서 |
|---|---|
| 마개를 돌렸는데 시작 안 됨 | `raw_mask`(홀이 잡히나) → `pos_stable`(확정됐나) → `primed`(부팅 후 첫 확정이면 기동 안 함이 정상) → `g_app_mode` → 해당 시나리오 `state` |
| 시작하자마자 멈춤 | `g_jungji.last_src` — 2(마개 이탈)면 자석/거리, 3(다중 인식)이면 홀 배선 |
| 천천히 돌렸더니 엉뚱한 모드/정지 | 통과 위치(HS3/HS4)가 10샘플(≈1s) 확정된 것. `transitions` 증가 확인 → `MODEARB_CONFIRM_SAMPLES` 상향 검토 |
| 정지했는데 분쇄날이 계속 돔 | `g_jungji.last_kind`가 0(NORMAL=코스트)인지 확인. 1인데도 오래 돌면 `JUNGJI_BRAKE_MS` 상향 |
| 디버거로 `g_app_mode`를 썼는데 되돌아감 | 중재자가 소유자. `g_modearb.dbg_disable = 1` 선행 필요 |

---

## 9. (참고) 기타 튜닝 매크로 기본값

| 분류 | 매크로 = 값 |
|---|---|
| RPM | `DJ_STIR_RPM=30`(**배출 전용**) · `DJ_DRY_STIR_RPM=20`(건조, R3 C034·C035) · `DJ_COOL_STIR_RPM=20`(식힘 80℃↓, C043) · ~~`DJ_STIR_RPM_HISPEED=40`~~(R3 C036 폐기) · `DJ_COOL_HOT_STIR_RPM=20`(식힘 80℃↑ 지속CW 전용) · `DJ_RINSE_STIR_RPM=30` · `DJ_STIR_RPM_HILOAD=30`(대기, 이제 증속 아님) · `DJ_GRIND_COARSE_RPM=1500` · `DJ_GRIND_FINE_RPM=1000` · `DJ_GRIND_FINAL_RPM=2000` |
| 헹굼 교반 | ★개정4 ④: **회전수 운전** WASH/DRAIN 각 10회(`rotation_port`: P54 `ROT_M2_TICKS_PER_REV=822`, 정지 2s, P55 `ROT_FEEDBACK_TIMEOUT_MS=5000`) · `DJ_RINSE_STIR_RPM=30`. `DJ_RINSE_CW_MS=3000`·`STOP_MS=1000`·`CCW_MS=3000` 은 앱 보고용으로만 남음. `DJ_RINSE_STIR_MS` 는 `ZG_TRACKED_ONLY`(기본 0 → 정의 없음) |
| 건조 교반 | ★R3 C035 **패턴 A 복원**: `DJ_DRY_FWD_MS=5000` · `DJ_DRY_STOP_MS=2000` · `DJ_DRY_REV_MS=3000` · `DJ_DRY_FWD_GROUP=5` (1cycle 40s, `dj_stir_dry_tick`, 묶음 카운터 `stir_reps`) |
| 배출 교반 | `DJ_S313_CW_MS=6000` · `DJ_S313_STOP_MS=1000` · `DJ_S313_CCW_MS=6000` (3구간 1사이클 13s, `dj_stir313_tick`) |
| 식힘 80℃↓ 교반 | `DJ_COOL_S313_CW_MS=3000` · `DJ_COOL_S313_STOP_MS=1000` · `DJ_COOL_S313_CCW_MS=3000` (4구간 정3/정지1/역3/정지1 = 8s, R3 C043) |
| 분쇄 | 1차 토글 `DJ_GRIND_RUN_MS=5000`(R3 C034)/`STOP_MS=2000` · 110분↑ 고속 **`DJ_GRIND_RUN_FINAL_MS=4000`/`STOP_FINAL_MS=2000`(4초구동/2초정지 토글)**, 최초 진입만 `DJ_GRIND_FINAL_DECEL_MS=1500`(급반전 방지 감속)+슬루 상승 후 토글 |
| 팬 | **FAN_VAPOR**(방수팬)=THERM3 100/84℃ vapor시퀀스(duty 없음) · **FAN_EXHAUST**(배기팬)=시나리오 전체 **`DJ_FANX_ON_MS=10초`/`OFF=5초` 독립 duty**(THERM3 분리 2026-08-17, R3 C056 으로 15분/2분→10초/5초) · **BLDC식힘팬** `DJ_FANB_ON_MS=30000`/`OFF=10000`(분쇄 동작 중 OR ≥80℃, C057) |
| 헹굼/배출 | ~~`DJ_RINSE_STIR_MS=120000` · `DJ_RINSE1_DRAIN_MS=30000` · `DJ_RINSE2_DRAIN_MS=90000`~~(★개정4 ④ `ZG_TRACKED_ONLY` 추적용 — 종료는 WASH/DRAIN 10회) · `DJ_FILL_EXTRA_MS=0`(R3 C017) · 배출 교반 종료=**TIMER-OUT(PF8)↓** (백업 `DJ_DISCHARGE_STIR_MS=120000`) / **배출문 닫기 개시=`DJ_T_LATCH_MS=135분`** |
| 스테퍼 | **구동시간 기준(2026-08-25 벤치확정, 스텝수 기준 폐지)**: `DJ_STEP1_RUN_MS=15000`(관로 개/폐 각 15s, 방향무관) · `DJ_STEP2_OPEN_MS=1000`(흡입 열림) · `DJ_STEP2_CLOSE_MS=2000`(흡입 닫힘) · STEP1 `DJ_STEP_INTERVAL_MS=3`(333PPS)/`START=10`(100PPS) · STEP2 `DJ_STEP2_INTERVAL_MS=3`(333PPS, **2026-08-25 6→3 복귀**)/**`DJ_STEP2_START_MS=20`(50PPS 시작램프 유지 — 1s 창의 488ms를 먹으므로 이동량 부족 시 1순위 조정 대상)** · 램프 `DJ_STEP_RAMP_STEPS=40`(선형가속) · **`DJ_STEP_RELEASE_DUCT_ON_OPEN=1`**(STEP1 개방후 코일해제→STEP2에 레일전류 양보) · 개폐 후 정지 |
| 식힘 80℃ | 분쇄 OFF·교반(20rpm 4구간) 전환 조건 = **(THERM1 && THERM2 둘 다 유효 && <`DJ_TEMP_COOL_GRIND_OFF_D10`=800) ‖ 바이메탈 하강엣지(`DJ_COOL_USE_BIMETAL80=1` 일 때만)**. `DJ_COOL_USE_THERM2=1`(THERM2 AND 추가, CH1=PC1), `DJ_COOL_USE_BIMETAL80=`**`0`**(★REV02: PF0 가 70℃ 접점이 되어 근거 상실, 구현현황 §0.17.3 ②). 보수적: 써미스터 에러 시 '안 식음'. COOLDOWN 진입 시 stale 엣지 클리어 |
| 수거통 | `DJ_MAX_CYCLES=6` · `DJ_BIN_FULL_PCT=90` · `DJ_BIN_CHECK_ENABLE=1` |
| 채널/정책 | `DJ_HS_START_IDX=1`(HS2) · `DJ_TEMP_CH=0`(THERM1) · **`DJ_VAPOR_TEMP_CH=2`(THERM3/J23,PC2)** · `DJ_WATER_ACTIVE_LOW=1` · `DJ_FILL_USE_WATER_ON=1` · `DJ_DRAIN_VALVE_USE=1` |
| 안전 | `DJ_FILL_TIMEOUT_MS=120000` · `DJ_LID_CONFIRM_SAMPLES=3`. **도어(2026-08-25 이관)**: `WDOOR_CLOSE_DUTY=80`/`WDOOR_CLOSE_MS=4200` · `WDOOR_OPEN_KICK_DUTY=80`/`WDOOR_OPEN_KICK_MS=2000`/`WDOOR_OPEN_RUN_DUTY=65`/`WDOOR_OPEN_MAX_MS=6000`(wdoor.h) · `TDOOR_DUTY=80`/`TDOOR_OPEN_OVERRUN_MS=200`/`TDOOR_CLOSE_OVERRUN_MS=2800`(2026-09-22)/`TDOOR_OPEN_MAX_MS`=`TDOOR_CLOSE_MAX_MS`=`20000`(tdoor.h). ~~`DJ_DOOR_TIMEOUT_MS`·`DJ_DOOR_BENCH_MS`·`DJ_DOOR_DUTY_PCT`~~ 폐지 |
