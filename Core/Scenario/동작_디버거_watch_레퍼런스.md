# 동작(dongjak) 시나리오 — 디버거 Watch 레퍼런스

> STM32CubeIDE **Live Expressions**(실행 중 갱신) / **Expressions** 뷰에 등록해 동작 시나리오의
> **단계·시간·온도·서브동작·안전**을 런타임 추적하기 위한 참조표.
> 대상 코드: [dongjak.c](dongjak.c) / [dongjak.h](dongjak.h). 온도는 0.1℃ 단위(d10, 예 800 = 80.0℃).
> `foodcleaner.launch`에 `enable_live_expr=true` 확인됨(Live Expressions 사용 가능).
>
> **★2026-08-15 갱신**: 시작 트리거와 정지가 dongjak 밖으로 나갔다. 모드 선택은 중재자
> [mode_arbiter.c](mode_arbiter.c)(`g_modearb`), 정지는 [jungji.c](jungji.c)(`g_jungji`)가 담당한다.
> **`g_app_mode`를 디버거로 직접 쓰던 방식은 더 이상 그대로 동작하지 않는다** — 중재자가 되돌릴 수
> 있으므로 수동 고정이 필요하면 `g_modearb.dbg_disable = 1`을 먼저 세팅할 것(§6·§8).
>
> **★2026-08-18 갱신**: `DJ_TEST_FAST_TIMING=0`(양산 타임라인 135분) + **단계 전환 비프 OFF**(테스트 기능).
> 멤브레인 버튼 제어 폐지로 물리 정지버튼 경로는 없다 — 정지는 **HS3 / 마개이탈 / 앱 `PROTO_ACT_STOP` /
> `g_jungji.dbg_stop_req`**. 관찰 중 만나는 임시 완화 플래그의 배경은 [HW미검증_항목.md](HW미검증_항목.md) 참조.

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

헹굼 2회를 **고정 상수로 언롤** — moeum `MoeumState`처럼 1차(1~6)·2차(7~12) 세부단계를 모두 최상위 `state`로 노출(카운터 변수 `rinse_iter` 폐지). 1·2차 로직 동일, **차이는 배수 교반시간뿐**: 1차 30초 / 2차 90초(잔수 흡수). 2차 배수 후 곧바로 `DJ_HEAT`(별도 잔수배수 상태 없음). 타이밍은 `el = uwTick - state_since`.

| 값 | 상태 | 하는 일 | 다음 단계 전이 조건 |
|---|---|---|---|
| 0 | `DJ_IDLE` | 대기 | `start_req`(**중재자가 HS2 확정 시 호출** / `dbg_force_start`) → `scn_start` 후 RINSE1_CLOSE |
| 1 | `DJ_RINSE1_CLOSE` | 1차 배수문 닫힘 구동(80 % 고정) | `WDoor_ReachedClose()` **또는** `el ≥ WDOOR_CLOSE_MS`(4.2초) → RINSE1_FILL. **ERROR 없음** |
| 2 | `DJ_RINSE1_FILL` | 1차 급수밸브 ON, 수위 대기 | `water_reached` → RINSE1_FILL_EXTRA / `el ≥ DJ_FILL_TIMEOUT_MS` → ERROR |
| 3 | `DJ_RINSE1_FILL_EXTRA` | 1차 수위 후 2초 추가급수 | `el ≥ DJ_FILL_EXTRA_MS`(2초) → 밸브OFF·RINSE1_STIR |
| 4 | `DJ_RINSE1_STIR` | 1차 교반 CW6/정지1/CCW6 헹굼(30RPM) | `el ≥ DJ_RINSE_STIR_MS`(**117초 = 13s×9**) → RINSE1_OPEN |
| 5 | `DJ_RINSE1_OPEN` | 1차 배수문 열림 구동(80 %×2초→65 %) | `WDoor_ReachedOpen()` **또는** `el ≥ WDOOR_OPEN_MAX_MS`(6초) → RINSE1_DRAIN. **ERROR 없음** |
| 6 | `DJ_RINSE1_DRAIN` | 1차 배수 교반(물빼기) | `el ≥ DJ_RINSE1_DRAIN_MS`(**30초**) → RINSE2_CLOSE |
| 7 | `DJ_RINSE2_CLOSE` | 2차 배수문 닫힘 구동 | `WDoor_AtClose()` → RINSE2_FILL / 타임아웃 → ERROR |
| 8 | `DJ_RINSE2_FILL` | 2차 급수밸브 ON, 수위 대기 | `water_reached` → RINSE2_FILL_EXTRA / 타임아웃 → ERROR |
| 9 | `DJ_RINSE2_FILL_EXTRA` | 2차 수위 후 2초 추가급수 | `el ≥ DJ_FILL_EXTRA_MS`(2초) → 밸브OFF·RINSE2_STIR |
| 10 | `DJ_RINSE2_STIR` | 2차 교반 CW6/정지1/CCW6 헹굼(30RPM) | `el ≥ DJ_RINSE_STIR_MS`(**117초 = 13s×9**) → RINSE2_OPEN |
| 11 | `DJ_RINSE2_OPEN` | 2차 배수문 열림 구동 | `WDoor_AtOpen()` → RINSE2_DRAIN / 타임아웃 → ERROR |
| 12 | `DJ_RINSE2_DRAIN` | 2차 배수 교반(물빼기30초+잔수60초) | `el ≥ DJ_RINSE2_DRAIN_MS`(**90초**) → HEAT |
| 13 | `DJ_HEAT` | 건조(히터+교반 30rpm CW6/정지1/CCW6+분쇄+수증기+팬. 110분↑ 교반만 40rpm) | `sel ≥ DJ_T_COOLDOWN_MS`(120분) → COOLDOWN |
| 14 | `DJ_COOLDOWN` | 식힘(히터OFF). `cool_phase` 0=뜨거움(교반 지속CW+분쇄1000CCW) / 1=식음(80℃미만: 분쇄OFF+교반313) | `sel ≥ DJ_T_DISCHARGE_MS`(130분) → BIN_CHECK |
| 15 | `DJ_BIN_CHECK` | 수거통 확인 | `cycle_count < 6` **&&** `bin_fill_pct < 90` → DISCHARGE (아니면 대기) |
| 16 | `DJ_DISCHARGE` | 배출문 개방(CW, **타임아웃 없이 리미트까지**)+교반313 2분(TIMER-OUT↓)→교반정지·개방유지→**135분에 닫기(CCW, 타임아웃 없음)**→배수부 개방 | `disc_phase` 배출 FSM 완료 → `cycle_count++` → DONE |
| 17 | `DJ_DONE` | 완료 | 즉시 → IDLE |
| 18 | `DJ_ERROR` | 도어/급수 타임아웃 정지(`err_code`=원인) | **복구(3경로): ① `g_jungji.dbg_stop_req=1`(전역 정지, 모드/중재자 무관 — 벤치 확실) ② 정지요청 `Dongjak_RequestStop()`/`dbg_force_stop=1`/`err_clear_req=1`(동작 모드 tick 필요) ③ 정상모드에서 마개 '정지'(HS3)로 돌림 → 모드전환 → Abort** |
| 19 | `DJ_ABORTED` | 4.5 비상정지(안전 식힘) | `temp_d10 < 800`(80℃) 냉각 후 → IDLE |

> ⚠️ **`DJ_ABORTED`는 이제 잘 보이지 않는다.** 마개 이탈/HS3 정지는 중재자 → `jungji`가 처리하며
> `Dongjak_Abort()`로 **곧바로 `DJ_IDLE`** 로 내린다(잔열 냉각은 `g_jungji.cooling`이 이어받음 — §8).
> `DJ_ABORTED`에 들어가는 경로는 `Dongjak_RequestStop()` / `g_dongjak.dbg_force_stop = 1` 뿐이다.

---

## 2. 시간 마커 매크로 — `sel`(시나리오 절대경과) 기준

> `sel` = `uwTick - g_dongjak.scn_start` (ms). 분 = ÷60000.
>
> ✅ **현재 `DJ_TEST_FAST_TIMING=0`(2026-08-18)**: 아래 표의 **양산 타임라인이 그대로 적용**되며 단계 비프도 없다(전 구간 1회 관찰 = 약 135분 + 헹굼 ~6분).
>
> 벤치에서 다시 압축이 필요하면 `DJ_TEST_FAST_TIMING=1`: **110→7 / 120→8 / 130→9 / 135→12분, 1차 분쇄 3분→30초, 배기팬 15분/2분→15초/2초**로 축소되고(★135는 10분→**12분**: 배출 창 3분 > HW 배출문 2분 타이머라야 닫힘 검증 가능) 각 전환점에서 스피커 비프(7분대 삑 / 8분대 삑삑 / 9분대 삑삑삑)가 울린다.

| 매크로 | 값 | 게이팅 전이 |
|---|---|---|
| `DJ_GRIND_COARSE_MS` | 3분 | 1차 분쇄(1500 CW 3/2초) 지속시간 |
| `DJ_T_HISPEED_MS` | **110분** | 분쇄 → 2000 CCW 4/2초 **+ 교반 RPM 20→27** |
| `DJ_T_COOLDOWN_MS` | **120분** | HEAT → COOLDOWN |
| `DJ_T_DISCHARGE_MS` | **130분** | COOLDOWN → BIN_CHECK |
| `DJ_T_LATCH_MS` | **135분** | **배출문 닫기 개시**(교반은 2분에 정지, 문은 이때까지 개방 유지). 배출 진입이 늦었으면 `DJ_DISCH_WINDOW_MS`(5분) 확보 후 |

---

## 3. 온도 임계 매크로 — `g_dongjak.temp_d10` 기준

| 매크로 | d10 / ℃ | 게이팅 |
|---|---|---|
| `DJ_TEMP_GRIND_ON_D10` | **650 / 65.0** | 건조 분쇄 허용 하한(센서65=실제80) |
| `DJ_TEMP_COOL_GRIND_OFF_D10` | **800 / 80.0** | 식힘 중 분쇄 OFF 하한 (**THERM1 AND THERM2 둘 다 <80℃** ‖ 바이메탈) |
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
| `g_dongjak.stir_phase` | 교반 위상(건조 0 FWD/1 STOP/2 REV · 313 0 CW/1 STOP/2 CCW) |
| `g_dongjak.grind_mode` | `DjGrindMode` 0 OFF/1 COARSE/2 FINE/3 FINAL/4 COOL |
| `g_dongjak.grind_phase` | 토글 0 RUN/1 STOP (COARSE·110분↑ FINAL 공용) |
| `g_dongjak.grind_final_ph` | 110분 고속전환 0=감속대기(급반전 방지) / 1=역회전 확립→**4초구동/2초정지 토글** |
| `g_dongjak.cool_phase` | 식힘 0=뜨거움(교반 지속CW+분쇄1000CCW) / 1=식음(분쇄OFF+교반313) |
| `g_dongjak.grind_start` | 분쇄 허용온도(센서65℃) 최초 도달 tick(1차 3분 판정) |
| `g_dongjak.vapor_phase` | `DjVaporPhase` 0 CLOSED/1 OPEN_DUCT(STEP1)/2 OPEN_AIR(STEP2)/3 OPEN/4 CLOSE_AIR/5 CLOSE_DUCT |
| `g_dongjak.vapor_step_cnt` | 현재 개/폐 동작 내 스텝 순번 — **시작 램프 인덱스 전용**(종료 판정에는 안 쓰임) |
| `g_dongjak.vapor_move_since` | 현재 개/폐 동작 시작 tick. **종료는 `now-vapor_move_since >= 구동시간`**(STEP1 15s / STEP2 열림1s·닫힘2s) |
| `g_dongjak.fanb_on` | BLDC 식힘팬(BLDC_FAN) 현재 ON?(분쇄 동작 중 30/10). FAN_VAPOR(방수팬)은 `vapor_phase`로 확인(0=OFF, 1~5=ON) |
| `g_dongjak.fanx_on` | **FAN_EXHAUST(배기팬) 현재 ON?** — 시나리오 전체 15분ON/2분OFF 독립 duty(THERM3 무관). `fanx_since`=현 구간 시작 tick |
| `g_dongjak.disc_phase` | `DjDischPhase` 0 OPEN_T(개방CW)/1 EXPEL(교반2분)/**2 HOLD(교반정지·개방유지, 135분 대기)**/3 CLOSE_T(닫힘CCW)/4 OPEN_W ← ★2026-08-17 HOLD 추가로 CLOSE_T·OPEN_W 값이 2/3→3/4로 밀림 |
| `g_dongjak.disc_open_ms` | 배출문 **개방** THALL_OPEN 인식까지 실측 ms (**0 = 미인식**, 135분 백스톱으로 닫기 전환). 타임아웃 값 재산정용 |
| `g_dongjak.disc_close_ms` | 배출문 **닫힘** THALL_CLOSE 인식까지 실측 ms (**0 = 아직 미인식** — NO_TIMEOUT이라 계속 CCW 구동 중) |
| `g_dongjak.disc_pulse_off` | ~~배출문 간헐 구동 휴지구간~~ — **2026-08-25부터 항상 0**. 펄스 개시(15초)가 배출문 상한(14.3초)보다 늦어 도달하지 않는다 |

### 4.3 4.5 비상정지 / 시작·정지 제어
| 식 | 의미 |
|---|---|
| `g_dongjak.start_req` | 시작 래치(**중재자**/`dbg_force_start` → MotorTick 소비) |
| `g_dongjak.dbg_force_start` | **[쓰기]** 1 = HS 없이 강제 시작(1회성). **중재자 도입 후에도 그대로 유효** |
| `g_dongjak.dbg_force_stop` | **[쓰기]** 1 = 정지버튼 모사 → `DJ_ABORTED`(시나리오 자체 안전 식힘 경로) |
| `g_dongjak.abort_req` | 정지 요청 래치 |
| `g_dongjak.err_code` | `DjErrCode` DJ_ERROR 원인: ~~1 RINSE_CLOSE~~ / 2 RINSE_FILL / ~~3 RINSE_OPEN~~ / ~~4 HEAT_DOOR~~ / ~~5 DISCH_OPEN~~ / ~~6 DISCH_CLOSE~~ / ~~7 DISCH_WOPEN~~. **2026-08-25: 도어 관련 코드(1·3·4·5·6·7)는 발생하지 않는다** — 시간 상한이 정상 종료라 급수(2)만 남는다 |
| `g_dongjak.err_clear_req` | **[쓰기]** 1 = DJ_ERROR→IDLE 복구(1회성, `Dongjak_ClearError()`와 동일) |
| ~~`g_dongjak.lid_guard`~~ | **미사용**(`DJ_HS_TRIGGER_INTERNAL=0`). 마개 이탈 감시는 중재자의 `HS_LOST`가 담당 |
| ~~`g_dongjak.lid_low_cnt`~~ | **미사용**(상동). 디바운스는 `g_modearb.cand_cnt` |
| ~~`g_dongjak.hs_prev`~~ | **미사용**(상동). 위치 판정은 `g_modearb.pos_stable` |

> 위 3개는 `DJ_HS_TRIGGER_INTERNAL=1`(구 동작 복원)일 때만 갱신된다. 기본값 0에서는 0으로 고정이니
> 이 값들을 보고 마개 상태를 판단하지 말 것 — §8의 `g_modearb.*`를 볼 것.

---

## 5. 경과시간 Watch 식 (uwTick = HAL 틱 카운터 전역)

```text
(uwTick - g_dongjak.scn_start) / 60000      # 시나리오 경과(분)
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
| `g_dongjak.dbg_force_start = 1` | HS2 없이 시작(벤치, IDLE→RINSE1_CLOSE). 1회성 |
| `g_dongjak.dbg_enter_cool = 1` | **헹굼·건조 생략, 식힘 교반(80℃ 미만)부터.** 경과를 120분 지점으로 맞춰 넣어 10분 뒤 배출로 이어진다. 앱 `동작 (식힘부터)` 버튼 / `PROTO_ACT_DJ_COOL`(0x09) / `send_action.py dj-cool`. 1회성 |
| `g_dongjak.dbg_enter_heat = 1` | **헹굼 건너뛰고 DJ_HEAT부터 시작**(1=도어 닫힘·WHALL 대기부터, 리미트 직접 조작 시). MotorTick가 1회 소비. `scn_start`=now 리셋 → 경과 0부터. **먼저 동작 모드 진입**(위 dbg_disable+app_mode) 필요 |
| `g_dongjak.dbg_enter_heat = 2` | 위와 동일하되 **도어 대기까지 생략**(교반/분쇄/수증기/팬 정상 개시, 바로 가열중) |
| `g_dongjak.dbg_beep = N` | **[TEST] 즉시 N회 비프**(스피커/오디오 경로 확인용). ※`DJ_TEST_FAST_TIMING=1`일 때만 컴파일됨 — 현재 0이라 무효 |
| `g_dongjak.err_clear_req = 1` | **DJ_ERROR → IDLE 복구**(에러 원인은 `err_code`로 먼저 확인). `Dongjak_ClearError()`와 동일. ※동작 모드에서 tick이 돌아야 소비됨 |
| `g_jungji.dbg_stop_req = 1` | **DJ_ERROR 복구에 가장 확실** — `Jungji_Tick`은 모드/`dbg_disable` 무관하게 항상 돌며 `Dongjak_Abort()`(에러클리어 포함) 호출 → IDLE |
| `g_dongjak.dbg_force_stop = 1` | 처리 중 즉시 비상정지(`DJ_ABORTED`). **DJ_ERROR에서는 IDLE 복구**(정지=리셋 겸용) |
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
g_dongjak.err_code
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
> 대상 코드: [mode_arbiter.c](mode_arbiter.c) / [jungji.c](jungji.c).

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
| `g_modearb.cand` / `cand_cnt` | 디바운스 후보와 연속 카운트(`MODEARB_CONFIRM_SAMPLES`=3에 도달하면 확정) |
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
| `g_jungji.last_src` | 마지막 정지 사유(`JungjiSrc`): 0 NONE / 1 **HS3 정지** / 2 **마개 이탈** / 3 홀 다중인식 / 4 모드전환 / 5 시나리오 / 6 디버거 |
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
| 천천히 돌렸더니 엉뚱한 모드/정지 | 통과 위치(HS3/HS4)가 3샘플 확정된 것. `transitions` 증가 확인 → `MODEARB_CONFIRM_SAMPLES` 상향 검토 |
| 정지했는데 분쇄날이 계속 돔 | `g_jungji.last_kind`가 0(NORMAL=코스트)인지 확인. 1인데도 오래 돌면 `JUNGJI_BRAKE_MS` 상향 |
| 디버거로 `g_app_mode`를 썼는데 되돌아감 | 중재자가 소유자. `g_modearb.dbg_disable = 1` 선행 필요 |

---

## 9. (참고) 기타 튜닝 매크로 기본값

| 분류 | 매크로 = 값 |
|---|---|
| RPM | `DJ_STIR_RPM=30`(패턴 교반 공통: 건조·식힘80℃미만·배출) · **`DJ_STIR_RPM_HISPEED=40`(110분↑)** · `DJ_COOL_HOT_STIR_RPM=20`(식힘 80℃↑ 지속CW 전용) · `DJ_RINSE_STIR_RPM=30` · `DJ_STIR_RPM_HILOAD=30`(대기, 이제 증속 아님) · `DJ_GRIND_COARSE_RPM=1500` · `DJ_GRIND_FINE_RPM=1000` · `DJ_GRIND_FINAL_RPM=2000` |
| 헹굼 교반 | `DJ_RINSE_STIR_RPM=30` · `DJ_RINSE_CW_MS=6000` · `DJ_RINSE_STOP_MS=1000` · `DJ_RINSE_CCW_MS=6000` (1사이클 13s) · `DJ_RINSE_STIR_MS=117000`(9사이클, 2분 이하 정수 사이클) ★2026-08-26 |
| 건조·식힘·배출 교반 | `DJ_S313_CW_MS=6000` · `DJ_S313_STOP_MS=1000` · `DJ_S313_CCW_MS=6000` (1사이클 13s) ★2026-08-26. **건조 전용 패턴 A(`DJ_STIR_FWD/STOP/REV_MS`, `DJ_STIR_FWD_REPS`, `dj_stir_dry_*`, `stir_reps`)는 폐지**됐다 |
| 313 교반 | `DJ_S313_CW_MS=3000` · `DJ_S313_STOP_MS=1000` · `DJ_S313_CCW_MS=3000` |
| 분쇄 | 1차 토글 `DJ_GRIND_RUN_MS=3000`/`STOP_MS=2000` · 110분↑ 고속 **`DJ_GRIND_RUN_FINAL_MS=4000`/`STOP_FINAL_MS=2000`(4초구동/2초정지 토글)**, 최초 진입만 `DJ_GRIND_FINAL_DECEL_MS=1500`(급반전 방지 감속)+슬루 상승 후 토글 |
| 팬 | **FAN_VAPOR**(방수팬)=THERM3 100/84℃ vapor시퀀스(duty 없음) · **FAN_EXHAUST**(배기팬)=시나리오 전체 **`DJ_FANX_ON_MS=15분`/`OFF=2분` 독립 duty**(THERM3 분리, 2026-08-17; TEST=15초/2초) · **BLDC식힘팬** `DJ_FANB_ON_MS=30000`/`OFF=10000`(분쇄 동작 중) |
| 헹굼/배출 | `DJ_RINSE_STIR_MS=120000`(2분) · `DJ_RINSE1_DRAIN_MS=30000`(30초) · `DJ_RINSE2_DRAIN_MS=90000`(90초,잔수포함) · `DJ_FILL_EXTRA_MS=2000` · 배출 교반 종료=**TIMER-OUT(PF8)↓** (백업 `DJ_DISCHARGE_STIR_MS=120000`) / **배출문 닫기 개시=`DJ_T_LATCH_MS=135분`** |
| 스테퍼 | **구동시간 기준(2026-08-25 벤치확정, 스텝수 기준 폐지)**: `DJ_STEP1_RUN_MS=15000`(관로 개/폐 각 15s, 방향무관) · `DJ_STEP2_OPEN_MS=1000`(흡입 열림) · `DJ_STEP2_CLOSE_MS=2000`(흡입 닫힘) · STEP1 `DJ_STEP_INTERVAL_MS=3`(333PPS)/`START=10`(100PPS) · STEP2 `DJ_STEP2_INTERVAL_MS=3`(333PPS, **2026-08-25 6→3 복귀**)/**`DJ_STEP2_START_MS=20`(50PPS 시작램프 유지 — 1s 창의 488ms를 먹으므로 이동량 부족 시 1순위 조정 대상)** · 램프 `DJ_STEP_RAMP_STEPS=40`(선형가속) · **`DJ_STEP_RELEASE_DUCT_ON_OPEN=1`**(STEP1 개방후 코일해제→STEP2에 레일전류 양보) · 개폐 후 정지 |
| 식힘 80℃ | 분쇄 OFF·교반313 전환 조건 = **(THERM1 && THERM2 둘 다 유효 && <`DJ_TEMP_COOL_GRIND_OFF_D10`=800) ‖ 바이메탈80 하강엣지**. `DJ_COOL_USE_THERM2=1`(THERM2 AND 추가, CH1=PC1), `DJ_COOL_USE_BIMETAL80=1`(`GPIO_EXTI_BIMETAL_80` PF0). 보수적: 써미스터 에러 시 '안 식음'. COOLDOWN 진입 시 stale 엣지 클리어 |
| 수거통 | `DJ_MAX_CYCLES=6` · `DJ_BIN_FULL_PCT=90` · `DJ_BIN_CHECK_ENABLE=1` |
| 채널/정책 | `DJ_HS_START_IDX=1`(HS2) · `DJ_TEMP_CH=0`(THERM1) · **`DJ_VAPOR_TEMP_CH=2`(THERM3/J23,PC2)** · `DJ_WATER_ACTIVE_LOW=1` · `DJ_FILL_USE_WATER_ON=1` · `DJ_DRAIN_VALVE_USE=1` |
| 안전 | `DJ_FILL_TIMEOUT_MS=120000` · `DJ_LID_CONFIRM_SAMPLES=3`. **도어(2026-08-25 이관)**: `WDOOR_CLOSE_DUTY=80`/`WDOOR_CLOSE_MS=4200` · `WDOOR_OPEN_KICK_DUTY=80`/`WDOOR_OPEN_KICK_MS=2000`/`WDOOR_OPEN_RUN_DUTY=65`/`WDOOR_OPEN_MAX_MS=6000`(wdoor.h) · `TDOOR_DUTY=80`/`TDOOR_OVERRUN_MS=1000`/`TDOOR_OPEN_MAX_MS`=`TDOOR_CLOSE_MAX_MS`=`14300`(tdoor.h). ~~`DJ_DOOR_TIMEOUT_MS`·`DJ_DOOR_BENCH_MS`·`DJ_DOOR_DUTY_PCT`~~ 폐지 |
