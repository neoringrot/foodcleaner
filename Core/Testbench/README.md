# Testbench (개별 부품 검사용 테스트베드)

각 액추에이터를 **시나리오(모음/동작)와 무관하게 개별적으로** 구동/검사하기 위한 테스트베드 모음.
모든 테스트베드는 `volatile` 전역 변수를 노출하며, **디버거(live watch / expression)에서 값을 바꾸면 즉시 반영**된다.

## ⚠️ 2026-08-15 변경 — 벤치 사용 전 반드시 읽을 것

`g_app_mode`의 소유자가 바뀌었다. 이제 [`Core/Scenario/mode_arbiter.c`](../Scenario/mode_arbiter.c)가
**마개 위치 홀센서(HS1~HS5)를 모드와 무관하게 100 ms마다 디코딩**해서 모드를 결정한다.
벤치 작업에 미치는 영향은 3가지다:

1. **마개(자석)가 HS1/HS2/HS4/HS5 위치에 얹혀 있으면 벤치 모드가 유지되지 않는다.**
   중재자가 해당 시나리오 모드로 전환해 버리므로 `TB_*_Poll()`이 통째로 멈춘다.
   → 벤치를 쓸 때는 자석을 치우거나, **`g_modearb.dbg_disable = 1`** 로 중재자를 꺼 둘 것.
2. **정지가 걸리면 `tb_*_enable` 플래그가 전부 0으로 눌린다.**
   HS3(정지) 확정 / 마개 이탈 / 홀 다중 인식 시 [`jungji.c`](../Scenario/jungji.c)의
   `Jungji_Testbench()`가 `tb_wdoor_enable`·`tb_tdoor_enable`·`tb_step1/2_enable`·`tb_lift_enable`·
   `tb_water_enable`·`tb_speaker_enable`·`tb_heat_enable`·`tb_valve_*`·`tb_fan_*`를 모두 0으로 만든다.
   **벤치 중 갑자기 다 꺼졌다면 `g_jungji.last_src`를 먼저 볼 것**(1=HS3 정지, 2=마개 이탈, 3=홀 다중 인식).
   예외: 잔열 냉각 중(`g_jungji.cooling=1`)에는 `tb_fan_exhaust_en`/`tb_bldc_fan_en`이 **1로 유지**된다.
3. **BLDC 단락제동 홀드(`JUNGJI_BRAKE_MS`=500 ms) 동안 키패드(`TB_TCA9554_Poll`)가 건너뛰어진다.**
   `BldcCtrl_Start()`가 nBRAKE를 풀어 버리기 때문. `g_jungji.braking`으로 확인 가능.

관찰용 변수 정리는 [동작_디버거_watch_레퍼런스.md §8](../Scenario/동작_디버거_watch_레퍼런스.md) 참조.

## 공통 동작 모델

- 모든 `TB_*_Poll()`은 `freertos.c`의 `MotorTask_RunTestbench()`에서만 호출된다
  → **테스트벤치 모드(`AppMode_IsBenchIdle()` = 대기 `APP_MODE_TESTBENCH` 또는 정지 `APP_MODE_JUNGJI`, 부팅 기본값은 대기)에서만** 동작하고,
  모음/동작/강음/배수 시나리오 모드에서는 자동으로 무시된다
  (해당 모드에서는 시나리오 코드가 동일 액추에이터를 직접 소유).
  단 위 ⚠️ 항목대로 **모드를 정하는 주체가 마개 홀센서**라는 점에 주의.
- `TB_*_Init()`은 `StartMotorTask()` 초기화 시퀀스에서 1회 호출된다.
- 폴링 주기는 `osDelay(1)` (~1 ms).
- 빌드: `.cproject`가 `Core` 폴더 전체를 소스 경로로 포함 → Testbench에 `.c`를 추가하면 자동 컴파일(별도 등록 불필요).

## 구성 파일 / 담당 액추에이터

| 파일 | 대상 | 드라이버 | 제어 방식 |
|------|------|----------|-----------|
| `tb_tca9554` | 분쇄(M1)·교반(M2) BLDC | DRV8306 + bldc_ctrl (closed-loop) | 키패드(U9) + **디버거 enable 변수** |
| `tb_drv8871` | 배수문(WDoor/U5)·투입문(TDoor/U7) | DRV8871 DC | 디버거 enable/duty/reverse |
| `tb_stepmotor` | STEP1(24BYJ48)·STEP2(35BYJ46) | 유니폴라 4상 | 디버거 enable/dir/period/hold |
| `tb_lift` | 리프트(U6) | DRV8871 DC (SW-PWM) | 디버거 enable/reverse/duty/period |
| `tb_gpioout` | 배수/급수 밸브 + 팬 3종 | GPIO on/off | 디버거 enable 변수 |
| `tb_thermistor` | NTC 온도센서 3종(THERMISTER1/2/3) | ADC1(adc_ctrl) | 디버거 enable 변수(모니터링) |
| `tb_water` | 급수 WATER-ON + 수위센서 SEN1/SEN2 | GPIO out + EXTI in | 디버거 enable 변수(출력+모니터링) |
| `tb_doorhall` | 도어 리밋홀 WHALL/THALL(배수문·배출문 개폐) | EXTI in | 디버거 enable 변수(모니터링, 입력전용) |
| `tb_protocol` | R0 프로토콜 코덱(액추에이터 없음) | protocol_r0 | 디버거 `tb_proto_run_once=1`(1회) / `tb_proto_enable=1`(반복) |

> **ADC 계열 예외(중요)**: `tb_thermistor`는 **센서 모니터** 테스트베드로, 위 "공통 동작 모델"과 달리
> `MotorTask_RunTestbench()`가 아니라 **`StartDefaultTask()`(~100 ms)** 에서 폴링한다.
> 이유: ADC1 읽기는 재진입 불가라서 단일 태스크(defaultTask)만 소유해야 한다(`adc_ctrl.h` 주석).
> 따라서 `g_app_mode`(테스트벤치/시나리오)와 **무관하게** 항상 폴링되지만, `tb_therm_enable`이 0이면
> 아무 읽기도 하지 않으므로 사실상 정지 상태다.

---

## 개별기능검증R1.xls 대조 현황 (2026-08-12)

`doc/개별기능검증R1.xlsx`의 항목별 HW 검증 진행 상태. (✅ HW검증완료 · ⏳ 미검증 · ⚠️ 부분/이슈 · ❌ 없음)

> 미검증 항목의 **상세·검증절차·그동안 코드에 넣어 둔 임시 완화 플래그**는
> **[../Scenario/HW미검증_항목.md](../Scenario/HW미검증_항목.md)** 에 정리되어 있다(2026-08-18).

| # | xls 항목 | 테스트벤치 / 변수 | 핀·커넥터 | 상태 |
|---|----------|-------------------|-----------|------|
| 1 | 수중기배출팬(증기) | `tb_fan_vapor_en` | PB15 | ✅ HW 검증완료 |
| 2 | 환풍팬(배기) | `tb_fan_exhaust_en` | PG2 | ✅ HW 검증완료 |
| 3 | 환풍팬(BLDC 냉각) | `tb_bldc_fan_en` | PG1 | ✅ HW 검증완료 |
| 4 | 수위센서 | `tb_water` (SEN1/SEN2) | PF6/PF7 | ✅ HW 검증완료 |
| 5 | 배수구세척솔벨브 | `tb_valve_drain_en` | PB14 | ✅ HW 검증완료 |
| 6 | 건조통 급수 솔벨브 | `tb_valve_dry_en` | PB13 | ✅ HW 검증완료 |
| 7 | STEP 공기배출 DOOR | `tb_step1_*` | **J31** (24BYJ48-895) | ✅ HW 검증완료 |
| 8 | STEP 공기흡입 DOOR | `tb_step2_*` | **J33** (35BYJ46-1014) | ✅ HW 검증완료 (2026-08-15, 회전확인). 공통=**Yellow**(pin1), pin2 Red/pin3 Orange/pin4 Blue/pin5 Pink. **24V 확정(2026-08-17)** |
| 9 | DC 배수부 모터(DOOR) | `tb_wdoor_*` | U5 (DRV8871) | ✅ HW 검증완료 |
| 10 | DC 배출부 모터(DOOR) | `tb_tdoor_*` | U7 (DRV8871) | ✅ HW 검증완료 (2026-08-15, 회로 담당자 검증) |
| 11 | 상단 동작 스위치(=HS1~5) | `tb_hall_*` (P0~P4) | U24/J26 | ✅ HW 검증완료 (HS1~3 초기 단락은 **리워크로 해소**, 2026-08-14 §6) |
| 12 | 교반원점 홀센서 | `tb_hall_stir_home` (HS6) | U24 P5/J28 | ✅ HW 검증완료 (2026-08-15) · ❌ **FW 소비 로직 없음** |
| 13 | 수거통 홀센서 | `tb_hall_bin` (HS7) | U24 P6/J29 | ✅ HW 검증완료 (2026-08-15) · ❌ **FW 소비 로직 없음 → 수거통 없이 배출됨(안전 공백)** |
| 14 | 리프트 하단 홀센서 | `tb_hall_lift_bottom` (HS8) | U24 P7/J30 | ✅ HW 검증완료 (2026-08-15) · ❌ **FW 소비 로직 없음** |
| 15 | 히터 | `tb_heat_*` (`tb_heat`) | **PA12** (HT-POWER) | ⏳ 미검증 (테스트벤치 추가됨 §9). ⚠️`tb_heat` 기본임계 **190/195℃는 폐기된 구 스펙** — 시나리오는 **114/117℃**(`DJ_TEMP_HEATER_ON/OFF_D10`) |

**xls 미포함(테스트벤치에만 있는) 액추에이터:**

| 항목 | 테스트벤치 / 변수 | 상태 |
|------|-------------------|------|
| 분쇄 BLDC (M1) | `tb_grind_en` 외 (`tb_tca9554`) | ✅ HW 검증완료 |
| 교반 BLDC (M2) | `tb_stir_en` 외 (`tb_tca9554`) | ✅ HW 검증완료 |
| 리프트 모터 (U6) | `tb_lift_enable` 외 (`tb_lift`) | ✅ HW 검증완료 |
| NTC 온도센서 3종 | `tb_therm_*` (`tb_thermistor`) | ⏳ read 동작 O / **기준 온도계 실측 대조 미완** (§7) |
| 홀센서 HS1~HS8 (U24 8채널) | `tb_hall_*` (P0~P7) | ✅ **전 채널 정상 동작 확인** (2026-08-15, §6) |
| 도어 리밋홀 WHALL/THALL (PF3/PF4/PF2/PF5) | `tb_doorhall_*` (`tb_doorhall`) | ⏳ 테스트벤치 추가됨 · HW 미검증 (§10). 배수문/배출문 개폐 게이팅 |

---

## 1. `tb_tca9554` — 분쇄(M1)·교반(M2) BLDC

키패드 버튼과 **동일한 효과**를 내는 디버거 enable 변수를 추가함(이번 세션 작업).
BLDC는 closed-loop(PI + soft-lock jam 보호)이라, enable은 **edge-triggered**로 버튼 1회 누름을 모사한다.

| 변수 | 동작 | 대응 버튼 |
|------|------|-----------|
| `tb_grind_en` | 0→1 시작 / 1→0 정지 (분쇄 M1) | SW1/SW3/SW5 |
| `tb_grind_rev` | 시작 방향 (0=정방향, 1=역방향), start 시점만 반영 | SW1 vs SW5 |
| `tb_grind_spd_req` | 1 쓰면 속도 1단계↑ (800→1200→1600→2000→2500 rpm), 자동 0 | SW7 |
| `tb_stir_en` | 0→1 시작 / 1→0 정지 (교반 M2) | SW2/SW4/SW6 |
| `tb_stir_rev` | 시작 방향 (0=정방향, 1=역방향), start 시점만 반영 | SW2 vs SW6 |
| `tb_stir_spd_req` | 1 쓰면 속도 1단계↑ (20→25→30→35→40 rpm, 출력축), 자동 0 | SW8 |

주의:
- **edge 방식**: `en`은 0→1 순간에만 Start, 1→0 순간에만 Stop 호출(매 poll 재호출 아님).
- **방향 변경**: 회전 중 `rev`를 바꿔도 반영 안 됨 → `en=0` 후 `rev` 변경 후 `en=1`.
- **jam soft-lock**: 잼 감지 시 컨트롤러가 스스로 정지(`en`은 1로 남음). 재시도하려면 `en`을 0→1 재토글.
- `BldcCtrl_Tick(&g_grind_ctrl / &g_stir_ctrl)`가 매 사이클 돌아야 회전 유지(freertos.c에서 보장).

키패드 사용 시 매핑: 홀수 SW1/3/5/7=M1(분쇄), 짝수 SW2/4/6/8=M2(교반). (보드 배선상 U8/U9 주소는 넷리스트와 반전됨 — 헤더 주석 참조.)

## 2. `tb_drv8871` — 배수문(WDoor/U5)·투입문(TDoor/U7)

DC 도어 모터. **레벨(level) 방식** (enable 동안 계속 구동, VM 전원은 전환 시에만 토글).

| 변수 | 동작 | 기본값 |
|------|------|--------|
| `tb_wdoor_enable` | 1=구동 / 0=정지(coast+VM off) — 배수문 | 0 |
| `tb_wdoor_duty` | PWM 0~100 % — **프로파일 ON 중에는 펌웨어가 덮어쓰는 현재 duty 표시값** | 50 |
| `tb_wdoor_reverse` | **0=닫힘(Close), 1=열림(Open)** — 벤치 확정 (투입문과 반대!) | 0 |
| `tb_tdoor_enable` | 1=구동 / 0=정지 — 투입문 | 0 |
| `tb_tdoor_duty` | PWM 0~100 % — **프로파일 ON 중에는 펌웨어가 덮어쓰는 현재 duty 표시값** | 50 |
| `tb_tdoor_reverse` | 0=열림(Open), 1=닫힘(Close) — 벤치 확정 (배수문과 반대!) | 0 |
| `tb_wdoor_limit_stop` / `tb_tdoor_limit_stop` | 1=리밋 도달 시 자동 정지 / 0=프리런(구 동작) | 1 |
| `tb_wdoor_limit_hit` / `tb_tdoor_limit_hit` | (읽기전용) 1=리밋으로 자동 정지됨 | 0 |
| `tb_wdoor_profile` | 1=방향별 duty/시간 프로파일 / 0=`tb_wdoor_duty` 수동 | 1 |
| `tb_wdoor_open_kick_duty` | [열림] 기동 구간 duty [%] | 80 |
| `tb_wdoor_open_kick_ms` | [열림] 기동 구간 길이 [ms] | 2000 |
| `tb_wdoor_open_run_duty` | [열림] 기동 후 유지 duty [%] | 65 |
| `tb_wdoor_open_ms` | [열림] 최대 동작 시간 [ms] (리밋 미인식 시 백스톱) | 6000 |
| `tb_wdoor_close_duty` | [닫힘] duty [%] (전 구간 고정) | 80 |
| `tb_wdoor_close_ms` | [닫힘] 동작 시간 [ms], 만료 시 자동 정지 | 4200 |
| `tb_wdoor_time_hit` | (읽기전용) 1=시간 만료로 정지됨 (리밋 아님) | 0 |
| `tb_tdoor_profile` | 1=투입문 duty/추가회전 프로파일 / 0=`tb_tdoor_duty` 수동 | 1 |
| `tb_tdoor_run_duty` | [투입문] duty [%] (전 구간 고정) | 80 |
| `tb_tdoor_overrun_ms` | [투입문] 홀 인식 후 **추가 회전 시간** [ms] | 1000 |
| `tb_tdoor_open_max_ms` | [투입문/열림] 최대 동작 시간 [ms] | 14300 |
| `tb_tdoor_close_max_ms` | [투입문/닫힘] 최대 동작 시간 [ms] | 14300 |
| `tb_tdoor_overrun` | (읽기전용) 1=홀 인식됨, 추가 회전 중 | 0 |
| `tb_tdoor_time_hit` | (읽기전용) 1=상한 시간으로 정지됨 (홀 아님) | 0 |

**배수문 방향별 프로파일 (기본 ON, WDoor 전용).** 방향은 벤치 확정 — `tb_wdoor_reverse` **0=닫힘 / 1=열림**
(투입문 `tb_tdoor_reverse`와 반대다. 두 문의 모터 배선 방향이 서로 반대라 그렇다).
두 방향의 동작이 다르며, 기준 시각은 둘 다 "구동 시작 시점"이다.

| 방향 | duty | 정지 조건 |
|------|------|-----------|
| 열림 (`reverse=1`) | 80 %(2 s) → 65 % 유지 | **WHALL-OPEN 리밋** (`tb_wdoor_limit_hit=1`). 리밋 미인식 대비 **6 s 상한**(`tb_wdoor_time_hit=1`) |
| 닫힘 (`reverse=0`) | 80 % 고정 | **4.2 s 경과**(`tb_wdoor_time_hit=1`). 그 전에 WHALL-CLOSE 인식되면 리밋으로 정지 |

구동 시작 = `tb_wdoor_enable`의 0→1 엣지. 구동 중 `tb_wdoor_reverse`를 뒤집으면 새 구동으로 보고
기준 시각이 재시작된다(정지 상태에서 다시 떼어내야 하므로).
프로파일이 켜져 있는 동안 `TB_DRV8871_Poll()`이 적용 중인 duty를 `tb_wdoor_duty`에 **직접 써 넣으므로**,
디버거에서 그 변수로 현재 duty를 그대로 볼 수 있다. 반대로 그 사이에 손으로 duty를 쓰면 다음 폴에서
덮어써진다 — 수동으로 잡고 싶으면 `tb_wdoor_profile = 0` (이때는 닫힘 4.2 s 시간제한도 적용되지 않는다).

**투입문 프로파일 (기본 ON, TDoor 전용).** `tb_tdoor_enable=1`이면 `tb_tdoor_run_duty`(80 %) 고정으로 구동한다.
배수문과 달리 **홀 리밋에서 즉시 서지 않는다** — 구동 방향의 홀(`TDoor_AtOpen()`/`AtClose()`)이 처음 확정되는
순간 추가회전 구간을 걸어두고 `tb_tdoor_overrun_ms`(1000 ms) 동안 계속 돌린 뒤 coast + VM off,
`tb_tdoor_enable`을 0으로 내리고 `tb_tdoor_limit_hit=1`을 세운다. 그 사이 `tb_tdoor_overrun=1`.
추가회전 구간은 **래치**다 — 문이 자석을 지나가 홀 레벨이 풀려도 1초를 끝까지 채운다.
(구동 중 `tb_tdoor_reverse`를 뒤집으면 반대쪽 리밋을 향한 새 구동으로 보고 래치를 버린다.)
`tb_tdoor_profile = 0`이면 `tb_tdoor_duty` 수동 + 홀 인식 즉시 정지(추가회전 0).
방향은 벤치 확정 — `tb_tdoor_reverse` 0=열림 / 1=닫힘. 두 방향 모두 동작이 같다(80 % 고정 + 홀 후 1 s 추가회전).

| 방향 | duty | 정지 조건 |
|------|------|-----------|
| 열림 (`reverse=0`) | 80 % 고정 | THALL-OPEN 인식 → **+1 s 후** 정지(`tb_tdoor_limit_hit=1`). 상한 **14.3 s**(`tb_tdoor_time_hit=1`) |
| 닫힘 (`reverse=1`) | 80 % 고정 | THALL-CLOSE 인식 → **+1 s 후** 정지(`tb_tdoor_limit_hit=1`). 상한 **14.3 s**(`tb_tdoor_time_hit=1`) |

상한은 방향별 변수로 **개별 관리**한다(`tb_tdoor_open_max_ms` / `tb_tdoor_close_max_ms`, 기본 둘 다 14300).
상한은 구동 시작(=`tb_tdoor_enable` 0→1 엣지, 방향 전환 시 재시작)부터 재는 **구동 전체의 하드 상한**이라,
상한 직전에 홀이 인식되면 추가회전 1 s를 다 채우지 못하고 상한에서 잘린다(최대 구동시간이 14.3 s를 넘지 않음).

**시나리오(모음·동작)와 값 공유.** 위 프로파일의 실제 수치는 이 벤치가 아니라 드라이버 헤더에 있다 —
배수문은 `wdoor.h`의 `WDOOR_OPEN_KICK_DUTY/KICK_MS/RUN_DUTY/MAX_MS`·`WDOOR_CLOSE_DUTY/CLOSE_MS`,
배출문은 `tdoor.h`의 `TDOOR_DUTY`·`TDOOR_OVERRUN_MS`·`TDOOR_OPEN_MAX_MS`/`TDOOR_CLOSE_MAX_MS`.
`moeum.c`/`dongjak.c`가 같은 매크로를 쓰므로 **벤치에서 확인한 동작 = 제품 시나리오 동작**이다.
벤치의 `tb_*` 변수는 그 값을 런타임에 흔들어보기 위한 복사본이고, 확정값은 헤더에 반영해야 한다.

**리밋 자동 정지 (기본 ON).** `tb_*_limit_stop=1`이면 `TB_DRV8871_Poll()`이 *구동 중인 방향의* 홀 리밋만 감시한다 —
`tb_*_reverse=0`(Open)이면 `WDoor_AtOpen()`/`TDoor_AtOpen()`, `=1`(Close)이면 `AtClose()` (시나리오와 동일한 디코드).
연속 `TB_DOOR_LIMIT_CONFIRM`(5회 ≈ 5 ms @MotorTask 1 ms) 동안 asserted면 `Stop()`(coast) + `Disable()`(VM off) 후
`tb_*_enable`을 스스로 0으로 내리고 `tb_*_limit_hit=1`을 세운다.
**단 투입문(TDoor)은 즉시 서지 않고 `tb_tdoor_overrun_ms`(1 s)만큼 더 돈 뒤 정지한다** — 위 투입문 프로파일 참고.
반대 방향 리밋은 막지 않으므로,
리밋 위에 얹힌 상태에서 `tb_*_reverse`를 뒤집고 다시 `enable=1` 하면 리밋에서 빠져나올 수 있다.
방향 미확정이거나 홀이 미결선이면 `tb_*_limit_stop=0`으로 두되 — 그 경우 **엔드스톱에서 계속 스톨**하므로
직접 `tb_*_enable=0`을 써서 멈춰야 한다.

(WDoor = 배수문 = 실질적 배수구 도어. 시간 제어는 없으므로 duty·시간은 여전히 직접 관리.)

## 3. `tb_stepmotor` — STEP1·STEP2 스테퍼

유니폴라 4상. `HAL_GetTick()` 기반으로 `period_ms`마다 1스텝 진행. 커넥터 핀맵(확정):
- STEP1(24BYJ48-895/J31): pin1 Red(공통)/pin2 Orange/pin3 Yellow/pin4 Pink/pin5 Blue.
- STEP2(35BYJ46-1014/J33): pin1 **Yellow(공통)**/pin2 Red/pin3 Orange/pin4 Blue/pin5 Pink
  (2026-08-15 회전확인, 공통이 Red 아닌 Yellow). **24V 확정(2026-08-17)** — 24V/~300Ω 실장.

| 변수 | 동작 |
|------|------|
| `tb_step1_enable` / `tb_step2_enable` | 1=구동 / 0=정지 |
| `tb_step1_dir` / `tb_step2_dir` | 0=정방향, 1=역방향 |
| `tb_step1_period_ms` / `tb_step2_period_ms` | 스텝당 ms (≥1); 작을수록 빠름(과속 시 탈조) |
| `tb_step1_hold` / `tb_step2_hold` | 0=정지 시 코일 해제, 1=토크 유지 |

## 4. `tb_lift` — 리프트(U6)

DRV8871 DC. PG3/PG4는 타이머 채널이 없어 **소프트웨어 PWM**(`TB_Lift_Poll()` 내 tick 기반)으로 속도 제어.

| 변수 | 동작 |
|------|------|
| `tb_lift_enable` | 1=구동 / 0=정지 |
| `tb_lift_reverse` | 0=Up, 1=Down (회전 중 변경 시 즉시 반전 — 정지 상태에서 변경 권장) |
| `tb_lift_duty` | 0~100 % (0=coast) |
| `tb_lift_pwm_period_ms` | SW-PWM 주기(≥1 ms). 10 ms≈100 Hz/~10 % 스텝, 20 ms≈50 Hz/~5 % 스텝 |

## 5. `tb_gpioout` — 밸브·팬 (on/off GPIO) — 이번 세션 신규 파일

모터 드라이버가 없는 단순 ON/OFF GPIO 부하(밸브 2 + 팬 3). duty/방향 없이 enable 하나씩.
**레벨 미러링**: `TB_GpioOut_Poll()`이 매 poll 각 플래그를 `gpio_ctrl_set()`으로 핀에 반영(stateless). 부팅 시 전부 off.

| 변수 | 대상 | 핀 | 넷 |
|------|------|----|----|
| `tb_valve_drain_en` | 배수 밸브 | PB14 | o_VALVE_DRAIN_CLN |
| `tb_valve_dry_en` | 급수 밸브 | PB13 | o_VALVE_DRY_IN |
| `tb_fan_vapor_en` | 증기 팬 | PB15 | o_FAN_VAPOR |
| `tb_fan_exhaust_en` | 배기 팬 | PG2 | o_FAN_EXHAUST |
| `tb_bldc_fan_en` | BLDC 냉각 팬 | PG1 | o_BLDC_FAN |

참고:
- `GPIO_OUT_VALVE_DRAIN_CLN`(PB14)은 배수 **밸브 솔레노이드**(모터 아님), 배수문 **모터(WDoor)**는 `tb_wdoor_*`로 별도 제어. 실제 배수는 둘을 함께 여는 구조.
- 급수 라인에는 `GPIO_OUT_WATER_ON`(PE2)도 관여할 수 있음(현재 tb 변수 미포함).

## 6. `tb_hallsensor` — U24 홀센서 모니터 (이번 세션 신규 파일)

U24(TCA9554A, I2C1, 0x3B)에 연결된 홀센서 8입력(P0~P7)을 **모니터링**하는 테스트베드.
액추에이터가 아닌 **센서 모니터**라 다른 tb와 폴링 위치가 다르다:

- **`freertos.c`의 `StartDefaultTask`에서 폴링**(U24 단일 소유 태스크). `MotorTask`가 아니므로
  **`g_app_mode`와 무관하게(테스트벤치/모음/동작 모드 전부) 항상 모니터**된다.
- 프로덕션 `hallsensor.c` 드라이버(`HallSensor_ServiceInt/_Update/_GetMask`)를 재사용 →
  값이 펌웨어와 100 % 일치. 이 tb는 마스크를 채널로 분해만 한다.
- **`tb_hall_enable`** = 1 → 매 poll에서 HALL-INT1 서비스 + U24 읽어 스냅샷 갱신 / 0 → 정지(마지막 스냅샷 동결, 버스 미접근).

**인터럽트**: U24가 입력 변화 시 HALL-INT1(PF9, active-low OC, R117 4.7K 풀업)을 LOW로 당김 →
`HallSensor_ServiceInt()`로 엣지 서비스 후 주기 읽기를 안전망으로 수행.

### 채널 맵 (마스크 bit i = HS(i+1) = U24 Pi, 1 = 자석 검출)

| U24 핀 | bit | 신호 | 변수(레벨) | 엣지 카운터 |
|--------|-----|------|-----------|-------------|
| P0 (HS1) | 0 | 강음(강한음식물) | `tb_hall_trig[0]` / `tb_hall_hard_food` | `tb_hall_trig_events[0]` |
| P1 (HS2) | 1 | 동작 | `tb_hall_trig[1]` / `tb_hall_run` | `tb_hall_trig_events[1]` |
| P2 (HS3) | 2 | 정지 | `tb_hall_trig[2]` / `tb_hall_stop` | `tb_hall_trig_events[2]` |
| P3 (HS4) | 3 | 배수 | `tb_hall_trig[3]` / `tb_hall_drain` | `tb_hall_trig_events[3]` |
| P4 (HS5) | 4 | 모음 | `tb_hall_trig[4]` / `tb_hall_collect` | `tb_hall_trig_events[4]` |
| P5 (HS6) | 5 | 교반원점 | `tb_hall_stir_home` | `tb_hall_stir_home_events` |
| P6 (HS7) | 6 | 수거통 | `tb_hall_bin` | `tb_hall_bin_events` |
| P7 (HS8) | 7 | 리프트하단 | `tb_hall_lift_bottom` | `tb_hall_lift_bottom_events` |

기타 관찰 변수:
- `tb_hall_mask` : 검출 마스크(P0~P7), `tb_hall_trig_mask` : 트리거군(P0~P4) 패킹(0~0x1F)
- `tb_hall_pinlevel` : **실제 핀 전압 레벨(1=HIGH)** = `~tb_hall_mask` (HW 폴라리티 0xFF 반전). 배선 결함 추적용 진단 뷰.
- `tb_hall_samples`(enable 중 poll마다 ++), `tb_hall_int_events`(HALL-INT1 서비스 횟수), `tb_hall_changes`(마스크 변화 횟수)

넷리스트 배선: HS1~HS5 → 커넥터 **J26**(핀 2~6), HS6/HS7/HS8 → **J28/J29/J30**. HS 라인에 **풀업 저항 없음**(각 네트 = J핀 + U24핀 2점).

> **★2026-08-25 라벨 재정의**: 마개 순서가 `강음 / 동작 / 정지 / 배수 / 모음` 으로 바뀌어 **HS2 ↔ HS5 의 의미(모음/동작)가 맞교환**되었다. 배선·비트 위치는 그대로이고 라벨만 바뀐 것이라, 펌웨어에서는 `mode_arbiter.c` 디코드 표 · `MOEUM_HS_START_IDX`(1→4) · `DJ_HS_START_IDX`(4→1) · `TB_HALL_BIT_COLLECT`/`TB_HALL_BIT_RUN` 만 반대로 잡았다.

> **★2026-08-15 — 이 모니터는 이제 "구경만 하는" 채널이 아니다.**
> `tb_hall_trig_mask`(P0~P4)와 **완전히 같은 비트**를 [`mode_arbiter.c`](../Scenario/mode_arbiter.c)가
> 소비해서 실제 모드 전환·정지를 일으킨다(`HallSensor_GetMask() & 0x1F`). 즉 여기서 보이는 트리거군
> 비트가 곧 동작이다. 대응 관계:
>
> | `tb_hall_trig_mask` | `g_modearb.pos_stable` | 결과 |
> |---|---|---|
> | `0x01` HS1 | 1 `LID_POS_KANGEUM` | `APP_MODE_KANGEUM`(스텁) |
> | `0x02` HS2 | 5 `LID_POS_DONGJAK` | `APP_MODE_DONGJAK` + `Dongjak_Start()` |
> | `0x04` HS3 | 3 `LID_POS_JUNGJI` | **비상정지** + `APP_MODE_JUNGJI`(5) (★2026-08-25, 구: 대기 복귀) |
> | `0x08` HS4 | 4 `LID_POS_BAESU` | `APP_MODE_BAESU`(스텁) |
> | `0x10` HS5 | 2 `LID_POS_MOEUM` | `APP_MODE_MOEUM` + `Moeum_Start()` |
> | `0x00` | 0 `LID_POS_NONE` | **비상정지**(마개 이탈), 모드는 유지 |
> | 2비트 이상 | 6 `LID_POS_MULTI` | **비상정지**(홀 이상으로 판정) |
>
> `tb_hall_enable`을 0으로 두어도 **중재자는 계속 동작한다**(프로덕션 `hallsensor.c`를 직접 읽으므로).
> 중재자를 멈추려면 `g_modearb.dbg_disable = 1`. 반대로 **자석 없이 위치를 흉내내려면**
> `g_modearb.dbg_pos_force`에 위 `pos_stable` 값을 써 넣으면 된다(0 = 미사용).
> HS6~HS8(교반원점/수거통/리프트하단)은 마개 위치가 아니므로 중재자 대상이 아니다 — **소비 로직 TODO**.

### 검증 진행 상태 / 이슈 (2026-08-15 현재)

| 채널 | 핀 | 상태 |
|------|----|------|
| HS1 강음 / HS2 동작 / HS3 정지 / HS4 배수 / HS5 모음 | P0~P4 | ✅ **회로 수정 후 정상 동작 확인** |
| HS6 교반원점 / HS7 수거통 / HS8 리프트하단 | P5 / P6 / P7 | ✅ **정상 동작 확인 (2026-08-15)** |

> **U24 홀센서 8채널(HS1~HS8) 전부 검증 완료.** HS1/HS2/HS3(P0/P1/P2)의 초기 전기적 커플링(아래 원인 분석)은
> **회로 수정(리워크)으로 해소**되었고, 전 채널이 단독 트리거 시 해당 비트만 반응하는 것을 확인함.

**[해결됨] HS1/HS2/HS3 (P0/P1/P2) 초기 문제 — 원인 분석 기록:**

- 증상: 셋 중 **하나만 트리거해도 3비트가 동시에 1**로 올라옴. `[3][4]`(P3/P4)는 정상.
- 펌웨어 배제: `decode_mask()`의 `[0]~[4]`는 `(mask>>i)&1`로 완전 대칭이고 읽기 경로(`TCA9554_ReadInput`→1바이트)도 비트별 처리 없음 →
  코드가 특정 세 비트만 묶을 수 없음. 넷리스트 매핑·주소(0x3B)도 정확. 즉 **U24 INPUT 레지스터 단계에서 이미 세 비트가 묶여 올라옴**.
- `tb_hall_pinlevel`이 **흔들리지 않고 안정적으로 함께 움직임** 확인 → 고임피던스 플로팅 픽업이 아니라
  **저임피던스 단락/브리지**로 세 네트가 한 노드로 묶인 상태로 판정.
- 물리 의심 지점: **U24 핀 4·5·6(P0/P1/P2 인접, P3=핀7은 제외됨과 일치)** 솔더 브리지, 또는 **J26 핀 2·3·4** 브리지/이물.

**확인·조치 절차(전원 OFF, 저항 측정):**
1. HS1–HS2, HS2–HS3, HS1–HS3 저항 → 0Ω 근처면 단락 확정 (대조: HS3–HS4는 열려 있어야 정상)
2. 단락 확인 시 U24 핀 4-5-6 / J26 핀 2-3-4 확대경 점검 후 리터치(리플로우)
3. 브리지 제거 후 `tb_hall_pinlevel`로 HS1 단독 트리거 시 bit0만 LOW로 떨어지는지 재확인
4. (추가) HS 라인 풀업 부재 → 센서가 오픈드레인이면 단락 제거 후 별도 풀업(4.7K~10K→VCC) 필요할 수 있음. `tb_hall_pinlevel` 안정성으로 판별.

> 펌웨어로는 단락된 노드를 분리할 수 없어 **코드 수정 없음**. → **회로 수정(리워크)으로 해결, HS1~HS5 정상 확인(2026-08-14).**

## 7. `tb_thermistor` — NTC 온도센서 3종 모니터 (이번 세션 신규 파일)

NTC 서미스터 3개(THERMISTER1/2/3, HCET-103F3950 10k B3950)를 **모니터링**하는 테스트베드.
액추에이터가 아닌 **센서 모니터**라 폴링 위치가 다르다(§구성 파일 표의 ADC 예외 참조):

- **`freertos.c`의 `StartDefaultTask`(~100 ms)에서 폴링** — ADC1은 재진입 불가라 단일 소유 태스크(defaultTask)에서만 읽어야 함.
  `MotorTask`가 아니므로 **`g_app_mode`와 무관하게 항상** 폴링되지만, `tb_therm_enable`이 0이면 아무 읽기도 안 함.
- 프로덕션 `thermistor.c` API(`Thermistor_ReadRaw/_ReadCelsius/_ReadCelsius_d10`)를 재사용 → 값이 `g_therm_c_d10[]`과 일치.
- **`tb_therm_enable`** = 1 → 매 poll에서 3채널 갱신 / 0 → 정지(마지막 스냅샷 동결, ADC 미접근).

### 하드웨어 배선

| 신호 | MCU 핀 | ADC 채널 | 커넥터 | 분압 고정저항(GND) / 직렬(ADC) |
|------|--------|----------|--------|----------------------|
| THERMISTER1 | PC0 | ADC1_IN10 | J19 | **R107(10k)** / R49(1k) |
| THERMISTER2 | PC1 | ADC1_IN11 | J22 | **R111(10k)** / R55(1k) |
| THERMISTER3 | PC2 | ADC1_IN12 | J26 | **R109(10k)** / R68(1k) |

> **고정저항 정정(2026-08-12)**: 분압 하단(GND) 고정저항은 **10kΩ(R107/R111/R109)** — 이전 BOM 판독의 1k가 아님(실측 확인).
> 직렬저항(ADC핀, 1k)은 ADC가 고임피던스라 분압비에 무관. 코드 `THERM_R_FIXED`도 10000으로 반영.

분압: `3.3V —[NTC(외부프로브)]—+—[10k 고정]—GND`, 노드에서 `[1k 직렬]—ADC핀`(+0.1uF). 비율식이라 Vref 상쇄:
`Rntc = Rfixed·(4095−raw)/raw` (Rfixed=**10k**). **온도 산출은 R-T 룩업테이블 + 보간**으로 함(아래 §온도 연산 참조).

**분압 특성(중요)**: 10k NTC + 10k 고정 → **25℃가 midscale(raw≈2048)**. 상온 분해능은 좋으나 **고온 분해능이 나쁨**:
150~200℃가 raw 46카운트뿐(200℃에서 ~0.7 count/℃, raw가 4095로 포화 접근). 건조/히터 고온 정밀도가 중요하면 하단 저항을
1k~2.2k로 낮추는 HW 변경을 검토(그 경우 `THERM_R_FIXED`도 함께 수정). 연산식(R-T 테이블) 자체는 무관.

### 온도 연산 방식 — R-T 룩업테이블 (2026-08-12 변경)

**기존**: 단일 Beta식(T0=25℃, R0=10k, B=3950). **변경**: Adafruit `103_3950` R-T 룩업테이블(−40~200℃, 10℃ 간격 25점, `thermistor.c`에 내장) + **(ln R, 1/T) 구간 선형보간**.

- **변경 이유**: 데이터시트 B는 **B(25/50)** — 25~50℃에서만 정확. 단일 Beta식은 이 표 대비 고온에서 계속 과대 판독:
  100℃ **+1.2℃**, 150℃ **+5.6℃**, 200℃ **+10.7℃** (동작 시나리오 건조/히터 온도대라 무시 못 함).
- **효과**: 보간이 (ln R, 1/T) 도메인에서 구간별 Beta와 같아 **−40~200℃ 전 구간 ≈±0.1℃**(행 지점 정확, 행 사이 최대 ~0.09℃). 25℃는 표에 없지만 20/30℃ 사이 보간으로 24.98℃.
- **에러 처리**: `raw==0`(오픈, Rntc→∞) / `raw≥4095`(쇼트, Rntc→0) → NAN. 표 범위 밖(−40℃ 미만/200℃ 초과)도 NAN. 끝점 −40/200℃는 유효.
- 출처: <https://cdn-shop.adafruit.com/datasheets/103_3950_lookuptable.pdf> (R25=10K, B25/50=3950 — HCET-103F3950과 동일 계열). 실측이 어긋나면 `THERM_RT[]` 값 또는 `THERM_R_FIXED` 보정 대상.

### 관찰 변수 (idx 0..2 = THERMISTER1..3)

| 변수 | 내용 |
|------|------|
| `tb_therm_enable` | 1=모니터링 / 0=정지(마지막 스냅샷 유지) |
| `tb_therm_raw[3]` | 12비트 ADC raw 카운트(직전 Tick, 노이즈 있음) |
| `tb_therm_c[3]` | **평활(EMA) 섭씨** float — 시스템이 쓰는 값(=`g_therm_c_d10`) |
| `tb_therm_c_d10[3]` | 평활 0.1℃ 단위 (253=25.3℃; 오류 시 −32768) |
| `tb_therm_c_inst[3]` | **순간(비평활) 섭씨** — 필터 효과 비교용 |
| `tb_therm_samples` | enable 중 poll마다 ++ (liveness) |
| `g_therm_filter_alpha` | EMA 새 샘플 가중치(0~1, 기본 0.3). **디버거로 실시간 튜닝** |

### 온도 평활(complementary/EMA 저역통과) — 2026-08-12 추가

순간 read는 특히 고온에서 분해능이 거칠어 몇 ℃씩 튄다(위 분압 특성). 이를 막기 위해 `thermistor.c`에 **1차 EMA**를 넣음:

```
filtered = filtered·(1−α) + sample·α        (α = g_therm_filter_alpha, 기본 0.3)
```

- **방식 선택**: 100개 배열 이동평균 대신 **EMA 채택**. 이유 — 상태 O(1)(채널당 float 1개 vs 300샘플), 링버퍼 관리 불필요, 같은 노이즈 억제에 지연 더 적음, α로 튜닝 자유.
- **소유권**: `Thermistor_Tick()`이 100 ms마다 **`StartDefaultTask`에서 1회만** 3채널 read+EMA 갱신(고정 dt·단일 writer). `g_therm_c_d10[]`·시나리오·테스트베드 모두 이 평활값을 공유. (테스트베드는 캐시된 getter만 읽어 ADC 중복 없음.)
- **특성(α=0.3, 100 ms)**: 시정수 ~0.23 s(스텝 0.8 s에 94%), 노이즈 std를 **원신호의 42%로 감소**. 열부하엔 지연 무시 수준이라 히터 제어에도 유리.
- **에러 처리**: 짧은 글리치(NAN)는 마지막 값 유지로 무시, 연속 `THERM_ERR_LIMIT`(5틱=0.5 s) 지속 시 평활값 NAN/에러. 첫 정상샘플로 seed(초기 램프 없음).
- **튜닝**: `g_therm_filter_alpha`를 디버거에서 조정 — **작게(0.1)=더 매끄럽고 느림 / 크게(0.5)=빠르고 덜 매끄럼**. `tb_therm_c_inst`(순간)와 `tb_therm_c`(평활)를 나란히 보며 결정. 값은 (0,1]로 클램프됨.

### 검증 진행 상태 / 이슈 (2026-08-12 현재)

| 항목 | 상태 |
|------|------|
| 3채널 read 동작 (ADC 변환·`tb_therm_samples` 증가·raw/온도 갱신) | ✅ **동작 확인** |
| 온도 연산식 (R-T 룩업+보간, −40~200℃ ±0.1℃) | ✅ **적용·자체검증 완료** |
| 온도 **값 실측 정확도** (기준 온도계 대비) | ⏳ **벤치 대조 미완료** |

- **동작 확인됨**: enable=1 시 3채널 raw/섭씨 값 갱신 + `tb_therm_samples` 증가, enable=0 시 정지.
- **연산식 검증됨**: 단일 Beta식 → R-T 룩업테이블 보간으로 교체(위 §온도 연산 방식). 알고리즘을 표 대비 자체 대조 →
  행 지점 정확, 행 사이 ≤~0.09℃, raw 왕복(25℃→24.96, 150℃→149.96) 오차 무시 수준. **고온 과대판독(최대 +10℃) 해소.**
- **남은 것 = 실측 대조**: 표시 온도(`tb_therm_c`/`tb_therm_c_d10`)를 **기준 온도계로 2점 이상 대조**해야 최종 신뢰.
  - 확인 항목: ① 상온(≈25℃)에서 `tb_therm_raw`≈**2048**(분압 midscale)인지, ② 알려진 2점(상온/온수 등) 오차,
    ③ 3채널 편차, ④ 극단(오픈/쇼트) 시 NAN. 어긋나면 `THERM_RT[]` 또는 `THERM_R_FIXED`(10k) 보정.

> 현 시점 결론: **연산식은 룩업테이블로 검증 완료(고온 오차 해결)** — 남은 것은 기준 온도계 실측 대조뿐.

### 실측 대조 절차 (기준 온도계 대비) — TODO, 벤치 진행 지침

목적: `tb_therm_c_d10`이 **실제 온도**와 일치하는지 확인하고, 어긋나면 원인(분압저항/프로브/연산)을 분리한다.
연산식은 이미 자체검증됐으므로, 실측에서 오차가 나오면 **연산이 아니라 하드웨어(분압 R, 프로브 개체차, 접촉열저항)** 를 먼저 의심한다.

**준비물**: 기준 온도계(K타입 열전대 또는 정밀 유리온도계, ±0.5℃급), 물/용기(상온·냉수·온수), 디버거(live watch), (선택) 정밀저항 2개(10kΩ·3.3kΩ 1%).

**기대값 기준표** (분압 **Rfixed=10kΩ**, 12-bit. `tb_therm_raw` 및 `tb_therm_c_d10` 목표):

| 온도 | Rntc(Ω) | 기대 `tb_therm_raw` | 기대 `tb_therm_c_d10` | Vadc(V) | 감도(counts/℃) |
|---:|---:|---:|---:|---:|---:|
| 0℃ (얼음물) | 31,770 | **980** | 0 | 0.790 | ~31.8 |
| 20℃ | 12,470 | **1,822** | 200 | 1.469 | ~44.2 |
| 25℃ (상온) | 10,000 | **2,048** (midscale) | 250 | 1.650 | ~45.2 |
| 40℃ | 5,327 | **2,672** | 400 | 2.153 | ~40.5 |
| 60℃ | 2,472 | **3,283** | 600 | 2.646 | ~27.0 |
| 80℃ | 1,243 | **3,642** | 800 | 2.935 | ~15.2 |
| 100℃ (끓는물) | 674 | **3,836** | 1000 | 3.092 | ~8.2 |
| 150℃ | 177 | **4,024** | 1500 | 3.243 | ~1.9 |
| 200℃ | 62 | **4,070** | 2000 | 3.280 | ~0.7 |

- **감도**는 그 온도 부근에서 raw 1℃당 변화량. 10k 고정저항이라 **저~중온(0~40℃)에서 분해능 최대(~40 counts/℃)**, **25℃가 midscale(raw 2048)**.
- **⚠ 고온 분해능 열화**: 100℃~ 부터 감도가 급락(100℃ ~8, 150℃ ~1.9, 200℃ ~0.7 count/℃)하고 raw가 4095로 포화 접근 → **150℃ 이상은 ADC 1카운트가 0.5~1.5℃**에 해당해 노이즈로 몇 ℃씩 흔들릴 수 있음. 건조 온도 정밀도가 중요하면 하단저항 인하(HW) 검토(위 §하드웨어 배선 주석).

**절차 A — 2점 실측 대조(권장, 실제 프로브·회로 통째 검증):**
1. 디버거에서 `tb_therm_enable = 1`, `tb_therm_samples`가 증가하는지 먼저 확인(모니터링 살아있음).
2. **상온점**: 프로브 3개를 기준 온도계와 같은 환경(공기 또는 물)에 함께 두고 5분 이상 안정화 → `tb_therm_raw`가 위 표의 상온값(**≈2048@25℃, midscale**) 근처인지, `tb_therm_c_d10/10`이 기준계와 ±1℃ 이내인지.
3. **온수점**: 40~80℃ 온수에 프로브+기준계를 함께 담그고(금속 M6 헤드가 완전 침수) 안정화 → 표의 해당 raw/온도와 대조. (끓는물 100℃도 가능하나 프로브 정격·화상 주의)
4. 두 점 모두 ±1~2℃ 이내면 **PASS**(±는 R25±1%·B±1% 공차 하한). 초과 시 아래 원인분리.
5. **3채널 편차**: 같은 환경에서 THERMISTER1/2/3의 `tb_therm_c_d10` 최대편차 기록(개체차·배선 확인). 통상 ±1℃ 이내 기대.

**절차 B — 저항 치환(회로/연산만 분리 검증, 프로브 배제):**
- 프로브 커넥터(J19/J22/J26) 자리에 **정밀 10kΩ** 삽입 → `tb_therm_raw`≈**2048**(midscale) / `tb_therm_c_d10`≈250(25.0℃) 나오면 **분압·ADC·연산 정상**.
- 이어 **정밀 3.3kΩ** 삽입(≈Rntc@52℃) → raw≈**3079** / d10≈522(≈52℃) 근처. 두 치환이 맞으면 오차 원인은 **프로브 쪽**으로 확정.
  (참고: 10kΩ 치환이 정확하면 분압 하단저항이 실제 10k임도 함께 확인됨 — raw가 2048 아닌 ~372면 저항이 1k라는 뜻.)

**극단(에러 처리) 확인:**
- 프로브 탈거(오픈) → `tb_therm_raw`=0 부근, `tb_therm_c[i]`=NAN, `tb_therm_c_d10[i]`=**−32768**(THERMISTOR_ERR_D10).
- 프로브핀 GND 단락 → 동일하게 오픈 취급(raw≈0). 3.3V 단락 → raw≈4095 → NAN/−32768.

**판정 후 보정 가이드:**
| 증상 | 추정 원인 | 조치 |
|------|-----------|------|
| 절차 B(저항치환)는 정확한데 절차 A만 오차 | 프로브 개체차/접촉열저항/침수 부족 | 프로브 완전 침수·안정화 시간 확대, 개체 R25 확인 |
| 전 온도 일정 오프셋(예: 항상 +2℃) | 분압 `Rfixed` 실제값≠10kΩ, ADC ref | `THERM_R_FIXED` 실측저항으로 보정 |
| 상온에서 raw가 2048 아님(예: ~372) | 하단저항이 10k 아닌 1k | `THERM_R_FIXED`를 실제값으로 수정 |
| 고온(≥150℃)만 값이 크게 튐/포화 | 분압 midscale이 25℃라 고온 분해능 부족(HW 특성) | 하단저항 인하(1k~2.2k) HW 변경 + `THERM_R_FIXED` 동반 수정 |
| 고온으로 갈수록 커지는 오차 | 프로브 B값이 3950과 상이 | `THERM_RT[]`를 실측 프로브 R-T로 교체 |
| 특정 1채널만 오차 | 해당 채널 분압저항/배선 | 해당 J커넥터·R값 점검 |

> 기록 위치: 실측 완료 시 위 "검증 진행 상태" 표의 ⏳ 항목을 결과(PASS/보정내역)로 갱신할 것.

## 8. `tb_water` — 급수(WATER-ON) + 수위센서(WATER-SEN1/SEN2) (이번 세션 신규 파일)

깨끗한 물 급수 경로를 검사하는 테스트베드. **출력 1개 + 입력(EXTI) 2개**를 한 묶음으로 다룬다:

- `o_WATER_ON` (PE2) : 급수 밸브/펌프 enable 출력(active-high)
- `exti6_WATER_SEN1` (PF6) / `exti7_WATER_SEN2` (PF7) : 수위센서 2개(EXTI, falling edge)

**동작 흐름**: `tb_water_enable`을 그대로 WATER-ON에 미러링한다.
```
if (tb_water_enable) { WATER-ON = HIGH; SEN1/SEN2 인식(레벨/present/엣지); }
else                 { WATER-ON = LOW  (급수 OFF, 안전); }
```

- 홀/서미스터와 마찬가지로 **센서가 EXTI 라인이라 `StartDefaultTask`(~100 ms)에서 폴링**한다(EXTI 플래그를 폴링/클리어하는 태스크). `MotorTask`가 아니므로 **`g_app_mode`와 무관하게 항상** 폴링되지만, `tb_water_enable`이 0이면 WATER-ON은 OFF이고 아무 감지도 하지 않는다.
- **극성**: 수위센서는 보드상 active-low(물이 프로브를 이으면 핀이 LOW) — 동작 시나리오의 `DJ_WATER_ACTIVE_LOW=1`과 동일. `present`는 이 반전을 적용해 **1=물 도달**로 읽힌다(`TB_WATER_ACTIVE_LOW`로 재정의 가능).
- **엣지 카운트**: PF6/PF7은 `GPIO_MODE_IT_FALLING` → 공유 `gpio_ctrl` EXTI 플래그로 falling edge를 카운트. enable 엣지에서 stale 플래그를 1회 클리어해, 시작 시점에 이미 눌린 레벨을 가짜 엣지로 세지 않는다.

### 관찰/조작 변수

| 변수 | 내용 |
|------|------|
| `tb_water_enable` | 1=급수 ON + 감지 / 0=급수 OFF, 정지(마지막 스냅샷 유지) |
| `tb_water_on_state` | WATER-ON 출력 래치 read-back(0/1) |
| `tb_water_sen1_level` / `tb_water_sen2_level` | SEN1/SEN2 raw 핀 레벨(0/1) |
| `tb_water_sen1_present` / `tb_water_sen2_present` | 물 도달 여부(극성 적용, 1=도달) |
| `tb_water_present` | 둘 중 하나라도 도달 시 1 |
| `tb_water_sen1_events` / `tb_water_sen2_events` | SEN1/SEN2 falling-edge 카운트 |
| `tb_water_samples` | enable 중 poll마다 ++ (liveness) |

> **주의(시나리오 충돌)**: 이 tb는 항상 폴링되지만 WATER-ON을 **구동**하고 WATER-SEN 플래그를 **소비(클리어)** 한다.
> 모음/동작 시나리오는 동일 급수 라인·SEN 플래그를 직접 소유하므로, **시나리오 실행 중에는 `tb_water_enable=0`을 유지**할 것.
> (현재 `o_WATER_ON`을 구동하는 프로덕션 코드는 없어 테스트벤치 모드에서 단독 소유가 가능.)

### 검증 진행 상태 / 이슈 (2026-08-12 현재)

| 항목 | 상태 |
|------|------|
| 빌드 (타깃 `arm-none-eabi-gcc`, cortex-m3, `-Wall -Wextra` 구문검사) | ✅ `tb_water.c` / `freertos.c` 통과 |
| 실제 HW 동작 (WATER-ON 구동, SEN1/SEN2 레벨·엣지 인식) | ✅ **HW 검증완료** (2026-08-12, 개별기능검증R1 항목4) |

- 코드/통합은 완료: `freertos.c`의 `StartDefaultTask`에 include + `TB_Water_Init()` / `TB_Water_Poll()` 반영, `Debug/.../subdir.mk` 빌드 목록 추가.
- 확인 필요 항목: ① enable=1 시 WATER-ON 실제 전압/밸브 동작, ② 프로브에 물 접촉 시 `sen*_present`가 1로, `sen*_events` 증가, ③ 센서 폴라리티(active-low 가정)가 실제 배선과 일치하는지, ④ SEN1/SEN2 개별 응답 분리.

---

## 9. `tb_heat` — 히터(HT-POWER) + 온도센서 연동 (이번 세션 신규 파일)

> ⚠️ **220V 실히터를 스위칭한다. 모든 기본값을 "무장"으로 간주하고, 프로브를 붙이고 감시하며 시험한다.** 실제 과열 안전 주체는 HW 바이메탈(60/80℃ EXTI + 210℃)이고, FW는 감시·차단만 한다.

- **대상**: HT-POWER = PA12 = `o_HT_POWER` = `GPIO_OUT_HT_POWER`. MOC3063 제로크로스 옵토트라이악(Q31/Q32) → AC 히터.
- **온도센서 연동(핵심)**: 히터는 눈감고 못 켠다. `tb_heat`는 NTC 프로브 1개(`tb_heat_ch`, 기본 CH0 = `DJ_TEMP_CH` = 처리통)의 **평활(EMA) 온도**(`Thermistor_GetCelsius_d10`)를 읽어 **동작 시나리오와 동일한 190/195℃ 히스테리시스**로 히터를 제어한다 → 센서가 루프 안에 있다. 자체 ADC는 하지 않고 `Thermistor_Tick()`가 캐시한 값을 재사용한다.
- **공유 판정 함수**: HYST 모드의 ON/OFF/과열 판정은 `interface/heater_hyst.h`의 순수함수 `Heater_HystDecide()` / `Heater_IsOverTemp()`가 담당하며, **동작 시나리오 `dj_heater_tick()`도 같은 함수를 사용**한다 → 임계/거동이 이원화되지 않는다. MANUAL 모드와 idle-guard 워치독은 **테스트벤치 전용** 확장이고, 동작 시나리오에는 둘 다 없다.
- **배치(중요)**: 온도(ADC1)는 `StartDefaultTask`만 읽는 규칙이라, `tb_heat`도 같은 태스크에서 `Thermistor_Tick()` **직후** 폴링한다. `TB_Heat_Poll(AppMode_IsBenchIdle(g_app_mode))`로 호출 → **대기/정지 모드일 때만** HT-POWER를 구동하고, 동작 시나리오 모드에서는 핀에 손대지 않는다(시나리오의 `dj_heater_tick`가 단독 소유). `tb_heat_enable=0`이면 항상 강제 OFF.

### 타임드 로깅 세션 (5분 / 30초 샘플)

`tb_heat_enable`이 **0→1이 되는 순간 5분짜리 세션이 시작**된다. 세션 동안 히터는 평소대로 동작하고, **30초마다 상태를 `tb_heat_log[]` 배열에 스냅샷**해 런타임(디버거 배열 watch)에서 열 응답을 그대로 읽을 수 있다. 세션은 **둘 중 하나로 종료**된다:

- **5분 경과** → 자동 종료: `tb_heat_enable`을 코드가 0으로 내리고 히터 강제 OFF.
- **`tb_heat_enable`을 0으로** → 조작자 중지: 세션 닫고 히터 OFF.

- 샘플 지점: `t = 0, 30, 60, … 300초` (총 **11 슬롯**). 각 슬롯 = `{ t_ms, temp_d10, out, fault }`.
- 재시작: 다시 `enable` 0→1 하면 로그를 지우고 새 세션 시작. 종료 후에도 마지막 로그는 다음 재시작 전까지 **보존**되어 확인 가능.
- 상수: `TB_HEAT_LOG_PERIOD_MS=30000`, `TB_HEAT_SESSION_MS=300000`, `TB_HEAT_LOG_COUNT=11` (`tb_heat.h`).

| 변수 | 의미 |
|------|------|
| `tb_heat_log[0..count-1]` | 30초 간격 스냅샷 배열(`t_ms`/`temp_d10`/`out`/`fault`) |
| `tb_heat_log_count` | 채워진 슬롯 수(0..11) |
| `tb_heat_session_active` | 1=세션 진행중, 0=종료/유휴 |
| `tb_heat_session_ms` | 현재 세션 경과시간[ms] (0→300000) |

### 모드

| `tb_heat_mode` | 동작 |
|----------------|------|
| `TB_HEAT_MODE_HYST` (1, 기본) | `on_d10 ≤ t ≤ off_d10` 밴드 히스테리시스, 밴드 안에서는 직전 상태 유지. **유효 온도 필요**(프로브 ERR 시 OFF) |
| `TB_HEAT_MODE_MANUAL` (0) | `tb_heat_manual_on`을 그대로 핀에 반영 (전기 결선 확인용). **센서 게이트 없음**(프로브 없이도 동작), 단 유효 온도가 과열이면 차단 |

### 안전

| 보호 | 임계 / 기본 | 종류 | 적용 |
|------|-------------|------|------|
| 과열 차단 | `tb_heat_safety_d10` = 2100 (210.0℃, = `DJ_TEMP_SAFETY`) | 래치 | 두 모드 (유효 온도일 때) |
| idle-guard 워치독 | `tb_heat_max_on_ms` = **0(기본 해제)**, 원하면 예 600000(10분) | 래치 | 테스트벤치 전용 |
| 센서 결함(open/short → ERR) | HYST에서만 OFF | 라이브(복구 시 자동 해제) | HYST 모드 |

- 래치 해제: `tb_heat_enable`을 0→1로 토글하거나 `tb_heat_clear_fault=1`(자동 0 복귀).
- 실제 과열 안전 주체는 HW 바이메탈(60/80℃ + 210℃)이고, 위 과열 차단은 FW 백스톱이다.

> **⚠️ 버그 수정 이력(리팩터링 시):** (1) 기존 무조건 센서 게이트가 MANUAL까지 차단 → HYST에만 적용하도록 수정(프로브 없이 전기시험 가능). (2) 워치독 기본 60초가 실제 운전을 끊음 → 기본 0(해제)으로 변경. 진짜 안전(과열·센서·HW 바이메탈)은 유지.

### 관찰/조작 변수

| 변수 | 의미 |
|------|------|
| `tb_heat_enable` | 0=대기/OFF, 1=테스트벤치 실행 |
| `tb_heat_mode` / `tb_heat_ch` | 모드 / 프로브 인덱스(0..2) |
| `tb_heat_manual_on` | MANUAL 모드 핀 레벨 |
| `tb_heat_on_d10` / `_off_d10` / `_safety_d10` | 히스테리시스 밴드 + 안전(0.1℃), 기본 1900/1950/2100 |
| `tb_heat_max_on_ms` | idle-guard 워치독(ms), **0=해제(기본)** |
| `tb_heat_out` | HT-POWER에 실제 명령한 레벨 0/1 |
| `tb_heat_temp_d10` | 이번 사이클 사용 온도(ERR=-32768) |
| `tb_heat_fault` | 비트: 1=센서 2=과열 4=워치독 |
| `tb_heat_on_ms` / `tb_heat_samples` | 현재 연속 ON시간 / 라이브니스 |

### 벤치 온도로 루프 시험 (190℃ 오븐 없이)

`on/off`를 예: `300/350`(30/35℃)으로 낮추고 프로브를 손·열풍기로 데우면, `tb_heat_out`이 `on` 이하에서 1, `off` 이상에서 0으로 토글하는지 육안 확인 가능. 과열(210℃) 백스톱은 유지된다. 확인 후 값을 실제 1900/1950으로 복원.

### 동작 시나리오 연동 (공유 함수로 통합 완료)

- 히스테리시스/과열 판정을 `interface/heater_hyst.h`의 **순수함수로 추출**하여 `dongjak.c dj_heater_tick()`와 `tb_heat`가 **공유**한다 → 임계/거동 이원화 제거. `dj_heater_tick`은 리팩터링 후에도 기존 거동(ERR=핀 유지, 210℃ 안전, 190/195 밴드)을 그대로 보존한다.
- 시나리오 히터 구동 자체는 그대로: `dj_heater_tick()`가 `g_therm_c_d10[DJ_TEMP_CH]`를 읽어 HT-POWER 구동, `DJ_COOLDOWN`/`dj_all_off`에서 OFF. 별도 히터 device 없음.
- **워치독(60s)·MANUAL 모드는 시나리오에 미적용** — 공유 함수에 넣지 않고 `tb_heat`에만 둠. 실제 운전은 이 두 기능이 필요 없고, 오히려 60초 워치독은 운전을 끊으므로 시나리오에서 배제.
- 소유권 충돌 방지: `tb_heat`는 `defaultTask`, 시나리오 히터는 `MotorTask`. `TB_Heat_Poll`의 `testbench_active` 게이트로 동시 구동을 배제한다(모드 게이트가 이미 차단하므로 시나리오 중 `tb_heat_enable`은 무해하나 0 유지 권장).

### 검증 진행 상태 / 이슈

| 항목 | 상태 |
|------|------|
| 빌드 (타깃 `arm-none-eabi-gcc`, cortex-m3, `-Wall -Wextra` 구문검사) | ✅ `heater_hyst.c` / `tb_heat.c` / `dongjak.c` / `freertos.c` 통과 |
| 실제 HW 동작 (HT-POWER 트라이악 구동, 온도 추종 토글) | ⏳ 미검증 (보드 실측 대기) |

- 코드/통합 완료: `heater_hyst.c/.h` 신규(공유 판정), `freertos.c` `StartDefaultTask`에 include + `TB_Heat_Init()` / `TB_Heat_Poll()` 반영, `dongjak.c` 공유 함수 적용, `Debug/.../subdir.mk`(Interface·Testbench) 빌드 목록 추가.
- 확인 필요: ① MANUAL로 (프로브 없이도) 트라이악/릴레이 실제 스위칭, ② 밴드 낮춰 프로브 가열 시 `tb_heat_out` 토글, ③ 유효 온도 210℃ 도달 시 과열 래치 후 강제 OFF, ④ HYST에서 센서 탈거 시 즉시 OFF.

---

## 10. `tb_doorhall` — 도어 리밋홀 WHALL/THALL 모니터 (이번 세션 신규 파일)

배수문(WDoor/U5)·배출문(TDoor/U7)의 **열림/닫힘 리밋 홀센서 4입력**을 모니터링하는
테스트베드. 이 리밋 신호가 모음/동작 시나리오의 도어 개폐 전이(다음 단계 진행/ERROR
게이팅)를 좌우하는데 기존엔 전용 벤치가 없었다. 액추에이터가 아닌 **센서 모니터**라
`tb_water`와 동일하게 폴링 위치가 다르다:

- **`StartDefaultTask`(~100 ms)에서 폴링** — EXTI 플래그를 폴링/클리어하는 소유 태스크.
  `MotorTask`가 아니므로 **`g_app_mode`와 무관하게 항상** 모니터되지만 `tb_doorhall_enable=0`이면 정지.
- 프로덕션 드라이버(`WDoor_/TDoor_AtOpen/AtClose`)를 그대로 재사용 → at-limit 판정이
  시나리오와 100 % 일치. 이 tb는 raw 레벨과 엣지 카운트를 덧붙일 뿐.
- **입력 전용** — 아무 것도 구동하지 않으므로 시나리오와 충돌하지 않는다(항상 켜둬도 안전).

### 핀 맵 (EXTI, GPIO_MODE_IT_FALLING · 극성 `DOOR_LIMIT_ACTIVE_HIGH`=0 active-low)

| 신호 | MCU 핀 | EXTI | 드라이버 판정 | at-limit 변수 | 레벨 | 엣지 카운터 |
|------|--------|------|--------------|--------------|------|-------------|
| WHALL-CLOSE | PF3 | exti3 | `WDoor_AtClose()` | `tb_wdoor_at_close` | `tb_whall_close_level` | `tb_whall_close_events` |
| WHALL-OPEN  | PF4 | exti4 | `WDoor_AtOpen()`  | `tb_wdoor_at_open`  | `tb_whall_open_level`  | `tb_whall_open_events`  |
| THALL-CLOSE | PF2 | exti2 | `TDoor_AtClose()` | `tb_tdoor_at_close` | `tb_thall_close_level` | `tb_thall_close_events` |
| THALL-OPEN  | PF5 | exti5 | `TDoor_AtOpen()`  | `tb_tdoor_at_open`  | `tb_thall_open_level`  | `tb_thall_open_events`  |

기타: `tb_doorhall_enable`(1=모니터/0=정지), `tb_doorhall_samples`(enable 중 poll마다 ++).
엣지 카운트는 main.c 중앙 EXTI 라우터(`gpio_ctrl_exti_dispatch`)가 래치한 falling edge를 센다.
enable 엣지에서 stale 플래그를 1회 클리어해 시작 시점 레벨을 가짜 엣지로 세지 않는다.

### 동작 시나리오 연동 (dongjak)

- 시나리오는 이미 `WDoor_/TDoor_AtOpen/AtClose`로 이 리밋을 **게이팅**한다(닫힘/열림 도달 시
  다음 단계, 미도달 시 `DJ_DOOR_TIMEOUT_MS`(15s) 후 `DJ_ERROR`).
- **`DJ_DOOR_LIMIT_OPTIONAL`(dongjak.h, 기본 0)** 추가 — 모음의 `MOEUM_DOOR_LIMIT_OPTIONAL`과
  동일 기조. `1`로 두면 리밋 미도달이라도 `DJ_DOOR_BENCH_MS`(4s)만 구동 후 진행(센서 무시).
  **리밋홀 HW 검증 전 문 동작 육안 확인용 임시 모드.** 검증 완료되면 0으로 되돌린다.

### 검증 진행 상태 (신규)

| 항목 | 상태 |
|------|------|
| 빌드 (arm-none-eabi-gcc, cortex-m3, `-Wall -Wextra` 구문검사) | ✅ `tb_doorhall.c`/`freertos.c`/`dongjak.c` 통과 |
| 실제 HW 동작 (문 여닫을 때 at-limit/level/events 인식) | ⏳ 미검증 (보드 실측 대기) |

- 코드/통합 완료: `freertos.c` `StartDefaultTask`에 include + `TB_DoorHall_Init()`/`TB_DoorHall_Poll()`, `Debug/.../subdir.mk` 빌드 목록 추가.
- 확인 필요: ① 문을 손으로(또는 `tb_wdoor_*`/`tb_tdoor_*`로) 여닫을 때 해당 at-limit이 1로, `_events` 증가, ② 극성(`DOOR_LIMIT_ACTIVE_HIGH`) 실제 배선 일치, ③ 열림/닫힘 리밋 분리, ④ 4개 채널 개별 응답.

---

## 사용 예시 (디버거)

```
// 급수 밸브 열고 배기 팬 가동
tb_valve_dry_en   = 1
tb_fan_exhaust_en = 1

// 교반 모터 정방향 시작 후 속도 2단계 올리기
tb_stir_rev     = 0
tb_stir_en      = 1
tb_stir_spd_req = 1     // (자동 0으로 클리어 후 다시 1 쓰면 또 1단계)

// 배수문 개방
tb_wdoor_reverse = 1    // Open (배수문은 1이 열림)
tb_wdoor_enable  = 1     // duty 80%(2s) -> 65% 자동 적용, tb_wdoor_duty에서 관찰
                         // WHALL-OPEN 도달 시 자동으로 0, tb_wdoor_limit_hit=1 (미인식 시 6s 상한)

// 배수문 닫힘 (80% 고정, 4.2초 후 자동 정지)
tb_wdoor_reverse = 0    // Close (배수문은 0이 닫힘)
tb_wdoor_enable  = 1     // 4.2s 경과 시 자동으로 0, tb_wdoor_time_hit=1

// 투입문 개방 (80% 고정, 홀 인식 후 1초 더 돌고 정지)
tb_tdoor_reverse = 0    // Open
tb_tdoor_enable  = 1     // THALL-OPEN 인식 -> tb_tdoor_overrun=1 -> 1s 후
                         // 자동으로 0, tb_tdoor_limit_hit=1
                         // (홀 미인식 시 14.3s 상한, tb_tdoor_time_hit=1)

// 급수 ON + 수위센서 감지 (tb_water_sen1/2_present, _events 관찰)
tb_water_enable  = 1    // WATER-ON HIGH, SEN1/SEN2 인식 시작
// ...확인 후...
tb_water_enable  = 0    // 급수 OFF

// [히터] ⚠️ 220V. AppMode_IsBenchIdle(g_app_mode) (대기/정지) 상태에서만 동작.
// (A) 전기 결선만 확인 (MANUAL): 프로브 없이 트라이악 스위칭
tb_heat_mode      = 0   // MANUAL
tb_heat_manual_on = 1   // HT-POWER HIGH (안전차단은 계속 적용)
tb_heat_enable    = 1
// ...tb_heat_out / tb_heat_on_ms 확인 후...
tb_heat_enable    = 0

// (B) 온도센서 연동 히스테리시스 루프 (벤치 온도로 시험)
tb_heat_ch      = 0     // CH0 = 처리통 프로브
tb_heat_on_d10  = 300   // 30.0℃ 이하 ON  (실사용값 1900)
tb_heat_off_d10 = 350   // 35.0℃ 이상 OFF (실사용값 1950)
tb_heat_mode    = 1     // HYST
tb_heat_enable  = 1     // 프로브 가열/냉각하며 tb_heat_out 토글 관찰
// tb_heat_fault (1=센서 2=과열 4=워치독), 래치 시 tb_heat_clear_fault=1 로 해제
```

## 이번 세션 진행 결과 요약

1. `tb_tca9554`: 분쇄/교반 BLDC에 버튼 없이 구동 가능한 디버거 enable 변수 추가
   (`tb_grind_en/rev/spd_req`, `tb_stir_en/rev/spd_req`, edge-triggered, `apply_enable()`).
2. `tb_gpioout` **신규 생성**: 배수/급수 밸브 + 팬 3종(총 5개) on/off 개별 검사용.
   `freertos.c`에 include + `TB_GpioOut_Init()` / `TB_GpioOut_Poll()` 통합 완료.
3. `tb_hallsensor` **신규 생성**: U24 홀센서 8입력(트리거군 P0~P4 + 교반원점/수거통/리프트하단)
   `tb_hall_enable` 게이트 모니터. `freertos.c`의 `StartDefaultTask`에 include + `TB_HallSensor_Init()` / `TB_HallSensor_Poll()` 통합 완료.
   - 트리거 라벨 확정: HS1 강음 / HS2 동작 / HS3 정지 / HS4 배수 / HS5 모음 (2026-08-25 재정의, 이전 HS2 모음 / HS5 동작).
   - **디버깅 결과**: 초기 HS1/HS2/HS3(P0/P1/P2) 전기적 커플링(저임피던스 단락) → **회로 수정으로 해결**.
     **U24 홀센서 8채널(HS1~HS8) 전부 정상 확인(2026-08-15)**. 펌웨어 정상(디코드/읽기 경로 대칭) 확인. 상세는 위 §6 참조.
4. `tb_thermistor` **신규 생성**: NTC 온도센서 3종(THERMISTER1/2/3) `tb_therm_enable` 게이트 모니터.
   `freertos.c`의 `StartDefaultTask`에 include + `TB_Thermistor_Init()` / `TB_Thermistor_Poll()` 통합 완료(ADC1 단일 소유).
   - **동작 확인 O**: enable 시 3채널 raw·섭씨 갱신 및 `tb_therm_samples` 증가 확인됨.
   - **온도 연산식 교체(단일 Beta → R-T 룩업+보간)**: 데이터시트 B(25/50)의 고온 과대판독(100℃ +1.2 / 150℃ +5.6 / 200℃ +10.7℃)을
     Adafruit 103_3950 R-T 테이블 보간으로 해소(−40~200℃ ≈±0.1℃, 자체검증 완료). `thermistor.c`에 `THERM_RT[]` 내장.
   - **분압저항 정정(2026-08-12)**: 하단 고정저항이 1k가 아니라 **10k(R107/R111/R109)** 로 확인 → `THERM_R_FIXED=10000`. 25℃가 midscale(raw≈2048), 단 고온 분해능 열화(HW 특성).
   - **온도 평활(EMA) 추가**: `thermistor.c`에 1차 EMA(`filt=filt·(1−α)+x·α`, α=`g_therm_filter_alpha` 기본 0.3) 도입. `Thermistor_Tick()`을 `StartDefaultTask`에서 100 ms마다 1회 호출(단일 owner) →
     `g_therm_c_d10[]`·시나리오·테스트베드가 평활값 공유. 노이즈 std 42%로 감소, 스텝 0.8 s에 94%. 테스트베드에 순간값 `tb_therm_c_inst` 비교 필드 추가. 구문검사(`-Wall -Wextra -Wconversion`) 통과.
   - **남은 것**: 기준 온도계 실측 2점 대조(상온 raw≈2048 확인 등). 상세는 위 §7 참조.
5. `tb_water` **신규 생성**: 급수 `WATER-ON`(PE2) 출력 + 수위센서 `WATER-SEN1/SEN2`(PF6/PF7 EXTI) 인식.
   `tb_water_enable`을 WATER-ON에 미러링하고 SEN1/SEN2의 레벨·present(active-low)·falling-edge 카운트를 노출.
   `freertos.c`의 `StartDefaultTask`에 include + `TB_Water_Init()` / `TB_Water_Poll()` 통합 완료, `Debug/.../subdir.mk` 빌드 목록 추가.
   - **빌드 구문검사 통과 / HW 동작 미검증**(보드 실측 대기). 시나리오 실행 중에는 급수 라인 충돌 방지를 위해 `tb_water_enable=0` 유지. 상세는 위 §8 참조.
6. `tb_heat` **신규 생성**: 히터 `HT-POWER`(PA12) 제어 + **온도센서 연동**. NTC 평활온도(`Thermistor_GetCelsius_d10`)를 읽어 동작 시나리오와 동일한 190/195℃ 히스테리시스로 히터 제어(센서 in-loop), MANUAL 강제모드 별도.
   - 안전 3중: 210℃ 과열(래치)·최대 연속 ON 60s 워치독(래치)·센서 ERR 즉시 OFF(라이브). 기본 disable/off.
   - **소유권 게이트**: 온도(ADC1) 규칙상 `defaultTask`에서 `Thermistor_Tick()` 직후 폴링. `TB_Heat_Poll(AppMode_IsBenchIdle(g_app_mode))`로 대기/정지 모드일 때만 HT-POWER 구동 → 동작 시나리오(`MotorTask`의 `dj_heater_tick`)와 핀 충돌 배제.
   - `freertos.c`에 include + `TB_Heat_Init()` / `TB_Heat_Poll()` 통합, `Debug/.../subdir.mk` 빌드 목록 추가. 구문검사(`arm-none-eabi-gcc -Wall`) 통과 / HW 미검증. **동작 시나리오는 이미 히터-온도 연동 완료** 상태이며 임계 기본값을 `DJ_TEMP_*`와 일치시킴. 상세는 위 §9 참조.
7. `tb_doorhall` **신규 생성**: 도어 리밋홀 4입력(WHALL/THALL open/close, PF3/PF4/PF2/PF5 EXTI) `tb_doorhall_enable` 게이트 모니터. 프로덕션 `WDoor_/TDoor_AtOpen/AtClose` 재사용으로 at-limit이 시나리오와 일치, raw 레벨·falling-edge 카운트 노출.
   - `freertos.c` `StartDefaultTask`에 include + `TB_DoorHall_Init()` / `TB_DoorHall_Poll()` 통합, `Debug/.../subdir.mk` 빌드 목록 추가. 입력 전용이라 시나리오와 무충돌(항상 켜둬도 안전). 구문검사(`-Wall -Wextra`) 통과 / HW 미검증. 상세는 위 §10 참조.
8. **동작 시나리오(dongjak) 연동 보강**:
   - **급수 WATER-ON(PE2) 연동**: `DJ_FILL_USE_WATER_ON`(기본 1) + `dj_fill_on/off()` 헬퍼로 헹굼 FILL 시 VALVE_DRY_IN(PB13)과 WATER_ON(PE2)을 함께 ON. 모음(`MOEUM_FILL_USE_WATER_ON`)과 동일 — 수위센서 통수 조건 확보(미연동 시 FILL 감지 실패→`DJ_ERROR` 위험 제거).
   - **배수밸브 VALVE_DRAIN_CLN(PB14) 연동**: `DJ_DRAIN_VALVE_USE`(기본 1) + `dj_wdoor_open/close()` 래퍼로 배수문 개방=배수밸브 ON, 닫힘=OFF 불변식 확립(실제 배수는 문+밸브 동반 개방). PB14 벤치는 `tb_valve_drain_en`으로 이미 HW 검증완료(항목5).
   - **도어 리밋 벤치옵션**: `DJ_DOOR_LIMIT_OPTIONAL`(기본 0) + `DJ_DOOR_BENCH_MS`(4s) + `dj_door_done()` 게이트 판정. 리밋홀 HW 검증 전 문 동작 육안 확인용(모음과 동일 기조). 6개 도어 게이트 전이에 적용.
   - 구문검사: `dongjak.c`가 기본/`DJ_DOOR_LIMIT_OPTIONAL=1`/플래그 off(0) 조합 모두 `-Wall -Wextra` 통과.
