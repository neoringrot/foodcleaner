# Testbench (개별 부품 검사용 테스트베드)

각 액추에이터를 **시나리오(모음/동작)와 무관하게 개별적으로** 구동/검사하기 위한 테스트베드 모음.
모든 테스트베드는 `volatile` 전역 변수를 노출하며, **디버거(live watch / expression)에서 값을 바꾸면 즉시 반영**된다.

## 공통 동작 모델

- 모든 `TB_*_Poll()`은 `freertos.c`의 `MotorTask_RunTestbench()`에서만 호출된다
  → **테스트벤치 모드(`g_app_mode` 기본)에서만** 동작하고, 모음/동작 시나리오 모드에서는 자동으로 무시된다
  (해당 모드에서는 시나리오 코드가 동일 액추에이터를 직접 소유).
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

> **ADC 계열 예외(중요)**: `tb_thermistor`는 **센서 모니터** 테스트베드로, 위 "공통 동작 모델"과 달리
> `MotorTask_RunTestbench()`가 아니라 **`StartDefaultTask()`(~100 ms)** 에서 폴링한다.
> 이유: ADC1 읽기는 재진입 불가라서 단일 태스크(defaultTask)만 소유해야 한다(`adc_ctrl.h` 주석).
> 따라서 `g_app_mode`(테스트벤치/시나리오)와 **무관하게** 항상 폴링되지만, `tb_therm_enable`이 0이면
> 아무 읽기도 하지 않으므로 사실상 정지 상태다.

---

## 개별기능검증R1.xls 대조 현황 (2026-08-12)

`doc/개별기능검증R1.xlsx`의 항목별 HW 검증 진행 상태. (✅ HW검증완료 · ⏳ 미검증 · ⚠️ 부분/이슈 · ❌ 없음)

| # | xls 항목 | 테스트벤치 / 변수 | 핀·커넥터 | 상태 |
|---|----------|-------------------|-----------|------|
| 1 | 수중기배출팬(증기) | `tb_fan_vapor_en` | PB15 | ✅ HW 검증완료 |
| 2 | 환풍팬(배기) | `tb_fan_exhaust_en` | PG2 | ✅ HW 검증완료 |
| 3 | 환풍팬(BLDC 냉각) | `tb_bldc_fan_en` | PG1 | ✅ HW 검증완료 |
| 4 | 수위센서 | `tb_water` (SEN1/SEN2) | PF6/PF7 | ✅ HW 검증완료 |
| 5 | 배수구세척솔벨브 | `tb_valve_drain_en` | PB14 | ✅ HW 검증완료 |
| 6 | 건조통 급수 솔벨브 | `tb_valve_dry_en` | PB13 | ✅ HW 검증완료 |
| 7 | STEP 공기배출 DOOR | `tb_step1_*` | **J31** (24BYJ48-895) | ✅ HW 검증완료 |
| 8 | STEP 공기흡입 DOOR | `tb_step2_*` | **J33** (35BYJ46-1014) | ⏳ 미검증 (커넥터 re-pin 필요, RED→pin1) |
| 9 | DC 배수부 모터(DOOR) | `tb_wdoor_*` | U5 (DRV8871) | ✅ HW 검증완료 |
| 10 | DC 배출부 모터(DOOR) | `tb_tdoor_*` | U7 (DRV8871) | ⏳ 미검증 |
| 11 | 상단 동작 스위치(=HS1~5) | `tb_hall_*` (P0~P4) | U24/J26 | ⚠️ HS4/HS5 정상 · HS1/HS2/HS3 단락으로 사용 불가 (§6) |
| 12 | 교반원점 홀센서 | `tb_hall_stir_home` (HS6) | U24 P5/J28 | ⏳ 미검증 |
| 13 | 수거통 홀센서 | `tb_hall_bin` (HS7) | U24 P6/J29 | ⏳ 미검증 |
| 14 | 리프트 하단 홀센서 | `tb_hall_lift_bottom` (HS8) | U24 P7/J30 | ⏳ 미검증 |
| 15 | 히터 | (없음) | — | ❌ 테스트벤치 없음 |

**xls 미포함(테스트벤치에만 있는) 액추에이터:**

| 항목 | 테스트벤치 / 변수 | 상태 |
|------|-------------------|------|
| 분쇄 BLDC (M1) | `tb_grind_en` 외 (`tb_tca9554`) | ✅ HW 검증완료 |
| 교반 BLDC (M2) | `tb_stir_en` 외 (`tb_tca9554`) | ✅ HW 검증완료 |
| 리프트 모터 (U6) | `tb_lift_enable` 외 (`tb_lift`) | ✅ HW 검증완료 |
| NTC 온도센서 3종 | `tb_therm_*` (`tb_thermistor`) | ⏳ read 동작 O / 온도값 정확도 미검증 (§7) |
| 홀센서 강음/모음/정지 (HS1~3) | `tb_hall_*` (P0~P2) | ❌ 전기적 단락으로 사용 불가 (§6) |
| 홀센서 배수/동작 (HS4~5) | `tb_hall_*` (P3~P4) | ✅ 정상 동작 확인 (§6) |

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
| `tb_wdoor_duty` | PWM 0~100 % | 50 |
| `tb_wdoor_reverse` | 0=Open, 1=Close | 0 |
| `tb_tdoor_enable` | 1=구동 / 0=정지 — 투입문 | 0 |
| `tb_tdoor_duty` | PWM 0~100 % | 50 |
| `tb_tdoor_reverse` | 0=Open, 1=Close | 0 |

(WDoor = 배수문 = 실질적 배수구 도어. 물리 리밋/시간 제어는 없으므로 duty·시간 직접 관리.)

## 3. `tb_stepmotor` — STEP1·STEP2 스테퍼

24 V 유니폴라 4상. `HAL_GetTick()` 기반으로 `period_ms`마다 1스텝 진행.
※ STEP2(35BYJ46)는 사용 전 커넥터 re-pin 필요(RED→pin1). 배선은 헤더 주석 참조.

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
| P1 (HS2) | 1 | 모음 | `tb_hall_trig[1]` / `tb_hall_collect` | `tb_hall_trig_events[1]` |
| P2 (HS3) | 2 | 정지 | `tb_hall_trig[2]` / `tb_hall_stop` | `tb_hall_trig_events[2]` |
| P3 (HS4) | 3 | 배수 | `tb_hall_trig[3]` / `tb_hall_drain` | `tb_hall_trig_events[3]` |
| P4 (HS5) | 4 | 동작 | `tb_hall_trig[4]` / `tb_hall_run` | `tb_hall_trig_events[4]` |
| P5 (HS6) | 5 | 교반원점 | `tb_hall_stir_home` | `tb_hall_stir_home_events` |
| P6 (HS7) | 6 | 수거통 | `tb_hall_bin` | `tb_hall_bin_events` |
| P7 (HS8) | 7 | 리프트하단 | `tb_hall_lift_bottom` | `tb_hall_lift_bottom_events` |

기타 관찰 변수:
- `tb_hall_mask` : 검출 마스크(P0~P7), `tb_hall_trig_mask` : 트리거군(P0~P4) 패킹(0~0x1F)
- `tb_hall_pinlevel` : **실제 핀 전압 레벨(1=HIGH)** = `~tb_hall_mask` (HW 폴라리티 0xFF 반전). 배선 결함 추적용 진단 뷰.
- `tb_hall_samples`(enable 중 poll마다 ++), `tb_hall_int_events`(HALL-INT1 서비스 횟수), `tb_hall_changes`(마스크 변화 횟수)

넷리스트 배선: HS1~HS5 → 커넥터 **J26**(핀 2~6), HS6/HS7/HS8 → **J28/J29/J30**. HS 라인에 **풀업 저항 없음**(각 네트 = J핀 + U24핀 2점).

### 검증 진행 상태 / 이슈 (2026-08-12 현재)

| 채널 | 핀 | 상태 |
|------|----|------|
| HS4 배수 / HS5 동작 | P3 / P4 | ✅ 정상 동작 확인 |
| HS1 강음 / HS2 모음 / HS3 정지 | P0 / P1 / P2 | ❌ **전기적 커플링으로 현재 사용 불가** |
| HS6 교반원점 / HS7 수거통 / HS8 리프트하단 | P5 / P6 / P7 | ⏳ 미검증 |

**HS1/HS2/HS3 (P0/P1/P2) 문제 — 원인 분석 결과:**

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

> 펌웨어로는 단락된 노드를 분리할 수 없어 **코드 수정 없음**. 하드웨어 리워크 후 재검증 대상.

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
tb_wdoor_reverse = 0    // Open
tb_wdoor_duty    = 60
tb_wdoor_enable  = 1

// 급수 ON + 수위센서 감지 (tb_water_sen1/2_present, _events 관찰)
tb_water_enable  = 1    // WATER-ON HIGH, SEN1/SEN2 인식 시작
// ...확인 후...
tb_water_enable  = 0    // 급수 OFF
```

## 이번 세션 진행 결과 요약

1. `tb_tca9554`: 분쇄/교반 BLDC에 버튼 없이 구동 가능한 디버거 enable 변수 추가
   (`tb_grind_en/rev/spd_req`, `tb_stir_en/rev/spd_req`, edge-triggered, `apply_enable()`).
2. `tb_gpioout` **신규 생성**: 배수/급수 밸브 + 팬 3종(총 5개) on/off 개별 검사용.
   `freertos.c`에 include + `TB_GpioOut_Init()` / `TB_GpioOut_Poll()` 통합 완료.
3. `tb_hallsensor` **신규 생성**: U24 홀센서 8입력(트리거군 P0~P4 + 교반원점/수거통/리프트하단)
   `tb_hall_enable` 게이트 모니터. `freertos.c`의 `StartDefaultTask`에 include + `TB_HallSensor_Init()` / `TB_HallSensor_Poll()` 통합 완료.
   - 트리거 라벨 확정: HS1 강음 / HS2 모음 / HS3 정지 / HS4 배수 / HS5 동작.
   - **디버깅 결과**: HS4/HS5(P3/P4) 정상, **HS1/HS2/HS3(P0/P1/P2)은 전기적 커플링(저임피던스 단락 추정)으로 현재 사용 불가**,
     HS6/HS7/HS8(P5/P6/P7) 미검증. 펌웨어 정상 확인(디코드/읽기 경로 대칭), 하드웨어 리워크 대상. 상세는 위 §6 참조.
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
