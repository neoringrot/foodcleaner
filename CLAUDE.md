# foodcleaner — STM32F103ZET6 음식물처리기 펌웨어 (R2 → R3 작업 지침)

> 이 파일은 Claude Code 세션이 매번 읽는 프로젝트 지침이다. 2026-09-20 작성(장형동).
> 현재 리비전: **R2** (git `525c4f2` "r2 complete. but board error"). 다음 목표: **R3 (REV02 보드 + 제로지오 시나리오 2판 + 09.16 자가세척 + ★09.21 회전수 운전 0.3.0)**.
> **★2026-09-21 개정 4** — 교반·분쇄 **시간 패턴 전부 폐지 → "가이드 2회전 + 정지 2초 = 1회 운전"** 회전수 운전(`rotation.c`)으로 교체. 2단계(§0.13)의 4구간 패턴 작업은 무효. 검토서 **§17** 이 §4·§16.1보다 우선한다.

## 1. 먼저 읽을 것 (순서대로)

1. `Core/doc/R3_변경범위_검토R3.md` — **R3 변경범위 검토서 개정 4.** 무엇을 왜 바꾸는지의 유일한 기준. **§17(09.21 회전수 운전)** → §0(결론) → §14(REV02 회로) → §15(ioc) → §16(착수 순서) 순으로 읽는다.
1-1. `../../doc/R3/개발사 전달(2026.09.21_회전 운전 수정)/04. C언어 프로그램(개발사 수정·통합용)/` — **0.3.0 원본.** `src/rotation.c`·`include/rotation.h`는 HW 무관 순수 로직이라 **그대로 이식**한다(검토서 §17.3). `generated/config.c` P47~P53, `board/port_contract.json` I19·rotation_contract.
2. `../../doc/R3/R3_변경범위_리스트.xlsx` — 위 문서의 항목별 표(C001~C095, 13시트). `08_회로변경`·`09_ioc수정`·`10_음성파일`·**`12_0921변경`** 시트가 코드 작업의 직접 입력이다.
3. `Core/doc/구현현황_및_미구현_점검R3.md` — R2가 무엇을 어떻게 구현했는지(§0.5~§0.9가 최신 지시 이력).
4. `../../doc/R3/2_음식물처리기_동작시퀀스_펌웨어정리_장형동.xlsx` — R3 사양 원문 정리(14시트). **`13_0921_회전운전`** 시트에 변경 전/후·rotation 입력 계약·벤치 항목 V01~V12. 06 시트 P47~P55.
5. `../../sch/R3/FoodDisposal_REV02_NETLIST.NET` — 현재 보드 넷리스트. 핀·부품 질문은 이 파일로 답한다(`sch/R1`은 이전 보드).
6-1. `Core/doc/음성다운로드_프로토콜R3.md` — **VU 다운로드 규격서.** UART5 소유권 전환·프레임·커맨드·설계 판단·관찰변수·검증순서. 음성 굽는 일은 이 문서 하나로 끝난다(§0.14).
6. `Core/doc/U21_음성플래시_구조R3.md` + `../../doc/R3/U21_음성플래시_구조.xlsx` — **4단계(음성 경로) 설계서.** 슬롯 256 KB × 32, 디렉터리/헤더 바이트 맵, 재생 경로, 새 파일 목록, 검증 항목. 4단계는 이 문서대로 구현한다.
7-1. `Core/doc/I02_엣지실측_기록R3.md` — **I02(가이드 엣지 정의) 벤치 기록지.** Watch 목록·사전확인·run A/B/C·판정표. 측정이 끝나면 §6 결론을 보고 `rinse_lock` 을 착수한다.
7. `Core/doc/공통헹굼_모듈_구조R3.md` — **5단계(헹굼 계층 B안) 설계서.** `rinse.*` / `rinse_lock.*`(UL1 상당) / `guide_edge.*` 분리, R001~R019 대응 `RinseStep`, cnt 20/40, 신호 출처 매핑, 검증 V1~V12, 작업 순서. **★개정 4로 §5.1 `RinseCtx`의 `fwd_ms/stop_ms/rev_ms`와 §8 `Rinse_StirTick()`(dj_stir_alt_tick 재사용)은 무효** — `zg_rotation` 래퍼로 다시 설계한 뒤 구현한다(검토서 §17.5·§17.8).

## 2. 절대 규칙

- **`src/pythonapp/`는 건드리지 않는다.** STM32 코드가 확정된 뒤 마지막 단계(8단계)에서 일괄 동기화한다.
- **git 작업 트리의 89개 파일 "수정"은 CRLF/공백 차이뿐이다** (`git diff --ignore-all-space --stat`이 비어 있음). 이 파일들을 커밋하지 말 것. `git add`는 실제로 고친 파일만 명시적으로.
- 코드 변경은 **검토서 §16.1 착수 순서**를 따른다. 단계를 건너뛰지 않는다. 특히 **3단계(.ioc/REV02 반영)를 하지 않은 채 벤치 코드를 고치지 않는다** — PF7이 아직 수위2로 잡혀 있어 새 홀이 수위 인터럽트로 들어온다.
- 상수 하나를 바꾸면 그 상수가 **어느 사용자 지시(구현현황 §0.x) 또는 R3 항목(Cxxx)** 에서 왔는지 헤더 주석에 남긴다. 이 저장소의 관례다(`moeum.h`·`dongjak.h` 참조).
- 회로가 하드웨어로 강제하는 것(K1~K3, U26/U35 AND, U30 mux)을 **소프트웨어로 대체하거나 우회하지 않는다.** MCU는 관측·명령만 한다.
- 미확정 항목(검토서 N5·N7~N11·**N12~N17**, I02·I03·I10·I14·**I19**)에 걸린 값은 **임의로 박지 않는다.** `#ifndef` 기본값 + `/* TBD: Nx */` 주석으로 자리만 만든다. 특히 **P54(tick/회전)·P55(피드백 타임아웃)·DIR 극성(N14)** 은 실측 전 값을 넣지 않는다 — DIR 상수는 확인 전 `#error` 가드.
- **회전 방향 정의**: "위에서 볼 때 시계방향(CW) = 정회전" (0.3.0). 코드·주석·벤치 로그 전부 이 정의로 쓴다.
- **UART5 보율은 115200 이다**(2026-09-20, 음성 다운로더). `.ioc`·`usart.c`·`PROTO_UART_BAUD` **세 곳이 같이** 움직여야 한다 — `PROTO_UART_BAUD` 는 주석이 아니라 모니터링 주기 하한의 실제 입력이다. 앱 쪽 포트 설정도 115200 으로 맞춰야 한다.
- 문서 갱신: 코드에 반영한 내용은 `Core/doc/구현현황_및_미구현_점검R3.md`에 새 §0.x 절로 기록한다(지시 원문 → 반영 파일·상수 → 검증 상태). `R3_변경범위_검토R3.md`는 검토 문서이므로 "반영 완료" 표시만 단다.

## 3. 저장소 구조

```
Core/Devices/    드라이버 (Motor: bldc_ctrl·drv8306·step_motor·lift_motor / DCMotor: drv8871·wdoor·tdoor /
                 ExtGpio: tca9554·keypad / Speaker: lm4871 / EEPROM: w25q128(U21 SPI1) / Comm: protocol_r0·ble·wifi / thermistor·distance·hallsensor·dc_current)
Core/Interface/  gpio_ctrl(핀 테이블) · adc_ctrl · uart_ctrl · heater_hyst
Core/Scenario/   moeum(모음) · dongjak(동작) · mode_arbiter(마개 홀 중재자, g_app_mode 소유) · jungji(정지 단일화) ·
                 kangeum·baesu(스텁) · *.md 검토/현황 문서
Core/Testbench/  tb_* 벤치 (g_app_mode == TESTBENCH 일 때만 폴링)
Core/Src         CubeMX 생성 + freertos.c(StartDefaultTask 100ms 센서 / StartMotorTask 1ms 모터)
foodcleaner.ioc  STM32CubeMX 6.18 — 핀·주변장치의 단일 출처. 라벨 변경은 여기서 → 재생성 → gpio_ctrl.c 테이블 동기화
```

## 4. 빌드·검증

- STM32CubeIDE 프로젝트(`.cproject`). 커맨드라인 컴파일 점검은 `arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -std=gnu11 -Wall -DUSE_HAL_DRIVER -DSTM32F103xE` + `-I Core/Inc -I Drivers/STM32F1xx_HAL_Driver/Inc -I Drivers/CMSIS/Device/ST/STM32F1xx/Include -I Drivers/CMSIS/Include -I Middlewares/Third_Party/FreeRTOS/Source/include -I Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3` 로 `Core/**/*.c` 각 TU를 `-c`. **경고 0**이 기준(R2 관례).
- HW 검증은 사용자가 벤치에서 한다. 코드만으로 "검증 완료"라고 쓰지 않는다 — `HW미검증_항목R3.md`에 항목을 추가한다.

## 5. 지금 할 일 (검토서 §16.1)

| 단계 | 내용 | 상태 |
|---|---|---|
| 0 | ~~N5~~·N7~N11 회신 대기. **N0는 B안(자체 구현)으로 진행** | **N5 해소(2026-09-20)** — 사용자 확정 *"2026.09.16이 최종이므로 그 기준에 따라 작성된 문서들을 따라야 한다"*. N7~N11은 회신 대기 |
| 1 | 회로 무관 수치 개정 (C017·C043·C056·C057·C058·C074 + ★C092 교반 30rpm·P22=1200) | ✅ **완료(§0.12)** — 57 TU `-Wall` 경고 0. ★개정 4: **C034(5s/2s/120s)는 추적용으로 강등**(C093), `DJ_STIR_RPM` 건조 20→30·`DJ_GRIND_INIT_RPM` 1500→1200 소량 재작업 필요 |
| 2 | ~~건조·식힘 패턴 개정 (C013·C035)~~ → **heater_since 기산(C005·C041)·C036·C040만 유효** | ✅ §0.13 완료였으나 ★**개정 4로 4구간 시간 패턴(`dj_stir_alt_tick`)은 무효** — 코드는 건드리지 말고 5단계에서 통째로 교체. ✅ **`heater_since` 공정 시계 반영(§0.24, 2026-09-21)** — 110/120분(+130/135분, 식힘 10분 유지) 기산 = 최초 HT_POWER ON(래치 되읽기). HW 1-31 대기. **2단계 코드 잔여 없음.** 벤치 1-19의 교반 패턴 항목은 13시트 V01~V12로 대체 |
| 3 | `.ioc` REV02 반영(ioc-1~5) + ~~tca9554 주소표~~ + ~~키패드/LED 드라이버 복원~~ **→ 완료(§0.10)** + **SW 분쇄 게이트 제거(C031)** + `DJ_COOL_USE_BIMETAL80` 정리(C032) + **C028 가열 직후 대기**(2단계에서 이관 — 전이 조건이 `grind_allowed && outlet_closed` 라 C031 과 같은 신호) | ✅ **완료(§0.17·§0.19, 2026-09-20)** — ioc-1~5 반영·generate, 수위 SEN2 소멸 정리, C032(`DJ_COOL_USE_BIMETAL80`=0), **N2 해소**(SB11 오픈·SB13 쇼트·J7=70℃) → C031 은 `dj_grind_allowed()`=(PF0==0)&&(PF3==0) 관측 게이트로 교체, C028 은 분쇄·교반 둘 다 70℃ 대기(사용자 선택 (가)). 코드 잔여 없음. N9·N10(J38 50℃)은 회신 대기지만 코드 영향 없음 |
| 4 | 음성 재생 경로: `U21_음성플래시_구조R3.md` §7 골격대로 — ~~`Devices/w25q.*`~~ **→ `Devices/EEPROM/w25q128.*` 작성완료(§0.11)** · `Speaker/voice.*` · `voice_table.h`(도구 생성) · `Testbench/tb_voice.*` **→ 플래시측 완료(§0.11.5)** · **`EEPROM/voiceupdater.*` + `src/voice_updater/` 다운로더 완료(§0.14)** · TIM2/DAC/DMA2_CH3 ioc | ◐ **구현 완료** — `voice.*`(TIM2/DAC/DMA2_CH3 코드 소유, `.ioc` 비기재) · 다운로더. HW: 다운로드 1-21 ✅ · 재생 1-22 ◐(기본 동작) · 시나리오 음성 1차 8종 1-23 ⏳ |
| 5 | ★**회전수 운전 엔진 + 헹굼 계층 재작성(B안)**: ① `Scenario/rotation.c/.h` 0.3.0 이식 ② 위치 피드백 `stir_position_ticks`(FG×DIR + HS6 재동기)·`BldcCtrl_IsAtRest()`·타임아웃(C087~C090) ③ `tb_rotation` run F(2회전+정지 2s × N) 벤치 + 벤더 test_rotation 타깃 자가시험 ④ WASH/DRAIN 프로파일로 헹굼(C013·C014) ⑤ PROCESS 프로파일·110분 반대방향 2000rpm(C034~C040) ⑥ 준비 탐색·cnt 20/40·t_cycle·과속·순이동 + 공통 헹굼 모듈 | ★**⑥ rinse_lock ✅ 코드 완료(§0.34, 2026-09-22)** — 헹굼 종료 = 가이드 cnt 20/40(상승엣지·미도달 15 s E09 사용자 확정), 재급수 차단·과속 60 s·이상 래치. HW 1-38 대기. ★**순이동 C060·C094 ✅(§0.38)** — 운전 중 가이드 소실 1.25 회전 → E01(B006), HW 1-42. ★**과속 보조 C061 ✅(§0.41)** — A 날개 rpm > 40 300 ms · B 가이드 간격 < 1.5 s ×2 → E02, HW 1-45. ★**예외처리 §0.33** — 에러 중 냉각·배기팬 유지, 해제형(E01·E02)/래치형(E03·E04·E08·E09 — ClearError 만), `last_err`. HW 1-39. ★**⑤ ✅ 코드 완료(§0.31, 2026-09-22)** — 건조 교반·분쇄 = PROCESS 회전수 운전(분쇄 동기, `RotStir_StartProcess/TickEx`), N14 M1 확정(reverse0=CW, **M2 와 반대**), 허가 = 분쇄허가 && 배출문 닫힘(`DJ_DRY_REQUIRE_OUTLET`), HW 1-37 대기. **①~④ ✅ 코드 완료(§0.23·§0.25·§0.26, 2026-09-21)** — ④: 헹굼 교반 = WASH/DRAIN(`rotation_port`), HW 1-32 대기. ★**⑥ 중 준비 탐색(R001~R006) 앞당겨 완료(§0.27)** — `rinse.c` 1차, `MOEUM_PREP`(9)·`DJ_PREP`(20)·`DJ_ERR_GUIDE`(9), HW 1-33 대기(1-32 앞단). 이하 ①~③ 기록: **①~③ ✅ 코드 완료(§0.23, 2026-09-21)** — `rotation.c/h` 본문 무수정 이식(출처 주석만, sha256 검증) · `BldcCtrl_Position()`(FG×**DIR핀 기준** 부호)·`BldcCtrl_IsAtRest()`(`BLDC_REST_MS 300` TBD:N15) · `tb_rotation`. **HW 1-30 대기.** P54=**822** 실측 완료(§0.22). **④ 착수 조건 전부 충족 ✅ (2026-09-21)** — 1-30 A 자가시험 · N14(M2: DIR_CW = 위에서 반시계) · B-2 at_rest(`BldcCtrl_CoastAwake`, 관성 ≤25 ms) · N13(FG×DIR, P54=822 검증). **시나리오 위치원은 FG(`BldcCtrl_Position`)** — HS6 단순안은 양자화가 실측돼 벤치 전용. ⑤는 ④ + N16(`heater_since` 는 §0.24 완료). ⑥은 N17·I15·`공통헹굼_모듈_구조R3.md` 개정 |
| 6 | 배출·공통 해제·자가세척: 무게 ADC 2.108s·E08·mux 구간 FG 폐루프 정지·자가세척 재진입 | ★**배출 회로 대응 ◐ 코드 완료(§0.37, 2026-09-22)** — 교반 = 회로(U30 SEL=B, MCU 무명령·FG 관측), 타이머 완료 즉시 닫기, SW 백업 140 s·닫기 전 TIMER-OUT 만 해제 인정, 130분 AND 식힘 완료, 무게 PC3 관측(임계 N20, 해제 미사용 N7). HW 1-41. 배출 중 마개 이탈은 질의 N21. ★**자가세척 ◐ 코드 완료(§0.30, 2026-09-22)** — 매 정상 처리·배출 후 자동(R3), `DJ_SELFCLEAN`(21), 헹굼 회차 루프 `rinse.c` 이식(rinse_lock 은 §0.34). HW 1-36 대기. 나머지(무게·E08·mux FG)는 N7 회신 후 |
| 7 | 음성 매핑 26종 · 버튼 8개 배정 · LED · ★반복 설정 N(C004) | N8·I10 회신 후. ★**C004: 모음·동작 헹굼 1회 고정(§0.35, 사용자 지시) — N 입력 경로는 TODO(N18), 버튼 배정과 함께 차후 질의.** ★**C068: U32 LED 8개 기능 미정의 → 질의 N22**(회신 전 = 누른 키 LED 만 점등). **드라이버는 준비됨** — `Keypad_TakePress/TakeLongPress` 소비자만 붙이면 된다(§0.10.5) |
| 8 | `src/pythonapp/` 일괄 동기화 | ◐ **선행 동기화(§0.40, 2026-09-22 사용자 지시)** — 보율 115200·상태/에러 코드·공정 시계·회전수 운전 표시·검증 규칙 R3화, `동작 (자가세척부터)`(0x0A)·`에러 해제` 버튼. HW 1-44. 이후 펌웨어가 바뀌면 이 앱도 같이 고친다 |

**다음 세션 권장 시작 프롬프트 (질의 회신 반영)**: "검토서 §17.7 N17~N21·§16 N7 과 구현현황 §0.33~§0.37 을 읽고 **회신이 온 질의만** 반영해(N18 반복 N·버튼, N19 락 추정, N20 무게 임계, N21 배출 중 마개 이탈, N7 무게 AND). 회신 전 항목의 상수·스위치는 건드리지 않는다. 벤치 결과(HW 1-32~1-41)가 있으면 먼저 그 판정부터. 구현현황 **다음 빈 §0.x**(쓰기 직전 grep — §0.37 배출), HW미검증 새 항목(1-42~), 클린 빌드 `-Wall` 경고 0."
