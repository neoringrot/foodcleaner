#ifndef SRC_TB_ROTATION_H_
#define SRC_TB_ROTATION_H_

#include "main.h"
#include "rotation.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * tb_rotation - R3 개정 4(2026.09.21) 회전수 운전 엔진 `rotation.c/h` 벤치.
 *
 * 근거: 검토서 [R3_변경범위_검토R3.md] §17.3·§17.6·§17.8 · 벤더 0.3.0
 *       `board/port_contract.json` rotation_contract · `tests/test_rotation.c`.
 *
 * `Core/Scenario/rotation.c/h` 는 벤더 0.3.0 원본이다 — 출처 주석 블록만 앞에 붙였고
 * **본문은 바이트 그대로**다(원본 sha256 rotation.c 81a57eb1… / rotation.h 3bc35145…,
 * 파일 머리 주석에 전체 해시). 이 벤치는 그 엔진을 두 가지로
 * 검증한다. **시나리오(moeum/dongjak)에는 아직 연결하지 않았다** — 5단계 다음 작업.
 *
 * ── A. 자가시험 (모터 불필요) ────────────────────────────────────────────
 *   벤더 `tests/test_rotation.c` 를 **이 MCU·이 컴파일러**에서 돌린다. 벤더는 호스트
 *   (zig cc)에서만 돌렸고 PCB 시험은 없다(verification/현재 검증 결과.json).
 *     tb_rotation_selftest_once = 1
 *     → tb_rotation_selftest_done == 1 && tb_rotation_selftest_fail_line == 0 이면 통과.
 *       fail_line 은 **tb_rotation.c 의 줄 번호**다(벤더 줄이 아니다).
 *
 * ── B. run F — 실모터 "2회전 + 정지 2초 × N회" (§17.8) ──────────────────
 *   교반(M2)만 구동한다. 분쇄(M1)는 **돌리지 않는다** — 70℃ 허가 없이 명령하면
 *   ENABLE 이 LOW 라 소프트락된다(§0.19.2). PROCESS 에서도 grind_at_rest 는 실제
 *   M1 정지 관측값을 넣는다(구동 안 하니 늘 정지).
 *     tb_rotation_profile = 1(WASH) / 2(DRAIN) / 3(PROCESS)
 *     tb_rotation_once    = 1
 *   기대값:
 *     | profile | legs | dir_mask     | edges(HS6) |
 *     |---------|------|--------------|------------|
 *     | WASH    | 10   | 0x00000140   | 20         |  6·8번째 운전이 CCW
 *     | DRAIN   | 10   | 0x00000000   | 20         |  CW 만
 *     | PROCESS | 36   | 0xC0000000   | 72         |  30 CW → 2 CCW → 4 CW (max_legs 36)
 *   edges 가 2×legs 이고 leg_edges 가 매번 2 면 "2회전" 이 HS6 기준으로 맞은 것이다.
 *
 * ★위치원(pos_src) — 계약서는 "position 은 보정된 가이드 운동량이지 **홀 통과 횟수가
 *   아니다**" 라고 한다. 그래서:
 *     0 = HS6 엣지 × DIR 부호, ticks_per_rev = 1  — **§17.6 단순안. 벤치 1차 전용**.
 *         2회전 판정이 자석 위치에 양자화된다(±1회전). 역전 직후 운전은 자석이 바로
 *         뒤에 있어 1회전 남짓에서 끝날 수 있다 — WASH 7·9번째 운전을 볼 것.
 *     1 = M2 FG 하강엣지 × DIR 부호, ticks_per_rev = P54 — §17.6 주안(N13).
 *         P54 = **822** FG엣지/날개 1회전 (run G 실측 ±0.08%, 기록지 §11.7 · 구현현황 §0.22).
 *         감속비 환산(82 × 5/6)과 무관한 **직접 실측값**이다 — 계산값으로 바꾸지 말 것.
 *         pos_src=1 일 때 ticks_per_rev 가 1 이면 시작 시 822 로 채운다.
 *
 * ★at_rest — 계약서: "STOP 명령이나 고정 true 가 아니라 **실제 정지 관측**".
 *   드라이버 `BldcCtrl_IsAtRest()`(C089)를 쓴다: running==0 이고 FG 가 rest_ms 동안
 *   안 바뀌면 정지. 판정 시간은 **g_stir_ctrl.rest_ms**(기본 BLDC_REST_MS 300)를 직접 바꾼다.
 *   rest_wait_ms(정지 명령 → 정지 확인) 가 N15 의 실측값이다.
 * ★position(pos_src=1) — 드라이버 `BldcCtrl_Position()`(C088)은 **DIR 핀 기준 부호**다.
 *   "위에서 본 CW=+" 로의 변환은 이 벤치의 dir_invert 한 곳에서만 한다.
 *
 * ★방향(N14) — 엔진 반환 1=CW / 2=CCW 는 "위에서 볼 때" 다.
 *   **확정 [2026-09-21 run F, 사용자 육안]: M2 BldcCtrl reverse=0(DRV8306_DIR_CW) 는 위에서 볼 때
 *   반시계**다 → 기본 dir_invert = 1. (M1 분쇄 극성은 미확인.)
 * ★at_rest(§0.25) — 운전 사이 정지는 BldcCtrl_CoastAwake()(드라이버 깨운 채 duty 0).
 *   Stop() 의 슬립은 FG 풀업(DVDD)을 꺼 관성 회전을 가린다 — 첫 run F 의 rest_wait=301 이 그 착시였다.
 *
 * WHERE TO POLL: StartMotorTask(1ms) 테스트벤치 분기, BldcCtrl_Tick 앞.
 * 안전: 벤치 유휴에서만 시작, Jungji 제동 중 시작 안 함, tb_rinse 와 배타,
 *       벤치 이탈·LOCKED·엔진 실패 시 즉시 정지.
 * ---------------------------------------------------------------------- */

#define TB_ROT_ST_IDLE        0U
#define TB_ROT_ST_RUN         1U
#define TB_ROT_ST_DONE        2U

#define TB_ROT_RES_NONE       0U
#define TB_ROT_RES_COMPLETE   1U   /* WASH/DRAIN 10회 / PROCESS max_legs 도달      */
#define TB_ROT_RES_FAILED     2U   /* 엔진 failed (시나리오에선 E09) — fail_where   */
#define TB_ROT_RES_ABORT      3U   /* stop_once / 벤치 이탈 / BLDC LOCKED            */
#define TB_ROT_RES_BUSY       4U   /* tb_rinse 가 교반을 쓰는 중이라 거부           */

#define TB_ROT_FAIL_NONE      0U
#define TB_ROT_FAIL_CONFIG    1U   /* valid=0 또는 cfg 검사 실패(ticks·timeout 등)  */
#define TB_ROT_FAIL_PROGRESS  2U   /* 운전 중 timeout_ms 동안 위치 무진행            */
#define TB_ROT_FAIL_REST      3U   /* 정지 명령 후 timeout_ms 안에 정지 미확인       */
#define TB_ROT_FAIL_TRAVEL    4U   /* 명령 반대로 2^31 이상 이동(부호 뒤집힘)       */

/* ---- A. 자가시험 ---------------------------------------------------- */
extern volatile uint8_t  tb_rotation_selftest_once;
extern volatile uint8_t  tb_rotation_selftest_done;      /* 1 = 끝남            */
extern volatile uint32_t tb_rotation_selftest_asserts;   /* 통과한 REQUIRE 수   */
extern volatile uint32_t tb_rotation_selftest_fail_line; /* 0 = 전부 통과       */

/* ---- B. run F 설정 (시작 전에 쓴다) ---------------------------------- */
extern volatile uint8_t  tb_rotation_profile;      /* 1 WASH / 2 DRAIN / 3 PROCESS (기본 1) */
extern volatile uint16_t tb_rotation_rpm;          /* 교반 지령 (기본 30 = P08·P47)     */
extern volatile uint8_t  tb_rotation_pos_src;      /* 0 HS6 단순안 / 1 FG (기본 0)      */
extern volatile uint32_t tb_rotation_ticks_per_rev;/* P54. pos_src=0 이면 1 강제 / 1 이면 기본 822 */
extern volatile uint32_t tb_rotation_timeout_ms;   /* P55 벤치값 (기본 8000, >2000 필수). 30rpm 1회전 2.0s + 램프 1.5s */
extern volatile uint8_t  tb_rotation_max_legs;     /* PROCESS 종료 운전 수 (기본 36)    */
extern volatile uint8_t  tb_rotation_late;         /* PROCESS late_requested 수동       */
extern volatile uint8_t  tb_rotation_dir_invert;   /* N14: 1 = CW/CCW ↔ reverse 반전 (확정 기본 1) */
extern volatile uint8_t  tb_rotation_once;         /* ★시작 원샷                        */
extern volatile uint8_t  tb_rotation_stop_once;    /* 즉시 중단                          */

/* ---- B. run F 결과 (RO) ---------------------------------------------- */
extern volatile uint8_t  tb_rotation_state;        /* TB_ROT_ST_*                       */
extern volatile uint8_t  tb_rotation_result;       /* TB_ROT_RES_*                      */
extern volatile uint8_t  tb_rotation_fail_where;   /* TB_ROT_FAIL_*                     */
extern volatile uint32_t tb_rotation_legs;         /* 완료한 운전 수                    */
extern volatile uint32_t tb_rotation_dir_mask;     /* bit i = i번째 운전이 CCW (32개)   */
extern volatile uint32_t tb_rotation_edges;        /* 시작 이후 HS6 엣지 (기대 2×legs)  */
extern volatile uint32_t tb_rotation_leg_edges;    /* 직전 운전의 HS6 엣지 (기대 2)     */
extern volatile uint32_t tb_rotation_leg_ms;       /* 직전 운전 길이                    */
extern volatile uint32_t tb_rotation_leg_min_ms;
extern volatile uint32_t tb_rotation_leg_max_ms;
extern volatile uint32_t tb_rotation_leg_fg;       /* 직전 운전의 M2 FG 엣지 (P54 교차검증) */
extern volatile uint32_t tb_rotation_rest_wait_ms; /* ★N15: 직전 정지 명령 → 정지 확인 */
extern volatile uint32_t tb_rotation_rest_wait_max_ms;
extern volatile uint32_t tb_rotation_elapsed_ms;
extern volatile uint8_t  tb_rotation_locked;       /* 1 = 도중 BLDC_LOCKED              */
/* 엔진 미러 */
extern volatile uint32_t tb_rotation_position;
extern volatile uint8_t  tb_rotation_dir;          /* 엔진 반환 0/1/2                   */
extern volatile uint32_t tb_rotation_cycle;
extern volatile uint32_t tb_rotation_phase;        /* PROCESS: 0 초기 / 1 역 / 2 정     */
extern volatile uint8_t  tb_rotation_initial_done;
extern volatile uint8_t  tb_rotation_is_late;
extern volatile uint8_t  tb_rotation_stir_rest;
extern volatile uint8_t  tb_rotation_grind_rest;

void    TB_Rotation_Init(void);
void    TB_Rotation_Poll(uint8_t bench_idle);
uint8_t TB_Rotation_Busy(void);   /* 1 = run F 가 교반을 쓰는 중 (tb_rinse 배타용) */

#ifdef __cplusplus
}
#endif

#endif /* SRC_TB_ROTATION_H_ */
