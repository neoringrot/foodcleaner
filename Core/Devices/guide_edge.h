#ifndef DEVICES_GUIDE_EDGE_H_
#define DEVICES_GUIDE_EDGE_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * guide_edge - 가이드(교반 원점) 홀 엣지 카운터.  R3 5단계 1순위 작업.
 *
 * 근거: 검토서 §4.2 · 설계서 [공통헹굼_모듈_구조R3.md] §4·§11-1 · 항목 C007.
 * 레퍼런스 `controller.c` 의 `zg_inputs.guide_edge_count` / `guide_last_edge_ms`
 * 자리를 채운다. 이 카운터 위에서 rinse_lock 이 cnt 20/40 을 합성한다(C014).
 *
 * 채널: **HS6 = U24 P5 = J28-2**  (검토서 §7 Q9 에서 확정. R2 가
 *       `tb_hall_stir_home` 으로 라벨링해 2026-08-15 에 HW 검증까지 마친 채널이다.)
 *
 * ── 왜 별도 모듈인가 ───────────────────────────────────────────────────
 * hallsensor.c 의 캐시는 **두 태스크가 갱신한다**:
 *   - StartDefaultTask(100ms) : HallSensor_ServiceInt() + HallSensor_Update()
 *   - StartMotorTask(1ms)     : 이 모듈의 GuideEdge_Tick()
 * 그래서 엣지 판정을 캐시 갱신 지점에 두면 어느 쪽이 먼저 보느냐에 따라 엣지가
 * 중복되거나 누락된다. 이 모듈은 **자기 prev 를 따로 소유**해서, 캐시를 누가
 * 언제 갱신했든 "내가 마지막으로 본 값과 다르면 1회" 로만 센다.
 *
 * ── 응답시간 (C007 / TB-B04: 감지~정지 ≤50ms) ────────────────────────
 * U24 는 입력이 바뀌면 HALL-INT1(PF9, active-low open-drain)을 끌어내린다.
 * GuideEdge_Tick() 은 1ms 마다 그 EXTI 플래그만 보고, **플래그가 섰을 때만**
 * HallSensor_ServiceInt() 로 I2C 1회를 읽는다. 평상시 버스 트래픽은 0 이고,
 * 엣지가 오면 ≤1ms 안에 카운트된다(100ms 폴링만으로는 50ms 요구를 못 맞춘다).
 * I2C1 은 tca9554.c 의 i2c1_mutexHandle 로 직렬화되므로 keypad/defaultTask 와
 * 경합해도 안전하다.
 *
 * ── 카운터 규약 ───────────────────────────────────────────────────────
 * `GuideEdge_Count()` 는 **단조 증가만 하고 리셋되지 않는다**. 회차/준비 탐색은
 * 호출부가 baseline 을 떠서 차이로 본다(레퍼런스 `guide_baseline` 과 동일):
 *
 *     base = GuideEdge_Count();
 *     ...
 *     cnt  = GuideEdge_Count() - base;     // uint32 modular, 랩어라운드 안전
 *
 * 이렇게 하면 카운터를 0 으로 되돌리는 순간이 없어 tick 과의 경쟁이 없다.
 *
 * ── 엣지 정의 — ★2026-09-22 사용자 확정: **상승엣지** ────────────────
 * cnt 20/40 의 "1" = 자석 도착 1회 = 가이드 1회전(I02 실측: 정·역 대칭, 양엣지는 정확히 2배).
 * rinse_lock(cnt 20/40)이 이 정의를 전제로 한다 — 양엣지로 바꾸면 40 이 20회전이 된다.
 * GuideEdge_SetBothEdges() 는 벤치 계측용으로만 남긴다.
 * ---------------------------------------------------------------------- */

/* HallSensor 마스크 안에서의 가이드 채널 비트. U24 P5 = HS6 = J28-2. */
#ifndef GUIDE_EDGE_HS_BIT
#define GUIDE_EDGE_HS_BIT        5U
#endif

/* 엣지 정의. 0 = 상승엣지만(자석 도착), 1 = 양엣지(도착+이탈).
 * ★2026-09-22 확정: 0(상승엣지). 제품 빌드에서 바꾸지 말 것(rinse_lock cnt 20/40 의 전제). */
#ifndef GUIDE_EDGE_BOTH
#define GUIDE_EDGE_BOTH          0U
#endif

/* 채터 억제: 직전에 센 엣지로부터 이 시간 안에 온 변화는 세지 않는다.
 * 30rpm 출력축이면 1회전 2초라 수십 ms 단위로 지나간다 - 5ms 는 충분히 짧다.
 * 0 으로 두면 디바운스 없음(원시 파형 관찰용). */
#ifndef GUIDE_EDGE_DEBOUNCE_MS
#define GUIDE_EDGE_DEBOUNCE_MS   5U
#endif

/* ---- 수명주기 ----------------------------------------------------------- */
/* HallSensor_Init() 뒤에 1회. 카운터 0, prev 미확정(첫 Tick 이 seed 한다 -
 * 부팅 시점의 레벨을 엣지로 세지 않기 위함). */
void     GuideEdge_Init(void);

/* 1ms, StartMotorTask. PF9 INT 플래그가 섰을 때만 U24 를 읽고, 가이드 비트의
 * 변화를 자기 prev 와 비교해 센다. 플래그가 없으면 캐시만 보고 즉시 반환한다
 * (누가 캐시를 갱신했든 변화는 여기서 한 번만 소비된다). */
void     GuideEdge_Tick(uint32_t now_ms);

/* ---- 관측 -------------------------------------------------------------- */
uint32_t GuideEdge_Count(void);        /* 단조 증가. baseline 차이로 사용     */
uint32_t GuideEdge_LastEdgeMs(void);   /* 마지막으로 센 엣지의 tick           */
uint8_t  GuideEdge_Level(void);        /* 현재 가이드 비트(1 = 자석 감지)     */
uint8_t  GuideEdge_HasPrev(void);      /* 0 = 아직 seed 전                    */

/* 마지막 엣지 간격(ms). 회전 주기 실측·과속 판정(C061)의 1차 재료.
 * 엣지가 2회 미만이면 0. */
uint32_t GuideEdge_LastIntervalMs(void);

/* ---- I02 실측용 ---------------------------------------------------------- */
/* 엣지 정의 전환. both=0 상승엣지만 / both=1 양엣지. 전환해도 카운터는 유지된다
 * (단조 증가 규약). 벤치에서 같은 회전수로 양쪽을 재 비교하는 용도. */
void     GuideEdge_SetBothEdges(uint8_t both);
uint8_t  GuideEdge_GetBothEdges(void);

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_GUIDE_EDGE_H_ */
