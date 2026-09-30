#ifndef SCENARIO_ROTATION_PORT_H_
#define SCENARIO_ROTATION_PORT_H_

#include "main.h"
#include <stdbool.h>
#include "rotation.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * rotation_port - 벤더 0.3.0 회전수 엔진(rotation.c) ↔ 교반 BLDC(g_stir_ctrl) 어댑터.
 * ★R3 개정4 5단계 ④ [2026-09-21, 구현현황 §0.26] — C013·C014·C087~C091.
 *
 * 엔진은 HW 를 모른다. 이 모듈이 엔진 입력 4종을 만들고 반환(0 정지/1 CW/2 CCW)을
 * 교반 모터 명령으로 바꾼다. **교반 모터는 활성 RotStir 하나만 만진다** — 호출부는
 * 회전수 운전 구간에서 BldcCtrl 교반 API 를 직접 부르지 않는다.
 *
 *   position      = BldcCtrl_Position(&g_stir_ctrl) 를 "위에서 본 CW = +" 로 변환 (C088)
 *                   P54 = ROT_M2_TICKS_PER_REV (run G 실측 822, §0.22·1-30 B-2 교차검증)
 *   stir_at_rest  = BldcCtrl_IsAtRest(&g_stir_ctrl)  — 운전 사이 정지는 CoastAwake 라 실측 (C089, §0.25)
 *   grind_at_rest = BldcCtrl_IsAtRest(&g_grind_ctrl) — WASH/DRAIN 은 엔진이 보지 않는다
 *   permitted     = 호출부 인자. WASH/DRAIN 은 true (벤더 controller.c 와 같다)
 *   late_requested= 호출부 인자. WASH/DRAIN 은 false
 *
 * ★⑤ [2026-09-22, 구현현황 §0.31] PROCESS(건조) = **분쇄 동기(paired)**. 벤더 controller.c ZG_D008~D010:
 *   - 분쇄는 교반과 **같은 방향·같은 가동 구간**(엔진 반환이 0 이면 둘 다 정지, 정지 2 s 동안 분쇄도 정지).
 *   - 엔진 `late` 가 서면(heater_since+110분 && 초기 30회 완료) **분쇄만 교반 반대 방향**, rpm = late_grind.
 *   - 정지 확인은 교반·분쇄 **둘 다**(엔진 paired rest). 운전 사이 정지는 둘 다 CoastAwake.
 *   - 방향 극성은 모터마다 다르다: M2 CW = reverse 1, M1 CW = reverse 0 (N14).
 *
 * 하지 않는 것 (벤더 계약 count_mismatch):
 *   - **HW 20/40 감지 마일스톤을 합성하지 않는다.** 엔진이 10회 운전(20회전)을 끝낸 것은
 *     "소프트웨어 회전수 완료" 일 뿐이다. cnt 20/40·t_cycle·과속 판정은 ⑥ rinse_lock 몫이다.
 *     HS6 엣지 증가분은 `edges` 로 **관측만** 남긴다.
 * ========================================================================== */

/* ---- 추적용 상수 스위치 (검토서 §17.5) ----------------------------------- */
/* 회전수 운전으로 대체된 **시간 종료 상수**(헹굼 120초·18사이클·배수 30/90초 등)는 값 보존만
 * 하고 제어에 쓰지 않는다. 0(기본)이면 정의 자체가 빠져, 실수로 다시 쓰면 빌드가 깨진다. */
#ifndef ZG_TRACKED_ONLY
#define ZG_TRACKED_ONLY             0
#endif

/* ---- 방향 극성 (N14 / C091) --------------------------------------------- */
/* ★확정 [2026-09-21 run F, 사용자 육안]: M2 DRV8306_DIR_CW(reverse=0) = 위에서 볼 때 반시계.
 * → 엔진 CW(위에서 시계, 0.3.0 정회전) = reverse 1.  (구현현황 §0.25, HW 1-30) */
#ifndef ROT_M2_CW_IS_REVERSE
#define ROT_M2_CW_IS_REVERSE        1U
#endif
/* ★확정 [2026-09-22 벤치, 사용자 육안 — 분쇄날 기준]: M1 DRV8306 reverse=0(tb_grind_rev=0) = 위에서 볼 때 **시계**.
 * → 엔진 CW = reverse 0. **M2 와 극성이 반대다**(M2 는 CW = reverse 1) — 같은 방향으로 돌리려면
 *   두 모터에 서로 다른 DIR 을 줘야 한다. (N14 M1, C091) */
#ifndef ROT_M1_CW_IS_REVERSE
#define ROT_M1_CW_IS_REVERSE        0U
#endif
/* 가드는 남겨 둔다 — 누가 위 정의를 지우면 ⑤ 분쇄 연결이 빌드되지 않는다. */
#if defined(ROT_USE_GRINDER) && !defined(ROT_M1_CW_IS_REVERSE)
#error "N14(M1): 분쇄 M1 방향 극성 미확인 - 위에서 본 CW 가 DRV8306 DIR 어느 쪽인지 실측 후 ROT_M1_CW_IS_REVERSE 정의"
#endif

/* ---- P54 / P55 ---------------------------------------------------------- */
/* ★2026-09-22 사용자 확정: 822 → **826** (보정 여유). 실측 가이드 1회전 = 822.2~823.2 FG(30rpm),
 * 825.2(40rpm) — 822 로 두면 "2회전" 운전 1회가 실제 ≈1.998 회전이라 DRAIN 20회전 끝에서 ≈10° 모자라
 * **cnt 40 직전 엣지를 놓쳤다**(모음 09:22·동작 09:03 벤치, 둘 다 cnt 39 → E09). 준비 탐색이 자석 위에서
 * 멈춰 cnt 20/40 엣지가 늘 운전 경계에 오기 때문이다(구현현황 §0.51). 벤더 port_contract count_mismatch
 * "investigate calibration" 에 따른 보정 — 실측 최대(825.2)보다 크게 잡아 운전 1회가 항상 ≥ 2 회전.
 * 30rpm 에서 ≈2.007 회전/운전(20회전당 ≈+25°). 순이동 한도(1.25×P54)는 1028 → 1033. 실측값 822 는
 * 벤치 tb_rotation.c TB_ROT_P54_FG_PER_REV 에 그대로 남긴다. */
#ifndef ROT_M2_TICKS_PER_REV
#define ROT_M2_TICKS_PER_REV        826U     /* P54 보정값 (실측 822.2~825.2 + 여유). 종전 822 */
#endif
/* P55: 운전 중 위치 무진행 · 정지 명령 후 정지 미확인 허용 시간. 엔진 조건 > P53(2000).
 * 무부하 실측: 관성 정지 ≤25 ms(1-30 B-2). 부하에서 기동 지연·잼 해제(정지 1.5 s + 역 0.7 s)가
 * 끼어도 오판하지 않도록 잠정 5000. 계약: "정상 부하와 10~20% 편차에서 오판 금지". */
#ifndef ROT_FEEDBACK_TIMEOUT_MS
#define ROT_FEEDBACK_TIMEOUT_MS     5000U    /* TBD: N15 — 부하(음식물) 헹굼 실측 후 확정 */
#endif

/* ---- RotStir_Tick 반환 -------------------------------------------------- */
#define ROT_ST_IDLE      0U   /* 프로파일 없음(Start 전 / Stop 후)               */
#define ROT_ST_RUNNING   1U   /* 진행 중                                        */
#define ROT_ST_DONE      2U   /* WASH/DRAIN 10회 운전 완료 — 모터는 깨운 채 정지 */
#define ROT_ST_FAILED    3U   /* 엔진 실패(시나리오 E09) — 교반 정지·슬립 완료   */

/* WASH/DRAIN 한 프로파일의 운전 수. rotation.c 에 10 이 하드코딩돼 있다(`cycle >= 10U`). */
#define ROT_WASH_DRAIN_LEGS          10U

typedef struct
{
	zg_rotation         eng;          /* 벤더 엔진 상태                             */
	zg_rotation_profile prof;         /* 현재 프로파일 (ZG_ROT_NONE = 비활성)       */
	uint16_t            rpm;          /* 이 프로파일의 교반 지령 (날개 rpm)          */
	uint8_t             out;          /* 직전 엔진 반환 0/1/2                       */
	uint32_t            out_since;    /* out 이 마지막으로 바뀐 tick (앱 표시용)    */
	uint32_t            edge_base;    /* Start 시 GuideEdge_Count()                 */
	/* ★⑤ PROCESS 분쇄 동기 (paired = 0 이면 분쇄를 만지지 않는다) */
	uint8_t             paired;
	uint16_t            grind_rpm;    /* 초기·반복 구간 분쇄 rpm (P22 1200)         */
	uint16_t            late_rpm;     /* late 구간 분쇄 rpm (P28 2000)              */
	uint8_t             late_on;      /* 분쇄를 반대 방향으로 명령 중(엔진 late 적용) */
	/* 관측 (RO) — watch·앱 보고용. 제어에 쓰지 않는다. */
	volatile uint32_t   edges;        /* Start 이후 HS6 엣지. **HW 마일스톤 아님**   */
	volatile uint8_t    failed;       /* 1 = E09                                    */
} RotStir_t;

/* 새 회전수 운전 시작 — 엔진을 **초기화**한다(이전 실패 래치 포함). */
void    RotStir_Start (RotStir_t *r, zg_rotation_profile p, uint16_t rpm);
/* 프로파일 전환(WASH→DRAIN) — 엔진을 **유지**한다. 엔진이 스스로 "돌고 있는 모터를
 * 뒤집지 않도록" 정지 확인 + 2 s 대기 후 새 프로파일을 시작한다(rotation.c 주석). */
void    RotStir_Switch(RotStir_t *r, zg_rotation_profile p, uint16_t rpm);
/* 1 ms MotorTick. 교반 모터를 이 함수만 만진다. 반환 ROT_ST_*. (late_requested = false) */
uint8_t RotStir_Tick  (RotStir_t *r, bool permitted, uint32_t now_ms);

/* ★⑤ PROCESS 시작 — 교반 + 분쇄 동기. 엔진 초기화(초기 30회부터). 이후 RotStir_TickEx 로 진행한다.
 * PROCESS 는 끝나지 않는다(반환 RUNNING/FAILED) — 호출부가 120분에 RotStir_Stop. */
void    RotStir_StartProcess(RotStir_t *r, uint16_t stir_rpm, uint16_t grind_rpm, uint16_t late_rpm);
/* permitted: false 면 엔진이 두 모터를 세우고 기다린다(카운트 유지 — 허가 복귀 시 이어서).
 * late_requested: heater_since+110분. 엔진이 초기 30회 완료 뒤에만 적용한다. */
uint8_t RotStir_TickEx(RotStir_t *r, bool permitted, bool late_requested, uint32_t now_ms);
/* 관측: PROCESS 초기 30회 완료 / late 적용 여부 */
uint8_t RotStir_InitialDone(const RotStir_t *r);
uint8_t RotStir_Late       (const RotStir_t *r);
/* 비활성화 + 교반 정지·**슬립**(BldcCtrl_Stop). paired 면 분쇄도. 운전 구간을 끝낼 때 부른다. */
void    RotStir_Stop  (RotStir_t *r);

/* 회전수 운전 **밖**의 교반 직접 구동 — 가이드 준비 탐색(rinse.c R001~R005) 전용.
 * ccw: 0 = 위에서 CW(정), 1 = CCW(역). 방향 극성은 이 파일의 ROT_M2_CW_IS_REVERSE 하나로 푼다.
 * rpm 0 = 깨운 채 정지(CoastAwake). 끝낼 때는 RotStir_Stop()(슬립).
 * 계약: 호출 시점에 활성 RotStir 가 없어야 한다(교반 구동 주체는 항상 하나). */
void    RotStir_Manual(uint8_t ccw, uint16_t rpm);

/* 앱 보고용 보조 */
uint8_t  RotStir_IsActive(const RotStir_t *r);          /* 프로파일 활성 && 미실패 */
uint32_t RotStir_Legs    (const RotStir_t *r);          /* 현재 프로파일에서 완료한 운전 수 */

#ifdef __cplusplus
}
#endif

#endif /* SCENARIO_ROTATION_PORT_H_ */
