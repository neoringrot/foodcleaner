#ifndef DEVICES_KEYPAD_H_
#define DEVICES_KEYPAD_H_

#include "main.h"
#include "tca9554.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * keypad - 전면 멤브레인 버튼 8개(U31) + 표시 LED 8개(U32) 제품 드라이버.
 *
 * 근거: R3_변경범위_검토R3.md §14.2/§14.3, 변경리스트 C076(주소표 재정의)·
 *       C084(J27 8버튼+8LED 보드). 2026-08-18 폐지한 `membrane.*`가 REV02에서
 *       하드웨어로 부활한 자리다.
 *
 * 하드웨어 (sch/R3/FoodDisposal_REV02_NETLIST.NET)
 *   U31 TCA9554APWR  A2,A1,A0 = 0,0,1 -> 0x39   P0..P7 = DIS-SW1..8   (입력)
 *   U32 TCA9554APWR  A2,A1,A0 = 0,0,0 -> 0x38   P0..P7 = DIS-LED1..8  (출력)
 *   J27 52207-1860 (FFC 18핀) 1=GND / 2~9=DIS-SW1~8 / 10=GND / 11~18=DIS-LED1~8
 *   U31 INT -> `HALL-INT2`(PF10, R179 4.7k 풀업) = GPIO_EXTI_HALL_INT2
 *   U32 INT -> `HALL-INT3`(PF11, R180 4.7k 풀업) = 출력 전용 칩이라 미사용
 *
 * 극성 (2026-09-20 REV02 벤치 확인):
 *   SW  = ACTIVE-LOW  (누름 = 핀 LOW). U31 의 극성반전 레지스터(0xFF)로
 *         "누름 = 논리 1" 로 정규화하므로 이 파일 밖에서는 전기적 극성을 몰라도
 *         된다(hallsensor.c 와 같은 방식).
 *   LED = ACTIVE-HIGH (점등 = 핀 HIGH). ★R2 시절 tb_tca9554 는 active-low 로
 *         적어 두고 "평소 전부 점등, 누른 키만 소등" 정책을 함께 썼는데, 반전이
 *         두 번 걸려 실제 파형은 "누른 키만 HIGH" 였다. REV02 실보드에서
 *         평상시 전 LED 소등 + 누른 키만 점등이 관측되어 LED 는 active-high 로
 *         확정했다. 파형은 그대로 두고 상수·정책의 의미만 바로잡은 것이다.
 *
 * I2C 주소가 넷리스트 라벨과 반대라는 R2 시절 메모가 tb_tca9554.h 에 있으나,
 * REV02 넷리스트 자체가 U31(SW)=0x39 / U32(LED)=0x38 이므로 결과는 같다.
 * 이 드라이버는 역할 이름 매크로(TCA9554_U31_SW_ADDR / TCA9554_U32_LED_ADDR)만
 * 쓴다.
 *
 * 소유권: U31/U32 버스 접근은 이 파일 하나로 모은다. tb_tca9554 는 자체
 * I2C 접근을 버리고 이 드라이버의 이벤트/LED API 만 쓴다(이중 소유 금지).
 *
 * 호출 모델 (freertos.c, StartMotorTask 1 ms 루프)
 *     Keypad_Init();
 *     for (;;) { ...; Keypad_Tick(HAL_GetTick()); switch (g_app_mode) {...} }
 *   Tick 은 KEYPAD_POLL_MS 마다(또는 U31 INT 가 떴을 때 즉시) U31 를 1회 읽고,
 *   LED 래치가 바뀐 경우에만 U32 에 1회 쓴다. 모드와 무관하게 항상 돌려야
 *   한다 - 버튼은 시나리오 실행 중에도 눌린다.
 *
 * 이벤트 모델: 눌림 엣지는 "래치"된다(1회 폴 펄스가 아니다). 100 ms 주기인
 *   StartDefaultTask 쪽 소비자도 놓치지 않게 하기 위함이고, 아무도 가져가지
 *   않은 이벤트는 KEYPAD_EVENT_TTL_MS 뒤 자동 폐기되어 모드가 바뀐 뒤 과거
 *   누름이 뒤늦게 실행되는 일을 막는다.
 *
 * ★미확정(I10·I14): 버튼 8개의 "기능 배정"은 아직 없다. 이 드라이버는 입력
 *   처리(디바운스·엣지·길게누름·LED)까지만 책임지고, SWn -> 동작 연결은
 *   keypad_map.* (미작성)에서 한다. 아래 keypad_key_t 는 패널 번호일 뿐
 *   기능 이름이 아니다.
 * ---------------------------------------------------------------------- */

#define KEYPAD_KEY_COUNT   8U   /* DIS-SW1..8 / DIS-LED1..8 */

/* 패널 번호(0-base). 기능 이름이 아니다 - 배정 확정(I10·I14) 전까지 의도적으로
 * 중립적인 이름만 둔다. */
typedef enum
{
	KEYPAD_SW1 = 0, KEYPAD_SW2, KEYPAD_SW3, KEYPAD_SW4,
	KEYPAD_SW5,     KEYPAD_SW6, KEYPAD_SW7, KEYPAD_SW8
} keypad_key_t;

/* ---- 타이밍 / 극성 상수 --------------------------------------------- */

/* 버스 폴 주기 - 2단이다.
 *
 * 이 드라이버는 시나리오가 도는 MotorTask(1 ms)에서 돌기 때문에, 블로킹
 * HAL_I2C 전송(100 kHz에서 레지스터 1회 읽기 ~0.45 ms)을 매 사이클 넣으면
 * 스테퍼 페이싱에 그만큼 지터가 실린다. 그래서 아무도 버튼을 만지지 않는
 * 동안에는 KEYPAD_IDLE_POLL_MS 로만(=안전망) 읽고, U31 INT(PF10)가 뜨거나
 * 눌린 키가 있는 "활성" 구간에서만 KEYPAD_POLL_MS 로 촘촘히 읽는다.
 *   유휴: 0.45 ms / 200 ms = 0.2 % - 스테퍼에 영향 없음
 *   활성: INT 로 즉시 1회 + 5 ms 폴 -> 누름 확정까지 ~15~20 ms
 * INT 가 죽어 있어도 동작은 한다(유휴 폴만으로 최대 ~400 ms 지연). */
#ifndef KEYPAD_POLL_MS
#define KEYPAD_POLL_MS            5U
#endif
#ifndef KEYPAD_IDLE_POLL_MS
#define KEYPAD_IDLE_POLL_MS       200U
#endif
/* 마지막 변화 이후 이 시간까지는 활성 구간을 유지한다(뗄 때의 디바운스까지). */
#ifndef KEYPAD_ACTIVE_MS
#define KEYPAD_ACTIVE_MS          150U
#endif

/* 같은 레벨이 이 시간만큼 연속 유지돼야 채택. 샘플 수가 아니라 시간으로 잡는
 * 이유는 INT 가 뜨면 폴 주기보다 빨리 읽기 때문이다(샘플 수 기준이면 바운스
 * 구간에서 INT 가 몰리며 디바운스가 짧아진다). 15 ms = 폐지된 membrane.* 와
 * 같은 폭. */
#ifndef KEYPAD_DEBOUNCE_MS
#define KEYPAD_DEBOUNCE_MS        15U
#endif

/* 길게누름 판정. 사용자 시나리오 §217 "정지 버튼 3초 이상 -> 완전 정지"에서
 * 온 값이다. 기능 배정(I14) 확정 전까지는 값만 잡아 둔다. TBD: I14 */
#ifndef KEYPAD_LONG_PRESS_MS
#define KEYPAD_LONG_PRESS_MS      3000U
#endif

/* 아무도 소비하지 않은 눌림/길게누름 이벤트의 유효기간. */
#ifndef KEYPAD_EVENT_TTL_MS
#define KEYPAD_EVENT_TTL_MS       500U
#endif

/* 이 마스크에 든 키(bit i = SW(i+1))는 눌림 이벤트를 "뗄 때" 낸다. 짧게누름과
 * 길게누름이 서로 배타여야 하는 키(예: 정지 3초 홀드) 전용이다 - 누를 때 바로
 * 내면 3초를 채우기 전에 짧은 동작이 먼저 실행돼 버린다. 기본 0 = 전 키 누르는
 * 순간 발행. TBD: I14 (배정 확정 시 해당 키 비트를 세운다) */
#ifndef KEYPAD_LONG_ONLY_MASK
#define KEYPAD_LONG_ONLY_MASK     0x00U
#endif

/* LED 래치를 주기적으로 재기록해 FFC 접촉 불량/글리치에서 복구한다. 0 = 끔. */
#ifndef KEYPAD_LED_REFRESH_MS
#define KEYPAD_LED_REFRESH_MS     1000U
#endif

/* 눌렸을 때 U31 핀의 전기적 레벨. 이 보드는 ACTIVE-LOW 이므로 0. */
#ifndef KEYPAD_SW_ACTIVE_HIGH
#define KEYPAD_SW_ACTIVE_HIGH     0U
#endif

/* LED 를 켜는 U32 출력 레벨. 이 보드는 ACTIVE-HIGH (핀 HIGH = 점등) 이므로 1.
 * 2026-09-20 REV02 벤치 확인값 - 평상시 전 LED 소등, 누른 키만 점등. */
#ifndef KEYPAD_LED_ACTIVE_HIGH
#define KEYPAD_LED_ACTIVE_HIGH    1U
#endif

/* 기본 LED 정책: 평상시 전부 소등, 누르고 있는 키의 LED 만 점등.
 * (REV02 벤치에서 관측된 동작과 동일. 표시 사양 C068 이 정해지면 0 으로 바꾸고
 * Keypad_SetLed*() 로 상태 표시를 직접 몰면 된다 - 그때는 mask 비트 1 = 점등이
 * 그대로 성립한다.) TBD: C068 — ★2026-09-22 U32 LED 8개의 기능이 정의되지 않았다(사용자 확인) → 질의 N22
 * (검토서 §17.7). 회신 전에는 이 기본 정책을 바꾸지 않는다. */
#ifndef KEYPAD_LED_FOLLOW_PRESS
#define KEYPAD_LED_FOLLOW_PRESS   1U
#endif

/* ---- 디버거용 스냅샷 -------------------------------------------------- */
typedef struct
{
	uint8_t  pressed;      /* 1 = 눌린 상태(디바운스 완료)                 */
	uint8_t  press_evt;    /* 1 = 소비되지 않은 눌림 이벤트가 대기 중       */
	uint8_t  long_evt;     /* 1 = 소비되지 않은 길게누름 이벤트가 대기 중   */
	uint8_t  long_fired;   /* 1 = 이번 누름에서 길게누름이 이미 발생        */
	uint32_t press_ms;     /* 눌림이 채택된 시각(HAL_GetTick 기준)          */
	uint32_t press_evt_ms; /* press_evt 가 생긴 시각(TTL 계산용)            */
	uint32_t long_evt_ms;  /* long_evt 가 생긴 시각(TTL 계산용)             */
	uint32_t press_cnt;    /* 누적 누름 횟수(진단)                          */
} keypad_btn_t;

extern volatile keypad_btn_t g_keypad_btn[KEYPAD_KEY_COUNT];
extern volatile uint8_t      g_keypad_mask;      /* bit i = SW(i+1) 눌림     */
extern volatile uint8_t      g_keypad_led_mask;  /* bit i = LED(i+1) 점등    */
extern volatile uint32_t     g_keypad_bus_err;   /* I2C 실패 누적(진단)      */

/* ---- 수명주기 -------------------------------------------------------- */
/* U32(전 출력, LED 정책 초기값) -> U31(전 입력, 극성반전) 순으로 올리고 현재
 * 눌림 상태로 디바운스를 시드한다(부팅 시 눌려 있던 키가 가짜 엣지를 내지
 * 않게). MX_I2C1_Init() 이후에 1회 호출. 두 확장기가 모두 응답해야 HAL_OK. */
HAL_StatusTypeDef Keypad_Init(void);
uint8_t           Keypad_IsPresent(void);   /* 1 = U31·U32 둘 다 ACK        */

/* 한 사이클: (INT 또는 주기 도달 시) U31 1회 읽기 -> 디바운스 -> 엣지/길게누름
 * 래치 -> TTL 만료 정리 -> 변경 시 U32 1회 쓰기. 모든 모드에서 호출한다. */
void Keypad_Tick(uint32_t now_ms);

/* ---- 입력 조회 -------------------------------------------------------- */
uint8_t Keypad_GetMask(void);                  /* 눌림 마스크               */
uint8_t Keypad_IsPressed(uint8_t key);         /* 현재 눌림 여부            */
uint32_t Keypad_HeldMs(uint8_t key, uint32_t now_ms); /* 누른 채 경과(ms)   */

/* 이벤트 소비(one-shot). 1을 반환하면 해당 이벤트를 "가져간" 것이고 래치는
 * 지워진다. 소비자는 하나여야 한다 - 두 모듈이 같은 키를 소비하면 먼저 부른
 * 쪽만 받는다. */
uint8_t Keypad_TakePress(uint8_t key);         /* 눌림 엣지                 */
uint8_t Keypad_TakeLongPress(uint8_t key);     /* KEYPAD_LONG_PRESS_MS 도달 */

/* 대기 중인 모든 이벤트를 버린다(모드 전환·정지 직후 등). */
void    Keypad_FlushEvents(void);

/* ---- LED -------------------------------------------------------------- */
/* mask/level 의 비트 1 = 점등(전기적 극성은 KEYPAD_LED_ACTIVE_HIGH 가 흡수).
 * KEYPAD_LED_FOLLOW_PRESS 가 1이면 Tick 이 매 사이클 래치를 덮어쓰므로 아래
 * 설정은 유지되지 않는다. 상태 표시로 쓰려면 그 매크로를 0 으로. */
uint8_t           Keypad_GetLedMask(void);
HAL_StatusTypeDef Keypad_SetLed(uint8_t key, uint8_t on);
HAL_StatusTypeDef Keypad_SetLedMask(uint8_t mask);

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_KEYPAD_H_ */
