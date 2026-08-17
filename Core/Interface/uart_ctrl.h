#ifndef INTERFACE_UART_CTRL_H_
#define INTERFACE_UART_CTRL_H_

#include "main.h"
#include "usart.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * uart_ctrl - UART5 인터럽트 송수신 계층 (프로토콜 전송로).
 *
 * 역할: "바이트 스트림"만 책임진다. 패킷 구조(STX/DLE/ETX, R/W/M, CMD)는
 *       Core/Devices/Comm/protocol_r0.c 가 담당하고, 이 파일은 그 아래에서
 *       링버퍼 + 인터럽트로 바이트를 넣고 빼는 일만 한다.
 *
 * 하드웨어(usart.c / R1 넷리스트):
 *   UART5  PC12 = uart5_BLE_TX, PD2 = uart5_BLE_RX, 9600 8N1, NVIC prio 5.
 *   물리적으로는 CHIPSEN BoT-nLE521 BLE 모듈에 연결되어 있고, 모듈은 BYPASS
 *   모드(o_BLE_MODE=LOW)에서 HOST UART <-> 원격 피어를 그대로 통과시킨다.
 *   즉 "UART5 로 프로토콜 패킷을 쓴다" = "앱(BLE 피어)에게 패킷을 보낸다".
 *
 * ★UART5 소유권 (중요)
 *   ble.c 도 같은 UART5 에 자기 링버퍼와 HAL_UART_Receive_IT 를 걸도록 작성되어
 *   있다. HAL 의 RX 는 한 번에 하나만 무장되므로 두 모듈이 동시에 소유할 수 없다.
 *   따라서:
 *     - RX/TX 데이터 경로 소유자 = 이 파일(uart_ctrl).
 *     - usart.c 의 HAL_UART_RxCplt/TxCplt/ErrorCallback 디스패처가 UART5 를
 *       BLE_UART_*ISR() 대신 UartCtrl_*ISR() 로 보낸다.
 *     - main.c 는 BLE_Init() 대신 BLE_SetMode(BLE_MODE_BYPASS) + UartCtrl_Init()
 *       를 호출한다(모듈을 BYPASS 로 두는 GPIO 설정만 ble.c 에서 재사용).
 *   ble.c/ble.h 자체는 손대지 않았다. AT 커맨드로 모듈을 설정할 일이 생기면
 *   그때 UartCtrl 를 정지(UartCtrl_DeInit)시키고 BLE_Init() 로 넘기면 된다.
 *
 * RX: 1바이트 HAL_UART_Receive_IT -> UartCtrl_RxCpltISR() 이 링에 push 하고 재무장.
 *     소비자는 태스크에서 UartCtrl_Read()/ReadByte() 로 pop 한다.
 *     링이 꽉 차면 새 바이트를 버리고 rx_drop 을 올린다(오래된 프레임 보존).
 * TX: UartCtrl_Send() 가 링에 넣고 "킥"만 한다. 실제 송신은
 *     HAL_UART_Transmit_IT(1바이트) -> UartCtrl_TxCpltISR() 이 다음 바이트를
 *     이어서 물리는 방식이라 호출자는 절대 블록되지 않는다(9600 baud 에서
 *     30바이트 프레임 = 약 31ms. 블로킹 송신이면 100ms 태스크가 흔들린다).
 *     tail 은 "송신 완료 확인" 시점에만 전진하므로, Transmit_IT 무장이 실패하면
 *     바이트를 잃지 않고 다음 킥에서 재시도한다.
 *
 * 스레드 안전:
 *   UartCtrl_Send()/Read() 는 각각 단일 생산자/단일 소비자 전제(현재는
 *   protocol_r0 만 호출). 링 인덱스 갱신 구간은 PRIMASK 임계구역으로 ISR 과
 *   분리한다. 여러 태스크에서 Send 할 계획이 생기면 뮤텍스를 추가할 것.
 * ========================================================================== */

/* ---- 버퍼 크기(2의 거듭제곱 - 마스크 연산) ------------------------------- */
#ifndef UART_CTRL_RX_BUFSZ
#define UART_CTRL_RX_BUFSZ      256U
#endif
/* TX 링은 "한 주기 모니터링 버스트"가 통째로 들어가야 한다. 프레임은 부분 적재를
 * 하지 않으므로(UartCtrl_SendFrame), 링이 작으면 큰 패킷이 통째로 버려진다.
 * 전 패킷(STATUS+시나리오+SENSOR+MOTOR+OUTPUT+JUNGJI) 최악 버스트 = 약 330B
 * 이므로 512(가용 511)로 잡는다. 2의 거듭제곱 유지. */
#ifndef UART_CTRL_TX_BUFSZ
#define UART_CTRL_TX_BUFSZ      512U
#endif

/* ---- 관찰 카운터 (디버거 watch) ----------------------------------------- */
typedef struct
{
	volatile uint32_t rx_bytes;   /* 링에 적재된 누적 수신 바이트            */
	volatile uint32_t tx_bytes;   /* 송신 완료된 누적 바이트                 */
	volatile uint16_t rx_drop;    /* RX 링 풀로 버린 바이트                  */
	volatile uint16_t tx_drop;    /* TX 링 풀로 못 넣은 바이트               */
	volatile uint16_t err_cnt;    /* HAL UART 에러 콜백 횟수(ORE/NE/FE/PE)   */
	volatile uint16_t tx_arm_err; /* Transmit_IT 무장 실패(다음 킥에서 재시도)*/
	volatile uint16_t rx_arm_err; /* Receive_IT 무장 실패(다음 Read에서 재시도)*/
	volatile uint8_t  tx_busy;    /* 1 = 바이트 1개가 송신 중                */
	volatile uint8_t  rx_armed;   /* 1 = RX 인터럽트 무장 상태(0이면 수신 정지)*/
	volatile uint8_t  ready;      /* 1 = UartCtrl_Init() 완료                */
} UartCtrlStat;

extern UartCtrlStat g_uart_ctrl;

/* ---- 라이프사이클 -------------------------------------------------------- */
/* 링 초기화 + RX 인터럽트 무장. MX_UART5_Init() 이후에 호출할 것.
 * (스케줄러 시작 전 main.c 에서 호출) */
void     UartCtrl_Init(void);
/* 송수신 중단 + RX 무장 해제. UART5 를 ble.c 에 되돌려줄 때만 사용. */
void     UartCtrl_DeInit(void);

/* ---- 송신 (send) --------------------------------------------------------- */
/* 링에 적재하고 즉시 반환한다. 반환값 = 실제로 적재된 바이트 수(공간이 부족하면
 * len 보다 작다; 부족분은 tx_drop 으로 집계). 부분 적재를 원하지 않으면
 * UartCtrl_TxFree() 로 먼저 여유를 확인할 것 - 프로토콜 계층은 프레임이 쪼개져
 * 나가면 안 되므로 그렇게 쓴다. */
uint16_t UartCtrl_Send(const uint8_t *data, uint16_t len);
uint16_t UartCtrl_SendString(const char *str);
/* 프레임 단위 송신: 여유가 len 이상일 때만 통째로 적재한다.
 * 1 = 적재됨, 0 = 공간 부족(아무것도 넣지 않음, tx_drop += len). */
uint8_t  UartCtrl_SendFrame(const uint8_t *data, uint16_t len);

/* ---- 수신 (read) --------------------------------------------------------- */
uint16_t UartCtrl_Read(uint8_t *buf, uint16_t max);  /* pop N, 반환=복사 수 */
uint8_t  UartCtrl_ReadByte(uint8_t *out);            /* 1 = 1바이트 얻음    */

/* ---- 상태 --------------------------------------------------------------- */
uint16_t UartCtrl_RxCount(void);   /* RX 링에 대기 중인 바이트 수           */
uint16_t UartCtrl_TxPending(void); /* TX 링에서 아직 나가지 않은 바이트 수  */
uint16_t UartCtrl_TxFree(void);    /* TX 링 여유 바이트 수                  */
uint8_t  UartCtrl_TxBusy(void);    /* 1 = 송신 진행 중(링 비었어도 마지막 1B)*/
void     UartCtrl_FlushRx(void);
void     UartCtrl_FlushTx(void);

/* ---- 요청/응답 (request) -------------------------------------------------
 * tx 를 보낸 뒤 응답 바이트를 모아 rx 에 담는다. 반환 = 수신 바이트 수.
 *   timeout_ms : 첫 바이트를 기다리는 전체 상한
 *   idle_ms    : 첫 바이트 수신 후, 이 시간 동안 추가 바이트가 없으면 종료
 *                (0 이면 rxmax 가 차거나 timeout_ms 만료까지 계속 모은다)
 * HAL_GetTick() 기반 바쁜대기다(ble.c 와 동일 기조). 100ms 센서 태스크에서
 * 부르면 그 시간만큼 다른 폴링이 밀리므로, 시나리오 운전 중에는 쓰지 말고
 * 벤치/브링업 용도로만 쓸 것. 평시 수신은 UartCtrl_Read() 논블로킹 경로다. */
uint16_t UartCtrl_Request(const uint8_t *tx, uint16_t txlen,
                          uint8_t *rx, uint16_t rxmax,
                          uint32_t timeout_ms, uint32_t idle_ms);

/* ---- ISR 훅 (usart.c 디스패처에서만 호출) ------------------------------- */
void UartCtrl_RxCpltISR(void);   /* UART5 1바이트 수신 완료                 */
void UartCtrl_TxCpltISR(void);   /* UART5 1바이트 송신 완료                 */
void UartCtrl_ErrorISR(void);    /* UART5 에러: RX 재무장                   */

#ifdef __cplusplus
}
#endif

#endif /* INTERFACE_UART_CTRL_H_ */
