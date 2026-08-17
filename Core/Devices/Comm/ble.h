#ifndef DEVICES_BLE_H_
#define DEVICES_BLE_H_

#include "main.h"
#include "usart.h"
#include "gpio_ctrl.h"
#include <stdint.h>
#include <stddef.h>

/* =========================================================================
 * ★BLE 드라이버 일괄 비활성화 스위치 (2026-08-17)
 *
 * 0 = 비활성(현재). ble.h/ble.c 전체가 컴파일에서 빠진다.
 * 1 = 활성. 아래 원본 드라이버가 그대로 살아난다.
 *
 * 왜 껐는가:
 *   UART5 는 R0 프로토콜 전송로(Core/Interface/uart_ctrl.c)가 소유한다. HAL 은
 *   포트당 RX 를 하나만 무장할 수 있으므로 ble.c 와 uart_ctrl 이 동시에 데이터
 *   경로를 가질 수 없다. BLE 모듈은 BYPASS 모드(o_BLE_MODE=LOW)에서 HOST UART 를
 *   원격 피어로 그대로 통과시키므로, 모듈 드라이버 없이도 UART5 = 앱 링크가 된다.
 *   BYPASS 선택에 필요한 것은 o_BLE_MODE 를 LOW 로 두는 것뿐이고, 그것은 main.c
 *   에서 gpio_ctrl 로 직접 처리한다(ble.c 불필요).
 *
 * 다시 켜는 방법(AT 커맨드로 모듈을 설정해야 할 때):
 *   1) 여기 BLE_ENABLE 을 1 로
 *   2) main.c : gpio_ctrl_off(GPIO_OUT_BLE_MODE) 대신 BLE_Init() 호출.
 *      단, 그 전에 UartCtrl_DeInit() 로 UART5 RX 를 반납해야 한다.
 *   3) usart.c : HAL_UART_RxCplt/ErrorCallback 의 UART5 분기를
 *      UartCtrl_*ISR() -> BLE_UART_*ISR() 로 되돌린다.
 *   (즉 BLE_ENABLE=1 로만 두고 위 2~3 을 안 하면 UART5 는 여전히 uart_ctrl 것이고
 *    ble.c 의 송수신 함수는 동작하지 않는다. 컴파일은 된다.)
 * ========================================================================= */
#ifndef BLE_ENABLE
#define BLE_ENABLE      0
#endif

#if BLE_ENABLE

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * ble - CHIPSEN BoT-nLE521 Bluetooth LE module driver (BYPASS-centric).
 *
 * Transport: UART5 (huart5), PC12 = uart5_BLE_TX, PD2 = uart5_BLE_RX.
 * Control lines (only two of the module's five control GPIO are wired on this
 * board), accessed through gpio_ctrl:
 *   o_BLE_MODE   (PD4, GPIO_OUT_BLE_MODE)  -> module "AT Command / BYPASS" pin
 *   i_BLE_STATUS (PD3, GPIO_IN_BLE_STATUS) -> module "Connection Status" pin
 *
 * Serial conventions (BoT-nLE521 AT COMMAND USER MANUAL v2.2.2):
 *   Baud rate      : 9600 8N1  (matches MX_UART5_Init() after the 9600 change)
 *   Line terminator: CR only ("\r", 0x0D)   <-- note: NOT CR-LF like ESP-AT
 *   Result codes   : "+OK" / "+ERROR"; power-up notify is "+READY".
 *
 * Operating modes (manual s3.2), selected by the o_BLE_MODE pin *while
 * CONNECTED*:
 *   BYPASS (default) : o_BLE_MODE LOW  -- HOST UART data is passed straight
 *                      through to the remote peer. This is the mode this board
 *                      runs in, so BLE_Init() drives o_BLE_MODE LOW.
 *   AT COMMAND       : o_BLE_MODE HIGH -- while connected only *query* commands
 *                      are honoured (e.g. AT+INFO?, AT+CONNRSSI?).
 * When NOT connected the module is always in AT-COMMAND mode regardless of the
 * pin, so configuration commands are issued in the disconnected/advertising
 * state.
 *
 * Connection status is a hardware line, read directly (no polling of the
 * module needed): i_BLE_STATUS HIGH = connected, LOW = disconnected.
 *
 * RX path mirrors wifi.c: UART5 bytes are buffered under interrupt (UART5_IRQ
 * -> HAL_UART_RxCpltCallback -> BLE_UART_RxCpltISR) into a private ring. In
 * BYPASS mode the ring carries raw peer data (BLE_Read); in AT mode it carries
 * CR-terminated response lines (BLE_ReadLine / the command helpers).
 * ---------------------------------------------------------------------- */

typedef enum
{
	BLE_MODE_BYPASS = 0,   /* o_BLE_MODE LOW  - transparent passthrough (default) */
	BLE_MODE_AT     = 1,   /* o_BLE_MODE HIGH - AT command (query-only if connected) */
} ble_mode_t;

typedef enum
{
	BLE_OK = 0,       /* "+OK" received                          */
	BLE_ERROR,        /* "+ERROR" received                       */
	BLE_TIMEOUT,      /* no result code before timeout           */
	BLE_INVALID,      /* bad argument                            */
} ble_status_t;

/* ---- Lifecycle ------------------------------------------------------- */
void BLE_Init(void);                 /* force BYPASS, arm RX IT             */

/* ---- Mode + status (hardware GPIO, see gpio_ctrl) -------------------- */
void       BLE_SetMode(ble_mode_t mode); /* drive o_BLE_MODE                */
ble_mode_t BLE_GetMode(void);            /* read back the o_BLE_MODE latch  */
uint8_t    BLE_IsConnected(void);        /* i_BLE_STATUS: 1 = connected     */

/* ---- BYPASS data path (transparent to the connected peer) ------------ */
/* Send raw bytes to the remote peer. Only meaningful in BYPASS mode while
 * connected; returns immediately (blocking UART TX). */
void BLE_Send(const uint8_t *data, uint16_t len);
void BLE_SendString(const char *str);

/* Pull up to max bytes of received peer data from the ring into buf.
 * Returns the number of bytes copied (0 if none buffered). Non-blocking. */
uint16_t BLE_Read(uint8_t *buf, uint16_t max);

/* ---- AT command path (issue while disconnected, or query-only if not) - */
/* Send a raw AT command (CR appended) and wait for "+OK"/"+ERROR". Response
 * body lines before the result code are copied into resp if provided. */
ble_status_t BLE_SendAT(const char *cmd, char *resp, size_t resp_sz,
                        uint32_t timeout_ms);

ble_status_t BLE_GetVersion(char *buf, size_t sz, uint32_t timeout_ms); /* AT+VER?  */
ble_status_t BLE_GetInfo(char *buf, size_t sz, uint32_t timeout_ms);    /* AT+INFO? */
ble_status_t BLE_Disconnect(uint32_t timeout_ms);                       /* AT+DISCONNECT */

/* Pull one CR-terminated line (CR/LF stripped) from the ring into line.
 * Returns length, or -1 if no complete line before timeout_ms. */
int  BLE_ReadLine(char *line, size_t sz, uint32_t timeout_ms);
void BLE_FlushRx(void);

/* ---- ISR hooks (called only from usart.c dispatcher) ----------------- */
void BLE_UART_RxCpltISR(void);   /* one byte arrived on UART5           */
void BLE_UART_ErrorISR(void);    /* UART5 error: re-arm RX              */

#ifdef __cplusplus
}
#endif

#endif /* BLE_ENABLE */

#endif /* DEVICES_BLE_H_ */
