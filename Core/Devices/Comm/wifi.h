#ifndef DEVICES_WIFI_H_
#define DEVICES_WIFI_H_

#include "main.h"
#include "usart.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * wifi - Espressif ESP32-C3-WROOM-02 Wi-Fi co-processor driver (ESP-AT).
 *
 * Transport: UART4 (huart4), PC10 = uart4_WIFI_TX, PC11 = uart4_WIF_RX.
 * The ESP32-C3 is assumed to run stock ESP-AT firmware and is driven purely
 * over the serial AT command set:
 *   https://docs.espressif.com/projects/esp-at/en/latest/esp32c3/AT_Command_Set/index.html
 *
 * ESP-AT serial conventions (differ from the BLE module, see ble.h):
 *   Baud rate      : 115200 8N1  (matches MX_UART4_Init())
 *   Line terminator: CR-LF ("\r\n") on both command and response lines
 *   Result codes   : "OK" / "ERROR", plus "ready" after reset/boot and
 *                    "busy p..." while a previous command is still running.
 *
 * RX path: bytes are received one-at-a-time under interrupt (UART4_IRQHandler
 * -> HAL_UART_RxCpltCallback -> WiFi_UART_RxCpltISR) into a private ring
 * buffer, so unsolicited URCs (e.g. "WIFI GOT IP", "+IPD") are never lost
 * between polls. The single shared HAL_UART_RxCpltCallback dispatcher lives in
 * usart.c (USER CODE) and forwards UART4 events here.
 *
 * The command helpers below are blocking with a timeout: they transmit, then
 * drain response lines from the ring buffer until a final result code arrives
 * or the timeout elapses. Fine for bring-up / configuration; move to an
 * event-driven state machine if the application needs concurrent throughput.
 * ---------------------------------------------------------------------- */

/* AT command mode select (mirrors ESP-AT AT+CWMODE values). */
typedef enum
{
	WIFI_MODE_STATION  = 1,  /* station (join an AP)            */
	WIFI_MODE_SOFTAP   = 2,  /* soft-AP (host an AP)            */
	WIFI_MODE_STA_AP   = 3,  /* station + soft-AP              */
} wifi_mode_t;

typedef enum
{
	WIFI_OK = 0,      /* final "OK" received                    */
	WIFI_ERROR,       /* final "ERROR" received                 */
	WIFI_TIMEOUT,     /* no result code before timeout          */
	WIFI_INVALID,     /* bad argument                           */
} wifi_status_t;

/* ---- Lifecycle ------------------------------------------------------- */
void          WiFi_Init(void);                 /* arm RX IT; does not touch the module */
wifi_status_t WiFi_Reset(uint32_t timeout_ms); /* AT+RST, waits for "ready"            */

/* ---- Low-level AT plumbing ------------------------------------------- */
/* Send a raw command (terminator appended automatically) and wait for the
 * final OK/ERROR result code. Response body lines are discarded. */
wifi_status_t WiFi_SendCommand(const char *cmd, uint32_t timeout_ms);

/* Same, but copy the response body (everything before the result code) into
 * resp[0..resp_sz-1], NUL-terminated. Pass NULL/0 to ignore the body. */
wifi_status_t WiFi_Query(const char *cmd, char *resp, size_t resp_sz,
                         uint32_t timeout_ms);

/* ---- Common station workflow ----------------------------------------- */
wifi_status_t WiFi_Test(uint32_t timeout_ms);              /* bare "AT" ping        */
wifi_status_t WiFi_SetEcho(uint8_t on, uint32_t timeout_ms);/* ATE1 / ATE0          */
wifi_status_t WiFi_GetVersion(char *buf, size_t sz, uint32_t timeout_ms); /* AT+GMR  */
wifi_status_t WiFi_SetMode(wifi_mode_t mode, uint32_t timeout_ms);        /* AT+CWMODE */
wifi_status_t WiFi_ConnectAP(const char *ssid, const char *pass,
                             uint32_t timeout_ms);          /* AT+CWJAP             */
wifi_status_t WiFi_DisconnectAP(uint32_t timeout_ms);       /* AT+CWQAP            */
wifi_status_t WiFi_GetIP(char *buf, size_t sz, uint32_t timeout_ms);      /* AT+CIPSTA? */

/* ---- Minimal TCP client ---------------------------------------------- */
wifi_status_t WiFi_TcpOpen(const char *host, uint16_t port, uint32_t timeout_ms); /* AT+CIPSTART */
wifi_status_t WiFi_TcpSend(const uint8_t *data, uint16_t len, uint32_t timeout_ms); /* AT+CIPSEND */
wifi_status_t WiFi_TcpClose(uint32_t timeout_ms);                                 /* AT+CIPCLOSE */

/* ---- RX access ------------------------------------------------------- */
/* Pull one CRLF-terminated line (CR/LF stripped) from the RX ring into
 * line[0..sz-1]. Returns line length, or -1 if no complete line is buffered
 * before timeout_ms. Use to consume URCs from the main loop. */
int WiFi_ReadLine(char *line, size_t sz, uint32_t timeout_ms);

void WiFi_FlushRx(void);   /* discard everything currently buffered */

/* ---- ISR hooks (called only from usart.c dispatcher) ----------------- */
void WiFi_UART_RxCpltISR(void);   /* one byte arrived on UART4        */
void WiFi_UART_ErrorISR(void);    /* UART4 error: re-arm RX           */

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_WIFI_H_ */
