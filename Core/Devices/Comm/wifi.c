#include "wifi.h"
#include <string.h>
#include <stdio.h>

/* ESP32-C3 (ESP-AT) over UART4. See wifi.h for the wiring and protocol notes.
 *
 * Commands and responses are ASCII terminated with CR-LF. This driver keeps a
 * private single-producer/single-consumer RX ring filled from the UART4 RX
 * interrupt; the command helpers are blocking-with-timeout line readers. */

#define WIFI_UART        (&huart4)
#define WIFI_RX_BUFSZ    512U     /* power of two keeps the mask cheap */
#define WIFI_RX_MASK     (WIFI_RX_BUFSZ - 1U)
#define WIFI_LINE_MAX    128U     /* longest response line we assemble */

/* CR-LF: ESP-AT command / response terminator. */
static const char WIFI_EOL[] = "\r\n";

/* ---- RX ring (head = ISR producer, tail = main consumer) ------------- */
static volatile uint8_t  rx_buf[WIFI_RX_BUFSZ];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static uint8_t           rx_byte;   /* HAL landing byte for Receive_IT */

/* ---------------------------------------------------------------------- */
/* RX ring helpers                                                         */
/* ---------------------------------------------------------------------- */
static void rx_arm(void)
{
	/* Re-arm single-byte interrupt reception. Ignore the return: if the
	 * peripheral is momentarily busy the error path re-arms us again. */
	(void)HAL_UART_Receive_IT(WIFI_UART, &rx_byte, 1);
}

static int rx_pop(uint8_t *out)
{
	if (rx_head == rx_tail)
		return 0;                       /* empty */
	*out = rx_buf[rx_tail];
	rx_tail = (uint16_t)((rx_tail + 1U) & WIFI_RX_MASK);
	return 1;
}

/* ---------------------------------------------------------------------- */
/* ISR hooks - invoked from the shared dispatcher in usart.c               */
/* ---------------------------------------------------------------------- */
void WiFi_UART_RxCpltISR(void)
{
	uint16_t next = (uint16_t)((rx_head + 1U) & WIFI_RX_MASK);
	if (next != rx_tail)                 /* drop on overflow, never block */
	{
		rx_buf[rx_head] = rx_byte;
		rx_head = next;
	}
	rx_arm();
}

void WiFi_UART_ErrorISR(void)
{
	/* ORE/FE/NE/PE: HAL has aborted the IT reception, so re-arm it. The
	 * offending byte is discarded; the next AT round-trip re-syncs. */
	rx_arm();
}

/* ---------------------------------------------------------------------- */
/* Lifecycle                                                               */
/* ---------------------------------------------------------------------- */
void WiFi_Init(void)
{
	rx_head = 0;
	rx_tail = 0;
	rx_arm();
}

void WiFi_FlushRx(void)
{
	rx_tail = rx_head;
}

/* ---------------------------------------------------------------------- */
/* TX helpers                                                              */
/* ---------------------------------------------------------------------- */
static void wifi_write(const uint8_t *data, uint16_t len)
{
	HAL_UART_Transmit(WIFI_UART, (uint8_t *)data, len, HAL_MAX_DELAY);
}

/* ---------------------------------------------------------------------- */
/* Line reader                                                             */
/* ---------------------------------------------------------------------- */
int WiFi_ReadLine(char *line, size_t sz, uint32_t timeout_ms)
{
	uint32_t start = HAL_GetTick();
	size_t   n     = 0;
	uint8_t  c;

	if (line == NULL || sz == 0)
		return -1;

	for (;;)
	{
		if (rx_pop(&c))
		{
			if (c == '\n')               /* end of a CR-LF line */
			{
				line[n] = '\0';
				return (int)n;           /* may be 0 for a blank line */
			}
			if (c == '\r')
				continue;                /* strip CR, wait for LF */
			if (n < sz - 1U)
				line[n++] = (char)c;
			/* else: overflow -> keep scanning until EOL, line stays clipped */
		}
		else if ((HAL_GetTick() - start) >= timeout_ms)
		{
			return -1;                   /* no complete line in time */
		}
	}
}

/* Read response lines until a final result code, optionally capturing the
 * body. "OK" -> WIFI_OK, "ERROR"/"FAIL" -> WIFI_ERROR. */
static wifi_status_t wifi_collect(char *resp, size_t resp_sz, uint32_t timeout_ms)
{
	char     line[WIFI_LINE_MAX];
	uint32_t start = HAL_GetTick();
	size_t   used  = 0;

	if (resp && resp_sz)
		resp[0] = '\0';

	for (;;)
	{
		uint32_t elapsed = HAL_GetTick() - start;
		if (elapsed >= timeout_ms)
			return WIFI_TIMEOUT;

		int len = WiFi_ReadLine(line, sizeof(line), timeout_ms - elapsed);
		if (len < 0)
			return WIFI_TIMEOUT;
		if (len == 0)
			continue;                    /* blank separator line */

		if (strcmp(line, "OK") == 0)
			return WIFI_OK;
		if (strcmp(line, "ERROR") == 0 || strcmp(line, "FAIL") == 0)
			return WIFI_ERROR;
		if (strncmp(line, "busy", 4) == 0)
			continue;                    /* module still chewing; keep waiting */

		/* Body line: append "<line>\n" to the caller's buffer if room. */
		if (resp && resp_sz)
		{
			size_t l = strlen(line);
			if (used + l + 1U < resp_sz)
			{
				memcpy(resp + used, line, l);
				used += l;
				resp[used++] = '\n';
				resp[used]   = '\0';
			}
		}
	}
}

/* ---------------------------------------------------------------------- */
/* Core AT plumbing                                                        */
/* ---------------------------------------------------------------------- */
wifi_status_t WiFi_Query(const char *cmd, char *resp, size_t resp_sz,
                         uint32_t timeout_ms)
{
	if (cmd == NULL)
		return WIFI_INVALID;

	WiFi_FlushRx();                       /* drop stale URCs before the round-trip */
	wifi_write((const uint8_t *)cmd, (uint16_t)strlen(cmd));
	wifi_write((const uint8_t *)WIFI_EOL, 2);

	return wifi_collect(resp, resp_sz, timeout_ms);
}

wifi_status_t WiFi_SendCommand(const char *cmd, uint32_t timeout_ms)
{
	return WiFi_Query(cmd, NULL, 0, timeout_ms);
}

/* ---------------------------------------------------------------------- */
/* High-level station workflow                                             */
/* ---------------------------------------------------------------------- */
wifi_status_t WiFi_Test(uint32_t timeout_ms)
{
	return WiFi_SendCommand("AT", timeout_ms);
}

wifi_status_t WiFi_Reset(uint32_t timeout_ms)
{
	char     line[WIFI_LINE_MAX];
	uint32_t start = HAL_GetTick();

	WiFi_FlushRx();
	wifi_write((const uint8_t *)"AT+RST", 6);
	wifi_write((const uint8_t *)WIFI_EOL, 2);

	/* After reset the module reboots and finally emits "ready". */
	for (;;)
	{
		uint32_t elapsed = HAL_GetTick() - start;
		if (elapsed >= timeout_ms)
			return WIFI_TIMEOUT;
		int len = WiFi_ReadLine(line, sizeof(line), timeout_ms - elapsed);
		if (len > 0 && strcmp(line, "ready") == 0)
			return WIFI_OK;
	}
}

wifi_status_t WiFi_SetEcho(uint8_t on, uint32_t timeout_ms)
{
	return WiFi_SendCommand(on ? "ATE1" : "ATE0", timeout_ms);
}

wifi_status_t WiFi_GetVersion(char *buf, size_t sz, uint32_t timeout_ms)
{
	return WiFi_Query("AT+GMR", buf, sz, timeout_ms);
}

wifi_status_t WiFi_SetMode(wifi_mode_t mode, uint32_t timeout_ms)
{
	char cmd[16];
	if (mode != WIFI_MODE_STATION && mode != WIFI_MODE_SOFTAP &&
	    mode != WIFI_MODE_STA_AP)
		return WIFI_INVALID;
	snprintf(cmd, sizeof(cmd), "AT+CWMODE=%d", (int)mode);
	return WiFi_SendCommand(cmd, timeout_ms);
}

wifi_status_t WiFi_ConnectAP(const char *ssid, const char *pass,
                             uint32_t timeout_ms)
{
	char cmd[128];
	if (ssid == NULL)
		return WIFI_INVALID;
	/* AT+CWJAP="ssid","pass" -- ESP-AT wants quoted, comma-separated args. */
	snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"",
	         ssid, pass ? pass : "");
	return WiFi_SendCommand(cmd, timeout_ms);
}

wifi_status_t WiFi_DisconnectAP(uint32_t timeout_ms)
{
	return WiFi_SendCommand("AT+CWQAP", timeout_ms);
}

wifi_status_t WiFi_GetIP(char *buf, size_t sz, uint32_t timeout_ms)
{
	return WiFi_Query("AT+CIPSTA?", buf, sz, timeout_ms);
}

/* ---------------------------------------------------------------------- */
/* Minimal single-connection TCP client                                    */
/* ---------------------------------------------------------------------- */
wifi_status_t WiFi_TcpOpen(const char *host, uint16_t port, uint32_t timeout_ms)
{
	char cmd[128];
	if (host == NULL)
		return WIFI_INVALID;
	/* Single-connection mode (AT+CIPMUX=0 is the default). */
	snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%u",
	         host, (unsigned)port);
	return WiFi_SendCommand(cmd, timeout_ms);
}

wifi_status_t WiFi_TcpSend(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
	char     cmd[24];
	char     line[WIFI_LINE_MAX];
	uint32_t start = HAL_GetTick();

	if (data == NULL || len == 0)
		return WIFI_INVALID;

	/* AT+CIPSEND=<len> -> module answers ">" then expects <len> raw bytes. */
	snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u", (unsigned)len);
	WiFi_FlushRx();
	wifi_write((const uint8_t *)cmd, (uint16_t)strlen(cmd));
	wifi_write((const uint8_t *)WIFI_EOL, 2);

	/* Wait for the ">" data prompt. */
	for (;;)
	{
		uint8_t  c;
		uint32_t elapsed = HAL_GetTick() - start;
		if (elapsed >= timeout_ms)
			return WIFI_TIMEOUT;
		if (rx_pop(&c) && c == '>')
			break;
	}

	wifi_write(data, len);

	/* ESP-AT confirms with "SEND OK" (followed by OK on some versions). */
	for (;;)
	{
		uint32_t elapsed = HAL_GetTick() - start;
		if (elapsed >= timeout_ms)
			return WIFI_TIMEOUT;
		int l = WiFi_ReadLine(line, sizeof(line), timeout_ms - elapsed);
		if (l < 0)
			return WIFI_TIMEOUT;
		if (strcmp(line, "SEND OK") == 0 || strcmp(line, "OK") == 0)
			return WIFI_OK;
		if (strcmp(line, "ERROR") == 0 || strcmp(line, "SEND FAIL") == 0)
			return WIFI_ERROR;
	}
}

wifi_status_t WiFi_TcpClose(uint32_t timeout_ms)
{
	return WiFi_SendCommand("AT+CIPCLOSE", timeout_ms);
}
