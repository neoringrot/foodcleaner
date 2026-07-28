#include "ble.h"
#include <string.h>

/* CHIPSEN BoT-nLE521 over UART5. See ble.h for the wiring, mode model and
 * protocol notes. BYPASS is the default and only mode this board runs in for
 * data; AT commands are used for bring-up/configuration while disconnected. */

#define BLE_UART        (&huart5)
#define BLE_RX_BUFSZ    256U      /* power of two -> cheap mask */
#define BLE_RX_MASK     (BLE_RX_BUFSZ - 1U)
#define BLE_LINE_MAX    96U

/* BoT-nLE521 terminates every command/response line with a bare CR (0x0D). */
static const char BLE_EOL = '\r';

/* ---- RX ring (head = ISR producer, tail = main consumer) ------------- */
static volatile uint8_t  rx_buf[BLE_RX_BUFSZ];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static uint8_t           rx_byte;   /* HAL landing byte for Receive_IT */

/* ---------------------------------------------------------------------- */
/* RX ring helpers                                                         */
/* ---------------------------------------------------------------------- */
static void rx_arm(void)
{
	(void)HAL_UART_Receive_IT(BLE_UART, &rx_byte, 1);
}

static int rx_pop(uint8_t *out)
{
	if (rx_head == rx_tail)
		return 0;
	*out = rx_buf[rx_tail];
	rx_tail = (uint16_t)((rx_tail + 1U) & BLE_RX_MASK);
	return 1;
}

/* ---------------------------------------------------------------------- */
/* ISR hooks - invoked from the shared dispatcher in usart.c               */
/* ---------------------------------------------------------------------- */
void BLE_UART_RxCpltISR(void)
{
	uint16_t next = (uint16_t)((rx_head + 1U) & BLE_RX_MASK);
	if (next != rx_tail)
	{
		rx_buf[rx_head] = rx_byte;
		rx_head = next;
	}
	rx_arm();
}

void BLE_UART_ErrorISR(void)
{
	rx_arm();
}

/* ---------------------------------------------------------------------- */
/* Lifecycle                                                               */
/* ---------------------------------------------------------------------- */
void BLE_Init(void)
{
	rx_head = 0;
	rx_tail = 0;
	/* BYPASS is the board default: drive the AT/BYPASS pin LOW. */
	BLE_SetMode(BLE_MODE_BYPASS);
	rx_arm();
}

void BLE_FlushRx(void)
{
	rx_tail = rx_head;
}

/* ---------------------------------------------------------------------- */
/* Mode + connection status (hardware GPIO via gpio_ctrl)                  */
/* ---------------------------------------------------------------------- */
void BLE_SetMode(ble_mode_t mode)
{
	/* o_BLE_MODE: HIGH = AT command, LOW = BYPASS (manual s4.2.1). */
	if (mode == BLE_MODE_AT)
		gpio_ctrl_on(GPIO_OUT_BLE_MODE);
	else
		gpio_ctrl_off(GPIO_OUT_BLE_MODE);
}

ble_mode_t BLE_GetMode(void)
{
	return gpio_ctrl_is_on(GPIO_OUT_BLE_MODE) ? BLE_MODE_AT : BLE_MODE_BYPASS;
}

uint8_t BLE_IsConnected(void)
{
	/* i_BLE_STATUS: HIGH = connected (manual s4.2.2). */
	return gpio_ctrl_read(GPIO_IN_BLE_STATUS);
}

/* ---------------------------------------------------------------------- */
/* BYPASS data path                                                        */
/* ---------------------------------------------------------------------- */
static void ble_write(const uint8_t *data, uint16_t len)
{
	HAL_UART_Transmit(BLE_UART, (uint8_t *)data, len, HAL_MAX_DELAY);
}

void BLE_Send(const uint8_t *data, uint16_t len)
{
	if (data && len)
		ble_write(data, len);
}

void BLE_SendString(const char *str)
{
	if (str)
		ble_write((const uint8_t *)str, (uint16_t)strlen(str));
}

uint16_t BLE_Read(uint8_t *buf, uint16_t max)
{
	uint16_t n = 0;
	uint8_t  c;
	if (buf == NULL)
		return 0;
	while (n < max && rx_pop(&c))
		buf[n++] = c;
	return n;
}

/* ---------------------------------------------------------------------- */
/* AT command path                                                         */
/* ---------------------------------------------------------------------- */
int BLE_ReadLine(char *line, size_t sz, uint32_t timeout_ms)
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
			if (c == '\r')               /* end of line (BoT terminator) */
			{
				line[n] = '\0';
				return (int)n;
			}
			if (c == '\n')
				continue;                /* tolerate stray LF */
			if (n < sz - 1U)
				line[n++] = (char)c;
		}
		else if ((HAL_GetTick() - start) >= timeout_ms)
		{
			return -1;
		}
	}
}

/* Collect response lines until "+OK"/"+ERROR", optionally capturing the body. */
static ble_status_t ble_collect(char *resp, size_t resp_sz, uint32_t timeout_ms)
{
	char     line[BLE_LINE_MAX];
	uint32_t start = HAL_GetTick();
	size_t   used  = 0;

	if (resp && resp_sz)
		resp[0] = '\0';

	for (;;)
	{
		uint32_t elapsed = HAL_GetTick() - start;
		if (elapsed >= timeout_ms)
			return BLE_TIMEOUT;

		int len = BLE_ReadLine(line, sizeof(line), timeout_ms - elapsed);
		if (len < 0)
			return BLE_TIMEOUT;
		if (len == 0)
			continue;

		if (strcmp(line, "+OK") == 0)
			return BLE_OK;
		if (strcmp(line, "+ERROR") == 0)
			return BLE_ERROR;

		/* Body line (e.g. the value returned by a query): stash if room. */
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

ble_status_t BLE_SendAT(const char *cmd, char *resp, size_t resp_sz,
                        uint32_t timeout_ms)
{
	if (cmd == NULL)
		return BLE_INVALID;

	BLE_FlushRx();
	ble_write((const uint8_t *)cmd, (uint16_t)strlen(cmd));
	ble_write((const uint8_t *)&BLE_EOL, 1);   /* CR terminator */

	return ble_collect(resp, resp_sz, timeout_ms);
}

ble_status_t BLE_GetVersion(char *buf, size_t sz, uint32_t timeout_ms)
{
	return BLE_SendAT("AT+VER?", buf, sz, timeout_ms);
}

ble_status_t BLE_GetInfo(char *buf, size_t sz, uint32_t timeout_ms)
{
	return BLE_SendAT("AT+INFO?", buf, sz, timeout_ms);
}

ble_status_t BLE_Disconnect(uint32_t timeout_ms)
{
	return BLE_SendAT("AT+DISCONNECT", NULL, 0, timeout_ms);
}
