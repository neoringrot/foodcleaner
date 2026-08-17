#include "uart_ctrl.h"
#include <string.h>

/* UART5 인터럽트 송수신. 설계 근거/소유권 규칙은 uart_ctrl.h 헤더 주석 참조. */

#define UART_CTRL_HANDLE   (&huart5)
#define RX_MASK            (UART_CTRL_RX_BUFSZ - 1U)
#define TX_MASK            (UART_CTRL_TX_BUFSZ - 1U)

UartCtrlStat g_uart_ctrl;

/* ---- RX 링: head = ISR 생산자, tail = 태스크 소비자 --------------------- */
static volatile uint8_t  rx_buf[UART_CTRL_RX_BUFSZ];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static uint8_t           rx_byte;    /* HAL 착지 바이트(Receive_IT)          */

/* ---- TX 링: head = 태스크 생산자, tail = ISR 소비자 -------------------- */
static volatile uint8_t  tx_buf[UART_CTRL_TX_BUFSZ];
static volatile uint16_t tx_head;
static volatile uint16_t tx_tail;
static uint8_t           tx_byte;    /* HAL 이 읽어가는 송신 바이트          */

/* ---------------------------------------------------------------------- */
/* 임계구역 (ISR 과 링 인덱스를 공유하므로 PRIMASK 로 짧게 막는다)          */
/* ---------------------------------------------------------------------- */
static inline uint32_t crit_enter(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	return primask;
}

static inline void crit_exit(uint32_t primask)
{
	__set_PRIMASK(primask);
}

/* ---------------------------------------------------------------------- */
/* RX                                                                     */
/* ---------------------------------------------------------------------- */
/* RX 무장. 실패(HAL_BUSY 등)를 반드시 기록해야 한다 - 무장이 한 번 빠지면 그
 * 뒤로 수신이 영구히 죽기 때문이다. rx_armed=0 이면 다음 Read 호출에서 재시도한다. */
static void rx_arm(void)
{
	if (HAL_UART_Receive_IT(UART_CTRL_HANDLE, &rx_byte, 1) == HAL_OK)
	{
		g_uart_ctrl.rx_armed = 1U;
	}
	else
	{
		g_uart_ctrl.rx_armed = 0U;
		g_uart_ctrl.rx_arm_err++;
	}
}

static void tx_kick(void);

/* 인터럽트 무장이 빠진 상태를 되살리는 안전망. 소비자(Read/ReadByte)가 매 폴링
 * 주기마다 부르므로, 어떤 경로로 무장을 놓쳐도 최대 한 주기 뒤에 복구된다.
 *   RX: Receive_IT 가 실패해 rx_armed 가 0 이면 재무장.
 *   TX: 링에 보낼 바이트가 남았는데 송신이 멈춰 있으면(무장 실패 등) 재킥.
 *       tx_kick 실패는 다음 Send 나 TxCplt 를 기다려야 풀리는데, 마지막 프레임을
 *       넣은 뒤 Send 가 더 안 오면 그대로 잠들어 버리기 때문이다. */
static void service_idle(void)
{
	if (!g_uart_ctrl.ready)
		return;

	if (!g_uart_ctrl.rx_armed)
	{
		uint32_t primask = crit_enter();
		rx_arm();
		crit_exit(primask);
	}
	if (!g_uart_ctrl.tx_busy && tx_head != tx_tail)
		tx_kick();
}

void UartCtrl_RxCpltISR(void)
{
	uint16_t next = (uint16_t)((rx_head + 1U) & RX_MASK);
	if (next != rx_tail)
	{
		rx_buf[rx_head] = rx_byte;
		rx_head = next;
		g_uart_ctrl.rx_bytes++;
	}
	else
	{
		/* 링 풀: 새 바이트를 버린다(이미 받은 프레임을 살리는 쪽을 택한다). */
		g_uart_ctrl.rx_drop++;
	}
	rx_arm();
}

void UartCtrl_ErrorISR(void)
{
	g_uart_ctrl.err_cnt++;
	/* ORE/NE/FE/PE 로 RX 가 풀린 상태이므로 다시 무장한다. */
	rx_arm();
}

uint16_t UartCtrl_Read(uint8_t *buf, uint16_t max)
{
	uint16_t n = 0;

	if (buf == NULL)
		return 0;

	service_idle();

	while (n < max && rx_head != rx_tail)
	{
		buf[n++] = rx_buf[rx_tail];
		rx_tail = (uint16_t)((rx_tail + 1U) & RX_MASK);
	}
	return n;
}

uint8_t UartCtrl_ReadByte(uint8_t *out)
{
	if (out == NULL)
		return 0;

	if (rx_head == rx_tail)
	{
		service_idle();             /* 링이 비었을 때만 확인(핫루프 부담 없음) */
		return 0;
	}

	*out = rx_buf[rx_tail];
	rx_tail = (uint16_t)((rx_tail + 1U) & RX_MASK);
	return 1;
}

uint16_t UartCtrl_RxCount(void)
{
	return (uint16_t)((rx_head - rx_tail) & RX_MASK);
}

void UartCtrl_FlushRx(void)
{
	rx_tail = rx_head;
}

/* ---------------------------------------------------------------------- */
/* TX                                                                     */
/* ---------------------------------------------------------------------- */
/* 송신 킥. 태스크(Send)와 ISR(TxCplt) 양쪽에서 불린다.
 * tx_tail 은 여기서 전진시키지 않는다 - "송신 완료"를 확인한 TxCpltISR 에서만
 * 전진하므로 Transmit_IT 무장이 실패해도 바이트를 잃지 않고 재시도된다.
 *
 * ★HAL_UART_Transmit_IT 호출까지 임계구역 안에 넣는 이유(중요):
 *   Transmit_IT 는 마지막에 __HAL_UART_ENABLE_IT(TXE) = SET_BIT(CR1, TXEIE) 를
 *   하는데, 이 매크로는 원자적이지 않은 read-modify-write 다. UART5 의 RX 와 TX 는
 *   같은 UART5_IRQn 을 공유하므로, 태스크가 CR1 을 읽은 직후 UART5 인터럽트가
 *   떠서 그 안에서 rx_arm() -> SET_BIT(CR1, RXNEIE) 를 하면, 돌아온 태스크가
 *   RXNEIE 가 빠진 낡은 값을 덮어써 수신 인터럽트가 영구히 죽는다(1바이트
 *   Receive_IT 는 매 바이트마다 RXNEIE 를 껐다 켜므로 항상 이 패턴에 노출된다).
 *   HAL 호출 전체를 IRQ 금지 구간에 넣어 CR1 갱신을 직렬화한다(수십 사이클). */
static void tx_kick(void)
{
	uint32_t primask = crit_enter();

	if (g_uart_ctrl.tx_busy || tx_head == tx_tail)
	{
		crit_exit(primask);
		return;
	}
	tx_byte = tx_buf[tx_tail];

	if (HAL_UART_Transmit_IT(UART_CTRL_HANDLE, &tx_byte, 1) == HAL_OK)
	{
		g_uart_ctrl.tx_busy = 1U;
	}
	else
	{
		/* 소비되지 않았다(tail 그대로) - 다음 킥에서 재시도한다. */
		g_uart_ctrl.tx_arm_err++;
	}
	crit_exit(primask);
}

void UartCtrl_TxCpltISR(void)
{
	/* 방금 나간 바이트를 링에서 확정 제거한다. */
	tx_tail = (uint16_t)((tx_tail + 1U) & TX_MASK);
	g_uart_ctrl.tx_bytes++;
	g_uart_ctrl.tx_busy = 0U;
	tx_kick();                      /* 남은 바이트 이어서 송신 */
}

uint16_t UartCtrl_TxPending(void)
{
	return (uint16_t)((tx_head - tx_tail) & TX_MASK);
}

uint16_t UartCtrl_TxFree(void)
{
	/* 링은 head==tail 을 "빈 상태"로 쓰므로 실제 적재 가능량은 크기-1. */
	return (uint16_t)(TX_MASK - UartCtrl_TxPending());
}

uint8_t UartCtrl_TxBusy(void)
{
	return (uint8_t)(g_uart_ctrl.tx_busy || (tx_head != tx_tail));
}

uint16_t UartCtrl_Send(const uint8_t *data, uint16_t len)
{
	uint16_t n = 0;

	if (data == NULL || len == 0U || !g_uart_ctrl.ready)
		return 0;

	while (n < len)
	{
		uint16_t next = (uint16_t)((tx_head + 1U) & TX_MASK);
		if (next == tx_tail)
			break;                  /* 링 풀 */
		tx_buf[tx_head] = data[n++];
		tx_head = next;
	}
	if (n < len)
		g_uart_ctrl.tx_drop = (uint16_t)(g_uart_ctrl.tx_drop + (len - n));

	tx_kick();
	return n;
}

uint8_t UartCtrl_SendFrame(const uint8_t *data, uint16_t len)
{
	if (data == NULL || len == 0U || !g_uart_ctrl.ready)
		return 0;

	/* 프레임이 중간에서 잘리면 수신측 파서가 리싱크로 프레임 하나를 통째로
	 * 날린다. 통째로 들어갈 여유가 없으면 아예 넣지 않고 버린 것으로 집계한다. */
	if (UartCtrl_TxFree() < len)
	{
		g_uart_ctrl.tx_drop = (uint16_t)(g_uart_ctrl.tx_drop + len);
		return 0;
	}
	return (uint8_t)(UartCtrl_Send(data, len) == len);
}

uint16_t UartCtrl_SendString(const char *str)
{
	if (str == NULL)
		return 0;
	return UartCtrl_Send((const uint8_t *)str, (uint16_t)strlen(str));
}

void UartCtrl_FlushTx(void)
{
	uint32_t primask = crit_enter();
	tx_head = tx_tail;              /* 진행 중인 1바이트는 그대로 끝난다 */
	crit_exit(primask);
}

/* ---------------------------------------------------------------------- */
/* 요청/응답 (바쁜대기 - 벤치/브링업 전용)                                  */
/* ---------------------------------------------------------------------- */
uint16_t UartCtrl_Request(const uint8_t *tx, uint16_t txlen,
                          uint8_t *rx, uint16_t rxmax,
                          uint32_t timeout_ms, uint32_t idle_ms)
{
	uint32_t start;
	uint32_t last;
	uint16_t n = 0;

	if (rx == NULL || rxmax == 0U)
		return 0;

	UartCtrl_FlushRx();            /* 이전 응답 잔여 제거 */
	if (tx != NULL && txlen != 0U)
		(void)UartCtrl_Send(tx, txlen);

	start = HAL_GetTick();
	last  = start;

	for (;;)
	{
		uint8_t c;
		if (UartCtrl_ReadByte(&c))
		{
			rx[n++] = c;
			last = HAL_GetTick();
			if (n >= rxmax)
				break;
			continue;
		}
		if ((HAL_GetTick() - start) >= timeout_ms)
			break;
		/* 첫 바이트가 온 뒤 idle_ms 동안 조용하면 응답 끝으로 본다. */
		if (n != 0U && idle_ms != 0U && (HAL_GetTick() - last) >= idle_ms)
			break;
	}
	return n;
}

/* ---------------------------------------------------------------------- */
/* 라이프사이클                                                            */
/* ---------------------------------------------------------------------- */
void UartCtrl_Init(void)
{
	uint32_t primask = crit_enter();
	rx_head = 0;
	rx_tail = 0;
	tx_head = 0;
	tx_tail = 0;
	crit_exit(primask);

	memset((void *)&g_uart_ctrl, 0, sizeof(g_uart_ctrl));

	/* 이전 소유자(ble.c 등)가 이미 RX 를 무장해 두었으면 Receive_IT 가 BUSY 로
	 * 실패해 영구히 수신이 죽는다. 먼저 끊고 우리 것으로 무장한다. */
	(void)HAL_UART_AbortReceive(UART_CTRL_HANDLE);

	g_uart_ctrl.ready = 1U;

	primask = crit_enter();
	rx_arm();                       /* CR1 갱신 - tx_kick 과 직렬화(tx_kick 주석) */
	crit_exit(primask);
}

void UartCtrl_DeInit(void)
{
	g_uart_ctrl.ready = 0U;
	(void)HAL_UART_AbortReceive(UART_CTRL_HANDLE);
	(void)HAL_UART_AbortTransmit(UART_CTRL_HANDLE);
	g_uart_ctrl.tx_busy = 0U;
	UartCtrl_FlushRx();
	UartCtrl_FlushTx();
}
