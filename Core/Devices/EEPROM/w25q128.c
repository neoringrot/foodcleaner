#include "w25q128.h"
#include "gpio_ctrl.h"
#include "cmsis_os.h"   /* osDelay() while polling BUSY, once the RTOS is up */
#include <string.h>

/* Winbond W25Q128JV (U21) over SPI1 + PC4 chip-select.  ★R3 신규(4단계, §0.11).
 * See w25q128.h for the wiring, the 1-1-1 transfer model and the NOR rules. */

/* ---- Timeouts ---------------------------------------------------------
 * The HAL timeout covers one chunk of bus traffic; the erase/program timeouts
 * are the datasheet worst cases (tW/tPP/tSE/tBE1/tBE2/tCE) with margin. */
#define W25Q_SPI_TIMEOUT     1000U     /* ms, one HAL transfer chunk       */
#define W25Q_PROG_TIMEOUT    100U      /* page program, max 3ms            */
#define W25Q_ERASE_SECTOR_TO 1000U     /* 4KB sector erase, max 400ms      */
#define W25Q_ERASE_BLOCK_TO  4000U     /* 32/64KB block erase, max 2000ms  */
#define W25Q_ERASE_CHIP_TO   250000U   /* chip erase, max 200s             */

/* HAL_SPI_Transmit/Receive take a uint16_t length, so long reads and writes are
 * split into chunks. CS stays low across the chunks of one instruction. */
#define W25Q_XFER_CHUNK      4096U

/* ---- Chip select (active low, PC4 = o_SPI1_EEPROM_CS) ---------------- */
static inline void cs_select(void)   { gpio_ctrl_off(GPIO_OUT_SPI1_EEPROM_CS); }
static inline void cs_release(void)  { gpio_ctrl_on(GPIO_OUT_SPI1_EEPROM_CS); }

/* ---- Leaf bus transfers (CS is owned by the caller) ------------------ */
static bool spi_tx(const uint8_t *buf, uint32_t len)
{
	while (len)
	{
		uint16_t n = (len > W25Q_XFER_CHUNK) ? (uint16_t)W25Q_XFER_CHUNK
		                                    : (uint16_t)len;
		if (HAL_SPI_Transmit(&hspi1, (uint8_t *)buf, n, W25Q_SPI_TIMEOUT) != HAL_OK)
			return false;
		buf += n;
		len -= n;
	}
	return true;
}

static bool spi_rx(uint8_t *buf, uint32_t len)
{
	while (len)
	{
		uint16_t n = (len > W25Q_XFER_CHUNK) ? (uint16_t)W25Q_XFER_CHUNK
		                                    : (uint16_t)len;
		/* STM32F1 HAL turns a 2-line master receive into a transmit-receive on
		 * the SAME buffer, so whatever sits in buf is clocked out on MOSI.
		 * Pre-fill with 0xFF to keep MOSI idle-high during a read. */
		memset(buf, 0xFF, n);
		if (HAL_SPI_Receive(&hspi1, buf, n, W25Q_SPI_TIMEOUT) != HAL_OK)
			return false;
		buf += n;
		len -= n;
	}
	return true;
}

/* ---- Instruction helpers --------------------------------------------- */

/* Single opcode, no address, no data (WREN, WRDI, chip erase, reset, ...). */
static bool cmd_only(uint8_t opcode)
{
	bool ok;
	cs_select();
	ok = spi_tx(&opcode, 1U);
	cs_release();
	return ok;
}

/* opcode + 24-bit big-endian address, no data (sector/block erase). */
static bool cmd_addr(uint8_t opcode, uint32_t addr)
{
	uint8_t frame[4];
	bool ok;

	frame[0] = opcode;
	frame[1] = (uint8_t)(addr >> 16);
	frame[2] = (uint8_t)(addr >> 8);
	frame[3] = (uint8_t)addr;

	cs_select();
	ok = spi_tx(frame, sizeof frame);
	cs_release();
	return ok;
}

/* Read one status register byte (SR1 = 05h, SR2 = 35h). */
static bool read_status(uint8_t opcode, uint8_t *out)
{
	bool ok;
	cs_select();
	ok = spi_tx(&opcode, 1U) && spi_rx(out, 1U);
	cs_release();
	return ok;
}

/* Let other tasks run while waiting on a slow erase/program. Before the
 * scheduler starts (W25Q_Init at boot) osDelay() must not be called. */
static void poll_yield(void)
{
	if (osKernelGetState() == osKernelRunning)
		(void)osDelay(1U);
}

/* Poll SR1.BUSY until the internal program/erase cycle ends. */
static w25q_status_t wait_busy(uint32_t timeout_ms)
{
	uint32_t t0 = HAL_GetTick();
	uint8_t  sr1;

	for (;;)
	{
		if (!read_status(W25Q_CMD_READ_SR1, &sr1)) return W25Q_ERROR;
		if ((sr1 & W25Q_SR1_BUSY) == 0U)           return W25Q_OK;
		if ((HAL_GetTick() - t0) >= timeout_ms)    return W25Q_TIMEOUT;
		poll_yield();
	}
}

/* 06h, then confirm SR1.WEL actually latched - it stays clear while the chip
 * is still BUSY, or when write protection blocked the latch. */
static w25q_status_t write_enable(void)
{
	uint8_t sr1;

	if (!cmd_only(W25Q_CMD_WRITE_ENABLE))      return W25Q_ERROR;
	if (!read_status(W25Q_CMD_READ_SR1, &sr1)) return W25Q_ERROR;
	return (sr1 & W25Q_SR1_WEL) ? W25Q_OK : W25Q_ERROR;
}

/* Erase one unit. addr is truncated to that unit alignment before it is sent. */
static w25q_status_t erase_unit(uint8_t opcode, uint32_t addr,
                               uint32_t unit, uint32_t timeout_ms)
{
	w25q_status_t st;

	if (addr >= W25Q_TOTAL_SIZE) return W25Q_PARAM;
	addr &= ~(unit - 1U);

	st = write_enable();
	if (st != W25Q_OK)           return st;
	if (!cmd_addr(opcode, addr)) return W25Q_ERROR;
	return wait_busy(timeout_ms);
}

/* Program one page: <=256B and guaranteed not to cross a page boundary. */
static w25q_status_t page_program(uint32_t addr, const uint8_t *src, uint32_t len)
{
	uint8_t frame[4];
	w25q_status_t st;
	bool ok;

	st = write_enable();
	if (st != W25Q_OK) return st;

	frame[0] = W25Q_CMD_PAGE_PROGRAM;
	frame[1] = (uint8_t)(addr >> 16);
	frame[2] = (uint8_t)(addr >> 8);
	frame[3] = (uint8_t)addr;

	cs_select();
	ok = spi_tx(frame, sizeof frame) && spi_tx(src, len);
	cs_release();
	if (!ok) return W25Q_ERROR;

	return wait_busy(W25Q_PROG_TIMEOUT);
}

/* ---- Public API ------------------------------------------------------ */

w25q_status_t W25Q_ReadID(uint32_t *jedec_id)
{
	uint8_t opcode = W25Q_CMD_JEDEC_ID;
	uint8_t rx[3]  = { 0U, 0U, 0U };
	bool ok;

	if (jedec_id == NULL) return W25Q_PARAM;

	cs_select();
	ok = spi_tx(&opcode, 1U) && spi_rx(rx, sizeof rx);
	cs_release();
	if (!ok) return W25Q_ERROR;

	*jedec_id = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | (uint32_t)rx[2];
	return W25Q_OK;
}

w25q_status_t W25Q_Init(void)
{
	uint32_t id = 0U;
	w25q_status_t st;

	cs_release();                       /* make sure /CS is idle-high first */

	st = W25Q_ReadID(&id);
	if (st != W25Q_OK) return st;

	/* 0xFFFFFF or 0x000000 here means no chip / dead bus, not a wrong part. */
	if ((id != W25Q_JEDEC_ID) && (id != W25Q_JEDEC_ID_DTR))
		return W25Q_BAD_ID;

	return W25Q_OK;
}

bool W25Q_IsBusy(void)
{
	uint8_t sr1;

	if (!read_status(W25Q_CMD_READ_SR1, &sr1)) return true;  /* fail safe */
	return (sr1 & W25Q_SR1_BUSY) != 0U;
}

w25q_status_t W25Q_Read(uint32_t addr, void *dst, uint32_t len)
{
	uint8_t frame[4];
	bool ok;

	if ((dst == NULL) || (len == 0U))   return W25Q_PARAM;
	if (addr >= W25Q_TOTAL_SIZE)        return W25Q_PARAM;
	if (len > (W25Q_TOTAL_SIZE - addr)) return W25Q_PARAM;

	/* 03h Read Data: the address auto-increments and wraps at the chip end,
	 * so one instruction covers the whole range with CS held low. */
	frame[0] = W25Q_CMD_READ_DATA;
	frame[1] = (uint8_t)(addr >> 16);
	frame[2] = (uint8_t)(addr >> 8);
	frame[3] = (uint8_t)addr;

	cs_select();
	ok = spi_tx(frame, sizeof frame) && spi_rx((uint8_t *)dst, len);
	cs_release();

	return ok ? W25Q_OK : W25Q_ERROR;
}

w25q_status_t W25Q_Write(uint32_t addr, const void *src, uint32_t len)
{
	const uint8_t *p = (const uint8_t *)src;

	if ((src == NULL) || (len == 0U))   return W25Q_PARAM;
	if (addr >= W25Q_TOTAL_SIZE)        return W25Q_PARAM;
	if (len > (W25Q_TOTAL_SIZE - addr)) return W25Q_PARAM;

	while (len)
	{
		/* 02h must not wrap inside a 256B page - split on the page boundary. */
		uint32_t page_off = addr % W25Q_PAGE_SIZE;
		uint32_t chunk    = W25Q_PAGE_SIZE - page_off;
		w25q_status_t st;

		if (chunk > len) chunk = len;

		st = page_program(addr, p, chunk);
		if (st != W25Q_OK) return st;

		addr += chunk;
		p    += chunk;
		len  -= chunk;
	}
	return W25Q_OK;
}

w25q_status_t W25Q_Verify(uint32_t addr, const void *src, uint32_t len)
{
	/* static, not a local: StartDefaultTask has a 1KB stack (freertos.c) and a
	 * 256B frame here plus the HAL call frames is too much of it. Safe because
	 * the whole driver is thread-context-only and single-owner (see header). */
	static uint8_t rb[W25Q_PAGE_SIZE];
	const uint8_t *p = (const uint8_t *)src;

	if ((src == NULL) || (len == 0U))   return W25Q_PARAM;
	if (addr >= W25Q_TOTAL_SIZE)        return W25Q_PARAM;
	if (len > (W25Q_TOTAL_SIZE - addr)) return W25Q_PARAM;

	while (len)
	{
		uint32_t n = (len > sizeof rb) ? (uint32_t)sizeof rb : len;
		w25q_status_t st = W25Q_Read(addr, rb, n);

		if (st != W25Q_OK)         return st;
		if (memcmp(rb, p, n) != 0) return W25Q_MISMATCH;

		addr += n;
		p    += n;
		len  -= n;
	}
	return W25Q_OK;
}

w25q_status_t W25Q_EraseSector(uint32_t addr)
{
	return erase_unit(W25Q_CMD_SECTOR_ERASE, addr,
	                  W25Q_SECTOR_SIZE, W25Q_ERASE_SECTOR_TO);
}

w25q_status_t W25Q_EraseBlock32(uint32_t addr)
{
	return erase_unit(W25Q_CMD_BLOCK_ERASE_32, addr,
	                  W25Q_BLOCK32_SIZE, W25Q_ERASE_BLOCK_TO);
}

w25q_status_t W25Q_EraseBlock64(uint32_t addr)
{
	return erase_unit(W25Q_CMD_BLOCK_ERASE_64, addr,
	                  W25Q_BLOCK64_SIZE, W25Q_ERASE_BLOCK_TO);
}

w25q_status_t W25Q_EraseChip(void)
{
	w25q_status_t st = write_enable();

	if (st != W25Q_OK)                  return st;
	if (!cmd_only(W25Q_CMD_CHIP_ERASE)) return W25Q_ERROR;
	return wait_busy(W25Q_ERASE_CHIP_TO);
}

w25q_status_t W25Q_Reset(void)
{
	if (!cmd_only(W25Q_CMD_RESET_ENABLE)) return W25Q_ERROR;
	if (!cmd_only(W25Q_CMD_RESET_DEVICE)) return W25Q_ERROR;
	return W25Q_OK;                       /* tRST 30us, then ready */
}
