#ifndef DEVICES_W25Q128_H_
#define DEVICES_W25Q128_H_

#include "main.h"
#include "spi.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * w25q128 - Winbond W25Q128JV (128Mbit / 16MB serial NOR flash) driver.
 *
 * ★**R3 (Revision 3) 신규 파일** — 착수 4단계(음성 경로). 구현현황 §0.11.
 *   R2 까지는 U21 을 쓰는 코드가 없었다.
 *
 * Part / wiring (REV02, U21 = W25Q128JVSIQ, sch/R3/FoodDisposal_REV02_NETLIST):
 *   U21.6 CLK        <- PA5  (SPI1_SCK,  label spi1_EEPROM_SCK)
 *   U21.2 DO(IO1)    -> PA6  (SPI1_MISO, label spi1_EEPROM_MISO)  via R81 22R
 *   U21.5 DI(IO0)    <- PA7  (SPI1_MOSI, label spi1_EEPROM_MOSI)  via R82 22R
 *   U21.1 /CS        <- PC4  (GPIO out,  label o_SPI1_EEPROM_CS)  + R80 10k pull-up
 *   U21.3 /WP(IO2)   -> SB21 -> +3.3V   (solder bridge, NOT MCU controlled)
 *   U21.7 /HOLD      -> SB22 -> +3.3V   (solder bridge, NOT MCU controlled)
 *
 * The board wires only IO0/IO1, so this is a plain single-line (1-1-1) SPI
 * driver - no Quad mode, no QE bit. SB21/SB22 must be closed for /WP and /HOLD
 * to sit high; if either bridge is open the chip reads 0xFF and never programs.
 *
 * SPI1 (Core/Src/spi.c): master, mode 0 (CPOL=0/CPHA=1EDGE), 8-bit, MSB first,
 * NSS soft, prescaler 4 -> 16MHz. (SYSCLK is 64MHz: the board runs off the HSI
 * RC, HSI/2 * PLL16 -- not an 8MHz HSE. SPI1 is on APB2 = HCLK = 64MHz.)
 * That is inside the 50MHz ceiling of
 * the plain Read Data (03h) instruction, so this driver uses 03h and needs no
 * dummy-cycle handling. Raising the prescaler past 50MHz would require switching
 * the read path to Fast Read (0Bh) + 8 dummy clocks.
 *
 * U21 is the only device on SPI1, so there is no bus arbitration here. Every
 * call is BLOCKING (HAL polling + status-register polling): call from a thread
 * context only - never from an ISR, and not from the 1ms StartMotorTask while a
 * program/erase is outstanding (a 4KB sector erase can take up to 400ms).
 * Erase/program waits yield with osDelay(1) once the scheduler is running.
 *
 * NOR semantics (this is flash, not a byte-writable EEPROM):
 *   - erase sets bits to 1; program can only clear 1 -> 0.
 *   - the smallest erase unit is one 4KB sector; there is no byte erase.
 *   => W25Q_Write() does NOT erase. To change bytes in a live sector, read
 *      the 4KB sector out, erase it, patch the copy in RAM and write it back.
 * ---------------------------------------------------------------------- */

/* ---- Geometry (128Mbit = 16MB, 24-bit address) ----------------------- */
#define W25Q_PAGE_SIZE      256U
#define W25Q_SECTOR_SIZE    4096U
#define W25Q_BLOCK32_SIZE   32768U
#define W25Q_BLOCK64_SIZE   65536U
#define W25Q_TOTAL_SIZE     (16U * 1024U * 1024U)

/* JEDEC ID (9Fh): EF = Winbond, 40 = SPI NOR, 18 = 128Mbit.
 * The -IQ/-JQ/-SIQ parts fitted here answer 0xEF4018; the DTR (-IM/-JM) variant
 * answers 0xEF7018. Both are accepted by W25Q_Init(). */
#define W25Q_JEDEC_ID       0x00EF4018U
#define W25Q_JEDEC_ID_DTR   0x00EF7018U

/* ---- Instruction opcodes (datasheet Table 8.1.2) --------------------- */
#define W25Q_CMD_WRITE_ENABLE   0x06U
#define W25Q_CMD_WRITE_DISABLE  0x04U
#define W25Q_CMD_READ_SR1       0x05U
#define W25Q_CMD_READ_SR2       0x35U
#define W25Q_CMD_READ_DATA      0x03U   /* 1-1-1, no dummy, <=50MHz      */
#define W25Q_CMD_FAST_READ      0x0BU   /* 1-1-1 + 8 dummy clocks        */
#define W25Q_CMD_PAGE_PROGRAM   0x02U   /* <=256B, no page-boundary wrap */
#define W25Q_CMD_SECTOR_ERASE   0x20U   /* 4KB                           */
#define W25Q_CMD_BLOCK_ERASE_32 0x52U   /* 32KB                          */
#define W25Q_CMD_BLOCK_ERASE_64 0xD8U   /* 64KB                          */
#define W25Q_CMD_CHIP_ERASE     0xC7U
#define W25Q_CMD_JEDEC_ID       0x9FU
#define W25Q_CMD_RESET_ENABLE   0x66U
#define W25Q_CMD_RESET_DEVICE   0x99U

/* ---- Status register-1 bits ------------------------------------------ */
#define W25Q_SR1_BUSY           0x01U   /* WIP: program/erase in progress */
#define W25Q_SR1_WEL            0x02U   /* write enable latch             */

typedef enum
{
	W25Q_OK = 0,      /* success                                        */
	W25Q_ERROR,       /* SPI transfer failed / WEL never latched        */
	W25Q_TIMEOUT,     /* BUSY still set after the op's timeout          */
	W25Q_BAD_ID,      /* 9Fh did not answer a W25Q128JV id              */
	W25Q_PARAM,       /* NULL pointer, len 0, or address out of range   */
	W25Q_MISMATCH     /* Verify(): read-back differs from source        */
} w25q_status_t;

/* ---- API --------------------------------------------------------------
 * addr is a raw 0-based flash offset (0 .. W25Q_TOTAL_SIZE-1). */

/* Probe the chip: read the JEDEC id and check it. Call once at start-up,
 * after MX_SPI1_Init()/MX_GPIO_Init(). */
w25q_status_t W25Q_Init(void);

/* Raw JEDEC id from 9Fh, packed as 0x00MMTTCC (manufacturer/type/capacity). */
w25q_status_t W25Q_ReadID(uint32_t *jedec_id);

/* true while a program/erase is still running (SR1.BUSY). Bus error -> true. */
bool W25Q_IsBusy(void);

/* Read len bytes from addr into dst. No alignment or length limit. */
w25q_status_t W25Q_Read(uint32_t addr, void *dst, uint32_t len);

/* Program len bytes of src at addr, splitting on 256B page boundaries.
 * The target range MUST have been erased first (see the NOR note above). */
w25q_status_t W25Q_Write(uint32_t addr, const void *src, uint32_t len);

/* Read the range back and compare with src. W25Q_OK = identical. */
w25q_status_t W25Q_Verify(uint32_t addr, const void *src, uint32_t len);

/* Erase one unit containing addr; addr is truncated to that unit's alignment. */
w25q_status_t W25Q_EraseSector(uint32_t addr);    /* 4KB   */
w25q_status_t W25Q_EraseBlock32(uint32_t addr);   /* 32KB  */
w25q_status_t W25Q_EraseBlock64(uint32_t addr);   /* 64KB  */

/* Erase the whole 16MB. Blocks for up to ~200s - bench/factory use only. */
w25q_status_t W25Q_EraseChip(void);

/* 66h+99h software reset. Aborts nothing that is already BUSY. */
w25q_status_t W25Q_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_W25Q128_H_ */
