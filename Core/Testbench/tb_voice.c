#include "tb_voice.h"

#include "w25q128.h"
#include "voice.h"
#include "voice_table.h"
#include <string.h>

/* U21 음성 플래시 테스트벤치.  ★R3 신규(4단계, §0.11.5).
 * 사용법·안전장치·태스크 배치는 tb_voice.h 참조.
 * 여기서는 설계서(U21_음성플래시_구조R3.md)의 바이트 맵을 offset 상수로 읽는다 —
 * packed 구조체는 4단계에서 `Speaker/voice.h` 가 소유할 것이고, 벤치가 미리
 * 정의해 두면 그때 두 정의가 갈라진다. */

/* ---- 디렉터리 필드 오프셋 (§2) ---------------------------------------- */
#define DIR_O_MAGIC        0x000u   /* 'Z''G''V''C'                          */
#define DIR_O_VERSION      0x004u   /* u16                                   */
#define DIR_O_SLOT_KB      0x006u   /* u16                                   */
#define DIR_O_SLOT_COUNT   0x008u   /* u8                                    */
#define DIR_O_ENTRY_SIZE   0x009u   /* u8                                    */
#define DIR_O_RATE         0x00Au   /* u16                                   */
#define DIR_O_ENTRY0       0x010u   /* 엔트리 n = +16n                       */
#define DIR_ENTRY_SIZE     16u
#define DIR_E_O_DATA_LEN   4u       /* 엔트리 내부: u32, 0 = 빈 슬롯         */
#define DIR_O_CRC32        0x210u   /* u32, 0x000~0x20F 에 대한 CRC32        */

/* ---- 슬롯 헤더 필드 오프셋 (§3 + §7.2 구조체 꼬리) -------------------- */
#define HDR_O_MAGIC        0x00u    /* 'Z''G''V''F'                          */
#define HDR_O_ID           0x04u    /* u8                                    */
#define HDR_O_DATA_LEN     0x0Cu    /* u32                                   */
#define HDR_O_DATA_CRC32   0x14u    /* u32                                   */
#define HDR_O_DUR_MS       0x18u    /* u32                                   */
#define HDR_O_CRC32        0xFCu    /* u32, 0x00~0xFB 에 대한 CRC32          */

/* ---- 자가검사 파라미터 -------------------------------------------------
 * 기록 시작을 페이지 중간(+0xF0)으로 잡은 것이 요점이다: 256B 페이지 경계를
 * 반드시 걸치게 해서 W25Q_Write 의 분할 로직을 실제로 태운다. 02h 는 페이지
 * 안에서 랩어라운드하므로 분할이 틀리면 여기서 바로 깨진다. */
#define TEST_OFF           0x0F0u   /* 첫 페이지의 끝 16B 부터 시작          */
#define TEST_LEN           512u     /* → 3개 페이지에 걸친다                 */
#define SECTOR_BYTES       4096u

/* ---- 커맨드 ------------------------------------------------------------ */
volatile uint8_t  tb_voice_probe_once;
volatile uint8_t  tb_voice_dir_once;
volatile uint8_t  tb_voice_hdr_once;
volatile uint8_t  tb_voice_dump_once;
volatile uint8_t  tb_voice_selftest_once;
volatile uint8_t  tb_voice_play_once;
volatile uint8_t  tb_voice_stop_once;

/* ---- 파라미터 ---------------------------------------------------------- */
volatile uint8_t  tb_voice_slot;
volatile uint32_t tb_voice_addr;
volatile uint32_t tb_voice_test_addr = TB_VOICE_TEST_DEFAULT;

/* ---- 결과 -------------------------------------------------------------- */
volatile uint32_t tb_voice_jedec_id;
volatile int8_t   tb_voice_status;
volatile uint32_t tb_voice_runs;

volatile uint8_t  tb_voice_dir_valid;
volatile uint8_t  tb_voice_dir_crc_ok;
volatile uint8_t  tb_voice_dir_blank;
volatile uint8_t  tb_voice_dir_slots;
volatile uint16_t tb_voice_dir_version;
volatile uint16_t tb_voice_dir_rate;

volatile uint8_t  tb_voice_hdr_valid;
volatile uint8_t  tb_voice_hdr_crc_ok;
volatile uint8_t  tb_voice_hdr_blank;
volatile uint32_t tb_voice_hdr_len;
volatile uint32_t tb_voice_hdr_crc;
volatile uint32_t tb_voice_hdr_dur_ms;
volatile uint32_t tb_voice_hdr_addr;

volatile uint8_t  tb_voice_dump[TB_VOICE_DUMP_BYTES];

volatile uint8_t  tb_voice_play_err;
volatile uint32_t tb_voice_play_dur_ms;

volatile uint8_t  tb_voice_step;
volatile uint16_t tb_voice_fails;
volatile uint32_t tb_voice_bad_off;
volatile uint8_t  tb_voice_bad_exp;
volatile uint8_t  tb_voice_bad_got;
volatile uint32_t tb_voice_erase_ms;
volatile uint32_t tb_voice_prog_ms;
volatile uint32_t tb_voice_total_ms;

/* ---- 버퍼 --------------------------------------------------------------
 * 전부 static. defaultTask 스택은 1KB 뿐이다(freertos.c) — 532B 디렉터리를
 * 지역변수로 잡으면 넘친다. 이 벤치는 defaultTask 단독 소유라 안전하다. */
static uint8_t s_dir[TB_VOICE_DIR_BYTES];    /* 532B */
static uint8_t s_hdr[TB_VOICE_HDR_BYTES];    /* 256B */
static uint8_t s_pat[TEST_LEN];              /* 512B 기대 패턴 */
static uint8_t s_rb[256];                    /* 읽기 비교용 청크 */

/* ---- 리틀엔디안 언패커 (설계서 §2 "리틀 엔디언") ---------------------- */
static uint16_t rd16(const uint8_t *p, uint32_t off)
{
	return (uint16_t)((uint16_t)p[off] | ((uint16_t)p[off + 1u] << 8));
}

static uint32_t rd32(const uint8_t *p, uint32_t off)
{
	return  (uint32_t)p[off]
	     | ((uint32_t)p[off + 1u] << 8)
	     | ((uint32_t)p[off + 2u] << 16)
	     | ((uint32_t)p[off + 3u] << 24);
}

/* CRC32 (IEEE 802.3, reflected, init/xorout 0xFFFFFFFF) = 호스트 도구
 * voice_image.py 의 zlib.crc32 와 같은 값. 테이블 없이 비트 단위로 돌린다 —
 * 532B 면 수십 us 라 벤치에서는 충분하고 1KB 룩업테이블을 안 먹는다.
 * ★4단계에서 voice.c 가 슬롯 전체(최대 256KB) CRC 를 돌리게 되면 그쪽은
 *   테이블 방식으로 올려야 한다(비트 단위로는 256KB 에 수십 ms). */
static uint32_t crc32_ieee(const uint8_t *p, uint32_t len)
{
	uint32_t crc = 0xFFFFFFFFu;

	while (len--)
	{
		crc ^= (uint32_t)*p++;
		for (uint8_t b = 0u; b < 8u; b++)
			crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
	}
	return crc ^ 0xFFFFFFFFu;
}

static uint8_t all_ff(const uint8_t *p, uint32_t len)
{
	while (len--)
	{
		if (*p++ != 0xFFu) return 0u;
	}
	return 1u;
}

/* 재현 가능한 의사난수 패턴. 0xFF 로 채우면 "기록이 안 돼도 통과"하고 0x00 만
 * 쓰면 NOR 의 1->0 방향만 타니, 두 값이 섞인 바이트열이어야 의미가 있다. */
static void make_pattern(void)
{
	uint32_t lfsr = 0xACE1u;

	for (uint32_t i = 0u; i < TEST_LEN; i++)
	{
		lfsr = (lfsr * 1103515245u) + 12345u;     /* 고정 시드 = 매번 같은 패턴 */
		s_pat[i] = (uint8_t)(lfsr >> 16);
	}
	s_pat[0]           = 0x00u;                   /* 전부-1 에서 전부-0 전이 포함 */
	s_pat[TEST_LEN - 1u] = 0xA5u;                 /* 끝 바이트 잘림 감지          */
}

/* ---- 커맨드 구현 ------------------------------------------------------- */

static void cmd_probe(void)
{
	uint32_t id = 0u;

	(void)W25Q_ReadID(&id);            /* 원시 ID 는 실패해도 남긴다 */
	tb_voice_jedec_id = id;
	tb_voice_status   = (int8_t)W25Q_Init();
}

static void cmd_dir(void)
{
	uint32_t n;

	tb_voice_dir_valid  = 0u;
	tb_voice_dir_crc_ok = 0u;
	tb_voice_dir_blank  = 0u;
	tb_voice_dir_slots  = 0u;

	tb_voice_status = (int8_t)W25Q_Read(TB_VOICE_DIR_ADDR, s_dir, TB_VOICE_DIR_BYTES);
	if (tb_voice_status != (int8_t)W25Q_OK) return;

	/* 0xFF 로 가득 = "디렉터리 없음"(§2). 아직 이미지를 안 구운 보드의 정상 상태다. */
	tb_voice_dir_blank = all_ff(s_dir, TB_VOICE_DIR_BYTES);
	if (tb_voice_dir_blank) return;

	tb_voice_dir_valid = (uint8_t)((s_dir[DIR_O_MAGIC]      == (uint8_t)'Z')
	                            && (s_dir[DIR_O_MAGIC + 1u] == (uint8_t)'G')
	                            && (s_dir[DIR_O_MAGIC + 2u] == (uint8_t)'V')
	                            && (s_dir[DIR_O_MAGIC + 3u] == (uint8_t)'C'));

	tb_voice_dir_version = rd16(s_dir, DIR_O_VERSION);
	tb_voice_dir_rate    = rd16(s_dir, DIR_O_RATE);

	/* DIR_CRC32 는 0x000~0x20F 구간에 대한 값이다(자기 자신은 제외). */
	tb_voice_dir_crc_ok = (uint8_t)(crc32_ieee(s_dir, DIR_O_CRC32)
	                                == rd32(s_dir, DIR_O_CRC32));

	for (n = 0u; n < TB_VOICE_SLOT_COUNT; n++)
	{
		uint32_t e = DIR_O_ENTRY0 + (n * DIR_ENTRY_SIZE);
		if (rd32(s_dir, e + DIR_E_O_DATA_LEN) != 0u)
			tb_voice_dir_slots++;
	}
}

static void cmd_hdr(void)
{
	uint8_t slot = tb_voice_slot;

	tb_voice_hdr_valid  = 0u;
	tb_voice_hdr_crc_ok = 0u;
	tb_voice_hdr_blank  = 0u;
	tb_voice_hdr_len    = 0u;
	tb_voice_hdr_crc    = 0u;
	tb_voice_hdr_dur_ms = 0u;

	if (slot >= TB_VOICE_SLOT_COUNT)
	{
		tb_voice_status = (int8_t)W25Q_PARAM;
		return;
	}

	tb_voice_hdr_addr = TB_VOICE_SLOT_BASE((uint32_t)slot);
	tb_voice_status   = (int8_t)W25Q_Read(tb_voice_hdr_addr, s_hdr, TB_VOICE_HDR_BYTES);
	if (tb_voice_status != (int8_t)W25Q_OK) return;

	tb_voice_hdr_blank = all_ff(s_hdr, TB_VOICE_HDR_BYTES);
	if (tb_voice_hdr_blank) return;

	/* ID 가 슬롯 번호와 다르면 무효다(§3) — 잘못된 슬롯에 구워진 파일을 잡는다. */
	tb_voice_hdr_valid = (uint8_t)((s_hdr[HDR_O_MAGIC]      == (uint8_t)'Z')
	                            && (s_hdr[HDR_O_MAGIC + 1u] == (uint8_t)'G')
	                            && (s_hdr[HDR_O_MAGIC + 2u] == (uint8_t)'V')
	                            && (s_hdr[HDR_O_MAGIC + 3u] == (uint8_t)'F')
	                            && (s_hdr[HDR_O_ID] == slot));

	tb_voice_hdr_len    = rd32(s_hdr, HDR_O_DATA_LEN);
	tb_voice_hdr_crc    = rd32(s_hdr, HDR_O_DATA_CRC32);
	tb_voice_hdr_dur_ms = rd32(s_hdr, HDR_O_DUR_MS);

	tb_voice_hdr_crc_ok = (uint8_t)(crc32_ieee(s_hdr, HDR_O_CRC32)
	                                == rd32(s_hdr, HDR_O_CRC32));
}

/* 슬롯 재생. 비블로킹 - Voice_Play 가 DMA 를 걸고 바로 돌아온다. */
static void cmd_play(void)
{
	uint32_t dur = 0u;

	/* 벤치는 표의 종류를 그대로 따른다 - 에러 안내를 고르면 에러 우선순위로
	 * 들어가므로 §7.4 규칙까지 같이 시험된다. */
	/* Voice_Play 는 비블로킹 요청이라 유효성을 돌려주지 않는다 - 벤치는
	 * 결과를 보여 줘야 하므로 프로브를 먼저 한다. */
	tb_voice_play_err = (uint8_t)Voice_Probe(tb_voice_slot, NULL, &dur);
	if (tb_voice_play_err == (uint8_t)VOICE_ST_OK)
	{
		/* 표의 종류를 그대로 따른다 - 에러 안내를 고르면 에러 우선순위로
		 * 들어가므로 §7.4 규칙까지 같이 시험된다. */
		(void)Voice_Play((voice_id_t)tb_voice_slot,
		        Voice_IsErrorId((voice_id_t)tb_voice_slot) ? VOICE_PRIO_ERROR
		                                                   : VOICE_PRIO_NORMAL);
	}
	tb_voice_play_dur_ms = dur;
}

static void cmd_dump(void)
{
	tb_voice_status = (int8_t)W25Q_Read(tb_voice_addr, s_rb, TB_VOICE_DUMP_BYTES);
	if (tb_voice_status != (int8_t)W25Q_OK) return;

	for (uint32_t i = 0u; i < TB_VOICE_DUMP_BYTES; i++)
		tb_voice_dump[i] = s_rb[i];
}

/* 시험 섹터 4KB 를 256B 씩 읽어 전부 0xFF 인지 본다. 실패 위치를 기록한다. */
static uint8_t blank_check(uint32_t base)
{
	for (uint32_t off = 0u; off < SECTOR_BYTES; off += sizeof s_rb)
	{
		if (W25Q_Read(base + off, s_rb, sizeof s_rb) != W25Q_OK)
		{
			tb_voice_status = (int8_t)W25Q_ERROR;
			return 0u;
		}
		for (uint32_t i = 0u; i < sizeof s_rb; i++)
		{
			if (s_rb[i] != 0xFFu)
			{
				tb_voice_fails++;
				tb_voice_bad_off = off + i;
				tb_voice_bad_exp = 0xFFu;
				tb_voice_bad_got = s_rb[i];
				return 0u;
			}
		}
	}
	return 1u;
}

/* W25Q_Verify 와는 별개로 직접 다시 읽어 비교한다. Verify 가 통과하는데 이게
 * 깨지면 Verify 쪽(또는 읽기 경로)을 의심해야 하므로 교차확인 가치가 있다. */
static uint8_t reread_check(uint32_t addr)
{
	uint32_t done = 0u;

	while (done < TEST_LEN)
	{
		uint32_t n = TEST_LEN - done;
		if (n > sizeof s_rb) n = sizeof s_rb;

		if (W25Q_Read(addr + done, s_rb, n) != W25Q_OK)
		{
			tb_voice_status = (int8_t)W25Q_ERROR;
			return 0u;
		}
		for (uint32_t i = 0u; i < n; i++)
		{
			if (s_rb[i] != s_pat[done + i])
			{
				tb_voice_fails++;
				tb_voice_bad_off = done + i;
				tb_voice_bad_exp = s_pat[done + i];
				tb_voice_bad_got = s_rb[i];
				return 0u;
			}
		}
		done += n;
	}
	return 1u;
}

static void cmd_selftest(uint8_t tb_active)
{
	uint32_t base = tb_voice_test_addr;
	uint32_t t_all;
	uint32_t t0;

	tb_voice_step      = (uint8_t)TB_VOICE_ST_REFUSED;
	tb_voice_fails     = 0u;
	tb_voice_bad_off   = 0u;
	tb_voice_bad_exp   = 0u;
	tb_voice_bad_got   = 0u;
	tb_voice_erase_ms  = 0u;
	tb_voice_prog_ms   = 0u;
	tb_voice_total_ms  = 0u;

	/* ★안전장치 (1): 주소 화이트리스트. 디렉터리 섹터(0x000000)와 슬롯 영역
	 * (0x040000~)은 여기서 끊는다. 4KB 정렬도 요구한다 — 정렬이 안 맞으면
	 * 드라이버가 절삭해 버려서 의도한 섹터와 다른 곳을 지우게 된다. */
	if ((base < TB_VOICE_TEST_LO) || (base >= TB_VOICE_TEST_HI)
	    || ((base & (SECTOR_BYTES - 1u)) != 0u))
		return;

	/* ★안전장치 (2): 시나리오 중에는 defaultTask 를 1초 묶지 않는다. */
	if (!tb_active) return;

	t_all = HAL_GetTick();
	make_pattern();

	/* 1) 소거 */
	tb_voice_step = (uint8_t)TB_VOICE_ST_ERASE1;
	t0 = HAL_GetTick();
	tb_voice_status = (int8_t)W25Q_EraseSector(base);
	tb_voice_erase_ms = HAL_GetTick() - t0;
	if (tb_voice_status != (int8_t)W25Q_OK) { tb_voice_fails++; return; }

	/* 2) 소거가 실제로 먹었는지 4KB 전수 확인 */
	tb_voice_step = (uint8_t)TB_VOICE_ST_BLANK1;
	if (!blank_check(base)) return;

	/* 3) 페이지 경계를 걸친 512B 기록 */
	tb_voice_step = (uint8_t)TB_VOICE_ST_PROGRAM;
	t0 = HAL_GetTick();
	tb_voice_status = (int8_t)W25Q_Write(base + TEST_OFF, s_pat, TEST_LEN);
	tb_voice_prog_ms = HAL_GetTick() - t0;
	if (tb_voice_status != (int8_t)W25Q_OK) { tb_voice_fails++; return; }

	/* 4) 드라이버 Verify */
	tb_voice_step = (uint8_t)TB_VOICE_ST_VERIFY;
	tb_voice_status = (int8_t)W25Q_Verify(base + TEST_OFF, s_pat, TEST_LEN);
	if (tb_voice_status != (int8_t)W25Q_OK) { tb_voice_fails++; return; }

	/* 5) 독립 재읽기 교차확인 */
	tb_voice_step = (uint8_t)TB_VOICE_ST_REREAD;
	if (!reread_check(base + TEST_OFF)) return;

	/* 6) 시험 섹터는 소거 상태로 되돌려 둔다(다음 회차도 같은 조건에서 시작). */
	tb_voice_status = (int8_t)W25Q_EraseSector(base);
	if (tb_voice_status != (int8_t)W25Q_OK) { tb_voice_fails++; return; }

	tb_voice_step     = (uint8_t)TB_VOICE_ST_DONE;
	tb_voice_total_ms = HAL_GetTick() - t_all;
}

/* ---- 공개 API ---------------------------------------------------------- */

void TB_Voice_Init(void)
{
	tb_voice_probe_once    = 0u;
	tb_voice_dir_once      = 0u;
	tb_voice_hdr_once      = 0u;
	tb_voice_dump_once     = 0u;
	tb_voice_selftest_once = 0u;
	tb_voice_play_once     = 0u;
	tb_voice_stop_once     = 0u;

	tb_voice_play_err    = 0u;
	tb_voice_play_dur_ms = 0u;

	tb_voice_slot      = 0u;
	tb_voice_addr      = TB_VOICE_DIR_ADDR;
	tb_voice_test_addr = TB_VOICE_TEST_DEFAULT;

	tb_voice_jedec_id = 0u;
	tb_voice_status   = 0;
	tb_voice_runs     = 0u;

	tb_voice_dir_valid  = 0u;
	tb_voice_dir_crc_ok = 0u;
	tb_voice_dir_blank  = 0u;
	tb_voice_dir_slots  = 0u;
	tb_voice_dir_version = 0u;
	tb_voice_dir_rate    = 0u;

	tb_voice_hdr_valid  = 0u;
	tb_voice_hdr_crc_ok = 0u;
	tb_voice_hdr_blank  = 0u;
	tb_voice_hdr_len    = 0u;
	tb_voice_hdr_crc    = 0u;
	tb_voice_hdr_dur_ms = 0u;
	tb_voice_hdr_addr   = 0u;

	memset((void *)tb_voice_dump, 0, sizeof tb_voice_dump);

	tb_voice_step     = (uint8_t)TB_VOICE_ST_IDLE;
	tb_voice_fails    = 0u;
	tb_voice_bad_off  = 0u;
	tb_voice_bad_exp  = 0u;
	tb_voice_bad_got  = 0u;
	tb_voice_erase_ms = 0u;
	tb_voice_prog_ms  = 0u;
	tb_voice_total_ms = 0u;
}

void TB_Voice_Poll(uint8_t tb_active)
{
	/* 한 폴에 커맨드 1개만 처리한다. 자가검사가 ~0.9초라 여러 개를 한 번에
	 * 돌리면 defaultTask 가 그만큼 더 길게 묶인다. */
	if (tb_voice_probe_once)
	{
		tb_voice_probe_once = 0u;
		cmd_probe();
		tb_voice_runs++;
		return;
	}
	if (tb_voice_selftest_once)
	{
		tb_voice_selftest_once = 0u;
		cmd_selftest(tb_active);
		tb_voice_runs++;
		return;
	}
	if (tb_voice_dir_once)
	{
		tb_voice_dir_once = 0u;
		cmd_dir();
		tb_voice_runs++;
		return;
	}
	if (tb_voice_hdr_once)
	{
		tb_voice_hdr_once = 0u;
		cmd_hdr();
		tb_voice_runs++;
		return;
	}
	if (tb_voice_play_once)
	{
		tb_voice_play_once = 0u;
		cmd_play();
		tb_voice_runs++;
		return;
	}
	if (tb_voice_stop_once)
	{
		tb_voice_stop_once = 0u;
		Voice_Stop();
		tb_voice_runs++;
		return;
	}
	if (tb_voice_dump_once)
	{
		tb_voice_dump_once = 0u;
		cmd_dump();
		tb_voice_runs++;
	}
}
