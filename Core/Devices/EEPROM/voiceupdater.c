#include "voiceupdater.h"

#include "w25q128.h"
#include "voice.h"
#include "uart_ctrl.h"
#include "mode_arbiter.h"
#include "moeum.h"
#include "dongjak.h"
#include "cmsis_os.h"
#include <string.h>

/* UART5 음성 이미지 로더.  ★R3 신규(4단계 부수, §0.14).
 * 프레이밍·커맨드·소유권 전환은 voiceupdater.h 참조. */

VoiceUpdaterStat g_vu;

/* ---- 수신 버퍼 ----------------------------------------------------------
 * static. defaultTask 스택은 1KB 뿐이라(freertos.c) 4KB 프레임을 지역변수로
 * 잡을 수 없다. 로더는 defaultTask 단독 소유라 재진입 문제도 없다. */
static uint8_t  s_pay[VU_MAX_PAYLOAD];   /* 수신 중인 페이로드 (약 4.1KB)   */
static uint8_t  s_hdr[VU_HDR_BYTES];     /* BEGIN 이 준 헤더, END 에서 기록 */

/* 현재 슬롯 세션 */
static uint32_t s_slot_base;             /* SLOT_BASE(slot)                 */
static uint32_t s_crc_run;               /* 러닝 CRC32(비최종값)            */
static uint32_t s_crc_expect;            /* BEGIN 이 선언한 PCM CRC32       */

/* 세션 동안 잠시 바꿔 두는 모드. 끝나면 그대로 되돌린다(Enter/restore_mode). */
static app_mode_t s_saved_mode;
static uint8_t    s_saved_dbg;

/* ---- CRC32 (IEEE 802.3, reflected) ------------------------------------
 * 니블 테이블 = poly 0xEDB88320 을 4비트씩. 표 16개(64B)로 바이트당 2회.
 * 비트단위(8회)보다 4배 빠르다 — 2.8MB 전체에 약 0.4초. */
static const uint32_t s_crc_tab[16] =
{
	0x00000000u, 0x1DB71064u, 0x3B6E20C8u, 0x26D930ACu,
	0x76DC4190u, 0x6B6B51F4u, 0x4DB26158u, 0x5005713Cu,
	0xEDB88320u, 0xF00F9344u, 0xD6D6A3E8u, 0xCB61B38Cu,
	0x9B64C2B0u, 0x86D3D2D4u, 0xA00AE278u, 0xBDBDF21Cu
};

uint32_t VU_Crc32Init(void)  { return 0xFFFFFFFFu; }
uint32_t VU_Crc32Final(uint32_t crc) { return crc ^ 0xFFFFFFFFu; }

uint32_t VU_Crc32Update(uint32_t crc, const uint8_t *p, uint32_t len)
{
	while (len--)
	{
		crc ^= (uint32_t)*p++;
		crc = (crc >> 4) ^ s_crc_tab[crc & 0x0Fu];
		crc = (crc >> 4) ^ s_crc_tab[crc & 0x0Fu];
	}
	return crc;
}

uint32_t VU_Crc32(const uint8_t *p, uint32_t len)
{
	return VU_Crc32Final(VU_Crc32Update(VU_Crc32Init(), p, len));
}

/* ---- 리틀엔디안 헬퍼 --------------------------------------------------- */
static uint16_t rd16(const uint8_t *p)
{
	return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t rd32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
	     | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void wr16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
}

static void wr32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
	p[3] = (uint8_t)(v >> 24);
}

/* ---- 응답 --------------------------------------------------------------
 * RESP 는 고정 18바이트(SYNC2 + TYPE1 + LEN2 + PAYLOAD8 + CRC4). TX 링
 * (512B)에 늘 여유가 있으므로 통째로 적재된다. */
static void send_resp(uint8_t req_type, uint8_t code, uint16_t seq, uint32_t info)
{
	uint8_t f[2u + 1u + 2u + VU_RESP_LEN + 4u];

	f[0] = VU_SYNC0;
	f[1] = VU_SYNC1;
	f[2] = (uint8_t)VU_T_RESP;
	wr16(&f[3], (uint16_t)VU_RESP_LEN);
	f[5] = req_type;
	f[6] = code;
	wr16(&f[7], seq);
	wr32(&f[9], info);
	wr32(&f[13], VU_Crc32(&f[2], 1u + 2u + VU_RESP_LEN));  /* TYPE..PAYLOAD */

	g_vu.last_type = req_type;
	g_vu.last_code = code;

	(void)UartCtrl_SendFrame(f, (uint16_t)sizeof f);
}

/* ---- 커맨드 처리 ------------------------------------------------------- */

static void do_begin(const uint8_t *p, uint16_t len)
{
	uint32_t data_len;
	uint32_t span;
	uint32_t blocks;
	uint8_t  slot;

	if (len != VU_BEGIN_LEN) { send_resp(VU_T_BEGIN, VU_E_LEN, 0u, len); return; }
	/* 재생 중이면 SPI1 이 겹친다 - 멈추고 시작한다(voice.h "SPI1 공유"). */
	if (Voice_IsBusy())      { Voice_Stop(); send_resp(VU_T_BEGIN, VU_E_STATE, 0u, 0u); return; }

	slot     = p[0];
	data_len = rd32(&p[4]);

	if ((slot >= VU_SLOT_COUNT) || (data_len == 0u) || (data_len > VU_DATA_MAX))
	{
		send_resp(VU_T_BEGIN, VU_E_SLOT, 0u, data_len);
		return;
	}

	s_slot_base  = VU_SLOT_BASE((uint32_t)slot);
	s_crc_expect = rd32(&p[8]);
	memcpy(s_hdr, &p[12], VU_HDR_BYTES);      /* END 에서 기록 */

	/* 슬롯이 실제로 쓰는 64KB 블록만 지운다. 헤더(0x100)+데이터가 차지하는
	 * 구간을 64KB 로 올림 — 설계서 §8 은 4개 고정이지만, 짧은 멘트는 1~2개면
	 * 끝나므로 그만큼 시간을 아낀다(블록당 최대 2초). */
	span   = VU_SLOT_DATA_OFF + data_len;
	blocks = (span + 0xFFFFu) >> 16;
	if (blocks > 4u) blocks = 4u;

	for (uint32_t i = 0u; i < blocks; i++)
	{
		if (W25Q_EraseBlock64(s_slot_base + (i * 0x10000u)) != W25Q_OK)
		{
			send_resp(VU_T_BEGIN, VU_E_FLASH, 0u, i);
			return;
		}
	}

	s_crc_run      = VU_Crc32Init();
	g_vu.slot      = slot;
	g_vu.data_len  = data_len;
	g_vu.recv_len  = 0u;
	g_vu.seq       = 0u;
	g_vu.in_slot   = 1u;

	send_resp(VU_T_BEGIN, VU_OK, 0u, blocks);
}

static void do_data(const uint8_t *p, uint16_t len)
{
	uint16_t seq;
	uint32_t n;
	uint32_t addr;

	if (!g_vu.in_slot)   { send_resp(VU_T_DATA, VU_E_STATE, 0u, 0u); return; }
	if (len < 2u)        { send_resp(VU_T_DATA, VU_E_LEN, 0u, len);  return; }

	seq = rd16(p);
	n   = (uint32_t)len - 2u;

	/* 순번 확인. NOR 은 되돌릴 수 없으므로 어긋나면 슬롯을 통째로 다시 한다. */
	if (seq != g_vu.seq)
	{
		send_resp(VU_T_DATA, VU_E_SEQ, seq, g_vu.seq);
		return;
	}
	if ((n == 0u) || (n > VU_CHUNK_MAX))
	{
		send_resp(VU_T_DATA, VU_E_LEN, seq, n);
		return;
	}
	if ((g_vu.recv_len + n) > g_vu.data_len)
	{
		send_resp(VU_T_DATA, VU_E_OVERRUN, seq, g_vu.recv_len);
		return;
	}

	addr = s_slot_base + VU_SLOT_DATA_OFF + g_vu.recv_len;
	if (W25Q_Write(addr, &p[2], n) != W25Q_OK)
	{
		send_resp(VU_T_DATA, VU_E_FLASH, seq, addr);
		return;
	}

	s_crc_run      = VU_Crc32Update(s_crc_run, &p[2], n);
	g_vu.recv_len += n;
	g_vu.bytes    += n;
	g_vu.seq       = (uint16_t)(seq + 1u);

	send_resp(VU_T_DATA, VU_OK, seq, g_vu.seq);
}

static void do_end(uint16_t len)
{
	uint32_t crc;

	(void)len;

	if (!g_vu.in_slot) { send_resp(VU_T_END, VU_E_STATE, 0u, 0u); return; }

	if (g_vu.recv_len != g_vu.data_len)
	{
		g_vu.in_slot = 0u;
		send_resp(VU_T_END, VU_E_LEN, 0u, g_vu.recv_len);
		return;
	}

	crc = VU_Crc32Final(s_crc_run);
	if (crc != s_crc_expect)
	{
		/* 헤더를 쓰지 않고 끝낸다 → 그 슬롯은 "빈 슬롯"으로 남는다(§8.4). */
		g_vu.in_slot = 0u;
		send_resp(VU_T_END, VU_E_DATA_CRC, 0u, crc);
		return;
	}

	/* 이제서야 헤더 페이지를 굽는다. 헤더가 있다 = 데이터가 온전하다. */
	if (W25Q_Write(s_slot_base, s_hdr, VU_HDR_BYTES) != W25Q_OK)
	{
		g_vu.in_slot = 0u;
		send_resp(VU_T_END, VU_E_FLASH, 0u, s_slot_base);
		return;
	}
	if (W25Q_Verify(s_slot_base, s_hdr, VU_HDR_BYTES) != W25Q_OK)
	{
		g_vu.in_slot = 0u;
		send_resp(VU_T_END, VU_E_FLASH, 1u, s_slot_base);
		return;
	}

	g_vu.in_slot = 0u;
	g_vu.slots_done++;
	send_resp(VU_T_END, VU_OK, 0u, crc);
}

static void do_dir(const uint8_t *p, uint16_t len)
{
	if (len != VU_DIR_BYTES) { send_resp(VU_T_DIR, VU_E_LEN, 0u, len); return; }

	if (W25Q_EraseSector(VU_DIR_ADDR) != W25Q_OK)
	{
		send_resp(VU_T_DIR, VU_E_FLASH, 0u, 0u);
		return;
	}
	if (W25Q_Write(VU_DIR_ADDR, p, VU_DIR_BYTES) != W25Q_OK)
	{
		send_resp(VU_T_DIR, VU_E_FLASH, 1u, 0u);
		return;
	}
	if (W25Q_Verify(VU_DIR_ADDR, p, VU_DIR_BYTES) != W25Q_OK)
	{
		send_resp(VU_T_DIR, VU_E_FLASH, 2u, 0u);
		return;
	}

	g_vu.dir_done = 1u;
	send_resp(VU_T_DIR, VU_OK, 0u, VU_Crc32(p, VU_DIR_BYTES));
}

/* 검증용 재생. 굽자마자 바로 들어볼 수 있게 로더 안에 둔다.
 * ★기록 중(BEGIN~END)에는 거부한다 - 재생기(Voice_Task)와 로더가 SPI1 을
 *   동시에 쓰면 안 된다. w25q128 에는 락이 없고, 둘 다 정비 행위라
 *   동시에 할 이유도 없다(voice.h "SPI1 공유"). */
static void do_play(const uint8_t *p, uint16_t len)
{
	uint32_t dur = 0u;

	voice_status_t st;

	if (len != 1u)    { send_resp(VU_T_PLAY, VU_E_LEN, 0u, len); return; }
	if (g_vu.in_slot) { send_resp(VU_T_PLAY, VU_E_STATE, 0u, 0u); return; }

	/* Voice_Play 는 요청만 남기고 즉시 돌아오므로(비블로킹) 성공/실패를 알려
	 * 주지 않는다. 호스트에 정확한 사유를 돌려주려면 여기서 먼저 확인한다.
	 * 지금은 재생 중이 아니어야 하고(in_slot 검사 통과), 로더 세션이라
	 * Voice_Task 와 SPI 가 겹칠 일도 없다. */
	st = Voice_Probe(p[0], NULL, &dur);
	if (st != VOICE_ST_OK)
	{
		send_resp(VU_T_PLAY, VU_E_PLAY, 0u, (uint32_t)st);
		return;
	}
	(void)Voice_Play((voice_id_t)p[0], VOICE_PRIO_NORMAL);
	send_resp(VU_T_PLAY, VU_OK, 0u, dur);      /* 비블로킹 - 소리는 DMA 가 흘린다 */
}

static void do_stop(void)
{
	Voice_Stop();
	send_resp(VU_T_STOP, VU_OK, 0u, g_voice.played);
}

static void do_ping(void)
{
	uint32_t id = 0u;

	(void)W25Q_ReadID(&id);
	/* seq 자리에 프로토콜 리비전(R3)을 실어 보낸다 — 호스트가 펌웨어와
	 * 도구의 세대를 맞춰볼 수 있게(voiceupdater.h "프로토콜 리비전"). */
	send_resp(VU_T_PING, VU_OK, (uint16_t)VU_PROTO_REV, id);
}

static void handle(uint8_t type, const uint8_t *p, uint16_t len)
{
	g_vu.frames++;

	switch ((vu_type_t)type)
	{
	case VU_T_BEGIN: do_begin(p, len);        break;
	case VU_T_DATA:  do_data(p, len);         break;
	case VU_T_END:   do_end(len);             break;
	case VU_T_DIR:   do_dir(p, len);          break;
	case VU_T_PING:  do_ping();               break;
	case VU_T_PLAY:  do_play(p, len);         break;
	case VU_T_STOP:  do_stop();               break;
	case VU_T_EXIT:
		Voice_Stop();          /* 검증 재생을 켜 둔 채 나가지 않는다 */
		send_resp(VU_T_EXIT, VU_OK, 0u, g_vu.slots_done);
		g_vu.active  = 0u;                    /* 세션 종료 → protocol_r0 복귀 */
		g_vu.in_slot = 0u;
		break;
	default:
		send_resp(type, VU_E_TYPE, 0u, 0u);
		break;
	}
}

/* ---- 프레임 파서 -------------------------------------------------------
 * SYNC 2바이트로 동기를 잡고, 헤더 3바이트(TYPE+LEN)를 읽은 뒤 페이로드와
 * CRC32 를 모은다. 어디서든 틀어지면 st=0 으로 돌아가 바이트를 하나씩 밀며
 * 다시 SYNC 를 찾는다(g_vu.resync). */
static uint8_t  s_st;          /* 0=SYNC0 1=SYNC1 2=TYPE 3=LEN_L 4=LEN_H 5=PAY 6=CRC */
static uint8_t  s_type;
static uint16_t s_len;
static uint16_t s_got;
static uint8_t  s_crc_buf[4];

static void parser_reset(void)
{
	s_st  = 0u;
	s_got = 0u;
}

static void parse_byte(uint8_t c)
{
	switch (s_st)
	{
	case 0u:
		if (c == VU_SYNC0) s_st = 1u;
		else               g_vu.resync++;
		break;

	case 1u:
		if (c == VU_SYNC1)      { s_st = 2u; }
		else if (c == VU_SYNC0) { /* 0x56 0x56 ... : 두 번째를 새 SYNC0 로 */ }
		else                    { s_st = 0u; g_vu.resync++; }
		break;

	case 2u:
		s_type = c;
		s_st   = 3u;
		break;

	case 3u:
		s_len = c;
		s_st  = 4u;
		break;

	case 4u:
		s_len = (uint16_t)(s_len | ((uint16_t)c << 8));
		if (s_len > VU_MAX_PAYLOAD)
		{
			parser_reset();            /* 말이 안 되는 길이 → 재동기 */
			g_vu.resync++;
			break;
		}
		s_got = 0u;
		s_st  = (uint8_t)((s_len == 0u) ? 6u : 5u);
		break;

	case 5u:
		s_pay[s_got++] = c;
		if (s_got >= s_len) { s_got = 0u; s_st = 6u; }
		break;

	case 6u:
		s_crc_buf[s_got++] = c;
		if (s_got >= 4u)
		{
			uint32_t want = rd32(s_crc_buf);
			uint32_t calc;
			uint8_t  hdr3[3];

			/* CRC 대상 = TYPE + LEN(2, LE) + PAYLOAD. 헤더 3바이트를 재구성해
			 * 이어붙인 것과 같은 값을 증분으로 계산한다. */
			hdr3[0] = s_type;
			wr16(&hdr3[1], s_len);
			calc = VU_Crc32Update(VU_Crc32Init(), hdr3, 3u);
			calc = VU_Crc32Final(VU_Crc32Update(calc, s_pay, s_len));

			if (calc == want)
			{
				parser_reset();
				handle(s_type, s_pay, s_len);
			}
			else
			{
				g_vu.crc_err++;
				parser_reset();
				send_resp(s_type, VU_E_CRC, 0u, calc);
			}
		}
		break;

	default:
		parser_reset();
		break;
	}
}

/* ---- 공개 API ---------------------------------------------------------- */

void VoiceUpdater_Init(void)
{
	memset((void *)&g_vu, 0, sizeof g_vu);
	parser_reset();
}

uint8_t VoiceUpdater_Enter(void)
{
	if (g_vu.active) return 0u;

	/* ★거부하는 유일한 조건: 시나리오가 실제로 돌고 있을 때.
	 * 세션 동안 defaultTask 가 묶여 센서 폴링과 SenseTick 이 멈추므로,
	 * 운전 중인 모음/동작을 중간에 두고 들어가면 안 된다. */
	if (Moeum_IsBusy() || Dongjak_IsBusy()) return 0u;

	/* 마개가 시나리오 위치(HS1/2/4/5)에 얹혀 있어도 다운로드는 되어야 한다
	 * (사용자 지시 2026-09-20). 중재자는 100ms 마다 홀을 디코딩해 모드를
	 * 되돌리므로, dbg_disable 로 잠시 무력화하고 모드를 대기(APP_MODE_TESTBENCH=0)로 고정한다.
	 * 세션이 끝나면 restore_mode() 가 둘 다 원래대로 돌려놓는다 —
	 * 다운로드는 정식 기능이 아니라 1회성 정비 행위이므로, 평상시 동작에
	 * 흔적을 남기지 않는 것이 원칙이다. */
	s_saved_mode = g_app_mode;
	s_saved_dbg  = g_modearb.dbg_disable;
	g_modearb.dbg_disable = 1u;
	g_app_mode            = APP_MODE_TESTBENCH;

	parser_reset();
	g_vu.in_slot    = 0u;
	g_vu.frames     = 0u;
	g_vu.bytes      = 0u;
	g_vu.crc_err    = 0u;
	g_vu.resync     = 0u;
	g_vu.slots_done = 0u;
	g_vu.dir_done   = 0u;
	g_vu.session_ms = 0u;
	g_vu.active     = 1u;
	return 1u;
}

uint8_t VoiceUpdater_IsActive(void)
{
	return g_vu.active;
}

void VoiceUpdater_Poll(void)
{
	uint32_t t0;
	uint32_t last_rx;

	if (!g_vu.active) return;

	/* 진입 시점에 링에 남아 있던 protocol_r0 잔여 바이트를 버린다 — 업데이트
	 * 모드 진입 프레임 뒤에 붙어 온 것은 없어야 정상이고, 있으면 VU 파서가
	 * 엉뚱한 SYNC 를 잡는다. */
	UartCtrl_FlushRx();

	t0      = HAL_GetTick();
	last_rx = t0;

	/* 세션 루프: 끝날 때까지 이 안에서 돈다(voiceupdater.h "태스크 영향").
	 * osDelay(1) 양보라 MotorTask(1ms)는 정상 동작한다. */
	while (g_vu.active)
	{
		uint8_t  c;
		uint16_t guard = UART_CTRL_RX_BUFSZ;   /* 한 바퀴에 링 1회분까지 */
		uint8_t  got   = 0u;

		while (guard-- && UartCtrl_ReadByte(&c))
		{
			got = 1u;
			parse_byte(c);
			if (!g_vu.active) break;           /* EXIT 처리됨 */
		}

		if (got)
		{
			last_rx = HAL_GetTick();
		}
		else if ((HAL_GetTick() - last_rx) >= VU_SESSION_TIMEOUT_MS)
		{
			/* 호스트가 사라졌다. 영구 점유를 막고 protocol_r0 로 돌려준다. */
			g_vu.last_code = (uint8_t)VU_E_STATE;
			g_vu.in_slot   = 0u;
			g_vu.active    = 0u;
			break;
		}

		if (g_vu.active) osDelay(1u);
	}

	g_vu.session_ms = HAL_GetTick() - t0;
	parser_reset();
	UartCtrl_FlushRx();        /* R0 디코더가 VU 잔여 바이트를 먹지 않도록 */

	/* 모드와 중재자를 진입 전 상태로 되돌린다. dbg_disable 을 먼저 풀면 그
	 * 사이 한 틱에 중재자가 끼어들 수 있으므로 모드를 먼저 복원한다. */
	g_app_mode            = s_saved_mode;
	g_modearb.dbg_disable = s_saved_dbg;
}
