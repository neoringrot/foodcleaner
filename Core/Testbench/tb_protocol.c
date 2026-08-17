#include "tb_protocol.h"
#include "protocol_r0.h"
#include <string.h>

/* R0 프로토콜 자체검사. 설계 의도/사용법은 tb_protocol.h 참조.
 * 벡터는 호스트 검증(src/pythonapp/tests/test_protocol.py)과 같은 것을 쓴다 -
 * 양쪽이 같은 벡터를 통과하면 앱/장치 상호운용이 사실상 보장된다. */

volatile uint8_t  tb_proto_enable;
volatile uint8_t  tb_proto_run_once;
volatile uint16_t tb_proto_checks;
volatile uint16_t tb_proto_fails;
volatile uint16_t tb_proto_first_fail;
volatile uint16_t tb_proto_runs;
volatile uint8_t  tb_proto_done;
volatile uint32_t tb_proto_us;
volatile int32_t  tb_proto_want;
volatile int32_t  tb_proto_got;

/* 이 벤치 전용 버퍼. protocol_r0.c 의 정적 버퍼(s_wire/s_dec/...)를 절대 쓰지
 * 않는다 - 앱 통신이 살아 있는 상태에서 돌려도 안전해야 한다. */
static uint8_t      tb_wire[PROTO_WIRE_MAX];
static uint8_t      tb_data[PROTO_DATA_MAX];
static ProtoDecoder tb_dec;
static ProtoFrame   tb_frame;

/* ---------------------------------------------------------------------- */
/* 검사 헬퍼                                                              */
/* ---------------------------------------------------------------------- */
static uint16_t s_idx;      /* 현재 검사 번호(1부터) */

static void chk(int32_t want, int32_t got)
{
	s_idx++;
	tb_proto_checks++;
	if (want != got)
	{
		tb_proto_fails++;
		if (tb_proto_first_fail == 0U)
			tb_proto_first_fail = s_idx;
		tb_proto_want = want;
		tb_proto_got  = got;
	}
}

/* 프레임을 통째로 디코더에 밀어 넣는다. 반환: 마지막 DecodeByte 결과. */
static int8_t feed(const uint8_t *wire, uint16_t n)
{
	int8_t r = 0;
	uint16_t i;
	Proto_DecoderReset(&tb_dec);
	for (i = 0; i < n; i++)
		r = Proto_DecodeByte(&tb_dec, wire[i], &tb_frame);
	return r;
}

/* ---------------------------------------------------------------------- */
/* 검사 본문                                                              */
/* ---------------------------------------------------------------------- */

/* 1) 평문 인코딩 + M 의 R/W 이스케이프 (호스트 test_encode_plain 과 동일 벡터).
 *    M(0x02) 은 STX 와 값이 같아 R/W 가 [DLE]0x02 로 나가야 한다. */
static void t_encode_m_escape(void)
{
	static const uint8_t exp[] = { 0x02, 0x10, 0x02, 0x20, 0x03, 0x00,
	                               0x11, 0x22, 0x33, 0x0D, 0x0A };
	uint8_t d[3] = { 0x11, 0x22, 0x33 };
	uint16_t n = Proto_Encode((uint8_t)PROTO_TYPE_M, 0x20, d, 3,
	                          tb_wire, (uint16_t)sizeof tb_wire);
	chk((int32_t)sizeof exp, (int32_t)n);
	chk(0, (n == sizeof exp) ? memcmp(tb_wire, exp, n) : -1);
}

/* 2) DATA 4바이트가 모두 이스케이프 대상 + LEN 은 DLE 를 세지 않는다 */
static void t_encode_escape_all(void)
{
	static const uint8_t exp[] = { 0x02, 0x01, 0x21, 0x04, 0x00,
	                               0x10, 0x02, 0x10, 0x0D, 0x10, 0x0A, 0x10, 0x10,
	                               0x0D, 0x0A };
	uint8_t d[4] = { 0x02, 0x0D, 0x0A, 0x10 };
	uint16_t n = Proto_Encode((uint8_t)PROTO_TYPE_R, 0x21, d, 4,
	                          tb_wire, (uint16_t)sizeof tb_wire);
	chk((int32_t)sizeof exp, (int32_t)n);
	chk(0, (n == sizeof exp) ? memcmp(tb_wire, exp, n) : -1);
	chk(4, tb_wire[3]);          /* LEN 하위 = DATA 길이 4 (DLE 미포함) */
	chk(0, tb_wire[4]);
}

/* 3) LEN 자체가 이스케이프 대상인 경우 (N=13 -> 0x0D, N=16 -> 0x10) */
static void t_encode_escape_len(void)
{
	uint8_t d[16];
	uint16_t i, n;
	for (i = 0; i < 16U; i++)
		d[i] = (uint8_t)(0x40U + i);

	n = Proto_Encode((uint8_t)PROTO_TYPE_W, 0x22, d, 13,
	                 tb_wire, (uint16_t)sizeof tb_wire);
	chk(PROTO_DLE, tb_wire[3]);
	chk(0x0D, tb_wire[4]);
	chk(1 + 1 + 1 + 3 + 13 + 2, (int32_t)n);

	/* N=16 은 MOEUM/SENSOR/OUTPUT 이 항상 밟는 경로다 */
	(void)Proto_Encode((uint8_t)PROTO_TYPE_W, 0x23, d, 16,
	                   tb_wire, (uint16_t)sizeof tb_wire);
	chk(PROTO_DLE, tb_wire[3]);
	chk(0x10, tb_wire[4]);
}

/* 4) 라운드트립 0~PROTO_DATA_MAX. 길이/내용이 그대로 돌아와야 한다. */
static void t_roundtrip(void)
{
	uint16_t len, i, n;
	for (len = 0; len <= PROTO_DATA_MAX; len++)
	{
		for (i = 0; i < len; i++)
			tb_data[i] = (uint8_t)((i * 7U) + len);

		n = Proto_Encode((uint8_t)PROTO_TYPE_M, 0x22, tb_data, len,
		                 tb_wire, (uint16_t)sizeof tb_wire);
		if (n == 0U)
		{
			chk(1, 0);                  /* 인코딩 실패 */
			continue;
		}
		chk(1, feed(tb_wire, n));
		chk(PROTO_TYPE_M, tb_frame.type);
		chk(0x22, tb_frame.cmd);
		chk((int32_t)len, (int32_t)tb_frame.len);
		chk(0, (len != 0U) ? memcmp(tb_frame.data, tb_data, len) : 0);
	}
}

/* 5) 전 바이트 이스케이프 = 최악 와이어 크기. PROTO_WIRE_MAX 안에 들어가야 한다. */
static void t_roundtrip_worst(void)
{
	static const uint8_t esc[4] = { 0x02, 0x0D, 0x0A, 0x10 };
	uint16_t i, n;
	for (i = 0; i < PROTO_DATA_MAX; i++)
		tb_data[i] = esc[i & 3U];

	n = Proto_Encode((uint8_t)PROTO_TYPE_R, 0x21, tb_data, PROTO_DATA_MAX,
	                 tb_wire, (uint16_t)sizeof tb_wire);
	chk(135, (int32_t)n);                 /* 1+1+1+2+128+2 */
	chk(1, (n <= PROTO_WIRE_MAX) ? 1 : 0);
	chk(1, feed(tb_wire, n));
	chk(PROTO_DATA_MAX, (int32_t)tb_frame.len);
	chk(0, memcmp(tb_frame.data, tb_data, PROTO_DATA_MAX));
}

/* 6) 선행 잡음 무시 + 잘린 프레임 뒤 재동기 */
static void t_resync(void)
{
	static const uint8_t junk[5] = { 0xFF, 0x55, 0x06, 0x15, 0x0A };
	uint8_t d[2] = { 0xAA, 0xBB };
	uint16_t n, i;
	int8_t r = 0;

	n = Proto_Encode((uint8_t)PROTO_TYPE_M, 0x20, d, 2,
	                 tb_wire, (uint16_t)sizeof tb_wire);

	/* 잡음 -> 정상 프레임 */
	Proto_DecoderReset(&tb_dec);
	for (i = 0; i < 5U; i++)
		chk(0, Proto_DecodeByte(&tb_dec, junk[i], &tb_frame));
	for (i = 0; i < n; i++)
		r = Proto_DecodeByte(&tb_dec, tb_wire[i], &tb_frame);
	chk(1, r);
	chk(0xAA, tb_frame.data[0]);
	chk(0xBB, tb_frame.data[1]);

	/* 앞 4바이트만 온 잘린 프레임 -> 정상 프레임 */
	Proto_DecoderReset(&tb_dec);
	for (i = 0; i < 4U; i++)
		(void)Proto_DecodeByte(&tb_dec, tb_wire[i], &tb_frame);
	for (i = 0; i < n; i++)
		r = Proto_DecodeByte(&tb_dec, tb_wire[i], &tb_frame);
	chk(1, r);
	chk(2, (int32_t)tb_frame.len);
}

/* 7) 규격 위반 프레임은 거부해야 한다.
 *    ★R/W 를 W(0x00) 로 쓴 이유: M(0x02) 을 raw 로 넣으면 디코더가 그 바이트를
 *      STX 로 보고 리싱크하므로 LEN 규칙이 아니라 리싱크를 시험하게 된다. */
static void t_reject_malformed(void)
{
	/* LEN=5 인데 DATA 2바이트 */
	static const uint8_t bad_len[] = { 0x02, 0x00, 0x20, 0x05, 0x00,
	                                   0x11, 0x22, 0x0D, 0x0A };
	/* ETX1 없는 단독 0x0A */
	static const uint8_t bare_lf[] = { 0x02, 0x00, 0x20, 0x00, 0x00, 0x0A };
	/* 0x0D 뒤에 0x0A 가 아닌 바이트 */
	static const uint8_t cr_x[]    = { 0x02, 0x00, 0x20, 0x00, 0x00, 0x0D, 0x99 };

	chk(-1, feed(bad_len, (uint16_t)sizeof bad_len));
	chk(-1, feed(bare_lf, (uint16_t)sizeof bare_lf));
	chk(-1, feed(cr_x,    (uint16_t)sizeof cr_x));
}

/* 8) 버퍼 부족 / 과대 길이는 0 을 돌려주고 절대 넘겨쓰지 않아야 한다.
 *    카나리로 오버런을 직접 확인한다(호스트 테스트로는 잡기 어려운 부분). */
static void t_bounds(void)
{
	uint8_t small[16];
	uint8_t canary[8];
	uint8_t d[8];
	uint16_t i;

	for (i = 0; i < 8U; i++)
		d[i] = PROTO_DLE;                 /* 전부 이스케이프 -> 23바이트 필요 */
	memset(canary, 0x5A, sizeof canary);

	chk(0, Proto_Encode((uint8_t)PROTO_TYPE_M, 0x20, d, 8,
	                    small, (uint16_t)sizeof small));
	/* 과대 길이 거부 */
	chk(0, Proto_Encode((uint8_t)PROTO_TYPE_M, 0x20, tb_data,
	                    PROTO_DATA_MAX + 1U, tb_wire, (uint16_t)sizeof tb_wire));
	/* NULL 인자 거부 */
	chk(0, Proto_Encode((uint8_t)PROTO_TYPE_M, 0x20, NULL, 4,
	                    tb_wire, (uint16_t)sizeof tb_wire));
	chk(0, Proto_Encode((uint8_t)PROTO_TYPE_M, 0x20, d, 8, NULL, 32));

	for (i = 0; i < 8U; i++)               /* 카나리 무손상 */
		chk(0x5A, canary[i]);
}

/* 9) 페이로드 빌더: 규격 길이대로 채우는지 + 버퍼 부족 시 0 인지.
 *    실제 값은 장치 상태에 달렸으니 여기서는 길이 계약만 검사한다. */
static void t_payload_lengths(void)
{
	struct { uint8_t cmd; uint16_t len; } t[] = {
		{ (uint8_t)PROTO_CMD_SYS_INFO, PROTO_LEN_SYS_INFO },
		{ (uint8_t)PROTO_CMD_STATUS,   PROTO_LEN_STATUS   },
		{ (uint8_t)PROTO_CMD_MOEUM,    PROTO_LEN_MOEUM    },
		{ (uint8_t)PROTO_CMD_DONGJAK,  PROTO_LEN_DONGJAK  },
		{ (uint8_t)PROTO_CMD_SENSOR,   PROTO_LEN_SENSOR   },
		{ (uint8_t)PROTO_CMD_JUNGJI,   PROTO_LEN_JUNGJI   },
		{ (uint8_t)PROTO_CMD_MOTOR,    PROTO_LEN_MOTOR    },
		{ (uint8_t)PROTO_CMD_OUTPUT,   PROTO_LEN_OUTPUT   },
		{ (uint8_t)PROTO_CMD_MON_CFG,  PROTO_LEN_MON_CFG  },
	};
	uint8_t i;

	for (i = 0; i < (uint8_t)(sizeof t / sizeof t[0]); i++)
	{
		chk((int32_t)t[i].len,
		    (int32_t)Proto_BuildPayload(t[i].cmd, tb_data, (uint16_t)sizeof tb_data));
		/* 버퍼가 1바이트 부족하면 0 이어야 한다(부분 기록 금지) */
		chk(0, (int32_t)Proto_BuildPayload(t[i].cmd, tb_data,
		                                   (uint16_t)(t[i].len - 1U)));
	}
	/* 미지원 CMD */
	chk(0, (int32_t)Proto_BuildPayload(0x99, tb_data, (uint16_t)sizeof tb_data));
}

/* 10) 리틀엔디안 직렬화가 실제로 LE 로 나오는지 + i16 음수 부호 보존.
 *     STATUS 를 인코딩해 되읽어, LEN 이 하위바이트 먼저인지 확인한다. */
static void t_endianness(void)
{
	uint16_t n;

	/* LEN = 20 (0x14) -> 하위 0x14, 상위 0x00 */
	(void)Proto_BuildPayload((uint8_t)PROTO_CMD_STATUS, tb_data,
	                         (uint16_t)sizeof tb_data);
	n = Proto_Encode((uint8_t)PROTO_TYPE_R, (uint8_t)PROTO_CMD_STATUS,
	                 tb_data, PROTO_LEN_STATUS, tb_wire, (uint16_t)sizeof tb_wire);
	chk(0x14, tb_wire[3]);
	chk(0x00, tb_wire[4]);
	chk(1, feed(tb_wire, n));
	chk(PROTO_LEN_STATUS, (int32_t)tb_frame.len);

	/* i16 음수: 온도 d10 은 음수가 될 수 있다(-40.0℃ = -400 = 0xFE70).
	 * 직접 페이로드를 만들어 왕복시켜 부호 확장이 깨지지 않는지 본다. */
	{
		int16_t v = -400;
		uint8_t p[2];
		p[0] = (uint8_t)((uint16_t)v & 0xFFU);
		p[1] = (uint8_t)((uint16_t)v >> 8);
		chk(0x70, p[0]);
		chk(0xFE, p[1]);
		n = Proto_Encode((uint8_t)PROTO_TYPE_M, 0x23, p, 2,
		                 tb_wire, (uint16_t)sizeof tb_wire);
		chk(1, feed(tb_wire, n));
		chk(-400, (int32_t)(int16_t)((uint16_t)tb_frame.data[0] |
		                             ((uint16_t)tb_frame.data[1] << 8)));
	}
}

/* 11) 모니터링 주기 하한: 마스크가 넓어지면 하한이 커져야 한다.
 *     앱(packets.mon_min_period_ms)과 같은 값이 나와야 한다. */
static void t_mon_min_period(void)
{
	chk(200, Proto_MonMinPeriodMs(PROTO_MON_STATUS));
	chk(289, Proto_MonMinPeriodMs(PROTO_MON_MASK_DEFAULT));
	chk(318, Proto_MonMinPeriodMs(PROTO_MON_ALL));
	chk(200, Proto_MonMinPeriodMs(0));
	/* 기본 마스크는 기본 주기로 돌 수 있어야 한다 */
	chk(1, (Proto_MonMinPeriodMs(PROTO_MON_MASK_DEFAULT)
	        <= PROTO_MON_PERIOD_DEFAULT_MS) ? 1 : 0);
}

/* 12) CMD 값이 이스케이프 대상값을 피해 배정되어 있는지(로그 가독성 규칙) */
static void t_cmd_ids(void)
{
	static const uint8_t cmds[] = {
		(uint8_t)PROTO_CMD_SYS_INFO, (uint8_t)PROTO_CMD_STATUS,
		(uint8_t)PROTO_CMD_MOEUM,    (uint8_t)PROTO_CMD_DONGJAK,
		(uint8_t)PROTO_CMD_SENSOR,   (uint8_t)PROTO_CMD_JUNGJI,
		(uint8_t)PROTO_CMD_MOTOR,    (uint8_t)PROTO_CMD_OUTPUT,
		(uint8_t)PROTO_CMD_MON_CFG,
	};
	uint8_t i;
	for (i = 0; i < (uint8_t)(sizeof cmds / sizeof cmds[0]); i++)
	{
		uint8_t c = cmds[i];
		chk(0, (c == PROTO_STX || c == PROTO_ETX1 ||
		        c == PROTO_ETX2 || c == PROTO_DLE) ? 1 : 0);
	}
}

/* ---------------------------------------------------------------------- */
/* 실행                                                                   */
/* ---------------------------------------------------------------------- */
uint16_t TB_Protocol_RunOnce(void)
{
	uint32_t t0 = HAL_GetTick();

	s_idx = 0;
	tb_proto_checks = 0;
	tb_proto_fails = 0;
	tb_proto_first_fail = 0;

	t_encode_m_escape();
	t_encode_escape_all();
	t_encode_escape_len();
	t_roundtrip();
	t_roundtrip_worst();
	t_resync();
	t_reject_malformed();
	t_bounds();
	t_payload_lengths();
	t_endianness();
	t_mon_min_period();
	t_cmd_ids();

	/* HAL_GetTick 은 1ms 분해능이라 us 는 근사값이다(1회 약 1~2ms). */
	tb_proto_us = (HAL_GetTick() - t0) * 1000UL;
	tb_proto_runs++;
	tb_proto_done = 1U;
	return tb_proto_fails;
}

void TB_Protocol_Init(void)
{
	tb_proto_enable = 0U;
	tb_proto_run_once = 0U;
	tb_proto_checks = 0U;
	tb_proto_fails = 0U;
	tb_proto_first_fail = 0U;
	tb_proto_runs = 0U;
	tb_proto_done = 0U;
	tb_proto_us = 0U;
	tb_proto_want = 0;
	tb_proto_got = 0;
	Proto_DecoderReset(&tb_dec);
}

void TB_Protocol_Poll(void)
{
	if (tb_proto_run_once)
	{
		tb_proto_run_once = 0U;      /* 1회성: 소비 즉시 클리어 */
		(void)TB_Protocol_RunOnce();
		return;
	}
	if (tb_proto_enable)
		(void)TB_Protocol_RunOnce();
}
