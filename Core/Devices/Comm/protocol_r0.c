/* ==========================================================================
 * protocol_r0.c - ZEROGEO 앱<->장치 프로토콜 R0 (doc/R1/zerogeo_protocol_r0.xlsx)
 *
 * 구성:
 *   §A 프레임 코덱   - STX/DLE/ETX 조립·해체. 장치 상태 의존 없음(순수).
 *   §B 페이로드 빌더 - 모음/동작/센서/정지 상태를 바이트 배열로 직렬화.
 *   §C 명령 처리     - R(요청) 응답 / W(쓰기) + 되읽기 검증 후 ACK|NAK.
 *   §D 서비스 틱     - RX 소진 -> 디스패치 -> 주기·변화 M(모니터링) 송신.
 *
 * 페이로드 바이트 배치와 CMD 값은 protocol_r0.h 의 §4·§7 표가 원본이다.
 * 이 파일을 고칠 때 표도 함께 고칠 것(앱과 공유하는 유일한 규격 문서).
 * ========================================================================== */

#include "protocol_r0.h"
#include "uart_ctrl.h"

#include "mode_arbiter.h"
#include "moeum.h"
#include "dongjak.h"
#include "jungji.h"
#include "bldc_ctrl.h"
#include "gpio_ctrl.h"
#include "thermistor.h"
#include "wdoor.h"
#include "tdoor.h"
#include "lift_motor.h"

#include <string.h>

/* freertos.c 가 100ms 마다 갱신하는 센서 스냅샷(dongjak.c 와 동일 관례로 여기서
 * extern 선언한다 - 이 전역들은 헤더가 없다). */
extern volatile int16_t  g_therm_c_d10[];
extern volatile uint8_t  g_bin_fill_pct;
extern volatile uint8_t  g_hall_mask;
extern volatile uint16_t g_distance_mm;

ProtoCtx g_proto;

/* 전방 선언: push_on_change 가 빌더와 백프레셔 판정을 쓴다(정의는 아래 §B/§D). */
static uint16_t build_motor(uint8_t *b, uint32_t now_ms);
static uint16_t build_output(uint8_t *b, uint32_t now_ms);
static uint8_t  line_backlogged(void);

/* 수신 디코더 + 프레임 조립 버퍼. 소유 태스크는 defaultTask 하나뿐이므로
 * 정적 할당해 스택(256*4B)을 아낀다. */
static ProtoDecoder s_dec;
static ProtoFrame   s_frame;
static uint8_t      s_wire[PROTO_WIRE_MAX];
static uint8_t      s_payload[PROTO_DATA_MAX];
/* 변화 감지용 스크래치(push_on_change). s_payload 와 분리해 두어야 비교 중에
 * 송신 경로가 같은 버퍼를 덮어쓰는 일이 없다. */
static uint8_t      s_chg[PROTO_DATA_MAX];

/* ==========================================================================
 * §A 프레임 코덱
 * ========================================================================== */

/* 이스케이프 대상: STX/ETX1/ETX2/DLE 값과 같은 바이트 (xlsx §1 DLE 규칙) */
static inline uint8_t needs_dle(uint8_t b)
{
	return (uint8_t)(b == PROTO_STX || b == PROTO_ETX1 ||
	                 b == PROTO_ETX2 || b == PROTO_DLE);
}

/* out 에 1바이트를 이스케이프 적용해 쓴다. 반환 = 쓴 바이트 수(1 또는 2),
 * 공간이 부족하면 0. */
static uint16_t put_esc(uint8_t *out, uint16_t pos, uint16_t out_sz, uint8_t b)
{
	if (needs_dle(b))
	{
		if ((uint16_t)(pos + 2U) > out_sz)
			return 0;
		out[pos]      = PROTO_DLE;
		out[pos + 1U] = b;
		return 2U;
	}
	if ((uint16_t)(pos + 1U) > out_sz)
		return 0;
	out[pos] = b;
	return 1U;
}

uint16_t Proto_Encode(uint8_t type, uint8_t cmd,
                      const uint8_t *data, uint16_t len,
                      uint8_t *out, uint16_t out_sz)
{
	uint8_t  hdr[PROTO_HDR_LEN];
	uint16_t pos = 0;
	uint16_t i;
	uint16_t n;

	if (out == NULL || out_sz < 7U || len > PROTO_DATA_MAX)
		return 0;
	if (len != 0U && data == NULL)
		return 0;

	/* STX 는 생 바이트(프레임 시작 표지) */
	out[pos++] = PROTO_STX;

	hdr[0] = type;
	hdr[1] = cmd;
#if PROTO_LEN_BIG_ENDIAN
	hdr[2] = (uint8_t)(len >> 8);
	hdr[3] = (uint8_t)(len & 0xFFU);
#else
	hdr[2] = (uint8_t)(len & 0xFFU);
	hdr[3] = (uint8_t)(len >> 8);
#endif

	/* R/W, CMD, LEN, DATA 는 모두 이스케이프 대상 (xlsx §1 마지막 줄) */
	for (i = 0; i < PROTO_HDR_LEN; i++)
	{
		n = put_esc(out, pos, out_sz, hdr[i]);
		if (n == 0U)
			return 0;
		pos = (uint16_t)(pos + n);
	}
	for (i = 0; i < len; i++)
	{
		n = put_esc(out, pos, out_sz, data[i]);
		if (n == 0U)
			return 0;
		pos = (uint16_t)(pos + n);
	}

	/* ETX 도 생 바이트 */
	if ((uint16_t)(pos + 2U) > out_sz)
		return 0;
	out[pos++] = PROTO_ETX1;
	out[pos++] = PROTO_ETX2;
	return pos;
}

void Proto_DecoderReset(ProtoDecoder *d)
{
	if (d == NULL)
		return;
	d->state = (uint8_t)PROTO_RX_IDLE;
	d->esc   = 0;
	d->n     = 0;
}

/* raw(이스케이프 해제됨) 를 검사해 프레임으로 확정한다. 1 = OK. */
static int8_t dec_finish(ProtoDecoder *d, ProtoFrame *f)
{
	uint16_t len;

	if (d->n < PROTO_HDR_LEN)
		return -1;                     /* R/W|CMD|LEN 도 못 채운 조각 */

#if PROTO_LEN_BIG_ENDIAN
	len = (uint16_t)(((uint16_t)d->raw[2] << 8) | d->raw[3]);
#else
	len = (uint16_t)(((uint16_t)d->raw[3] << 8) | d->raw[2]);
#endif
	if (len > PROTO_DATA_MAX)
		return -1;
	/* LEN 과 실제 수집량이 일치해야 한다(DLE 는 이미 벗겨져 있다). */
	if ((uint16_t)(PROTO_HDR_LEN + len) != d->n)
		return -1;

	f->type = d->raw[0];
	f->cmd  = d->raw[1];
	f->len  = len;
	if (len != 0U)
		memcpy(f->data, &d->raw[PROTO_HDR_LEN], len);
	return 1;
}

int8_t Proto_DecodeByte(ProtoDecoder *d, uint8_t b, ProtoFrame *f)
{
	if (d == NULL || f == NULL)
		return -1;

	/* 이스케이프된 바이트는 값이 무엇이든 데이터다(STX/ETX 판정보다 먼저 처리).
	 * esc 는 PROTO_RX_BODY 에서만 세워지므로 여기 도달했을 때 state 는 BODY 다. */
	if (d->esc)
	{
		d->esc = 0;
		if (d->n >= PROTO_RAW_MAX)
		{
			Proto_DecoderReset(d);
			return -1;
		}
		d->raw[d->n++] = b;
		return 0;
	}

	switch ((proto_rx_state_t)d->state)
	{
	case PROTO_RX_IDLE:
		if (b == PROTO_STX)
		{
			d->state = (uint8_t)PROTO_RX_BODY;
			d->esc   = 0;
			d->n     = 0;
		}
		/* STX 밖의 바이트는 조용히 버린다(BLE 링크의 잡음/재동기 구간). */
		return 0;

	case PROTO_RX_BODY:
		if (b == PROTO_DLE)
		{
			d->esc = 1;
			return 0;
		}
		if (b == PROTO_STX)
		{
			/* 생 STX = 이전 프레임이 잘렸다. 여기서 새로 시작(리싱크). */
			d->n = 0;
			return -1;
		}
		if (b == PROTO_ETX1)
		{
			d->state = (uint8_t)PROTO_RX_ETX2;
			return 0;
		}
		if (b == PROTO_ETX2)
		{
			/* ETX1 없이 0x0A 단독 = 규격 위반 */
			Proto_DecoderReset(d);
			return -1;
		}
		if (d->n >= PROTO_RAW_MAX)
		{
			Proto_DecoderReset(d);
			return -1;
		}
		d->raw[d->n++] = b;
		return 0;

	case PROTO_RX_ETX2:
		if (b == PROTO_ETX2)
		{
			int8_t ok = dec_finish(d, f);
			Proto_DecoderReset(d);
			return ok;
		}
		if (b == PROTO_STX)
		{
			d->state = (uint8_t)PROTO_RX_BODY;
			d->esc   = 0;
			d->n     = 0;
			return -1;
		}
		Proto_DecoderReset(d);
		return -1;

	default:
		Proto_DecoderReset(d);
		return -1;
	}
}

/* ==========================================================================
 * §B 페이로드 빌더
 * ========================================================================== */

/* 리틀엔디안(§3) 직렬화 헬퍼. 구조체 memcpy 대신 바이트 단위로 쓰는 이유는
 * 패킹/정렬 옵션에 관계없이 와이어 포맷을 코드에서 눈으로 확인하기 위해서다. */
static inline void put_u8(uint8_t *b, uint16_t i, uint8_t v)  { b[i] = v; }

static inline void put_u16(uint8_t *b, uint16_t i, uint16_t v)
{
#if PROTO_LEN_BIG_ENDIAN
	b[i]      = (uint8_t)(v >> 8);
	b[i + 1U] = (uint8_t)(v & 0xFFU);
#else
	b[i]      = (uint8_t)(v & 0xFFU);
	b[i + 1U] = (uint8_t)(v >> 8);
#endif
}

static inline void put_i16(uint8_t *b, uint16_t i, int16_t v)
{
	put_u16(b, i, (uint16_t)v);
}

static inline void put_u32(uint8_t *b, uint16_t i, uint32_t v)
{
#if PROTO_LEN_BIG_ENDIAN
	b[i]      = (uint8_t)(v >> 24);
	b[i + 1U] = (uint8_t)(v >> 16);
	b[i + 2U] = (uint8_t)(v >> 8);
	b[i + 3U] = (uint8_t)(v & 0xFFU);
#else
	b[i]      = (uint8_t)(v & 0xFFU);
	b[i + 1U] = (uint8_t)(v >> 8);
	b[i + 2U] = (uint8_t)(v >> 16);
	b[i + 3U] = (uint8_t)(v >> 24);
#endif
}

/* 온도: 써미스터 에러값(-32768)을 그대로 보내면 앱에서 잘못 표시되므로 0 으로
 * 뭉갠다. 유효성은 STATUS/DONGJAK 의 temp_valid 비트로 판단하게 한다. */
static int16_t temp_or_zero(int16_t d10)
{
	return (d10 == THERMISTOR_ERR_D10) ? (int16_t)0 : d10;
}

/* 현재 모드의 시나리오 상태 enum 값 (STATUS 바이트 2) */
static uint8_t scn_state_of_mode(void)
{
	switch (g_app_mode)
	{
	case APP_MODE_MOEUM:   return (uint8_t)g_moeum.state;
	case APP_MODE_DONGJAK: return (uint8_t)g_dongjak.state;
	default:               return 0U;
	}
}

/* 현재 모드의 시나리오가 도는 중인가 (STATUS busy / 경과시간 기준점) */
static uint8_t scn_busy(void)
{
	switch (g_app_mode)
	{
	case APP_MODE_MOEUM:   return Moeum_IsBusy();
	case APP_MODE_DONGJAK: return Dongjak_IsBusy();
	default:               return 0U;
	}
}

static uint8_t scn_err(void)
{
	if (g_app_mode == APP_MODE_MOEUM)
		return (uint8_t)(g_moeum.state == MOEUM_ERROR);
	if (g_app_mode == APP_MODE_DONGJAK)
		return (uint8_t)(g_dongjak.state == DJ_ERROR);
	return 0U;
}

/* g_proto.run_start(busy 0->1) 기준 경과 초. 미동작이면 0.
 * 모음에는 dongjak.scn_start 같은 시작 tick 이 없어 프로토콜이 자체로 잡는다. */
static uint16_t run_seconds(uint32_t now_ms)
{
	uint32_t ms;
	if (!g_proto.prev_busy)
		return 0U;
	ms = now_ms - g_proto.run_start;
	if (ms > (65535UL * 1000UL))
		return 65535U;
	return (uint16_t)(ms / 1000UL);
}

static uint16_t build_sys_info(uint8_t *b, uint32_t now_ms)
{
	uint32_t s = now_ms / 1000UL;

	put_u8 (b, 0, PROTO_REV);
	put_u8 (b, 1, PROTO_FW_MAJOR);
	put_u8 (b, 2, PROTO_FW_MINOR);
	put_u8 (b, 3, PROTO_FW_PATCH);
	put_u8 (b, 4, (uint8_t)g_app_mode);
	put_u8 (b, 5, (uint8_t)ModeArbiter_GetPos());
	put_u16(b, 6, (s > 65535UL) ? 65535U : (uint16_t)s);
	return PROTO_LEN_SYS_INFO;
}

static uint16_t build_status(uint8_t *b, uint32_t now_ms)
{
	uint8_t flags = 0;

	if (scn_busy())                       flags |= PROTO_ST_BUSY;
	if (Jungji_IsBraking())               flags |= PROTO_ST_BRAKING;
	if (Jungji_IsCooling())               flags |= PROTO_ST_COOLING;
	if (scn_err())                        flags |= PROTO_ST_ERROR;
	if (gpio_ctrl_is_on(GPIO_OUT_HT_POWER)) flags |= PROTO_ST_HEATER;
	if (g_grind_ctrl.meas_out_rpm != 0U)  flags |= PROTO_ST_GRIND;
	if (g_stir_ctrl.meas_out_rpm  != 0U)  flags |= PROTO_ST_STIR;
	if (g_app_mode == APP_MODE_MOEUM)
	{
		if (g_moeum.water_reached)        flags |= PROTO_ST_WATER;
	}
	else if (g_app_mode == APP_MODE_DONGJAK)
	{
		if (g_dongjak.water_reached)      flags |= PROTO_ST_WATER;
	}

	put_u8 (b,  0, (uint8_t)g_app_mode);
	put_u8 (b,  1, (uint8_t)ModeArbiter_GetPos());
	put_u8 (b,  2, scn_state_of_mode());
	put_u8 (b,  3, flags);
	put_u16(b,  4, run_seconds(now_ms));
	put_i16(b,  6, temp_or_zero(g_therm_c_d10[0]));
	put_i16(b,  8, temp_or_zero(g_therm_c_d10[2]));
	put_u8 (b, 10, g_bin_fill_pct);
	put_u8 (b, 11, g_hall_mask);
	put_u8 (b, 12, (g_app_mode == APP_MODE_DONGJAK) ? g_dongjak.err_code : 0U);
	put_u8 (b, 13, g_jungji.last_src);
	put_u16(b, 14, g_jungji.stop_count);
	put_u8 (b, 16, g_proto.mon_seq);
	put_u8 (b, 17, 0U);
	put_u16(b, 18, (uint16_t)(((now_ms / 1000UL) > 65535UL) ? 65535UL : (now_ms / 1000UL)));
	return PROTO_LEN_STATUS;
}

static uint16_t build_moeum(uint8_t *b, uint32_t now_ms)
{
	uint8_t flags = 0;
	uint8_t io    = 0;

	if (Moeum_IsBusy())        flags |= PROTO_MO_BUSY;
	if (g_moeum.start_req)     flags |= PROTO_MO_START_REQ;
	if (g_moeum.water_reached) flags |= PROTO_MO_WATER;
	if (g_moeum.abort_req)     flags |= PROTO_MO_ABORT_REQ;
	if (g_moeum.lid_guard)     flags |= PROTO_MO_LID_GUARD;
	if (g_moeum.stir_active)   flags |= PROTO_MO_STIR_ACTIVE;

	if (gpio_ctrl_is_on(GPIO_OUT_VALVE_DRY_IN))    io |= PROTO_MO_IO_VALVE_IN;
	if (gpio_ctrl_is_on(GPIO_OUT_WATER_ON))        io |= PROTO_MO_IO_WATER_ON;
	if (gpio_ctrl_is_on(GPIO_OUT_EN_DOOR_WATER))   io |= PROTO_MO_IO_DOOR_EN;
	if (WDoor_AtOpen())                            io |= PROTO_MO_IO_WHALL_OPEN;
	if (WDoor_AtClose())                           io |= PROTO_MO_IO_WHALL_CLOSE;
	if (gpio_ctrl_is_on(GPIO_OUT_VALVE_DRAIN_CLN)) io |= PROTO_MO_IO_DRAIN_CLN;

	put_u8 (b,  0, (uint8_t)g_moeum.state);
	put_u8 (b,  1, flags);
	put_u8 (b,  2, g_moeum.stir_phase);
	put_u8 (b,  3, io);
	put_u32(b,  4, now_ms - g_moeum.state_since);
	put_u16(b,  8, g_moeum.stir_cycles);
	put_u16(b, 10, (uint16_t)MOEUM_STIR_CYCLES);
	put_u16(b, 12, g_stir_ctrl.meas_out_rpm);
	put_u16(b, 14, run_seconds(now_ms));
	return PROTO_LEN_MOEUM;
}

static uint16_t build_dongjak(uint8_t *b, uint32_t now_ms)
{
	uint8_t flags = 0;
	uint8_t io    = 0;

	if (Dongjak_IsBusy())        flags |= PROTO_DJ_BUSY;
	if (g_dongjak.heat_started)  flags |= PROTO_DJ_HEAT_STARTED;
	if (g_dongjak.temp_valid)    flags |= PROTO_DJ_TEMP_VALID;
	if (g_dongjak.water_reached) flags |= PROTO_DJ_WATER;
	if (g_dongjak.abort_req)     flags |= PROTO_DJ_ABORT_REQ;
	if (g_dongjak.lid_guard)     flags |= PROTO_DJ_LID_GUARD;
	if (g_dongjak.fanb_on)       flags |= PROTO_DJ_FAN_BLDC;
	if (g_dongjak.fanx_on)       flags |= PROTO_DJ_FAN_EXHAUST;

	if (gpio_ctrl_is_on(GPIO_OUT_HT_POWER))        io |= PROTO_DJ_IO_HEATER;
	if (gpio_ctrl_is_on(GPIO_OUT_VALVE_DRY_IN))    io |= PROTO_DJ_IO_VALVE_IN;
	if (gpio_ctrl_is_on(GPIO_OUT_VALVE_DRAIN_CLN)) io |= PROTO_DJ_IO_DRAIN_CLN;
	if (gpio_ctrl_is_on(GPIO_OUT_FAN_VAPOR))       io |= PROTO_DJ_IO_FAN_VAPOR;
	if (gpio_ctrl_is_on(GPIO_OUT_WATER_ON))        io |= PROTO_DJ_IO_WATER_ON;
	if (TDoor_AtOpen())                            io |= PROTO_DJ_IO_TDOOR_OPEN;
	if (TDoor_AtClose())                           io |= PROTO_DJ_IO_TDOOR_CLOSE;
	if (WDoor_AtClose())                           io |= PROTO_DJ_IO_WDOOR_CLOSE;

	put_u8 (b,  0, (uint8_t)g_dongjak.state);
	put_u8 (b,  1, flags);
	put_u8 (b,  2, g_dongjak.err_code);
	put_u8 (b,  3, g_dongjak.grind_mode);
	put_u8 (b,  4, g_dongjak.stir_phase);
	put_u8 (b,  5, g_dongjak.vapor_phase);
	put_u8 (b,  6, g_dongjak.disc_phase);
	put_u8 (b,  7, g_dongjak.cool_phase);
	/* IDLE 이면 scn_start 가 지난 회차 값이라 경과가 무의미하므로 0 을 보낸다. */
	put_u32(b,  8, Dongjak_IsBusy() ? (now_ms - g_dongjak.scn_start) : 0UL);
	put_u32(b, 12, now_ms - g_dongjak.state_since);
	put_i16(b, 16, g_dongjak.temp_d10);
	put_i16(b, 18, g_dongjak.vapor_temp_d10);
	put_u16(b, 20, g_grind_ctrl.meas_out_rpm);
	put_u16(b, 22, g_stir_ctrl.meas_out_rpm);
	put_u8 (b, 24, g_dongjak.bin_fill_pct);
	put_u8 (b, 25, io);
	put_u16(b, 26, g_dongjak.cycle_count);
	return PROTO_LEN_DONGJAK;
}

static uint16_t build_sensor(uint8_t *b)
{
	uint8_t d1 = 0, d2 = 0, lim = 0, tv = 0;

	/* PF0~PF7 */
	if (gpio_ctrl_exti_read(GPIO_EXTI_BIMETAL_80))  d1 |= PROTO_SEN1_BIMETAL_80;
	if (gpio_ctrl_exti_read(GPIO_EXTI_BIMETAL_60))  d1 |= PROTO_SEN1_BIMETAL_60;
	if (gpio_ctrl_exti_read(GPIO_EXTI_THALL_CLOSE)) d1 |= PROTO_SEN1_THALL_CLOSE;
	if (gpio_ctrl_exti_read(GPIO_EXTI_WHALL_CLOSE)) d1 |= PROTO_SEN1_WHALL_CLOSE;
	if (gpio_ctrl_exti_read(GPIO_EXTI_WHALL_OPEN))  d1 |= PROTO_SEN1_WHALL_OPEN;
	if (gpio_ctrl_exti_read(GPIO_EXTI_THALL_OPEN))  d1 |= PROTO_SEN1_THALL_OPEN;
	if (gpio_ctrl_exti_read(GPIO_EXTI_WATER_SEN1))  d1 |= PROTO_SEN1_WATER_SEN1;
	if (gpio_ctrl_exti_read(GPIO_EXTI_WATER_SEN2))  d1 |= PROTO_SEN1_WATER_SEN2;
	/* PF8~PF15 + BLE 상태 */
	if (gpio_ctrl_exti_read(GPIO_EXTI_TIMER_OUT))   d2 |= PROTO_SEN2_TIMER_OUT;
	if (gpio_ctrl_exti_read(GPIO_EXTI_HALL_INT1))   d2 |= PROTO_SEN2_HALL_INT1;
	if (gpio_ctrl_exti_read(GPIO_EXTI_HALL_INT2))   d2 |= PROTO_SEN2_HALL_INT2;
	if (gpio_ctrl_exti_read(GPIO_EXTI_M2_FGOT))     d2 |= PROTO_SEN2_M2_FGOUT;
	if (gpio_ctrl_exti_read(GPIO_EXTI_M2_nFAULT))   d2 |= PROTO_SEN2_M2_NFAULT;
	if (gpio_ctrl_exti_read(GPIO_EXTI_M1_nFAULT))   d2 |= PROTO_SEN2_M1_NFAULT;
	if (gpio_ctrl_exti_read(GPIO_EXTI_M1_FGOT))     d2 |= PROTO_SEN2_M1_FGOUT;
	if (gpio_ctrl_read(GPIO_IN_BLE_STATUS))         d2 |= PROTO_SEN2_BLE_STATUS;
	/* 드라이버가 극성까지 디코드한 리미트 도달 여부(원시 레벨은 din1 에 있다) */
	if (WDoor_AtOpen())                            lim |= PROTO_LIM_WDOOR_OPEN;
	if (WDoor_AtClose())                           lim |= PROTO_LIM_WDOOR_CLOSE;
	if (TDoor_AtOpen())                            lim |= PROTO_LIM_TDOOR_OPEN;
	if (TDoor_AtClose())                           lim |= PROTO_LIM_TDOOR_CLOSE;
	/* 써미스터 유효성: 온도 필드는 에러를 0 으로 뭉개므로 이 비트로 구분한다 */
	for (uint8_t i = 0; i < 3U; i++)
	{
		if (g_therm_c_d10[i] != THERMISTOR_ERR_D10)
			tv = (uint8_t)(tv | (1U << i));
	}

	put_i16(b,  0, temp_or_zero(g_therm_c_d10[0]));
	put_i16(b,  2, temp_or_zero(g_therm_c_d10[1]));
	put_i16(b,  4, temp_or_zero(g_therm_c_d10[2]));
	put_u16(b,  6, g_distance_mm);
	put_u8 (b,  8, g_bin_fill_pct);
	put_u8 (b,  9, g_hall_mask);
	put_u8 (b, 10, d1);
	put_u8 (b, 11, d2);
	put_u8 (b, 12, lim);
	put_u8 (b, 13, tv);
	put_u8 (b, 14, 0U);
	put_u8 (b, 15, 0U);
	return PROTO_LEN_SENSOR;
}

/* ==========================================================================
 * 0x25 MOTOR
 *
 * 앱이 "1000RPM CW, 구동 3초 / 정지 2초, 현재 구동구간 1.4초 경과" 처럼 그리려면
 * RPM/방향(드라이버가 아는 값)과 구간 패턴(시나리오가 아는 값)을 함께 실어야 한다.
 *
 * RPM·방향·running·duty 는 BldcCtrl_t 에서 그대로 읽는다(실측/지령 모두 있음).
 * 구간 길이(on_ms/off_ms)는 시나리오가 쓰는 컴파일 상수를 "현재 상태 -> 패턴"
 * 으로 골라 실어 준다. 상수 자체는 moeum.h/dongjak.h 의 매크로를 직접 참조하므로
 * 값이 복제되지는 않는다. 복제되는 것은 "어느 상태가 어느 패턴인가" 의 매핑뿐이고,
 * 이는 doc/R1/시나리오_상태별_동작표.xlsx 의 교반·분쇄 열과 1:1 이다.
 * ★시나리오의 구간 패턴을 바꾸면 아래 두 함수도 함께 고칠 것.
 * ========================================================================== */

typedef struct
{
	uint8_t  pattern;    /* PROTO_PAT_*                                     */
	uint8_t  phase;      /* 구간 코드                                        */
	uint8_t  rep;        /* 반복 회차                                        */
	uint16_t on_ms;      /* 구동 구간 길이(0 = 연속)                         */
	uint16_t off_ms;     /* 정지 구간 길이(0 = 없음)                         */
	uint32_t since;      /* 현 구간 시작 tick                                */
} MotorPattern;

/* 교반 M2 의 현재 패턴. 상태표 "교반 M2" 열과 대응. */
static void stir_pattern(MotorPattern *p)
{
	p->pattern = PROTO_PAT_OFF;
	p->phase = 0; p->rep = 0; p->on_ms = 0; p->off_ms = 0; p->since = 0;

	if (g_app_mode == APP_MODE_MOEUM)
	{
		if (!g_moeum.stir_active)
			return;
		/* 모음: CW 3s / 정지 1s / CCW 3s = 1cycle 7s (moeum.h) */
		p->pattern = PROTO_PAT_TRI;
		p->phase   = g_moeum.stir_phase;
		p->rep     = (uint8_t)(g_moeum.stir_cycles & 0xFFU);
		p->on_ms   = (uint16_t)MOEUM_STIR_CCW_MS;
		p->off_ms  = (uint16_t)MOEUM_STIR_DELAY_MS;
		p->since   = g_moeum.stir_phase_since;
		return;
	}
	if (g_app_mode != APP_MODE_DONGJAK)
		return;

	p->phase = g_dongjak.stir_phase;
	p->since = g_dongjak.stir_since;

	switch (g_dongjak.state)
	{
	/* 헹굼 교반 = 식힘/배출과 같은 313 패턴 (CW3 / 정지1 / CCW3) */
	case DJ_RINSE1_STIR: case DJ_RINSE1_DRAIN:
	case DJ_RINSE2_STIR: case DJ_RINSE2_DRAIN:
		p->pattern = PROTO_PAT_TRI;
		p->on_ms   = (uint16_t)DJ_S313_CW_MS;
		p->off_ms  = (uint16_t)DJ_S313_STOP_MS;
		break;
	/* 건조 교반 = CW 3s + 정지 2s 를 5회 반복 후 CCW 3s (신스펙) */
	case DJ_HEAT:
		p->pattern = PROTO_PAT_TRI;
		p->rep     = g_dongjak.stir_reps;
		p->on_ms   = (uint16_t)DJ_STIR_FWD_MS;
		p->off_ms  = (uint16_t)DJ_STIR_STOP_MS;
		break;
	/* 식힘: 뜨거울 때는 CW 연속, 식으면 313 패턴 */
	case DJ_COOLDOWN:
		if (g_dongjak.cool_phase == 0U)
		{
			p->pattern = PROTO_PAT_CONT;
		}
		else
		{
			p->pattern = PROTO_PAT_TRI;
			p->on_ms   = (uint16_t)DJ_S313_CW_MS;
			p->off_ms  = (uint16_t)DJ_S313_STOP_MS;
		}
		break;
	/* 배출 교반 = 313 패턴 (배출 위상 EXPEL 구간에서만) */
	case DJ_DISCHARGE:
		p->pattern = PROTO_PAT_TRI;
		p->on_ms   = (uint16_t)DJ_S313_CW_MS;
		p->off_ms  = (uint16_t)DJ_S313_STOP_MS;
		break;
	default:
		break;
	}
	/* 지령이 0 이면 실제로는 돌지 않는 구간이다 */
	if (g_stir_ctrl.target_out_rpm == 0U && !g_stir_ctrl.running)
		p->pattern = PROTO_PAT_OFF;
}

/* 분쇄 M1 의 현재 패턴. 상태표 "분쇄 M1" 열과 대응. 모음에는 분쇄가 없다. */
static void grind_pattern(MotorPattern *p)
{
	p->pattern = PROTO_PAT_OFF;
	p->phase = 0; p->rep = 0; p->on_ms = 0; p->off_ms = 0; p->since = 0;

	if (g_app_mode != APP_MODE_DONGJAK)
		return;

	p->phase = g_dongjak.grind_phase;
	p->since = g_dongjak.grind_since;

	switch (g_dongjak.grind_mode)
	{
	case DJ_GM_COARSE:   /* 1차 거친 분쇄 1500 CW, 구동 3s / 정지 2s */
		p->pattern = PROTO_PAT_TOGGLE;
		p->on_ms   = (uint16_t)DJ_GRIND_RUN_MS;
		p->off_ms  = (uint16_t)DJ_GRIND_STOP_MS;
		break;
	case DJ_GM_FINE:     /* 연속 분쇄 1000 CW */
		p->pattern = PROTO_PAT_CONT;
		break;
	case DJ_GM_FINAL:    /* 110분↑ 2000 CCW, 구동 4s / 정지 2s */
		p->pattern = PROTO_PAT_TOGGLE;
		p->on_ms   = (uint16_t)DJ_GRIND_RUN_FINAL_MS;
		p->off_ms  = (uint16_t)DJ_GRIND_STOP_FINAL_MS;
		break;
	case DJ_GM_COOL:     /* 식힘 중 1000 CCW 연속 */
		p->pattern = PROTO_PAT_CONT;
		break;
	case DJ_GM_OFF:
	default:
		break;
	}
}

/* 경과 ms 를 u16 으로 클램프. since==0(패턴 없음)이면 0. */
static uint16_t phase_elapsed(uint32_t since, uint32_t now_ms)
{
	uint32_t d;
	if (since == 0UL)
		return 0U;
	d = now_ms - since;
	return (d > 65535UL) ? 65535U : (uint16_t)d;
}

/* BLDC 1대를 20바이트 블록에 직렬화 */
static void put_bldc(uint8_t *b, uint16_t off, const BldcCtrl_t *c,
                     const MotorPattern *p, uint32_t now_ms)
{
	put_u8 (b, off + PROTO_MOT_RUNNING,   c->running);
	put_u8 (b, off + PROTO_MOT_STATE,     c->state);
	put_u8 (b, off + PROTO_MOT_DIR,       c->reverse);
	put_u8 (b, off + PROTO_MOT_FAULT,     c->fault);
	put_u16(b, off + PROTO_MOT_TARGET,    c->target_out_rpm);
	put_u16(b, off + PROTO_MOT_SETPOINT,  c->sp_out_rpm);
	put_u16(b, off + PROTO_MOT_MEASURED,  c->meas_out_rpm);
	put_i16(b, off + PROTO_MOT_DUTY,      c->duty_pm);
	put_u8 (b, off + PROTO_MOT_PHASE,     p->phase);
	put_u8 (b, off + PROTO_MOT_PHASE_REP, p->rep);
	put_u16(b, off + PROTO_MOT_ON_MS,     p->on_ms);
	put_u16(b, off + PROTO_MOT_OFF_MS,    p->off_ms);
	put_u16(b, off + PROTO_MOT_PHASE_MS,  phase_elapsed(p->since, now_ms));
}

static uint16_t build_motor(uint8_t *b, uint32_t now_ms)
{
	MotorPattern m1, m2;

	grind_pattern(&m1);
	stir_pattern(&m2);

	put_bldc(b, PROTO_MOTOR_OFF_M1, &g_grind_ctrl, &m1, now_ms);
	put_bldc(b, PROTO_MOTOR_OFF_M2, &g_stir_ctrl,  &m2, now_ms);
	put_u8 (b, 40, m1.pattern);
	put_u8 (b, 41, m2.pattern);
	return PROTO_LEN_MOTOR;
}

/* ==========================================================================
 * 0x26 OUTPUT
 * ========================================================================== */

/* 스테퍼 통전 여부: 4상 중 하나라도 HIGH 면 통전 중. 코일 핀이 gpio_ctrl 출력이라
 * 래치를 그대로 읽으면 되고, 별도 드라이버 API 가 필요 없다(무여자 = 전부 LOW). */
static uint8_t step_energized(gpio_out_t m1, gpio_out_t m2,
                              gpio_out_t m3, gpio_out_t m4)
{
	return (uint8_t)(gpio_ctrl_is_on(m1) || gpio_ctrl_is_on(m2) ||
	                 gpio_ctrl_is_on(m3) || gpio_ctrl_is_on(m4));
}

static uint16_t build_output(uint8_t *b, uint32_t now_ms)
{
	uint8_t o1 = 0, o2 = 0, o3 = 0, dc = 0;
	uint32_t fanx_el, fanb_el;

	if (gpio_ctrl_is_on(GPIO_OUT_HT_POWER))        o1 |= PROTO_OUT1_HEATER;
	if (gpio_ctrl_is_on(GPIO_OUT_VALVE_DRY_IN))    o1 |= PROTO_OUT1_VALVE_IN;
	if (gpio_ctrl_is_on(GPIO_OUT_WATER_ON))        o1 |= PROTO_OUT1_WATER_ON;
	if (gpio_ctrl_is_on(GPIO_OUT_VALVE_DRAIN_CLN)) o1 |= PROTO_OUT1_DRAIN_CLN;
	if (gpio_ctrl_is_on(GPIO_OUT_FAN_VAPOR))       o1 |= PROTO_OUT1_FAN_VAPOR;
	if (gpio_ctrl_is_on(GPIO_OUT_FAN_EXHAUST))     o1 |= PROTO_OUT1_FAN_EXHAUST;
	if (gpio_ctrl_is_on(GPIO_OUT_BLDC_FAN))        o1 |= PROTO_OUT1_FAN_BLDC;
	if (gpio_ctrl_is_on(GPIO_OUT_EN_SPK))          o1 |= PROTO_OUT1_SPK_EN;

	if (gpio_ctrl_is_on(GPIO_OUT_EN_DOOR_WATER))   o2 |= PROTO_OUT2_WDOOR_VM;
	if (gpio_ctrl_is_on(GPIO_OUT_EN_DOOR_TRASH))   o2 |= PROTO_OUT2_TDOOR_VM;
	if (gpio_ctrl_is_on(GPIO_OUT_MTR_DC_LIFT))     o2 |= PROTO_OUT2_LIFT_VM;
	if (gpio_ctrl_is_on(GPIO_OUT_M1_ENABLE))       o2 |= PROTO_OUT2_M1_ENABLE;
	if (gpio_ctrl_is_on(GPIO_OUT_M2_ENABLE))       o2 |= PROTO_OUT2_M2_ENABLE;
	if (gpio_ctrl_is_on(GPIO_OUT_M1_nBRAKE))       o2 |= PROTO_OUT2_M1_NBRAKE;
	if (gpio_ctrl_is_on(GPIO_OUT_M2_nBRAKE))       o2 |= PROTO_OUT2_M2_NBRAKE;
	if (step_energized(GPIO_OUT_STEP1_M1, GPIO_OUT_STEP1_M2,
	                   GPIO_OUT_STEP1_M3, GPIO_OUT_STEP1_M4))
		o2 |= PROTO_OUT2_STEP1_ON;

	if (step_energized(GPIO_OUT_STEP2_M1, GPIO_OUT_STEP2_M2,
	                   GPIO_OUT_STEP2_M3, GPIO_OUT_STEP2_M4))
		o3 |= PROTO_OUT3_STEP2_ON;
	if (gpio_ctrl_is_on(GPIO_OUT_BLE_MODE))        o3 |= PROTO_OUT3_BLE_MODE;

	/* DC 3대: drv8871_drive_t 를 2비트씩 (0 코스트/1 정회전/2 역회전/3 제동) */
	dc = (uint8_t)(( WDoor_GetDrive()       & 0x03U) |
	               ((TDoor_GetDrive() & 0x03U) << 2) |
	               ((Lift_GetDrive()  & 0x03U) << 4));

	/* 팬 duty 구간. 배기팬은 15분/2분이라 ms 로는 u16 을 넘겨 초 단위로 싣는다.
	 * fanx_since/fanb_since 는 동작 시나리오만 갱신하므로 다른 모드에서는 0. */
	fanx_el = (g_app_mode == APP_MODE_DONGJAK && g_dongjak.fanx_since != 0UL)
	          ? ((now_ms - g_dongjak.fanx_since) / 1000UL) : 0UL;
	fanb_el = (g_app_mode == APP_MODE_DONGJAK && g_dongjak.fanb_since != 0UL)
	          ? (now_ms - g_dongjak.fanb_since) : 0UL;

	put_u8 (b,  0, o1);
	put_u8 (b,  1, o2);
	put_u8 (b,  2, o3);
	put_u8 (b,  3, dc);
	put_u8 (b,  4, WDoor_GetDuty());
	put_u8 (b,  5, TDoor_GetDuty());
	put_u16(b,  6, (fanx_el > 65535UL) ? 65535U : (uint16_t)fanx_el);
	put_u16(b,  8, (uint16_t)(DJ_FANX_ON_MS  / 1000UL));
	put_u16(b, 10, (uint16_t)(DJ_FANX_OFF_MS / 1000UL));
	put_u16(b, 12, (fanb_el > 65535UL) ? 65535U : (uint16_t)fanb_el);
	put_u8 (b, 14, (uint8_t)(DJ_FANB_ON_MS  / 1000UL));
	put_u8 (b, 15, (uint8_t)(DJ_FANB_OFF_MS / 1000UL));
	return PROTO_LEN_OUTPUT;
}

static uint16_t build_jungji(uint8_t *b)
{
	uint8_t flags = 0;

	if (g_jungji.braking) flags |= PROTO_JG_BRAKING;
	if (g_jungji.cooling) flags |= PROTO_JG_COOLING;

	put_u8 (b, 0, g_jungji.req);
	put_u8 (b, 1, g_jungji.req_src);
	put_u8 (b, 2, g_jungji.req_kind);
	put_u8 (b, 3, g_jungji.last_src);
	put_u8 (b, 4, g_jungji.last_kind);
	put_u8 (b, 5, flags);
	put_u16(b, 6, g_jungji.stop_count);
	put_i16(b, 8, g_jungji.temp_d10);
	return PROTO_LEN_JUNGJI;
}

static uint16_t build_mon_cfg(uint8_t *b)
{
	put_u8 (b, 0, g_proto.mon_mask);
	put_u16(b, 1, g_proto.mon_period_ms);
	return PROTO_LEN_MON_CFG;
}

static uint16_t build_control(uint8_t *b)
{
	/* 래치가 소비되었는지: 중재자의 pend_valid/start_wait 가 내려갔으면 적용됨. */
	uint8_t pending = (uint8_t)((g_modearb.pend_valid != 0U) ||
	                           (g_modearb.start_wait != 0U));
	put_u8(b, 0, g_proto.act_last);
	put_u8(b, 1, g_proto.act_result);
	put_u8(b, 2, pending);
	put_u8(b, 3, g_proto.last_nak);
	return PROTO_LEN_CONTROL;
}

uint16_t Proto_BuildPayload(uint8_t cmd, uint8_t *buf, uint16_t buf_sz)
{
	uint32_t now = HAL_GetTick();

	if (buf == NULL)
		return 0;

	switch ((proto_cmd_t)cmd)
	{
	case PROTO_CMD_SYS_INFO:
		return (buf_sz < PROTO_LEN_SYS_INFO) ? 0U : build_sys_info(buf, now);
	case PROTO_CMD_STATUS:
		return (buf_sz < PROTO_LEN_STATUS)   ? 0U : build_status(buf, now);
	case PROTO_CMD_MOEUM:
		return (buf_sz < PROTO_LEN_MOEUM)    ? 0U : build_moeum(buf, now);
	case PROTO_CMD_DONGJAK:
		return (buf_sz < PROTO_LEN_DONGJAK)  ? 0U : build_dongjak(buf, now);
	case PROTO_CMD_SENSOR:
		return (buf_sz < PROTO_LEN_SENSOR)   ? 0U : build_sensor(buf);
	case PROTO_CMD_JUNGJI:
		return (buf_sz < PROTO_LEN_JUNGJI)   ? 0U : build_jungji(buf);
	case PROTO_CMD_MOTOR:
		return (buf_sz < PROTO_LEN_MOTOR)    ? 0U : build_motor(buf, now);
	case PROTO_CMD_OUTPUT:
		return (buf_sz < PROTO_LEN_OUTPUT)   ? 0U : build_output(buf, now);
	case PROTO_CMD_MON_CFG:
		return (buf_sz < PROTO_LEN_MON_CFG)  ? 0U : build_mon_cfg(buf);
	case PROTO_CMD_CONTROL:
		return (buf_sz < PROTO_LEN_CONTROL)  ? 0U : build_control(buf);
	default:
		return 0;
	}
}

/* ==========================================================================
 * 모니터링 주기 하한 계산
 *
 * 켜진 패킷들의 "최선 와이어 바이트 합"(이스케이프 0개 가정)으로 한 주기 버스트의
 * 전송 시간을 구하고 여유계수를 붙인다. 최선값을 쓰는 이유: 최악(전 바이트
 * 이스케이프)은 실측에서 거의 나오지 않는데 그것으로 하한을 잡으면 쓸 수 있는
 * 주기가 지나치게 느려진다. 대신 실제로 밀릴 때는 line_backlogged() 가 주기를
 * 건너뛰어 회선을 보호한다.
 * ========================================================================== */
uint16_t Proto_MonMinPeriodMs(uint8_t mask)
{
	/* 프레임 오버헤드: STX(1) + R/W(M 이므로 이스케이프되어 2) + CMD(1) + LEN(2) + ETX(2) */
	const uint32_t OVH = 8UL;
	uint32_t bytes = 0;
	uint32_t ms;

	if (mask & PROTO_MON_STATUS)  bytes += OVH + PROTO_LEN_STATUS;
	if (mask & PROTO_MON_MOEUM)   bytes += OVH + PROTO_LEN_MOEUM;
	if (mask & PROTO_MON_DONGJAK) bytes += OVH + PROTO_LEN_DONGJAK;
	if (mask & PROTO_MON_SENSOR)  bytes += OVH + PROTO_LEN_SENSOR;
	if (mask & PROTO_MON_JUNGJI)  bytes += OVH + PROTO_LEN_JUNGJI;
	if (mask & PROTO_MON_MOTOR)   bytes += OVH + PROTO_LEN_MOTOR;
	if (mask & PROTO_MON_OUTPUT)  bytes += OVH + PROTO_LEN_OUTPUT;

	if (bytes == 0UL)
		return PROTO_MON_MIN_MS;

	/* 8N1 = 10 bit/byte */
	ms = (bytes * 10UL * 1000UL) / PROTO_UART_BAUD;
	ms = (ms * PROTO_MON_HEADROOM_PCT) / 100UL;

	if (ms < PROTO_MON_MIN_MS)
		ms = PROTO_MON_MIN_MS;
	return (ms > 65535UL) ? 65535U : (uint16_t)ms;
}

/* ==========================================================================
 * §C 송신 + 명령 처리
 * ========================================================================== */

uint8_t Proto_Send(uint8_t type, uint8_t cmd, const uint8_t *data, uint16_t len)
{
	uint16_t n = Proto_Encode(type, cmd, data, len, s_wire, (uint16_t)sizeof(s_wire));

	if (n == 0U || !UartCtrl_SendFrame(s_wire, n))
	{
		g_proto.tx_fail++;
		return 0;
	}
	g_proto.tx_frames++;
	if (type == (uint8_t)PROTO_TYPE_M)
		g_proto.mon_frames++;
	return 1;
}

uint8_t Proto_SendAck(uint8_t cmd, uint8_t ok)
{
	uint8_t v = ok ? (uint8_t)PROTO_ACK : (uint8_t)PROTO_NAK;
	return Proto_Send((uint8_t)PROTO_TYPE_W, cmd, &v, 1U);
}

uint8_t Proto_SendPayload(uint8_t type, uint8_t cmd)
{
	uint16_t n = Proto_BuildPayload(cmd, s_payload, (uint16_t)sizeof(s_payload));

	if (n == 0U)
		return 0;
	return Proto_Send(type, cmd, s_payload, n);
}

/* ---- R : 요청 응답 (R | CMD | N | DATA) --------------------------------- */
static void handle_read(const ProtoFrame *f)
{
	if (!Proto_SendPayload((uint8_t)PROTO_TYPE_R, f->cmd))
	{
		/* 미지원 CMD 는 R 로 답할 페이로드가 없다. 규격에 R 용 에러 표현이
		 * 없으므로 W 응답 형식의 NAK 로 알린다(앱에서 "그 CMD 없음"으로 처리). */
		g_proto.last_nak = (uint8_t)PROTO_NAK_UNKNOWN_CMD;
		(void)Proto_SendAck(f->cmd, 0U);
	}
}

/* ---- 0x31 CONTROL : 시나리오 시작/정지 ---------------------------------
 * ★이 함수는 defaultTask(100ms) 문맥이다. 액추에이터를 절대 직접 만지지 않고
 *   요청만 래치한다:
 *     시작 -> ModeArbiter_RequestMode()  (pend_* 래치, MotorTask 가 적용)
 *     정지 -> Jungji_Request()           (래치만, MotorTask 의 Jungji_Tick 이 실행)
 *   모터 소유자는 MotorTask 이므로, 여기서 직접 구동하면 두 태스크가 같은 모터를
 *   놓고 경합한다. protocol_r0.h §6 의 규칙이 바로 이것이다. */
static uint8_t handle_control(const ProtoFrame *f)
{
	uint8_t act;

	if (f->len != PROTO_LEN_CONTROL_W)
	{
		g_proto.last_nak = (uint8_t)PROTO_NAK_BAD_LEN;
		return 0U;
	}
	/* 오조작 방지 매직. 이 명령 하나가 135분 가열 사이클을 시작시킨다. */
	if (f->data[1] != PROTO_CTRL_MAGIC)
	{
		g_proto.last_nak = (uint8_t)PROTO_NAK_BAD_VALUE;
		return 0U;
	}

	act = f->data[0];
	g_proto.act_last = act;

	switch ((proto_action_t)act)
	{
	case PROTO_ACT_NONE:
		return 1U;                                  /* 핑: 접수만 */

	case PROTO_ACT_MOEUM:
		if (!ModeArbiter_RequestMode(APP_MODE_MOEUM, 1U, 0U))
		{
			g_proto.last_nak = (uint8_t)PROTO_NAK_BAD_VALUE;
			return 0U;
		}
		return 1U;

	case PROTO_ACT_DONGJAK:
		if (!ModeArbiter_RequestMode(APP_MODE_DONGJAK, 1U, 0U))
		{
			g_proto.last_nak = (uint8_t)PROTO_NAK_BAD_VALUE;
			return 0U;
		}
		return 1U;

	case PROTO_ACT_DJ_HEAT:
	case PROTO_ACT_DJ_HEAT_ND:
		/* 헹굼을 건너뛰고 DJ_HEAT 부터. 여기서는 요청만 남긴다 - 이 함수는
		 * defaultTask 문맥이고 모터/시나리오 소유자는 MotorTask 다. 중재자가
		 * 모드 전환과 제동 해제를 끝낸 뒤에 트리거를 세운다(모드 전환 정리가
		 * g_dongjak.dbg_enter_heat 를 지우기 때문). */
		if (!ModeArbiter_RequestDongjakHeat(
		        (uint8_t)(act == (uint8_t)PROTO_ACT_DJ_HEAT_ND)))
		{
			g_proto.last_nak = (uint8_t)PROTO_NAK_BAD_VALUE;
			return 0U;
		}
		return 1U;

	case PROTO_ACT_STOP:
		/* 비상정지 + 대기(TESTBENCH) 모드로 복귀. 정지는 jungji 단일 모듈이
		 * 담당하므로 여기서는 요청만 남긴다(등급은 강한 쪽이 이긴다). */
		Jungji_Request(JUNGJI_SRC_APP, JUNGJI_KIND_EMERGENCY);
		(void)ModeArbiter_RequestMode(APP_MODE_TESTBENCH, 0U, 1U);
		return 1U;

	case PROTO_ACT_BAESU:
	case PROTO_ACT_KANGEUM:
		/* TODO: 배수(baesu.c) / 강음(kangeum.c) 시나리오 미구현 스텁.
		 * 코드는 배치해 두지만 동작하지 않아야 하므로 NAK 한다. 구현이 끝나면
		 * PROTO_CTRL_ALLOW_STUB=1 로 열거나 이 case 를 위와 같은 형태로 바꾼다. */
#if PROTO_CTRL_ALLOW_STUB
		if (!ModeArbiter_RequestMode(
		        (act == (uint8_t)PROTO_ACT_BAESU) ? APP_MODE_BAESU : APP_MODE_KANGEUM,
		        1U, 1U))
		{
			g_proto.last_nak = (uint8_t)PROTO_NAK_BAD_VALUE;
			return 0U;
		}
		return 1U;
#else
		g_proto.last_nak = (uint8_t)PROTO_NAK_NOT_WRITABLE;   /* 미구현 */
		return 0U;
#endif

	case PROTO_ACT_CLEAR_ERR:
		/* err_clear_req 는 dongjak 이 1회성으로 소비하는 래치다. */
		g_dongjak.err_clear_req = 1U;
		return 1U;

	default:
		g_proto.last_nak = (uint8_t)PROTO_NAK_BAD_VALUE;
		return 0U;
	}
}

/* ---- W : 쓰기 + 되읽기 검증 후 ACK|NAK (xlsx §2 마지막 줄) -------------- */
static void handle_write(const ProtoFrame *f)
{
	uint8_t  ok = 0;
	uint16_t period;
	uint8_t  mask;

	switch ((proto_cmd_t)f->cmd)
	{
	case PROTO_CMD_CONTROL:
		ok = handle_control(f);
		g_proto.act_result = (uint8_t)(ok ? 0U : 1U);
		if (ok)
		{
			g_proto.act_count++;
			g_proto.act_pending = 1U;
		}
		break;

	case PROTO_CMD_MON_CFG:
		if (f->len != PROTO_LEN_MON_CFG)
		{
			g_proto.last_nak = (uint8_t)PROTO_NAK_BAD_LEN;
			break;
		}
		mask = f->data[0];
#if PROTO_LEN_BIG_ENDIAN
		period = (uint16_t)(((uint16_t)f->data[1] << 8) | f->data[2]);
#else
		period = (uint16_t)(((uint16_t)f->data[2] << 8) | f->data[1]);
#endif
		if ((mask & (uint8_t)~PROTO_MON_ALL) != 0U)
		{
			g_proto.last_nak = (uint8_t)PROTO_NAK_BAD_VALUE;
			break;
		}
		/* 0 = 주기송신 정지. 그 외에는 "이 마스크를 이 보율로 보낼 수 있는가"
		 * 를 검사한다. 마스크가 넓어지면 하한이 자동으로 커진다. */
		if (period != 0U && period < Proto_MonMinPeriodMs(mask))
		{
			g_proto.last_nak = (uint8_t)PROTO_NAK_BAD_VALUE;
			break;
		}
		g_proto.mon_mask      = mask;
		g_proto.mon_period_ms = period;

		/* §2 규칙: 쓴 뒤 "다시 읽어서" 비교한다.
		 * 변수를 방금 넣은 지역변수와 비교하면 항상 참인 동어반복이므로, R 응답과
		 * 똑같은 직렬화 경로(build_mon_cfg)로 되읽어 수신 DATA 바이트와 그대로
		 * 비교한다. 이렇게 하면 저장값뿐 아니라 와이어 표현(엔디안/자리)까지
		 * 검증되고, 앱이 R 로 다시 조회했을 때 볼 값과 ACK 가 일치한다. */
		{
			uint8_t back[PROTO_LEN_MON_CFG];
			uint16_t n = build_mon_cfg(back);
			ok = (uint8_t)((n == PROTO_LEN_MON_CFG) &&
			               (memcmp(back, f->data, PROTO_LEN_MON_CFG) == 0));
		}
		if (!ok)
			g_proto.last_nak = (uint8_t)PROTO_NAK_VERIFY_FAIL;
		break;

	case PROTO_CMD_SYS_INFO:
	case PROTO_CMD_STATUS:
	case PROTO_CMD_MOEUM:
	case PROTO_CMD_DONGJAK:
	case PROTO_CMD_SENSOR:
	case PROTO_CMD_JUNGJI:
	case PROTO_CMD_MOTOR:
	case PROTO_CMD_OUTPUT:
		/* 모니터링 계열은 읽기 전용이다. 시나리오를 제어하는 W 명령을 추가할
		 * 때는 여기서 액추에이터를 직접 만지지 말고 요청 플래그만 세울 것
		 * (이 함수는 defaultTask 문맥이고 모터 소유자는 MotorTask 다). */
		g_proto.last_nak = (uint8_t)PROTO_NAK_NOT_WRITABLE;
		break;

	default:
		g_proto.last_nak = (uint8_t)PROTO_NAK_UNKNOWN_CMD;
		break;
	}

	(void)Proto_SendAck(f->cmd, ok);
}

static void dispatch(const ProtoFrame *f)
{
	g_proto.rx_frames++;
	g_proto.last_type = f->type;
	g_proto.last_cmd  = f->cmd;

	switch ((proto_type_t)f->type)
	{
	case PROTO_TYPE_R:
		handle_read(f);
		break;
	case PROTO_TYPE_W:
		handle_write(f);
		break;
	case PROTO_TYPE_M:
		/* M 은 장치->앱 전용이다. 앱이 보낸 M 은 규격 위반이므로 무시한다. */
		g_proto.rx_err++;
		break;
	default:
		g_proto.rx_err++;
		break;
	}
}

/* ==========================================================================
 * §D 서비스 틱
 * ========================================================================== */

void Proto_Init(void)
{
	memset((void *)&g_proto, 0, sizeof(g_proto));
	g_proto.mon_mask      = PROTO_MON_MASK_DEFAULT;
	g_proto.mon_period_ms = PROTO_MON_PERIOD_DEFAULT_MS;
	g_proto.mon_last      = HAL_GetTick();
	g_proto.prev_mode     = (uint8_t)g_app_mode;
	g_proto.prev_state    = scn_state_of_mode();
	g_proto.prev_err      = 0U;
	g_proto.prev_busy     = 0U;
	g_proto.run_start     = g_proto.mon_last;
	Proto_DecoderReset(&s_dec);
}

/* STATUS 를 M 으로 송신.
 *
 * ★mon_seq 는 "STATUS 를 보낸 횟수"다. 반드시 여기서만 증가시킨다.
 *   버스트 단위로 올리면, 모터/출력만 바뀐 즉시송신(STATUS 는 나가지 않는다)에서도
 *   번호가 건너뛰어 앱이 "STATUS 유실"로 오탐한다. seq 가 STATUS 페이로드 안에만
 *   실리므로, 세는 대상도 STATUS 하나여야 앞뒤가 맞다.
 *   (mask 에서 STATUS 를 끄면 seq 는 멈춘다 - 그때는 유실 감지 수단이 없다.) */
static uint8_t push_status(void)
{
	if (!(g_proto.mon_mask & PROTO_MON_STATUS))
		return 0;
	g_proto.mon_seq++;
	return Proto_SendPayload((uint8_t)PROTO_TYPE_M, (uint8_t)PROTO_CMD_STATUS);
}

/* 현재 모드에 해당하는 시나리오 M 패킷만 보낸다(모음 모드에서 동작 패킷은
 * 의미가 없고 9600 회선만 잡아먹는다). */
static void push_scenario(uint8_t type)
{
	if (g_app_mode == APP_MODE_MOEUM)
	{
		if (g_proto.mon_mask & PROTO_MON_MOEUM)
			(void)Proto_SendPayload(type, (uint8_t)PROTO_CMD_MOEUM);
	}
	else if (g_app_mode == APP_MODE_DONGJAK)
	{
		if (g_proto.mon_mask & PROTO_MON_DONGJAK)
			(void)Proto_SendPayload(type, (uint8_t)PROTO_CMD_DONGJAK);
	}
}

/* 0x25 MOTOR 에서 "변화 감지"에 쓸 바이트만 뽑는다.
 * ★측정값(측정RPM / duty / 구간경과ms)은 일부러 제외한다. 그것들은 매 100ms 틱마다
 *   조금씩 변하므로 포함하면 "변화 즉시 송신"이 사실상 10Hz 주기 송신이 되어 9600
 *   회선을 포화시킨다. 여기서 감시하는 것은 사람이 "설정/상태가 바뀌었다"고 볼 값
 *   (회전 여부, 제어상태, 방향, 폴트, 지령 RPM, 패턴 종류)뿐이다. */
static void motor_sig(const uint8_t *payload, uint8_t out[14])
{
	uint8_t i;
	for (i = 0; i < 6U; i++)                      /* M1: running..target(hi) */
		out[i] = payload[PROTO_MOTOR_OFF_M1 + i];
	for (i = 0; i < 6U; i++)                      /* M2: 같은 배치            */
		out[6U + i] = payload[PROTO_MOTOR_OFF_M2 + i];
	out[12] = payload[40];                        /* m1_pattern              */
	out[13] = payload[41];                        /* m2_pattern              */
}

/* 상태가 바뀐 순간을 잡아 즉시 M 을 밀어 올린다.
 * 주기 송신을 대체하지 않고 "추가로" 보낸다 - 모니터 화면에는 같은 스냅샷이 두 번
 * 와도 무해하지만, 모터가 섰는데 1초를 기다려야 보이는 것은 공장 모니터링에서
 * 곤란하기 때문이다. 변화 폭주는 line_backlogged() 가 막는다. */
static void push_on_change(uint32_t now_ms)
{
	uint8_t mode  = (uint8_t)g_app_mode;
	uint8_t state = scn_state_of_mode();
	uint8_t err   = (g_app_mode == APP_MODE_DONGJAK) ? g_dongjak.err_code : 0U;
	uint8_t busy  = scn_busy();
	uint8_t scn_changed;
	uint8_t mot[14];
	uint8_t mot_changed = 0;
	uint8_t out_changed = 0;

	/* 시나리오 시작(busy 0->1) 시각을 잡아 경과시간(run_s)의 기준으로 쓴다.
	 * 모음/동작 어느 쪽이든 동일하게 동작한다. */
	if (busy && !g_proto.prev_busy)
		g_proto.run_start = now_ms;

	scn_changed = (uint8_t)((mode  != g_proto.prev_mode)  ||
	                        (state != g_proto.prev_state) ||
	                        (err   != g_proto.prev_err)   ||
	                        (busy  != g_proto.prev_busy));

	g_proto.prev_mode  = mode;
	g_proto.prev_state = state;
	g_proto.prev_err   = err;
	g_proto.prev_busy  = busy;

	/* 모터/출력 변화 판정. 페이로드를 한 번 만들어 비교용 바이트만 떼어 본다. */
	(void)build_motor(s_chg, now_ms);
	motor_sig(s_chg, mot);
	if (memcmp(mot, g_proto.prev_mot, sizeof mot) != 0)
	{
		memcpy(g_proto.prev_mot, mot, sizeof mot);
		mot_changed = g_proto.prev_valid;      /* 첫 채움은 송신하지 않는다 */
	}
	(void)build_output(s_chg, now_ms);
	if (memcmp(s_chg, g_proto.prev_out, sizeof g_proto.prev_out) != 0)
	{
		memcpy(g_proto.prev_out, s_chg, sizeof g_proto.prev_out);
		out_changed = g_proto.prev_valid;
	}
	g_proto.prev_valid = 1U;

#if PROTO_MON_ON_CHANGE
	if (g_proto.mon_mask == 0U)
		return;
	if (!(scn_changed || mot_changed || out_changed))
		return;
	if (line_backlogged())
		return;

	if (scn_changed)
	{
		(void)push_status();
		push_scenario((uint8_t)PROTO_TYPE_M);
	}
	if (mot_changed && (g_proto.mon_mask & PROTO_MON_MOTOR))
		(void)Proto_SendPayload((uint8_t)PROTO_TYPE_M, (uint8_t)PROTO_CMD_MOTOR);
	if (out_changed && (g_proto.mon_mask & PROTO_MON_OUTPUT))
		(void)Proto_SendPayload((uint8_t)PROTO_TYPE_M, (uint8_t)PROTO_CMD_OUTPUT);
#else
	(void)scn_changed; (void)mot_changed; (void)out_changed;
#endif
}

/* 주기 송신을 시작하기 전에 회선이 따라오는지 확인한다.
 * 9600 baud 에서 한 주기 버스트는 최악 219B(약 228ms) 이므로, 마스크를 넓게 켜고
 * 주기를 짧게 두면 링이 계속 밀린다. 이전 주기분이 아직 절반 이상 남아 있으면
 * 이번 주기를 통째로 건너뛴다 - 일부 패킷만 나가 앱이 오래된 값과 새 값을
 * 섞어 보는 상황을 막고, tx_drop 으로 조용히 사라지는 것보다 진단이 쉽다. */
static uint8_t line_backlogged(void)
{
	if (UartCtrl_TxPending() > (UART_CTRL_TX_BUFSZ / 2U))
	{
		g_proto.mon_skipped++;
		return 1U;
	}
	return 0U;
}

static void push_periodic(uint32_t now_ms)
{
	if (g_proto.mon_period_ms == 0U || g_proto.mon_mask == 0U)
	{
		g_proto.mon_last = now_ms;      /* 재개 시 즉시 폭주하지 않도록 갱신 */
		return;
	}
	if ((now_ms - g_proto.mon_last) < g_proto.mon_period_ms)
		return;
	g_proto.mon_last = now_ms;

	if (line_backlogged())
		return;

	/* 순서: 요약 -> 시나리오 -> 모터 -> 출력 -> 센서 -> 정지.
	 * 앱 화면의 위에서 아래 순서와 같게 두어, 회선이 느릴 때도 중요한 것이 먼저
	 * 도착하도록 한다. mon_seq 는 push_status() 안에서만 증가한다. */
	(void)push_status();
	push_scenario((uint8_t)PROTO_TYPE_M);
	if (g_proto.mon_mask & PROTO_MON_MOTOR)
		(void)Proto_SendPayload((uint8_t)PROTO_TYPE_M, (uint8_t)PROTO_CMD_MOTOR);
	if (g_proto.mon_mask & PROTO_MON_OUTPUT)
		(void)Proto_SendPayload((uint8_t)PROTO_TYPE_M, (uint8_t)PROTO_CMD_OUTPUT);
	if (g_proto.mon_mask & PROTO_MON_SENSOR)
		(void)Proto_SendPayload((uint8_t)PROTO_TYPE_M, (uint8_t)PROTO_CMD_SENSOR);
	if (g_proto.mon_mask & PROTO_MON_JUNGJI)
		(void)Proto_SendPayload((uint8_t)PROTO_TYPE_M, (uint8_t)PROTO_CMD_JUNGJI);
}

/* 디버거에서 dbg_push_* 에 1 을 쓰면 그 패킷을 1회 즉시 송신(앱 없이 회선 확인) */
static void push_debug(void)
{
	if (g_proto.dbg_push_status)
	{
		g_proto.dbg_push_status = 0U;
		(void)Proto_SendPayload((uint8_t)PROTO_TYPE_M, (uint8_t)PROTO_CMD_STATUS);
	}
	if (g_proto.dbg_push_moeum)
	{
		g_proto.dbg_push_moeum = 0U;
		(void)Proto_SendPayload((uint8_t)PROTO_TYPE_M, (uint8_t)PROTO_CMD_MOEUM);
	}
	if (g_proto.dbg_push_dongjak)
	{
		g_proto.dbg_push_dongjak = 0U;
		(void)Proto_SendPayload((uint8_t)PROTO_TYPE_M, (uint8_t)PROTO_CMD_DONGJAK);
	}
	if (g_proto.dbg_push_sensor)
	{
		g_proto.dbg_push_sensor = 0U;
		(void)Proto_SendPayload((uint8_t)PROTO_TYPE_M, (uint8_t)PROTO_CMD_SENSOR);
	}
}

void Proto_Tick(uint32_t now_ms)
{
	uint8_t c;
	uint16_t guard = UART_CTRL_RX_BUFSZ;   /* 한 틱에 링 1회분까지만 처리 */

	/* 1) 수신 소진 -> 프레임 완성 시 즉시 응답 */
	while (guard-- && UartCtrl_ReadByte(&c))
	{
		int8_t r = Proto_DecodeByte(&s_dec, c, &s_frame);
		if (r == 1)
			dispatch(&s_frame);
		else if (r < 0)
			g_proto.rx_err++;
	}

	/* 2) 모니터링(M) 송신. 변화 즉시 송신 -> 주기 송신 순서.
	 *    둘은 배타가 아니다(push_on_change 주석 참고). */
	push_on_change(now_ms);
	push_periodic(now_ms);

	push_debug();
}
