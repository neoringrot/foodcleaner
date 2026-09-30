#ifndef DEVICES_EEPROM_VOICEUPDATER_H_
#define DEVICES_EEPROM_VOICEUPDATER_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * voiceupdater - UART5 로 음성 이미지를 받아 U21(W25Q128)에 기록하는 로더.
 *
 * ★**R3 (Revision 3) 신규 파일** — 착수 4단계 부수 작업. 구현현황 §0.14.
 *   정식 기능이 아닌 1회성 정비 경로다(아래 참조).
 *
 * 설계서: [doc/U21_음성플래시_구조R3.md] §1(맵)·§2(디렉터리)·§3(슬롯 헤더)·§8(기록 절차)
 * 드라이버: `w25q128.*` (§0.11) · 전송로: `Interface/uart_ctrl.*` (UART5)
 * 호스트 도구: `src/voice_updater/voice_updater.py`
 *
 * ---- 물리 경로 ----------------------------------------------------------
 * REV02 에서 UART5(PC12/PD2)는 **테스트포인트 TP6(TX)/TP1(RX)에만** 연결돼 있다.
 * 넷리스트에 BLE 모듈 부품이 없다 — `BLE-TXD` 넷 = TP6 + U22.113 이 전부다.
 * 즉 PC 의 USB-UART 어댑터를 TP1/TP6 에 직결하는 유선 경로다(BLE 경유 아님).
 * 보율 **115200 8N1** (2026-09-20 변경, 종전 9600).
 *
 * ---- UART5 소유권 전환 --------------------------------------------------
 * 평시 UART5 의 RX 소비자는 `protocol_r0`(앱 프로토콜)다. 로더는 R0 프레임
 * 하나로 진입한 뒤 **세션이 끝날 때까지 UART5 를 독점**한다:
 *
 *   1. 앱/PC → `W | PROTO_CMD_VOICE_UPD(0x32) | LEN=1 | 0x01`
 *   2. 펌웨어 → R0 ACK 를 TX 링에 넣고 `VoiceUpdater_Enter()`
 *   3. 이후 `Proto_Tick()` 은 RX 를 소비하지 않는다(즉시 return).
 *      `VoiceUpdater_Poll()` 이 아래 VU 프레이밍으로 직접 파싱한다.
 *   4. EXIT 프레임 / 타임아웃 / 에러 → 모드 해제, protocol_r0 복귀.
 *
 * 진입 조건은 **"시나리오가 실제로 돌고 있지 않을 것"** 하나다
 * (`Moeum_IsBusy()`/`Dongjak_IsBusy()`). 운전 중이면 NAK — 세션 동안
 * defaultTask 가 묶여 센서 폴링과 SenseTick 이 멈추기 때문이다.
 *
 * 마개가 시나리오 위치(HS1/2/4/5)에 얹혀 있어도 다운로드는 된다. 중재자가
 * 100ms 마다 홀을 디코딩해 모드를 되돌리므로, 진입할 때 `g_modearb.dbg_disable`
 * 을 1 로 두고 `g_app_mode` 를 대기(`APP_MODE_TESTBENCH`=0, 대기/벤치)로 **임시 고정**했다가 세션이 끝나면 둘 다
 * 원래 값으로 복원한다. 다운로드는 정식 기능이 아닌 1회성 정비 행위이므로
 * 평상시 동작에 흔적을 남기지 않는다.
 *
 * ---- 프로토콜 리비전 ----------------------------------------------------
 * 이 프로토콜은 **R3(Revision 3)에서 신설**됐다. R2 까지는 U21 에 무언가를
 * 굽는 경로 자체가 없었다. 와이어에도 실어 둔다 — PING 응답의 `seq` 필드가
 * 비어 있어(항상 0) 거기에 `VU_PROTO_REV` 를 담는다. 호스트는 첫 PING 에서
 * 이 값을 읽어 **펌웨어와 도구의 세대가 맞는지** 확인할 수 있다.
 * (`seq` 는 DATA 에서만 의미가 있으므로 기존 파서를 깨지 않는다.)
 *
 * ---- VU 프레이밍 (이 파일이 규정한다) -----------------------------------
 * R0 의 DLE 이스케이프를 쓰지 않는 **순수 바이너리**다. 페이로드가 PCM(랜덤에
 * 가까운 바이트열)이라 이스케이프를 쓰면 평균 +1.6%, 최악 2배가 되고
 * `PROTO_DATA_MAX`(64B)도 4KB 로 키워야 해서 분리했다.
 *
 *   SYNC0 SYNC1 | TYPE | LEN(2, LE) | PAYLOAD[LEN] | CRC32(4, LE)
 *     0x56 0x55    1        2            LEN             4
 *     ('V' 'U')
 *
 *   CRC32 = IEEE 802.3 (reflected, init/xorout 0xFFFFFFFF) — 호스트
 *           zlib.crc32 와 동일. **TYPE + LEN(2) + PAYLOAD** 에 대해 계산한다
 *           (SYNC 와 CRC 자신은 제외).
 *   LEN   = 0 .. VU_MAX_PAYLOAD. 이 범위를 벗어나면 프레임을 버리고 재동기한다.
 *   동기  = SYNC 2바이트. 깨지면 바이트를 하나씩 밀며 다시 찾는다.
 *
 * ---- 커맨드 (호스트 → 장치) ---------------------------------------------
 *
 *  TYPE 0x01 BEGIN  슬롯 1개 기록 시작. payload 268B:
 *      +0   u8   slot        0..31
 *      +1   u8   flags       설계서 §2 FLAGS (bit0 VALID/bit1 ERROR/bit2 GAP)
 *      +2   u16  reserved    0
 *      +4   u32  data_len    PCM 바이트 (≤ VOICE_DATA_MAX = 261,888)
 *      +8   u32  data_crc32  PCM 전체 CRC32 (END 에서 대조)
 *      +12  u8   hdr[256]    §3 슬롯 헤더 그대로. **END 에서** 기록한다
 *    장치: 슬롯이 쓰는 64KB 블록만 이레이즈(최대 4개) → 세션 상태 초기화 → ACK.
 *    ★헤더를 마지막에 쓰는 이유: 도중에 전원이 끊기면 헤더가 없어 그 슬롯은
 *      "빈 슬롯"으로 남는다. 반쯤 쓰인 데이터가 유효한 척하지 않는다(§8.4).
 *
 *  TYPE 0x02 DATA   PCM 청크. payload = u16 seq + bytes[N], N ≤ VU_CHUNK_MAX
 *    장치: 기록 주소 = SLOT_BASE(slot) + 0x100 + 지금까지 받은 길이.
 *          seq 는 순번 확인용(기대값과 다르면 NAK BAD_SEQ).
 *          페이지 프로그램 후 러닝 CRC32 갱신 → ACK(info = 다음 기대 seq).
 *    ★재전송 규약: 이미 프로그램한 청크는 되돌릴 수 없다(NOR). seq 가 어긋나면
 *      그 **슬롯을 BEGIN 부터 다시** 한다. 청크 단위 재전송은 지원하지 않는다.
 *
 *  TYPE 0x03 END    슬롯 마감. payload 0B
 *    장치: 받은 길이 == data_len 확인 → 러닝 CRC32 == data_crc32 확인 →
 *          헤더 페이지 256B 기록 + 되읽기 검증 → ACK(info = 계산된 CRC32).
 *          CRC 가 어긋나면 NAK(DATA_CRC) 하고 헤더를 쓰지 않는다(= 무효 슬롯).
 *
 *  TYPE 0x04 DIR    디렉터리 기록. payload 532B(§2)
 *    장치: 섹터 0 이레이즈 → 532B 기록 → 되읽기 검증 → ACK.
 *    ★맨 마지막에 보낸다(§8.4). 중간에 전원이 끊겨도 **이전 디렉터리**가 살아
 *      있고, 그마저 깨지면 슬롯 헤더 32개로 RAM 에서 복구한다(§2).
 *
 *  TYPE 0x05 EXIT   업데이트 모드 종료. payload 0B → ACK 후 protocol_r0 복귀.
 *  TYPE 0x06 PING   생존/칩 확인. payload 0B →
 *                   ACK(seq = VU_PROTO_REV(3), info = JEDEC ID 0xEF4018).
 *
 *  TYPE 0x07 PLAY   슬롯 재생(검증용). payload = slot(1)
 *    장치: `Voice_Play(slot)` → info = 그 슬롯의 DUR_MS. **비블로킹** —
 *          ACK 는 즉시 나가고 소리는 DAC DMA 가 흘린다(Speaker/voice.*).
 *          ★기록 중(BEGIN~END)에는 거부한다 — 재생기와 로더가 SPI1 을
 *            동시에 쓰면 안 되기 때문(voice.h "SPI1 공유").
 *  TYPE 0x08 STOP   재생 중지. payload 0B → info = 재생했던 바이트 수.
 *
 * ---- 응답 (장치 → 호스트) -----------------------------------------------
 *  TYPE 0x80 RESP   payload 8B:
 *      +0  u8  req_type   응답 대상 TYPE
 *      +1  u8  code       vu_code_t (0 = OK, 그 외 실패 사유)
 *      +2  u16 seq        DATA 면 해당 seq, PING 이면 VU_PROTO_REV, 아니면 0
 *      +4  u32 info       커맨드별 부가값 (PING=JEDEC, DATA=다음 seq, END=CRC32)
 *  응답은 커맨드 1개당 정확히 1개. 호스트는 **stop-and-wait** 로 다음을 보낸다.
 *
 * ---- 왜 stop-and-wait 인가 ----------------------------------------------
 * 청크마다 ACK 를 기다리므로, 장치가 플래시를 굽는 동안(페이지 프로그램,
 * 블록 이레이즈 최대 2초) 호스트는 아무것도 보내지 않는다. 덕분에 UART RX 링이
 * 작아도 절대 넘치지 않는다 — 링은 "한 청크를 받는 동안 쌓이는 양"만 감당하면
 * 되고, 로더가 1ms 마다 비워내므로 실제 점유는 수십 바이트다.
 *
 * ---- 왜 256KB 버퍼가 아닌가 ---------------------------------------------
 * STM32F103ZET6 의 SRAM 은 **64KB 전부**다(현재 약 18KB 사용). 256KB 버퍼는
 * 물리적으로 불가능하다. 대신 청크를 받는 즉시 플래시에 굽는 스트리밍이라
 * RAM 은 프레임 버퍼 1개(약 4.1KB) + 헤더 보관 256B 면 끝난다.
 *
 * ---- 태스크 영향 (중요) --------------------------------------------------
 * `VoiceUpdater_Poll()` 은 StartDefaultTask(100ms)에서 호출되고, **활성화되면
 * 세션이 끝날 때까지 그 안에서 돌아간다.** 즉 업데이트 중에는 defaultTask 의
 * 센서 폴링/Proto_Tick 이 멈춘다. 내부 대기는 `osDelay(1)` 양보라 MotorTask(1ms)
 * 는 정상 동작한다. 업데이트는 정비 행위이고 진입 조건이 "시나리오 미운전"이라
 * 이 트레이드오프를 택했다.
 * 무응답이 `VU_SESSION_TIMEOUT_MS` 를 넘으면 스스로 빠져나온다(영구 점유 방지).
 *
 * 전송 시간 참고: 115200 8N1 = 11.52 kB/s. 26개 전체 PCM 약 2.8MB →
 * 순수 전송 약 4분 + 이레이즈/프로그램 약 20초.
 * ========================================================================== */

/* ---- 프로토콜 리비전 ---------------------------------------------------
 * R3 에서 신설. 바꿀 때는 호스트 `vu_link.py` 의 VU_PROTO_REV 도 같이. */
#define VU_PROTO_REV        3u

/* ---- 프레이밍 상수 ------------------------------------------------------ */
#define VU_SYNC0            0x56u      /* 'V' */
#define VU_SYNC1            0x55u      /* 'U' */

#define VU_CHUNK_MAX        4096u      /* DATA 1회 최대 PCM 바이트          */
#define VU_MAX_PAYLOAD      (VU_CHUNK_MAX + 8u)   /* seq(2) + 여유           */
#define VU_RESP_LEN         8u

#define VU_HDR_BYTES        256u       /* §3 슬롯 헤더                      */
#define VU_DIR_BYTES        532u       /* §2 디렉터리 (0x214)               */
#define VU_BEGIN_LEN        (12u + VU_HDR_BYTES)  /* 268                     */

/* 슬롯 맵 (§1). 4단계에서 `Speaker/voice.h` 가 생기면 그쪽이 정본이 된다. */
#define VU_SLOT_COUNT       32u
#define VU_SLOT_BYTES       0x40000u
#define VU_SLOT_BASE(n)     (VU_SLOT_BYTES * ((n) + 1u))
#define VU_SLOT_DATA_OFF    0x100u
#define VU_DATA_MAX         (VU_SLOT_BYTES - VU_SLOT_DATA_OFF)  /* 261,888   */
#define VU_DIR_ADDR         0x000000u

/* 무응답 상한. 넘으면 세션을 버리고 protocol_r0 로 돌아간다. */
#ifndef VU_SESSION_TIMEOUT_MS
#define VU_SESSION_TIMEOUT_MS   15000u
#endif

/* ---- 프레임 TYPE -------------------------------------------------------- */
typedef enum
{
	VU_T_BEGIN = 0x01,
	VU_T_DATA  = 0x02,
	VU_T_END   = 0x03,
	VU_T_DIR   = 0x04,
	VU_T_EXIT  = 0x05,
	VU_T_PING  = 0x06,
	VU_T_PLAY  = 0x07,
	VU_T_STOP  = 0x08,
	VU_T_RESP  = 0x80
} vu_type_t;

/* ---- 응답 코드 ---------------------------------------------------------- */
typedef enum
{
	VU_OK = 0,
	VU_E_CRC,          /* 1 프레임 CRC32 불일치                              */
	VU_E_LEN,          /* 2 페이로드 길이가 커맨드 규격과 다름               */
	VU_E_TYPE,         /* 3 모르는 TYPE                                      */
	VU_E_SEQ,          /* 4 DATA seq 가 기대값과 다름 → 슬롯 재시작          */
	VU_E_SLOT,         /* 5 slot >= 32 또는 data_len 초과                    */
	VU_E_FLASH,        /* 6 w25q128 오류(이레이즈/프로그램/검증)             */
	VU_E_STATE,        /* 7 BEGIN 없이 DATA/END                              */
	VU_E_DATA_CRC,     /* 8 END 에서 PCM 전체 CRC32 불일치 → 헤더 미기록     */
	VU_E_OVERRUN,      /* 9 받은 길이가 data_len 초과                        */
	VU_E_PLAY          /* 10 재생 실패(빈 슬롯·헤더 불일치·이미 재생 중)     */
} vu_code_t;

/* ---- 관찰 변수 (디버거 watch) ------------------------------------------ */
typedef struct
{
	volatile uint8_t  active;        /* 1 = 업데이트 모드(UART5 독점)        */
	volatile uint8_t  in_slot;       /* 1 = BEGIN~END 진행 중                */
	volatile uint8_t  slot;          /* 현재 슬롯 번호                       */
	volatile uint8_t  last_code;     /* 마지막 응답 코드(vu_code_t)          */
	volatile uint8_t  last_type;     /* 마지막으로 처리한 TYPE               */
	volatile uint16_t seq;           /* 다음 기대 seq                        */
	volatile uint32_t recv_len;      /* 현재 슬롯에서 받은 PCM 바이트        */
	volatile uint32_t data_len;      /* BEGIN 이 선언한 길이                 */
	volatile uint32_t frames;        /* 처리한 프레임 수                     */
	volatile uint32_t bytes;         /* 플래시에 기록한 누적 바이트          */
	volatile uint16_t crc_err;       /* 프레임 CRC 실패 수                   */
	volatile uint16_t resync;        /* SYNC 재동기 횟수                     */
	volatile uint16_t slots_done;    /* END 까지 성공한 슬롯 수              */
	volatile uint8_t  dir_done;      /* 1 = 디렉터리 기록 성공               */
	volatile uint32_t session_ms;    /* 세션 소요 시간                       */
} VoiceUpdaterStat;

extern VoiceUpdaterStat g_vu;

/* ---- API ---------------------------------------------------------------- */

/* 상태 초기화. MX_UART5_Init()/W25Q 프로브 이후 1회. */
void VoiceUpdater_Init(void);

/* 업데이트 모드 진입. protocol_r0 의 W|0x32 핸들러가 호출한다.
 * 1 = 진입함, 0 = 거부(이미 활성이거나 시나리오 운전 중). */
uint8_t VoiceUpdater_Enter(void);

/* 1 = 업데이트 모드. Proto_Tick() 이 이 값을 보고 RX 소비를 멈춘다. */
uint8_t VoiceUpdater_IsActive(void);

/* 활성일 때 세션 전체를 처리한다(끝나면 반환). StartDefaultTask 에서 호출. */
void VoiceUpdater_Poll(void);

/* ---- CRC32 (IEEE, reflected) — 다른 모듈도 쓴다 ------------------------
 * 니블(16엔트리) 테이블 방식: 바이트당 2회 반복으로 비트단위보다 4배 빠르고
 * 테이블은 64B 다. 증분 계산을 지원해야 청크마다 이어서 돌릴 수 있다.
 *   uint32_t c = VU_Crc32Init();
 *   c = VU_Crc32Update(c, buf, n);   // 여러 번
 *   uint32_t crc = VU_Crc32Final(c);
 * 호스트의 zlib.crc32 와 같은 값이 나온다. */
uint32_t VU_Crc32Init(void);
uint32_t VU_Crc32Update(uint32_t crc, const uint8_t *p, uint32_t len);
uint32_t VU_Crc32Final(uint32_t crc);
uint32_t VU_Crc32(const uint8_t *p, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_EEPROM_VOICEUPDATER_H_ */
