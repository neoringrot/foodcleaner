#ifndef TESTBENCH_TB_VOICE_H_
#define TESTBENCH_TB_VOICE_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * tb_voice - U21 음성 플래시(W25Q128) 테스트벤치 (one-shot 커맨드 방식)
 *
 * ★**R3 (Revision 3) 신규 파일** — 착수 4단계. 구현현황 §0.11.5.
 *
 * 대상: `Devices/EEPROM/w25q128.*` 드라이버 + 그 위에 올라갈 음성 이미지.
 * 설계서: [doc/U21_음성플래시_구조R3.md] §1(맵)·§2(디렉터리)·§3(슬롯 헤더)·§9(검증).
 * 검증 항목: HW미검증 **1-17**, 설계서 **V1**(JEDEC ID)·**V2**(디렉터리 왕복)·**V4**(무결성).
 *
 * ---- tb_speaker 와의 관계 (헷갈리지 말 것) ------------------------------
 *   tb_speaker : 아날로그 출력단. LM4871 SPK-EN(PA3) + DAC(PA4) 톤 생성.
 *                → "스피커에서 소리가 나는가"
 *   tb_voice   : 플래시 안의 음성 데이터. SPI1 + /CS(PC4) 의 U21.
 *                → "음성 데이터가 거기 있고 멀쩡한가"
 * 핀도 주변장치도 상태도 공유하지 않는다. 동시에 켜도 서로 간섭하지 않는다.
 *
 * ---- 지금 되는 것 / 안 되는 것 ------------------------------------------
 * 되는 것(이 파일): 플래시 쪽 전부 — ID 프로브, 디렉터리/슬롯헤더 읽기·검증,
 *   임의 주소 덤프, **예약영역 한정** 소거/기록/검증 왕복(= 1-17 체크리스트).
 *
 * 재생도 된다(2026-09-20 추가): `tb_voice_slot` 을 정하고 `tb_voice_play_once=1`
 *   → `Speaker/voice.*` 가 그 슬롯을 DAC DMA 로 흘린다. 비블로킹이라 벤치가
 *   묶이지 않는다. 중지는 `tb_voice_stop_once=1`. 결과는 `tb_voice_play_err`
 *   (voice_status_t) 와 `g_voice`(VoiceStat) 로 본다.
 *
 * 안 되는 것: **UART 슬롯 로더**(`tb_voice_load_slot`, 설계서 §8) —
 *   그건 `EEPROM/voiceupdater.*` 가 별도로 구현했다(정비용, §0.14).
 *
 * ---- 사용법 (다른 tb_* 와 동일 관례) ------------------------------------
 * 디버거 Live Expressions 에서 `tb_voice_*_once` 에 1 을 쓰면 다음 폴(100ms)에
 * 1회 실행되고 플래그는 자동으로 0 이 된다(tb_protocol 과 같은 방식).
 *
 *   1) tb_voice_probe_once = 1
 *        → tb_voice_jedec_id == 0xEF4018, tb_voice_status == 0(W25Q_OK)
 *          0xFFFFFF 면 SB21/SB22(/WP,/HOLD 브리지)부터 본다. (§0.11)
 *   2) tb_voice_selftest_once = 1        ★ 소거/기록을 한다. 아래 안전장치 참조
 *        → tb_voice_step 이 TB_VOICE_ST_DONE(7) 이고 tb_voice_fails == 0 이면 통과
 *   3) tb_voice_dir_once = 1             (이미지 기록 후에 의미가 있다)
 *        → tb_voice_dir_valid / tb_voice_dir_slots / tb_voice_dir_crc_ok
 *   4) tb_voice_slot = N; tb_voice_hdr_once = 1
 *        → tb_voice_hdr_valid / tb_voice_hdr_len / tb_voice_hdr_dur_ms
 *   5) tb_voice_addr = 0x...; tb_voice_dump_once = 1
 *        → tb_voice_dump[64] 에 그 주소 64바이트 (읽기 전용, 어디든 안전)
 *
 * ---- ★ 쓰기 안전장치 (반드시 읽을 것) ----------------------------------
 * 자가검사는 **소거와 기록을 실제로 한다.** 잘못된 주소를 넣으면 음성 이미지가
 * 날아가므로 두 겹으로 막아 뒀다:
 *
 *   (1) 주소 화이트리스트 - tb_voice_test_addr 는 **0x001000 ~ 0x00FFFF** 의
 *       4KB 정렬 주소만 받는다(§1 맵의 "블록 0 섹터 1~15 예약" 영역).
 *       0x000000(디렉터리 섹터)과 0x040000 이상(슬롯 32개)은 **거부**하고
 *       tb_voice_step = TB_VOICE_ST_REFUSED 로 끝난다. 기본값 0x001000.
 *   (2) 모드 게이트 - `tb_active`(= AppMode_IsBenchIdle) 가 아니면 소거/기록
 *       커맨드는 실행하지 않는다. 시나리오가 도는 중에 defaultTask 를 1초쯤
 *       묶어 버리면 센서 폴링과 SenseTick 이 그만큼 밀리기 때문이다.
 *       읽기 전용 커맨드(probe/dir/hdr/dump)는 수 ms 라 모드와 무관하게 돈다.
 *
 * 자가검사가 끝나면 시험 섹터는 **소거된 상태(0xFF)로 남겨 둔다.**
 *
 * ---- 태스크 배치 / 비용 -------------------------------------------------
 * TB_Voice_Poll() 은 **StartDefaultTask(100ms)** 에서만 부른다.
 * w25q128 드라이버는 전 함수가 블로킹이고 섹터 이레이즈가 최대 400ms 다 —
 * 1ms 주기의 StartMotorTask 에 올리면 모터 틱을 놓친다. 절대 올리지 말 것.
 *   읽기 커맨드: ~1ms 이하
 *   자가검사 1회: 이레이즈 2회(각 최대 400ms) + 4KB 읽기 2회 + 512B 기록
 *                 ≈ 0.9초. 그 동안 defaultTask 는 다른 일을 못 한다(벤치 전용).
 *
 * 버퍼는 전부 **파일 스코프 static** 이다. defaultTask 스택이 1KB(freertos.c)
 * 뿐이라 532B 디렉터리 버퍼를 지역변수로 잡으면 넘친다.
 *
 * ---- jungji(정지)와의 관계 ----------------------------------------------
 * `Jungji_Testbench()` 의 enable 일괄 0 목록에 tb_voice 는 **넣지 않았다.**
 * 이 벤치는 액추에이터를 구동하지 않고, 진행 중인 플래시 사이클은 칩 내부에서
 * 끝나므로 중간에 플래그를 꺼도 얻는 게 없다. 커맨드는 어차피 1회성이다.
 * ========================================================================== */

/* ---- 슬롯 맵 상수 (설계서 §1·§7.2) -------------------------------------
 * 4단계에서 `Speaker/voice.h` 가 생기면 그쪽의 VOICE_* 가 정본이 된다. 이름을
 * TB_ 로 구분해 둔 것은 그때 충돌 없이 갈아끼우기 위해서다. */
#define TB_VOICE_SLOT_COUNT     32u
#define TB_VOICE_SLOT_BYTES     0x40000u                       /* 256KB        */
#define TB_VOICE_SLOT_BASE(n)   (TB_VOICE_SLOT_BYTES * ((n) + 1u))
#define TB_VOICE_DIR_ADDR       0x000000u                      /* §2 디렉터리  */
#define TB_VOICE_DIR_BYTES      0x214u                          /* 532B        */
#define TB_VOICE_HDR_BYTES      0x100u                          /* §3 256B     */

/* 자가검사가 허용하는 유일한 구간 (§1 "블록 0 섹터 1~15 예약"). */
#define TB_VOICE_TEST_LO        0x001000u
#define TB_VOICE_TEST_HI        0x010000u   /* exclusive */
#define TB_VOICE_TEST_DEFAULT   0x001000u

#define TB_VOICE_DUMP_BYTES     64u

/* 자가검사 진행 단계 = 실패 시 어디서 멈췄는지 그대로 읽힌다. */
typedef enum
{
	TB_VOICE_ST_IDLE = 0,
	TB_VOICE_ST_REFUSED,      /* 1 주소 화이트리스트 위반 / 벤치모드 아님    */
	TB_VOICE_ST_ERASE1,       /* 2 시험 섹터 소거                            */
	TB_VOICE_ST_BLANK1,       /* 3 소거 후 4KB 가 전부 0xFF 인지             */
	TB_VOICE_ST_PROGRAM,      /* 4 페이지 경계를 걸친 512B 기록              */
	TB_VOICE_ST_VERIFY,       /* 5 W25Q_Verify 비교                          */
	TB_VOICE_ST_REREAD,       /* 6 독립 재읽기 비교(Verify 와 교차확인)      */
	TB_VOICE_ST_DONE          /* 7 통과 — 섹터는 소거 상태로 남는다          */
} tb_voice_step_t;

/* ---- one-shot 커맨드 (1 을 쓰면 실행 후 자동 0) ------------------------ */
extern volatile uint8_t  tb_voice_probe_once;    /* JEDEC ID + W25Q_Init      */
extern volatile uint8_t  tb_voice_dir_once;      /* 디렉터리 읽기·검증 (§2)   */
extern volatile uint8_t  tb_voice_hdr_once;      /* 슬롯 헤더 읽기·검증 (§3)  */
extern volatile uint8_t  tb_voice_dump_once;     /* tb_voice_addr 64B 덤프    */
extern volatile uint8_t  tb_voice_selftest_once; /* ★소거/기록 왕복 (1-17)   */
extern volatile uint8_t  tb_voice_play_once;     /* tb_voice_slot 재생        */
extern volatile uint8_t  tb_voice_stop_once;     /* 재생 중지                 */

/* ---- 파라미터 ---------------------------------------------------------- */
extern volatile uint8_t  tb_voice_slot;          /* 0..31, hdr_once 대상      */
extern volatile uint32_t tb_voice_addr;          /* dump_once 대상 주소       */
extern volatile uint32_t tb_voice_test_addr;     /* 자가검사 섹터(화이트리스트)*/

/* ---- 공통 결과 --------------------------------------------------------- */
extern volatile uint32_t tb_voice_jedec_id;      /* 9Fh 원시 응답             */
extern volatile int8_t   tb_voice_status;        /* 마지막 w25q_status_t (0=OK)*/
extern volatile uint32_t tb_voice_runs;          /* 커맨드 실행 누계          */

/* ---- 디렉터리 (§2) ----------------------------------------------------- */
extern volatile uint8_t  tb_voice_dir_valid;     /* 1 = MAGIC 'ZGVC' 일치     */
extern volatile uint8_t  tb_voice_dir_crc_ok;    /* 1 = DIR_CRC32 일치        */
extern volatile uint8_t  tb_voice_dir_blank;     /* 1 = 전부 0xFF(미기록)     */
extern volatile uint8_t  tb_voice_dir_slots;     /* DATA_LEN != 0 인 엔트리 수*/
extern volatile uint16_t tb_voice_dir_version;
extern volatile uint16_t tb_voice_dir_rate;      /* SAMPLE_RATE (16000 기대)  */

/* ---- 슬롯 헤더 (§3), tb_voice_slot 대상 -------------------------------- */
extern volatile uint8_t  tb_voice_hdr_valid;     /* 1 = MAGIC 'ZGVF' + ID 일치*/
extern volatile uint8_t  tb_voice_hdr_crc_ok;    /* 1 = HDR_CRC32 일치        */
extern volatile uint8_t  tb_voice_hdr_blank;     /* 1 = 빈 슬롯(0xFF)         */
extern volatile uint32_t tb_voice_hdr_len;       /* DATA_LEN (PCM 바이트)     */
extern volatile uint32_t tb_voice_hdr_crc;       /* DATA_CRC32 (저장값)       */
extern volatile uint32_t tb_voice_hdr_dur_ms;    /* DUR_MS                    */
extern volatile uint32_t tb_voice_hdr_addr;      /* 이 슬롯의 SLOT_BASE       */

/* ---- 덤프 -------------------------------------------------------------- */
extern volatile uint8_t  tb_voice_dump[TB_VOICE_DUMP_BYTES];

/* ---- 자가검사 결과 ----------------------------------------------------- */
/* ---- 재생 결과 (Speaker/voice.* 경유) ---------------------------------- */
extern volatile uint8_t  tb_voice_play_err;      /* voice_status_t (0=OK)     */
extern volatile uint32_t tb_voice_play_dur_ms;   /* 그 슬롯의 DUR_MS          */

extern volatile uint8_t  tb_voice_step;          /* tb_voice_step_t           */
extern volatile uint16_t tb_voice_fails;         /* 0 이어야 통과             */
extern volatile uint32_t tb_voice_bad_off;       /* 첫 불일치 오프셋          */
extern volatile uint8_t  tb_voice_bad_exp;       /* 기대 바이트               */
extern volatile uint8_t  tb_voice_bad_got;       /* 실제 바이트               */
extern volatile uint32_t tb_voice_erase_ms;      /* 섹터 이레이즈 1회 (max400)*/
extern volatile uint32_t tb_voice_prog_ms;       /* 512B 기록                 */
extern volatile uint32_t tb_voice_total_ms;      /* 자가검사 1회 전체         */

/* 모든 커맨드 플래그/결과를 초기화한다. MX_SPI1_Init() 뒤에 1회. */
void TB_Voice_Init(void);

/* 대기 중인 one-shot 커맨드를 1개 처리한다. 매 폴(100ms) 호출.
 * tb_active: 1 = 벤치 소유 가능(AppMode_IsBenchIdle). 0 이면 **소거/기록만**
 * 거부하고 읽기 커맨드는 그대로 수행한다(SPI1 에는 U21 뿐이라 시나리오와
 * 다툴 상대가 없다 - tb_heat/tb_water 의 핀 소유권 문제와는 상황이 다르다). */
void TB_Voice_Poll(uint8_t tb_active);

#ifdef __cplusplus
}
#endif

#endif /* TESTBENCH_TB_VOICE_H_ */
