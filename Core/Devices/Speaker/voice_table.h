#ifndef DEVICES_SPEAKER_VOICE_TABLE_H_
#define DEVICES_SPEAKER_VOICE_TABLE_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * voice_table - 음성 ID 목록. 시나리오가 "무슨 멘트를 틀지"를 말하는 어휘다.
 *
 * ★**R3 (Revision 3) 신규 파일** — 착수 4단계. 구현현황 §0.14.10.
 * 정본: [doc/U21_음성플래시_구조R3.md] §4(ID 표) · §7.2(enum·구조체)
 *
 * ---- 핵심: enum 값 = 슬롯 번호 = 배열 인덱스 --------------------------
 * 설계서 §4 가 "음성 ID = 슬롯 번호"로 못박았으므로 셋이 같은 수다.
 *     Voice_Play(VOICE_MOEUM_DONE)      // 슬롯 4 를 재생
 *     g_voice_table[VOICE_MOEUM_DONE]   // 그 슬롯의 메타데이터
 * 따로 매핑 함수가 필요 없다. 아래 static assert 로 어긋남을 컴파일 때 잡는다.
 *
 * ---- 이 파일(.h)은 손으로, `voice_table.c` 는 생성 --------------------
 * 설계서 §7.1 은 "voice_table.h = 생성 파일"이라 했지만 **.h/.c 를 갈랐다**:
 *   - enum·구조체는 **녹음이 바뀌어도 그대로**다(이름은 기능에서 온다).
 *   - 반면 재생 길이(dur_10ms)는 **WAV 를 다시 변환할 때마다 바뀐다**.
 * 둘을 한 파일에 두면 길이 하나 바뀔 때마다 enum 까지 재생성되어, 시나리오가
 * 참조하는 어휘가 도구 실행 결과에 흔들린다. 그래서 어휘는 여기 고정하고,
 * 흔들리는 값만 `voice_table.c` 로 뽑았다.
 *     생성: python src/voice_updater/voice_updater.py --gen-table
 *
 * ---- VOICE_E_* 프리픽스 주의 -------------------------------------------
 * 여기의 `VOICE_E_*` 는 **"에러 안내 멘트"의 ID** 다(설계서 §4 종류 = E).
 * `voice.h` 의 재생 **상태** 코드는 2026-09-20 에 `VOICE_ST_*` 로 바꿔 비켰다 —
 * 그 전에는 `VOICE_E_FLASH`(상태)와 `VOICE_E_FILL`(멘트 ID)이 한 프리픽스를
 * 나눠 써서 시나리오 코드에서 섞이기 쉬웠다.
 *
 * ---- 녹음이 없는 ID ------------------------------------------------------
 * 결번 3개(6·7·26)와 예비 3개(29~31)는 **파일이 없다.** 자리를 비워 두면
 * 나중에 녹음이 와도 슬롯 번호가 밀리지 않는다(설계서 §4).
 * 시나리오는 `Voice_Exists(id)` 로 거르거나, 그냥 `Voice_Play()` 를 불러도 된다 —
 * 빈 슬롯이면 `VOICE_ST_EMPTY` 를 돌려주고 **아무 소리도 내지 않는다**.
 * ========================================================================== */

/* ---- FLAGS (설계서 §2 디렉터리 엔트리와 같은 비트) --------------------- */
#define VOICE_F_VALID   0x01u   /* 녹음이 있다(= 구울 파일이 있다)          */
#define VOICE_F_ERROR   0x02u   /* 에러 안내(§4 종류 = E). 우선순위 판단용  */
#define VOICE_F_GAP     0x04u   /* 결번/예비 — 파일 없음                    */

/* ---- 음성 ID (설계서 §7.2 그대로) --------------------------------------
 * 값 = 슬롯 번호. 순서를 바꾸거나 중간에 끼워 넣지 말 것 — 이미 구워진
 * 플래시의 슬롯 배치가 이 순서다. 새 멘트는 예비(29~31)부터 쓴다. */
typedef enum
{
	/* 01. 배수 타임 */
	VOICE_DRAIN_START      = 0,   /* 01-01 30초간 배수 타임이 시작됩니다     */
	VOICE_DRAIN_END        = 1,   /* 01-02 배수 타임이 끝났습니다            */

	/* 02. 모음 기능 */
	VOICE_MOEUM_START      = 2,   /* 02-01 염분세척하여 보관하겠습니다       */
	VOICE_MOEUM_LID_OPEN   = 3,   /* 02-02 E 마개를 닫고 모음 위치에         */
	VOICE_MOEUM_DONE       = 4,   /* 02-03 세척·배수 완료 (최장 7.83s)       */

	/* 03. 동작 기능 */
	VOICE_DONGJAK_START    = 5,   /* 03-01 전 과정을 자동으로 처리하겠습니다 */
	VOICE_RSV_HEAT_START   = 6,   /* 03-02 결번 — 건조 시작 예약             */
	VOICE_RSV_GRIND_START  = 7,   /* 03-03 결번 — 분쇄 시작 예약             */
	VOICE_BIN_MISSING      = 8,   /* 03-04 E 수거통을 위치에 넣어 주세요     */
	VOICE_DISCHARGE_DONE   = 9,   /* 03-05 배출 완료, 수거통 확인            */

	/* 04. 자가세척 */
	VOICE_SELFCLEAN_START  = 10,  /* 04-01 자가세척이 시작됩니다             */
	VOICE_SELFCLEAN_END    = 11,  /* 04-02 자가세척이 끝났습니다             */

	/* 05. 락 관련 오류 (전부 E) */
	VOICE_E_GUIDE          = 12,  /* 05-01 교반가이드 미확인                 */
	VOICE_E_STIR_SPEED     = 13,  /* 05-02 교반 속도 기준 이탈               */
	VOICE_E_PIN_CAM        = 14,  /* 05-03 마개 스위치 잠금 핀 모터          */
	VOICE_E_WDOOR_CLOSE    = 15,  /* 05-04 배수부 문 닫힘 미확인             */
	VOICE_E_WDOOR_OPEN     = 16,  /* 05-05 배수부 문 열림 미확인             */
	VOICE_E_DISCHARGE_LOCK = 17,  /* 05-06 배출 미확인 → 잠금 유지           */

	/* 06. 부품 오류 (전부 E) */
	VOICE_E_M_STIR         = 18,  /* 06-01 교반 모터                         */
	VOICE_E_M_WDOOR        = 19,  /* 06-02 배수부 모터                       */
	VOICE_E_M_TDOOR        = 20,  /* 06-03 배출부 모터                       */
	VOICE_E_M_GRIND        = 21,  /* 06-04 분쇄 모터                         */
	VOICE_E_M_LIFT         = 22,  /* 06-05 상하 높낮이 모터                  */
	VOICE_E_M_STEP1        = 23,  /* 06-06 수증기 개폐부 모터                */
	VOICE_E_M_STEP2        = 24,  /* 06-07 공기 흡입 모터                    */
	VOICE_E_M_FAN          = 25,  /* 06-08 수증기 팬                         */

	/* 07. 급수·설정 오류 */
	VOICE_RSV_26           = 26,  /* 07-01 결번 — 예약                       */
	VOICE_E_FILL           = 27,  /* 07-02 E 급수가 확인되지 않습니다        */
	VOICE_E_RINSE_COUNT    = 28,  /* 07-03 E 헹굼 횟수 설정 확인             */

	/* 예비 — E11 복구 · E12 통신 · 기타 (설계서 §4) */
	VOICE_RSV_29           = 29,
	VOICE_RSV_30           = 30,
	VOICE_RSV_31           = 31,

	VOICE_COUNT            = 32
} voice_id_t;

/* ---- 슬롯 메타데이터 ----------------------------------------------------
 * ★설계서 §7.2 의 voice_desc_t 에서 `addr` 과 `len` 을 뺐다:
 *   - addr 은 `VOICE_SLOT_BASE(id)` 로 계산된다(중복 저장 = 어긋날 거리).
 *   - len 은 **슬롯 헤더가 정본**이다. 재생기가 매번 헤더에서 읽으므로
 *     (voice.c `Voice_Probe`), 표에 박아 두면 다시 구울 때마다 거짓말이 된다.
 * dur_10ms 만 남긴 이유: 시나리오가 "이 멘트 끝날 때까지 기다린다" 같은 판단을
 * **플래시를 읽지 않고** 하고 싶을 때가 있다. 이건 생성 시점 값이라 재생성으로
 * 갱신된다. 정확한 길이가 필요하면 `Voice_Probe()` 로 헤더를 읽을 것. */
typedef struct
{
	uint8_t     id;         /* = 배열 인덱스 = 슬롯 번호 (자기검사용)        */
	uint8_t     flags;      /* VOICE_F_*                                     */
	uint16_t    dur_10ms;   /* 재생 길이 /10ms. 0 = 녹음 없음                */
	const char *src;        /* 원본 식별 "02-03" (없으면 "-")                */
	const char *text;       /* 멘트. VOICE_TABLE_TEXT=0 이면 NULL            */
} voice_desc_t;

/* 멘트 문자열을 펌웨어에 넣을지. 26개 UTF-8 합계 약 2.4KB(플래시).
 * 디버거에서 "지금 무슨 멘트냐"를 바로 보려면 1, 플래시가 아쉬우면 0. */
#ifndef VOICE_TABLE_TEXT
#define VOICE_TABLE_TEXT    1
#endif

extern const voice_desc_t g_voice_table[VOICE_COUNT];

/* ---- 조회 (전부 플래시를 읽지 않는다 - 표만 본다) --------------------- */

/* 범위 밖이면 NULL. */
const voice_desc_t *Voice_Desc(voice_id_t id);

/* 1 = 녹음이 있는 ID(결번·예비가 아니다). 구웠는지 여부는 알 수 없다 —
 * 그건 슬롯 헤더를 봐야 한다(`Voice_Probe`). */
uint8_t Voice_Exists(voice_id_t id);

/* 1 = 에러 안내 멘트(§4 종류 E). 설계서 §7.4 의 "에러 > 정상" 판단에 쓴다. */
uint8_t Voice_IsErrorId(voice_id_t id);

/* 표에 적힌 재생 길이[ms]. 녹음이 없으면 0. */
uint16_t Voice_DurMsOf(voice_id_t id);

/* 멘트 문자열. VOICE_TABLE_TEXT=0 이거나 범위 밖이면 NULL. */
const char *Voice_TextOf(voice_id_t id);

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_SPEAKER_VOICE_TABLE_H_ */
