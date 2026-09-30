/* ==========================================================================
 * voice_table.c - **생성 파일. 손으로 고치지 말 것.**
 *
 *   생성: python src/voice_updater/voice_updater.py --gen-table
 *   출처: 설계서 §4 ID 표(어휘 = vu_names.py) + 실제 WAV 변환 결과(길이)
 *
 * 어휘(enum)는 `voice_table.h` 가 손으로 들고 있고, 이 파일에는 **녹음을 다시
 * 변환하면 바뀌는 값**만 있다 - 그래서 재생성해도 시나리오 코드가 참조하는
 * 이름은 흔들리지 않는다. 자세한 이유는 voice_table.h 상단 참조.
 *
 * dur_10ms 는 **생성 시점의 값**이다. 정확한 길이는 슬롯 헤더가 정본이므로
 * 런타임에는 Voice_Probe() 를 쓸 것.
 * ========================================================================== */
#include "voice_table.h"

/* enum 값 = 슬롯 번호 = 배열 인덱스. 어긋나면 여기서 컴파일이 깨진다. */
_Static_assert((int)VOICE_COUNT == 32, "VOICE_COUNT != 32 (설계서 §4 슬롯 수)");

#if VOICE_TABLE_TEXT
#define T(s) (s)
#else
#define T(s) NULL
#endif

const voice_desc_t g_voice_table[VOICE_COUNT] =
{
	[VOICE_DRAIN_START] = {  0u, VOICE_F_VALID,   260u, "01-01"  , T("30초간 배수 타임이 시작됩니다") },
	[VOICE_DRAIN_END] = {  1u, VOICE_F_VALID,   171u, "01-02"  , T("배수 타임이 끝났습니다") },
	[VOICE_MOEUM_START] = {  2u, VOICE_F_VALID,   347u, "02-01"  , T("음식물을 염분세척하여 보관하겠습니다") },
	[VOICE_MOEUM_LID_OPEN] = {  3u, VOICE_F_VALID | VOICE_F_ERROR,   500u, "02-02"  , T("음식물 세척 중입니다. 마개를 닫고 모음 위치에 놓아 주세요") },
	[VOICE_MOEUM_DONE] = {  4u, VOICE_F_VALID,   783u, "02-03"  , T("세척 완료 및 배수가 끝났습니다. 처리통에 음식물이 다 차기 전에 동작 위치에 놓고 처리해 주세요") },
	[VOICE_DONGJAK_START] = {  5u, VOICE_F_VALID,   371u, "03-01"  , T("염분 세척 후 전 과정을 자동으로 처리하겠습니다") },
	[VOICE_RSV_HEAT_START] = {  6u, VOICE_F_GAP,     0u, "-"      , T("결번 — 건조 시작 예약") },
	[VOICE_RSV_GRIND_START] = {  7u, VOICE_F_GAP,     0u, "-"      , T("결번 — 분쇄 시작 예약") },
	[VOICE_BIN_MISSING] = {  8u, VOICE_F_VALID | VOICE_F_ERROR,   213u, "03-04"  , T("수거통을 위치에 넣어 주세요") },
	[VOICE_DISCHARGE_DONE] = {  9u, VOICE_F_VALID,   641u, "03-05"  , T("처리가 끝나고, 배출이 완료되었습니다. 수거통을 확인 후 넘치기 전에 비워 주세요") },
	[VOICE_SELFCLEAN_START] = { 10u, VOICE_F_VALID,   208u, "04-01"  , T("자가세척 기능이 시작됩니다") },
	[VOICE_SELFCLEAN_END] = { 11u, VOICE_F_VALID,   195u, "04-02"  , T("자가세척 기능이 끝났습니다") },
	[VOICE_E_GUIDE] = { 12u, VOICE_F_VALID | VOICE_F_ERROR,   538u, "05-01"  , T("교반가이드가 확인되지 않습니다. 작동을 멈춥니다. 점검해 주세요") },
	[VOICE_E_STIR_SPEED] = { 13u, VOICE_F_VALID | VOICE_F_ERROR,   501u, "05-02"  , T("교반 속도가 설정 기준을 벗어나 회전을 멈춥니다. 점검해 주세요") },
	[VOICE_E_PIN_CAM] = { 14u, VOICE_F_VALID | VOICE_F_ERROR,   362u, "05-03"  , T("마개 스위치 잠금 핀 모터 상태를 확인해 주세요") },
	[VOICE_E_WDOOR_CLOSE] = { 15u, VOICE_F_VALID | VOICE_F_ERROR,   413u, "05-04"  , T("배수부 문 닫힘 위치가 확인되지 않습니다. 점검해 주세요") },
	[VOICE_E_WDOOR_OPEN] = { 16u, VOICE_F_VALID | VOICE_F_ERROR,   422u, "05-05"  , T("배수부 문 열림 위치가 확인되지 않습니다. 점검해 주세요") },
	[VOICE_E_DISCHARGE_LOCK] = { 17u, VOICE_F_VALID | VOICE_F_ERROR,   496u, "05-06"  , T("배출이 확인되지 않아 잠금을 유지합니다. 점검을 요청해 주세요") },
	[VOICE_E_M_STIR] = { 18u, VOICE_F_VALID | VOICE_F_ERROR,   218u, "06-01"  , T("교반 모터 상태를 확인해 주세요") },
	[VOICE_E_M_WDOOR] = { 19u, VOICE_F_VALID | VOICE_F_ERROR,   236u, "06-02"  , T("배수부 모터 상태를 확인해 주세요") },
	[VOICE_E_M_TDOOR] = { 20u, VOICE_F_VALID | VOICE_F_ERROR,   246u, "06-03"  , T("배출부 모터 상태를 확인해 주세요") },
	[VOICE_E_M_GRIND] = { 21u, VOICE_F_VALID | VOICE_F_ERROR,   222u, "06-04"  , T("분쇄 모터 상태를 확인해 주세요") },
	[VOICE_E_M_LIFT] = { 22u, VOICE_F_VALID | VOICE_F_ERROR,   292u, "06-05"  , T("상하 높낮이 모터 상태를 확인해 주세요") },
	[VOICE_E_M_STEP1] = { 23u, VOICE_F_VALID | VOICE_F_ERROR,   260u, "06-06"  , T("수증기 개폐부 모터를 확인해 주세요") },
	[VOICE_E_M_STEP2] = { 24u, VOICE_F_VALID | VOICE_F_ERROR,   255u, "06-07"  , T("공기 흡입 모터 상태를 확인해 주세요") },
	[VOICE_E_M_FAN] = { 25u, VOICE_F_VALID | VOICE_F_ERROR,   213u, "06-08"  , T("수증기 팬 상태를 확인해 주세요") },
	[VOICE_RSV_26] = { 26u, VOICE_F_GAP,     0u, "-"      , T("결번 — 예약") },
	[VOICE_E_FILL] = { 27u, VOICE_F_VALID | VOICE_F_ERROR,   185u, "07-02"  , T("급수가 확인되지 않습니다") },
	[VOICE_E_RINSE_COUNT] = { 28u, VOICE_F_VALID | VOICE_F_ERROR,   232u, "07-03"  , T("헹굼 횟수 설정을 확인해 주세요") },
	[VOICE_RSV_29] = { 29u, VOICE_F_GAP,     0u, "-"      , T("예비 (E11 복구 등)") },
	[VOICE_RSV_30] = { 30u, VOICE_F_GAP,     0u, "-"      , T("예비 (E12 통신 등)") },
	[VOICE_RSV_31] = { 31u, VOICE_F_GAP,     0u, "-"      , T("예비") },
};

#undef T

const voice_desc_t *Voice_Desc(voice_id_t id)
{
	return ((uint32_t)id < (uint32_t)VOICE_COUNT) ? &g_voice_table[id] : NULL;
}

uint8_t Voice_Exists(voice_id_t id)
{
	const voice_desc_t *d = Voice_Desc(id);
	return (uint8_t)((d != NULL) && ((d->flags & VOICE_F_VALID) != 0u));
}

uint8_t Voice_IsErrorId(voice_id_t id)
{
	const voice_desc_t *d = Voice_Desc(id);
	return (uint8_t)((d != NULL) && ((d->flags & VOICE_F_ERROR) != 0u));
}

uint16_t Voice_DurMsOf(voice_id_t id)
{
	const voice_desc_t *d = Voice_Desc(id);
	return (d != NULL) ? (uint16_t)(d->dur_10ms * 10u) : 0u;
}

const char *Voice_TextOf(voice_id_t id)
{
	const voice_desc_t *d = Voice_Desc(id);
	return (d != NULL) ? d->text : NULL;
}
