#ifndef DEVICES_SPEAKER_VOICE_H_
#define DEVICES_SPEAKER_VOICE_H_

#include "main.h"
#include "voice_table.h"   /* voice_id_t - 재생 단위는 슬롯이 아니라 음성 ID 다 */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * voice - U21 슬롯의 PCM 을 DAC+DMA 로 스피커에 내보내는 재생기.
 *
 * ★**R3 (Revision 3) 신규 파일** — 착수 4단계(음성 재생 경로).
 *   구현현황 §0.14.6 · 바지-인 수정 §0.14.7 · 검증 HW미검증 1-22.
 *
 * 설계서: [doc/U21_음성플래시_구조R3.md] §3(슬롯 헤더)·§7.3(재생 경로)
 * 데이터: `EEPROM/w25q128.*` 가 읽고, `EEPROM/voiceupdater.*` 가 구운 이미지.
 * 출력  : `Speaker/lm4871.*` (PA4 DAC_OUT1 -> Ci/Ri -> -IN, SD = PA3 active-low)
 *
 * ---- 신호 경로 (설계서 §7.3) --------------------------------------------
 *   TIM2 (PSC=3, ARR=999, TRGO=Update)  -> 16.000 kHz
 *     └ DAC1 CH1 (PA4), Trigger = T2_TRGO, 12B 우측정렬, 출력버퍼 ON
 *         └ DMA2_Channel3, Mem->Periph, half-word, **Circular**, 2048 elements
 *
 *   s_pcm[2048] 를 1024 샘플씩 핑퐁으로 쓴다. 한 쪽 = 1024/16k = **64 ms**.
 *   DMA 가 전반부를 다 내보내면 HT, 후반부를 다 내보내면 TC 인터럽트가 뜨고,
 *   그때 **방금 비워진 쪽**을 SPI 로 다음 2KB 를 읽어 채운다.
 *   SPI 2KB @16MHz ≈ 1.1 ms ≪ 64 ms 라 여유가 크다.
 *
 * ---- 클럭 주의 -----------------------------------------------------------
 * 이 보드의 SYSCLK 은 **HSI(내부 RC) / 2 × PLL16 = 64 MHz** 다(HSE 아님).
 * TIM2 는 APB1(32MHz)의 타이머 클럭 ×2 = 64MHz 를 받으므로
 *   64 MHz / (PSC+1=4) / (ARR+1=1000) = **16.000 kHz** 정확히 떨어진다.
 * 다만 HSI 는 ±1% RC 라 재생 피치에 그만큼 오차가 있다(청감상 무의미).
 * 정밀이 필요해지면 HSE 로 옮기고 VOICE_TIM_PSC/ARR 을 다시 계산할 것.
 *
 * ---- .ioc 와의 관계 (중요) ----------------------------------------------
 * TIM2 · DMA2_CH3 · DAC 트리거는 **`.ioc` 에 없다.** 이 파일이 코드로 직접
 * 설정한다(`Voice_Init`). 그렇게 한 이유:
 *   - `.ioc` 는 다른 세션이 3단계(REV02 반영)로 편집 중이라 지금 건드리면 충돌한다.
 *   - 셋 다 **핀 배정이 아니다**. PA4(DAC_OUT1)·PA3(SD)는 이미 `.ioc` 에 있다.
 * 따라서 CubeMX 재생성이 이 코드를 지우지 않는다. 다만 4단계에서 검토서
 * ioc-6~8 로 정식 반영할 때는 **이 파일의 수동 설정을 걷어내야** 중복되지 않는다.
 * `DMA2_Channel3_IRQHandler` 도 여기서 정의한다 — `.ioc` 에서 NVIC 를 켜면
 * `stm32f1xx_it.c` 에도 생겨 **중복 정의로 링크가 깨진다.** 그때 이쪽을 지울 것.
 *
 * ---- 태스크 배치 ---------------------------------------------------------
 * 전용 태스크 `StartVoiceTask`(freertos.c, prio Normal, stack 512 words).
 * DMA HT/TC ISR 이 세마포어를 주고 태스크가 SPI 로 다음 반쪽을 채운다.
 *   - **비블로킹**: 시나리오/모터 태스크는 전혀 기다리지 않는다.
 *   - 업데이트 모드(voiceupdater)에서 defaultTask 가 묶여 있어도 재생은 돈다.
 *
 * ---- SPI1 공유 ------------------------------------------------------------
 * SPI1 에는 U21 하나뿐이지만 **소비자는 둘**이다(재생기 / 로더·벤치).
 * w25q128 드라이버에는 락이 없으므로, 규약으로 겹치지 않게 한다:
 *   - 로더가 PLAY 를 받을 때 기록 중(`g_vu.in_slot`)이면 거부한다(voiceupdater.c).
 *   - 로더는 재생 중에 BEGIN 을 받으면 재생을 멈추고 거부한다.
 * 둘 다 정비/검증 행위라 동시에 할 이유가 없다.
 *
 * 재생기 **안에서도** 소비자가 둘이다 — `Voice_Task`(반쪽 채우기)와
 * `Voice_Play`(바지-인 정지 + 선채움). 둘 다 s_pcm 과 SPI 를 만지므로
 * `s_fill_mtx` 로 직렬화한다. 이게 없으면 바지-인 순간 태스크가 끝낸 옛 읽기가
 * 새 재생의 버퍼를 덮어쓴다(SPI 2KB ≈ 1.1ms 의 창).
 *
 * ---- 폴백 -----------------------------------------------------------------
 * 슬롯 헤더 매직/ID 불일치, 빈 슬롯, 길이 0 → 재생하지 않고 에러를 남긴다.
 * (설계서 §7.3 의 비프 폴백은 상위 시나리오가 붙을 때 결정한다 —
 *  검증용 재생에서 블로킹 비프를 울리면 MotorTick 이 멈춘다.)
 * ========================================================================== */

/* ---- 타이밍 (설계서 §7.3) ---------------------------------------------- */
#define VOICE_RATE_HZ       16000u
#define VOICE_TIM_PSC       3u        /* 64MHz / 4            */
#define VOICE_TIM_ARR       999u      /*        / 1000 = 16kHz */

/* 핑퐁 버퍼. 반쪽 = 1024 샘플 = 64ms. 전체 2048 샘플 = 4KB RAM. */
#define VOICE_HALF          1024u
#define VOICE_BUF           (VOICE_HALF * 2u)

#define VOICE_MID           2048u     /* 12bit 중간값 = 무음 */

/* 슬롯 맵 (§1). voiceupdater.h 의 VU_* 와 같은 값 — 둘 다 설계서가 원본이다. */
#define VOICE_SLOT_COUNT    32u
#define VOICE_SLOT_BYTES    0x40000u
#define VOICE_SLOT_BASE(n)  (VOICE_SLOT_BYTES * ((n) + 1u))
#define VOICE_SLOT_DATA_OFF 0x100u
#define VOICE_DATA_MAX      (VOICE_SLOT_BYTES - VOICE_SLOT_DATA_OFF)

/* 앰프를 켜고 DC 가 자리잡을 때까지(§7.3 "5ms 세틀"). */
#define VOICE_SETTLE_MS     5u
/* 끝나고 DAC 를 중간값에 둔 뒤 앰프를 끄기까지 — 팝 노이즈 방지(§7.3). */
#define VOICE_MUTE_DELAY_MS 20u
/* ---- 전환 팝 억제 (2026-09-20, 실기에서 팝 확인 후 추가) --------------
 * 바지-인은 파형 **한가운데**를 끊는다. 거기서 곧장 중간값으로 쓰면 최악
 * ±2048 LSB(≈1.65V)의 **계단**이 생기고, 그 계단의 고주파 성분이 그대로
 * '팝'으로 들린다. (정상 종료 때 팝이 없던 이유는 클립 끝이 이미 무음이라
 *  계단이 없기 때문이다 — 그래서 바지-인에서만 나타났다.)
 *
 * 같은 전압 이동을 수 ms 에 걸쳐 나누면 저주파가 되어 앰프의 출력 커패시터와
 * 스피커의 고역통과에 묻힌다. 8ms 램프면 등가 기본파가 약 60Hz 대라 소형
 * 스피커(대개 300Hz 이하 롤오프)에서 크게 감쇠한다.
 *   계단 한 칸 = 2048/256 = 8 LSB ≈ 6.6mV — 개별 계단은 들리지 않는다.
 *
 * ★**8ms 는 실기 청취로 확정한 값**(사용자 확정 2026-09-20, 구현현황 §0.14.9).
 *   5ms 에서는 팝이 뚜렷했고, 8ms 에서 "간혹 살짝 들리지만 거슬리지 않다".
 *   **원리상 0 이 되지는 않는다** — 끊는 지점의 진폭이 클수록 이동폭도 커진다.
 *   더 지우려면 10~12ms(전환이 굼떠진다), 그래도 남으면 원인이 DAC 계단이 아니라
 *   `HAL_DAC_Stop_DMA` 의 순간적 채널 차단일 수 있다 — DMA 만 Abort 하는 쪽을 볼 것. */
#define VOICE_RAMP_MS       8u        /* 실기 확정 - 위 주석 참조 */
/* 256 단계. (8*1000)/256 = 31us(정수절삭) x 256 = 실제 7.94ms — 무해한 오차다. */
#define VOICE_RAMP_STEPS    256u
/* 램프가 끝난 뒤 다음 재생을 걸기까지. 램프 자체가 세틀 역할을 하므로 짧다. */
#define VOICE_BARGE_MS      1u

/* ---- 우선순위·큐 (설계서 §7.4) ----------------------------------------
 * 정상 안내 중에 에러 안내가 들어오면 **에러가 이긴다** — 에러는 '지금 뭔가
 * 잘못됐다'를 말하므로 늦게 나가면 의미가 없다. 반대로 정상끼리는 뒤엣것이
 * 앞엣것을 끊을 이유가 없어 큐에 넣는다.
 *
 *   정상 재생 중 + 에러 요청  -> 정상을 즉시 끊고 에러 재생(바지-인)
 *   에러 재생 중 + 에러 요청  -> 에러 FIFO 에 쌓는다(최대 4개)
 *   재생 중     + 정상 요청   -> 정상 큐 1칸(**최신 것만** 남는다)
 *   재생 중인 ID 와 같은 요청 -> 무시(설계서 §7.4)
 *
 * ★끊긴 정상 안내는 **다시 큐에 넣지 않는다.** 그 멘트는 그 순간의 상태를
 *   말하는 것이라, 에러가 끝난 뒤에 나가면 이미 틀린 말이 된다.
 * 정상 큐가 1칸인 것도 같은 이유다 — 밀린 안내를 몰아서 읽어 주는 것은
 *   쓸모가 없고 가장 최근 상태 하나만 의미가 있다.
 *
 * 실제로 겹치는 자리: 모음 완료 틱에 ID 1(배수 끝, 1.7s)과 ID 4(세척 완료,
 * 7.8s)가 같이 나가야 한다(moeum.c).
 * ★Voice_Play 를 두 번 부르면 안 된다: 호출 태스크와 Voice_Task 가 같은
 *   우선순위(osPriorityNormal)라 세마포어를 줘도 선점이 없다 → 두 번째
 *   Voice_Play 가 태스크가 돌기 전에(playing==0) 정상 1칸을 덮어써서 ID 1 이
 *   빠지고 ID 4 만 나간다(실측·코드 확인).
 * ★2026-09-22 해소: **Voice_PlaySeq2(a, b)** — "a 가 끝나면 b" 를 한 번에
 *   요청한다. 정상 큐 뒤에 **연결 1칸(nq2)** 이 붙는다. 이 연결은 PlaySeq2 만
 *   쓰고, 이후 일반 Voice_Play(정상)가 오면 "최신 것만" 규칙대로 **연결째
 *   버려진다**(그 순간 상태가 바뀐 것이므로). 에러가 끼어들면 a 는 끊기고(재큐
 *   없음) 에러가 끝난 뒤 b 가 나간다. */
#define VOICE_ERRQ_DEPTH    4u

typedef enum
{
	VOICE_PRIO_NORMAL = 0,
	VOICE_PRIO_ERROR  = 1
} voice_prio_t;

/* 재생 API 의 **상태** 코드. 2026-09-20 에 VOICE_E_* -> VOICE_ST_* 로 바꿨다 —
 * 설계서 §7.2 의 **음성 ID** 에도 VOICE_E_GUIDE/VOICE_E_FILL 처럼 VOICE_E_ 가
 * 쓰여서(그쪽은 '에러 안내 멘트'라는 뜻), 같은 프리픽스가 두 가지를 가리키면
 * 시나리오 코드에서 Voice_Play(VOICE_E_FILL) 과 st == VOICE_E_FLASH 가 섞인다.
 * 공개 어휘인 음성 ID 쪽(voice_table.h)을 살리고 이쪽을 양보했다. */
typedef enum
{
	VOICE_ST_OK = 0,
	VOICE_ST_SLOT,      /* 1 슬롯 번호 범위 밖                              */
	VOICE_ST_FLASH,     /* 2 w25q128 읽기 실패                              */
	VOICE_ST_EMPTY,     /* 3 빈 슬롯(0xFF) — 아직 굽지 않았다               */
	VOICE_ST_HEADER,    /* 4 매직/ID 불일치 또는 길이 이상                   */
	VOICE_ST_BUSY       /* 5 이미 재생 중이거나 플래시 기록 중              */
} voice_status_t;

/* ---- 관찰 변수 (디버거 watch) ------------------------------------------ */
typedef struct
{
	volatile uint8_t  playing;     /* 1 = 재생 중                           */
	volatile uint8_t  slot;        /* 재생 중인(마지막) 슬롯                */
	volatile uint8_t  last_err;    /* voice_status_t                        */
	volatile uint8_t  stopping;    /* 1 = 꼬리 무음을 내보내는 중           */
	volatile uint32_t data_len;    /* 그 슬롯의 PCM 바이트                  */
	volatile uint32_t played;      /* 지금까지 DMA 로 내보낸 바이트         */
	volatile uint32_t dur_ms;      /* 헤더의 DUR_MS                         */
	volatile uint32_t refills;     /* 반쪽 채운 횟수                        */
	volatile uint16_t underrun;    /* 제때 못 채운 횟수 — 0 이어야 정상     */
	volatile uint16_t plays;       /* 재생 시작 누계                        */
	volatile uint8_t  prio;        /* 지금 재생 중인 것의 voice_prio_t      */
	volatile uint8_t  nq_id;       /* 정상 큐(1칸). VOICE_COUNT = 비었음    */
	volatile uint8_t  nq2_id;      /* 정상 연결 1칸(PlaySeq2 의 두 번째)    */
	volatile uint8_t  eq_cnt;      /* 에러 FIFO 에 쌓인 수                  */
	volatile uint16_t dropped;     /* 큐가 차서 버린 요청 수                */
	volatile uint16_t preempted;   /* 에러가 정상을 끊은 횟수               */
} VoiceStat;

extern VoiceStat g_voice;

/* ---- API ---------------------------------------------------------------- */

/* TIM2/DAC/DMA 를 설정하고 재생기를 대기 상태로 둔다.
 * MX_DAC_Init()·MX_SPI1_Init()·LM4871_BoardInit() 이후, 스케줄러 시작 전에 1회. */
void Voice_Init(void);

/* 음성 id 를 재생한다. **비블로킹** — 즉시 반환하고 DMA 가 흘린다.
 *
 * prio 에 따라 위 "우선순위·큐" 규칙이 적용된다. 시나리오는 정상 안내에
 * VOICE_PRIO_NORMAL, 에러 안내에 VOICE_PRIO_ERROR 를 준다
 * (`Voice_IsErrorId(id)` 로 표에서 확인할 수 있다).
 *
 * ★**요청만 남기고 즉시 돌아온다.** 플래시를 읽지도, 기다리지도 않는다 —
 * 시나리오는 MotorTask(1ms)에서 부르므로 여기서 앰프 세틀(5ms)·램프(8ms)·
 * SPI 선채움(2.2ms)을 하면 모터 틱을 7~11개 놓친다. 실제 시작은 Voice_Task
 * 가 하고, 그쪽이 프로브를 먼저 하므로 **무효한 ID 는 지금 나오는 소리를
 * 끊지 않고** 조용히 버려진다.
 *
 * 반환: VOICE_ST_OK = 요청 접수(또는 같은 ID 라 무시).
 *       VOICE_ST_SLOT = id 범위 밖.
 * ⚠ **그 ID 가 실제로 재생 가능한지는 알려 주지 않는다**(빈 슬롯 등).
 *    알아야 하면 `Voice_Probe()` 를 먼저 부를 것 — voiceupdater/tb_voice 가
 *    그렇게 한다. 시나리오는 알 필요가 없다(없으면 조용히 안 나올 뿐). */
voice_status_t Voice_Play(voice_id_t id, voice_prio_t prio);

/* 정상 안내 두 개를 **순서대로 이어서** 재생한다(first 가 끝나면 second).
 * 비블로킹, 정상 우선순위. 대기 중이던 정상 요청은 이것으로 대체된다.
 * first 가 지금 재생 중인 ID 와 같으면 first 는 건너뛰고 second 만 잇는다.
 * 반환: VOICE_ST_OK / VOICE_ST_SLOT(둘 중 하나라도 범위 밖이면 아무것도 안 넣음). */
voice_status_t Voice_PlaySeq2(voice_id_t first, voice_id_t second);

/* 재생을 멈추고 **큐도 비운다**(꼬리를 무음으로 채우고 정리).
 * 큐를 남기면 멈춘 직후 다음 것이 튀어나온다. */
void Voice_Stop(void);

/* 1 = 재생 중(꼬리 정리 포함). */
uint8_t Voice_IsBusy(void);

/* 슬롯 헤더만 읽어 길이/시간을 본다(재생 없이 목록 표시용).
 * out_len/out_dur_ms 는 NULL 가능. */
voice_status_t Voice_Probe(uint8_t slot, uint32_t *out_len, uint32_t *out_dur_ms);

/* 전용 태스크 본체. freertos.c 가 osThreadNew 로 띄운다. 돌아오지 않는다. */
void Voice_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_SPEAKER_VOICE_H_ */
