#include "voice.h"

#include "w25q128.h"
#include "lm4871.h"
#include "dac.h"
#include "cmsis_os.h"
#include <string.h>

/* U21 슬롯 재생기.  ★R3 신규(4단계, §0.14.6 / 바지-인 §0.14.7).
 * 신호 경로·클럭·.ioc 관계는 voice.h 참조. */

VoiceStat g_voice;

/* ---- 슬롯 헤더 오프셋 (설계서 §3) ------------------------------------- */
#define HDR_O_MAGIC      0x00u
#define HDR_O_ID         0x04u
#define HDR_O_DATA_LEN   0x0Cu
#define HDR_O_DUR_MS     0x18u
#define HDR_BYTES        0x100u

/* ---- 하드웨어 핸들 ------------------------------------------------------ */
static TIM_HandleTypeDef s_tim2;
static DMA_HandleTypeDef s_dma;

/* ---- 재생 상태 ---------------------------------------------------------- */
static uint16_t s_pcm[VOICE_BUF];      /* 핑퐁 4KB */
static uint32_t s_addr;                /* 다음에 읽을 플래시 주소 */
static uint32_t s_left;                /* 아직 읽지 않은 PCM 바이트 */
static volatile uint8_t s_half_req;    /* bit0 = 전반부 채워야 함, bit1 = 후반부 */
static volatile uint8_t s_tail;        /* 무음으로 채운 반쪽 수(끝 판정) */
static osSemaphoreId_t s_sem;
/* 버퍼 채우기 상호배제. Voice_Task 가 SPI 로 반쪽을 채우는 동안 Voice_Play 가
 * 끼어들어 DMA 를 멈추고 다시 걸면, 태스크가 끝낸 옛 읽기가 새 재생의 버퍼를
 * 덮어쓴다(SPI 2KB ≈ 1.1ms 의 창). 바지-인을 넣으면서 실제로 열린 구멍이라
 * 뮤텍스로 닫는다. */
static osMutexId_t s_fill_mtx;
/* 재생 세대. Play 마다 증가 — 태스크가 자기가 시작된 재생이 아직 유효한지
 * 확인하는 데 쓴다(옛 재생의 꼬리 정리가 새 재생을 끄는 것을 막는다). */
static volatile uint8_t s_gen;

/* ---- 큐 (설계서 §7.4, voice.h "우선순위·큐") --------------------------
 * 정상 1칸 + 에러 FIFO 4칸. 인덱스 조작은 짧아서 임계구역 없이 두되,
 * 요청(태스크/시나리오 문맥)과 꺼내기(Voice_Task) 가 겹치지 않도록
 * 둘 다 s_fill_mtx 안에서만 만진다. */
static volatile uint8_t s_nq;                   /* 정상 1칸. VOICE_COUNT = 빔 */
static volatile uint8_t s_nq2;                  /* 정상 연결 1칸(PlaySeq2). s_nq 가 빠지면 올라간다 */
static volatile uint8_t s_eq[VOICE_ERRQ_DEPTH]; /* 에러 FIFO */
static volatile uint8_t s_eq_head, s_eq_tail, s_eq_cnt;

/* ★요청 큐는 **MotorTask(1ms 시나리오)** 가 쓰고 **Voice_Task** 가 읽는다.
 * 뮤텍스를 쓰면 시나리오가 재생 준비(앰프 세틀·램프·SPI 선채움, 합 7~11ms)
 * 동안 막혀 모터 틱을 놓친다 — 그래서 인덱스 조작만 짧게 PRIMASK 로 막는다
 * (uart_ctrl.c 와 같은 기조). 버퍼·DMA·SPI 는 여전히 s_fill_mtx 가 지킨다. */
static inline uint32_t crit_enter(void)
{
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	return pm;
}

static inline void crit_exit(uint32_t pm)
{
	if (!pm) __enable_irq();
}

/* ---- 리틀엔디안 -------------------------------------------------------- */
static uint32_t rd32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
	     | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* ================================================= 하드웨어 설정 (코드 직접)
 * .ioc 에 없는 것을 여기서 만든다 — 이유는 voice.h "★.ioc 와의 관계" 참조. */

static void tim2_init(void)
{
	TIM_MasterConfigTypeDef mc = {0};

	__HAL_RCC_TIM2_CLK_ENABLE();

	s_tim2.Instance               = TIM2;
	s_tim2.Init.Prescaler         = VOICE_TIM_PSC;
	s_tim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
	s_tim2.Init.Period            = VOICE_TIM_ARR;
	s_tim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
	s_tim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	(void)HAL_TIM_Base_Init(&s_tim2);

	/* Update 이벤트를 TRGO 로 내보낸다 = DAC 의 샘플 클럭. */
	mc.MasterOutputTrigger = TIM_TRGO_UPDATE;
	mc.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
	(void)HAL_TIMEx_MasterConfigSynchronization(&s_tim2, &mc);
}

static void dma_init(void)
{
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DAC_CH1 은 F103 고밀도에서 DMA2_Channel3 에 붙는다. */
	s_dma.Instance                 = DMA2_Channel3;
	s_dma.Init.Direction           = DMA_MEMORY_TO_PERIPH;
	s_dma.Init.PeriphInc           = DMA_PINC_DISABLE;
	s_dma.Init.MemInc              = DMA_MINC_ENABLE;
	s_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	s_dma.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
	s_dma.Init.Mode                = DMA_CIRCULAR;
	s_dma.Init.Priority            = DMA_PRIORITY_MEDIUM;
	(void)HAL_DMA_Init(&s_dma);

	__HAL_LINKDMA(&hdac, DMA_Handle1, s_dma);

	HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
}

static void dac_trigger_t2(void)
{
	DAC_ChannelConfTypeDef cfg = {0};

	/* MX_DAC_Init 은 Trigger=NONE 으로 잡아 둔다(비프가 SetValue 로 쓰므로).
	 * 재생할 때만 T2_TRGO 로 바꾸고, 끝나면 dac_trigger_none() 으로 되돌린다 —
	 * 그래야 LM4871_Beep 의 소프트 톤 생성이 계속 동작한다. */
	cfg.DAC_Trigger      = DAC_TRIGGER_T2_TRGO;
	cfg.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	(void)HAL_DAC_ConfigChannel(&hdac, &cfg, DAC_CHANNEL_1);
}

static void dac_trigger_none(void)
{
	DAC_ChannelConfTypeDef cfg = {0};

	cfg.DAC_Trigger      = DAC_TRIGGER_NONE;
	cfg.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	(void)HAL_DAC_ConfigChannel(&hdac, &cfg, DAC_CHANNEL_1);
}

/* ★DMA2_Channel3 ISR. `.ioc` 에서 이 NVIC 를 켜면 stm32f1xx_it.c 에도 같은
 * 심볼이 생겨 링크가 깨진다 — 그때는 이 정의를 지우고 그쪽에서
 * HAL_DMA_IRQHandler(&s_dma) 를 부르게 할 것(voice.h 참조). */
void DMA2_Channel3_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&s_dma);
}

/* ================================================= 버퍼 채우기 (태스크 문맥) */

/* 반쪽 하나를 무음(중간값)으로 채운다. */
static void fill_silence(uint16_t *dst)
{
	for (uint32_t i = 0u; i < VOICE_HALF; i++)
		dst[i] = VOICE_MID;
}

/* 반쪽 하나를 플래시에서 채운다. 남은 데이터가 모자라면 뒤를 무음으로 메운다.
 * 반환 1 = 이번 반쪽에 실제 데이터가 들어갔다, 0 = 전부 무음(= 꼬리). */
static uint8_t fill_half(uint16_t *dst)
{
	uint32_t n = s_left;

	if (n == 0u)
	{
		fill_silence(dst);
		return 0u;
	}
	if (n > (VOICE_HALF * 2u))
		n = VOICE_HALF * 2u;            /* 반쪽 = 1024 샘플 = 2048 바이트 */

	if (W25Q_Read(s_addr, (uint8_t *)dst, n) != W25Q_OK)
	{
		g_voice.last_err = (uint8_t)VOICE_ST_FLASH;
		fill_silence(dst);
		s_left = 0u;
		return 0u;
	}
	s_addr += n;
	s_left -= n;
	g_voice.played += n;

	/* 마지막 조각이 반쪽을 못 채우면 나머지는 무음으로. */
	if (n < (VOICE_HALF * 2u))
	{
		for (uint32_t i = n / 2u; i < VOICE_HALF; i++)
			dst[i] = VOICE_MID;
	}
	g_voice.refills++;
	return 1u;
}

/* DWT 마이크로초 지연. lm4871.c 와 같은 방식이고, DWT 는 LM4871_BoardInit()
 * 이 이미 켜 둔다(main.c 에서 Voice_Init 보다 먼저). 여기서 다시 켜는 것은
 * 멱등이라 순서가 바뀌어도 안전하다. 램프 한 칸이 수십 us 라 osDelay 로는
 * 만들 수 없어 busy-wait 를 쓴다(총 5ms). */
static void voice_delay_us(uint32_t us)
{
	uint32_t start = DWT->CYCCNT;
	uint32_t ticks = us * (SystemCoreClock / 1000000u);

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
	while ((DWT->CYCCNT - start) < ticks) { /* busy-wait */ }
}

/* 현재 출력값에서 중간값까지 선형으로 내린다 — 전환 팝 억제(voice.h 참조). */
static void ramp_to_mid(uint16_t from)
{
	const int32_t f  = (int32_t)from;
	const int32_t d  = (int32_t)VOICE_MID - f;
	const uint32_t us = (VOICE_RAMP_MS * 1000u) / VOICE_RAMP_STEPS;

	if (d == 0) return;                 /* 이미 중간값(정상 종료) - 할 일 없다 */

	for (uint32_t i = 1u; i <= VOICE_RAMP_STEPS; i++)
	{
		int32_t v = f + ((d * (int32_t)i) / (int32_t)VOICE_RAMP_STEPS);
		(void)HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)v);
		voice_delay_us(us);
	}
}

/* DMA/TIM 을 끊고 DAC 를 **램프로** 중간값까지 내린다. **앰프는 켜 둔 채**다 —
 * 바지-인에서 곧바로 다음 재생이 이어지기 때문이다. 앰프를 껐다 켜면 그때마다
 * 5ms 세틀 + 팝이 붙는다. */
static void stop_now(void)
{
	uint16_t last;

	/* ★순서: **타이머를 먼저** 세운다. 그래야 DAC 가 더는 변환하지 않고 마지막
	 * 값을 유지하므로, 아래에서 읽는 DOR 이 '실제로 스피커에 걸려 있는 전압'이다.
	 * DMA 를 먼저 끄면 그 사이 트리거가 몇 번 더 들어와 값이 달라진다. */
	(void)HAL_TIM_Base_Stop(&s_tim2);
	last = (uint16_t)HAL_DAC_GetValue(&hdac, DAC_CHANNEL_1);

	(void)HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);   /* 채널이 잠깐 꺼진다 */

	dac_trigger_none();
	(void)HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	/* 재활성화 직후 **끊긴 그 값으로 되돌려** 둔다. 곧장 중간값을 쓰면 여기서
	 * 계단이 생겨 램프를 넣은 의미가 없어진다. */
	(void)HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)last);

	ramp_to_mid(last);

	g_voice.playing  = 0u;
	g_voice.stopping = 0u;
	s_half_req = 0u;
	s_tail     = 0u;
	s_left     = 0u;
}

/* 재생 정리: 즉시 정지 + 앰프 끄기(팝 방지). */
static void teardown(void)
{
	stop_now();
	osDelay(VOICE_MUTE_DELAY_MS);   /* DAC 가 중간값에 자리잡은 뒤 앰프 차단 */
	LM4871_Disable(&lm4871);
}

/* ===================================================================== ISR */

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hd)
{
	(void)hd;
	if (s_half_req & 0x01u) g_voice.underrun++;   /* 아직 안 채웠는데 또 왔다 */
	s_half_req |= 0x01u;                          /* 전반부가 비었다 */
	if (s_sem != NULL) (void)osSemaphoreRelease(s_sem);
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hd)
{
	(void)hd;
	if (s_half_req & 0x02u) g_voice.underrun++;
	s_half_req |= 0x02u;                          /* 후반부가 비었다 */
	if (s_sem != NULL) (void)osSemaphoreRelease(s_sem);
}

/* ---- 큐 조작 (s_fill_mtx 를 쥔 채로만 부른다) ------------------------- */
static void q_clear(void)
{
	uint32_t pm = crit_enter();

	s_nq = (uint8_t)VOICE_COUNT;
	s_nq2 = (uint8_t)VOICE_COUNT;
	s_eq_head = 0u; s_eq_tail = 0u; s_eq_cnt = 0u;
	g_voice.nq_id  = (uint8_t)VOICE_COUNT;
	g_voice.nq2_id = (uint8_t)VOICE_COUNT;
	g_voice.eq_cnt = 0u;
	crit_exit(pm);
}

static void q_push_err(uint8_t id)
{
	uint32_t pm = crit_enter();

	if (s_eq_cnt >= VOICE_ERRQ_DEPTH) { g_voice.dropped++; }
	else
	{
		s_eq[s_eq_tail] = id;
		s_eq_tail = (uint8_t)((s_eq_tail + 1u) % VOICE_ERRQ_DEPTH);
		s_eq_cnt++;
		g_voice.eq_cnt = s_eq_cnt;
	}
	crit_exit(pm);
}

/* 다음에 재생할 것을 꺼낸다. 에러가 항상 먼저다. 없으면 VOICE_COUNT. */
static uint8_t q_pop(uint8_t *out_prio)
{
	uint32_t pm = crit_enter();
	uint8_t  id = (uint8_t)VOICE_COUNT;

	if (s_eq_cnt != 0u)                       /* 에러가 항상 먼저다 */
	{
		id = s_eq[s_eq_head];
		s_eq_head = (uint8_t)((s_eq_head + 1u) % VOICE_ERRQ_DEPTH);
		s_eq_cnt--;
		g_voice.eq_cnt = s_eq_cnt;
		*out_prio = (uint8_t)VOICE_PRIO_ERROR;
	}
	else if (s_nq < (uint8_t)VOICE_COUNT)
	{
		id = s_nq;
		s_nq  = s_nq2;                        /* 연결이 있으면 다음 차례로 올린다 */
		s_nq2 = (uint8_t)VOICE_COUNT;
		g_voice.nq_id  = s_nq;
		g_voice.nq2_id = s_nq2;
		*out_prio = (uint8_t)VOICE_PRIO_NORMAL;
	}
	crit_exit(pm);
	return id;
}

/* 대기 중인 에러 요청이 있나(정상 재생을 끊을지 판단용). */
static uint8_t q_has_err(void)
{
	return (uint8_t)(s_eq_cnt != 0u);
}

/* =================================================================== 공개 */

void Voice_Init(void)
{
	memset((void *)&g_voice, 0, sizeof g_voice);
	s_half_req = 0u;
	s_tail     = 0u;
	s_left     = 0u;

	tim2_init();
	dma_init();

	if (s_sem == NULL)
		s_sem = osSemaphoreNew(4u, 0u, NULL);     /* HT/TC 신호용 카운팅 */
	if (s_fill_mtx == NULL)
		s_fill_mtx = osMutexNew(NULL);
	q_clear();
}

voice_status_t Voice_Probe(uint8_t slot, uint32_t *out_len, uint32_t *out_dur_ms)
{
	uint8_t  hdr[HDR_BYTES];
	uint32_t len;
	uint8_t  blank = 1u;

	if (slot >= VOICE_SLOT_COUNT) return VOICE_ST_SLOT;

	if (W25Q_Read(VOICE_SLOT_BASE((uint32_t)slot), hdr, HDR_BYTES) != W25Q_OK)
		return VOICE_ST_FLASH;

	for (uint32_t i = 0u; i < HDR_BYTES; i++)
	{
		if (hdr[i] != 0xFFu) { blank = 0u; break; }
	}
	if (blank) return VOICE_ST_EMPTY;

	if ((hdr[HDR_O_MAGIC] != (uint8_t)'Z') || (hdr[HDR_O_MAGIC + 1u] != (uint8_t)'G')
	 || (hdr[HDR_O_MAGIC + 2u] != (uint8_t)'V') || (hdr[HDR_O_MAGIC + 3u] != (uint8_t)'F')
	 || (hdr[HDR_O_ID] != slot))
		return VOICE_ST_HEADER;

	len = rd32(&hdr[HDR_O_DATA_LEN]);
	if ((len == 0u) || (len > VOICE_DATA_MAX)) return VOICE_ST_HEADER;

	if (out_len)    *out_len    = len;
	if (out_dur_ms) *out_dur_ms = rd32(&hdr[HDR_O_DUR_MS]);
	return VOICE_ST_OK;
}

/* 실제 시작. **s_fill_mtx 를 쥔 채** 부른다. 큐 판단은 호출자가 끝낸 상태. */
static voice_status_t start_locked(uint8_t slot, uint8_t prio, uint32_t len, uint32_t dur)
{

	s_gen++;                       /* 옛 재생의 꼬리 정리를 무효화한다 */

	/* ★순서 주의: 정지를 **새 상태를 세팅하기 전에** 해야 한다.
	 * stop_now() 는 s_left/s_half_req/s_tail 을 0 으로 되돌리므로, 뒤에 부르면
	 * 방금 세팅한 새 슬롯 길이를 지워 재생이 통째로 무음이 된다.
	 *
	 * 시작 순서(§7.3): (정지 또는 앰프 ON) -> 틈 -> 반쪽 2개 선채움 -> DMA -> TIM.
	 * 바지-인이면 앰프는 이미 켜져 있다 — 껐다 켜면 그때마다 팝이 붙으므로
	 * DMA 만 끊고 DAC 가 중간값에 자리잡을 짧은 틈만 준다. */
	if (g_voice.playing)
	{
		stop_now();
		osDelay(VOICE_BARGE_MS);
	}
	else
	{
		LM4871_Enable(&lm4871);
		osDelay(VOICE_SETTLE_MS);
	}

	g_voice.slot     = slot;
	g_voice.prio     = prio;
	g_voice.data_len = len;
	g_voice.dur_ms   = dur;
	g_voice.played   = 0u;
	g_voice.refills  = 0u;
	g_voice.underrun = 0u;
	g_voice.last_err = (uint8_t)VOICE_ST_OK;

	s_addr     = VOICE_SLOT_BASE((uint32_t)slot) + VOICE_SLOT_DATA_OFF;
	s_left     = len;
	s_half_req = 0u;
	s_tail     = 0u;

	(void)fill_half(&s_pcm[0]);
	(void)fill_half(&s_pcm[VOICE_HALF]);

	dac_trigger_t2();
	g_voice.playing = 1u;
	g_voice.plays++;

	if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)s_pcm,
	                      (uint32_t)VOICE_BUF, DAC_ALIGN_12B_R) != HAL_OK)
	{
		g_voice.playing  = 0u;
		g_voice.last_err = (uint8_t)VOICE_ST_FLASH;
		dac_trigger_none();
		LM4871_Disable(&lm4871);
		return VOICE_ST_FLASH;
	}
	(void)HAL_TIM_Base_Start(&s_tim2);
	return VOICE_ST_OK;
}

voice_status_t Voice_Play(voice_id_t id, voice_prio_t prio)
{
	uint8_t slot = (uint8_t)id;

	if (slot >= (uint8_t)VOICE_COUNT) return VOICE_ST_SLOT;

	/* ★여기서는 **요청만 남긴다.** 플래시도 읽지 않고 기다리지도 않는다.
	 *
	 * 시나리오(moeum/dongjak)는 **MotorTask 1ms** 에서 이 함수를 부른다.
	 * 재생 준비는 앰프 세틀 5ms + (바지-인이면) 램프 8ms + SPI 선채움 2.2ms 라
	 * 여기서 해 버리면 **모터 틱을 7~11개 놓친다.** 게다가 Voice_Probe 의 SPI
	 * 접근이 Voice_Task 의 반쪽 채우기와 락 없이 겹친다(w25q128 에 락이 없다).
	 * 그래서 판정·플래시·DMA 는 전부 Voice_Task 로 넘기고, 이 함수는
	 * "무엇을 틀어 달라"만 남기고 즉시 돌아온다 — 설계서 §7.3 의 비블로킹 요구.
	 *
	 * 대신 **요청한 ID 가 유효한지는 여기서 알 수 없다.** 호출자가 알아야 하면
	 * Voice_Probe() 를 먼저 부를 것(voiceupdater/tb_voice 가 그렇게 한다). */
	if (g_voice.playing && (g_voice.slot == slot))
		return VOICE_ST_OK;        /* §7.4: 재생 중인 것과 같은 ID 는 무시 */

	if (prio == VOICE_PRIO_ERROR)
	{
		q_push_err(slot);
	}
	else
	{
		uint32_t pm = crit_enter();
		s_nq  = slot;              /* 정상은 1칸 - 최신 것만 남는다 */
		s_nq2 = (uint8_t)VOICE_COUNT; /* PlaySeq2 연결도 함께 버린다(상태가 바뀌었다) */
		g_voice.nq_id  = s_nq;
		g_voice.nq2_id = s_nq2;
		crit_exit(pm);
	}

	/* 태스크를 깨운다. 재생 중이 아니면 DMA 인터럽트가 없어 깨울 사람이 없다. */
	if (s_sem != NULL) (void)osSemaphoreRelease(s_sem);
	return VOICE_ST_OK;
}

voice_status_t Voice_PlaySeq2(voice_id_t first, voice_id_t second)
{
	uint8_t  a = (uint8_t)first;
	uint8_t  b = (uint8_t)second;
	uint32_t pm;

	if ((a >= (uint8_t)VOICE_COUNT) || (b >= (uint8_t)VOICE_COUNT)) return VOICE_ST_SLOT;

	/* Voice_Play 와 같은 이유로 요청만 남긴다. 두 칸을 **한 임계구역 안에서**
	 * 세워야 Voice_Task 가 중간 상태(a 만 있고 b 없음)를 꺼내 가지 않는다. */
	pm = crit_enter();
	if (g_voice.playing && (g_voice.slot == a))
	{
		s_nq  = b;                 /* a 는 이미 나오는 중 — b 만 잇는다 */
		s_nq2 = (uint8_t)VOICE_COUNT;
	}
	else
	{
		s_nq  = a;
		s_nq2 = b;
	}
	g_voice.nq_id  = s_nq;
	g_voice.nq2_id = s_nq2;
	crit_exit(pm);

	if (s_sem != NULL) (void)osSemaphoreRelease(s_sem);
	return VOICE_ST_OK;
}

void Voice_Stop(void)
{
	/* 큐를 남기면 멈춘 직후 다음 것이 튀어나온다. */
	if (s_fill_mtx != NULL) (void)osMutexAcquire(s_fill_mtx, osWaitForever);
	q_clear();
	if (s_fill_mtx != NULL) (void)osMutexRelease(s_fill_mtx);

	if (!g_voice.playing) return;
	/* 즉시 끊지 않는다 — 남은 데이터를 버리고 꼬리를 무음으로 흘려보내
	 * 스피커에 스텝(팝)이 실리지 않게 한다. 정리는 태스크가 한다. */
	s_left           = 0u;
	g_voice.stopping = 1u;
}

uint8_t Voice_IsBusy(void)
{
	return g_voice.playing;
}

void Voice_Task(void *argument)
{
	uint8_t gen;
	uint8_t ended;

	(void)argument;

	for (;;)
	{
		/* 타임아웃을 둔다 — 재생 중이 아니면 DMA 인터럽트가 없으므로, 요청이
		 * 세마포어를 놓치더라도 여기서 주기적으로 큐를 다시 본다(안전망). */
		if (s_sem != NULL)
			(void)osSemaphoreAcquire(s_sem, 20u);
		else
			osDelay(10u);

		if (s_fill_mtx != NULL)
			(void)osMutexAcquire(s_fill_mtx, osWaitForever);

		ended = 0u;

		/* ---- 1) 대기 중인 요청 처리 -------------------------------------
		 * 시작/정지/플래시 접근을 전부 이 문맥으로 모았다(Voice_Play 주석 참조).
		 * 정상 재생 중에 에러가 대기하면 §7.4 대로 **정상을 끊고** 에러로 간다. */
		if ((!g_voice.playing) ||
		    (q_has_err() && (g_voice.prio == (uint8_t)VOICE_PRIO_NORMAL)))
		{
			uint8_t nprio = (uint8_t)VOICE_PRIO_NORMAL;
			uint8_t nid   = q_pop(&nprio);

			if (nid < (uint8_t)VOICE_COUNT)
			{
				uint32_t len = 0u;
				uint32_t dur = 0u;

				/* 프로브가 먼저다 — 요청한 ID 가 빈 슬롯이면 지금 나오는
				 * 소리를 끊지 않고 조용히 버린다. */
				if (Voice_Probe(nid, &len, &dur) == VOICE_ST_OK)
				{
					if (g_voice.playing) g_voice.preempted++;
					(void)start_locked(nid, nprio, len, dur);
				}
				else
				{
					g_voice.last_err = (uint8_t)VOICE_ST_EMPTY;
				}
			}
		}

		if (!g_voice.playing)
		{
			if (s_fill_mtx != NULL) (void)osMutexRelease(s_fill_mtx);
			continue;
		}

		/* 기다리는 동안 Voice_Play 가 끼어들었을 수 있다. 그 경우 아래 작업은
		 * 전부 옛 재생 것이라 버린다 — 특히 teardown() 이 새 재생을 꺼 버린다. */
		gen = s_gen;

		/* 비워진 반쪽을 채운다. 둘 다 비었으면(언더런) 둘 다 채운다. */
		while ((s_half_req != 0u) && (gen == s_gen) && g_voice.playing)
		{
			uint8_t  req = s_half_req;
			uint16_t *dst;

			if (req & 0x01u) { dst = &s_pcm[0];          s_half_req &= (uint8_t)~0x01u; }
			else             { dst = &s_pcm[VOICE_HALF]; s_half_req &= (uint8_t)~0x02u; }

			if (fill_half(dst) == 0u)
			{
				/* 무음만 들어갔다 = 데이터 끝. 무음 반쪽이 DMA 로 실제
				 * 나갈 때까지(2회) 기다렸다가 정리해야 꼬리가 잘리지 않는다. */
				s_tail++;
				if (s_tail >= 2u)
				{
					if (gen == s_gen)
					{
						teardown();
						ended = 1u;       /* 큐에서 다음 것을 꺼낼 차례 */
					}
					break;
				}
			}
		}

		if (s_fill_mtx != NULL) (void)osMutexRelease(s_fill_mtx);

		/* 하나가 끝났으면 다음 바퀴의 1) 이 큐에서 이어 받는다. 여기서 바로
		 * 시작하지 않는 이유: teardown() 이 앰프까지 내린 직후라, 다음 것을
		 * 같은 바퀴에 걸면 껐다 켜는 꼴이 된다. 한 바퀴(최대 20ms) 늦는 대신
		 * 시작 경로가 한 곳으로 모인다. */
		if (ended && s_sem != NULL) (void)osSemaphoreRelease(s_sem);
	}
}
