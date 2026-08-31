/* ==========================================================================
 * mode_arbiter.c - 마개 위치 홀센서(HS1~HS5) -> 동작 모드 중재자.
 * 설계 배경/규칙은 mode_arbiter.h 참조.
 * ========================================================================== */

#include "mode_arbiter.h"

#include "hallsensor.h"
#include "jungji.h"
#include "moeum.h"
#include "dongjak.h"
#include "kangeum.h"
#include "baesu.h"

/* g_app_mode 의 정의는 여기 하나뿐(이전에는 freertos.c에 있었다). 기본값은
 * 대기/벤치 - 부팅 직후에는 어떤 시나리오도 돌지 않는다. */
volatile app_mode_t g_app_mode = APP_MODE_TESTBENCH;

ModeArbCtx g_modearb;

/* ---- 내부 헬퍼 ----------------------------------------------------------- */

/* HS1~5 마스크를 마개 위치로 디코딩. 정확히 1비트만 서야 유효하다.
 * (홀 5개는 물리적으로 배타이므로 2개 이상 = 센서/자석 이상 또는 과도구간) */
static uint8_t modearb_decode(uint8_t mask)
{
	switch (mask & (uint8_t)MODEARB_HS_MASK)
	{
	case 0x01U: return (uint8_t)LID_POS_KANGEUM;  /* HS1 강음 */
	case 0x02U: return (uint8_t)LID_POS_DONGJAK;  /* HS2 동작 */
	case 0x04U: return (uint8_t)LID_POS_JUNGJI;   /* HS3 정지 */
	case 0x08U: return (uint8_t)LID_POS_BAESU;    /* HS4 배수 */
	case 0x10U: return (uint8_t)LID_POS_MOEUM;    /* HS5 모음 */
	case 0x00U: return (uint8_t)LID_POS_NONE;
	default:    return (uint8_t)LID_POS_MULTI;    /* 2비트 이상 */
	}
}

/* 위치 -> 모드. 시나리오가 붙지 않는 위치는 대기(TESTBENCH). */
static uint8_t modearb_pos_to_mode(uint8_t pos)
{
	switch ((LidPos)pos)
	{
	case LID_POS_KANGEUM: return (uint8_t)APP_MODE_KANGEUM;
	case LID_POS_MOEUM:   return (uint8_t)APP_MODE_MOEUM;
	case LID_POS_BAESU:   return (uint8_t)APP_MODE_BAESU;
	case LID_POS_DONGJAK: return (uint8_t)APP_MODE_DONGJAK;
	case LID_POS_JUNGJI:  return (uint8_t)APP_MODE_JUNGJI;
	default:              return (uint8_t)APP_MODE_TESTBENCH;
	}
}

/* 확정 위치가 바뀐 순간의 처리. 여기서는 요청만 남기고(모터를 만지지 않는다)
 * 실제 정지/전환/시작은 MotorTick 이 수행한다. */
static void modearb_on_pos_change(ModeArbCtx *a, uint8_t pos)
{
	switch ((LidPos)pos)
	{
	case LID_POS_JUNGJI:
		/* HS3 정지: 비상정지 후 APP_MODE_JUNGJI 로. 4.5 정지버튼과 동일 취급.
		 * ★대기(TESTBENCH)가 아니라 전용 모드로 가는 이유: 마개가 정지 위치에
		 *   머무는 동안 g_app_mode 로 그 사실이 보여야 한다(디버거/앱 STATUS).
		 *   실행 성격은 대기와 같다 - 시나리오 tick 없음, 벤치 폴링 유지.
		 *   빠져나오는 길: 마개를 다른 위치로(= 확정 에지) 또는 앱 정지 명령
		 *   (PROTO_ACT_STOP -> APP_MODE_TESTBENCH). */
		Jungji_Request(JUNGJI_SRC_HS_STOP, JUNGJI_KIND_EMERGENCY);
		a->pend_mode  = (uint8_t)APP_MODE_JUNGJI;
		a->pend_start = 0U;
		a->pend_valid = 1U;
		break;

	case LID_POS_NONE:
		/* 마개가 어느 위치에도 없다 = 열렸거나 이동 중. 4.5 "추가 투입 금지"
		 * 대상이므로 즉시 정지. 모드는 유지한다(헤더 §정지 규칙 참조). */
		Jungji_Request(JUNGJI_SRC_HS_LOST, JUNGJI_KIND_EMERGENCY);
		break;

	case LID_POS_MULTI:
		/* 2채널 이상 동시 인식. 자석/홀 이상이거나 확정 샘플이 과도구간에
		 * 걸린 경우. 어느 쪽이든 신뢰할 수 없으므로 정지만 한다. */
		Jungji_Request(JUNGJI_SRC_HS_INVALID, JUNGJI_KIND_EMERGENCY);
		break;

	case LID_POS_KANGEUM:
	case LID_POS_MOEUM:
	case LID_POS_BAESU:
	case LID_POS_DONGJAK:
	default:
		a->pend_mode  = modearb_pos_to_mode(pos);
		a->pend_start = 1U;
		a->pend_valid = 1U;
		break;
	}
}

/* ---- API ----------------------------------------------------------------- */

void ModeArbiter_Init(void)
{
	ModeArbCtx *a = &g_modearb;

	a->raw_mask      = 0U;
	a->pos_raw       = (uint8_t)LID_POS_NONE;
	a->pos_stable    = (uint8_t)LID_POS_NONE;
	a->cand          = (uint8_t)LID_POS_NONE;
	a->cand_cnt      = 0U;
	a->primed        = 0U;
	a->pend_mode     = (uint8_t)APP_MODE_TESTBENCH;
	a->pend_valid    = 0U;
	a->pend_start    = 0U;
	a->start_wait    = 0U;
	a->pend_heat     = 0U;
	a->heat_wait     = 0U;
	a->pend_cool     = 0U;
	a->cool_wait     = 0U;
	a->dbg_disable   = 0U;
	a->dbg_pos_force = 0U;
	a->transitions   = 0U;
	a->mode_changes  = 0U;

	g_app_mode = APP_MODE_TESTBENCH;
}

/* 100ms, StartDefaultTask. HallSensor_Update() 가 같은 루프에서 이미 스냅샷을
 * 갱신한 뒤에 호출되어야 한다. 센서만 본다 - 모터를 만지지 않는다. */
void ModeArbiter_SenseTick(void)
{
	ModeArbCtx *a = &g_modearb;
	uint8_t     mask;
	uint8_t     pos;

	mask = (uint8_t)(HallSensor_GetMask() & (uint8_t)MODEARB_HS_MASK);
	a->raw_mask = mask;

	pos = modearb_decode(mask);
	if (a->dbg_pos_force != 0U)
	{
		pos = a->dbg_pos_force;   /* 홀 없이 위치를 흉내내는 벤치용 주입 */
	}
	a->pos_raw = pos;

	if (a->dbg_disable != 0U)
	{
		return;                   /* 디버거 수동 모드: 중재 일체 중지 */
	}

	/* 디바운스: 동일 위치가 MODEARB_CONFIRM_SAMPLES 회 연속이어야 확정. */
	if (pos == a->cand)
	{
		if (a->cand_cnt < 255U) { a->cand_cnt++; }
	}
	else
	{
		a->cand     = pos;
		a->cand_cnt = 1U;
	}

	if (a->cand_cnt < (uint8_t)MODEARB_CONFIRM_SAMPLES) { return; }
	if (pos == a->pos_stable)                           { return; }

	a->pos_stable = pos;
	if (a->transitions < 0xFFFFU) { a->transitions++; }

	/* 부팅 후 첫 확정은 기준선만 잡는다. 전원 인가 시점에 마개가 이미 모음/동작
	 * 위치에 있어도 저절로 기동하지 않게 하기 위함(헤더 §부팅 시 동작). */
	if (a->primed == 0U)
	{
		a->primed = 1U;
		return;
	}

	modearb_on_pos_change(a, pos);
}

/* 1ms, StartMotorTask. 모드 전환의 유일한 실행 지점. Jungji_Tick() 보다 뒤에
 * 호출되어야 제동 해제 상태(Jungji_IsBraking)를 같은 틱에 반영할 수 있다. */
/* 외부(앱) 모드 요청. 헤더 주석 참조 - 홀센서 경로와 같은 pend_* 래치로 모은다. */
uint8_t ModeArbiter_RequestMode(app_mode_t mode, uint8_t start, uint8_t allow_stub)
{
	ModeArbCtx *a = &g_modearb;

	/* 중재자를 디버거로 무력화한 상태에서는 모드 소유자가 사람이므로 개입하지 않는다. */
	if (a->dbg_disable != 0U)
		return 0U;

	/* 미구현 스텁 모드(강음/배수)는 기본 거절. 스텁이 "시작됨"으로 보이면 오해를 준다. */
	if ((allow_stub == 0U) &&
	    ((mode == APP_MODE_KANGEUM) || (mode == APP_MODE_BAESU)))
		return 0U;

	a->pend_mode  = (uint8_t)mode;
	a->pend_start = (uint8_t)(start ? 1U : 0U);
	a->pend_heat  = 0U;             /* 일반 시작 요청은 직행 요청을 취소한다   */
	a->pend_cool  = 0U;
	a->pend_valid = 1U;
	return 1U;
}

uint8_t ModeArbiter_RequestDongjakHeat(uint8_t skip_door)
{
	ModeArbCtx *a = &g_modearb;

	if (a->dbg_disable != 0U)
		return 0U;

	a->pend_mode  = (uint8_t)APP_MODE_DONGJAK;
	a->pend_start = 0U;             /* 정상 시작(헹굼부터)은 하지 않는다 */
	a->pend_heat  = (uint8_t)(skip_door ? 2U : 1U);
	a->pend_cool  = 0U;             /* 둘은 배타적이다 */
	a->pend_valid = 1U;
	return 1U;
}

uint8_t ModeArbiter_RequestDongjakCool(void)
{
	ModeArbCtx *a = &g_modearb;

	if (a->dbg_disable != 0U)
		return 0U;

	a->pend_mode  = (uint8_t)APP_MODE_DONGJAK;
	a->pend_start = 0U;
	a->pend_heat  = 0U;             /* 둘은 배타적이다 */
	a->pend_cool  = 1U;
	a->pend_valid = 1U;
	return 1U;
}

void ModeArbiter_MotorTick(uint32_t now_ms)
{
	ModeArbCtx *a = &g_modearb;

	if (a->pend_valid != 0U)
	{
		a->pend_valid = 0U;

		if ((app_mode_t)a->pend_mode != g_app_mode)
		{
			/* 전 모드가 쥐고 있던 액추에이터/FSM 정리 후 전환. Jungji_StopAll 이
			 * 각 시나리오 Abort() 를 부르므로 Start() 는 반드시 이 뒤에 온다. */
			Jungji_StopAll(JUNGJI_SRC_MODE_SWITCH, JUNGJI_KIND_NORMAL, now_ms);
			g_app_mode = (app_mode_t)a->pend_mode;
			if (a->mode_changes < 0xFFFFU) { a->mode_changes++; }
		}
		a->start_wait = a->pend_start;
		a->pend_start = 0U;
		a->heat_wait  = a->pend_heat;
		a->pend_heat  = 0U;
		a->cool_wait  = a->pend_cool;
		a->pend_cool  = 0U;
	}

	/* 정지/제동이 끝난 뒤에 시작 지령. 제동 홀드 중에 Start 하면 BLDC 가
	 * 단락제동 상태에서 기동 지령을 받게 되므로 반드시 기다린다. */
	if (((a->start_wait != 0U) || (a->heat_wait != 0U) || (a->cool_wait != 0U)) &&
	    (Jungji_IsBraking() == 0U))
	{
		uint8_t heat  = a->heat_wait;
		uint8_t cool  = a->cool_wait;
		a->start_wait = 0U;
		a->heat_wait  = 0U;
		a->cool_wait  = 0U;

		if (cool != 0U)
		{
			/* ★식힘 직행. 세우는 지점이 heat 와 같은 이유는 위 주석 참조. */
			if (g_app_mode == APP_MODE_DONGJAK)
			{
				g_dongjak.dbg_enter_cool = 1U;
			}
		}
		else if (heat != 0U)
		{
			/* ★DJ_HEAT 직행(헹굼 생략). 여기서 세우는 이유가 중요하다:
			 * 이 지점은 (1) 모드 전환의 Jungji_StopAll(=Dongjak_Abort, 이 변수를
			 * 0으로 지운다) 이후이고 (2) 제동 홀드가 끝난 뒤다. 요청 시점에
			 * g_dongjak.dbg_enter_heat 를 직접 세우면 전환 정리에 지워진다.
			 * 실제 점프는 Dongjak_MotorTick 이 이 값을 1회 소비하며 수행한다. */
			if (g_app_mode == APP_MODE_DONGJAK)
			{
				g_dongjak.dbg_enter_heat = heat;
			}
		}
		else
		{
			switch (g_app_mode)
			{
			case APP_MODE_MOEUM:   Moeum_Start();   break;
			case APP_MODE_DONGJAK: Dongjak_Start(); break;
			case APP_MODE_KANGEUM: Kangeum_Start(); break;
			case APP_MODE_BAESU:   Baesu_Start();   break;
			case APP_MODE_JUNGJI:                   /* 정지: 시작할 것이 없다 */
			case APP_MODE_TESTBENCH:
			default:                                break;
			}
		}
	}
}

LidPos ModeArbiter_GetPos(void)
{
	return (LidPos)g_modearb.pos_stable;
}

uint8_t ModeArbiter_PosIsRunnable(LidPos p)
{
	return (uint8_t)((p == LID_POS_KANGEUM) || (p == LID_POS_MOEUM) ||
	                 (p == LID_POS_BAESU)   || (p == LID_POS_DONGJAK));
}
