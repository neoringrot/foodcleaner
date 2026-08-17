#ifndef DEVICES_COMM_PROTOCOL_R0_H_
#define DEVICES_COMM_PROTOCOL_R0_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * protocol_r0 - ZEROGEO 앱<->장치 시리얼 프로토콜 R0.
 *
 * ★기준 원본: doc/R1/zerogeo_protocol_r0.xlsx ("패킷구조R0" 시트)
 *   §1 패킷구조 / §2 명령어 구조. 전송로는 UART5(Core/Interface/uart_ctrl.c),
 *   물리적으로는 BLE 모듈 BYPASS 통과 = 앱과 직결.
 *
 * --------------------------------------------------------------------------
 * §1. 패킷 구조 (xlsx §1 그대로)
 *
 *   STX | R/W | CMD | LEN | DATA[N] | ETX
 *    1     1     1     2       N       2      <- 할당크기(DLE 미포함)
 *
 *   STX = 0x02                패킷 시작
 *   ETX = 0x0D 0x0A           패킷 끝 (ETX1=0x0D, ETX2=0x0A)
 *   R/W : 0x00 = W (쓰기)  0x01 = R (요청)  0x02 = M (장치가 즉시 통보)
 *   LEN : DATA 의 길이 N. [DLE] 개수는 포함하지 않는다.
 *   DLE = 0x10                STX/ETX/DLE 값 앞에 붙는 이스케이프
 *   ACK = 0x06  NAK = 0x15    W 명령의 처리 결과
 *
 *   이스케이프는 R/W, CMD, LEN, DATA 에만 적용된다(프레임 선두 STX 와 말미 ETX
 *   자체는 생(raw) 바이트). 이스케이프 대상 값 = {0x02, 0x0D, 0x0A, 0x10}.
 *     [DLE][STX] 0x10 0x02           -> 데이터 0x02
 *     [DLE][ETX] 0x10 0x0D 0x10 0x0A -> 데이터 0x0D 0x0A
 *     [DLE][DLE] 0x10 0x10           -> 데이터 0x10
 *
 *   ⚠ 앱 구현자 주의: M = 0x02 는 STX 와 값이 같으므로 "R/W 필드에도 이스케이프를
 *     적용한다"는 위 규칙에 따라 모든 M 프레임의 R/W 바이트가 [DLE]0x02 로 나간다.
 *     즉 모니터링 프레임의 실제 선두는 항상
 *         0x02 0x10 0x02 <CMD> <LEN…>
 *     이다(첫 0x02 = STX, 두 번째 0x02 = 이스케이프된 M). W(0x00)/R(0x01) 은
 *     이스케이프 대상이 아니라 이 현상이 없다. 규격대로 DLE 를 벗기는 디코더면
 *     자동으로 처리되지만, "STX 다음 바이트 = R/W" 로 단순 파싱하면 M 을 놓친다.
 *
 * §2. 방향별 규칙 (xlsx §2)
 *   App -> Device :  W | CMD | LEN | DATA      (쓰기)
 *                    R | CMD | 1   | 0x00      (요청)
 *   Device -> App :  W | CMD | 1   | ACK/NAK   (쓰기에 대한 응답)
 *                    R | CMD | N   | DATA      (요청에 대한 응답)
 *                    M | CMD | N   | DATA      (요청 없이 즉시 = 모니터링)
 *   ★W 처리 규칙: 펌웨어는 값을 쓴 뒤 "다시 읽어서 검증"하고, 같으면 ACK,
 *     다르면 NAK 를 보낸다. Proto_HandleWrite() 가 이 순서를 강제한다.
 *
 * --------------------------------------------------------------------------
 * §3. LEN 바이트 순서 - ★리틀엔디안 확정 (2026-08-17)
 *   원본 xlsx 는 LEN 이 2바이트라는 것만 정하고 엔디안을 명시하지 않았다.
 *   → 리틀엔디안(하위바이트 먼저)으로 확정했다. 예: N=16 -> 0x10 0x00.
 *   DATA 안의 u16/i16/u32 도 모두 같은 규칙으로 직렬화한다.
 *   xlsx(doc/R1/zerogeo_protocol_r0.xlsx) 의 LEN 행과 [페이로드] 시트에도
 *   같은 내용이 반영되어 있다.
 *   PROTO_LEN_BIG_ENDIAN 매크로는 만약의 상호운용 문제에 대비해 남겨두지만,
 *   기본값 0(리틀엔디안)이 규격이다. 바꾸려면 앱과 동시에 바꿔야 한다.
 *
 * §4. CMD 값 - ⚠ 문서 미정의
 *   xlsx §2 "명령어 구조" 표의 CMD/CMD값 칸이 비어 있다(R/W/M 행만 있음).
 *   따라서 아래 proto_cmd_t 는 이 펌웨어가 임시 배정한 값이다. 앱과 표를 확정한
 *   뒤 값만 갱신하면 되도록 한 곳에 모아 두었다.
 *   배정 원칙: 이스케이프 대상값 {0x02,0x0A,0x0D,0x10} 을 CMD 로 쓰지 않는다
 *   (스니퍼 로그에서 DLE 가 섞여 읽기 어려워지는 것을 피한다).
 *
 * §5. 모니터링(M) 설계
 *   M 은 앱의 요청 없이 장치가 밀어 올린다. 두 가지 트리거를 쓴다.
 *     (a) 주기 송신 : g_proto.mon_period_ms 마다 g_proto.mon_mask 의 패킷들
 *     (b) 변화 즉시 : app_mode / 시나리오 상태 / 에러코드가 바뀌면 그 즉시
 *                     STATUS(+해당 시나리오) 를 1회 밀어 올린다
 *                     (PROTO_MON_ON_CHANGE, 기본 1)
 *   9600 baud = 약 960 B/s 이므로 프레임 하나(약 20~70B)에 20~70ms 가 걸린다.
 *   실측 계산(9600 8N1, 최선~최악=전 바이트 이스케이프):
 *     STATUS  N=16 -> 24~43B (25~45ms)    MOEUM   N=16 -> 24~43B (25~45ms)
 *     DONGJAK N=28 -> 36~67B (38~70ms)    SENSOR  N=12 -> 20~35B (21~37ms)
 *     JUNGJI  N=10 -> 18~31B (19~32ms)
 *   기본 설정(1000ms, STATUS + 현재 모드 시나리오) = 48~110 B/s = 회선 5~12%.
 *
 *   ⚠ 회선 상한: mon_mask 를 PROTO_MON_ALL(5종) 로 켜면 한 주기 버스트가 최악
 *     219B(약 228ms) 다. 따라서 mask=ALL 에서는 주기를 400ms 이상으로 둘 것.
 *     하한(PROTO_MON_MIN_MS=200ms)은 기본 마스크(2종) 기준이며, ALL + 200ms 는
 *     회선(960 B/s)을 넘긴다. 넘겨도 데이터가 깨지지는 않는다 - push_periodic 이
 *     TX 링 잔량을 보고 그 주기를 통째로 건너뛰고(g_proto.mon_skipped),
 *     프레임은 항상 통째로만 적재된다(UartCtrl_SendFrame). 진단은
 *     g_proto.mon_skipped / tx_fail, g_uart_ctrl.tx_drop 로 본다.
 *
 * §6. 태스크 배치 (freertos.c)
 *   Proto_Tick() - StartDefaultTask(100ms). RX 큐 소진 -> 프레임 디스패치 ->
 *                  주기/변화 M 송신. 센서 스냅샷을 읽어 패킷을 만들 뿐이고
 *                  모터를 만지지 않으므로 이 태스크가 맞다.
 *   ⚠ 앞으로 시나리오를 "제어"하는 W 명령(예: 강제 시작/정지)을 추가할 때는
 *     이 문맥에서 직접 액추에이터를 건드리지 말고, 기존 관례대로 요청 플래그만
 *     세워 MotorTask 가 소비하게 할 것(g_moeum.dbg_force_start 방식).
 * ========================================================================== */

/* ---- §1 프레임 상수 ------------------------------------------------------ */
#define PROTO_STX               0x02U
#define PROTO_ETX1              0x0DU
#define PROTO_ETX2              0x0AU
#define PROTO_DLE               0x10U
#define PROTO_ACK               0x06U
#define PROTO_NAK               0x15U

/* R/W 필드 */
typedef enum
{
	PROTO_TYPE_W = 0x00,   /* 쓰기 (App->Dev) / 쓰기 응답 ACK|NAK (Dev->App) */
	PROTO_TYPE_R = 0x01,   /* 요청 (App->Dev) / 요청 응답 DATA   (Dev->App) */
	PROTO_TYPE_M = 0x02    /* 모니터링 (Dev->App 즉시 통보)                  */
} proto_type_t;

/* ---- §3 LEN/멀티바이트 엔디안 ------------------------------------------- */
#ifndef PROTO_LEN_BIG_ENDIAN
#define PROTO_LEN_BIG_ENDIAN    0        /* 0 = 리틀엔디안 (규격 확정값) */
#endif

/* ---- 버퍼 한계 ----------------------------------------------------------- */
#ifndef PROTO_DATA_MAX
#define PROTO_DATA_MAX          64U      /* DATA[N] 최대 (현 최대 패킷 28B)   */
#endif
/* 프레임 원본(비이스케이프) = R/W(1)+CMD(1)+LEN(2)+DATA(N) */
#define PROTO_HDR_LEN           4U
#define PROTO_RAW_MAX           (PROTO_HDR_LEN + PROTO_DATA_MAX)
/* 최악의 경우 모든 바이트가 이스케이프됨: STX + 2*RAW + ETX(2) */
#define PROTO_WIRE_MAX          (1U + (2U * PROTO_RAW_MAX) + 2U)

/* ---- §4 명령어 (⚠ 임시 배정 - xlsx 표 공란) ----------------------------- */
typedef enum
{
	/* 시스템 */
	PROTO_CMD_SYS_INFO   = 0x01,  /* R : 프로토콜/FW 버전 + 현재 모드        */

	/* 모니터링(R 로 1회 조회, M 으로 주기/변화 통보 - 페이로드 동일) */
	PROTO_CMD_STATUS     = 0x20,  /* R/M : 전체 요약 (모드/상태/온도/에러)   */
	PROTO_CMD_MOEUM      = 0x21,  /* R/M : 모음 시나리오 상세                */
	PROTO_CMD_DONGJAK    = 0x22,  /* R/M : 동작 시나리오 상세                */
	PROTO_CMD_SENSOR     = 0x23,  /* R/M : 센서 전 채널 0/1 + 온도 d10       */
	PROTO_CMD_JUNGJI     = 0x24,  /* R/M : 정지 모듈 상태                    */
	PROTO_CMD_MOTOR      = 0x25,  /* R/M : 모터 전수(RPM 지령/측정/방향/구간)*/
	PROTO_CMD_OUTPUT     = 0x26,  /* R/M : 디지털 출력 전수 + 팬 duty 구간   */

	/* 설정 / 제어 */
	PROTO_CMD_MON_CFG    = 0x30,  /* R/W : 모니터링 마스크 + 주기            */
	PROTO_CMD_CONTROL    = 0x31   /* R/W : 시나리오 시작·정지 (아래 §8)      */
} proto_cmd_t;

/* NAK 사유(진단용, 회선에는 나가지 않는다 - g_proto.last_nak 로 관찰) */
typedef enum
{
	PROTO_NAK_NONE = 0,
	PROTO_NAK_UNKNOWN_CMD,     /* 미지원 CMD                                */
	PROTO_NAK_BAD_LEN,         /* LEN 이 그 CMD 의 규격과 다름               */
	PROTO_NAK_BAD_VALUE,       /* 값 범위 위반                              */
	PROTO_NAK_VERIFY_FAIL,     /* 써넣고 되읽은 값이 다름(§2 검증 실패)      */
	PROTO_NAK_NOT_WRITABLE     /* R 전용 CMD 에 W 를 시도                    */
} proto_nak_t;

/* ==========================================================================
 * §7. 모니터링 페이로드 바이트 배치
 *   모든 u16/i16 는 §3 규칙(기본 리틀엔디안). 온도는 0.1℃ 단위(d10):
 *   1140 = 114.0℃. 센서 에러 시 직전 유효값이 실린다(동작 시나리오 규칙).
 * ========================================================================== */

/* ---- 0x01 SYS_INFO : N = 8 ---------------------------------------------
 *  0    proto_rev        0x00 = R0
 *  1    fw_major
 *  2    fw_minor
 *  3    fw_patch
 *  4    app_mode         app_mode_t (0 대기/벤치, 1 모음, 2 동작, 3 강음, 4 배수)
 *  5    lid_pos          LidPos (마개 확정 위치)
 *  6-7  u16 uptime_s     부팅 후 경과 초
 */
#define PROTO_LEN_SYS_INFO      8U
#define PROTO_REV               0x00U
#ifndef PROTO_FW_MAJOR
#define PROTO_FW_MAJOR          0U
#endif
#ifndef PROTO_FW_MINOR
#define PROTO_FW_MINOR          1U
#endif
#ifndef PROTO_FW_PATCH
#define PROTO_FW_PATCH          0U
#endif

/* ---- 0x20 STATUS : N = 20 ----------------------------------------------
 *  0     app_mode        app_mode_t
 *  1     lid_pos         LidPos
 *  2     scn_state       현재 모드의 상태 enum 값
 *                        (모음 = MoeumState, 동작 = DongjakState, 그 외 0)
 *  3     flags           아래 PROTO_ST_* 비트
 *  4-5   u16 run_s       시나리오 시작(busy 0->1) 후 경과 초, 미동작 시 0
 *  6-7   i16 temp_pot    처리통 온도 d10 (THERM1/CH0)
 *  8-9   i16 temp_vapor  수증기 제어 온도 d10 (THERM3/CH2)
 * 10     bin_fill_pct    수거통 채움 0..100
 * 11     hall_mask       HS1..HS8 (bit0..bit7)
 * 12     err_code        DjErrCode (동작 전용, 그 외 0)
 * 13     jungji_src      JungjiSrc (마지막 정지 사유)
 * 14-15  u16 stop_count  누적 정지 실행 횟수
 * 16     mon_seq         ★"STATUS 를 M 으로 보낸 횟수" (0~255 순환). 유실 감지용.
 *                        앱에서 2 이상 뛰면 그 사이 STATUS 가 유실된 것이다.
 *                        버스트 단위가 아니라 STATUS 단위인 이유: 모터/출력만 바뀐
 *                        즉시송신에는 STATUS 가 실리지 않으므로, 버스트마다 올리면
 *                        번호만 건너뛰어 앱이 유실로 오탐한다. R 응답에는 현재값이
 *                        실리고 증가시키지 않는다.
 * 17     reserved        0
 * 18-19  u16 uptime_s    부팅 후 경과 초
 */
#define PROTO_LEN_STATUS        20U
#define PROTO_ST_BUSY           0x01U   /* 시나리오 진행 중                  */
#define PROTO_ST_BRAKING        0x02U   /* BLDC 단락제동 홀드 중             */
#define PROTO_ST_COOLING        0x04U   /* 잔열 냉각팬 유지 중               */
#define PROTO_ST_ERROR          0x08U   /* MOEUM_ERROR / DJ_ERROR            */
#define PROTO_ST_WATER          0x10U   /* 수위 감지됨                       */
#define PROTO_ST_HEATER         0x20U   /* HT-POWER ON                       */
#define PROTO_ST_GRIND          0x40U   /* 분쇄 M1 회전 중(측정 RPM>0)       */
#define PROTO_ST_STIR           0x80U   /* 교반 M2 회전 중(측정 RPM>0)       */

/* ---- 0x21 MOEUM : N = 16 -----------------------------------------------
 *  0     state           MoeumState (0 IDLE .. 8 ERROR)
 *  1     flags           아래 PROTO_MO_* 비트
 *  2     stir_phase      MoeumStirPhase (0 CCW, 1 DELAY, 2 CW)
 *  3     io_bits         아래 PROTO_MO_IO_* 비트
 *  4-7   u32 state_ms    현 상태 진입 후 경과 ms
 *  8-9   u16 stir_cycles 완료한 교반 사이클 수
 * 10-11  u16 stir_total  목표 사이클 수 (MOEUM_STIR_CYCLES) - 진행률 계산용
 * 12-13  u16 stir_rpm    교반 측정 출력축 RPM
 * 14-15  u16 run_s       모음 시작 후 경과 초
 */
#define PROTO_LEN_MOEUM         16U
#define PROTO_MO_BUSY           0x01U
#define PROTO_MO_START_REQ      0x02U
#define PROTO_MO_WATER          0x04U
#define PROTO_MO_ABORT_REQ      0x08U
#define PROTO_MO_LID_GUARD      0x10U
#define PROTO_MO_STIR_ACTIVE    0x20U
#define PROTO_MO_IO_VALVE_IN    0x01U   /* VALVE-DRY-IN (PB13) 급수솔        */
#define PROTO_MO_IO_WATER_ON    0x02U   /* WATER-ON (PE2) 급수 메인          */
#define PROTO_MO_IO_DOOR_EN     0x04U   /* EN-DOOR-WATER (PE4) 배수문 VM     */
#define PROTO_MO_IO_WHALL_OPEN  0x08U   /* W-HALL-OPEN 리미트 도달           */
#define PROTO_MO_IO_WHALL_CLOSE 0x10U   /* W-HALL-CLOSE 리미트 도달          */
#define PROTO_MO_IO_DRAIN_CLN   0x20U   /* VALVE-DRAIN-CLN (PB14)            */

/* ---- 0x22 DONGJAK : N = 28 ---------------------------------------------
 *  0     state           DongjakState (0 IDLE, 1..12 헹굼, 13 HEAT, ...)
 *  1     flags           아래 PROTO_DJ_* 비트
 *  2     err_code        DjErrCode
 *  3     grind_mode      DjGrindMode (0 OFF,1 COARSE,2 FINE,3 FINAL,4 COOL)
 *  4     stir_phase      DjStirPhase
 *  5     vapor_phase     DjVaporPhase
 *  6     disc_phase      DjDischPhase
 *  7     cool_phase      0 뜨거움 / 1 식음
 *  8-11  u32 run_ms      시나리오 시작 후 경과 ms (110/120/130/135분 마커 기준)
 * 12-15  u32 state_ms    현 상태 진입 후 경과 ms
 * 16-17  i16 temp_d10    처리통 온도
 * 18-19  i16 vapor_d10   수증기 제어 온도 (THERM3)
 * 20-21  u16 grind_rpm   분쇄 M1 측정 RPM (모터축)
 * 22-23  u16 stir_rpm    교반 M2 측정 RPM (출력축)
 * 24     bin_fill_pct
 * 25     io_bits         아래 PROTO_DJ_IO_* 비트
 * 26-27  u16 cycle_count 처리 완료 누적 횟수
 */
#define PROTO_LEN_DONGJAK       28U
#define PROTO_DJ_BUSY           0x01U
#define PROTO_DJ_HEAT_STARTED   0x02U
#define PROTO_DJ_TEMP_VALID     0x04U
#define PROTO_DJ_WATER          0x08U
#define PROTO_DJ_ABORT_REQ      0x10U
#define PROTO_DJ_LID_GUARD      0x20U
#define PROTO_DJ_FAN_BLDC       0x40U   /* BLDC 식힘팬 ON (30/10s duty)      */
#define PROTO_DJ_FAN_EXHAUST    0x80U   /* 배기팬 ON (15/2분 duty)           */
#define PROTO_DJ_IO_HEATER      0x01U   /* HT-POWER (PA12)                   */
#define PROTO_DJ_IO_VALVE_IN    0x02U   /* VALVE-DRY-IN (PB13)               */
#define PROTO_DJ_IO_DRAIN_CLN   0x04U   /* VALVE-DRAIN-CLN (PB14)            */
#define PROTO_DJ_IO_FAN_VAPOR   0x08U   /* FAN-VAPOR (PB15)                  */
#define PROTO_DJ_IO_WATER_ON    0x10U   /* WATER-ON (PE2)                    */
#define PROTO_DJ_IO_TDOOR_OPEN  0x20U   /* T-HALL-OPEN 리미트 도달(배출문)   */
#define PROTO_DJ_IO_TDOOR_CLOSE 0x40U   /* T-HALL-CLOSE 리미트 도달          */
#define PROTO_DJ_IO_WDOOR_CLOSE 0x80U   /* W-HALL-CLOSE 리미트 도달(배수문)  */

/* ---- 0x23 SENSOR : N = 16 ----------------------------------------------
 * "센서는 0,1 이면 된다 / 온도는 d10 을 지속 모니터링" 요구에 맞춘 전수 패킷.
 * 디지털은 전부 1비트, 아날로그는 d10(온도) 또는 mm(거리).
 *  0-1   i16 therm1_d10  처리통(CH0, PC0). 히터/분쇄/식힘 판정에 쓰는 채널
 *  2-3   i16 therm2_d10  (CH1, PC1)
 *  4-5   i16 therm3_d10  수증기(CH2, PC2 = J23)
 *  6-7   u16 distance_mm GP2Y0A41SK 거리
 *  8     bin_fill_pct    수거통 채움 0..100
 *  9     hall_mask       HS1..HS8 (bit0..bit7) - 마개위치 5 + 교반원점/수거통/리프트하단
 * 10     din1            아래 PROTO_SEN1_* (핀 전압 레벨 그대로 0/1)
 * 11     din2            아래 PROTO_SEN2_*
 * 12     limit           아래 PROTO_LIM_* (드라이버가 디코드한 리미트 도달 여부)
 * 13     therm_valid     b0 CH0, b1 CH1, b2 CH2 : 1 = 유효(써미스터 에러 아님)
 * 14-15  reserved        0
 */
#define PROTO_LEN_SENSOR        16U
/* din1 - PF0~PF7 */
#define PROTO_SEN1_BIMETAL_80   0x01U   /* PF0 80℃ 바이메탈                  */
#define PROTO_SEN1_BIMETAL_60   0x02U   /* PF1 60℃ 바이메탈                  */
#define PROTO_SEN1_THALL_CLOSE  0x04U   /* PF2 배출문 닫힘 홀                */
#define PROTO_SEN1_WHALL_CLOSE  0x08U   /* PF3 배수문 닫힘 홀                */
#define PROTO_SEN1_WHALL_OPEN   0x10U   /* PF4 배수문 열림 홀                */
#define PROTO_SEN1_THALL_OPEN   0x20U   /* PF5 배출문 열림 홀                */
#define PROTO_SEN1_WATER_SEN1   0x40U   /* PF6 수위센서1                     */
#define PROTO_SEN1_WATER_SEN2   0x80U   /* PF7 수위센서2                     */
/* din2 - PF8~PF15 + BLE */
#define PROTO_SEN2_TIMER_OUT    0x01U   /* PF8  배출 HW 2분 타이머 종료      */
#define PROTO_SEN2_HALL_INT1    0x02U   /* PF9  U24 TCA9554 INT (act.low)    */
#define PROTO_SEN2_HALL_INT2    0x04U   /* PF10 TCA9554 INT (act.low)        */
#define PROTO_SEN2_M2_FGOUT     0x08U   /* PF12 교반 M2 타코                 */
#define PROTO_SEN2_M2_NFAULT    0x10U   /* PF13 교반 M2 nFAULT (act.low)     */
#define PROTO_SEN2_M1_NFAULT    0x20U   /* PF14 분쇄 M1 nFAULT (act.low)     */
#define PROTO_SEN2_M1_FGOUT     0x40U   /* PF15 분쇄 M1 타코                 */
#define PROTO_SEN2_BLE_STATUS   0x80U   /* PD3  BLE 연결 상태 (1 = 연결)     */
/* limit - 드라이버 디코드 결과(극성 보정 포함) */
#define PROTO_LIM_WDOOR_OPEN    0x01U   /* 배수문 열림 리미트 도달           */
#define PROTO_LIM_WDOOR_CLOSE   0x02U   /* 배수문 닫힘 리미트 도달           */
#define PROTO_LIM_TDOOR_OPEN    0x04U   /* 배출문 열림 리미트 도달           */
#define PROTO_LIM_TDOOR_CLOSE   0x08U   /* 배출문 닫힘 리미트 도달           */

/* ---- 0x25 MOTOR : N = 42 -----------------------------------------------
 * "모터가 돌고 있으면 1, 멈추면 0" + "1000RPM / 2500RPM / CW·CCW / 동작 몇초,
 * 정지 몇초" 를 앱이 그대로 표시할 수 있게 만든 패킷.
 *
 * BLDC 2대는 20바이트 블록 2개로 같은 배치를 쓴다(앱은 파서를 한 벌만 만든다):
 *   분쇄 M1 (U11, 값은 모터축 RPM)  : 0 ~ 19
 *   교반 M2 (U16, 값은 출력축 RPM)  : 20 ~ 39
 * 블록 내부 오프셋 (PROTO_MOTOR_OFF_* 참고):
 *  +0     running       0 = 정지, 1 = 회전 중  ★"동작중" 표시등
 *  +1     state         bldc_state_t: 0 IDLE, 1 RUN, 2 UNJAM(잼 해소 역회전), 3 LOCKED
 *  +2     dir           0 = CW, 1 = CCW        ★방향 표시
 *  +3     fault         1 = nFAULT 래치됨
 *  +4-5   u16 target    지령 RPM (설정값. 예 1000 / 1500 / 2000)
 *  +6-7   u16 setpoint  슬루 중인 PI 설정점 (지령까지 올라가는 과정이 보인다)
 *  +8-9   u16 measured  측정 RPM (FGOUT 기반 실측)
 *  +10-11 i16 duty_pm   적용 duty [per-mille, 0~1000]
 *  +12    phase         구간 코드. pattern 에 따라 의미가 다르다:
 *                         PATTERN_TOGGLE : 0 = 구동구간, 1 = 정지구간
 *                         PATTERN_TRI    : 0 = 1차방향, 1 = 정지, 2 = 2차방향
 *                         그 외          : 0
 *  +13    phase_rep     현 구간의 반복 회차(동작 건조 교반의 "CW/정지 5회" 등). 없으면 0
 *  +14-15 u16 on_ms     ★현재 패턴의 구동 구간 길이 [ms] (0 = 연속구동)
 *  +16-17 u16 off_ms    ★현재 패턴의 정지 구간 길이 [ms] (0 = 정지구간 없음)
 *  +18-19 u16 phase_ms  현 구간 경과 [ms] (65535 클램프). 남은시간 = on/off_ms - phase_ms
 * 공통 꼬리:
 *  40     m1_pattern    아래 PROTO_PAT_* (분쇄 패턴 종류)
 *  41     m2_pattern    아래 PROTO_PAT_* (교반 패턴 종류)
 *
 * DC 모터(배수문/배출문/리프트)와 스테퍼는 피드백이 없어 0x26 OUTPUT 에 있다.
 */
#define PROTO_LEN_MOTOR         42U
#define PROTO_MOTOR_BLOCK       20U     /* BLDC 1대당 블록 크기               */
#define PROTO_MOTOR_OFF_M1      0U      /* 분쇄 M1 블록 시작                  */
#define PROTO_MOTOR_OFF_M2      20U     /* 교반 M2 블록 시작                  */
/* 블록 내부 오프셋 */
#define PROTO_MOT_RUNNING       0U
#define PROTO_MOT_STATE         1U
#define PROTO_MOT_DIR           2U
#define PROTO_MOT_FAULT         3U
#define PROTO_MOT_TARGET        4U
#define PROTO_MOT_SETPOINT      6U
#define PROTO_MOT_MEASURED      8U
#define PROTO_MOT_DUTY          10U
#define PROTO_MOT_PHASE         12U
#define PROTO_MOT_PHASE_REP     13U
#define PROTO_MOT_ON_MS         14U
#define PROTO_MOT_OFF_MS        16U
#define PROTO_MOT_PHASE_MS      18U
/* 패턴 종류 (앱이 표시 방식을 고르는 힌트) */
#define PROTO_PAT_OFF           0U      /* 미사용/정지                        */
#define PROTO_PAT_CONT          1U      /* 연속 구동(구간 토글 없음)          */
#define PROTO_PAT_TOGGLE        2U      /* 구동 on_ms / 정지 off_ms 반복      */
#define PROTO_PAT_TRI           3U      /* 1차방향 on / 정지 off / 2차방향 on */

/* ---- 0x26 OUTPUT : N = 16 ----------------------------------------------
 * 모든 디지털 출력의 현재 래치 상태(0/1)와, 팬 duty 구간 타이밍.
 * 출력 비트는 gpio_ctrl 의 출력 래치를 그대로 읽은 값이다(= 핀에 실제 나가는 값).
 *  0     dout1         아래 PROTO_OUT1_*
 *  1     dout2         아래 PROTO_OUT2_*
 *  2     dout3         아래 PROTO_OUT3_*
 *  3     dc_drive      DC 3대의 구동 상태를 2비트씩 묶음:
 *                        b1-0 배수문 WDoor, b3-2 배출문 TDoor, b5-4 리프트
 *                        값: 0 코스트, 1 정회전, 2 역회전, 3 제동 (drv8871_drive_t)
 *                        b7-6 = 0
 *  4     wdoor_duty    배수문 지령 duty [%]
 *  5     tdoor_duty    배출문 지령 duty [%]
 *  6-7   u16 fanx_el_s   배기팬 현 구간 경과 [초]   (15분/2분 duty라 초 단위)
 *  8-9   u16 fanx_on_s   배기팬 ON 구간 설정 [초]
 * 10-11  u16 fanx_off_s  배기팬 OFF 구간 설정 [초]
 * 12-13  u16 fanb_el_ms  BLDC 식힘팬 현 구간 경과 [ms]
 * 14     fanb_on_s     식힘팬 ON 구간 설정 [초] (30)
 * 15     fanb_off_s    식힘팬 OFF 구간 설정 [초] (10)
 */
#define PROTO_LEN_OUTPUT        16U
/* dout1 - 밸브/히터/팬 */
#define PROTO_OUT1_HEATER       0x01U   /* PA12 HT-POWER 히터                */
#define PROTO_OUT1_VALVE_IN     0x02U   /* PB13 VALVE-DRY-IN 급수솔          */
#define PROTO_OUT1_WATER_ON     0x04U   /* PE2  WATER-ON 급수 메인           */
#define PROTO_OUT1_DRAIN_CLN    0x08U   /* PB14 VALVE-DRAIN-CLN 배수세척솔   */
#define PROTO_OUT1_FAN_VAPOR    0x10U   /* PB15 FAN-VAPOR 방수팬             */
#define PROTO_OUT1_FAN_EXHAUST  0x20U   /* PG2  FAN-EXHAUST 배기팬           */
#define PROTO_OUT1_FAN_BLDC     0x40U   /* PG1  BLDC-FAN 식힘팬              */
#define PROTO_OUT1_SPK_EN       0x80U   /* PA3  EN-SPK 스피커                */
/* dout2 - 모터 전원/제동 */
#define PROTO_OUT2_WDOOR_VM     0x01U   /* PE4  EN-DOOR-WATER 배수문 VM      */
#define PROTO_OUT2_TDOOR_VM     0x02U   /* PE3  EN-DOOR-TRASH 배출문 VM      */
#define PROTO_OUT2_LIFT_VM      0x04U   /* PB12 MTR-DC-LIFT 리프트 VM        */
#define PROTO_OUT2_M1_ENABLE    0x08U   /* PE7  분쇄 게이트드라이버 ENABLE   */
#define PROTO_OUT2_M2_ENABLE    0x10U   /* PE8  교반 게이트드라이버 ENABLE   */
#define PROTO_OUT2_M1_NBRAKE    0x20U   /* PE13 분쇄 nBRAKE 래치(0=제동중)   */
#define PROTO_OUT2_M2_NBRAKE    0x40U   /* PE14 교반 nBRAKE 래치(0=제동중)   */
#define PROTO_OUT2_STEP1_ON     0x80U   /* STEP1 관로 4상 중 하나라도 통전   */
/* dout3 - 기타 */
#define PROTO_OUT3_STEP2_ON     0x01U   /* STEP2 흡입 4상 중 하나라도 통전   */
#define PROTO_OUT3_BLE_MODE     0x02U   /* PD4  BLE MODE (0=BYPASS)          */

/* ---- 0x24 JUNGJI : N = 10 ----------------------------------------------
 *  0     req             1 = 미처리 정지 요청 있음
 *  1     req_src         JungjiSrc
 *  2     req_kind        JungjiKind (0 NORMAL, 1 EMERGENCY)
 *  3     last_src        마지막 실행된 정지 사유
 *  4     last_kind
 *  5     flags           bit0 braking, bit1 cooling
 *  6-7   u16 stop_count
 *  8-9   i16 temp_d10    냉각 판정에 쓴 온도
 */
#define PROTO_LEN_JUNGJI        10U
#define PROTO_JG_BRAKING        0x01U
#define PROTO_JG_COOLING        0x02U

/* ---- 0x30 MON_CFG : N = 3 (R 응답 / W 요청 동일) -----------------------
 *  0     mask            PROTO_MON_* 비트 (0 = 주기 송신 전면 정지)
 *  1-2   u16 period_ms   주기 (0 = 주기 송신 정지, 그 외 최소 PROTO_MON_MIN_MS)
 * W 응답은 §2 규칙에 따라 W|CMD|LEN=1|ACK 또는 NAK.
 */
#define PROTO_LEN_MON_CFG       3U
#define PROTO_MON_STATUS        0x01U
#define PROTO_MON_MOEUM         0x02U
#define PROTO_MON_DONGJAK       0x04U
#define PROTO_MON_SENSOR        0x08U
#define PROTO_MON_JUNGJI        0x10U
#define PROTO_MON_MOTOR         0x20U
#define PROTO_MON_OUTPUT        0x40U
#define PROTO_MON_ALL           0x7FU

/* ==========================================================================
 * §8. 0x31 CONTROL — 시나리오 시작/정지 (앱 -> 장치)
 *
 * 동작 트리거는 두 가지다:
 *   (1) 장치 자체 : 마개 홀센서(HS1~5) 확정 에지 -> mode_arbiter 가 모드 전환 + 시작
 *   (2) 앱        : 이 W 명령 -> 장치가 ACK -> 시작
 * 둘 다 ModeArbiter 의 같은 pend_* 래치로 모이므로 "정지 -> 모드전환 -> (제동 해제
 * 대기) -> 시작" 순서 보장이 동일하다. 앱 경로가 지름길을 타지 않는다.
 *
 * 현재 시나리오 3종만 유효하다: 모음 / 동작 / 정지.
 *   배수(0x04)·강음(0x05)은 시나리오가 미구현 스텁이므로 코드만 배치하고 NAK 한다.
 *   앱은 버튼을 배치해도 되지만 눌리면 NAK 를 받는다(TODO 표시용).
 *
 * W 요청 : W | 0x31 | LEN=2 | [action, 0xA5]
 *   ★두 번째 바이트는 오조작 방지 매직이다. 이 명령 하나가 135분 가열 사이클을
 *     시작시키므로, 잡음 프레임이 우연히 통과하는 것을 막는다. 틀리면 NAK.
 * W 응답 : W | 0x31 | LEN=1 | ACK 또는 NAK
 *   ★ACK 의 의미 = "요청을 접수(래치)했다". 시나리오가 실제로 돌기 시작한 것은
 *     STATUS 의 app_mode / scn_state / flags.b0(동작중) 로 확인해야 한다. 값 쓰기가
 *     아니라 명령이므로 §2 의 "되읽어 검증" 은 적용되지 않는다(무엇을 되읽을지가
 *     없다). 대신 R 0x31 로 접수 결과를 조회할 수 있게 해 두었다.
 *
 * R 응답 : R | 0x31 | LEN=4 | [last_action, last_result, pending, nak_reason]
 *   last_action : 마지막으로 받은 action 코드
 *   last_result : 0 = ACK, 1 = NAK
 *   pending     : 1 = 래치가 아직 소비되지 않음(MotorTask 가 곧 적용)
 *   nak_reason  : proto_nak_t (마지막 NAK 사유)
 * ========================================================================== */
#define PROTO_LEN_CONTROL_W     2U      /* W 요청 페이로드 길이               */
#define PROTO_LEN_CONTROL       4U      /* R 응답 페이로드 길이               */
#define PROTO_CTRL_MAGIC        0xA5U   /* W 요청 2번째 바이트 (오조작 방지)  */

typedef enum
{
	PROTO_ACT_NONE      = 0x00,  /* 아무것도 하지 않음(핑)                    */
	PROTO_ACT_MOEUM     = 0x01,  /* 모음 시작                                 */
	PROTO_ACT_DONGJAK   = 0x02,  /* 동작 시작                                 */
	PROTO_ACT_STOP      = 0x03,  /* 정지(비상정지 + 대기모드 복귀)            */
	PROTO_ACT_BAESU     = 0x04,  /* 배수 - TODO 미구현 스텁 -> NAK            */
	PROTO_ACT_KANGEUM   = 0x05,  /* 강음 - TODO 미구현 스텁 -> NAK            */
	PROTO_ACT_CLEAR_ERR = 0x06,  /* 동작 DJ_ERROR -> IDLE 복구                */
	/* 동작을 헹굼 없이 DJ_HEAT 부터 시작(실장치 벤치용). 건조/분쇄/배출 본체만
	 * 반복 검증할 때 쓴다 - 헹굼은 물과 시간을 쓴다. 시나리오 경과는 0부터 다시
	 * 센다(Dongjak_DebugEnterHeat). */
	PROTO_ACT_DJ_HEAT   = 0x07,  /* 배수문 닫힘 대기부터 (사양에 가깝다)      */
	PROTO_ACT_DJ_HEAT_ND= 0x08   /* 도어 대기까지 생략, '가열중' 즉시 진입    */
} proto_action_t;

/* 1 = 배수/강음 스텁도 시작을 허용(벤치에서 스텁 진입만 보고 싶을 때).
 * 기본 0: 미구현 시나리오가 "시작됨"으로 보이면 오해를 준다. */
#ifndef PROTO_CTRL_ALLOW_STUB
#define PROTO_CTRL_ALLOW_STUB   0
#endif

/* 기본 모니터링 설정 = 공장 모니터링 화면이 바로 채워지는 조합.
 * STATUS + 시나리오 + MOTOR + OUTPUT + SENSOR (JUNGJI 는 STATUS 에 요약이 있어 제외).
 * 모음/동작을 함께 켜 두어도 "그 모드가 아닐 때는 보내지 않는다"(Proto_Tick 이
 * 모드로 게이팅)므로 둘 다 켜 둔다.
 * 1Hz 기준 최선 180 B/s / 최악 330 B/s = 9600 회선의 19~35%. */
#ifndef PROTO_MON_MASK_DEFAULT
#define PROTO_MON_MASK_DEFAULT  (PROTO_MON_STATUS | PROTO_MON_MOEUM | PROTO_MON_DONGJAK | \
                                 PROTO_MON_SENSOR | PROTO_MON_MOTOR | PROTO_MON_OUTPUT)
#endif
#ifndef PROTO_MON_PERIOD_DEFAULT_MS
#define PROTO_MON_PERIOD_DEFAULT_MS  1000U
#endif
/* 절대 하한. 이보다 짧은 주기는 마스크와 무관하게 NAK(BAD_VALUE).
 * 실제 하한은 마스크에 따라 더 커진다 - Proto_MonMinPeriodMs() 가 켜진 패킷들의
 * 최선 와이어 바이트수로 계산해 50% 여유를 붙인 값을 요구한다. */
#ifndef PROTO_MON_MIN_MS
#define PROTO_MON_MIN_MS        200U
#endif
/* 회선 여유 계수 [%]. 100 = 여유 없음(회선 100% 점유 허용), 150 = 50% 여유. */
#ifndef PROTO_MON_HEADROOM_PCT
#define PROTO_MON_HEADROOM_PCT  150U
#endif
/* UART5 보율. 주기 하한 계산에만 쓴다(실제 설정은 MX_UART5_Init). */
#ifndef PROTO_UART_BAUD
#define PROTO_UART_BAUD         9600UL
#endif
/* 상태가 바뀐 순간 주기를 기다리지 않고 즉시 M 을 밀어 올린다(앱 반응성). */
#ifndef PROTO_MON_ON_CHANGE
#define PROTO_MON_ON_CHANGE     1
#endif

/* ==========================================================================
 * 프레임 코덱 (순수 함수 - 장치 상태에 의존하지 않아 단독 시험 가능)
 * ========================================================================== */

/* 디코더 상태 */
typedef enum
{
	PROTO_RX_IDLE = 0,     /* STX 대기                                      */
	PROTO_RX_BODY,         /* 본문 수집 중                                  */
	PROTO_RX_ETX2          /* ETX1(0x0D) 받음, ETX2(0x0A) 대기              */
} proto_rx_state_t;

typedef struct
{
	uint8_t  type;                     /* proto_type_t                      */
	uint8_t  cmd;                      /* proto_cmd_t                       */
	uint16_t len;                      /* LEN (= data 유효 길이)            */
	uint8_t  data[PROTO_DATA_MAX];
} ProtoFrame;

typedef struct
{
	uint8_t  state;                    /* proto_rx_state_t                  */
	uint8_t  esc;                      /* 1 = 직전 바이트가 DLE             */
	uint16_t n;                        /* raw 수집 바이트 수                */
	uint8_t  raw[PROTO_RAW_MAX];       /* 이스케이프 해제된 R/W|CMD|LEN|DATA*/
} ProtoDecoder;

/* 프레임 조립. 반환 = out 에 쓴 총 바이트(0 = 버퍼 부족/인자 오류). */
uint16_t Proto_Encode(uint8_t type, uint8_t cmd,
                      const uint8_t *data, uint16_t len,
                      uint8_t *out, uint16_t out_sz);

void     Proto_DecoderReset(ProtoDecoder *d);
/* 바이트 1개 투입. 1 = 완성된 프레임을 f 에 채웠음, 0 = 계속 수집 중,
 * -1 = 프레임 오류(리싱크했음. g_proto.rx_err 로 집계된다). */
int8_t   Proto_DecodeByte(ProtoDecoder *d, uint8_t b, ProtoFrame *f);

/* ==========================================================================
 * 서비스 계층
 * ========================================================================== */

/* 관찰/제어 컨텍스트 (디버거 watch) */
typedef struct
{
	/* 통계 */
	volatile uint16_t rx_frames;     /* 정상 수신 프레임                    */
	volatile uint16_t rx_err;        /* 프레임 오류(리싱크)                 */
	volatile uint16_t tx_frames;     /* 송신한 프레임(R응답 + W응답 + M)    */
	volatile uint16_t tx_fail;       /* TX 링 부족으로 못 보낸 프레임        */
	volatile uint16_t mon_frames;    /* 그 중 M 프레임 수                   */
	volatile uint16_t mon_skipped;   /* 회선 밀림으로 건너뛴 주기 송신 횟수  */
	volatile uint8_t  last_type;     /* 마지막 수신 R/W                     */
	volatile uint8_t  last_cmd;      /* 마지막 수신 CMD                     */
	volatile uint8_t  last_nak;      /* proto_nak_t : 마지막 NAK 사유       */

	/* 0x31 CONTROL 접수 결과 (R 0x31 로 조회 가능) */
	volatile uint8_t  act_last;      /* 마지막 action 코드                  */
	volatile uint8_t  act_result;    /* 0 = ACK, 1 = NAK                    */
	volatile uint8_t  act_pending;   /* 1 = 래치 미소비(MotorTask 대기 중)  */
	volatile uint16_t act_count;     /* 접수(ACK)된 제어 명령 누적 횟수     */

	/* 모니터링 설정 (0x30 MON_CFG 로 앱이 변경 가능) */
	volatile uint8_t  mon_mask;      /* PROTO_MON_* 비트                    */
	volatile uint16_t mon_period_ms;
	uint32_t          mon_last;      /* 마지막 주기 송신 tick               */

	volatile uint8_t  mon_seq;       /* M 버스트마다 +1 (STATUS 바이트16)     */

	/* 변화 감지 스냅샷(PROTO_MON_ON_CHANGE) */
	uint8_t           prev_mode;
	uint8_t           prev_state;
	uint8_t           prev_err;
	uint8_t           prev_busy;
	uint32_t          run_start;     /* busy 0->1 시각 = 시나리오 경과 기준 */
	/* 0x25 MOTOR 의 "설정/상태" 바이트 사본(측정값 제외 - 아래 주석 참고).
	 * M1/M2 각 6바이트(running,state,dir,fault,target lo/hi) + pattern 2바이트 */
	uint8_t           prev_mot[14];
	/* 0x26 OUTPUT 의 앞 6바이트 사본(dout1~3, dc_drive, 도어 duty 2) */
	uint8_t           prev_out[6];
	uint8_t           prev_valid;    /* 1 = 위 두 사본이 채워졌음            */

	/* 벤치용: 1 을 쓰면 그 즉시 해당 M 패킷을 1회 송신(1회성) */
	volatile uint8_t  dbg_push_status;
	volatile uint8_t  dbg_push_moeum;
	volatile uint8_t  dbg_push_dongjak;
	volatile uint8_t  dbg_push_sensor;
} ProtoCtx;

extern ProtoCtx g_proto;

void    Proto_Init(void);
/* 100ms, StartDefaultTask. RX 소진 -> 디스패치 -> 주기/변화 M 송신. */
void    Proto_Tick(uint32_t now_ms);

/* 임의 프레임 송신(TX 링에 통째로 적재). 1 = 성공, 0 = 링 부족. */
uint8_t Proto_Send(uint8_t type, uint8_t cmd, const uint8_t *data, uint16_t len);
/* §2 W 응답: W | CMD | LEN=1 | ACK(ok!=0) | NAK(ok==0) */
uint8_t Proto_SendAck(uint8_t cmd, uint8_t ok);
/* CMD 의 모니터링 페이로드를 만들어 지정 타입으로 송신(R 응답/M 통보 공용). */
uint8_t Proto_SendPayload(uint8_t type, uint8_t cmd);

/* 페이로드 빌더(단독 시험/재사용용). 반환 = 채운 바이트 수, 0 = 미지원 CMD.
 * buf 는 최소 PROTO_DATA_MAX 바이트여야 한다. */
uint16_t Proto_BuildPayload(uint8_t cmd, uint8_t *buf, uint16_t buf_sz);

/* 이 마스크를 이 보율로 보낼 때 요구되는 최소 주기 [ms].
 * 켜진 패킷들의 최선 와이어 바이트 합 x 10bit/byte / baud x 여유계수.
 * MON_CFG(W) 가 이 값으로 주기를 검증하고, 미달이면 NAK 한다. 앱도 같은 식으로
 * 계산해 슬라이더 하한을 잡을 수 있다. */
uint16_t Proto_MonMinPeriodMs(uint8_t mask);

#ifdef __cplusplus
}
#endif

#endif /* DEVICES_COMM_PROTOCOL_R0_H_ */
