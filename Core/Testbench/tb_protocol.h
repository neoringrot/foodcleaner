#ifndef TESTBENCH_TB_PROTOCOL_H_
#define TESTBENCH_TB_PROTOCOL_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * tb_protocol - R0 프로토콜 자체검사 테스트벤치 (enable 게이트)
 *
 * 왜 필요한가:
 *   protocol_r0.c 의 프레임 코덱(§A)은 호스트에서 벡터 검증을 했지만, 그것은
 *   알고리즘 검증이고 "이 툴체인으로 컴파일된 이 바이너리"의 검증은 아니다.
 *   ARM 크로스 컴파일에서 실제로 틀어질 수 있는 것은 알고리즘이 아니라
 *   타입/정렬/부호 확장 쪽이다:
 *     - int 승격과 uint8_t 자르기 (put_esc / 마스크 연산)
 *     - i16 음수(온도 d10) 의 부호 보존
 *     - 리틀엔디안 직렬화가 실제로 LE 로 나오는지
 *     - 구조체 volatile 멤버 읽기
 *   이 벤치는 그 검사를 타깃에서 직접 돌리고 결과를 디버거 watch 변수로 남긴다.
 *
 * 사용법 (다른 tb_* 와 동일 관례):
 *   1) 디버거에서 tb_proto_enable = 1  (또는 tb_proto_run_once = 1)
 *   2) 한 사이클(defaultTask 100ms) 뒤 결과 확인:
 *        tb_proto_checks  - 수행한 검사 수
 *        tb_proto_fails   - 실패 수 (0 이어야 정상)
 *        tb_proto_first_fail - 처음 실패한 검사 번호(0 = 없음)
 *        tb_proto_done    - 1 = 한 회 완주
 *   3) tb_proto_enable = 0 으로 정지 (run_once 는 자동으로 0 이 된다)
 *
 * 통신을 하지 않는다. UART5 로 아무것도 내보내지 않고 순수하게 메모리 안에서
 * 인코딩->디코딩 왕복만 한다. 따라서 앱이 붙어 있어도, 시나리오가 돌고 있어도
 * 안전하다(회선/모터를 건드리지 않는다).
 *
 * 비용: 한 회 약 1.5ms (라운드트립 65회 + 케이스 10여개). 100ms 태스크에서
 * 한 회만 돌리므로 무해하지만, enable=1 로 계속 켜 두면 매 100ms 마다 돌아
 * defaultTask 여유를 조금 먹는다. 확인 후 끄는 것을 권한다.
 *
 * 태스크 배치: TB_Protocol_Poll() - StartDefaultTask(100ms).
 *   프로토콜 서비스(Proto_Tick)와 같은 태스크에 두어야 한다. s_dec/s_frame 같은
 *   프로토콜 정적 버퍼를 공유하지 않도록 이 벤치는 자기 버퍼만 쓴다.
 * ========================================================================== */

/* 1 = 매 100ms 마다 반복 검사. 0 = 정지. */
extern volatile uint8_t  tb_proto_enable;
/* 1 을 쓰면 1회만 검사하고 자동으로 0 으로 돌아간다(권장). */
extern volatile uint8_t  tb_proto_run_once;

/* ---- 결과 (RO 관찰) ---------------------------------------------------- */
extern volatile uint16_t tb_proto_checks;      /* 수행한 검사 수             */
extern volatile uint16_t tb_proto_fails;       /* 실패 수 (0 이어야 정상)    */
extern volatile uint16_t tb_proto_first_fail;  /* 첫 실패 검사 번호(0=없음)  */
extern volatile uint16_t tb_proto_runs;        /* 완주 횟수                  */
extern volatile uint8_t  tb_proto_done;        /* 1 = 최소 1회 완주          */
extern volatile uint32_t tb_proto_us;          /* 마지막 1회 소요 시간(us 근사)*/

/* 실패 시 진단용: 마지막으로 실패한 검사의 기대/실제 값 */
extern volatile int32_t  tb_proto_want;
extern volatile int32_t  tb_proto_got;

/* ---- API --------------------------------------------------------------- */
void TB_Protocol_Init(void);
void TB_Protocol_Poll(void);   /* 100ms, StartDefaultTask (enable 게이트) */

/* 게이트 없이 즉시 1회 실행. 반환 = 실패 수(0 = 전부 통과).
 * 부팅 자체검사로 쓰려면 main.c 에서 직접 부를 수도 있다. */
uint16_t TB_Protocol_RunOnce(void);

#ifdef __cplusplus
}
#endif

#endif /* TESTBENCH_TB_PROTOCOL_H_ */
