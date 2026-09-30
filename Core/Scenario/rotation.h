/* ============================================================================
 * rotation.h — 제로지오 0.3.0 (2026.09.21 회전 운전 수정) 원본 이식 [R3 개정4, 검토서 §17.3]
 * 출처: doc/R3/개발사 전달(2026.09.21_회전 운전 수정)/04. C언어 프로그램(개발사 수정·통합용)/include/rotation.h
 * 원본 sha256 3bc35145f7e94d2525c0ed2ed51b91b70b76f93c6c93b85c1db26df498bdc267
 * ★이 블록 아래 본문은 원본과 **바이트 단위로 같다 — 수정 금지** (CLAUDE.md §1-1).
 *   HW 무관 순수 상태기계. 입력(position/at_rest/permitted)은 bldc_ctrl·호출부가 만든다.
 *   벤더가 새 판을 보내면 이 블록 아래를 통째로 바꾸고 sha256 을 갱신한다.
 * ========================================================================== */
#ifndef ZG_ROTATION_H
#define ZG_ROTATION_H
#include <stdbool.h>
#include <stdint.h>

typedef enum { ZG_ROT_NONE, ZG_ROT_WASH, ZG_ROT_DRAIN, ZG_ROT_PROCESS } zg_rotation_profile;
typedef struct {
    uint32_t ticks_per_revolution, revolutions, stop_ms, feedback_timeout_ms;
    uint32_t initial_cycles, forward_cycles, reverse_cycles, wash_initial_cycles;
} zg_rotation_config;
typedef struct {
    zg_rotation_profile profile;
    uint32_t cycle, phase, origin, previous_position, progress_since, stop_since;
    bool active_leg, waiting, complete_leg, initial_done, late, failed, initialized;
    uint8_t direction;
} zg_rotation;
typedef struct {
    uint32_t position;
    bool valid, stir_at_rest, grind_at_rest, permitted, late_requested;
} zg_rotation_input;

void zg_rotation_reset(zg_rotation *r);
/* Return 0=stop, 1=clockwise, 2=counterclockwise, viewed from above. */
uint8_t zg_rotation_tick(zg_rotation *r, zg_rotation_profile profile,
    const zg_rotation_config *cfg, const zg_rotation_input *in, uint32_t now);
#endif
