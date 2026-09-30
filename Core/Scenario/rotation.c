/* ============================================================================
 * rotation.c — 제로지오 0.3.0 (2026.09.21 회전 운전 수정) 원본 이식 [R3 개정4, 검토서 §17.3]
 * 출처: doc/R3/개발사 전달(2026.09.21_회전 운전 수정)/04. C언어 프로그램(개발사 수정·통합용)/src/rotation.c
 * 원본 sha256 81a57eb1a5b839c3c39ef13c8d8a496e15d5c6d2b5f441151b2b5fa3e7682ff0
 * ★이 블록 아래 본문은 원본과 **바이트 단위로 같다 — 수정 금지** (CLAUDE.md §1-1).
 *   HW 무관 순수 상태기계. 입력(position/at_rest/permitted)은 bldc_ctrl·호출부가 만든다.
 *   벤더가 새 판을 보내면 이 블록 아래를 통째로 바꾸고 sha256 을 갱신한다.
 * ========================================================================== */
#include "rotation.h"

void zg_rotation_reset(zg_rotation *r) { *r = (zg_rotation){0}; }

static uint8_t direction(const zg_rotation *r, const zg_rotation_config *p) {
    if (r->profile == ZG_ROT_PROCESS) return r->phase == 1U ? 2U : 1U;
    if (r->profile == ZG_ROT_WASH && r->cycle >= p->wash_initial_cycles &&
        (r->cycle - p->wash_initial_cycles) % 2U != 0U) return 2U;
    return 1U;
}
static void advance(zg_rotation *r, const zg_rotation_config *p) {
    ++r->cycle;
    if (r->profile != ZG_ROT_PROCESS) return;
    if (r->phase == 0U && r->cycle >= p->initial_cycles) {
        r->initial_done = true; r->phase = 1U; r->cycle = 0U;
    } else if (r->phase == 1U && r->cycle >= p->reverse_cycles) {
        r->phase = 2U; r->cycle = 0U;
    } else if (r->phase == 2U && r->cycle >= p->forward_cycles) {
        r->phase = 1U; r->cycle = 0U;
    }
}
static void hold(zg_rotation *r, uint32_t now) {
    if (!r->waiting) { r->waiting = true; r->stop_since = now; }
}
uint8_t zg_rotation_tick(zg_rotation *r, zg_rotation_profile profile,
    const zg_rotation_config *p, const zg_rotation_input *in, uint32_t now) {
    bool paired = profile == ZG_ROT_PROCESS;
    bool rest = in->stir_at_rest && (!paired || in->grind_at_rest);
    if (r->failed) return 0U;
    if (!in->valid || p->ticks_per_revolution == 0U || p->revolutions != 2U ||
        p->ticks_per_revolution > 0x3FFFFFFFU || p->stop_ms == 0U ||
        p->feedback_timeout_ms <= p->stop_ms || p->feedback_timeout_ms >= 0x80000000U) {
        r->failed = true; return 0U;
    }
    if (!r->initialized || r->profile != profile) {
        bool changed = r->initialized;
        zg_rotation_reset(r); r->initialized = true; r->profile = profile;
        r->previous_position = in->position; r->progress_since = now;
        /* A wash-to-drain change must not reverse a moving motor. */
        if (changed || !rest) hold(r, now);
    }
    if (r->active_leg) {
        uint32_t travel = r->direction == 1U ? in->position - r->origin : r->origin - in->position;
        if (travel >= 0x80000000U) { r->failed = true; return 0U; }
        if (in->position != r->previous_position) r->progress_since = now;
        if (travel >= p->ticks_per_revolution * p->revolutions) {
            r->active_leg = false; r->complete_leg = true; hold(r, now);
        }
    }
    r->previous_position = in->position;
    if (paired && r->initial_done && in->late_requested && !r->late) {
        r->late = true; hold(r, now);
    }
    if (!in->permitted) { hold(r, now); return 0U; }
    if (r->waiting) {
        if (!rest && now - r->stop_since >= p->feedback_timeout_ms) {
            r->failed = true; return 0U;
        }
        if (!rest || now - r->stop_since < p->stop_ms) return 0U;
        r->waiting = false; r->progress_since = now;
        if (r->complete_leg) { advance(r, p); r->complete_leg = false; }
        /* Apply the late relation before starting the first post-initial leg. */
        if (paired && r->initial_done && in->late_requested) r->late = true;
    }
    if (!paired && r->cycle >= 10U) return 0U;
    if (!r->active_leg) {
        if (!rest) { hold(r, now); return 0U; }
        r->direction = direction(r, p); r->origin = in->position;
        r->active_leg = true; r->progress_since = now;
    }
    if (now - r->progress_since >= p->feedback_timeout_ms) {
        r->failed = true; return 0U;
    }
    return r->direction;
}
