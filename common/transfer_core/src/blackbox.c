#include "blackbox.h"

#include <string.h>

void bb_init(blackbox_t *bb)
{
    memset(bb, 0, sizeof(*bb));
}

bool bb_add(blackbox_t *bb, const bb_sample_t *s)
{
    if (bb->frozen) {
        return false;
    }
    bb->ring[bb->head] = *s;
    bb->head = (bb->head + 1u) % BB_LEN;
    if (bb->written < BB_LEN) {
        bb->written++;
    }
    if (bb->triggered) {
        if (bb->post_left > 0u) {
            bb->post_left--;
        }
        if (bb->post_left == 0u) {
            bb->frozen = true;
            return true;
        }
    }
    return false;
}

void bb_trigger(blackbox_t *bb, uint8_t cause, uint32_t now_ms)
{
    if (bb->triggered || bb->frozen) {
        return;
    }
    bb->triggered = true;
    bb->post_left = BB_POST;
    bb->cause = cause;
    bb->trigger_ms = now_ms;
}

uint32_t bb_count(const blackbox_t *bb)
{
    return bb->written;
}

const bb_sample_t *bb_at(const blackbox_t *bb, uint32_t i)
{
    /* oldest sample is at head when the ring is full, at 0 otherwise */
    const uint32_t oldest = (bb->written < BB_LEN) ? 0u : bb->head;
    return &bb->ring[(oldest + i) % BB_LEN];
}

void bb_rearm(blackbox_t *bb)
{
    bb->triggered = false;
    bb->frozen = false;
    bb->post_left = 0;
}
