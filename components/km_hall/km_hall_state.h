#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Raw observations, not a calibrated motor-position or sensor-health estimate.
 * All eight states are accepted until the motor's Hall arrangement is measured.
 * Counts wrap modulo 2^32. Multiple changed bits indicate ambiguous capture. */
typedef struct {
    uint32_t edges[3];
    uint32_t multi_changes;
    uint8_t bits;                 /* bit 0 = Hall 1, bit 1 = Hall 2, bit 2 = Hall 3 */
    bool seen_edge;
    int64_t last_edge_us;
    int64_t interval_us;           /* -1 until two distinct observations */
} km_hall_state_t;

static inline __attribute__((always_inline)) void
km_hall_observe(km_hall_state_t *s, uint8_t bits, int64_t now_us)
{
    uint8_t changed = s->bits ^ bits;
    if (!changed) return;          /* several pending GPIO callbacks can see the same state */
    for (unsigned i = 0; i < 3; ++i)
        if (changed & (1u << i)) ++s->edges[i];
    if (changed & (changed - 1u)) ++s->multi_changes;
    s->interval_us = s->seen_edge ? now_us - s->last_edge_us : -1;
    s->last_edge_us = now_us;
    s->seen_edge = true;
    s->bits = bits;
}
