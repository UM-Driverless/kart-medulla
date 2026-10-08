#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Unsigned elapsed ticks handle wrap. Delayed sends skip missed slots rather
 * than burst to catch up. Call with the same tick units for now and period. */
static inline bool km_telemetry_due(uint32_t now, uint32_t *last, uint32_t period)
{
    if ((uint32_t)(now - *last) < period) return false;
    *last = now;
    return true;
}
