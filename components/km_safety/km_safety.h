#ifndef KM_SAFETY_H
#define KM_SAFETY_H

#include <stdbool.h>
#include <stdint.h>

enum {
    KM_SAFETY_STEERING = 1u << 0,
    KM_SAFETY_TANK_INVALID = 1u << 1,
    KM_SAFETY_TANK_LOW = 1u << 2,
    KM_SAFETY_COMMAND_STALE = 1u << 3,
    KM_SAFETY_COMPRESSOR_DISABLED = 1u << 4,
    KM_SAFETY_INVALID_MODE = 1u << 5,
    KM_SAFETY_STATE_STALE = 1u << 6,
    KM_SAFETY_ORIN_EMERGENCY = 1u << 7,
    KM_SAFETY_HARDWARE_IO = 1u << 8
};

enum {
    KM_SAFETY_READY = 1u << 0,
    KM_SAFETY_EMERGENCY = 1u << 1,
    KM_SAFETY_BENCH = 1u << 2,
    KM_SAFETY_ARMED = 1u << 3
};

/* Drivers establish validity and freshness. This module owns permission to
 * actuate; it performs no hardware access, logging, or communications. */
typedef struct {
    bool hardware_ok;
    bool steering_valid;
    bool tank_valid;
    bool tank_pressure_ok;
    bool commands_fresh;
    bool state_fresh;
    bool compressor_disabled;
    int mission;
    int as_state;
    int steering_mode;
    bool targets_zero;
    int32_t reset_token;
} km_safety_inputs;

typedef struct {
    uint32_t active_faults;
    uint32_t latched_faults;
    uint32_t flags;
    int32_t reset_ack_token;
    int32_t last_reset_token;
    bool suppress_orin_emergency;
    bool was_armed;
    bool reset_hold;
    bool allow_manual_pedal;
    bool allow_throttle;
    bool allow_steering;
    bool close_shutdown;
} km_safety_state;

void KM_SAFETY_Update(km_safety_state *state, const km_safety_inputs *inputs);

#endif
