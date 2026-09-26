#include "km_safety.h"

void KM_SAFETY_Update(km_safety_state *s, const km_safety_inputs *i)
{
    // Orin states: OFF=0, READY=1, DRIVING=2, FINISHED=3, EMERGENCY=4.
    const bool manual = i->mission == 0;
    const bool remote = i->mission == 7;
    const bool emergency = i->as_state == 4;
    const bool bench = remote && i->steering_mode == 1 && i->as_state == 0;
    const bool valid_mode = i->mission >= 0 && i->mission <= 8
        && i->as_state >= 0 && i->as_state <= 4
        && (i->steering_mode == 0 || i->steering_mode == 1)
        && !(manual && i->as_state == 2);

    uint32_t drive_faults = 0;
    if (!i->hardware_ok) drive_faults |= KM_SAFETY_HARDWARE_IO;
    if (!i->steering_valid) drive_faults |= KM_SAFETY_STEERING;
    if (!i->tank_valid) drive_faults |= KM_SAFETY_TANK_INVALID;
    if (!i->tank_pressure_ok) drive_faults |= KM_SAFETY_TANK_LOW;
    if (!i->commands_fresh) drive_faults |= KM_SAFETY_COMMAND_STALE;
    if (!i->state_fresh) drive_faults |= KM_SAFETY_STATE_STALE;
    if (i->compressor_disabled) drive_faults |= KM_SAFETY_COMPRESSOR_DISABLED;
    if (!valid_mode) drive_faults |= KM_SAFETY_INVALID_MODE;
    s->active_faults = drive_faults;

    /* Bench steering cannot propel the kart or close the shutdown chain. Its
     * missing sensors remain visible as active faults, but do not trip a drive
     * session which was never armed. Switching modes cannot clear a latch. */
    const bool arm_requested = !manual && !bench && !emergency
        && ((!remote && (i->as_state == 1 || i->as_state == 2))
            || (remote && i->as_state == 0 && i->steering_mode == 0));
    if (s->was_armed && drive_faults) s->latched_faults |= drive_faults;
    if ((s->flags & KM_SAFETY_BENCH) && s->allow_steering)
        s->latched_faults |= drive_faults & (KM_SAFETY_COMMAND_STALE
            | KM_SAFETY_STATE_STALE | KM_SAFETY_INVALID_MODE | KM_SAFETY_HARDWARE_IO);
    if (!emergency) s->suppress_orin_emergency = false;
    if (emergency && !s->suppress_orin_emergency)
        s->latched_faults |= KM_SAFETY_ORIN_EMERGENCY;

    /* Each token is an attempt, not a pending request: recovery must never
     * silently execute a reset that was rejected while a fault was present. */
    if (s->reset_hold && i->as_state == 0 && (!remote || bench))
        s->reset_hold = false;
    if (i->reset_token > 0 && i->reset_token > s->last_reset_token) {
        s->last_reset_token = i->reset_token;
        if ((i->as_state == 0 || emergency) && !drive_faults && i->targets_zero) {
            s->latched_faults = 0;
            s->reset_hold = true;
            s->reset_ack_token = i->reset_token;
            s->suppress_orin_emergency = emergency;
        }
    }

    const bool healthy = !drive_faults && !s->latched_faults;
    const bool armed = arm_requested && healthy && !s->reset_hold;
    s->close_shutdown = armed;
    s->allow_manual_pedal = manual && i->as_state == 0 && !s->latched_faults;
    s->allow_throttle = armed && (i->as_state == 2 || remote);
    s->allow_steering = (s->allow_throttle && i->steering_mode == 0)
        || (bench && valid_mode && i->hardware_ok && i->commands_fresh && i->state_fresh
            && !s->latched_faults && !s->reset_hold);
    s->was_armed = armed;
    s->flags = (healthy ? KM_SAFETY_READY : 0)
        | (s->latched_faults ? KM_SAFETY_EMERGENCY : 0)
        | (bench ? KM_SAFETY_BENCH : 0)
        | (armed ? KM_SAFETY_ARMED : 0);
}
