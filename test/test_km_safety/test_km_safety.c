#include <unity.h>
#include "km_safety.h"
#include "km_safety.c"

static km_safety_state state;
static km_safety_inputs in;
void setUp(void) {
    state = (km_safety_state){0};
    in = (km_safety_inputs){.hardware_ok=true, .steering_valid=true, .tank_valid=true,
        .tank_pressure_ok=true, .commands_fresh=true, .state_fresh=true,
        .mission=8, .as_state=0, .steering_mode=1, .targets_zero=true};
}
void tearDown(void) {}
static void update(void) { KM_SAFETY_Update(&state, &in); }
static void drive(void) { in.as_state=2; in.steering_mode=0; update(); }

void test_startup_missing_sensor_inhibits_without_latch(void) {
    in.steering_valid=false; in.as_state=1; update();
    TEST_ASSERT_EQUAL(KM_SAFETY_STEERING,state.active_faults);
    TEST_ASSERT_EQUAL(0,state.latched_faults);
    TEST_ASSERT_FALSE(state.close_shutdown);
    TEST_ASSERT_FALSE(state.allow_throttle);
    in.steering_valid=true; update();
    TEST_ASSERT_TRUE(state.close_shutdown);
    TEST_ASSERT_FALSE(state.allow_steering);
}
void test_loss_while_ready_latches_and_return_does_not_restart(void) {
    in.as_state=1; update();
    in.steering_valid=false; update();
    TEST_ASSERT_EQUAL(KM_SAFETY_STEERING,state.latched_faults);
    in.steering_valid=true; drive();
    TEST_ASSERT_FALSE(state.allow_throttle);
    TEST_ASSERT_FALSE(state.allow_steering);
    TEST_ASSERT_FALSE(state.close_shutdown);
}
void test_every_drive_fault_stops_and_latches(void) {
    for (int fault=0;fault<7;fault++) {
        setUp(); drive(); TEST_ASSERT_TRUE(state.allow_throttle);
        switch (fault) {
        case 0: in.steering_valid=false; break;
        case 1: in.tank_valid=false; break;
        case 2: in.tank_pressure_ok=false; break;
        case 3: in.commands_fresh=false; break;
        case 4: in.compressor_disabled=true; break;
        case 5: in.mission=99; break;
        case 6: in.state_fresh=false; break;
        }
        update();
        TEST_ASSERT_EQUAL_UINT32(1u<<fault,state.latched_faults);
        TEST_ASSERT_FALSE(state.allow_throttle);
        TEST_ASSERT_FALSE(state.allow_steering);
        TEST_ASSERT_FALSE(state.close_shutdown);
    }
}
void test_reset_must_be_healthy_disarmed_and_deliberate(void) {
    drive(); in.tank_valid=false; update();
    in.as_state=4; in.reset_token=1; update();
    TEST_ASSERT_EQUAL(0,state.reset_ack_token);
    in.tank_valid=true; update(); // rejected attempt must not execute later
    TEST_ASSERT_NOT_EQUAL(0,state.latched_faults);
    in.reset_token=2; in.targets_zero=false; update();
    TEST_ASSERT_EQUAL(0,state.reset_ack_token);
    in.reset_token=3; in.targets_zero=true; in.as_state=2; update();
    TEST_ASSERT_EQUAL(0,state.reset_ack_token);
    in.reset_token=4; in.as_state=4; update();
    TEST_ASSERT_EQUAL(4,state.reset_ack_token);
    TEST_ASSERT_EQUAL(0,state.latched_faults);
    TEST_ASSERT_TRUE(state.flags & KM_SAFETY_READY);
    TEST_ASSERT_FALSE(state.allow_throttle);
    update(); TEST_ASSERT_EQUAL(0,state.latched_faults);
    in.as_state=0; update(); drive();
    TEST_ASSERT_TRUE(state.allow_throttle);
    in.as_state=4; in.reset_token=1; update(); // replay cannot clear new emergency
    TEST_ASSERT_NOT_EQUAL(0,state.latched_faults);
}
void test_bench_has_no_propulsion_and_cannot_clear_latch(void) {
    in.mission=7; in.steering_valid=false; in.tank_valid=false;
    in.tank_pressure_ok=false; update();
    TEST_ASSERT_TRUE(state.flags & KM_SAFETY_BENCH);
    TEST_ASSERT_TRUE(state.allow_steering);
    TEST_ASSERT_FALSE(state.allow_throttle);
    TEST_ASSERT_FALSE(state.close_shutdown);
    in.as_state=4; update();
    in.as_state=0; update();
    TEST_ASSERT_FALSE(state.allow_steering);
    in.reset_token=1; update();
    TEST_ASSERT_EQUAL(0,state.reset_ack_token);
}
void test_manual_never_drives_electronic_outputs(void) {
    in.mission=0; in.as_state=2; in.steering_mode=0; update();
    TEST_ASSERT_FALSE(state.allow_steering);
    TEST_ASSERT_FALSE(state.allow_throttle);
    TEST_ASSERT_FALSE(state.close_shutdown);
}
void test_start_serial_order_and_remote_pid(void) {
    in.as_state=1; update();
    TEST_ASSERT_TRUE(state.flags & KM_SAFETY_READY);
    in.as_state=2; update();
    TEST_ASSERT_TRUE(state.allow_throttle); // autonomous steering=None permits drive
    TEST_ASSERT_FALSE(state.allow_steering);
    TEST_ASSERT_EQUAL(0,state.latched_faults);
    in.steering_mode=0; update();
    TEST_ASSERT_TRUE(state.allow_throttle);
    setUp(); in.mission=7; in.steering_mode=0; update();
    TEST_ASSERT_TRUE(state.allow_throttle);
    in.as_state=3; update(); // finished is not emergency
    TEST_ASSERT_FALSE(state.allow_throttle);
    TEST_ASSERT_EQUAL(0,state.latched_faults);
}
void test_remote_reset_does_not_resume_propulsion(void) {
    in.mission=7; in.steering_mode=0; update();
    TEST_ASSERT_TRUE(state.allow_throttle);
    in.as_state=4; update();
    in.reset_token=1; update();
    TEST_ASSERT_EQUAL(1,state.reset_ack_token);
    in.as_state=0; update(); update();
    TEST_ASSERT_FALSE(state.allow_throttle);
    in.steering_mode=1; update();
    TEST_ASSERT_FALSE(state.allow_throttle);
    in.steering_mode=0; update();
    TEST_ASSERT_TRUE(state.allow_throttle);
}
void test_hardware_failure_blocks_drive_and_bench(void) {
    drive(); in.hardware_ok=false; update();
    TEST_ASSERT_EQUAL(KM_SAFETY_HARDWARE_IO,state.latched_faults);
    TEST_ASSERT_FALSE(state.allow_throttle);
    setUp(); in.mission=7; in.hardware_ok=false; update();
    TEST_ASSERT_FALSE(state.allow_steering);
}
void test_bench_comms_loss_latches_until_reset(void) {
    in.mission=7; update();
    TEST_ASSERT_TRUE(state.allow_steering);
    in.commands_fresh=false; update();
    TEST_ASSERT_EQUAL(KM_SAFETY_COMMAND_STALE,state.latched_faults);
    in.commands_fresh=true; update();
    TEST_ASSERT_FALSE(state.allow_steering);
}
void test_invalid_modes_fail_closed(void) {
    const int modes[]={-1,2,255};
    for (unsigned n=0;n<sizeof(modes)/sizeof(modes[0]);n++) {
        setUp(); in.steering_mode=modes[n]; update();
        TEST_ASSERT_TRUE(state.active_faults & KM_SAFETY_INVALID_MODE);
        TEST_ASSERT_FALSE(state.allow_steering);
    }
}
int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_startup_missing_sensor_inhibits_without_latch);
    RUN_TEST(test_loss_while_ready_latches_and_return_does_not_restart);
    RUN_TEST(test_every_drive_fault_stops_and_latches);
    RUN_TEST(test_reset_must_be_healthy_disarmed_and_deliberate);
    RUN_TEST(test_bench_has_no_propulsion_and_cannot_clear_latch);
    RUN_TEST(test_manual_never_drives_electronic_outputs);
    RUN_TEST(test_start_serial_order_and_remote_pid);
    RUN_TEST(test_invalid_modes_fail_closed);
    RUN_TEST(test_bench_comms_loss_latches_until_reset);
    RUN_TEST(test_hardware_failure_blocks_drive_and_bench);
    RUN_TEST(test_remote_reset_does_not_resume_propulsion);
    return UNITY_END();
}
