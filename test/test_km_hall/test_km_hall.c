#include <unity.h>
#include "km_hall_state.h"

void setUp(void) {}
void tearDown(void) {}

static void test_forward_and_reverse_counts(void)
{
    km_hall_state_t s = {.bits = 1, .interval_us = -1};
    /* Example six-state motor sequence, not a claim about the fitted motor. */
    const uint8_t forward[] = {3, 2, 6, 4, 5, 1};
    const uint8_t reverse[] = {5, 4, 6, 2, 3, 1};
    for (unsigned i = 0; i < 6; ++i) km_hall_observe(&s, forward[i], 100 + i * 100);
    for (unsigned i = 0; i < 3; ++i) TEST_ASSERT_EQUAL_UINT32(2, s.edges[i]);
    for (unsigned i = 0; i < 6; ++i) km_hall_observe(&s, reverse[i], 700 + i * 100);
    for (unsigned i = 0; i < 3; ++i) TEST_ASSERT_EQUAL_UINT32(4, s.edges[i]);
    TEST_ASSERT_EQUAL_UINT32(0, s.multi_changes);
    TEST_ASSERT_EQUAL_INT64(100, s.interval_us);
}

static void test_idle_duplicate_and_first_edge(void)
{
    km_hall_state_t s = {.bits = 0, .interval_us = -1};
    km_hall_observe(&s, 0, 100);
    TEST_ASSERT_FALSE(s.seen_edge);
    km_hall_observe(&s, 1, 200);
    TEST_ASSERT_TRUE(s.seen_edge);
    TEST_ASSERT_EQUAL_INT64(-1, s.interval_us);
    km_hall_observe(&s, 1, 300);
    TEST_ASSERT_EQUAL_INT64(200, s.last_edge_us);
    TEST_ASSERT_EQUAL_UINT32(1, s.edges[0]);
    km_hall_observe(&s, 0, 5000000000LL);
    TEST_ASSERT_EQUAL_INT64(4999999800LL, s.interval_us);
}

static void test_ambiguous_change_and_counter_wrap(void)
{
    km_hall_state_t s = {.bits = 0, .edges = {UINT32_MAX, 0, 0}, .interval_us = -1};
    km_hall_observe(&s, 7, 100);
    TEST_ASSERT_EQUAL_UINT32(0, s.edges[0]);
    TEST_ASSERT_EQUAL_UINT32(1, s.edges[1]);
    TEST_ASSERT_EQUAL_UINT32(1, s.edges[2]);
    TEST_ASSERT_EQUAL_UINT32(1, s.multi_changes);
    TEST_ASSERT_EQUAL_UINT8(7, s.bits);
    km_hall_observe(&s, 7, 200);
    TEST_ASSERT_EQUAL_UINT32(1, s.multi_changes);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_forward_and_reverse_counts);
    RUN_TEST(test_idle_duplicate_and_first_edge);
    RUN_TEST(test_ambiguous_change_and_counter_wrap);
    return UNITY_END();
}
