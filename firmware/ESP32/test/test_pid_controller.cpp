/**
 * @file test_pid_controller.cpp
 * @brief Unit tests for the generic PID controller (host-side, native env).
 *
 * Compile & run:
 *     pio test -e native
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */

#include <unity.h>

#include "control/pid_controller.hpp"

using thunders::control::PidController;

void setUp()    {}
void tearDown()  {}

// ─── Test: a P-only controller produces proportional output ────────────────
void test_p_only_proportional_output() {
    PidController pid;
    pid.configure({2.0f, 0.0f, 0.0f}, -100.0f, +100.0f, 100.0f);

    const float u = pid.update(10.0f, 0.0f, /*dt=*/0.01f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 20.0f, u);  // Kp=2, error=10 → 20
}

// ─── Test: integral term accumulates over time ───────────────────────────
void test_integral_accumulates() {
    PidController pid;
    pid.configure({0.0f, 1.0f, 0.0f}, -100.0f, +100.0f, 100.0f);

    float u = 0.0f;
    for (int i = 0; i < 10; ++i) u = pid.update(1.0f, 0.0f, /*dt=*/0.1f);
    // After 10 ticks with dt=0.1, error=1, integral = 1.0, Ki*integral = 1.0
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, u);
}

// ─── Test: anti-windup clamps the integral term ──────────────────────────
void test_integral_clamped_to_limit() {
    PidController pid;
    pid.configure({0.0f, 100.0f, 0.0f}, -100.0f, +100.0f,
                  /*integral_limit=*/10.0f);  // integral caps at 10

    float u = 0.0f;
    for (int i = 0; i < 1000; ++i) u = pid.update(1.0f, 0.0f, /*dt=*/0.1f);
    // Without clamping, output would be huge. With clamping it must be
    // exactly Ki * integral_limit = 100 * 10 = 1000, but output cap is 100.
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 100.0f, u);
}

// ─── Test: reset() clears integrator state ──────────────────────────────────
void test_reset_clears_state() {
    PidController pid;
    pid.configure({0.0f, 1.0f, 0.0f}, -100.0f, +100.0f, 100.0f);

    for (int i = 0; i < 50; ++i) pid.update(1.0f, 0.0f, 0.01f);
    TEST_ASSERT(pid.integralTerm() > 0.0f);

    pid.reset();
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid.integralTerm());
}

// ─── Test: output is clamped to the configured max ────────────────────────
void test_output_clamped() {
    PidController pid;
    pid.configure({1000.0f, 0.0f, 0.0f}, -100.0f, +100.0f, 100.0f);
    const float u = pid.update(10.0f, 0.0f, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 100.0f, u);
}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_p_only_proportional_output);
    RUN_TEST(test_integral_accumulates);
    RUN_TEST(test_integral_clamped_to_limit);
    RUN_TEST(test_reset_clears_state);
    RUN_TEST(test_output_clamped);
    return UNITY_END();
}
