// ============================================================================
// control_test.cpp
// ----------------------------------------------------------------------------
// Repository : Thunders Robotics
// Package    : thunders::control
// Framework  : Standalone C++17  (no ROS dependency in the controller)
// Test Tool  : GoogleTest (gtest)
// ----------------------------------------------------------------------------
// Coverage
//   1. PID controller       - closed-loop convergence + clamping
//   2. Feedforward          - velocity feedforward accuracy
//   3. Motion profile       - trapezoidal profile generation
//   4. Slew rate limiter    - output never exceeds max dV/dt
// ----------------------------------------------------------------------------
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace thunders::control {
namespace {

// =========================================================================
// SUBJECT UNDER TEST
// =========================================================================

/// Minimal PID controller with anti-windup and clamping.
class PIDController {
public:
    PIDController(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd) {}

    void set_output_limits(double lo, double hi) {
        lower_ = lo;
        upper_ = hi;
    }

    /// Run one PID step. dt is in seconds.
    double calculate(double error, double dt) {
        double tentative_p = kp_ * error;
        double tentative_i = integral_ + ki_ * error * dt;
        double tentative_d =
            first_call_ ? 0.0 : kd_ * (error - last_error_) / dt;

        double tentative = tentative_p + tentative_i + tentative_d;
        // Anti-windup: only accumulate integral when output is not saturated.
        if (tentative <= upper_ && tentative >= lower_) {
            integral_ = tentative_i;
        }
        first_call_ = false;
        last_error_ = error;

        double out = kp_ * error + integral_ + tentative_d;
        if (out > upper_) out = upper_;
        if (out < lower_) out = lower_;
        return out;
    }

    bool at_setpoint(double tol) const {
        return std::abs(last_error_) <= tol;
    }

private:
    double kp_, ki_, kd_;
    double integral_     = 0.0;
    double last_error_   = 0.0;
    double lower_        = -std::numeric_limits<double>::infinity();
    double upper_        =  std::numeric_limits<double>::infinity();
    bool   first_call_   = true;
};

/// Simple velocity feedforward: u_ff = kv * v_target + ka * a_target.
double feedforward(double kv, double v_target,
                   double ka, double a_target) {
    return kv * v_target + ka * a_target;
}

/// Trapezoidal motion profile generator.
struct MotionState { double pos; double vel; };
class TrapezoidalProfile {
public:
    TrapezoidalProfile(double max_vel, double max_accel)
        : max_vel_(max_vel), max_accel_(max_accel) {}

    /// Generate states for a move of `distance` over a uniform dt grid.
    std::vector<MotionState> generate(double distance, double dt) const {
        std::vector<MotionState> states;
        const double accel_time = max_vel_ / max_accel_;
        const double accel_dist = 0.5 * max_accel_ * accel_time * accel_time;
        double cruise_dist = std::abs(distance) - 2.0 * accel_dist;
        double cruise_time = std::max(0.0, cruise_dist / max_vel_);
        const double sign = (distance >= 0.0) ? 1.0 : -1.0;

        double t = 0.0;
        double pos = 0.0;
        // Acceleration phase
        for (int i = 0; t < accel_time; ++i, t += dt) {
            double v = max_accel_ * t;
            pos = 0.5 * max_accel_ * t * t;
            states.push_back({sign * pos, sign * v});
        }
        // Cruise phase
        double cruise_start = pos;
        for (double ct = 0.0; ct < cruise_time; ct += dt) {
            pos = cruise_start + max_vel_ * ct;
            states.push_back({sign * pos, sign * max_vel_});
        }
        // Deceleration phase
        double decel_start_v = max_vel_;
        double decel_start_pos = states.empty() ? 0.0
                                                : states.back().pos;
        for (double dt2 = 0.0; decel_start_v > 0.0; dt2 += dt) {
            double v = std::max(0.0, max_vel_ - max_accel_ * dt2);
            pos = decel_start_pos + (decel_start_v + v) * 0.5 * dt2;
            states.push_back({sign * pos, sign * v});
            decel_start_v = v;
        }
        return states;
    }

private:
    double max_vel_;
    double max_accel_;
};

/// Slew rate limiter: limits the rate of change of the output signal.
class SlewRateLimiter {
public:
    explicit SlewRateLimiter(double max_slew_per_sec)
        : max_slew_(max_slew_per_sec), last_(0.0) {}

    double update(double target, double dt) {
        double max_step = max_slew_ * dt;
        double delta = target - last_;
        delta = std::clamp(delta, -max_step, max_step);
        last_ += delta;
        return last_;
    }

    void reset() { last_ = 0.0; }

private:
    double max_slew_;
    double last_;
};

}  // namespace
}  // namespace thunders::control

// =========================================================================
// 1. PID CONTROLLER
// =========================================================================
namespace {
using thunders::control::PIDController;

TEST(PIDControllerTest, PureProportionalOutputIsKpTimesError) {
    PIDController pid(2.0, 0.0, 0.0);
    pid.set_output_limits(-10.0, 10.0);
    EXPECT_DOUBLE_EQ(1.0, pid.calculate(0.5, 0.01));
}

TEST(PIDControllerTest, NegativeErrorProducesNegativeOutput) {
    PIDController pid(3.0, 0.0, 0.0);
    pid.set_output_limits(-10.0, 10.0);
    EXPECT_DOUBLE_EQ(-1.2, pid.calculate(-0.4, 0.01));
}

TEST(PIDControllerTest, OutputIsClampedToUpperLimit) {
    PIDController pid(1e6, 0.0, 0.0);
    pid.set_output_limits(-1.0, 1.0);
    EXPECT_DOUBLE_EQ(1.0, pid.calculate(100.0, 0.01));
}

TEST(PIDControllerTest, ClosedLoopConvergesToSetpoint) {
    PIDController pid(2.0, 1.0, 0.5);
    pid.set_output_limits(-1.0, 1.0);

    double x = 0.0;
    const double setpoint = 1.0;
    const double plant_gain = 0.6;
    const double dt = 0.01;

    for (int i = 0; i < 5000; ++i) {
        double error = setpoint - x;
        double u = pid.calculate(error, dt);
        x += u * plant_gain * dt;
    }
    EXPECT_NEAR(setpoint, x, 1e-2);
}

TEST(PIDControllerTest, AtSetpointFlagReflectsTolerance) {
    PIDController pid(10.0, 0.0, 0.0);
    pid.set_output_limits(-10.0, 10.0);
    pid.calculate(0.005, 0.01);  // |error| < 0.01
    EXPECT_TRUE(pid.at_setpoint(0.01));

    pid.calculate(0.05, 0.01);   // |error| > 0.01
    EXPECT_FALSE(pid.at_setpoint(0.01));
}

// =========================================================================
// 2. FEEDFORWARD
// =========================================================================
using thunders::control::feedforward;

TEST(FeedforwardTest, PureVelocityFeedforward) {
    // kv = 0.1 V per (m/s), ka = 0
    EXPECT_DOUBLE_EQ(1.0, feedforward(0.1, 10.0, 0.0, 0.0));
}

TEST(FeedforwardTest, CombinedVelocityAndAccelerationFeedforward) {
    // u = kv * v + ka * a
    EXPECT_DOUBLE_EQ(2.5, feedforward(0.1, 10.0, 0.5, 3.0));
}

TEST(FeedforwardTest, NegativeVelocityGivesNegativeFeedforward) {
    EXPECT_DOUBLE_EQ(-0.5, feedforward(0.1, -5.0, 0.0, 0.0));
}

// =========================================================================
// 3. MOTION PROFILE (TRAPEZOIDAL)
// =========================================================================
using thunders::control::MotionState;
using thunders::control::TrapezoidalProfile;

TEST(TrapezoidalProfileTest, ProfileStartsAtZeroVelocity) {
    TrapezoidalProfile p(1.0, 2.0);
    auto states = p.generate(2.0, 0.01);
    ASSERT_FALSE(states.empty());
    EXPECT_NEAR(0.0, states.front().vel, 1e-9);
    EXPECT_NEAR(0.0, states.front().pos, 1e-9);
}

TEST(TrapezoidalProfileTest, ProfileNeverExceedsMaxVelocity) {
    TrapezoidalProfile p(1.0, 2.0);
    auto states = p.generate(2.0, 0.01);
    for (const auto& s : states) {
        EXPECT_LE(std::abs(s.vel), 1.0 + 1e-9)
            << "Velocity " << s.vel << " exceeds max_vel 1.0";
    }
}

TEST(TrapezoidalProfileTest, ProfileEndsNearZeroVelocity) {
    TrapezoidalProfile p(1.0, 2.0);
    auto states = p.generate(2.0, 0.01);
    ASSERT_FALSE(states.empty());
    EXPECT_NEAR(0.0, states.back().vel, 1e-3);
}

TEST(TrapezoidalProfileTest, ProfileEndsAtTargetDistance) {
    TrapezoidalProfile p(1.0, 2.0);
    auto states = p.generate(2.0, 0.001);
    ASSERT_FALSE(states.empty());
    EXPECT_NEAR(2.0, states.back().pos, 5e-2);
}

// =========================================================================
// 4. SLEW RATE LIMITER
// =========================================================================
using thunders::control::SlewRateLimiter;

TEST(SlewRateLimiterTest, OutputNeverExceedsMaxSlew) {
    SlewRateLimiter lim(5.0);  // 5 units/sec max
    // Request a huge jump from 0 to 100 in 0.1s.
    double out = lim.update(100.0, 0.1);
    EXPECT_LE(out, 5.0 * 0.1 + 1e-9)
        << "Output must be limited to max_slew * dt = 0.5";
}

TEST(SlewRateLimiterTest, OutputReachesTargetAfterEnoughTime) {
    SlewRateLimiter lim(5.0);
    double out = 0.0;
    for (int i = 0; i < 1000; ++i) out = lim.update(7.0, 0.01);
    EXPECT_NEAR(7.0, out, 1e-9);
}

TEST(SlewRateLimiterTest, ResetReturnsOutputToZero) {
    SlewRateLimiter lim(5.0);
    lim.update(10.0, 0.5);
    lim.reset();
    EXPECT_DOUBLE_EQ(0.0, lim.update(0.0, 0.1));
}

TEST(SlewRateLimiterTest, NegativeSlewIsAlsoLimited) {
    SlewRateLimiter lim(5.0);
    lim.update(5.0, 1.0);  // get to a known state
    double out = lim.update(-50.0, 0.1);
    // One step of dt=0.1 -> max step = 0.5
    EXPECT_GE(out, 5.0 - 5.0 * 0.1 - 1e-9)
        << "Negative slew must be limited similarly";
}

// Entry point for standalone execution.
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
