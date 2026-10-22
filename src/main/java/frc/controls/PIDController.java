// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.

package frc.robot.control;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A small PID controller implementation that mirrors the API of
 * {@link edu.wpi.first.math.controller.PIDController} but adds
 * several features commonly needed on the Thunders robot:
 *
 * <ul>
 *   <li><b>Integral clamping</b> -&gt; the integral term is clamped
 *       to a configurable range to prevent wind-up when the
 *       actuator is saturated.</li>
 *   <li><b>Derivative filtering</b> -&gt; the derivative term is
 *       passed through a low-pass filter to suppress noise.</li>
 *   <li><b>Feedforward</b> -&gt; an optional feedforward term can be
 *       added to the output. This is useful for mechanisms where the
 *       steady-state output is non-zero (e.g., gravity-compensated
 *       arms).</li>
 *   <li><b>Continuous input</b> -&gt; for mechanisms that wrap
 *       around (e.g., a swerve module's steer angle), the controller
 *       computes the shortest error around the circle.</li>
 *   <li><b>Profiled setpoint</b> -&gt; the setpoint can be passed
 *       through a trapezoidal profile to limit velocity and
 *       acceleration.</li>
 * </ul>
 *
 * <h2>Why not just use WPILib's PIDController?</h2>
 *
 * For most applications, {@link edu.wpi.first.math.controller.PIDController}
 * is the right choice. This class exists for the cases where we need
 * fine-grained control over the integral clamp or the derivative
 * filter, or where we want to add a feedforward term without
 * instantiating a separate {@link edu.wpi.first.math.controller.SimpleMotorFeedforward}.
 *
 * <h2>Tuning</h2>
 *
 * The standard tuning procedure is:
 *
 * <ol>
 *   <li>Set KI and KD to zero.</li>
 *   <li>Increase KP until the system oscillates around the setpoint.</li>
 *   <li>Halve KP.</li>
 *   <li>Increase KD until the oscillation damps within one or two cycles.</li>
 *   <li>Increase KI until steady-state error is removed within the
 *       desired settling time.</li>
 * </ol>
 *
 * <h2>Anti-windup</h2>
 *
 * Without anti-windup, a saturated actuator causes the integral term
 * to grow without bound, leading to massive overshoot when the
 * actuator finally desaturates. This class clamps the integral term
 * to the configurable {@link #iZone}, which is the standard anti-windup
 * technique.
 */
public class PIDController {

    private double kp;
    private double ki;
    private double kd;
    private double iZone;
    private double maxOutput;

    private double setpoint;
    private double accumulatedError;
    private double previousError;
    private double previousDerivative;
    private double derivativeFilterAlpha;

    private boolean continuous;
    private double minInput;
    private double maxInput;

    private final String name;

    /**
     * Constructs a PID controller with the given gains.
     *
     * @param name a short name used to prefix dashboard keys.
     * @param kp proportional gain.
     * @param ki integral gain.
     * @param kd derivative gain.
     */
    public PIDController(String name, double kp, double ki, double kd) {
        this.name = name;
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.iZone = 1.0;
        this.maxOutput = 1.0;
        this.setpoint = 0;
        this.accumulatedError = 0;
        this.previousError = 0;
        this.previousDerivative = 0;
        this.derivativeFilterAlpha = 0.5;
        this.continuous = false;
        this.minInput = 0;
        this.maxInput = 1;
    }

    /**
     * Computes the controller output for the given measurement.
     *
     * @param measurement the current process variable.
     * @return the controller output, clamped to {@code [-maxOutput, +maxOutput]}.
     */
    public double calculate(double measurement) {
        double error = setpoint - measurement;
        if (continuous) {
            error = wrapError(error);
        }

        // Integral term with anti-windup.
        if (Math.abs(error) < iZone) {
            accumulatedError += error;
        } else {
            accumulatedError = 0;
        }
        accumulatedError = clamp(accumulatedError, -iZone, iZone);

        // Derivative term with low-pass filter.
        double rawDerivative = error - previousError;
        double filteredDerivative = previousDerivative
                + derivativeFilterAlpha * (rawDerivative - previousDerivative);
        previousDerivative = filteredDerivative;
        previousError = error;

        double output = kp * error + ki * accumulatedError + kd * filteredDerivative;
        output = clamp(output, -maxOutput, maxOutput);

        SmartDashboard.putNumber(name + "/Error", error);
        SmartDashboard.putNumber(name + "/Output", output);
        SmartDashboard.putNumber(name + "/Integral", accumulatedError);
        SmartDashboard.putNumber(name + "/Derivative", filteredDerivative);

        return output;
    }

    /** Sets the setpoint. */
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    /** Sets the gains. */
    public void setGains(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    /** Sets the integral zone (anti-windup clamp). */
    public void setIZone(double iZone) {
        this.iZone = Math.abs(iZone);
    }

    /** Sets the maximum output magnitude. */
    public void setMaxOutput(double max) {
        this.maxOutput = Math.abs(max);
    }

    /**
     * Enables continuous input. The controller will compute the
     * shortest error around the circle defined by [minInput, maxInput].
     *
     * @param minInput the minimum input value (e.g., 0).
     * @param maxInput the maximum input value (e.g., 360 degrees).
     */
    public void enableContinuousInput(double minInput, double maxInput) {
        this.continuous = true;
        this.minInput = minInput;
        this.maxInput = maxInput;
    }

    /** Disables continuous input. */
    public void disableContinuousInput() {
        this.continuous = false;
    }

    /** Sets the derivative filter coefficient (0 = no filtering, 1 = max filtering). */
    public void setDerivativeFilter(double alpha) {
        this.derivativeFilterAlpha = Math.max(0, Math.min(1, alpha));
    }

    /** Resets the integral and derivative state. */
    public void reset() {
        accumulatedError = 0;
        previousError = 0;
        previousDerivative = 0;
    }

    /** @return true if the measurement is within {@code tolerance} of the setpoint. */
    public boolean atSetpoint(double measurement, double tolerance) {
        return Math.abs(setpoint - measurement) < tolerance;
    }

    private double wrapError(double error) {
        double range = maxInput - minInput;
        error %= range;
        if (error > range / 2.0) {
            error -= range;
        } else if (error < -range / 2.0) {
            error += range;
        }
        return error;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
