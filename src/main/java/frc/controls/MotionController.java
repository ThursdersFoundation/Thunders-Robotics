// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.

package frc.robot.control;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.RobotConstants;

/**
 * Generates trapezoidal motion profiles for single-DOF mechanisms
 * (e.g., the elevator or a single arm joint).
 *
 * <p>A trapezoidal profile is the simplest motion profile that
 * respects velocity and acceleration limits. It consists of three
 * phases:
 *
 * <ol>
 *   <li><b>Acceleration</b> -&gt; the velocity ramps up at the
 *       maximum allowed acceleration until the cruise velocity is
 *       reached.</li>
 *   <li><b>Cruise</b> -&gt; the velocity is held constant at the
 *       cruise velocity.</li>
 *   <li><b>Deceleration</b> -&gt; the velocity ramps down at the
 *       maximum allowed acceleration so that the velocity reaches
 *       zero exactly as the goal position is reached.</li>
 * </ol>
 *
 * For short moves, the cruise phase may be omitted (a "triangular"
 * profile). For very short moves, even the cruise and one of the
 * acceleration phases may be omitted.
 *
 * <h2>Why profile motion?</h2>
 *
 * Naively commanding a mechanism to "go to position X" causes the
 * motor to accelerate instantly to its maximum speed, which produces
 * a violent lurch and stresses the mechanical components. A motion
 * profile smooths out the command so the motor accelerates and
 * decelerates gradually, dramatically extending mechanism life and
 * improving accuracy.
 *
 * <h2>Integration with PID</h2>
 *
 * The profile generates a desired position and velocity at each
 * sample. A PID controller tracks the desired position; a feedforward
 * term proportional to the desired velocity provides the bulk of the
 * motor output. This "profile + PID + FF" structure is the standard
 * way to control a single-DOF mechanism in FRC.
 *
 * <h2>Gravity feedforward</h2>
 *
 * For mechanisms that work against gravity (e.g., the arm), the
 * gravity feedforward varies with the joint angle. This class
 * provides helper methods {@link #shoulderGravityFF(double)} and
 * {@link #elbowGravityFF(double, double)} that compute the
 * angle-dependent feedforward using a simple cosine model.
 */
public class MotionController {

    private final TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;
    private TrapezoidProfile profile;

    /** Constructs a MotionController with default constraints from {@link RobotConstants}. */
    public MotionController() {
        this.constraints = new TrapezoidProfile.Constraints(
                RobotConstants.ARM_MAX_VEL_DEG_PER_S,
                RobotConstants.ARM_MAX_ACCEL_DEG_PER_S2);
        this.goal = new TrapezoidProfile.State();
        this.setpoint = new TrapezoidProfile.State();
    }

    /**
     * Sets the goal state (position and velocity) of the profile.
     *
     * @param position the goal position.
     * @param velocity the goal velocity (usually 0 for a "stop at end" move).
     */
    public void setGoal(double position, double velocity) {
        this.goal = new TrapezoidProfile.State(position, velocity);
        this.profile = new TrapezoidProfile(constraints, goal, setpoint);
    }

    /**
     * Advances the profile by the given time step and returns the
     * new setpoint state.
     *
     * @param dtSeconds the time step, in seconds.
     * @return the setpoint state (position + velocity).
     */
    public TrapezoidProfile.State update(double dtSeconds) {
        if (profile == null) {
            return setpoint;
        }
        setpoint = profile.calculate(dtSeconds);
        return setpoint;
    }

    /** @return true if the profile has reached the goal. */
    public boolean atGoal() {
        if (profile == null) {
            return true;
        }
        return profile.isFinished(0);
    }

    /** Resets the profile state to the given position and zero velocity. */
    public void reset(double currentPosition) {
        this.setpoint = new TrapezoidProfile.State(currentPosition, 0);
        this.profile = new TrapezoidProfile(constraints, goal, setpoint);
    }

    /**
     * Computes the gravity feedforward for the shoulder joint.
     *
     * <p>The shoulder's centre of mass is at distance L from the
     * pivot, and the gravity torque is {@code m*g*L*cos(theta)}.
     * Normalising by the maximum torque gives a feedforward term
     * that varies between {@code -KG_MAX} (arm horizontal) and 0
     * (arm vertical).
     *
     * @param shoulderDegrees the current shoulder angle, in degrees.
     * @return the feedforward term, in the range [-1, 1].
     */
    public double shoulderGravityFF(double shoulderDegrees) {
        double rad = Math.toRadians(shoulderDegrees);
        return RobotConstants.ARM_SHOULDER_KG * Math.cos(rad);
    }

    /**
     * Computes the gravity feedforward for the elbow joint. The
     * elbow's torque depends on both the shoulder and elbow angles
     * because the shoulder angle affects the elbow's distance from
     * the gravity vector.
     *
     * @param shoulderDegrees the current shoulder angle, in degrees.
     * @param elbowDegrees the current elbow angle, in degrees.
     * @return the feedforward term.
     */
    public double elbowGravityFF(double shoulderDegrees, double elbowDegrees) {
        double shoulderRad = Math.toRadians(shoulderDegrees);
        double elbowRad = Math.toRadians(elbowDegrees);
        // The elbow's contribution is the cosine of the absolute angle.
        double absoluteAngle = shoulderRad + elbowRad;
        return RobotConstants.ARM_ELBOW_KG * Math.cos(absoluteAngle);
    }

    /** Publishes the current profile state to the dashboard for debugging. */
    public void publishDiagnostics(String prefix) {
        SmartDashboard.putNumber(prefix + "/GoalPos", goal.position);
        SmartDashboard.putNumber(prefix + "/SetpointPos", setpoint.position);
        SmartDashboard.putNumber(prefix + "/SetpointVel", setpoint.velocity);
        SmartDashboard.putBoolean(prefix + "/AtGoal", atGoal());
    }
}
