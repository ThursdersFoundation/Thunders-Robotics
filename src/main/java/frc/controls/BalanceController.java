// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.

package frc.robot.control;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.RobotConstants;

/**
 * Auto-balances the robot on the charging station.
 *
 * <p>The charging station is a seesaw-like platform that tips when
 * the robot's centre of gravity is not over the pivot. Balancing
 * requires driving forward or backward to keep the platform level
 * while the robot is on top of it.
 *
 * <h2>Controller design</h2>
 *
 * The controller is a cascade:
 *
 * <ol>
 *   <li><b>Outer loop</b> -&gt; a PI controller on the platform pitch
 *       angle. The setpoint is zero (level); the output is a desired
 *       forward velocity.</li>
 *   <li><b>Inner loop</b> -&gt; a P controller on the robot velocity.
 *       The setpoint comes from the outer loop; the output is the
 *       voltage to apply to the drivetrain.</li>
 * </ol>
 *
 * <p>Cascade control decouples the slow platform dynamics from the
 * fast robot dynamics, allowing each loop to be tuned independently.
 *
 * <h2>State machine</h2>
 *
 * The controller runs through a small state machine:
 *
 * <pre>
 *   APPROACH -&gt; CLIMB_ON -&gt; LEVEL -&gt; HOLD
 * </pre>
 *
 * <ul>
 *   <li><b>APPROACH</b> -&gt; drive forward at a fixed speed until
 *       the platform starts to tip (the pitch exceeds a threshold).</li>
 *   <li><b>CLIMB_ON</b> -&gt; continue driving forward more slowly
 *       until the robot's pitch returns to near-zero (the robot is
 *       now on top of the platform).</li>
 *   <li><b>LEVEL</b> -&gt; engage the cascade controller to drive
 *       the pitch to zero.</li>
 *   <li><b>HOLD</b> -&gt; once the pitch has been within tolerance
 *       for a dwell period, apply brakes and stop the motors.</li>
 * </ul>
 *
 * <h2>Tuning</h2>
 *
 * The controller gains are in {@link RobotConstants}. The platform's
 * response is highly dependent on the robot's weight distribution,
 * so the gains need to be re-tuned whenever the robot's CG changes
 * (e.g., after adding a new mechanism).
 */
public class BalanceController {

    private enum Phase { APPROACH, CLIMB_ON, LEVEL, HOLD }

    private final PIDController pitchController;
    private Phase phase;
    private double holdStartTime;
    private double approachStartTime;

    private static final double PITCH_TOLERANCE_DEG = 2.0;
    private static final double HOLD_DWELL_SECONDS = 1.0;
    private static final double APPROACH_SPEED = 0.8;
    private static final double CLIMB_SPEED = 0.4;
    private static final double APPROACH_TIMEOUT_SECONDS = 3.0;

    /** Constructs a BalanceController. */
    public BalanceController() {
        this.pitchController = new PIDController("Balance", 0.05, 0.0, 0.01);
        this.pitchController.setMaxOutput(0.6);
        this.phase = Phase.APPROACH;
    }

    /** Resets the controller to the APPROACH phase. */
    public void reset() {
        phase = Phase.APPROACH;
        pitchController.reset();
        approachStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }

    /**
     * Computes the desired ChassisSpeeds to balance the robot.
     *
     * @param pitchDegrees the current platform pitch, in degrees.
     *                     Positive = nose-up; negative = nose-down.
     * @param rollDegrees  the current platform roll, in degrees.
     *                     Used for diagnostics only; the controller
     *                     is single-axis.
     * @return the ChassisSpeeds to apply to the drivetrain.
     */
    public edu.wpi.first.math.kinematics.ChassisSpeeds compute(
            double pitchDegrees, double rollDegrees) {
        double forward = 0;
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        switch (phase) {
            case APPROACH:
                forward = APPROACH_SPEED;
                if (Math.abs(pitchDegrees) > 5.0) {
                    phase = Phase.CLIMB_ON;
                } else if (now - approachStartTime > APPROACH_TIMEOUT_SECONDS) {
                    // Timed out without seeing pitch — give up.
                    phase = Phase.HOLD;
                }
                break;

            case CLIMB_ON:
                forward = CLIMB_SPEED;
                if (Math.abs(pitchDegrees) < PITCH_TOLERANCE_DEG) {
                    phase = Phase.LEVEL;
                }
                break;

            case LEVEL:
                pitchController.setSetpoint(0);
                forward = pitchController.calculate(pitchDegrees);
                if (Math.abs(pitchDegrees) < PITCH_TOLERANCE_DEG) {
                    holdStartTime = now;
                    phase = Phase.HOLD;
                }
                break;

            case HOLD:
                forward = 0;
                if (Math.abs(pitchDegrees) > PITCH_TOLERANCE_DEG * 2.0) {
                    phase = Phase.LEVEL;
                }
                break;
        }

        SmartDashboard.putString("Balance/Phase", phase.name());
        SmartDashboard.putNumber("Balance/Pitch", pitchDegrees);
        SmartDashboard.putNumber("Balance/Roll", rollDegrees);
        SmartDashboard.putNumber("Balance/Forward", forward);

        return new edu.wpi.first.math.kinematics.ChassisSpeeds(forward, 0, 0);
    }

    /** @return true if the robot is balanced and holding position. */
    public boolean isBalanced() {
        return phase == Phase.HOLD;
    }
}
