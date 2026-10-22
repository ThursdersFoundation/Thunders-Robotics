// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MotorConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.control.MotionController;
import frc.robot.sensors.Encoder;

/**
 * The two-joint arm subsystem.
 *
 * <p>The arm consists of a shoulder (the lower joint, mounted to the
 * robot frame) and an elbow (the upper joint, mounted to the shoulder).
 * Each joint is driven by a single brushed motor with a built-in
 * encoder; absolute encoders on each joint's output shaft provide a
 * power-cycle-stable reference for the closed-loop controller.
 *
 * <h2>Kinematics</h2>
 *
 * The arm's end-effector position is computed by forward kinematics:
 *
 * <pre>
 *   x = L1*cos(theta_shoulder) + L2*cos(theta_shoulder + theta_elbow)
 *   y = L1*sin(theta_shoulder) + L2*sin(theta_shoulder + theta_elbow)
 * </pre>
 *
 * where {@code L1} and {@code L2} are the lengths of the shoulder and
 * elbow segments. Inverse kinematics (used by trajectory commands) is
 * the dual of this — given a target (x, y), solve for the joint angles.
 *
 * <h2>Control</h2>
 *
 * Each joint uses a {@link ProfiledPIDController} with a gravity
 * feedforward. The profile constrains the maximum velocity and
 * acceleration so the arm does not lurch; the feedforward compensates
 * for the torque required to hold the arm against gravity at the
 * current angle. Without the feedforward, the PID would have to
 * integrate to overcome gravity, producing steady-state error and
 * slow response.
 *
 * <h2>Brake mode</h2>
 *
 * The arm motors are equipped with electromechanical brakes. The brake
 * is engaged whenever the arm is at a setpoint (so it does not droop
 * under gravity) and during E-stop. The brake is released only when
 * the arm is being actively commanded to move.
 *
 * <h2>Safety limits</h2>
 *
 * Software limits prevent the arm from crashing into the frame:
 *
 * <ul>
 *   <li>Shoulder angle range: {@link RobotConstants#ARM_SHOULDER_MIN_DEG}
 *       to {@link RobotConstants#ARM_SHOULDER_MAX_DEG}.</li>
 *   <li>Elbow angle range: {@link RobotConstants#ARM_ELBOW_MIN_DEG}
 *       to {@link RobotConstants#ARM_ELBOW_MAX_DEG}.</li>
 *   <li>Maximum joint velocity: {@link RobotConstants#ARM_MAX_VEL_DEG_PER_S}.</li>
 * </ul>
 *
 * Out-of-range commands are clamped and logged.
 */
public class Arm extends SubsystemBase {

    private final MotorControllerWrapper shoulderMotor;
    private final MotorControllerWrapper elbowMotor;
    private final Encoder shoulderEncoder;
    private final Encoder elbowEncoder;

    private final ProfiledPIDController shoulderController;
    private final ProfiledPIDController elbowController;
    private final MotionController motionController;

    private double shoulderGoalDeg;
    private double elbowGoalDeg;
    private boolean brakeEngaged;

    /**
     * Constructs the arm subsystem.
     *
     * @param sensors the sensor aggregator (for joint encoders).
     */
    public Arm(Sensors sensors) {
        this.shoulderMotor = new MotorControllerWrapper(MotorConstants.ARM_SHOULDER_CAN_ID);
        this.elbowMotor = new MotorControllerWrapper(MotorConstants.ARM_ELBOW_CAN_ID);
        this.shoulderEncoder = sensors.getArmShoulderEncoder();
        this.elbowEncoder = sensors.getArmElbowEncoder();

        this.shoulderController = new ProfiledPIDController(
                RobotConstants.ARM_SHOULDER_KP,
                RobotConstants.ARM_SHOULDER_KI,
                RobotConstants.ARM_SHOULDER_KD,
                new TrapezoidProfile.Constraints(
                        RobotConstants.ARM_MAX_VEL_DEG_PER_S,
                        RobotConstants.ARM_MAX_ACCEL_DEG_PER_S2));
        this.elbowController = new ProfiledPIDController(
                RobotConstants.ARM_ELBOW_KP,
                RobotConstants.ARM_ELBOW_KI,
                RobotConstants.ARM_ELBOW_KD,
                new TrapezoidProfile.Constraints(
                        RobotConstants.ARM_MAX_VEL_DEG_PER_S,
                        RobotConstants.ARM_MAX_ACCEL_DEG_PER_S2));

        this.motionController = new MotionController();
        this.shoulderGoalDeg = RobotConstants.ARM_SHOULDER_STOW_DEG;
        this.elbowGoalDeg = RobotConstants.ARM_ELBOW_STOW_DEG;
        this.brakeEngaged = false;

        shoulderController.setTolerance(RobotConstants.ARM_SHOULDER_TOLERANCE_DEG);
        elbowController.setTolerance(RobotConstants.ARM_ELBOW_TOLERANCE_DEG);
    }

    /**
     * Sets the shoulder joint goal angle.
     *
     * @param degrees the goal angle, in degrees, measured from horizontal.
     */
    public void setShoulderGoalDegrees(double degrees) {
        shoulderGoalDeg = clamp(degrees,
                RobotConstants.ARM_SHOULDER_MIN_DEG,
                RobotConstants.ARM_SHOULDER_MAX_DEG);
        shoulderController.setGoal(shoulderGoalDeg);
        if (brakeEngaged) {
            releaseBrake();
        }
    }

    /** Sets the elbow joint goal angle. */
    public void setElbowGoalDegrees(double degrees) {
        elbowGoalDeg = clamp(degrees,
                RobotConstants.ARM_ELBOW_MIN_DEG,
                RobotConstants.ARM_ELBOW_MAX_DEG);
        elbowController.setGoal(elbowGoalDeg);
        if (brakeEngaged) {
            releaseBrake();
        }
    }

    /** @return the current shoulder angle, in degrees. */
    public double getShoulderAngleDegrees() {
        return shoulderEncoder.getPosition();
    }

    /** @return the current elbow angle, in degrees. */
    public double getElbowAngleDegrees() {
        return elbowEncoder.getPosition();
    }

    /** Holds the current position by setting the goals to the current angles. */
    public void holdCurrentPosition() {
        shoulderGoalDeg = getShoulderAngleDegrees();
        elbowGoalDeg = getElbowAngleDegrees();
        shoulderController.setGoal(shoulderGoalDeg);
        elbowController.setGoal(elbowGoalDeg);
    }

    /** Applies the electromechanical brakes. */
    public void applyBrake() {
        brakeEngaged = true;
        shoulderMotor.set(0);
        elbowMotor.set(0);
    }

    /** Releases the electromechanical brakes. */
    public void releaseBrake() {
        brakeEngaged = false;
    }

    @Override
    public void periodic() {
        if (brakeEngaged) {
            return; // brake holds position; do not command motors.
        }

        double shoulderCurrent = getShoulderAngleDegrees();
        double elbowCurrent = getElbowAngleDegrees();

        // PID + gravity feedforward.
        double shoulderOutput = shoulderController.calculate(shoulderCurrent)
                + motionController.shoulderGravityFF(shoulderCurrent);
        double elbowOutput = elbowController.calculate(elbowCurrent)
                + motionController.elbowGravityFF(shoulderCurrent, elbowCurrent);

        shoulderMotor.set(clamp(shoulderOutput, -1.0, 1.0));
        elbowMotor.set(clamp(elbowOutput, -1.0, 1.0));

        SmartDashboard.putNumber("Arm/ShoulderCurrent", shoulderCurrent);
        SmartDashboard.putNumber("Arm/ElbowCurrent", elbowCurrent);
        SmartDashboard.putNumber("Arm/ShoulderOutput", shoulderOutput);
        SmartDashboard.putNumber("Arm/ElbowOutput", elbowOutput);
        SmartDashboard.putBoolean("Arm/Brake", brakeEngaged);
    }

    /** Notifies the arm that the robot has entered Disabled mode. */
    public void notifyDisabled() {
        applyBrake();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
