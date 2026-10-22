// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModuleConstants;

/**
 * Top-level physical and operational constants for the Thunders robot.
 *
 * <p>This class is the single source of truth for the robot's physical
 * dimensions, performance limits, and operational parameters. Tuning
 * a constant here is a one-line change that takes effect on the next
 * deploy; tuning the same constant scattered across multiple files is
 * a multi-hour exercise in grep-and-edit.
 *
 * <h2>How to use this file</h2>
 *
 * <ul>
 *   <li>All constants are {@code public static final} so the compiler
 *       inlines them at call sites — there is no runtime cost.</li>
 *   <li>Constants are grouped by subsystem. Each subsystem has a
 *       comment block explaining its constants.</li>
 *   <li>Numeric constants include a unit suffix (e.g.,
 *       {@code MAX_DRIVE_SPEED_MPS}) so the unit is obvious at every
 *       call site.</li>
 *   <li>Build metadata ({@link #BUILD_VERSION}, {@link #BUILD_DATE},
 *       {@link #GIT_HASH}) is set by the Gradle build script and
 *       baked into the JAR at deploy time.</li>
 * </ul>
 *
 * <h2>Why not a YAML file?</h2>
 *
 * Constants in a YAML file would be tunable without recompiling, but
 * they would also be invisible to the compiler — a typo would not be
 * caught until runtime. The WPILib convention is to use Java
 * constants for compile-time safety and to use Shuffleboard for
 * runtime tuning.
 */
public final class RobotConstants {

    private RobotConstants() {
        throw new AssertionError("RobotConstants is a constants holder.");
    }

    // ----------------------------------------------------------------------
    // Build metadata
    // ----------------------------------------------------------------------

    /** The semantic version of the deployed code, e.g., "2026.1.0". */
    public static final String BUILD_VERSION = "2026.1.0";
    /** The build timestamp, in ISO 8601 format. */
    public static final String BUILD_DATE = "2026-01-15T08:00:00Z";
    /** The Git commit hash the build was created from. */
    public static final String GIT_HASH = "abc1234";

    // ----------------------------------------------------------------------
    // Loop timing
    // ----------------------------------------------------------------------

    /** The WPILib periodic loop period, in seconds. Default 20 ms. */
    public static final double LOOP_PERIOD_SECONDS = 0.020;

    // ----------------------------------------------------------------------
    // Driver station / operator interface
    // ----------------------------------------------------------------------

    /** The USB port the driver's Xbox controller is plugged into. */
    public static final int DRIVER_CONTROLLER_PORT = 0;
    /** The USB port the operator's joystick is plugged into. */
    public static final int OPERATOR_JOYSTICK_PORT = 1;

    // ----------------------------------------------------------------------
    // Drivetrain
    // ----------------------------------------------------------------------

    /** Maximum forward/lateral drive speed, in metres per second. */
    public static final double MAX_DRIVE_SPEED_MPS = 4.5;
    /** Maximum drive acceleration, in m/s^2. */
    public static final double MAX_DRIVE_ACCEL_M_PER_S2 = 6.0;
    /** Maximum rotational speed, in rad/s. */
    public static final double MAX_ANGULAR_SPEED_RAD_PER_S = Math.PI;
    /** Maximum rotational acceleration, in rad/s^2. */
    public static final double MAX_ANGULAR_ACCEL_RAD_PER_S2 = Math.PI * 2.0;
    /** Maximum centripetal acceleration, in m/s^2. Limits cornering speed. */
    public static final double MAX_CENTRIPETAL_ACCEL_M_PER_S2 = 4.0;

    /** Slew-rate limit for translational inputs, in (m/s)/s. */
    public static final double DRIVE_SLEW_ACCEL = 8.0;
    /** Slew-rate limit for rotational inputs, in (rad/s)/s. */
    public static final double DRIVE_SLEW_ROT = Math.PI * 2.0;

    /** Swerve module mounting positions, in metres from the robot centre. */
    public static final Translation2d FRONT_LEFT_OFFSET  = new Translation2d( 0.30,  0.30);
    public static final Translation2d FRONT_RIGHT_OFFSET = new Translation2d( 0.30, -0.30);
    public static final Translation2d BACK_LEFT_OFFSET   = new Translation2d(-0.30,  0.30);
    public static final Translation2d BACK_RIGHT_OFFSET  = new Translation2d(-0.30, -0.30);

    /** Swerve module configurations (CAN IDs, offsets, etc.). */
    public static final SwerveModuleConstants FRONT_LEFT_CONFIG  = new SwerveModuleConstants(1, 2, 3, Rotation2d.fromDegrees(-0.5));
    public static final SwerveModuleConstants FRONT_RIGHT_CONFIG = new SwerveModuleConstants(4, 5, 6, Rotation2d.fromDegrees( 90.2));
    public static final SwerveModuleConstants BACK_LEFT_CONFIG   = new SwerveModuleConstants(7, 8, 9, Rotation2d.fromDegrees(179.8));
    public static final SwerveModuleConstants BACK_RIGHT_CONFIG  = new SwerveModuleConstants(10,11,12, Rotation2d.fromDegrees(-89.9));

    /** Path-following PID gains. */
    public static final double PATH_X_KP = 1.5;
    public static final double PATH_X_KD = 0.0;
    public static final double PATH_Y_KP = 1.5;
    public static final double PATH_Y_KD = 0.0;
    public static final double PATH_THETA_KP = 3.0;
    public static final double PATH_THETA_KD = 0.0;

    /** Vision ambiguity threshold; estimates above this are discarded. */
    public static final double VISION_AMBIGUITY_MAX = 0.2;
    /** Camera-to-robot transform, in metres and radians. */
    public static final Transform3d CAMERA_TO_ROBOT_TRANSFORM = new Transform3d(
            new Translation3d(0.15, 0.0, 0.30),
            new edu.wpi.first.math.geometry.Rotation3d());

    // ----------------------------------------------------------------------
    // Arm
    // ----------------------------------------------------------------------

    public static final double ARM_SHOULDER_KP = 0.05;
    public static final double ARM_SHOULDER_KI = 0.0;
    public static final double ARM_SHOULDER_KD = 0.005;
    public static final double ARM_SHOULDER_KG = 0.20;
    public static final double ARM_ELBOW_KP = 0.04;
    public static final double ARM_ELBOW_KI = 0.0;
    public static final double ARM_ELBOW_KD = 0.004;
    public static final double ARM_ELBOW_KG = 0.15;

    public static final double ARM_MAX_VEL_DEG_PER_S = 180.0;
    public static final double ARM_MAX_ACCEL_DEG_PER_S2 = 360.0;
    public static final double ARM_SHOULDER_TOLERANCE_DEG = 2.0;
    public static final double ARM_ELBOW_TOLERANCE_DEG = 2.0;
    public static final double ARM_MOVE_TIMEOUT_SECONDS = 3.0;

    public static final double ARM_SHOULDER_MIN_DEG = -90.0;
    public static final double ARM_SHOULDER_MAX_DEG =  90.0;
    public static final double ARM_ELBOW_MIN_DEG = -135.0;
    public static final double ARM_ELBOW_MAX_DEG =  135.0;

    public static final double ARM_SHOULDER_STOW_DEG = 0.0;
    public static final double ARM_ELBOW_STOW_DEG = 0.0;
    public static final double ARM_SHOULDER_INTAKE_DEG = -45.0;
    public static final double ARM_ELBOW_INTAKE_DEG = 90.0;
    public static final double ARM_SHOULDER_SPEAKER_PODIUM_DEG = 20.0;
    public static final double ARM_ELBOW_SPEAKER_PODIUM_DEG = 30.0;
    public static final double ARM_SHOULDER_SPEAKER_SUB_DEG = 35.0;
    public static final double ARM_ELBOW_SPEAKER_SUB_DEG = 45.0;
    public static final double ARM_SHOULDER_AMP_DEG = 60.0;
    public static final double ARM_ELBOW_AMP_DEG = 60.0;
    public static final double ARM_SHOULDER_CLIMB_DEG = 80.0;
    public static final double ARM_ELBOW_CLIMB_DEG = 100.0;

    // ----------------------------------------------------------------------
    // Elevator
    // ----------------------------------------------------------------------

    public static final double ELEVATOR_KP = 5.0;
    public static final double ELEVATOR_KI = 0.0;
    public static final double ELEVATOR_KD = 0.5;
    public static final double ELEVATOR_KG = 0.10;
    public static final double ELEVATOR_MAX_VEL_M_PER_S = 1.5;
    public static final double ELEVATOR_MAX_ACCEL_M_PER_S2 = 3.0;
    public static final double ELEVATOR_TOLERANCE_M = 0.02;
    public static final double ELEVATOR_MAX_M = 1.20;
    public static final double ELEVATOR_CLIMB_MAX_M = 1.80;
    public static final int CLIMB_HOOK_SENSOR_CHANNEL = 9;

    // ----------------------------------------------------------------------
    // Intake
    // ----------------------------------------------------------------------

    public static final int PCM_ID = 0;
    public static final int INTAKE_DEPLOY_FORWARD = 0;
    public static final int INTAKE_DEPLOY_REVERSE = 1;
    public static final int INTAKE_BEAM_BREAK_CHANNEL = 8;
    public static final double INTAKE_MAX_VOLTAGE = 12.0;
    public static final double INTAKE_INTAKE_SPEED = 0.7;
    public static final double INTAKE_HOLD_SPEED = 0.15;
    public static final double INTAKE_FEED_SPEED = -0.9;
    public static final double INTAKE_HOLD_DWELL_SECONDS = 0.3;
    public static final double INTAKE_TIMEOUT_SECONDS = 3.0;

    // ----------------------------------------------------------------------
    // Shooter
    // ----------------------------------------------------------------------

    public static final double SHOOTER_KP = 0.0005;
    public static final double SHOOTER_KD = 0.0;
    public static final double SHOOTER_KS = 0.05;
    public static final double SHOOTER_KV = 0.012;
    public static final double SHOOTER_IDLE_RPM = 500.0;
    public static final double SHOOTER_SUB_RPM = 5500.0;
    public static final double SHOOTER_PODIUM_RPM = 4500.0;
    public static final double SHOOTER_WING_RPM = 3500.0;
}
