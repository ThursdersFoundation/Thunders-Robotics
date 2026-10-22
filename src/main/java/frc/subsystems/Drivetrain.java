// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.FollowPathCommand;
import frc.robot.autonomous.TrajectoryManager;
import frc.robot.constants.RobotConstants;
import frc.robot.control.BalanceController;
import frc.robot.robot.RobotState;
import frc.robot.sensors.Gyroscope;
import frc.robot.sensors.Encoder;

/**
 * The swerve drivetrain subsystem.
 *
 * <p>A swerve drivetrain consists of four independently-steerable
 * modules. Each module has two motors: one for driving (the wheel's
 * tangential velocity) and one for steering (the wheel's heading).
 * By coordinating the four modules, the robot can translate and rotate
 * simultaneously in any direction — there is no "front" the way there
 * is on a tank drive.
 *
 * <h2>Coordinate frames</h2>
 *
 * The robot uses three coordinate frames:
 *
 * <ul>
 *   <li><b>Robot frame</b>: origin at the centre of the robot, X axis
 *       pointing out the "front" (the side with the intake).</li>
 *   <li><b>Field frame</b>: origin at the corner of the field, X axis
 *       pointing toward the opposing alliance wall.</li>
 *   <li><b>Module frame</b>: origin at each module's pivot, X axis
 *       aligned with the wheel's tangent when the module is at zero
 *       steer angle.</li>
 * </ul>
 *
 * Conversion between robot and field frames uses the gyro's heading;
 * conversion between robot and module frames uses the
 * {@link SwerveDriveKinematics} class.
 *
 * <h2>Odometry</h2>
 *
 * Position is estimated by {@link SwerveDriveOdometry}, which fuses the
 * gyro heading with each module's drive distance and steer angle.
 * Odometry is updated on every periodic cycle (50 Hz). The estimate
 * drifts over time, so we periodically correct it using AprilTag
 * observations from the {@link Vision} subsystem.
 *
 * <h2>Brake mode</h2>
 *
 * The drivetrain supports brake mode (motor controllers short their
 * windings to resist motion) and coast mode (motors spin freely).
 * Brake mode is engaged whenever the robot is disabled or in E-stop;
 * coast mode is used during normal teleop so the robot can be pushed
 * by hand for testing.
 */
public class Drivetrain extends SubsystemBase {

    private final Gyroscope gyro;
    private final Vision vision;
    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final Field2d fieldDisplay;
    private final BalanceController balanceController;
    private boolean brakeMode;
    private boolean balanceModeActive;

    /**
     * Constructs the drivetrain subsystem.
     *
     * @param sensors the sensor aggregator (for the gyro).
     * @param vision the vision subsystem (for pose correction).
     */
    public Drivetrain(Sensors sensors, Vision vision) {
        this.gyro = sensors.getGyro();
        this.vision = vision;
        this.modules = new SwerveModule[] {
                new SwerveModule(RobotConstants.FRONT_LEFT_CONFIG),
                new SwerveModule(RobotConstants.FRONT_RIGHT_CONFIG),
                new SwerveModule(RobotConstants.BACK_LEFT_CONFIG),
                new SwerveModule(RobotConstants.BACK_RIGHT_CONFIG)
        };
        this.kinematics = new SwerveDriveKinematics(
                RobotConstants.FRONT_LEFT_OFFSET,
                RobotConstants.FRONT_RIGHT_OFFSET,
                RobotConstants.BACK_LEFT_OFFSET,
                RobotConstants.BACK_RIGHT_OFFSET);
        this.odometry = new SwerveDriveOdometry(
                kinematics, gyro.getRotation2d(), getModulePositions());
        this.fieldDisplay = new Field2d();
        this.balanceController = new BalanceController();
        this.brakeMode = false;
        this.balanceModeActive = false;

        SmartDashboard.putData("Field/Robot", fieldDisplay);
        setBrakeMode(true);
    }

    /**
     * Drives the robot at the given chassis speeds. The speeds are
     * converted to per-module states via the kinematics, then
     * dispatched to each module's closed-loop controller.
     *
     * @param speeds the desired chassis speeds, in either robot or
     *               field frame (depending on the calling command).
     */
    public void drive(ChassisSpeeds speeds) {
        if (balanceModeActive) {
            return; // balance controller takes over.
        }
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, RobotConstants.MAX_DRIVE_SPEED_MPS);
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(states[i]);
        }
    }

    /** Stops the drivetrain by commanding zero velocity. */
    public void stop() {
        drive(new ChassisSpeeds(0, 0, 0));
    }

    /** Resets the gyro's yaw to zero. Bound to the back button. */
    public void zeroYaw() {
        gyro.zero();
        DriverStationReport("Gyro zeroed");
    }

    /**
     * Resets the odometry to the given pose. Called by trajectory
     * following commands at the start of a path.
     *
     * @param pose the pose to reset to.
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
    }

    /** @return the current estimated pose of the robot on the field. */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /** @return the current yaw heading of the robot, in degrees. */
    public double getHeadingDegrees() {
        return gyro.getAngle();
    }

    /** Applies motor brakes so the robot does not coast. */
    public void applyBrake() { setBrakeMode(true); }

    /** Releases motor brakes. */
    public void releaseBrake() { setBrakeMode(false); }

    private void setBrakeMode(boolean brake) {
        brakeMode = brake;
        for (SwerveModule m : modules) {
            m.setBrakeMode(brake);
        }
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    /**
     * Periodic update. Updates odometry, fuses vision pose corrections,
     * and pushes diagnostic data to the dashboard.
     */
    @Override
    public void periodic() {
        odometry.update(gyro.getRotation2d(), getModulePositions());

        // If vision has a fresh pose, fuse it. We trust vision only
        // when the reported ambiguity is low.
        if (vision.hasNewPose()) {
            Pose2d visionPose = vision.getLatestPose();
            double ambiguity = vision.getAmbiguity();
            if (ambiguity < RobotConstants.VISION_AMBIGUITY_MAX) {
                odometry.resetPosition(gyro.getRotation2d(),
                        getModulePositions(), visionPose);
            }
        }

        fieldDisplay.setRobotPose(odometry.getPoseMeters());

        SmartDashboard.putNumber("Drive/PoseX", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Drive/PoseY", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Drive/Heading", gyro.getAngle());
        SmartDashboard.putBoolean("Drive/Brake", brakeMode);
        SmartDashboard.putBoolean("Drive/BalanceActive", balanceModeActive);
    }

    /** Notifies the subsystem that the robot has entered Disabled mode. */
    public void notifyDisabled() {
        setBrakeMode(true);
        balanceModeActive = false;
    }

    // --- Auto-balance -----------------------------------------------------

    /**
     * Returns a command that drives onto the charging station and
     * balances using the {@link BalanceController}.
     */
    public Command autoBalanceCommand() {
        return new edu.wpi.first.wpilibj2.command.FunctionalCommand(
                () -> { balanceModeActive = true; balanceController.reset(); },
                () -> drive(balanceController.compute(gyro.getPitch(), gyro.getRoll())),
                interrupted -> { balanceModeActive = false; stop(); },
                () -> false,
                this);
    }

    /** Returns a command that follows the trajectory with the given name. */
    public Command followPathCommand(String name) {
        return new FollowPathCommand(this, new TrajectoryManager(), name,
                RobotState.INSTANCE);
    }

    /** Returns a command that drives forward to leave the community zone. */
    public Command leaveCommunityCommand() {
        return followPathCommand("LeaveCommunity");
    }

    private static void DriverStationReport(String msg) {
        edu.wpi.first.wpilibj.DriverStation.reportWarning("[Drive] " + msg, false);
    }
}
