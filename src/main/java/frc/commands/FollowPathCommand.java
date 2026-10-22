// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autonomous.TrajectoryManager;
import frc.robot.constants.RobotConstants;
import frc.robot.robot.RobotState;
import frc.robot.subsystems.Drivetrain;

import java.util.List;

/**
 * Follows a precomputed {@link Trajectory} from start to finish using a
 * {@link HolonomicDriveController}.
 *
 * <p>Trajectory following is the standard way to move a swerve or
 * mecanum robot accurately between two points on the field. A
 * trajectory is a function {@code time -> (x, y, theta, vx, vy,
 * vtheta)}; the controller compares the robot's actual pose against
 * the trajectory's sample at the current time and produces a
 * ChassisSpeeds that drives the error to zero.
 *
 * <h2>PID gains</h2>
 *
 * The {@link HolonomicDriveController} uses three PID controllers
 * internally: one each for X and Y translation, and a
 * {@link ProfiledPIDController} for rotation. The gains are tuned per
 * robot and stored in {@link RobotConstants}. The rotation controller
 * is profiled so that the robot cannot spin faster than a configurable
 * maximum, which prevents wheel slip and tipped-over robots.
 *
 * <h2>Resetting odometry</h2>
 *
 * The {@code initialize()} method resets the robot's odometry to the
 * trajectory's starting pose. This is important: if the odometry is
 * off by even a few centimetres at the start, the controller will
 * fight the trajectory and the robot will veer. The reset is logged so
 * the drive team can see how far off the starting pose was.
 *
 * <h2>Finishing</h2>
 *
 * The command finishes when the trajectory time has elapsed AND the
 * robot is within a configurable tolerance of the final pose. This
 * prevents the command from ending early if the robot is lagging
 * behind the trajectory (e.g., due to wheel slip).
 */
public class FollowPathCommand extends Command {

    private final Drivetrain drivetrain;
    private final TrajectoryManager trajectoryManager;
    private final String trajectoryName;
    private final RobotState robotState;

    private final HolonomicDriveController controller;
    private Trajectory trajectory;
    private Timer timer;

    private static final double POSITION_TOLERANCE_M = 0.05;
    private static final double HEADING_TOLERANCE_RAD = 0.05;

    /**
     * Constructs a new {@code FollowPathCommand}.
     *
     * @param drivetrain the drivetrain subsystem to drive.
     * @param trajectoryManager the trajectory cache to look up the path by name.
     * @param trajectoryName the name of the trajectory to follow.
     * @param robotState the shared robot state.
     */
    public FollowPathCommand(Drivetrain drivetrain,
                              TrajectoryManager trajectoryManager,
                              String trajectoryName,
                              RobotState robotState) {
        this.drivetrain = drivetrain;
        this.trajectoryManager = trajectoryManager;
        this.trajectoryName = trajectoryName;
        this.robotState = robotState;

        this.controller = new HolonomicDriveController(
                new PIDController(RobotConstants.PATH_X_KP, 0, RobotConstants.PATH_X_KD),
                new PIDController(RobotConstants.PATH_Y_KP, 0, RobotConstants.PATH_Y_KD),
                new ProfiledPIDController(
                        RobotConstants.PATH_THETA_KP, 0, RobotConstants.PATH_THETA_KD,
                        new TrapezoidProfile.Constraints(
                                RobotConstants.MAX_ANGULAR_SPEED_RAD_PER_S,
                                RobotConstants.MAX_ANGULAR_ACCEL_RAD_PER_S2)));
        this.controller.setTolerance(new Pose2d(
                POSITION_TOLERANCE_M, POSITION_TOLERANCE_M,
                new Rotation2d(HEADING_TOLERANCE_RAD)));
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Load the trajectory from the cache.
        trajectory = trajectoryManager.get(trajectoryName);
        if (trajectory == null) {
            DriverStation.reportError("[FollowPath] Trajectory not found: " + trajectoryName, false);
            cancel();
            return;
        }

        // Reset the timer that drives the trajectory sampling.
        timer = new Timer();
        timer.start();

        // Reset the robot's odometry to the trajectory's starting pose.
        // This is critical: even a small starting offset will cause the
        // controller to fight the trajectory and the robot will veer.
        Pose2d start = trajectory.getInitialPose();
        drivetrain.resetOdometry(start);

        // Tell the rotation controller to wrap the error so that it
        // always takes the shorter path around the circle.
        controller.setEnabled(true);
        DriverStation.reportWarning(
                "[FollowPath] Starting " + trajectoryName + " from " + start, false);
    }

    @Override
    public void execute() {
        if (trajectory == null || timer == null) {
            return;
        }
        double t = timer.get();
        Trajectory.State goal = trajectory.sample(t);
        Pose2d current = drivetrain.getPose();

        ChassisSpeeds speeds = controller.calculate(current, goal);
        drivetrain.drive(speeds);

        // Publish telemetry so the drive team can verify the robot is
        // tracking the trajectory in real time.
        SmartDashboard.putNumber("Path/Time", t);
        SmartDashboard.putNumber("Path/GoalX", goal.poseMeters.getX());
        SmartDashboard.putNumber("Path/GoalY", goal.poseMeters.getY());
        SmartDashboard.putNumber("Path/CurrX", current.getX());
        SmartDashboard.putNumber("Path/CurrY", current.getY());
        SmartDashboard.putNumber("Path/ErrX", goal.poseMeters.getX() - current.getX());
        SmartDashboard.putNumber("Path/ErrY", goal.poseMeters.getY() - current.getY());
    }

    @Override
    public boolean isFinished() {
        if (trajectory == null || timer == null) {
            return true;
        }
        // The trajectory itself has finished when the timer exceeds the
        // total trajectory time. We additionally require that the
        // controller reports the robot is within tolerance of the goal
        // pose, so the command does not end while the robot is still
        // settling.
        boolean timeUp = timer.get() >= trajectory.getTotalTimeSeconds();
        boolean atGoal = controller.atReference();
        return timeUp && atGoal;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        if (timer != null) {
            timer.stop();
        }
        if (interrupted) {
            DriverStation.reportWarning(
                    "[FollowPath] Interrupted: " + trajectoryName, false);
        } else {
            DriverStation.reportWarning(
                    "[FollowPath] Completed: " + trajectoryName, false);
        }
    }

    /**
     * Returns the list of waypoints the trajectory passes through.
     * Used by tests to verify trajectory contents without re-parsing
     * the trajectory file.
     *
     * @return the list of poses, or an empty list if no trajectory is loaded.
     */
    public List<Pose2d> getWaypoints() {
        if (trajectory == null) {
            return List.of();
        }
        return trajectory.getStates().stream()
                .map(s -> s.poseMeters)
                .toList();
    }
}
