// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.RobotConstants;
import frc.robot.robot.RobotState;
import frc.robot.subsystems.Drivetrain;

/**
 * The default teleop driving command.
 *
 * <p>This command runs continuously while the drivetrain has no other
 * command scheduled against it (i.e., whenever the driver is in manual
 * control). It reads the left and right sticks of the Xbox controller,
 * applies deadband and slew-rate limiting, and feeds the resulting
 * ChassisSpeeds to the {@link Drivetrain}.
 *
 * <h2>Input mapping</h2>
 *
 * <ul>
 *   <li>Left stick Y -&gt; forward/backward translation (m/s)</li>
 *   <li>Left stick X -&gt; lateral translation (m/s)</li>
 *   <li>Right stick X -&gt; rotational velocity (rad/s)</li>
 *   <li>Left trigger -&gt; "precision mode" -&gt; scale speeds by 0.3</li>
 *   <li>Right trigger -&gt; "boost mode" -&gt; scale speeds by 1.0 (no ramp)</li>
 * </ul>
 *
 * <h2>Field-oriented vs robot-oriented</h2>
 *
 * By default the command drives in field-oriented mode: forward on the
 * stick always moves the robot toward the opposing alliance wall,
 * regardless of which way the robot's nose is pointing. The driver can
 * toggle to robot-oriented mode (sometimes called "tank-style") by
 * pressing the start button, which is useful when the gyro has drifted
 * or when driving near a wall where field orientation is confusing.
 *
 * <h2>Deadband and slew-rate limiting</h2>
 *
 * Xbox sticks typically report a small non-zero value (~0.05) even
 * when untouched. We apply a deadband of 0.08 to suppress this noise.
 * Slew-rate limiting smooths out the driver's input so the robot does
 * not lurch when the stick is snapped to full deflection — this is
 * both gentler on the driver's stomach and reduces wheel slip.
 *
 * <p>The slew-rate limit is asymmetric: acceleration is limited more
 * aggressively than deceleration, because the robot has more traction
 * when braking than when accelerating.
 */
public class DriveCommand extends Command {

    private final Drivetrain drivetrain;
    private final CommandXboxController controller;
    private final RobotState robotState;

    private final SlewRateLimiter forwardLimiter;
    private final SlewRateLimiter strafeLimiter;
    private final SlewRateLimiter rotationLimiter;

    private boolean fieldOriented = true;
    private static final double DEADBAND = 0.08;

    /**
     * Constructs a new {@code DriveCommand} bound to the given
     * drivetrain and controller.
     *
     * @param drivetrain the drivetrain subsystem to drive.
     * @param controller the driver's Xbox controller.
     * @param robotState the shared robot state used to read the current
     *                   heading (for field-oriented mode).
     */
    public DriveCommand(Drivetrain drivetrain,
                        CommandXboxController controller,
                        RobotState robotState) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        this.robotState = robotState;
        this.forwardLimiter = new SlewRateLimiter(RobotConstants.DRIVE_SLEW_ACCEL);
        this.strafeLimiter  = new SlewRateLimiter(RobotConstants.DRIVE_SLEW_ACCEL);
        this.rotationLimiter = new SlewRateLimiter(RobotConstants.DRIVE_SLEW_ROT);
        addRequirements(drivetrain);
    }

    /**
     * Called once when the command is first scheduled. Resets the
     * slew-rate limiters so the robot does not jump if the command is
     * interrupted and then rescheduled.
     */
    @Override
    public void initialize() {
        forwardLimiter.reset(0);
        strafeLimiter.reset(0);
        rotationLimiter.reset(0);
    }

    /**
     * Called on every 20 ms tick while the command is scheduled. Reads
     * the joystick, applies deadband and slew limiting, and writes the
     * resulting ChassisSpeeds to the drivetrain.
     */
    @Override
    public void execute() {
        double rawForward = -controller.getLeftY();
        double rawStrafe  = -controller.getLeftX();
        double rawRotate  = -controller.getRightX();

        // Apply deadband.
        double forward = applyDeadband(rawForward, DEADBAND);
        double strafe  = applyDeadband(rawStrafe, DEADBAND);
        double rotate  = applyDeadband(rawRotate, DEADBAND);

        // Precision mode: scale by 0.3 if left trigger is held.
        double triggerLeft = controller.getLeftTriggerAxis();
        double scale = 1.0 - 0.7 * triggerLeft;

        // Square inputs for finer low-speed control. The sign is
        // preserved so that direction is unaffected.
        forward = Math.copySign(forward * forward, forward) * scale;
        strafe  = Math.copySign(strafe * strafe, strafe) * scale;
        rotate  = Math.copySign(rotate * rotate, rotate) * scale;

        // Convert joystick units to physical units.
        double maxSpeed = RobotConstants.MAX_DRIVE_SPEED_MPS;
        double maxRot   = RobotConstants.MAX_ANGULAR_SPEED_RAD_PER_S;
        double vx = forward * maxSpeed;
        double vy = strafe  * maxSpeed;
        double omega = rotate * maxRot;

        // Slew-rate limit.
        vx    = forwardLimiter.calculate(vx);
        vy    = strafeLimiter.calculate(vy);
        omega = rotationLimiter.calculate(omega);

        // Build the chassis speeds. In field-oriented mode the speeds
        // are expressed relative to the field frame; in robot-oriented
        // mode they are relative to the robot frame.
        ChassisSpeeds speeds;
        if (fieldOriented) {
            Rotation2d heading = robotState.getHeading();
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, heading);
        } else {
            speeds = new ChassisSpeeds(vx, vy, omega);
        }

        drivetrain.drive(speeds);

        // Update diagnostic data on the dashboard.
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Drive/FieldOriented", fieldOriented);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Drive/Vx", vx);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Drive/Vy", vy);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Drive/Omega", omega);
    }

    /** Toggles between field- and robot-oriented mode. Bound to the start button. */
    public void toggleFieldOriented() {
        fieldOriented = !fieldOriented;
    }

    /**
     * Drive commands never finish on their own — they run until
     * interrupted by another command that requires the drivetrain.
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Called when the command is interrupted. Stops the drivetrain so
     * the robot does not coast on the previous velocity.
     */
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    /**
     * Applies a deadband to a joystick input. Values within the
     * deadband are zeroed; values outside are remapped to the
     * remaining range so the deadband does not introduce a "step" at
     * its edge.
     *
     * @param value the raw joystick value, in [-1, 1].
     * @param deadband the deadband threshold.
     * @return the deadbanded value, in [-1, 1].
     */
    private static double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        double sign = Math.signum(value);
        double remapped = (Math.abs(value) - deadband) / (1.0 - deadband);
        return MathUtil.clamp(sign * remapped, -1.0, 1.0);
    }
}
