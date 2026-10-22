// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot.RobotState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * The emergency-stop command.
 *
 * <p>Bound to the driver's Y button, this command cancels every
 * running command and forces every motor output to neutral in a
 * controlled, ordered sequence. It is the software equivalent of
 * pressing the robot's physical E-stop button: use it any time the
 * robot is behaving unexpectedly (runaway command, stuck motor,
 * tipped-over robot, jammed mechanism).
 *
 * <h2>Sequence</h2>
 *
 * The shutdown order is significant:
 *
 * <ol>
 *   <li><b>Cancel the scheduler.</b> Stop any in-flight commands so
 *       they do not fight the E-stop.</li>
 *   <li><b>Stop the shooter.</b> A spinning flywheel is the most
 *       dangerous mechanism on the robot; we cut it first.</li>
 *   <li><b>Stop the intake.</b> Rollers can grab fingers; cut them
 *       before touching anything else.</li>
 *   <li><b>Hold the arm and elevator.</b> Do not let them collapse
 *       under gravity — keep the brake engaged but stop commanded
 *       motion.</li>
 *   <li><b>Stop the drivetrain.</b> Cut drive last so the robot
 *       remains stationary while the operator inspects the
 *       situation.</li>
 *   <li><b>Engage brakes.</b> Apply motor brakes (where supported)
 *       so the robot does not coast.</li>
 *   <li><b>Notify the dashboard.</b> Set the E-stop flag so the
 *       drive team sees a clear visual indication.</li>
 * </ol>
 *
 * <h2>Recovery</h2>
 *
 * The E-stop is latched: once triggered, it stays active until the
 * driver presses the start button, which schedules a
 * recovery command that clears the latch and re-enables default
 * commands. This prevents the robot from "spontaneously recovering"
 * while the drive team is still inspecting the situation.
 */
public class EmergencyStopCommand extends Command {

    private final Drivetrain drivetrain;
    private final Arm arm;
    private final Elevator elevator;
    private final Intake intake;
    private final Shooter shooter;
    private final RobotState robotState;

    private boolean completed;

    /**
     * Constructs the E-stop command.
     *
     * @param drivetrain the drivetrain subsystem.
     * @param arm the arm subsystem.
     * @param elevator the elevator subsystem.
     * @param intake the intake subsystem.
     * @param shooter the shooter subsystem.
     */
    public EmergencyStopCommand(Drivetrain drivetrain,
                                 Arm arm,
                                 Elevator elevator,
                                 Intake intake,
                                 Shooter shooter) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.elevator = elevator;
        this.intake = intake;
        this.shooter = shooter;
        this.robotState = RobotState.INSTANCE;
        addRequirements(drivetrain, arm, elevator, intake, shooter);
    }

    @Override
    public void initialize() {
        completed = false;
        SmartDashboard.putString("EStop/State", "TRIGGERED");
        SmartDashboard.setBoolean("EStop/Active", true);
        DriverStation.reportWarning("[EStop] Triggered by driver", false);

        // 1. Cancel everything currently scheduled. This command will
        // remain scheduled because we just acquired its requirements.
        edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().cancelAll();

        // 2. Cut the shooter immediately. Spin-down takes a few
        // seconds, but we cut power now so it begins decelerating.
        shooter.emergencyStop();

        // 3. Cut the intake rollers.
        intake.emergencyStop();

        // 4. Hold the arm and elevator at their current positions.
        // Engaging the brake (where supported) is safer than letting
        // gravity pull them down.
        arm.holdCurrentPosition();
        arm.applyBrake();
        elevator.holdCurrentPosition();
        elevator.applyBrake();

        // 5. Stop the drivetrain. The drivetrain's drive() method
        // accepts zero ChassisSpeeds, but calling stop() ensures we
        // also disable the closed-loop velocity controllers.
        drivetrain.stop();
        drivetrain.applyBrake();

        // 6. Latch the E-stop state so other commands can detect it.
        robotState.setEmergencyStopActive(true);

        completed = true;
    }

    @Override
    public void execute() {
        // Continuously re-assert zero outputs. If a motor controller
        // re-enables itself (e.g., due to a brief brownout recovery),
        // we want to immediately cut it again.
        shooter.emergencyStop();
        intake.emergencyStop();
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        // The E-stop command runs forever once triggered; it must be
        // explicitly cancelled by the recovery command (which is
        // bound to the start button).
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Only clear the latch if we were interrupted (i.e., the
        // recovery command cancelled us). A natural end (which never
        // happens because isFinished returns false) would leave the
        // latch set.
        if (interrupted) {
            robotState.setEmergencyStopActive(false);
            drivetrain.releaseBrake();
            arm.releaseBrake();
            elevator.releaseBrake();
            SmartDashboard.putString("EStop/State", "RECOVERED");
            SmartDashboard.setBoolean("EStop/Active", false);
            DriverStation.reportWarning("[EStop] Recovered", false);
        }
    }

    /**
     * Static accessor used by the recovery command and by tests. We
     * piggy-back on a singleton-ish field on {@link RobotState}; the
     * field is updated by the E-stop command and read by anyone who
     * needs to know whether the robot is in E-stop.
     *
     * @return {@code true} if the E-stop is currently latched.
     */
    public static boolean isActive() {
        return RobotState.INSTANCE.isEmergencyStopActive();
    }
}
