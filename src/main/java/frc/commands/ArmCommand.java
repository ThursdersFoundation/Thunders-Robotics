// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Arm;

/**
 * Moves the arm to a specified setpoint using the arm's internal
 * closed-loop controller.
 *
 * <p>This command is the primary way the operator moves the arm. It is
 * constructed with a {@link Setpoint} enum value (e.g., STOW, INTAKE,
 * SPEAKER) which is mapped to physical joint angles via {@link
 * RobotConstants}. The command finishes when both joints report they
 * are within tolerance of the goal, OR when a configurable timeout
 * expires — whichever comes first.
 *
 * <h2>Why a Setpoint enum?</h2>
 *
 * Hardcoding raw angle values throughout the codebase makes tuning
 * painful: a one-degree change requires editing every call site.
 * Centralising the setpoints in an enum means tuning is done in a
 * single file and call sites are self-documenting:
 *
 * <pre>{@code
 * new ArmCommand(arm, ArmCommand.Setpoint.SPEAKER).schedule();
 * }</pre>
 *
 * <h2>Feedforward</h2>
 *
 * The arm uses a feedforward + PID controller. The feedforward
 * compensates for gravity (which varies with joint angle) so the PID
 * controller only needs to handle the dynamic error, not the static
 * load. This dramatically reduces steady-state error and oscillation.
 *
 * <h2>Default command</h2>
 *
 * The arm subsystem uses this command as its default, with the STOW
 * setpoint. This means the arm automatically returns to the stowed
 * position whenever no other arm command is scheduled, which keeps the
 * robot in a legal configuration for transport.
 */
public class ArmCommand extends Command {

    /** High-level arm positions. Mapped to joint angles via {@link RobotConstants}. */
    public enum Setpoint {
        /** Arm folded flat against the frame, ready for transport or climb. */
        STOW,
        /** Arm extended low to the ground for intaking game pieces. */
        INTAKE,
        /** Arm positioned to score in the speaker from the podium. */
        SPEAKER_PODIUM,
        /** Arm positioned to score in the speaker from the subwoofer. */
        SPEAKER_SUB,
        /** Arm positioned to score in the amp. */
        AMP,
        /** Arm raised as high as possible for climbing or trap scoring. */
        CLIMB,
        /** Custom setpoint provided at runtime via the dashboard. */
        CUSTOM
    }

    private final Arm arm;
    private final Setpoint setpoint;
    private double startTimeSeconds;
    private double shoulderGoal;
    private double elbowGoal;

    /**
     * Constructs a new {@code ArmCommand} that moves the arm to the
     * specified setpoint.
     *
     * @param arm the arm subsystem.
     * @param setpoint the target position.
     */
    public ArmCommand(Arm arm, Setpoint setpoint) {
        this.arm = arm;
        this.setpoint = setpoint;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        startTimeSeconds = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        // Resolve the setpoint to physical joint angles.
        switch (setpoint) {
            case STOW:
                shoulderGoal = RobotConstants.ARM_SHOULDER_STOW_DEG;
                elbowGoal    = RobotConstants.ARM_ELBOW_STOW_DEG;
                break;
            case INTAKE:
                shoulderGoal = RobotConstants.ARM_SHOULDER_INTAKE_DEG;
                elbowGoal    = RobotConstants.ARM_ELBOW_INTAKE_DEG;
                break;
            case SPEAKER_PODIUM:
                shoulderGoal = RobotConstants.ARM_SHOULDER_SPEAKER_PODIUM_DEG;
                elbowGoal    = RobotConstants.ARM_ELBOW_SPEAKER_PODIUM_DEG;
                break;
            case SPEAKER_SUB:
                shoulderGoal = RobotConstants.ARM_SHOULDER_SPEAKER_SUB_DEG;
                elbowGoal    = RobotConstants.ARM_ELBOW_SPEAKER_SUB_DEG;
                break;
            case AMP:
                shoulderGoal = RobotConstants.ARM_SHOULDER_AMP_DEG;
                elbowGoal    = RobotConstants.ARM_ELBOW_AMP_DEG;
                break;
            case CLIMB:
                shoulderGoal = RobotConstants.ARM_SHOULDER_CLIMB_DEG;
                elbowGoal    = RobotConstants.ARM_ELBOW_CLIMB_DEG;
                break;
            case CUSTOM:
                shoulderGoal = SmartDashboard.getNumber("Arm/CustomShoulderDeg",
                        RobotConstants.ARM_SHOULDER_STOW_DEG);
                elbowGoal = SmartDashboard.getNumber("Arm/CustomElbowDeg",
                        RobotConstants.ARM_ELBOW_STOW_DEG);
                break;
            default:
                shoulderGoal = RobotConstants.ARM_SHOULDER_STOW_DEG;
                elbowGoal    = RobotConstants.ARM_ELBOW_STOW_DEG;
        }

        arm.setShoulderGoalDegrees(shoulderGoal);
        arm.setElbowGoalDegrees(elbowGoal);
        SmartDashboard.putString("Arm/Setpoint", setpoint.name());
        SmartDashboard.putNumber("Arm/ShoulderGoal", shoulderGoal);
        SmartDashboard.putNumber("Arm/ElbowGoal", elbowGoal);
    }

    @Override
    public void execute() {
        // The arm subsystem handles the closed-loop control in its own
        // periodic(); we only need to check whether we have arrived.
        boolean shoulderAtGoal = Math.abs(arm.getShoulderAngleDegrees() - shoulderGoal)
                < RobotConstants.ARM_SHOULDER_TOLERANCE_DEG;
        boolean elbowAtGoal = Math.abs(arm.getElbowAngleDegrees() - elbowGoal)
                < RobotConstants.ARM_ELBOW_TOLERANCE_DEG;

        SmartDashboard.putBoolean("Arm/ShoulderAtGoal", shoulderAtGoal);
        SmartDashboard.putBoolean("Arm/ElbowAtGoal", elbowAtGoal);
        SmartDashboard.putNumber("Arm/ShoulderErr",
                shoulderGoal - arm.getShoulderAngleDegrees());
        SmartDashboard.putNumber("Arm/ElbowErr",
                elbowGoal - arm.getElbowAngleDegrees());
    }

    @Override
    public boolean isFinished() {
        boolean shoulderAtGoal = Math.abs(arm.getShoulderAngleDegrees() - shoulderGoal)
                < RobotConstants.ARM_SHOULDER_TOLERANCE_DEG;
        boolean elbowAtGoal = Math.abs(arm.getElbowAngleDegrees() - elbowGoal)
                < RobotConstants.ARM_ELBOW_TOLERANCE_DEG;
        double elapsed = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTimeSeconds;
        return (shoulderAtGoal && elbowAtGoal)
                || elapsed > RobotConstants.ARM_MOVE_TIMEOUT_SECONDS;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            // Hold the current goal so the arm does not droop under
            // gravity when the operator releases the button.
            arm.holdCurrentPosition();
            SmartDashboard.putString("Arm/State", "INTERRUPTED_HOLD");
        } else {
            SmartDashboard.putString("Arm/State", "AT_SETPOINT");
        }
    }
}
