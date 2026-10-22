// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autonomous.AutoRoutine;
import frc.robot.autonomous.PathPlanner;
import frc.robot.autonomous.TrajectoryManager;
import frc.robot.constants.RobotConstants;
import frc.robot.robot.RobotState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

/**
 * The top-level command that runs an entire autonomous routine.
 *
 * <p>Each autonomous routine is a sequence of: scoring preloaded game
 * pieces, following trajectories to acquire more pieces, and balancing
 * on the charging station. Building these sequences inline in the
 * {@link frc.robot.robot.RobotContainer} would produce a tangled mess
 * of {@code .andThen(...)} chains; instead, this command encapsulates
 * the routine's lifecycle and delegates the actual work to subordinate
 * commands (e.g., {@link FollowPathCommand}).
 *
 * <h2>State machine</h2>
 *
 * The command implements a simple state machine:
 *
 * <pre>
 *   INIT -&gt; SCORE_PRELOAD -&gt; ACQUIRE_PIECES -&gt; SCORE_ACQUIRED
 *                                         \-&gt; BALANCE
 *                                         \-&gt; DONE
 * </pre>
 *
 * Each state runs a sub-command; when the sub-command finishes, the
 * state machine advances to the next state. If any sub-command throws,
 * the routine aborts and the robot sits still for the remainder of the
 * auto period.
 *
 * <h2>Time budget</h2>
 *
 * The autonomous period is 15 seconds. This command tracks elapsed
 * time and, if the routine is running behind, will skip the balance
 * phase (which is the riskiest) to ensure the robot is parked before
 * the teleop period starts.
 */
public class AutonomousCommand extends Command {

    /** States of the autonomous state machine. */
    public enum Phase {
        INIT,
        SCORE_PRELOAD,
        ACQUIRE_PIECES,
        SCORE_ACQUIRED,
        BALANCE,
        DONE
    }

    private final AutoRoutine routine;
    private final Drivetrain drivetrain;
    private final Shooter shooter;
    private final Intake intake;
    private final RobotState robotState;
    private final PathPlanner pathPlanner;
    private final TrajectoryManager trajectoryManager;

    private Phase currentPhase = Phase.INIT;
    private Command currentSubCommand;
    private double startTimeSeconds;
    private int piecesScored;

    /** Maximum time in seconds to spend on the balance phase. */
    private static final double BALANCE_TIME_BUDGET = 5.0;

    /**
     * Constructs the autonomous command.
     *
     * @param routine the routine selected on the dashboard.
     * @param drivetrain the drivetrain subsystem.
     * @param shooter the shooter subsystem.
     * @param intake the intake subsystem.
     * @param robotState the shared robot state.
     */
    public AutonomousCommand(AutoRoutine routine,
                              Drivetrain drivetrain,
                              Shooter shooter,
                              Intake intake,
                              RobotState robotState) {
        this.routine = routine;
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.intake = intake;
        this.robotState = robotState;
        this.pathPlanner = new PathPlanner();
        this.trajectoryManager = new TrajectoryManager();
        addRequirements(drivetrain, shooter, intake);
    }

    @Override
    public void initialize() {
        startTimeSeconds = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        currentPhase = Phase.INIT;
        piecesScored = 0;
        robotState.setMode(RobotState.Mode.AUTONOMOUS);
        advancePhase();
    }

    /**
     * Advances to the next phase of the routine. Constructs the
     * sub-command for the new phase and schedules it.
     */
    private void advancePhase() {
        if (currentSubCommand != null) {
            currentSubCommand.cancel();
            currentSubCommand = null;
        }

        switch (routine) {
            case NONE:
                currentPhase = Phase.DONE;
                return;
            case LEAVE_COMMUNITY:
                transitionTo(Phase.ACQUIRE_PIECES);
                scheduleLeaveCommunity();
                break;
            case SCORE_AND_LEAVE:
                if (currentPhase == Phase.INIT) {
                    transitionTo(Phase.SCORE_PRELOAD);
                    scheduleScorePreload();
                } else if (currentPhase == Phase.SCORE_PRELOAD) {
                    transitionTo(Phase.ACQUIRE_PIECES);
                    scheduleLeaveCommunity();
                } else {
                    finishRoutine();
                }
                break;
            case SCORE_TWO_BALANCE:
                handleScoreTwoBalance();
                break;
            case MAX_MOBILITY:
                handleMaxMobility();
                break;
            default:
                finishRoutine();
        }
    }

    private void handleScoreTwoBalance() {
        if (currentPhase == Phase.INIT) {
            transitionTo(Phase.SCORE_PRELOAD);
            scheduleScorePreload();
        } else if (currentPhase == Phase.SCORE_PRELOAD) {
            transitionTo(Phase.ACQUIRE_PIECES);
            scheduleAcquireAndScore();
        } else if (currentPhase == Phase.ACQUIRE_PIECES) {
            double elapsed = elapsedSeconds();
            if (elapsed < (15.0 - BALANCE_TIME_BUDGET)) {
                transitionTo(Phase.BALANCE);
                scheduleBalance();
            } else {
                finishRoutine();
            }
        } else {
            finishRoutine();
        }
    }

    private void handleMaxMobility() {
        if (currentPhase == Phase.INIT) {
            transitionTo(Phase.SCORE_PRELOAD);
            scheduleScorePreload();
        } else if (currentPhase == Phase.SCORE_PRELOAD) {
            transitionTo(Phase.ACQUIRE_PIECES);
            scheduleAcquireAndScore();
        } else {
            finishRoutine();
        }
    }

    private void transitionTo(Phase next) {
        currentPhase = next;
    }

    private void finishRoutine() {
        currentPhase = Phase.DONE;
        if (currentSubCommand != null) {
            currentSubCommand.cancel();
        }
    }

    // --- Sub-command scheduling helpers -----------------------------------

    private void scheduleScorePreload() {
        currentSubCommand = shooter.scorePreloadCommand();
        currentSubCommand.schedule();
    }

    private void scheduleLeaveCommunity() {
        currentSubCommand = new FollowPathCommand(
                drivetrain, trajectoryManager, "LeaveCommunity", robotState);
        currentSubCommand.schedule();
    }

    private void scheduleAcquireAndScore() {
        currentSubCommand = new FollowPathCommand(
                drivetrain, trajectoryManager, "AcquireAndScore", robotState);
        currentSubCommand.schedule();
    }

    private void scheduleBalance() {
        currentSubCommand = drivetrain.autoBalanceCommand();
        currentSubCommand.schedule();
    }

    @Override
    public void execute() {
        if (currentPhase == Phase.DONE) {
            return;
        }
        if (currentSubCommand != null && !currentSubCommand.isScheduled()) {
            piecesScored++;
            advancePhase();
        }
        publishTelemetry();
    }

    @Override
    public boolean isFinished() {
        return currentPhase == Phase.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        if (currentSubCommand != null) {
            currentSubCommand.cancel();
        }
        drivetrain.stop();
        if (interrupted) {
            DriverStation.reportWarning("[Auto] Routine interrupted at phase " + currentPhase, false);
        } else {
            DriverStation.reportWarning("[Auto] Routine complete. Pieces scored: " + piecesScored, false);
        }
    }

    private double elapsedSeconds() {
        return edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTimeSeconds;
    }

    private void publishTelemetry() {
        SmartDashboard.putString("Auto/Phase", currentPhase.name());
        SmartDashboard.putNumber("Auto/Elapsed", elapsedSeconds());
        SmartDashboard.putNumber("Auto/PiecesScored", piecesScored);
    }
}
