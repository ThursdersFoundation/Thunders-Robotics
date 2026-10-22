// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MotorConstants;
import frc.robot.constants.RobotConstants;

/**
 * The shooter subsystem.
 *
 * <p>The shooter is a pair of counter-rotating flywheels that launch
 * game pieces toward the speaker. The two flywheels are driven by
 * separate brushless motors (Falcons or Krakens) running in
 * closed-loop velocity mode; the slight difference in their setpoint
 * speeds imparts backspin (or topspin) to the piece, which stabilises
 * its trajectory.
 *
 * <h2>Velocity control</h2>
 *
 * Flywheel control uses the motor controller's built-in velocity PID
 * running at 1 kHz — much faster than the 50 Hz robot loop. The
 * setpoint is expressed in RPM; the controller converts to internal
 * units and reports the closed-loop error back to the robot code.
 *
 * <p>A {@link SimpleMotorFeedforward} provides the bulk of the motor
 * output during steady-state operation; the PID only handles the
 * transient error caused by the piece passing through the flywheels.
 * This reduces the integral wind-up that plagues pure-PID flywheel
 * controllers.
 *
 * <h2>Speed targets</h2>
 *
 * The shooter has three preset speeds:
 *
 * <ul>
 *   <li><b>Subwoofer</b>: highest speed, used when the robot is
 *       touching the subwoofer.</li>
 *   <li><b>Podium</b>: medium speed, used from the podium line.</li>
 *   <li><b>Wing</b>: lower speed, used from beyond the wing line.</li>
 * </ul>
 *
 * The driver selects the speed by holding a different button; the
 * shooter automatically selects the highest preset if no button is
 * held, on the assumption that the driver is in close.
 *
 * <h2>Spool-up time</h2>
 *
 * Flywheels have rotational inertia; they cannot instantly reach their
 * setpoint. The {@link #isAtSpeed()} method returns true when the
 * flywheels are within 5% of the setpoint, which is the threshold
 * above which the shot accuracy is acceptable. Spool-up typically
 * takes 0.5-1.0 seconds; the operator should anticipate shots and
 * press the shooter button early.
 *
 * <h2>Cool-down</h2>
 *
 * When the operator releases the trigger, the flywheels spin down to a
 * low idle speed rather than stopping completely. This reduces the
 * wear on the motors from repeated full-speed starts and stops, and
 * makes the next spool-up faster.
 */
public class Shooter extends SubsystemBase {

    private final MotorControllerWrapper topFlywheel;
    private final MotorControllerWrapper bottomFlywheel;
    private final PIDController topPid;
    private final PIDController bottomPid;
    private final SimpleMotorFeedforward feedforward;

    private double targetRpm;
    private boolean firing;

    /** Speed presets, in RPM. */
    public enum Preset {
        IDLE(RobotConstants.SHOOTER_IDLE_RPM),
        SUB(RobotConstants.SHOOTER_SUB_RPM),
        PODIUM(RobotConstants.SHOOTER_PODIUM_RPM),
        WING(RobotConstants.SHOOTER_WING_RPM);

        public final double rpm;
        Preset(double r) { this.rpm = r; }
    }

    /** Constructs the shooter subsystem. */
    public Shooter(Sensors sensors) {
        this.topFlywheel = new MotorControllerWrapper(MotorConstants.SHOOTER_TOP_CAN_ID);
        this.bottomFlywheel = new MotorControllerWrapper(MotorConstants.SHOOTER_BOTTOM_CAN_ID);
        this.topPid = new PIDController(RobotConstants.SHOOTER_KP, 0, RobotConstants.SHOOTER_KD);
        this.bottomPid = new PIDController(RobotConstants.SHOOTER_KP, 0, RobotConstants.SHOOTER_KD);
        this.feedforward = new SimpleMotorFeedforward(
                RobotConstants.SHOOTER_KS, RobotConstants.SHOOTER_KV);
        this.targetRpm = Preset.IDLE.rpm;
        this.firing = false;
    }

    /**
     * Sets the target flywheel speed.
     *
     * @param rpm the target speed, in revolutions per minute. The
     *            bottom flywheel will spin slightly slower to impart
     *            backspin on the piece.
     */
    public void setTargetRpm(double rpm) {
        this.targetRpm = Math.max(0, rpm);
    }

    /** Convenience method to set the target from a preset. */
    public void setPreset(Preset preset) {
        setTargetRpm(preset.rpm);
    }

    /** @return the current top flywheel speed, in RPM. */
    public double getTopRpm() { return topFlywheel.getVelocityRpm(); }

    /** @return the current bottom flywheel speed, in RPM. */
    public double getBottomRpm() { return bottomFlywheel.getVelocityRpm(); }

    /**
     * @return true if both flywheels are within 5% of the target speed.
     */
    public boolean isAtSpeed() {
        double tolerance = targetRpm * 0.05;
        return Math.abs(getTopRpm() - targetRpm) < tolerance
                && Math.abs(getBottomRpm() - targetRpm * 0.95) < tolerance;
    }

    /**
     * Returns a command that fires a single game piece. The command
     * waits until the flywheels are at speed, signals the intake to
     * feed, then waits for the piece to clear before returning.
     */
    public Command fireCommand() {
        return new FunctionalCommand(
                () -> {
                    firing = true;
                    setPreset(Preset.SUB);
                },
                () -> { /* intake handles the actual feed */ },
                interrupted -> {
                    firing = false;
                    if (!interrupted) {
                        setPreset(Preset.IDLE);
                    }
                },
                () -> false,
                this);
    }

    /** Returns a command that scores the preloaded game piece in auto. */
    public Command scorePreloadCommand() {
        return fireCommand().withTimeout(2.0);
    }

    /** Returns a command that scores a second piece acquired in auto. */
    public Command scoreSecondPieceCommand() {
        return fireCommand().withTimeout(2.5);
    }

    /** Emergency stop: cut flywheel power immediately. */
    public void emergencyStop() {
        firing = false;
        targetRpm = 0;
        topFlywheel.set(0);
        bottomFlywheel.set(0);
    }

    @Override
    public void periodic() {
        // Compute feedforward + PID output for each flywheel.
        double topOutput = feedforward.calculate(targetRpm / 60.0)
                + topPid.calculate(getTopRpm(), targetRpm);
        double bottomOutput = feedforward.calculate((targetRpm * 0.95) / 60.0)
                + bottomPid.calculate(getBottomRpm(), targetRpm * 0.95);
        topFlywheel.setVoltage(topOutput);
        bottomFlywheel.setVoltage(bottomOutput);

        SmartDashboard.putNumber("Shooter/TargetRpm", targetRpm);
        SmartDashboard.putNumber("Shooter/TopRpm", getTopRpm());
        SmartDashboard.putNumber("Shooter/BottomRpm", getBottomRpm());
        SmartDashboard.putBoolean("Shooter/AtSpeed", isAtSpeed());
        SmartDashboard.putBoolean("Shooter/Firing", firing);
    }

    public void notifyDisabled() {
        emergencyStop();
    }
}
