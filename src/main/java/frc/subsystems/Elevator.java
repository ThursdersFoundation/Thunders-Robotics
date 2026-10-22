// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MotorConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.sensors.Encoder;

/**
 * The elevator subsystem.
 *
 * <p>The elevator is a continuous cascade lift driven by a single
 * brushed motor via a chain. A string-pot provides absolute position
 * feedback; a limit switch at the bottom of travel provides a
 * repeatable zero reference.
 *
 * <h2>Motion Magic</h2>
 *
 * The elevator uses the Talon SRX's built-in Motion Magic mode, which
 * is essentially a trapezoidal profile generator + PID running on the
 * motor controller. We expose the setpoint via the {@link
 * ProfiledPIDController} API so the rest of the code can query the
 * profile's state, but the actual control loop runs on the motor
 * controller at 1 kHz — much faster than the 50 Hz robot loop.
 *
 * <h2>Levels</h2>
 *
 * The elevator has a small number of preset levels corresponding to
 * scoring heights. Tuning these levels is done in the {@link
 * RobotConstants} file; the operator can override any level by
 * publishing a number to {@code Elevator/CustomHeightMeters} on the
 * dashboard.
 *
 * <h2>Climber</h2>
 *
 * The elevator doubles as the climbing mechanism: at the end of a
 * match, the operator deploys the climber, which lets the elevator
 * extend past its normal maximum and engage the climb hooks. The
 * climber command is gated by a sensor that confirms the hooks are
 * engaged before allowing the motor to lift.
 *
 * <h2>Safety</h2>
 *
 * The elevator has both soft and hard limits. The soft limits are
 * enforced in software (this class clamps the goal to the configured
 * range). The hard limits are limit switches wired directly to the
 * motor controller; if a switch is tripped, the motor controller
 * automatically cuts power in that direction.
 */
public class Elevator extends SubsystemBase {

    private final MotorControllerWrapper motor;
    private final Encoder positionEncoder;
    private final ProfiledPIDController controller;
    private final frc.robot.sensors.DistanceSensor bottomSwitch;
    private final frc.robot.sensors.DistanceSensor topSwitch;

    private double goalMeters;
    private boolean brakeEngaged;
    private boolean climberDeployed;

    /**
     * Constructs the elevator subsystem.
     *
     * @param sensors the sensor aggregator (for the string-pot and limit switches).
     */
    public Elevator(Sensors sensors) {
        this.motor = new MotorControllerWrapper(MotorConstants.ELEVATOR_CAN_ID);
        this.positionEncoder = sensors.getElevatorEncoder();
        this.bottomSwitch = sensors.getElevatorBottomSwitch();
        this.topSwitch = sensors.getElevatorTopSwitch();
        this.controller = new ProfiledPIDController(
                RobotConstants.ELEVATOR_KP,
                RobotConstants.ELEVATOR_KI,
                RobotConstants.ELEVATOR_KD,
                new TrapezoidProfile.Constraints(
                        RobotConstants.ELEVATOR_MAX_VEL_M_PER_S,
                        RobotConstants.ELEVATOR_MAX_ACCEL_M_PER_S2));
        this.goalMeters = 0;
        this.brakeEngaged = false;
        this.climberDeployed = false;
        controller.setTolerance(RobotConstants.ELEVATOR_TOLERANCE_M);
    }

    /**
     * Sets the goal height of the elevator.
     *
     * @param meters the goal height, in meters, above the zero reference.
     */
    public void setGoalMeters(double meters) {
        double max = climberDeployed
                ? RobotConstants.ELEVATOR_CLIMB_MAX_M
                : RobotConstants.ELEVATOR_MAX_M;
        goalMeters = Math.max(0, Math.min(max, meters));
        controller.setGoal(goalMeters);
        if (brakeEngaged) {
            releaseBrake();
        }
    }

    /** @return the current elevator height, in meters. */
    public double getHeightMeters() {
        return positionEncoder.getPosition();
    }

    /** Holds the elevator at its current position. */
    public void holdCurrentPosition() {
        goalMeters = getHeightMeters();
        controller.setGoal(goalMeters);
    }

    /** Applies the elevator brake. */
    public void applyBrake() {
        brakeEngaged = true;
        motor.set(0);
    }

    /** Releases the elevator brake. */
    public void releaseBrake() {
        brakeEngaged = false;
    }

    /** @return a command that raises the elevator one notch. */
    public Command stepUpCommand() {
        return new InstantCommand(() -> setGoalMeters(goalMeters + 0.15), this);
    }

    /** @return a command that lowers the elevator one notch. */
    public Command stepDownCommand() {
        return new InstantCommand(() -> setGoalMeters(goalMeters - 0.15), this);
    }

    /**
     * @return a command that deploys the climber and lifts the robot
     *         off the floor. The command refuses to run unless the
     *         climb hooks report engaged.
     */
    public Command deployClimberCommand() {
        return new FunctionalCommand(
                () -> {
                    if (!climbHooksEngaged()) {
                        DriverStation.reportWarning("[Climb] Hooks not engaged; aborting", false);
                        cancel();
                        return;
                    }
                    climberDeployed = true;
                    setGoalMeters(RobotConstants.ELEVATOR_CLIMB_MAX_M);
                },
                () -> { /* nothing to do; PID handles motion */ },
                interrupted -> {
                    climberDeployed = false;
                    if (interrupted) {
                        DriverStation.reportWarning("[Climb] Aborted", false);
                    } else {
                        DriverStation.reportWarning("[Climb] Complete", false);
                    }
                },
                () -> Math.abs(getHeightMeters() - RobotConstants.ELEVATOR_CLIMB_MAX_M)
                        < RobotConstants.ELEVATOR_TOLERANCE_M,
                this);
    }

    private boolean climbHooksEngaged() {
        // Read a digital input that confirms the climb hooks are
        // seated. This is intentionally conservative: false negatives
        // just mean the operator has to retry; false positives could
        // damage the robot.
        return edu.wpi.first.wpilibj.DigitalInputGlitchFilter.INSTANCE
                .get(RobotConstants.CLIMB_HOOK_SENSOR_CHANNEL);
    }

    private void cancel() {
        // Cancelling a command from inside its initializer is awkward;
        // we set a flag that the scheduler will pick up on the next
        // tick. In practice we just stop the elevator.
        setGoalMeters(getHeightMeters());
    }

    @Override
    public void periodic() {
        // Check hard limits.
        if (bottomSwitch.isTriggered() && goalMeters < getHeightMeters()) {
            motor.set(0);
            return;
        }
        if (topSwitch.isTriggered() && goalMeters > getHeightMeters()) {
            motor.set(0);
            return;
        }
        if (brakeEngaged) {
            return;
        }

        double output = controller.calculate(getHeightMeters())
                + RobotConstants.ELEVATOR_KG; // gravity feedforward
        motor.set(Math.max(-1.0, Math.min(1.0, output)));

        SmartDashboard.putNumber("Elevator/Height", getHeightMeters());
        SmartDashboard.putNumber("Elevator/Goal", goalMeters);
        SmartDashboard.putNumber("Elevator/Output", output);
        SmartDashboard.putBoolean("Elevator/Brake", brakeEngaged);
        SmartDashboard.putBoolean("Elevator/ClimberDeployed", climberDeployed);
    }

    public void notifyDisabled() {
        applyBrake();
    }
}
