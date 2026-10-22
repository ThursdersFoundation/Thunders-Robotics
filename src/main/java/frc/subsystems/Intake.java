// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MotorConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Shooter;

/**
 * The intake subsystem.
 *
 * <p>The intake is a pneumatically-deployed set of rollers that
 * collects game pieces from the floor and feeds them to the shooter.
 * A beam-break sensor at the throat of the intake detects when a
 * piece is seated; the {@link frc.robot.commands.IntakeCommand} uses
 * this signal to slow the rollers to a "hold" speed and stop intaking.
 *
 * <h2>Deployment</h2>
 *
 * The intake is held in a stowed position by a single-acting pneumatic
 * cylinder (spring-extended, air-retracted). To deploy, we open a
 * solenoid that supplies air to the retract side; the cylinder
 * extends and the intake drops to the floor. To stow, we close the
 * solenoid; the spring retracts the cylinder and the intake lifts.
 *
 * <h2>Roller control</h2>
 *
 * The rollers are driven by a single brushed motor with a built-in
 * encoder. We run the motor in voltage mode (not closed-loop velocity)
 * for intaking, because the rollers' job is to push pieces, not to
 * spin at a precise speed. For "hold" and "feed" modes we use
 * closed-loop velocity so the motor does not slow down when a piece
 * is in contact with it.
 *
 * <h2>Feed path</h2>
 *
 * When the operator wants to score, the rollers reverse briefly to
 * eject the piece into the shooter. This is the {@link #feed(Shooter)}
 * method; it is called by the driver's left bumper.
 *
 * <h2>Safety</h2>
 *
 * The intake has no hard limits; software limits enforce the maximum
 * roller voltage and the maximum deploy duration. If the intake is
 * deployed for more than 5 seconds without a piece being detected,
 * the rollers stop automatically to prevent overheating.
 */
public class Intake extends SubsystemBase {

    private final MotorControllerWrapper rollerMotor;
    private final edu.wpi.first.wpilibj.DoubleSolenoid deploySolenoid;
    private final edu.wpi.first.wpilibj.DigitalInput beamBreak;

    private double rollerSpeed;
    private boolean deployed;
    private double deployStartTime;

    /**
     * Constructs the intake subsystem.
     *
     * @param sensors the sensor aggregator (for the beam-break).
     */
    public Intake(Sensors sensors) {
        this.rollerMotor = new MotorControllerWrapper(MotorConstants.INTAKE_ROLLER_CAN_ID);
        this.deploySolenoid = new edu.wpi.first.wpilibj.DoubleSolenoid(
                RobotConstants.PCM_ID,
                edu.wpi.first.wpilibj.PneumaticsModuleType.CTREPCM,
                RobotConstants.INTAKE_DEPLOY_FORWARD,
                RobotConstants.INTAKE_DEPLOY_REVERSE);
        this.beamBreak = new edu.wpi.first.wpilibj.DigitalInput(RobotConstants.INTAKE_BEAM_BREAK_CHANNEL);
        this.rollerSpeed = 0;
        this.deployed = false;
        this.deployStartTime = 0;

        rollerMotor.configVoltage(0, RobotConstants.INTAKE_MAX_VOLTAGE);
    }

    /**
     * Deploys the intake to the floor.
     */
    public void deploy() {
        deploySolenoid.set(edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward);
        deployed = true;
        deployStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }

    /**
     * Retracts the intake to the stowed position.
     */
    public void retract() {
        deploySolenoid.set(edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse);
        deployed = false;
        rollerSpeed = 0;
    }

    /**
     * Sets the roller speed in the range [-1, 1]. Positive values
     * spin the rollers inward (intake direction).
     *
     * @param speed the desired roller speed.
     */
    public void setRollerSpeed(double speed) {
        rollerSpeed = Math.max(-1.0, Math.min(1.0, speed));
    }

    /**
     * Feeds a held game piece into the shooter. Reverses the rollers
     * briefly to eject the piece, then stops.
     *
     * @param shooter the shooter subsystem (used to verify it is up
     *                to speed before feeding).
     */
    public void feed(Shooter shooter) {
        if (!shooter.isAtSpeed()) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                    "[Intake] Feed requested but shooter not at speed", false);
            return;
        }
        setRollerSpeed(RobotConstants.INTAKE_FEED_SPEED);
        edu.wpi.first.wpilibj.Timer.delay(0.3);
        setRollerSpeed(0);
    }

    /** @return true if the beam-break sensor currently detects a game piece. */
    public boolean isGamePieceDetected() {
        return !beamBreak.get();
    }

    /** Emergency stop: cut roller power and retract the intake. */
    public void emergencyStop() {
        rollerSpeed = 0;
        rollerMotor.set(0);
        retract();
    }

    @Override
    public void periodic() {
        rollerMotor.set(rollerSpeed);

        // Auto-retract safety: if the intake has been deployed for
        // more than 5 seconds without a piece being detected, retract
        // to prevent motor overheating.
        if (deployed && (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - deployStartTime) > 5.0
                && !isGamePieceDetected()) {
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                    "[Intake] Auto-retract: 5s without piece", false);
            retract();
        }

        SmartDashboard.putNumber("Intake/RollerSpeed", rollerSpeed);
        SmartDashboard.putBoolean("Intake/Deployed", deployed);
        SmartDashboard.putBoolean("Intake/PieceDetected", isGamePieceDetected());
    }

    public void notifyDisabled() {
        emergencyStop();
    }
}
