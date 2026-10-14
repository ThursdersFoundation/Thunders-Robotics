/*----------------------------------------------------------------------------*/
/* Copyright (c) 2024 Thunders Robotic Team. All Rights Reserved.             */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * ============================================================
 *  THUNDERS ROBOTIC - ROBOT CONTAINER
 * ============================================================
 *
 * The RobotContainer declares the complete structure of the robot:
 *
 *   1. Subsystems  -> physical mechanisms (drivetrain, intake, etc.)
 *   2. Controllers -> driver / operator input devices
 *   3. Bindings    -> which buttons trigger which commands
 *   4. Defaults    -> commands that run automatically (e.g. driving)
 *   5. Autonomous  -> selectable auto routines on the dashboard
 */
public class RobotContainer {

  // ==========================================================
  //  SUBSYSTEMS
  // ==========================================================
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Climber m_climber = new Climber();

  // ==========================================================
  //  DRIVER & OPERATOR CONTROLS
  // ==========================================================
  private final CommandXboxController m_driver =
      new CommandXboxController(OIConstants.DRIVER_USB_PORT);
  private final CommandXboxController m_operator =
      new CommandXboxController(OIConstants.OPERATOR_USB_PORT);

  /** While held, caps robot speed for precision driving. */
  private final Trigger m_slowMode = m_driver.leftBumper();

  // Input smoothing (acceleration limiters, units per second)
  private final SlewRateLimiter m_throttleSlew = new SlewRateLimiter(3.0);
  private final SlewRateLimiter m_turnSlew = new SlewRateLimiter(6.0);

  // ==========================================================
  //  AUTONOMOUS CHOOSER
  // ==========================================================
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /** Constructor: builds the whole robot structure. */
  public RobotContainer() {
    configureDefaultCommands();
    configureButtonBindings();
    configureAutonomousChooser();

    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  // ==========================================================
  //  DEFAULT COMMANDS
  //  Run automatically whenever no other command uses the subsystem.
  // ==========================================================
  private void configureDefaultCommands() {
    // Arcade drive from the driver sticks (with deadband + smoothing).
    m_drivetrain.setDefaultCommand(
        new RunCommand(
            () ->
                m_drivetrain.arcadeDrive(
                    getThrottle() * speedScale(), getTurn() * speedScale()),
            m_drivetrain));

    // All mechanisms idle at 0% so they never move uncommanded.
    m_intake.setDefaultCommand(new RunCommand(m_intake::stop, m_intake));
    m_shooter.setDefaultCommand(new RunCommand(m_shooter::stop, m_shooter));
    m_climber.setDefaultCommand(new RunCommand(m_climber::stop, m_climber));
  }

  // ==========================================================
  //  BUTTON BINDINGS
  // ==========================================================
  private void configureButtonBindings() {

    // ---------- DRIVER ----------
    // E-STOP: press START once to freeze ALL mechanisms.
    // Press START again to release control back to the drivers.
    m_driver.start()
        .toggleOnTrue(
            Commands.parallel(
                Commands.idle(m_drivetrain),
                Commands.idle(m_intake),
                Commands.idle(m_shooter),
                Commands.idle(m_climber)));

    // ---------- OPERATOR: INTAKE ----------
    m_operator.rightTrigger(OIConstants.TRIGGER_THRESHOLD)
        .whileTrue(
            new RunCommand(() -> m_intake.setIntake(OIConstants.INTAKE_SPEED), m_intake));

    m_operator.leftTrigger(OIConstants.TRIGGER_THRESHOLD)
        .whileTrue(
            new RunCommand(() -> m_intake.setIntake(-OIConstants.INTAKE_SPEED), m_intake));

    // ---------- OPERATOR: SHOOTER ----------
    m_operator.a()
        .whileTrue(
            new RunCommand(
                () -> m_shooter.setSpeed(ShooterConstants.SHOOT_SPEED), m_shooter));

    m_operator.b()
        .whileTrue(
            new RunCommand(
                () -> m_shooter.setSpeed(-ShooterConstants.SHOOT_SPEED), m_shooter));

    // Full shot: spins the flywheel, waits for spin-up, then feeds the piece.
    m_operator.rightBumper()
        .whileTrue(
            Commands.sequence(
                Commands.run(
                        () -> m_shooter.setSpeed(ShooterConstants.SHOOT_SPEED), m_shooter)
                    .withTimeout(ShooterConstants.SPIN_UP_TIME),
                Commands.run(
                        () -> {
                          m_shooter.setSpeed(ShooterConstants.SHOOT_SPEED);
                          m_intake.setIntake(OIConstants.INTAKE_SPEED);
                        },
                        m_shooter,
                        m_intake)
                    .withTimeout(ShooterConstants.FEED_TIME)));

    // ---------- OPERATOR: CLIMBER ----------
    m_operator.x()
        .whileTrue(
            new RunCommand(
                () -> m_climber.setSpeed(ClimberConstants.CLIMB_SPEED), m_climber));

    m_operator.y()
        .whileTrue(
            new RunCommand(
                () -> m_climber.setSpeed(-ClimberConstants.CLIMB_SPEED), m_climber));
  }

  // ==========================================================
  //  AUTONOMOUS
  // ==========================================================
  private void configureAutonomousChooser() {
    m_autoChooser.setDefaultOption("Do Nothing", Commands.none());
    m_autoChooser.addOption("Taxi (2 s forward)", taxiAuto());
    m_autoChooser.addOption("Shoot + Taxi", shootAndTaxiAuto());
    m_autoChooser.addOption("Two-Ball Auto", twoBallAuto());
  }

  /** @return the command selected on the dashboard (used by Robot.java). */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  // ---------- Auto routine definitions ----------

  /** Open-loop forward drive at a given percent output. */
  private Command driveForward(double percent) {
    return Commands.run(() -> m_drivetrain.arcadeDrive(percent, 0.0), m_drivetrain);
  }

  /** Shared shoot sub-sequence: spin up -> feed -> stop. */
  private Command shootSequence() {
    return Commands.sequence(
        Commands.run(() -> m_shooter.setSpeed(ShooterConstants.SHOOT_SPEED), m_shooter)
            .withTimeout(ShooterConstants.SPIN_UP_TIME),
        Commands.run(
                () -> {
                  m_shooter.setSpeed(ShooterConstants.SHOOT_SPEED);
                  m_intake.setIntake(OIConstants.INTAKE_SPEED);
                },
                m_shooter,
                m_intake)
            .withTimeout(ShooterConstants.FEED_TIME),
        Commands.runOnce(
            () -> {
              m_shooter.stop();
              m_intake.stop();
            },
            m_shooter,
            m_intake));
  }

  private Command taxiAuto() {
    return Commands.sequence(
        driveForward(AutoConstants.TAXI_SPEED).withTimeout(AutoConstants.TAXI_TIME),
        Commands.runOnce(m_drivetrain::stop, m_drivetrain));
  }

  private Command shootAndTaxiAuto() {
    return Commands.sequence(
        shootSequence(),
        driveForward(-AutoConstants.TAXI_SPEED).withTimeout(AutoConstants.TAXI_TIME),
        Commands.runOnce(m_drivetrain::stop, m_drivetrain));
  }

  private Command twoBallAuto() {
    return Commands.sequence(
        // 1) Drive forward while intaking a second game piece simultaneously
        Commands.parallel(
            driveForward(AutoConstants.PICKUP_SPEED).withTimeout(AutoConstants.PICKUP_TIME),
            Commands.run(() -> m_intake.setIntake(OIConstants.INTAKE_SPEED), m_intake)
                .withTimeout(AutoConstants.PICKUP_TIME)),
        Commands.runOnce(
            () -> {
              m_drivetrain.stop();
              m_intake.stop();
            },
            m_drivetrain,
            m_intake),
        // 2) Turn roughly 180 degrees (open loop; replace with a gyro turn later)
        Commands.run(() -> m_drivetrain.arcadeDrive(0.0, AutoConstants.TURN_SPEED))
            .withTimeout(AutoConstants.TURN_TIME),
        Commands.runOnce(m_drivetrain::stop, m_drivetrain),
        // 3) Shoot both game pieces
        shootSequence());
  }

  // ==========================================================
  //  INPUT HELPERS
  // ==========================================================

  /** Smoothed, deadbanded, squared throttle from the driver left stick. */
  private double getThrottle() {
    double raw = MathUtil.applyDeadband(-m_driver.getLeftY(), OIConstants.DRIVE_DEADBAND);
    return m_throttleSlew.calculate(Math.copySign(raw * raw, raw));
  }

  /** Smoothed, deadbanded, squared turn from the driver right stick. */
  private double getTurn() {
    double raw = MathUtil.applyDeadband(-m_driver.getRightX(), OIConstants.DRIVE_DEADBAND);
    return m_turnSlew.calculate(Math.copySign(raw * raw, raw));
  }

  /** 1.0 normally, reduced scale while LEFT BUMPER is held. */
  private double speedScale() {
    return m_slowMode.getAsBoolean() ? OIConstants.SLOW_MODE_SCALE : 1.0;
  }
}
