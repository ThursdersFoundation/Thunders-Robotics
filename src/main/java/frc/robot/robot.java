/*----------------------------------------------------------------------------*/
/* Copyright (c) 2024 Thunders Robotic Team. All Rights Reserved.             */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Alliance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * ============================================================
 *  THUNDERS ROBOTIC - MAIN ROBOT CLASS
 * ============================================================
 *
 * The VM (RoboRIO) is configured to automatically run this class, and to call
 * the functions corresponding to each mode, as described in the TimedRobot
 * documentation.
 *
 * Lifecycle overview (order of execution):
 *
 *   1. robotInit()          -> called ONCE when the robot boots
 *   2. robotPeriodic()      -> called every 20 ms in ALL modes
 *
 *   Disabled:   disabledInit()   / disabledPeriodic()
 *   Autonomous: autonomousInit()/ autonomousPeriodic()
 *   Teleop:     teleopInit()     / teleopPeriodic()
 *   Test:       testInit()       / testPeriodic()
 *
 * NOTE: If you change the name of this class or the package after creating
 * this project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

  // ==========================================================
  //  CONSTANTS
  // ==========================================================

  /** Control loop period in seconds (20 ms = 50 Hz). */
  private static final double kLoopPeriod = 0.02;

  // ==========================================================
  //  CLASS MEMBERS
  // ==========================================================

  /** The command selected on the dashboard and run during autonomous. */
  private Command m_autonomousCommand;

  /** Container that holds all subsystems, commands, and button bindings. */
  private RobotContainer m_robotContainer;

  /** Power Distribution Hub used for battery / current monitoring. */
  private PowerDistribution m_powerDistribution;

  /** Timer that measures elapsed time in the current robot mode. */
  private final Timer m_modeTimer = new Timer();

  // ==========================================================
  //  ROBOT INITIALIZATION (runs ONCE at boot)
  // ==========================================================

  /**
   * This function is run exactly once when the robot is first started up.
   * Use it for any one-time initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate the RobotContainer. This performs all button bindings,
    // creates subsystems, and publishes the autonomous chooser to the
    // dashboard. MUST be created before anything else runs.
    m_robotContainer = new RobotContainer();

    // Initialize the Power Distribution Hub (PDH / PDP)
    m_powerDistribution = new PowerDistribution();
    m_powerDistribution.setSwitchableChannel(false);

    // Start logging all robot data (written to USB stick if attached,
    // otherwise stored internally on the RoboRIO).
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog(), true);

    // Start the driver camera stream for the Driver Station dashboard.
    CameraServer.startAutomaticCapture();

    // Print a banner so we can confirm the correct code is deployed.
    System.out.println("==================================================");
    System.out.println("  THUNDERS ROBOTIC - Robot initialization complete");
    System.out.println("==================================================");
  }

  // ==========================================================
  //  PERIODIC (runs every 20 ms in EVERY mode)
  // ==========================================================

  /**
   * This function is called every 20 ms, no matter the mode. Use this for
   * diagnostics and tasks that must run during disabled, autonomous,
   * teleoperated, and test modes.
   */
  @Override
  public void robotPeriodic() {
    // Runs the CommandScheduler. This is responsible for polling buttons,
    // scheduling new commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic()
    // methods. This MUST be called here for the Command-based framework
    // to work at all.
    CommandScheduler.getInstance().run();

    // Publish general telemetry to SmartDashboard / Shuffleboard.
    publishTelemetry();
  }

  /** Publishes general robot telemetry to the dashboard. */
  private void publishTelemetry() {
    // Battery health monitoring
    SmartDashboard.putNumber("Battery/Voltage (V)", m_powerDistribution.getVoltage());
    SmartDashboard.putNumber("Battery/Total Current (A)", m_powerDistribution.getTotalCurrent());

    // Match information
    SmartDashboard.putNumber("Match/Time Remaining (s)", DriverStation.getMatchTime());
    SmartDashboard.putBoolean("Match/FMS Attached", DriverStation.isFMSAttached());
    SmartDashboard.putString(
        "Match/Alliance",
        DriverStation.getAlliance()
            .map(a -> a == Alliance.Red ? "Red" : "Blue")
            .orElse("Unknown"));

    // Mode timer (useful at practice events where there is no FMS clock)
    SmartDashboard.putNumber("Match/Mode Elapsed (s)", m_modeTimer.get());
  }

  // ==========================================================
  //  DISABLED MODE
  // ==========================================================

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_modeTimer.restart();
    System.out.println("[MODE] Disabled");
  }

  /** Called every 20 ms while the robot is disabled. */
  @Override
  public void disabledPeriodic() {
    // Safe place for diagnostics; all outputs must remain neutral here.
  }

  // ==========================================================
  //  AUTONOMOUS MODE
  // ==========================================================

  /** This function is called once each time the robot enters Autonomous mode. */
  @Override
  public void autonomousInit() {
    m_modeTimer.restart();
    System.out.println("[MODE] Autonomous started");

    // Read the autonomous command selected on the dashboard chooser.
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Schedule the autonomous command (if one was selected).
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during Autonomous mode. */
  @Override
  public void autonomousPeriodic() {
    // All robot logic is handled by commands via the scheduler,
    // so nothing is needed here.
  }

  // ==========================================================
  //  TELEOPERATED MODE
  // ==========================================================

  /** This function is called once each time the robot enters Teleop mode. */
  @Override
  public void teleopInit() {
    m_modeTimer.restart();
    System.out.println("[MODE] Teleop started");

    // Make sure the autonomous command stops running when teleop starts.
    // If you want autonomous to continue until interrupted, remove this.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // All driver control is handled by commands bound in RobotContainer.
  }

  // ==========================================================
  //  TEST MODE
  // ==========================================================

  /** This function is called once each time the robot enters Test mode. */
  @Override
  public void testInit() {
    m_modeTimer.restart();
    System.out.println("[MODE] Test started");

    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during Test mode. */
  @Override
  public void testPeriodic() {
    // Use this mode to test individual mechanisms safely.
  }
}
