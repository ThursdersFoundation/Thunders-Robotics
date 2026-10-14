/*----------------------------------------------------------------------------*/
/* Copyright (c) 2024 Thunders Robotic Team. All Rights Reserved.             */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * ============================================================
 *  THUNDERS ROBOTIC - ROBOT STATE
 * ============================================================
 *
 * Defines and manages the state (status) of the robot and of the
 * mechanism systems in the Thunders Robotic repository.
 *
 * The enum contains two logical groups of states:
 *
 *   1. ROBOT MODES ....... set by the Driver Station / FMS
 *                          (DISABLED, INIT, AUTONOMOUS, TELEOP, TEST)
 *
 *   2. MECHANISM STATES .. gameplay states of the superstructure
 *                          (IDLE, INTAKE, HOLDING, SCORE, CLIMB)
 *
 * On top of the raw constants, this enum also acts as a lightweight
 * GLOBAL STATE MANAGER:
 *
 *      robot_State.setState(robot_State.TELEOP);   // transition
 *      robot_State.get();                          // read current state
 *      robot_State.CLIMB.isActive();               // quick check
 *      robot_State.hasGamePiece();                 // gameplay query
 *
 * NOTE: Only ONE robot program (JVM) runs on the RoboRIO at a time,
 * so static tracking fields inside this enum are safe to use.
 */
public enum robot_State {

  // ==========================================================
  //  GENERAL ROBOT STATES (Driver Station / FMS modes)
  // ==========================================================

  /** Robot disabled. All motor outputs must remain neutral. */
  DISABLED("Robot disabled - all outputs locked"),

  /** One-time initialization: gyro calibration, mechanism homing, etc. */
  INIT("Initializing - homing and calibration in progress"),

  /** Autonomous period: pre-programmed routine controls the robot. */
  AUTONOMOUS("Autonomous period - running pre-programmed routine"),

  /** Teleoperated period: human drivers control the robot. */
  TELEOP("Teleoperated period - drivers in control"),

  /** Test mode: individual mechanisms exercised for diagnostics. */
  TEST("Test mode - diagnostic mechanism exercise"),

  // ==========================================================
  //  MECHANISM STATES (gameplay / superstructure states)
  // ==========================================================

  /** All mechanisms neutral and no game piece is held. */
  IDLE("Idle - no game piece held"),

  /** Intake is running, attempting to capture a game piece. */
  INTAKE("Intaking - capturing game piece"),

  /** A game piece is securely held inside the robot. */
  HOLDING("Holding - game piece secured"),

  /** Robot is actively scoring a game piece. */
  SCORE("Scoring - releasing game piece to score"),

  /** Robot is climbing at the end of the match. */
  CLIMB("Climbing - ascending the climb structure");

  // ==========================================================
  //  ENUM FIELDS
  // ==========================================================

  /** Human-readable description of this state (for dashboards / logs). */
  private final String m_description;

  /**
   * Enum constructor: attaches a description to every constant.
   *
   * @param description human-readable explanation of the state
   */
  robot_State(String description) {
    m_description = description;
  }

  /** @return the human-readable description of this state. */
  public String getDescription() {
    return m_description;
  }

  /** @return "NAME - description", ideal for SmartDashboard / logs. */
  @Override
  public String toString() {
    return name() + " - " + m_description;
  }

  // ==========================================================
  //  CATEGORY CHECKS
  // ==========================================================

  /** @return true if this constant is a Driver Station / FMS mode. */
  public boolean isRobotMode() {
    return this == DISABLED
        || this == INIT
        || this == AUTONOMOUS
        || this == TELEOP
        || this == TEST;
  }

  /** @return true if this constant is a mechanism (gameplay) state. */
  public boolean isMechanismState() {
    return !isRobotMode();
  }

  // ==========================================================
  //  GLOBAL STATE TRACKER (static)
  // ==========================================================

  /** The state the robot is currently in. */
  private static robot_State s_current = DISABLED;

  /** The state the robot was in before the current one. */
  private static robot_State s_previous = DISABLED;

  /** FPGA timestamp (seconds) of the most recent state transition. */
  private static double s_lastChangeTime = 0.0;

  /**
   * Transitions the robot into a new state.
   *
   * No-op transitions (same state) are ignored so that
   * {@link #getTimeInState()} stays accurate. Illegal transitions are
   * NOT blocked, only reported — use
   * {@link #isTransitionValid(robot_State, robot_State)} to validate
   * beforehand if needed.
   *
   * @param next the state to transition into (null is ignored)
   * @return the resulting current state
   */
  public static robot_State setState(robot_State next) {
    if (next == null || next == s_current) {
      return s_current; // ignore no-op / invalid input
    }

    // Warn (but do not block) if the match flow looks unusual.
    if (!isTransitionValid(s_current, next)) {
      DriverStation.reportWarning(
          "[STATE] Unusual transition: " + s_current.name() + " -> " + next.name(),
          false);
    }

    s_previous = s_current;
    s_current = next;
    s_lastChangeTime = Timer.getFPGATimestamp();

    System.out.println("[STATE] " + s_previous.name() + " -> " + s_current.name());
    return s_current;
  }

  /** @return the state the robot is currently in. */
  public static robot_State get() {
    return s_current;
  }

  /** @return the state the robot was in before the current one. */
  public static robot_State getPrevious() {
    return s_previous;
  }

  /** @return seconds elapsed since the last state transition. */
  public static double getTimeInState() {
    return Timer.getFPGATimestamp() - s_lastChangeTime;
  }

  /**
   * Reads the Driver Station and updates the robot-mode part of the
   * state automatically. Call this from robotPeriodic() as a safety net
   * so the state can never drift from the real match mode.
   *
   * @return the resulting current state
   */
  public static robot_State syncWithDriverStation() {
    if (DriverStation.isDisabled()) {
      setState(DISABLED);
    } else if (DriverStation.isAutonomousEnabled()) {
      setState(AUTONOMOUS);
    } else if (DriverStation.isTeleopEnabled()) {
      setState(TELEOP);
    } else if (DriverStation.isTestEnabled()) {
      setState(TEST);
    }
    return s_current;
  }

  /** Resets the tracker to DISABLED (useful for simulation / unit tests). */
  public static void reset() {
    s_current = DISABLED;
    s_previous = DISABLED;
    s_lastChangeTime = Timer.getFPGATimestamp();
  }

  // ==========================================================
  //  TRANSITION RULES (state machine validation)
  // ==========================================================

  /**
   * Checks whether a transition between ROBOT MODES follows the legal
   * match flow. Mechanism states are always considered valid because
   * they are managed by commands, not by the match flow.
   *
   * <pre>
   *   DISABLED   -> INIT | DISABLED
   *   INIT       -> AUTONOMOUS | TELEOP | TEST | DISABLED
   *   AUTONOMOUS -> TELEOP (normal end of auto) | DISABLED (aborted)
   *   TELEOP     -> DISABLED
   *   TEST       -> DISABLED | AUTONOMOUS | TELEOP
   * </pre>
   *
   * @param from the state transitioning FROM
   * @param to   the state transitioning TO
   * @return true if the transition is legal
   */
  public static boolean isTransitionValid(robot_State from, robot_State to) {
    if (from.isMechanismState() || to.isMechanismState()) {
      return true; // mechanism states are free-form
    }

    switch (from) {
      case DISABLED:
        return to == INIT || to == DISABLED;
      case INIT:
        return to == AUTONOMOUS || to == TELEOP || to == TEST || to == DISABLED;
      case AUTONOMOUS:
        return to == TELEOP || to == DISABLED;
      case TEST:
        return to == DISABLED || to == AUTONOMOUS || to == TELEOP;
      case TELEOP:
        return to == DISABLED;
      default:
        return true;
    }
  }

  /**
   * Gameplay safety policy: which MECHANISM states are legal while the
   * robot is in the given ROBOT MODE. Adjust these rules to match your
   * team's game strategy and safety requirements.
   *
   * @param mode the robot mode to test against
   * @return true if this mechanism state is legal in that mode
   */
  public boolean isAllowedIn(robot_State mode) {
    if (isRobotMode()) {
      return true; // robot modes themselves are always "allowed"
    }

    switch (this) {
      case CLIMB:
        // Climbing is only safe (and only useful) during teleop.
        return mode == TELEOP;
      case SCORE:
        return mode == AUTONOMOUS || mode == TELEOP;
      case INTAKE:
      case HOLDING:
      case IDLE:
        return mode == AUTONOMOUS || mode == TELEOP || mode == TEST;
      default:
        return true;
    }
  }

  // ==========================================================
  //  QUICK QUERIES
  // ==========================================================

  /** @return true if the CURRENT robot state equals this constant. */
  public boolean isActive() {
    return s_current == this;
  }

  /**
   * @param states candidate states
   * @return true if the current state is ANY of the given states
   */
  public static boolean isAny(robot_State... states) {
    for (robot_State s : states) {
      if (s == s_current) {
        return true;
      }
    }
    return false;
  }

  /** @return true if the robot is in an enabled match mode (AUTO or TELEOP). */
  public static boolean isEnabledMode() {
    return s_current == AUTONOMOUS || s_current == TELEOP;
  }

  /** @return true if the superstructure currently holds a game piece. */
  public static boolean hasGamePiece() {
    return s_current == HOLDING || s_current == SCORE;
  }
}
