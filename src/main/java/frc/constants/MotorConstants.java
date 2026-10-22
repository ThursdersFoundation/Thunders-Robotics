// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.

package frc.robot.constants;

/**
 * CAN IDs and configuration parameters for every motor controller on
 * the Thunders robot.
 *
 * <p>The Thunders robot uses CTRE Talon FX (Kracken X60) motor
 * controllers for the swerve drive motors and the shooter flywheels,
 * and CTRE Talon SRX controllers for the arm and elevator motors. The
 * intake roller is driven by a REV Spark Max. All controllers are on
 * the same CAN bus.
 *
 * <h2>CAN ID assignment convention</h2>
 *
 * CAN IDs are assigned in groups by subsystem to make wiring easier:
 *
 * <pre>
 *   1-12  : Drivetrain (4 modules x 3 motors: drive, steer, encoder)
 *   13-15 : Arm (shoulder, elbow, spare)
 *   16-17 : Elevator (main, spare)
 *   18    : Intake roller
 *   19-20 : Shooter (top, bottom)
 *   21-22 : Climber (left, right)
 * </pre>
 *
 * <h2>Configuration</h2>
 *
 * Each motor controller has a small set of configuration parameters
 * that must be set on startup:
 *
 * <ul>
 *   <li><b>Motor inversion</b> -&gt; whether positive motor output
 *       corresponds to forward or backward motion.</li>
 *   <li><b>Sensor phase</b> -&gt; whether positive sensor velocity
 *       corresponds to positive motor output.</li>
 *   <li><b>Current limit</b> -&gt; the maximum continuous current the
 *       motor can draw without tripping the controller.</li>
 *   <li><b>Voltage compensation</b> -&gt; the nominal voltage the
 *       controller compensates to (typically 12 V).</li>
 *   <li><b>Brake mode</b> -&gt; whether the controller shorts its
 *       windings at zero output to resist motion.</li>
 * </ul>
 *
 * These parameters are stored in this file as {@code MotorConfig}
 * records and applied by the {@link frc.robot.subsystems.MotorControllerWrapper}
 * class at construction time.
 *
 * <h2>Why a separate file?</h2>
 *
 * Motor configurations are physically meaningful: they describe the
 * wiring of the robot. Putting them in a separate file from the
 * algorithmic constants ({@link RobotConstants}) makes it easier to
 * re-assign CAN IDs when a motor controller is replaced without
 * accidentally changing a PID gain.
 */
public final class MotorConstants {

    private MotorConstants() {
        throw new AssertionError("MotorConstants is a constants holder.");
    }

    // --- CAN IDs ----------------------------------------------------------

    /** Drivetrain CAN IDs (front-left, front-right, back-left, back-right). */
    public static final int FRONT_LEFT_DRIVE_CAN_ID  = 1;
    public static final int FRONT_LEFT_STEER_CAN_ID  = 2;
    public static final int FRONT_RIGHT_DRIVE_CAN_ID = 3;
    public static final int FRONT_RIGHT_STEER_CAN_ID = 4;
    public static final int BACK_LEFT_DRIVE_CAN_ID   = 5;
    public static final int BACK_LEFT_STEER_CAN_ID   = 6;
    public static final int BACK_RIGHT_DRIVE_CAN_ID  = 7;
    public static final int BACK_RIGHT_STEER_CAN_ID  = 8;

    /** Arm CAN IDs. */
    public static final int ARM_SHOULDER_CAN_ID = 13;
    public static final int ARM_ELBOW_CAN_ID    = 14;

    /** Elevator CAN IDs. */
    public static final int ELEVATOR_CAN_ID = 16;

    /** Intake CAN IDs. */
    public static final int INTAKE_ROLLER_CAN_ID = 18;

    /** Shooter CAN IDs. */
    public static final int SHOOTER_TOP_CAN_ID    = 19;
    public static final int SHOOTER_BOTTOM_CAN_ID = 20;

    /** Climber CAN IDs (reserved). */
    public static final int CLIMBER_LEFT_CAN_ID  = 21;
    public static final int CLIMBER_RIGHT_CAN_ID = 22;

    // --- Current limits ----------------------------------------------------

    /** Continuous current limit for swerve drive motors, in amps. */
    public static final int DRIVE_CURRENT_LIMIT_A = 40;
    /** Continuous current limit for swerve steer motors, in amps. */
    public static final int STEER_CURRENT_LIMIT_A = 20;
    /** Continuous current limit for arm motors, in amps. */
    public static final int ARM_CURRENT_LIMIT_A = 30;
    /** Continuous current limit for elevator motor, in amps. */
    public static final int ELEVATOR_CURRENT_LIMIT_A = 30;
    /** Continuous current limit for intake roller, in amps. */
    public static final int INTAKE_CURRENT_LIMIT_A = 20;
    /** Continuous current limit for shooter flywheels, in amps. */
    public static final int SHOOTER_CURRENT_LIMIT_A = 40;
    /** Continuous current limit for climber motors, in amps. */
    public static final int CLIMBER_CURRENT_LIMIT_A = 60;

    // --- Voltage compensation ---------------------------------------------

    /** Nominal battery voltage the motor controllers compensate to. */
    public static final double NOMINAL_VOLTAGE = 12.0;

    // --- Motor directions --------------------------------------------------

    /** True = inverted, False = normal. Determined by physically testing each motor. */
    public static final boolean FRONT_LEFT_DRIVE_INVERTED  = false;
    public static final boolean FRONT_LEFT_STEER_INVERTED  = false;
    public static final boolean FRONT_RIGHT_DRIVE_INVERTED = true;
    public static final boolean FRONT_RIGHT_STEER_INVERTED = false;
    public static final boolean BACK_LEFT_DRIVE_INVERTED   = false;
    public static final boolean BACK_LEFT_STEER_INVERTED   = false;
    public static final boolean BACK_RIGHT_DRIVE_INVERTED  = true;
    public static final boolean BACK_RIGHT_STEER_INVERTED  = false;

    public static final boolean ARM_SHOULDER_INVERTED = false;
    public static final boolean ARM_ELBOW_INVERTED = true;
    public static final boolean ELEVATOR_INVERTED = false;
    public static final boolean INTAKE_INVERTED = false;
    public static final boolean SHOOTER_TOP_INVERTED = true;
    public static final boolean SHOOTER_BOTTOM_INVERTED = false;

    // --- Gear ratios -------------------------------------------------------

    /** Drive motor gear ratio (motor revs per wheel rev). */
    public static final double DRIVE_GEAR_RATIO = 6.75;
    /** Steer motor gear ratio (motor revs per module rev). */
    public static final double STEER_GEAR_RATIO = 12.8;
    /** Arm shoulder gear ratio. */
    public static final double ARM_SHOULDER_GEAR_RATIO = 100.0;
    /** Arm elbow gear ratio. */
    public static final double ARM_ELBOW_GEAR_RATIO = 75.0;
    /** Elevator gear ratio (motor revs per drum rev). */
    public static final double ELEVATOR_GEAR_RATIO = 8.0;
    /** Shooter flywheel gear ratio. */
    public static final double SHOOTER_GEAR_RATIO = 1.0;

    // --- Wheel diameter ----------------------------------------------------

    /** Drive wheel diameter, in metres. */
    public static final double WHEEL_DIAMETER_M = 0.1016; // 4 inches.

    /** Drive wheel circumference, in metres. */
    public static final double WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_M * Math.PI;
}
