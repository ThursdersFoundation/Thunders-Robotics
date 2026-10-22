// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.

package frc.robot.constants;

/**
 * Wiring and configuration constants for every sensor on the Thunders
 * robot.
 *
 * <p>Sensors are wired to the RoboRIO's Digital I/O (DIO), Analog
 * Input (AI), I2C, SPI, and Relay ports. This file maps each sensor
 * to its physical port so that wiring changes can be reflected in a
 * single place.
 *
 * <h2>Port assignment convention</h2>
 *
 * <ul>
 *   <li><b>DIO 0-3</b> -&gt; Swerve module absolute encoders.</li>
 *   <li><b>DIO 4-5</b> -&gt; Arm joint absolute encoders.</li>
 *   <li><b>DIO 6</b> -&gt; Elevator string-pot encoder.</li>
 *   <li><b>DIO 7-8</b> -&gt; Elevator top/bottom limit switches.</li>
 *   <li><b>DIO 9</b> -&gt; Climb hook engagement sensor.</li>
 *   <li><b>AI 0</b> -&gt; Battery voltage monitor.</li>
 *   <li><b>AI 1</b> -&gt; Ultrasonic distance sensor (front).</li>
 *   <li><b>SPI 0</b> -&gt; ADXRS450 gyro (onboard FPGA).</li>
 *   <li><b>SPI 1</b> -&gt; ADIS16470 IMU.</li>
 *   <li><b>I2C 0</b> -&gt; Colour sensor (if equipped).</li>
 *   <li><b>Relay 0</b> -&gt; Compressor (controlled by PCM, not this).</li>
 * </ul>
 *
 * <h2>Why a separate file?</h2>
 *
 * Sensor wiring is the most common source of "the robot doesn't work"
 * issues. Centralising the wiring map in one file lets the
 * electrical team verify wiring against the code without digging
 * through subsystem implementations. When a wire is moved, only this
 * file changes; the subsystem code that uses the sensor is unchanged.
 *
 * <h2>Sensor offsets</h2>
 *
 * Absolute encoders report a non-zero angle when the mechanism is at
 * its "zero" position. This offset is determined experimentally by
 * rotating the mechanism to a known angle (e.g., using a spirit
 * level) and recording the encoder reading. The offset is then
 * subtracted from the raw reading at runtime to obtain the
 * mechanism-space angle.
 *
 * <p>Offsets are stored here so that they can be updated without
 * touching the {@link frc.robot.sensors.Encoder} class.
 */
public final class SensorConstants {

    private SensorConstants() {
        throw new AssertionError("SensorConstants is a constants holder.");
    }

    // --- Gyroscopes -------------------------------------------------------

    /** SPI port for the ADXRS450 yaw gyro. */
    public static final int GYRO_PORT = 0;
    /** SPI port for the ADIS16470 IMU. */
    public static final int IMU_PORT = 1;

    // --- Swerve module absolute encoders ---------------------------------

    public static final int FRONT_LEFT_ENCODER_PORT  = 0;
    public static final int FRONT_RIGHT_ENCODER_PORT = 1;
    public static final int BACK_LEFT_ENCODER_PORT   = 2;
    public static final int BACK_RIGHT_ENCODER_PORT  = 3;

    /** Encoder offset angles, in degrees, measured at mechanism zero. */
    public static final double FRONT_LEFT_ENCODER_OFFSET_DEG  = -0.5;
    public static final double FRONT_RIGHT_ENCODER_OFFSET_DEG =  90.2;
    public static final double BACK_LEFT_ENCODER_OFFSET_DEG   = 179.8;
    public static final double BACK_RIGHT_ENCODER_OFFSET_DEG  = -89.9;

    // --- Arm joint encoders -----------------------------------------------

    public static final int ARM_SHOULDER_ENCODER_PORT = 4;
    public static final int ARM_ELBOW_ENCODER_PORT    = 5;

    public static final double ARM_SHOULDER_ENCODER_OFFSET_DEG = 12.3;
    public static final double ARM_ELBOW_ENCODER_OFFSET_DEG    = -45.6;

    // --- Elevator sensors -------------------------------------------------

    public static final int ELEVATOR_ENCODER_PORT       = 6;
    public static final int ELEVATOR_BOTTOM_SWITCH_PORT = 7;
    public static final int ELEVATOR_TOP_SWITCH_PORT    = 8;

    /** Conversion factor from elevator encoder ticks to metres. */
    public static final double ELEVATOR_TICKS_PER_METER = 4096.0 / (Math.PI * 0.0254);

    // --- Ultrasonic distance sensor --------------------------------------

    /** Analog port for the front ultrasonic distance sensor. */
    public static final int FRONT_ULTRASONIC_PORT = 1;

    /** Quiescent voltage of the ultrasonic sensor, in volts. */
    public static final double ULTRASONIC_V_OFFSET = 0.0;
    /** Conversion factor from volts to metres. */
    public static final double ULTRASONIC_SCALE = 0.5; // Vcc/512 per inch -> ~0.5 V/m.
    /** Threshold for "object detected" trigger, in metres. */
    public static final double ULTRASONIC_THRESHOLD_M = 0.30;

    // --- Power distribution -----------------------------------------------

    /** CAN ID of the Power Distribution Panel (or Hub). */
    public static final int PDP_ID = 1;

    // --- Pneumatics -------------------------------------------------------

    /** CAN ID of the Pneumatics Control Module. */
    public static final int PCM_ID = 0;

    // --- Driver station inputs --------------------------------------------

    /** The driver station's "alliance station" port (set at event check-in). */
    public static final int ALLIANCE_STATION_PORT = 0;

    // --- Sensor sample rates ----------------------------------------------

    /** ADIS16470 IMU sample rate, in Hz. */
    public static final int IMU_SAMPLE_RATE_HZ = 200;
    /** Ultrasonic sensor filter cutoff, in Hz. */
    public static final double ULTRASONIC_FILTER_CUTOFF_HZ = 5.0;
}
