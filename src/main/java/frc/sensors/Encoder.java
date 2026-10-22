// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Abstraction over a single absolute rotary encoder.
 *
 * <p>The Thunders robot uses Through-Bore or MagEncoder absolute
 * encoders on each swerve module and each arm joint. These encoders
 * report a duty-cycle-encoded angle that is power-cycle-stable,
 * meaning the encoder reports the same angle after a power cycle
 * (unlike a relative encoder, which reports zero).
 *
 * <h2>Why absolute encoders?</h2>
 *
 * Swerve modules and arm joints need to know their absolute angle at
 * startup; a relative encoder would require a "homing" sequence that
 * rotates the mechanism until a limit switch is hit, which is slow
 * and risky. An absolute encoder reports the angle immediately,
 * allowing the closed-loop controller to engage the moment the robot
 * is enabled.
 *
 * <h2>Conversion</h2>
 *
 * The raw encoder reports a value in [0, 1) that corresponds to a
 * full revolution. This class converts the raw value to degrees
 * using the formula:
 *
 * <pre>
 *   degrees = (raw - offset) * 360
 * </pre>
 *
 * The offset accounts for the encoder being mounted in a non-zero
 * orientation (e.g., the encoder's "zero" position is at the
 * module's 45-degree mark). Offsets are determined experimentally
 * by rotating the mechanism to a known angle and recording the
 * raw encoder reading.
 *
 * <h2>Wrapping</h2>
 *
 * The encoder reports in [0, 1); subtracting an offset can produce
 * values outside this range. This class wraps the result to
 * (-180, +180] so that downstream controllers can use it directly
 * without worrying about wraparound.
 *
 * <h2>Disconnection detection</h2>
 *
 * If the encoder is disconnected, the {@link DutyCycleEncoder}
 * reports a fixed value (typically 0.0 or 0.5). This class detects
 * the disconnect by checking the {@link DutyCycleEncoder#isConnected()}
 * flag and publishes a fault to the dashboard.
 */
public class Encoder {

    private final DutyCycleEncoder encoder;
    private final double offsetDegrees;
    private final String name;

    /**
     * Constructs an Encoder on the given DIO port.
     *
     * @param dioPort the DIO port the encoder is wired to.
     * @param offsetDegrees the angle, in degrees, of the encoder's
     *                      zero position relative to the mechanism's
     *                      zero position.
     */
    public Encoder(int dioPort, double offsetDegrees) {
        this(dioPort, offsetDegrees, "Encoder" + dioPort);
    }

    /** Constructs an Encoder with a custom name for diagnostics. */
    public Encoder(int dioPort, double offsetDegrees, String name) {
        this.encoder = new DutyCycleEncoder(dioPort);
        this.offsetDegrees = offsetDegrees;
        this.name = name;
        encoder.setDistancePerRotation(360.0);
    }

    /**
     * @return the absolute angle reported by the encoder, in degrees,
     *         wrapped to (-180, +180]. Returns {@link Double#NaN} if
     *         the encoder is disconnected.
     */
    public double getPosition() {
        if (!encoder.isConnected()) {
            SmartDashboard.putBoolean(name + "/Connected", false);
            return Double.NaN;
        }
        SmartDashboard.putBoolean(name + "/Connected", true);
        double raw = encoder.getDistance() - offsetDegrees;
        return wrapDegrees(raw);
    }

    /** @return the raw encoder reading, in degrees, before offset and wrapping. */
    public double getRawPosition() {
        return encoder.getDistance();
    }

    /** @return the encoder's sample rate, in Hz. */
    public double getFrequency() {
        return encoder.getFrequency();
    }

    /** @return true if the encoder is connected and reporting a valid signal. */
    public boolean isConnected() {
        return encoder.isConnected();
    }

    /**
     * Wraps an angle to the range (-180, +180].
     *
     * @param degrees the input angle, in degrees.
     * @return the wrapped angle.
     */
    private static double wrapDegrees(double degrees) {
        degrees %= 360.0;
        if (degrees > 180.0) {
            degrees -= 360.0;
        } else if (degrees <= -180.0) {
            degrees += 360.0;
        }
        return degrees;
    }
}
