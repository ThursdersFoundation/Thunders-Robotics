// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Abstraction over a single distance-measuring sensor.
 *
 * <p>This class supports two physical sensor types commonly used on
 * the Thunders robot:
 *
 * <ul>
 *   <li><b>Analog ultrasonic sensors</b> (e.g., MaxBotix LV-MaxSonar)
 *       -&gt; report distance as an analog voltage proportional to
 *       range (typically Vcc/512 per inch).</li>
 *   <li><b>Digital limit switches</b> -&gt; report a binary "triggered"
 *       signal. Used when only a threshold crossing matters, not the
 *       actual distance.</li>
 * </ul>
 *
 * <h2>Why an abstraction?</h2>
 *
 * Different mechanisms need different sensor types: the elevator's
 * top/bottom limit switches are digital, while the intake's "is a
 * game piece nearby" sensor is ultrasonic. Wrapping both in a single
 * class lets the rest of the code treat them uniformly: a
 * {@code DistanceSensor} exposes both {@link #isTriggered()} (for
 * limit switch use cases) and {@link #getDistanceMeters()} (for
 * ranging use cases).
 *
 * <h2>Conversion</h2>
 *
 * For analog sensors, the distance is computed as:
 *
 * <pre>
 *   distanceMeters = (voltage - V_OFFSET) * SCALE
 * </pre>
 *
 * where {@code V_OFFSET} is the sensor's quiescent voltage and
 * {@code SCALE} is the conversion factor from volts to meters. Both
 * are configurable.
 *
 * <h2>Filtering</h2>
 *
 * Ultrasonic sensors are notoriously noisy; the reading can jump by
 * several centimetres between samples even when the target is
 * stationary. This class applies a simple exponential moving average
 * (EMA) filter to smooth the readings. The filter coefficient is
 * configurable; a value of 0 disables filtering.
 *
 * <h2>Threshold detection</h2>
 *
 * The {@link #isTriggered()} method returns true when the most recent
 * reading is below a configurable threshold (for digital inputs, the
 * threshold is ignored and the method just returns the digital state).
 * This is used by the elevator to detect when it has reached its top
 * or bottom limit.
 */
public class DistanceSensor {

    private final AnalogInput analogInput;
    private final DigitalInput digitalInput;
    private final double vOffset;
    private final double scale;
    private final double thresholdMeters;
    private final String name;

    private double filteredDistance;
    private double emaAlpha;

    /**
     * Constructs an analog distance sensor on the given analog port.
     *
     * @param analogPort the analog input port.
     * @param vOffset the sensor's quiescent voltage, in volts.
     * @param scale the conversion factor from volts to meters.
     * @param thresholdMeters the threshold used by {@link #isTriggered()}.
     * @param name a diagnostic name for the dashboard.
     */
    public DistanceSensor(int analogPort, double vOffset, double scale,
                          double thresholdMeters, String name) {
        this.analogInput = new AnalogInput(analogPort);
        this.digitalInput = null;
        this.vOffset = vOffset;
        this.scale = scale;
        this.thresholdMeters = thresholdMeters;
        this.name = name;
        this.filteredDistance = 0;
        this.emaAlpha = 0.3;
    }

    /**
     * Constructs a digital distance sensor (limit switch) on the
     * given DIO port.
     *
     * @param dioPort the DIO port.
     */
    public DistanceSensor(int dioPort) {
        this.digitalInput = new DigitalInput(dioPort);
        this.analogInput = null;
        this.vOffset = 0;
        this.scale = 1;
        this.thresholdMeters = 0;
        this.name = "LimitSwitch" + dioPort;
        this.filteredDistance = 0;
        this.emaAlpha = 0;
    }

    /**
     * @return the current distance reading, in meters. For digital
     *         sensors, returns 0 if triggered and 1 if not. Returns
     *         {@link Double#NaN} if the sensor is analog and not
     *         yet initialised.
     */
    public double getDistanceMeters() {
        if (digitalInput != null) {
            return digitalInput.get() ? 1.0 : 0.0;
        }
        if (analogInput == null) {
            return Double.NaN;
        }
        double voltage = analogInput.getVoltage();
        double raw = (voltage - vOffset) * scale;
        // Apply EMA filter.
        if (emaAlpha > 0 && filteredDistance != 0) {
            filteredDistance = filteredDistance + emaAlpha * (raw - filteredDistance);
        } else {
            filteredDistance = raw;
        }
        SmartDashboard.putNumber(name + "/Raw", raw);
        SmartDashboard.putNumber(name + "/Filtered", filteredDistance);
        SmartDashboard.putNumber(name + "/Voltage", voltage);
        return filteredDistance;
    }

    /**
     * @return true if the sensor is "triggered": for digital sensors,
     *         this is the digital state; for analog sensors, this is
     *         {@code distanceMeters < thresholdMeters}.
     */
    public boolean isTriggered() {
        if (digitalInput != null) {
            return !digitalInput.get(); // Limit switches are NC.
        }
        return getDistanceMeters() < thresholdMeters;
    }

    /** Sets the EMA filter coefficient (0 = no filtering, 1 = max). */
    public void setFilterAlpha(double alpha) {
        this.emaAlpha = Math.max(0, Math.min(1, alpha));
    }

    /** Resets the filter state. */
    public void reset() {
        filteredDistance = 0;
    }
}
