// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.

package frc.robot.sensors;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Abstraction over the robot's primary yaw-rate gyroscope.
 *
 * <p>The Thunders robot uses an ADXRS450 FPGA gyro (the default SPI
 * gyro on the RoboRIO) for yaw measurements. This class wraps the
 * {@link ADXRS450_Gyro} and exposes a simplified API:
 *
 * <ul>
 *   <li>{@link #getAngle()} -&gt; the cumulative yaw angle, in degrees.</li>
 *   <li>{@link #getRate()} -&gt; the instantaneous yaw rate, in deg/s.</li>
 *   <li>{@link #getRotation2d()} -&gt; the yaw as a {@link Rotation2d}.</li>
 *   <li>{@link #zero()} -&gt; resets the yaw to zero.</li>
 * </ul>
 *
 * <h2>Why an abstraction?</h2>
 *
 * Wrapping the gyro in an abstraction lets us swap implementations
 * without touching subsystem code. For example, if the team switches
 * from the ADXRS450 to a NavX or a Pigeon IMU, only this class needs
 * to change; the drivetrain subsystem that consumes the gyro is
 * unchanged.
 *
 * <p>The abstraction also enables testing: a mock {@code Gyroscope}
 * can be substituted in unit tests to simulate gyro drift, saturation,
 * or failure.
 *
 * <h2>Drift</h2>
 *
 * All MEMS gyros drift over time. The ADXRS450 typically drifts
 * ~1 degree per minute, which is acceptable for a 2-3 minute match
 * but unacceptable for long autonomous testing. The drift can be
 * compensated by fusing the gyro with vision-based pose estimation;
 * see {@link frc.robot.subsystems.Vision}.
 *
 * <h2>Calibration</h2>
 *
 * The gyro is calibrated at construction time by averaging ~1 second
 * of stationary readings. The robot MUST be stationary during this
 * period; otherwise the calibration will be wrong and the gyro will
 * report a spurious rate for the rest of the match. The {@link
 * frc.robot.robot.Robot#robotInit()} method ensures the robot is
 * stationary during construction.
 */
public class Gyroscope {

    private final ADXRS450_Gyro gyro;
    private double offsetDegrees;

    /**
     * Constructs a Gyroscope on the default SPI port.
     *
     * @param port the SPI port to use (ignored on the ADXRS450, which
     *             is always on the FPGA SPI port).
     */
    public Gyroscope(int port) {
        this.gyro = new ADXRS450_Gyro();
        this.offsetDegrees = 0;
        gyro.calibrate();
    }

    /**
     * @return the cumulative yaw angle since the last {@link #zero()},
     *         in degrees, in the range (-inf, +inf). A positive angle
     *         indicates a counter-clockwise rotation when viewed from
     *         above.
     */
    public double getAngle() {
        return gyro.getAngle() + offsetDegrees;
    }

    /**
     * @return the instantaneous yaw rate, in deg/s. Positive = CCW.
     */
    public double getRate() {
        return gyro.getRate();
    }

    /**
     * @return the yaw as a {@link Rotation2d}, normalised to [0, 360).
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getAngle());
    }

    /**
     * Resets the yaw to zero. The offset is applied on top of the
     * raw sensor reading so that the next call to {@link #getAngle()}
     * returns zero.
     */
    public void zero() {
        offsetDegrees = -gyro.getAngle();
        SmartDashboard.putNumber("Gyro/OffsetAtZero", offsetDegrees);
    }

    /**
     * Sets the yaw to the given angle. Used to align the gyro with
     * a known pose (e.g., when the robot is placed on the field at a
     * known starting position).
     *
     * @param angleDegrees the desired yaw angle, in degrees.
     */
    public void setAngle(double angleDegrees) {
        offsetDegrees = angleDegrees - gyro.getAngle();
    }

    /**
     * @return the pitch angle of the gyro, in degrees. The ADXRS450
     *         is single-axis (yaw only); pitch is always zero. This
     *         method exists so that callers that expect a multi-axis
     *         IMU can still compile against this class.
     */
    public double getPitch() { return 0; }

    /** @return the roll angle, in degrees. Always zero on the ADXRS450. */
    public double getRoll() { return 0; }
}
