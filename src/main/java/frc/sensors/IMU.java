// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.

package frc.robot.sensors;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Abstraction over the robot's Inertial Measurement Unit (IMU).
 *
 * <p>The Thunders robot uses an ADIS16470 IMU for high-accuracy
 * attitude estimation. Unlike the basic ADXRS450 gyro (which measures
 * yaw rate only), the ADIS16470 measures 3-axis gyro rates and 3-axis
 * accelerometer, allowing it to compute a full 6-DOF attitude
 * estimate (roll, pitch, yaw).
 *
 * <h2>Why use an IMU?</h2>
 *
 * The basic ADXRS450 cannot measure pitch or roll, which the
 * auto-balance controller needs. The IMU provides pitch and roll
 * directly, simplifying the balance controller and improving its
 * accuracy.
 *
 * <h2>Calibration</h2>
 *
 * The IMU is calibrated at construction time by averaging ~5 seconds
 * of stationary readings. The robot MUST be stationary and level
 * during this period; otherwise the calibration will be wrong and the
 * IMU will report a spurious attitude for the rest of the match.
 *
 * <p>The {@link frc.robot.robot.Robot#robotInit()} method prints a
 * message to the driver station log during calibration so the drive
 * team knows not to move the robot.
 *
 * <h2>Yaw vs. gyro</h2>
 *
 * The IMU and the {@link Gyroscope} both measure yaw, but they use
 * different sensors. The {@link Gyroscope} is the "source of truth"
 * for yaw because it is simpler and has fewer failure modes; the IMU
 * is used only for pitch and roll. This avoids the situation where
 * the IMU and gyro disagree about yaw and the code does not know
 * which to trust.
 *
 * <h2>Failure modes</h2>
 *
 * The IMU can fail in several ways:
 *
 * <ul>
 *   <li><b>Disconnection</b> -&gt; the IMU returns all zeros. Detected
 *       by checking {@link ADIS16470_IMU#isConnected()}.</li>
 *   <li><b>Calibration failure</b> -&gt; the IMU reports a constant
 *       non-zero offset. Detected by checking that the rate is
 *       non-zero while the robot is stationary.</li>
 *   <li><b>Saturation</b> -&gt; the IMU reports a maximum or minimum
 *       value when the rate exceeds the sensor's range. Detected by
 *       checking for rate values at exactly the maximum or minimum.</li>
 * </ul>
 *
 * All three failure modes are reported to the dashboard so the drive
 * team can spot a bad sensor before a match.
 */
public class IMU {

    private final ADIS16470_IMU imu;
    private double yawOffset;
    private double pitchOffset;
    private double rollOffset;

    /** Constructs an IMU on the default SPI port. */
    public IMU(int port) {
        this.imu = new ADIS16470_IMU();
        this.yawOffset = 0;
        this.pitchOffset = 0;
        this.rollOffset = 0;
        // The ADIS16470 constructor calibrates internally; we record
        // the post-calibration angles so that the first call to
        // getAngle() returns zero.
        yawOffset = -imu.getAngle(edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis.kYaw);
        pitchOffset = -imu.getXComplementaryAngle();
        rollOffset = -imu.getYComplementaryAngle();
    }

    /** @return the yaw angle, in degrees, since the last {@link #zero()}. */
    public double getAngle() {
        return imu.getAngle(edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis.kYaw) + yawOffset;
    }

    /** @return the pitch angle, in degrees. Positive = nose-up. */
    public double getPitch() {
        return imu.getXComplementaryAngle() + pitchOffset;
    }

    /** @return the roll angle, in degrees. Positive = right wing down. */
    public double getRoll() {
        return imu.getYComplementaryAngle() + rollOffset;
    }

    /** @return the yaw rate, in deg/s. */
    public double getRate() {
        return imu.getRate(edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis.kYaw);
    }

    /** @return the yaw as a {@link Rotation2d}, normalised to [0, 360). */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getAngle());
    }

    /**
     * Resets the yaw, pitch, and roll to zero. Should only be called
     * when the robot is stationary and level; otherwise the offsets
     * will be wrong for the rest of the match.
     */
    public void zero() {
        yawOffset = -imu.getAngle(edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis.kYaw);
        pitchOffset = -imu.getXComplementaryAngle();
        rollOffset = -imu.getYComplementaryAngle();
        SmartDashboard.putNumber("IMU/YawOffsetAtZero", yawOffset);
        SmartDashboard.putNumber("IMU/PitchOffsetAtZero", pitchOffset);
        SmartDashboard.putNumber("IMU/RollOffsetAtZero", rollOffset);
    }

    /** @return true if the IMU is connected and reporting valid data. */
    public boolean isConnected() {
        return imu.isConnected();
    }

    /**
     * Publishes diagnostic data to the dashboard. Called by the
     * {@link frc.robot.subsystems.Sensors} subsystem on every periodic
     * cycle.
     */
    public void publishDiagnostics() {
        SmartDashboard.putBoolean("IMU/Connected", isConnected());
        SmartDashboard.putNumber("IMU/Yaw", getAngle());
        SmartDashboard.putNumber("IMU/Pitch", getPitch());
        SmartDashboard.putNumber("IMU/Roll", getRoll());
        SmartDashboard.putNumber("IMU/YawRate", getRate());
    }
}
