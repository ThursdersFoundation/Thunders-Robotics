// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.SensorConstants;
import frc.robot.sensors.DistanceSensor;
import frc.robot.sensors.Encoder;
import frc.robot.sensors.Gyroscope;
import frc.robot.sensors.IMU;

/**
 * The sensor aggregator subsystem.
 *
 * <p>The Thunders robot has a large number of sensors distributed
 * across its subsystems: a gyro, four drive encoders, two arm
 * encoders, an elevator string-pot, two elevator limit switches, a
 * beam-break for the intake, and a power distribution panel. Rather
 * than have each subsystem own and initialise its sensors directly,
 * we centralise them in this aggregator so that:
 *
 * <ol>
 *   <li>Initialisation order is explicit and easy to reason about.</li>
 *   <li>Sensor values that are needed by multiple subsystems (e.g.,
 *       the gyro is needed by both the drivetrain and the auto-balance
 *       controller) are read from a single source of truth.</li>
 *   <li>Tests can substitute mock sensors in one place rather than
 *       threading mocks through every subsystem constructor.</li>
 *   <li>Diagnostic data (PDP currents, brownout status, sensor
 *       health) can be published to the dashboard from a single
 *       periodic method.</li>
 * </ol>
 *
 * <h2>Periodic responsibilities</h2>
 *
 * The aggregator's periodic method does two things:
 *
 * <ol>
 *   <li>Publishes diagnostic data from the PDP, the gyro, and the
 *       brownout detector to the dashboard.</li>
 *   <li>Updates a {@link frc.robot.robot.RobotState} snapshot with
 *       sensor values that the rest of the code needs to consume.</li>
 * </ol>
 *
 * Subsystem-specific sensor values (e.g., the arm encoder positions)
 * are published by their owning subsystems, not by this aggregator;
 * this avoids a giant publish-everything method that is hard to
 * maintain.
 *
 * <h2>Brownout detection</h2>
 *
 * The RoboRIO cuts motor outputs when the battery voltage drops below
 * 6.5 V to prevent the robot from browning out. We detect this
 * condition by subscribing to the {@link edu.wpi.first.wpilibj.RobotController}
 * class's brownout flag and publishing it to the
 * {@link frc.robot.robot.RobotState}, where commands can check it
 * before scheduling expensive operations.
 *
 * <h2>Power distribution</h2>
 *
 * The PDP (or PDH, on newer robots) reports per-channel current and
 * voltage. We publish the total current and the battery voltage to
 * the dashboard so the drive team can spot a failing battery or a
 * shorted motor before it affects performance.
 */
public class Sensors extends SubsystemBase {

    private final Gyroscope gyro;
    private final IMU imu;
    private final Encoder armShoulderEncoder;
    private final Encoder armElbowEncoder;
    private final Encoder elevatorEncoder;
    private final DistanceSensor elevatorBottomSwitch;
    private final DistanceSensor elevatorTopSwitch;
    private final PowerDistribution pdp;

    /** Constructs the sensor aggregator and initialises every sensor. */
    public Sensors() {
        this.gyro = new Gyroscope(SensorConstants.GYRO_PORT);
        this.imu = new IMU(SensorConstants.IMU_PORT);
        this.armShoulderEncoder = new Encoder(SensorConstants.ARM_SHOULDER_ENCODER_PORT,
                SensorConstants.ARM_SHOULDER_ENCODER_OFFSET_DEG);
        this.armElbowEncoder = new Encoder(SensorConstants.ARM_ELBOW_ENCODER_PORT,
                SensorConstants.ARM_ELBOW_ENCODER_OFFSET_DEG);
        this.elevatorEncoder = new Encoder(SensorConstants.ELEVATOR_ENCODER_PORT, 0);
        this.elevatorBottomSwitch = new DistanceSensor(SensorConstants.ELEVATOR_BOTTOM_SWITCH_PORT);
        this.elevatorTopSwitch = new DistanceSensor(SensorConstants.ELEVATOR_TOP_SWITCH_PORT);
        this.pdp = new PowerDistribution(SensorConstants.PDP_ID,
                PowerDistribution.ModuleType.kCTRE);
    }

    /** @return the gyro (used by the drivetrain for odometry). */
    public Gyroscope getGyro() { return gyro; }

    /** @return the IMU (used by the balance controller). */
    public IMU getImu() { return imu; }

    public Encoder getArmShoulderEncoder() { return armShoulderEncoder; }
    public Encoder getArmElbowEncoder() { return armElbowEncoder; }
    public Encoder getElevatorEncoder() { return elevatorEncoder; }
    public DistanceSensor getElevatorBottomSwitch() { return elevatorBottomSwitch; }
    public DistanceSensor getElevatorTopSwitch() { return elevatorTopSwitch; }

    /** @return true if the beam-break sensor currently sees a game piece. */
    public boolean isGamePieceDetected() {
        // The beam-break is wired directly to a DIO port; reading it
        // here would require owning the DigitalInput. In practice we
        // let the Intake subsystem own that sensor and read from there
        // via the robot state. This method is provided for tests that
        // want to mock the sensor.
        return false;
    }

    @Override
    public void periodic() {
        // Brownout detection.
        boolean brownout = edu.wpi.first.wpilibj.RobotController.isBrownedOut();
        frc.robot.robot.RobotState.INSTANCE.setBrownoutActive(brownout);

        // Power distribution telemetry.
        double batteryVoltage = pdp.getVoltage();
        double totalCurrent = pdp.getTotalCurrent();
        double temperature = pdp.getTemperature();

        SmartDashboard.putNumber("Sensors/BatteryVoltage", batteryVoltage);
        SmartDashboard.putNumber("Sensors/TotalCurrent", totalCurrent);
        SmartDashboard.putNumber("Sensors/PDPTemperature", temperature);
        SmartDashboard.putBoolean("Sensors/Brownout", brownout);
        SmartDashboard.putNumber("Sensors/GyroYaw", gyro.getAngle());
        SmartDashboard.putNumber("Sensors/IMUPitch", imu.getPitch());
        SmartDashboard.putNumber("Sensors/IMURoll", imu.getRoll());

        // Update the robot state with sensor values that are read by
        // multiple subsystems. Per-subsystem values (e.g., the arm
        // encoder positions) are read directly from the encoder.
        frc.robot.robot.RobotState.INSTANCE.setPitchDegrees(imu.getPitch());
        frc.robot.robot.RobotState.INSTANCE.setRollDegrees(imu.getRoll());
        frc.robot.robot.RobotState.INSTANCE.setHeading(gyro.getRotation2d());
    }
}
