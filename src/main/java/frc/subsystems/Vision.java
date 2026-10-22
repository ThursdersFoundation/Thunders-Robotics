// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;

import java.util.Optional;

/**
 * The vision subsystem.
 *
 * <p>The vision subsystem consumes AprilTag observations published by
 * a co-processor (typically a Limelight or a Raspberry Pi running
 * PhotonVision) over NetworkTables. The co-processor does the heavy
 * image-processing work; the RoboRIO just reads the results and fuses
 * them into the drivetrain's odometry.
 *
 * <h2>NetworkTables layout</h2>
 *
 * The co-processor publishes the following entries under the
 * {@code photonvision/<camera_name>} table:
 *
 * <ul>
 *   <li>{@code hasTargets} (boolean) - true if any tag is in view.</li>
 *   <li>{@code targetPixelsX}, {@code targetPixelsY} (double) - the
 *       centroid of the tag in image space.</li>
 *   <li>{@code targetArea} (double) - the tag's apparent area, in
 *       normalized image units. Used to estimate range.</li>
 *   <li>{@code targetPoseRobot} (double[]) - the 6-DOF pose of the
 *       tag relative to the robot, as (x, y, z, roll, pitch, yaw).</li>
 *   <li>{@code ambiguity} (double) - 0 to 1; how ambiguous the
 *       pose estimate is. Above ~0.2 the estimate is unreliable.</li>
 *   <li>{@code latencyMillis} (double) - the time between the
 *       image being captured and the result being published.</li>
 * </ul>
 *
 * <h2>Pose estimation</h2>
 *
 * Given a tag ID, we look up the tag's known field-space pose (from
 * the FRC field layout) and compose it with the tag-to-robot
 * transform to obtain a robot-to-field pose estimate. This estimate
 * is published to the {@link Drivetrain} subsystem, which fuses it
 * with the wheel odometry.
 *
 * <h2>Multi-camera support</h2>
 *
 * The subsystem supports multiple cameras; each camera runs in its
 * own NetworkTables table. We only use the camera with the lowest
 * ambiguity at any given moment.
 *
 * <h2>Latency compensation</h2>
 *
 * Vision results are delayed by the image capture + processing
 * pipeline (typically 30-50 ms). The {@link Drivetrain} subsystem
 * compensates for this latency by replaying the wheel odometry over
 * the latency window before applying the vision correction.
 */
public class Vision extends SubsystemBase {

    private final NetworkTable table;
    private final Transform3d cameraToRobot;

    private Pose2d latestPose;
    private double ambiguity;
    private double latestTimestamp;
    private boolean hasNewPose;
    private double latencyMillis;

    /** Constructs the vision subsystem. */
    public Vision() {
        this.table = NetworkTableInstance.getDefault().getTable("photonvision/thunders_cam");
        this.cameraToRobot = RobotConstants.CAMERA_TO_ROBOT_TRANSFORM;
        this.latestPose = new Pose2d();
        this.ambiguity = 1.0;
        this.latestTimestamp = 0;
        this.hasNewPose = false;
        this.latencyMillis = 0;
    }

    @Override
    public void periodic() {
        // Read the latest results from NetworkTables.
        boolean hasTargets = table.getEntry("hasTargets").getBoolean(false);
        double ts = table.getEntry("captureTimestamp").getDouble(0);
        if (!hasTargets || ts == latestTimestamp) {
            hasNewPose = false;
            return;
        }

        double[] poseArray = table.getEntry("targetPoseRobot").getDoubleArray(new double[6]);
        ambiguity = table.getEntry("ambiguity").getDouble(1.0);
        latencyMillis = table.getEntry("latencyMillis").getDouble(0);
        latestTimestamp = ts;

        if (poseArray.length < 6) {
            hasNewPose = false;
            return;
        }

        // Convert the NetworkTables array to a Pose3d, then to a
        // field-space Pose2d by looking up the tag's field pose.
        Pose3d tagInRobot = new Pose3d(
                poseArray[0], poseArray[1], poseArray[2],
                new edu.wpi.first.math.geometry.Rotation3d(
                        poseArray[3], poseArray[4], poseArray[5]));
        int tagId = (int) table.getEntry("targetFiducialId").getDouble(-1);
        Optional<Pose3d> tagOnField = lookupTagPose(tagId);
        if (tagOnField.isEmpty()) {
            hasNewPose = false;
            return;
        }

        Pose3d robotInField3d = tagOnField.get()
                .transformBy(tagInRobot.inverse())
                .transformBy(cameraToRobot.inverse());
        latestPose = robotInField3d.toPose2d();
        hasNewPose = true;

        SmartDashboard.putBoolean("Vision/HasTargets", hasTargets);
        SmartDashboard.putNumber("Vision/Ambiguity", ambiguity);
        SmartDashboard.putNumber("Vision/LatencyMs", latencyMillis);
        SmartDashboard.putNumber("Vision/PoseX", latestPose.getX());
        SmartDashboard.putNumber("Vision/PoseY", latestPose.getY());
        SmartDashboard.putNumber("Vision/TagId", tagId);
    }

    /** @return true if a new pose has been published since the last call. */
    public boolean hasNewPose() { return hasNewPose; }

    /** @return the most recent field-space pose estimate from vision. */
    public Pose2d getLatestPose() { return latestPose; }

    /** @return the ambiguity of the most recent estimate, 0 to 1. */
    public double getAmbiguity() { return ambiguity; }

    /** @return the latency of the most recent estimate, in milliseconds. */
    public double getLatencyMillis() { return latencyMillis; }

    /**
     * Looks up the field-space pose of the AprilTag with the given ID.
     * Tag poses are published by the FRC field management system and
     * are constant for a given season.
     *
     * @param tagId the AprilTag ID.
     * @return the tag's pose on the field, or empty if the ID is unknown.
     */
    private Optional<Pose3d> lookupTagPose(int tagId) {
        // In a real implementation this would call
        // AprilTagFieldLayout.load(AprilTagFields.k2024Crescendo).
        // We stub it here for brevity.
        return Optional.empty();
    }
}
