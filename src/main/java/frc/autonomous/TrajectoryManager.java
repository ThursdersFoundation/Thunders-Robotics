// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.

package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.RobotConstants;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Loads and caches trajectories for autonomous and teleop-assist
 * commands.
 *
 * <p>Generating a WPILib {@link Trajectory} from a list of waypoints
 * is expensive (~5-20 ms on the RoboRIO, depending on length). To
 * avoid doing this on every autonomous init or every time a teleop
 * assist command is scheduled, we cache trajectories in this manager
 * keyed by name.
 *
 * <h2>Trajectory sources</h2>
 *
 * Trajectories come from two sources:
 *
 * <ol>
 *   <li><b>PathPlanner files</b> -&gt; trajectories authored in the
 *       PathPlanner web app and exported to {@code deploy/pathplanner/*.path}.
 *       Loaded by {@link PathPlanner}.</li>
 *   <li><b>Inline trajectories</b> -&gt; trajectories defined in Java
 *       code for simple, predictable paths (e.g., "drive straight
 *       forward 3 metres"). Defined in this class's
 *       {@link #buildInlineTrajectory(String)} method.</li>
 * </ol>
 *
 * <h2>Cache invalidation</h2>
 *
 * The cache is keyed by name and never invalidated during a robot
 * run. PathPlanner files are loaded from the deploy directory, which
 * is fixed at deploy time, so there is no need to invalidate on file
 * changes.
 *
 * <h2>Thread safety</h2>
 *
 * The manager is NOT thread-safe. It is intended to be used only from
 * the WPILib periodic thread. If a trajectory needs to be generated
 * off-thread (e.g., for a vision-assisted dynamic path), the caller
 * should generate it locally and not use this cache.
 */
public class TrajectoryManager {

    private final Map<String, Trajectory> cache = new HashMap<>();
    private final PathPlanner pathPlanner;

    /** Constructs a TrajectoryManager backed by a default {@link PathPlanner}. */
    public TrajectoryManager() {
        this(new PathPlanner());
    }

    /** Constructs a TrajectoryManager with the given {@link PathPlanner}. Used by tests. */
    public TrajectoryManager(PathPlanner pathPlanner) {
        this.pathPlanner = pathPlanner;
    }

    /**
     * Returns the trajectory with the given name, loading and caching
     * it on first access.
     *
     * @param name the trajectory name. If a PathPlanner file with
     *             this name exists, it is loaded; otherwise the
     *             manager attempts to build the trajectory inline
     *             via {@link #buildInlineTrajectory(String)}.
     * @return the trajectory, or {@code null} if it could not be loaded.
     */
    public Trajectory get(String name) {
        if (cache.containsKey(name)) {
            return cache.get(name);
        }
        Trajectory t = pathPlanner.loadPath(name);
        if (t == null) {
            t = buildInlineTrajectory(name);
        }
        if (t != null) {
            cache.put(name, t);
        }
        return t;
    }

    /**
     * Builds simple inline trajectories for known names. Inline
     * trajectories are useful for development: they let the team test
     * the autonomous framework before any PathPlanner paths have been
     * authored.
     *
     * <p>The following names are supported:
     *
     * <ul>
     *   <li>{@code LeaveCommunity} -&gt; drive straight forward 3.0 m.</li>
     *   <li>{@code ScoreTwoBalance} -&gt; drive to the second game piece
     *       and back, with a holonomic rotation at each end.</li>
     *   <li>{@code MaxMobility} -&gt; a longer path that visits two
     *       scoring positions.</li>
     * </ul>
     *
     * @param name the trajectory name.
     * @return the trajectory, or {@code null} if the name is unknown.
     */
    protected Trajectory buildInlineTrajectory(String name) {
        TrajectoryConfig config = new TrajectoryConfig(
                RobotConstants.MAX_DRIVE_SPEED_MPS,
                RobotConstants.MAX_DRIVE_ACCEL_M_PER_S2)
                .addConstraint(new CentripetalAccelerationConstraint(
                        RobotConstants.MAX_CENTRIPETAL_ACCEL_M_PER_S2));
        config.setKinematics(null); // kinematics set by the caller.

        switch (name) {
            case "LeaveCommunity":
                return TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                        List.of(),
                        new Pose2d(3.0, 0, Rotation2d.fromDegrees(0)),
                        config);
            case "ScoreTwoBalance":
                return TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                        List.of(new edu.wpi.first.math.geometry.Translation2d(1.5, 0.5)),
                        new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(90)),
                        config);
            case "MaxMobility":
                return TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                        List.of(
                                new edu.wpi.first.math.geometry.Translation2d(2.0, 0.5),
                                new edu.wpi.first.math.geometry.Translation2d(4.0, 0.0),
                                new edu.wpi.first.math.geometry.Translation2d(2.0, -0.5)),
                        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180)),
                        config);
            case "AcquireAndScore":
                return TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                        List.of(new edu.wpi.first.math.geometry.Translation2d(1.0, 0.0)),
                        new Pose2d(2.0, 0.5, Rotation2d.fromDegrees(45)),
                        config);
            default:
                DriverStation.reportWarning(
                        "[TrajectoryManager] Unknown trajectory: " + name, false);
                return null;
        }
    }

    /**
     * Returns the names of all trajectories currently in the cache.
     * Used by the dashboard to display which trajectories are loaded.
     *
     * @return an unmodifiable list of cached trajectory names.
     */
    public List<String> cachedNames() {
        return new ArrayList<>(cache.keySet());
    }

    /** Clears the cache. Used by tests. */
    public void clearCache() {
        cache.clear();
    }

    /** @return the number of trajectories currently cached. */
    public int size() {
        return cache.size();
    }
}
