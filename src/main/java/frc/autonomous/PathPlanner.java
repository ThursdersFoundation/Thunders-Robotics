// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.

package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.RobotConstants;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * Wrapper around the PathPlanner path-format reader.
 *
 * <p>PathPlanner is a popular third-party tool that lets drive teams
 * design trajectories visually in a web app and export them as JSON
 * files. The JSON files are placed in the {@code deploy/pathplanner}
 * directory of the robot project and loaded at runtime by this class.
 *
 * <h2>Why use PathPlanner instead of WPILib trajectories?</h2>
 *
 * WPILib's built-in {@link TrajectoryGenerator} produces perfectly good
 * trajectories, but generating them in code is verbose: every waypoint
 * and holonomic rotation must be specified in Java, which makes tuning
 * the path a slow edit-compile-deploy loop. PathPlanner lets the drive
 * team drag waypoints in a GUI and see the trajectory update in real
 * time, then export the result as a JSON file that the robot loads at
 * startup.
 *
 * <h2>Coordinate frames</h2>
 *
 * PathPlanner paths are authored in field space: the origin is the
 * corner of the field, and the X axis points toward the opposing
 * alliance wall. When the robot is on the red alliance, the path is
 * mirrored about the field's centreline. The mirroring is handled by
 * PathPlannerLib at load time; this class is alliance-agnostic.
 *
 * <h2>Caching</h2>
 *
 * Loading and parsing a PathPlanner JSON file takes ~10 ms on the
 * RoboRIO. To avoid doing this on every autonomous init, we cache
 * loaded paths in a {@link java.util.Map} keyed by file name. The
 * cache is invalidated only when the robot code is restarted.
 *
 * <h2>Error handling</h2>
 *
 * If a path file is missing or malformed, this class logs a warning
 * and returns {@code null}. Callers must handle the null gracefully —
 * typically by falling back to a simpler routine or sitting still.
 */
public class PathPlanner {

    private final java.util.Map<String, Trajectory> cache = new java.util.HashMap<>();
    private final Path deployPath;

    /** Constructs a PathPlanner that reads from the standard deploy directory. */
    public PathPlanner() {
        this(Filesystem.getDeployDirectory().toPath().resolve("pathplanner"));
    }

    /** Constructs a PathPlanner that reads from a custom directory. Used by tests. */
    public PathPlanner(Path deployPath) {
        this.deployPath = deployPath;
    }

    /**
     * Loads the path with the given name from disk, caching the result.
     *
     * @param name the path name, without the {@code .path} extension.
     * @return the loaded trajectory, or {@code null} if loading failed.
     */
    public Trajectory loadPath(String name) {
        if (cache.containsKey(name)) {
            return cache.get(name);
        }
        Path file = deployPath.resolve(name + ".path");
        if (!Files.exists(file)) {
            DriverStation.reportError("[PathPlanner] File not found: " + file, false);
            return null;
        }
        try {
            String json = Files.readString(file);
            Trajectory t = parsePathPlannerJson(json);
            cache.put(name, t);
            return t;
        } catch (IOException ex) {
            DriverStation.reportError("[PathPlanner] Failed to read " + file + ": " + ex, false);
            return null;
        }
    }

    /**
     * Parses a PathPlanner JSON string into a WPILib {@link Trajectory}.
     *
     * <p>The PathPlanner format is documented at
     * <a href="https://pathplanner.dev">pathplanner.dev</a>. The key
     * fields are:
     *
     * <ul>
     *   <li>{@code waypoints} - the list of anchor points and control
     *       points that define the spline.</li>
     *   <li>{@code globalConstraints} - the maximum velocity and
     *       acceleration, with optional per-segment overrides.</li>
     *   <li>{@code rotationTargets} - the holonomic heading targets
     *       along the path.</li>
     * </ul>
     *
     * <p>This implementation is a stub; the real implementation would
     * use a JSON parser to construct the trajectory. For brevity we
     * build a simple two-point trajectory using the WPILib generator.
     *
     * @param json the PathPlanner JSON string.
     * @return the trajectory.
     */
    private Trajectory parsePathPlannerJson(String json) {
        // In production this would use com.pathplanner.lib.PathPlanner.
        // For this template we synthesise a simple trajectory.
        TrajectoryConfig config = new TrajectoryConfig(
                RobotConstants.MAX_DRIVE_SPEED_MPS,
                RobotConstants.MAX_DRIVE_ACCEL_M_PER_S2)
                .addConstraint(new CentripetalAccelerationConstraint(
                        RobotConstants.MAX_CENTRIPETAL_ACCEL_M_PER_S2));
        List<Translation2d> interior = new ArrayList<>();
        return TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                interior,
                new Pose2d(3.0, 0, Rotation2d.fromDegrees(0)),
                config);
    }

    /** Clears the cache. Used by tests between runs. */
    public void clearCache() {
        cache.clear();
    }

    /** @return the number of paths currently cached. */
    public int cacheSize() {
        return cache.size();
    }
}
