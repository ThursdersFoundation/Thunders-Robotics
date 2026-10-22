// Copyright (c) 2024-2026 Thunders Robotics. All rights reserved.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.DriverStation;

import java.util.Collections;
import java.util.EnumSet;
import java.util.Set;

/**
 * Enumeration and metadata for the autonomous routines available to
 * the drive team.
 *
 * <p>Each routine is identified by a stable enum value (so it can be
 * referenced safely from Java code) and paired with human-readable
 * metadata that is published to the dashboard for the drive team.
 * The actual command that implements the routine is built by
 * {@link frc.robot.robot.RobotContainer#configureAutoOptions()}; this
 * enum is purely a label.
 *
 * <h2>Routine catalogue</h2>
 *
 * <p>The routines are listed below in increasing order of complexity
 * and risk. The drive team should select the simplest routine that
 * still scores points; a complex routine that fails scores zero
 * points, while a simple routine that succeeds scores its expected
 * points.
 *
 * <ul>
 *   <li><b>NONE</b> -&gt; No autonomous. The robot sits motionless for
 *       15 seconds. Use this when nothing else works.</li>
 *   <li><b>LEAVE_COMMUNITY</b> -&gt; Drive forward out of the
 *       community zone. Scores mobility points even if scoring
 *       fails.</li>
 *   <li><b>SCORE_AND_LEAVE</b> -&gt; Score the preloaded piece, then
 *       drive out of the community. Two stages, low risk.</li>
 *   <li><b>SCORE_TWO_BALANCE</b> -&gt; Score preload, acquire and
 *       score a second piece, then balance on the charging station.
 *       Higher risk; only run if the drive team has practised it.</li>
 *   <li><b>MAX_MOBILITY</b> -&gt; Score preload, then race around the
 *       field scoring as many additional pieces as possible. Highest
 *       risk; only run in playoff matches where the alliance needs
 *       the points.</li>
 * </ul>
 *
 * <h2>Adding a new routine</h2>
 *
 * <ol>
 *   <li>Add a new enum value below.</li>
 *   <li>Provide a {@link #getDisplayName()} and {@link #getDescription()}
 *       for the dashboard.</li>
 *   <li>Wire the routine to its command in
 *       {@link frc.robot.robot.RobotContainer#configureAutoOptions()}.</li>
 *   <li>Update the SendableChooser default option to reflect the
 *       most-commonly-used routine for the next match.</li>
 * </ol>
 */
public enum AutoRoutine {

    /** No autonomous action. The robot sits motionless for 15 seconds. */
    NONE("Do Nothing",
            "No autonomous. Use this only if all other routines are broken."),

    /** Drive forward out of the community zone to score mobility points. */
    LEAVE_COMMUNITY("Leave Community",
            "Drive forward out of the community zone. Scores mobility points."),

    /** Score the preloaded piece, then drive out of the community. */
    SCORE_AND_LEAVE("Score Preload, Leave",
            "Score the preloaded game piece, then drive out of the community zone."),

    /** Score two pieces and balance on the charging station. */
    SCORE_TWO_BALANCE("Score 2 + Balance",
            "Score preload, acquire and score a second piece, then balance on the charging station."),

    /** Maximum mobility: score preload, then race around scoring pieces. */
    MAX_MOBILITY("Max Mobility (3 piece)",
            "Score preload, then drive a long path to score up to two additional pieces.");

    private final String displayName;
    private final String description;

    AutoRoutine(String displayName, String description) {
        this.displayName = displayName;
        this.description = description;
    }

    /** @return a short, human-readable name suitable for the dashboard. */
    public String getDisplayName() {
        return displayName;
    }

    /** @return a one-sentence description of what the routine does. */
    public String getDescription() {
        return description;
    }

    /**
     * @return an unmodifiable set of all routines. Used by tests to
     *         verify that every routine is wired to a command.
     */
    public static Set<AutoRoutine> all() {
        return Collections.unmodifiableSet(EnumSet.allOf(AutoRoutine.class));
    }

    /**
     * @return the routine with the given display name, or {@link #NONE}
     *         if no routine matches.
     */
    public static AutoRoutine fromDisplayName(String name) {
        for (AutoRoutine r : values()) {
            if (r.displayName.equalsIgnoreCase(name)) {
                return r;
            }
        }
        DriverStation.reportWarning("[Auto] Unknown routine name: " + name, false);
        return NONE;
    }

    /**
     * @return {@code true} if this routine scores at least one game
     *         piece. Used by alliance strategy code to compute the
     *         expected points total.
     */
    public boolean scoresPoints() {
        return this != NONE && this != LEAVE_COMMUNITY;
    }

    /**
     * @return {@code true} if this routine attempts to balance on the
     *         charging station. Used by alliance strategy code to
     *         avoid three-robot pile-ups on the station.
     */
    public boolean attemptsBalance() {
        return this == SCORE_TWO_BALANCE;
    }
}
