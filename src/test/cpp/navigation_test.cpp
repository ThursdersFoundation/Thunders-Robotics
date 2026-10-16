// ============================================================================
// navigation_test.cpp
// ----------------------------------------------------------------------------
// Repository : Thunders Robotics
// Package    : thunders::navigation
// Framework  : Standalone C++17  (no ROS dependency in the planner)
// Test Tool  : GoogleTest (gtest)
// ----------------------------------------------------------------------------
// Coverage
//   1. Waypoint path           - distance / cumulative length
//   2. Odometry pose tracking  - integrate Twist into Pose2D
//   3. Pure pursuit            - lookahead point selection + steering
//   4. Obstacle avoidance      - repulsion vector from a single obstacle
//   5. TF transform chain      - 2D transform composition & inversion
// ----------------------------------------------------------------------------
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace thunders::navigation {
namespace {

// =========================================================================
// TYPES
// =========================================================================

struct Point2D { double x; double y; };
struct Pose2D  { double x; double y; double theta; };
struct Twist2D { double v;   double w; };   // linear.x, angular.z

struct Transform2D {
    double dx, dy, dtheta;

    Pose2D apply(const Pose2D& p) const {
        double c = std::cos(dtheta);
        double s = std::sin(dtheta);
        return Pose2D{
            p.x * c - p.y * s + dx,
            p.x * s + p.y * c + dy,
            p.theta + dtheta
        };
    }

    Transform2D compose(const Transform2D& other) const {
        // Combine this -> other: rotate other's translation by this->dtheta.
        double c = std::cos(dtheta);
        double s = std::sin(dtheta);
        return Transform2D{
            dx + c * other.dx - s * other.dy,
            dy + s * other.dx + c * other.dy,
            dtheta + other.dtheta
        };
    }

    Transform2D inverse() const {
        double c = std::cos(-dtheta);
        double s = std::sin(-dtheta);
        return Transform2D{
            -(c * dx - s * dy),
            -(s * dx + c * dy),
            -dtheta
        };
    }
};

// =========================================================================
// HELPERS
// =========================================================================

inline double distance(const Point2D& a, const Point2D& b) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
}

inline double deg2rad(double deg) { return deg * M_PI / 180.0; }

/** Cumulative length of a polyline through the waypoints. */
double path_length(const std::vector<Point2D>& wps) {
    if (wps.size() < 2) return 0.0;
    double total = 0.0;
    for (size_t i = 1; i < wps.size(); ++i) {
        total += distance(wps[i - 1], wps[i]);
    }
    return total;
}

/** Integrate a body-frame Twist into a Pose2D over dt seconds. */
Pose2D integrate_pose(const Pose2D& start, const Twist2D& twist, double dt) {
    Pose2D out = start;
    if (std::abs(twist.w) < 1e-9) {
        out.x     += twist.v * std::cos(start.theta) * dt;
        out.y     += twist.v * std::sin(start.theta) * dt;
        out.theta += 0.0;
    } else {
        double r = twist.v / twist.w;
        out.theta += twist.w * dt;
        out.x += r * (std::sin(out.theta) - std::sin(start.theta));
        out.y -= r * (std::cos(out.theta) - std::cos(start.theta));
    }
    return out;
}

/** Find the lookahead point on the path for pure pursuit. */
Point2D lookahead_point(const std::vector<Point2D>& path,
                        const Point2D& robot, double lookahead) {
    Point2D best = path.back();
    double best_dist = 0.0;
    for (const auto& wp : path) {
        double d = distance(robot, wp);
        if (d >= lookahead && d > best_dist) {
            best = wp;
            best_dist = d;
        }
    }
    return best;
}

/** Repulsion vector away from an obstacle (inverse to distance squared). */
Point2D repulsion_vector(const Point2D& robot,
                         const Point2D& obstacle,
                         double gain) {
    double dx = robot.x - obstacle.x;
    double dy = robot.y - obstacle.y;
    double d2 = dx * dx + dy * dy;
    if (d2 < 1e-9) return {gain * 1e6, 0.0};
    double d = std::sqrt(d2);
    double mag = gain / d2;
    return {mag * dx / d, mag * dy / d};
}

}  // namespace
}  // namespace thunders::navigation

// =========================================================================
// 1. WAYPOINT PATH
// =========================================================================
using thunders::navigation::Point2D;
using thunders::navigation::path_length;

TEST(WaypointPathTest, EmptyPathHasZeroLength) {
    EXPECT_DOUBLE_EQ(0.0, path_length({}));
}

TEST(WaypointPathTest, SingleWaypointHasZeroLength) {
    EXPECT_DOUBLE_EQ(0.0, path_length({{1.0, 2.0}}));
}

TEST(WaypointPathTest, TwoPointsGiveEuclideanDistance) {
    std::vector<Point2D> wps = {{0.0, 0.0}, {3.0, 4.0}};
    EXPECT_DOUBLE_EQ(5.0, path_length(wps));
}

TEST(WaypointPathTest, MultipleWaypointsSumDistances) {
    std::vector<Point2D> wps = {
        {0.0, 0.0},
        {3.0, 0.0},   // +3
        {3.0, 4.0}    // +4
    };
    EXPECT_DOUBLE_EQ(7.0, path_length(wps));
}

// =========================================================================
// 2. ODOMETRY POSE TRACKING
// =========================================================================
using thunders::navigation::Pose2D;
using thunders::navigation::Twist2D;
using thunders::navigation::integrate_pose;

TEST(OdometryTest, StraightLineMotionUpdatesXOnly) {
    Pose2D p{0.0, 0.0, 0.0};
    Twist2D t{1.0, 0.0};
    p = integrate_pose(p, t, 1.0);
    EXPECT_NEAR(1.0, p.x, 1e-9);
    EXPECT_NEAR(0.0, p.y, 1e-9);
    EXPECT_NEAR(0.0, p.theta, 1e-9);
}

TEST(OdometryTest, ForwardAtTheta45UpdatesXAndYEqually) {
    Pose2D p{0.0, 0.0, M_PI / 4.0};
    Twist2D t{1.0, 0.0};
    p = integrate_pose(p, t, 1.0);
    EXPECT_NEAR(std::sqrt(0.5), p.x, 1e-6);
    EXPECT_NEAR(std::sqrt(0.5), p.y, 1e-6);
}

TEST(OdometryTest, PureRotationUpdatesThetaOnly) {
    Pose2D p{1.0, 2.0, 0.0};
    Twist2D t{0.0, 0.5};
    p = integrate_pose(p, t, 1.0);
    EXPECT_NEAR(0.5, p.theta, 1e-9);
    EXPECT_NEAR(1.0, p.x, 1e-9);
    EXPECT_NEAR(2.0, p.y, 1e-9);
}

TEST(OdometryTest, CircularArcEndsAtExpectedPosition) {
    // Quarter circle of radius 1: v=1, w=1, dt=pi/2 ends at (1, 1) - 1 = (1, 0)
    // Start at origin, heading +x. v/w = 1 -> radius 1.
    Pose2D p{0.0, 0.0, 0.0};
    Twist2D t{1.0, 1.0};
    p = integrate_pose(p, t, M_PI / 2.0);
    EXPECT_NEAR(1.0, p.x, 1e-6);
    EXPECT_NEAR(1.0, p.y, 1e-6);
    EXPECT_NEAR(M_PI / 2.0, p.theta, 1e-6);
}

// =========================================================================
// 3. PURE PURSUIT LOOKAHEAD
// =========================================================================
using thunders::navigation::lookahead_point;

TEST(PurePursuitTest, ReturnsFirstPointBeyondLookahead) {
    std::vector<Point2D> path = {
        {0.0, 0.0},
        {0.5, 0.0},
        {1.0, 0.0},
        {2.0, 0.0}
    };
    Point2D robot{0.0, 0.0};
    Point2D la = lookahead_point(path, robot, 0.7);
    EXPECT_NEAR(1.0, la.x, 1e-9);
}

TEST(PurePursuitTest, FallsBackToLastWaypointIfNoneBeyondLookahead) {
    std::vector<Point2D> path = {
        {0.0, 0.0},
        {0.2, 0.0},
        {0.3, 0.0}
    };
    Point2D robot{0.0, 0.0};
    Point2D la = lookahead_point(path, robot, 5.0);
    EXPECT_NEAR(0.3, la.x, 1e-9);
}

// =========================================================================
// 4. OBSTACLE AVOIDANCE
// =========================================================================
using thunders::navigation::repulsion_vector;

TEST(ObstacleAvoidanceTest, RepulsionPointsAwayFromObstacle) {
    Point2D robot{1.0, 0.0};
    Point2D obs{0.0, 0.0};
    Point2D r = repulsion_vector(robot, obs, 1.0);
    EXPECT_GT(r.x, 0.0);   // away from origin (positive x)
    EXPECT_NEAR(0.0, r.y, 1e-9);
}

TEST(ObstacleAvoidanceTest, CloserObstacleGivesStrongerRepulsion) {
    Point2D obs{0.0, 0.0};
    Point2D near_r{1.0, 0.0};
    Point2D far_r{5.0, 0.0};
    Point2D near_rep = repulsion_vector(near_r, obs, 1.0);
    Point2D far_rep  = repulsion_vector(far_r,  obs, 1.0);
    EXPECT_GT(near_rep.x, far_rep.x);
}

TEST(ObstacleAvoidanceTest, ObstacleDeadAheadRepulsionIsPureX) {
    Point2D robot{2.0, 0.0};
    Point2D obs{0.0, 0.0};
    Point2D r = repulsion_vector(robot, obs, 1.0);
    EXPECT_NEAR(0.0, r.y, 1e-9);
    EXPECT_GT(r.x, 0.0);
}

// =========================================================================
// 5. TF TRANSFORM CHAIN
// =========================================================================
using thunders::navigation::Transform2D;

TEST(Transform2DTest, PureTranslationMovesPose) {
    Transform2D t{1.0, 2.0, 0.0};
    Pose2D p{0.0, 0.0, 0.0};
    Pose2D q = t.apply(p);
    EXPECT_NEAR(1.0, q.x, 1e-9);
    EXPECT_NEAR(2.0, q.y, 1e-9);
}

TEST(Transform2DTest, PureRotationUpdatesTheta) {
    Transform2D t{0.0, 0.0, M_PI / 2.0};
    Pose2D p{0.0, 0.0, 0.0};
    Pose2D q = t.apply(p);
    EXPECT_NEAR(M_PI / 2.0, q.theta, 1e-9);
}

TEST(Transform2DTest, CompositionIsAssociative) {
    Transform2D a{1.0, 0.0, 0.0};
    Transform2D b{0.0, 1.0, 0.0};
    Transform2D ab = a.compose(b);
    Pose2D p{0.0, 0.0, 0.0};
    Pose2D q = ab.apply(p);
    EXPECT_NEAR(1.0, q.x, 1e-9);
    EXPECT_NEAR(1.0, q.y, 1e-9);
}

TEST(Transform2DTest, InverseCancelsForward) {
    Transform2D t{1.0, 2.0, M_PI / 4.0};
    Transform2D inv = t.inverse();
    Pose2D p{3.0, 4.0, 0.5};
    Pose2D q = inv.apply(t.apply(p));
    EXPECT_NEAR(p.x, q.x, 1e-9);
    EXPECT_NEAR(p.y, q.y, 1e-9);
    EXPECT_NEAR(p.theta, q.theta, 1e-9);
}

TEST(Transform2DTest, RotationThenTranslationMatchesExpected) {
    // Rotate +90deg then translate 1 in x' (which is now world +y).
    Transform2D rot{0.0, 0.0, M_PI / 2.0};
    Transform2D trans{1.0, 0.0, 0.0};
    Transform2D composed = rot.compose(trans);
    Pose2D p{0.0, 0.0, 0.0};
    Pose2D q = composed.apply(p);
    EXPECT_NEAR(0.0, q.x, 1e-9);
    EXPECT_NEAR(1.0, q.y, 1e-9);
    EXPECT_NEAR(M_PI / 2.0, q.theta, 1e-9);
}

// Entry point for standalone execution.
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
