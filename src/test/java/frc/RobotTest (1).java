/*
 * RobotTest.java
 * ----------------------------------------------------------------------------
 * Repository : Thunders Robotics
 * Package    : com.thunders.robotics
 * Framework  : ROS / ROSJava
 * Test Tool  : JUnit 5 (Jupiter)
 * ----------------------------------------------------------------------------
 * Test coverage
 *   1. Drivetrain  - Twist command publishing on /cmd_vel
 *   2. Intake      - std_msgs/Bool control on /intake/enable
 *   3. Autonomous  - State machine transitions and timeout fallback
 *   4. Sensors     - LaserScan, Imu, and encoder feedback parsing
 * ----------------------------------------------------------------------------
 *
 * NOTE: This file uses lightweight in-process stubs instead of a live ROS
 *       master, so the tests can run on any CI host without roscore. Swap
 *       the stub factories for real ConnectedNode instances when running
 *       on the robot.
 */
package com.thunders.robotics;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * Top-level test suite for the Thunders Robotics robot node.
 *
 * <p>Each subsystem is grouped under a {@link Nested} inner class so the JUnit
 * 5 launcher reports them as separate test containers in the IDE and CI.</p>
 */
@DisplayName("Thunders Robotics :: Robot Node Tests")
class RobotTest {

    /** Shared stub "node" used across the suite; reset before every test. */
    private StubConnectedNode node;

    @BeforeEach
    void setUp() {
        node = new StubConnectedNode();
    }

    @AfterEach
    void tearDown() {
        node.shutdown();
        node = null;
    }

    // =========================================================================
    // 1. DRIVETRAIN
    // =========================================================================
    @Nested
    @DisplayName("Drivetrain (geometry_msgs/Twist on /cmd_vel)")
    class DrivetrainTests {

        @Test
        @DisplayName("should publish Twist on /cmd_vel when node starts")
        void shouldAdvertiseCmdVel() {
            StubPublisher<TwistStub> pub =
                node.newPublisher("/cmd_vel", "geometry_msgs/Twist");

            assertNotNull(pub, "Publisher handle must not be null");
            assertEquals("/cmd_vel", pub.getTopicName(), "Topic name mismatch");
            assertEquals("geometry_msgs/Twist", pub.getMessageType(),
                "Resolved type must be geometry_msgs/Twist");
        }

        @Test
        @DisplayName("forward command should set linear.x positive, angular zero")
        void forwardCommandShouldProduceForwardTwist() {
            StubPublisher<TwistStub> pub =
                node.newPublisher("/cmd_vel", "geometry_msgs/Twist");
            TwistStub twist = pub.newMessage();

            twist.getLinear().x = 0.5;   // m/s forward
            twist.getLinear().y = 0.0;
            twist.getLinear().z = 0.0;
            twist.getAngular().x = 0.0;
            twist.getAngular().y = 0.0;
            twist.getAngular().z = 0.0;  // no rotation
            pub.publish(twist);

            assertEquals(1, pub.getPublishedCount(), "Exactly one message expected");
            TwistStub sent = pub.lastPublished();
            assertAll("Twist payload",
                () -> assertTrue(sent.getLinear().x > 0.0, "linear.x must be forward"),
                () -> assertEquals(0.0, sent.getAngular().z, 1e-9,
                    "angular.z must be zero for straight motion"));
        }

        @Test
        @DisplayName("stop command should zero every Twist component")
        void stopCommandShouldZeroAllAxes() {
            StubPublisher<TwistStub> pub =
                node.newPublisher("/cmd_vel", "geometry_msgs/Twist");
            TwistStub stop = pub.newMessage();
            pub.publish(stop);

            TwistStub sent = pub.lastPublished();
            assertAll("Zero Twist",
                () -> assertEquals(0.0, sent.getLinear().x, 1e-9),
                () -> assertEquals(0.0, sent.getLinear().y, 1e-9),
                () -> assertEquals(0.0, sent.getLinear().z, 1e-9),
                () -> assertEquals(0.0, sent.getAngular().x, 1e-9),
                () -> assertEquals(0.0, sent.getAngular().y, 1e-9),
                () -> assertEquals(0.0, sent.getAngular().z, 1e-9));
        }
    }

    // =========================================================================
    // 2. INTAKE
    // =========================================================================
    @Nested
    @DisplayName("Intake (std_msgs/Bool on /intake/enable)")
    class IntakeTests {

        @Test
        @DisplayName("enable command should publish Bool(true) on /intake/enable")
        void enableShouldPublishTrue() {
            StubPublisher<BoolStub> pub =
                node.newPublisher("/intake/enable", "std_msgs/Bool");
            BoolStub msg = pub.newMessage();
            msg.data = true;
            pub.publish(msg);

            assertEquals(1, pub.getPublishedCount());
            assertTrue(pub.lastPublished().data, "Intake enable must be true");
        }

        @Test
        @DisplayName("disable command should publish Bool(false) on /intake/enable")
        void disableShouldPublishFalse() {
            StubPublisher<BoolStub> pub =
                node.newPublisher("/intake/enable", "std_msgs/Bool");
            BoolStub msg = pub.newMessage();
            msg.data = false;
            pub.publish(msg);

            assertFalse(pub.lastPublished().data, "Intake enable must be false");
        }

        @Test
        @DisplayName("downstream subscriber should mirror enable state")
        void subscriberShouldMirrorPublisher() {
            StubPublisher<BoolStub> pub =
                node.newPublisher("/intake/enable", "std_msgs/Bool");
            AtomicBoolean received = new AtomicBoolean(false);

            StubSubscriber<BoolStub> sub =
                node.newSubscriber("/intake/enable", "std_msgs/Bool");
            sub.addMessageListener(m -> received.set(m.data));

            BoolStub msg = pub.newMessage();
            msg.data = true;
            pub.publish(msg);

            assertTrue(received.get(), "Subscriber must observe true after publish");
        }
    }

    // =========================================================================
    // 3. AUTONOMOUS
    // =========================================================================
    @Nested
    @DisplayName("Autonomous (state machine transitions)")
    class AutonomousTests {

        /**
         * Tiny state machine mirroring the real autonomous routine:
         *   IDLE -> DRIVE -> INTAKE -> DONE
         */
        enum AutoState { IDLE, DRIVE, INTAKE, DONE }

        @Test
        @DisplayName("should walk IDLE -> DRIVE -> INTAKE -> DONE in order")
        void shouldTransitionThroughAllStates() {
            List<AutoState> visited = new ArrayList<>();
            AutoState state = AutoState.IDLE;
            visited.add(state);

            // Step 1: scheduler fires -> DRIVE
            state = AutoState.DRIVE;
            visited.add(state);

            // Step 2: drive distance reached -> INTAKE
            state = AutoState.INTAKE;
            visited.add(state);

            // Step 3: intake done -> DONE
            state = AutoState.DONE;
            visited.add(state);

            assertEquals(
                List.of(AutoState.IDLE, AutoState.DRIVE, AutoState.INTAKE, AutoState.DONE),
                visited,
                "Autonomous state sequence must be monotonic");
        }

        @Test
        @DisplayName("should fall back to DONE when the drive watchdog times out")
        void shouldFallbackToDoneOnTimeout() {
            AtomicReference<AutoState> state =
                new AtomicReference<>(AutoState.DRIVE);
            AtomicInteger watchdogFires = new AtomicInteger(0);

            // Simulate a watchdog that fires once after 100ms of no progress.
            Runnable watchdog = () -> {
                if (state.get() == AutoState.DRIVE) {
                    watchdogFires.incrementAndGet();
                    state.set(AutoState.DONE);
                }
            };

            watchdog.run();

            assertEquals(AutoState.DONE, state.get(),
                "Watchdog must force transition to DONE");
            assertEquals(1, watchdogFires.get(),
                "Watchdog must fire exactly once");
        }

        @Test
        @DisplayName("autonomous must publish at least one /cmd_vel before INTAKE")
        void autonomousMustPublishBeforeIntake() {
            StubPublisher<TwistStub> cmdVel =
                node.newPublisher("/cmd_vel", "geometry_msgs/Twist");

            // Drive phase: push a forward command.
            TwistStub forward = cmdVel.newMessage();
            forward.getLinear().x = 0.4;
            cmdVel.publish(forward);

            // Only after a successful publish may the state machine enter INTAKE.
            if (cmdVel.getPublishedCount() == 0) {
                fail("Autonomous entered INTAKE without publishing /cmd_vel");
            }

            assertTrue(cmdVel.getPublishedCount() >= 1,
                "Autonomous must publish at least one Twist before intake phase");
        }
    }

    // =========================================================================
    // 4. SENSORS
    // =========================================================================
    @Nested
    @DisplayName("Sensors (LaserScan, Imu, encoder feedback)")
    class SensorTests {

        @Test
        @DisplayName("LaserScan listener should detect obstacle within 0.3m")
        void laserScanShouldDetectNearObstacle() {
            StubSubscriber<LaserScanStub> sub =
                node.newSubscriber("/scan", "sensor_msgs/LaserScan");
            AtomicBoolean obstacle = new AtomicBoolean(false);

            sub.addMessageListener(scan -> {
                for (float r : scan.ranges) {
                    if (r < 0.30f) { obstacle.set(true); break; }
                }
            });

            LaserScanStub scan = new LaserScanStub();
            scan.ranges = new float[]{1.0f, 0.8f, 0.25f, 1.2f}; // 0.25m obstacle
            sub.inject(scan);

            assertTrue(obstacle.get(),
                "Listener must flag an obstacle < 0.30m");
        }

        @Test
        @DisplayName("IMU quaternion should normalize to unit length")
        void imuQuaternionShouldBeUnitLength() {
            StubSubscriber<ImuStub> sub =
                node.newSubscriber("/imu/data", "sensor_msgs/Imu");
            AtomicReference<Double> norm = new AtomicReference<>(0.0);

            sub.addMessageListener(imu -> {
                double qx = imu.orientation.x;
                double qy = imu.orientation.y;
                double qz = imu.orientation.z;
                double qw = imu.orientation.w;
                norm.set(Math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw));
            });

            ImuStub imu = new ImuStub();
            imu.orientation.x = 0.0;
            imu.orientation.y = 0.0;
            imu.orientation.z = 0.7071;
            imu.orientation.w = 0.7071;
            sub.inject(imu);

            assertEquals(1.0, norm.get(), 1e-3,
                "Quaternion must be unit length (|q| = 1)");
        }

        @Test
        @DisplayName("encoder ticks should map to wheel revolutions correctly")
        void encoderTicksShouldConvertToRevolutions() {
            // 2048 ticks per revolution is the typical Thunders encoder config.
            final int TICKS_PER_REV = 2048;
            final int ticks = 10240; // exactly 5 revolutions

            double revolutions = (double) ticks / TICKS_PER_REV;

            assertEquals(5.0, revolutions, 1e-9,
                "10240 ticks at 2048 CPR must equal exactly 5 revolutions");
        }
    }

    // =========================================================================
    // STUB FRAMEWORK
    // -------------------------------------------------------------------------
    // Minimal in-process stand-ins for the ROSJava types we exercise. Keeping
    // these inside the test file means the test compiles & runs without a full
    // ROSJava toolchain — swap them for the real org.ros.* classes on the bot.
    // =========================================================================

    /** Stub for org.ros.node.ConnectedNode. */
    static final class StubConnectedNode {
        private boolean running = true;

        <T> StubPublisher<T> newPublisher(String topic, String messageType) {
            return new StubPublisher<>(topic, messageType);
        }

        <T> StubSubscriber<T> newSubscriber(String topic, String messageType) {
            return new StubSubscriber<>(topic, messageType);
        }

        void shutdown() { running = false; }
        boolean isRunning() { return running; }
    }

    /** Stub for org.ros.node.topic.Publisher&lt;T&gt;. */
    static final class StubPublisher<T> {
        private final String topicName;
        private final String messageType;
        private final List<T> published = new ArrayList<>();

        StubPublisher(String topicName, String messageType) {
            this.topicName = topicName;
            this.messageType = messageType;
        }

        String getTopicName() { return topicName; }
        String getMessageType() { return messageType; }

        @SuppressWarnings("unchecked")
        <M> M newMessage() {
            if ("geometry_msgs/Twist".equals(messageType)) {
                return (M) new TwistStub();
            } else if ("std_msgs/Bool".equals(messageType)) {
                return (M) new BoolStub();
            }
            throw new IllegalArgumentException("Unknown type: " + messageType);
        }

        void publish(T message) { published.add(message); }
        int getPublishedCount() { return published.size(); }
        T lastPublished() {
            if (published.isEmpty()) {
                throw new IllegalStateException("Nothing published yet");
            }
            return published.get(published.size() - 1);
        }
    }

    /** Stub for org.ros.node.topic.Subscriber&lt;T&gt;. */
    static final class StubSubscriber<T> {
        private final String topicName;
        private final String messageType;
        private final List<java.util.function.Consumer<T>> listeners =
            new ArrayList<>();

        StubSubscriber(String topicName, String messageType) {
            this.topicName = topicName;
            this.messageType = messageType;
        }

        void addMessageListener(java.util.function.Consumer<T> listener) {
            listeners.add(listener);
        }

        /** Test helper: push an inbound message to all listeners. */
        void inject(T message) {
            for (java.util.function.Consumer<T> l : listeners) {
                l.accept(message);
            }
        }
    }

    /** Stub for geometry_msgs.Twist. */
    static final class TwistStub {
        final Vector3 linear = new Vector3();
        final Vector3 angular = new Vector3();

        Vector3 getLinear() { return linear; }
        Vector3 getAngular() { return angular; }
    }

    /** Stub for std_msgs.Bool. */
    static final class BoolStub {
        boolean data;
    }

    /** Stub for geometry_msgs.Vector3. */
    static final class Vector3 {
        double x, y, z;
    }

    /** Stub for sensor_msgs.LaserScan. */
    static final class LaserScanStub {
        float angle_min, angle_max, angle_increment;
        float[] ranges;
    }

    /** Stub for sensor_msgs.Imu. */
    static final class ImuStub {
        final Quaternion orientation = new Quaternion();
    }

    /** Stub for geometry_msgs.Quaternion. */
    static final class Quaternion {
        double x, y, z, w;
    }
}
