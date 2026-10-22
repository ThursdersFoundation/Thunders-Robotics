<div align="center">

# ⚡ Thunders Robotics

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)]()
[![Java](https://img.shields.io/badge/Code-Java-red)]()
[![Python](https://img.shields.io/badge/Code-Python-blue)]()
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-ff69b4.svg)]()
[![Competition: FRC](https://img.shields.io/badge/Competition-FIRST%C2%AE%20Robotics%20Competition-orange)]()

*"Innovating the future, one robot at a time."*

**Thunders Robotics** is a student-led robotics team dedicated to designing, building, and programming high-performance robots for the **FIRST® Robotics Competition (FRC)**. Founded in **2019**, we have grown from a small group of enthusiasts into a competitive force in the national robotics scene. Beyond competition, we strive to foster a passion for STEM, engineering excellence, and teamwork within our community — because we believe every student deserves the chance to build something extraordinary.

</div>

---

<div align="center">

## 📑 Table of Contents

[🚀 About Us](#-about-us) • [🏅 Achievements](#-achievements) • [🤖 The Robot](#-the-robot-thunderbolt) • [🛠 Technical Specifications](#-technical-specifications) • [💻 Software Stack](#-software-stack) • [🚀 Quick Start](#-quick-start) • [📚 Documentation](#-documentation) • [📁 Repository Structure](#-repository-structure) • [🤝 Contributing](#-contributing) • [📜 Code of Conduct](#-code-of-conduct) • [🏆 Sponsors](#-sponsors) • [📞 Contact](#-contact) • [📄 License](#-license)

</div>

---

<div align="center">

## 🚀 About Us

Founded in **2019**, Thunders Robotics started as a small after-school club with a handful of students, one borrowed 3D printer, and an ambitious dream. Today, we are a fully student-driven team of engineers, programmers, designers, and marketers working side by side with dedicated mentors. Every season, we take a robot from a blank CAD file to a competition-ready machine — learning real-world engineering along the way.

Our team is organized into four core divisions that mirror a professional engineering company:

- **⚙️ Mechanical** — CAD design, CNC machining, 3D printing, and assembly.
- **⚡ Electrical** — Power distribution, wiring, sensor integration, and control boards.
- **💻 Software** — Robot code, computer vision, autonomous navigation, and dashboards.
- **📣 Outreach & Business** — Sponsorships, community events, and STEM education programs.

### Our Mission

To empower students with real-world engineering skills by promoting collaboration, critical thinking, and technical proficiency in a competitive environment. We don't just build robots — we build engineers, leaders, and problem-solvers who are ready to shape the future.

### Our Vision

To be a globally recognized leader in robotics innovation, setting the standard for performance, safety, and community outreach — while inspiring the next generation of engineers and innovators.

### ⚡ Core Values

- **Excellence** — We iterate, test, and refine until it's the best it can be.
- **Teamwork** — One team, one dream. Every subsystem wins together.
- **Curiosity** — We ask "what if?" before we ask "how much does it cost?"
- **Integrity** — We compete hard, play fair, and cheer for our opponents.
- **Community** — We share our knowledge freely through workshops and open source.

</div>

---

<div align="center">

## 🏅 Achievements

<table align="center">
  <thead>
    <tr>
      <th align="center">Year</th>
      <th align="center">Event</th>
      <th align="center">Result</th>
    </tr>
  </thead>
  <tbody>
    <tr><td align="center">2023</td><td align="center">FRC Regional — Rookie Inspiration Award</td><td align="center">🥇 Winner</td></tr>
    <tr><td align="center">2024</td><td align="center">FRC Regional — Playoff Alliance</td><td align="center">🏅 Semifinalist</td></tr>
    <tr><td align="center">2025</td><td align="center">National Robotics Festival — Best Autonomous Program</td><td align="center">🥈 Finalist</td></tr>
  </tbody>
</table>

> ⚠️ *Update the table above with your team's real results before publishing.*

</div>

---

<div align="center">

## 🤖 The Robot: THUNDERBOLT

**Codename:** `VLT-01` &nbsp;•&nbsp; **Competition Year:** 2026

**THUNDERBOLT** is our flagship machine, engineered for speed, precision, and reliability. Built around a full **swerve drivetrain**, it delivers omni-directional movement with independent wheel steering, allowing complex maneuvers during both autonomous and teleoperated periods. Its modular manipulator system can be reconfigured between matches to adapt to different game strategies.

### Key Features

- ✅ **High-Speed Swerve Drivetrain** — 4× independent SDS Mk4i modules, top speed **14.5 ft/s (~4.4 m/s)**.
- ✅ **Autonomous Navigation** — PathPlanner trajectory following with fused odometry (encoders + gyro + vision pose estimation).
- ✅ **Precision Manipulation** — Dual-roller intake with wrist mechanism, **~92%** game-piece acquisition success rate.
- ✅ **Smart Vision** — Limelight 3 running PhotonVision for real-time target tracking at **<30 ms** latency.
- ✅ **Data-Driven Development** — Full AdvantageKit logging; every match is replayable in simulation for tuning.

</div>

---

<div align="center">

## 🛠 Technical Specifications

<table align="center">
  <thead>
    <tr>
      <th align="center">Component</th>
      <th align="center">Specification</th>
    </tr>
  </thead>
  <tbody>
    <tr><td align="center"><b>Microcontroller</b></td><td align="center">roboRIO 2.0</td></tr>
    <tr><td align="center"><b>Motor Controllers</b></td><td align="center">CTRE Talon FX (CANivore) + REV Spark MAX</td></tr>
    <tr><td align="center"><b>Motors</b></td><td align="center">4× Falcon 500 (drivetrain), 2× NEO 550 (manipulator)</td></tr>
    <tr><td align="center"><b>Drivetrain</b></td><td align="center">Swerve — SDS Mk4i modules, 2nd stage reduction</td></tr>
    <tr><td align="center"><b>Sensors</b></td><td align="center">NavX2 MX+ gyro, quadrature encoders, beam-break sensors</td></tr>
    <tr><td align="center"><b>Vision Camera</b></td><td align="center">Limelight 3 + Microsoft LifeCam (driver feed)</td></tr>
    <tr><td align="center"><b>Power Source</b></td><td align="center">12V 18Ah SLA battery (non-spillable)</td></tr>
    <tr><td align="center"><b>Programming Language</b></td><td align="center">Java 17 (robot) / Python 3.10 (vision tooling)</td></tr>
    <tr><td align="center"><b>Framework</b></td><td align="center">WPILib 2026</td></tr>
    <tr><td align="center"><b>Weight</b></td><td align="center">~112 lbs (with bumpers)</td></tr>
  </tbody>
</table>

</div>

---

<div align="center">

## 💻 Software Stack

Our software architecture focuses on **modularity, reliability, and ease of debugging**. Every subsystem follows a command-based design, and all critical logic is unit-tested and logged.

- **Languages:** `Java 17` (robot core), `Python` (vision & data tooling)
- **Framework:** `WPILib 2026` — command-based robot architecture
- **Libraries:**
    - `PathPlanner` — autonomous trajectory generation and path following.
    - `PhotonVision` — computer vision processing and target tracking.
    - `AdvantageKit` — deterministic logging and replay-based testing.
    - `NT4 (NetworkTables)` — real-time robot-to-dashboard communication.
- **Dashboards:** `Elastic` (driver station), `AdvantageScope` (telemetry analysis)
- **Simulation:** WPILib Simulation (physics), AdvantageScope 3D replay
- **Version Control:** `Git & GitHub` (feature-branch workflow)
- **CI/CD:** `GitHub Actions` — automated build, test, and lint on every push
- **Code Quality:** `Spotless` (formatter), `Error-Prone` (static analysis)

</div>

---

<div align="center">

## 🚀 Quick Start

Prerequisites to run and deploy the code on the robot or in simulation.

### Prerequisites

- **OS:** Windows 10/11, macOS, or Linux
- **IDE:** VS Code with the **WPILib Extension 2026**
- **JDK:** Java 17 (Eclipse Temurin recommended — bundled with WPILib)
- **VCS:** Git
- *(Optional)* Python 3.10+ for vision/data scripts

### Installation

</div>

1. **Clone the repository**

   ```bash
   git clone https://github.com/ThundersRobotics/thunders-robotics-2026.git
   cd thunders-robotics-2026
   ```

2. **Build & run unit tests**

   ```bash
   ./gradlew build          # Windows: gradlew.bat build
   ```

3. **Deploy to Robot**

   ```bash
   # Make sure the roboRIO is reachable via USB/Ethernet
   ./gradlew deploy
   ```

4. **Run Simulation**

   ```bash
   ./gradlew simulateJava
   ```

5. *(Optional) Vision & data tooling*

   ```bash
   pip install -r vision/requirements.txt
   ```

<div align="center">

### Troubleshooting

- **Deploy fails?** Check that the Driver Station is installed and the team number matches `build.gradle`.
- **Gyro drifts?** Place the robot flat and let the NavX2 calibrate for ~10 s before enabling.
- **Sim crashes on launch?** Delete `build/` and re-run `./gradlew build`.

</div>

---

<div align="center">

## 📚 Documentation

Full technical documentation lives in the [`docs/`](docs/) folder:

<table align="center">
  <thead>
    <tr>
      <th align="center">Document</th>
      <th align="center">Description</th>
    </tr>
  </thead>
  <tbody>
    <tr><td align="center"><a href="docs/architecture.md"><code>architecture.md</code></a></td><td align="center">High-level software &amp; system architecture</td></tr>
    <tr><td align="center"><a href="docs/software.md"><code>software.md</code></a></td><td align="center">Command-based code conventions and tutorials</td></tr>
    <tr><td align="center"><a href="docs/hardware.md"><code>hardware.md</code></a></td><td align="center">Mechanical design decisions and CAD links</td></tr>
    <tr><td align="center"><a href="docs/electronics.md"><code>electronics.md</code></a></td><td align="center">Wiring diagrams, CAN bus map, and pinouts</td></tr>
    <tr><td align="center"><a href="docs/calibration.md"><code>calibration.md</code></a></td><td align="center">Sensor calibration and PID tuning guides</td></tr>
    <tr><td align="center"><a href="docs/safety.md"><code>safety.md</code></a></td><td align="center">Pit and field safety procedures</td></tr>
    <tr><td align="center"><a href="docs/troubleshooting.md"><code>troubleshooting.md</code></a></td><td align="center">Common issues and quick fixes</td></tr>
  </tbody>
</table>

</div>

---

<div align="center">

## 📁 Repository Structure

A clear breakdown of how our project is organized.

</div>

```
thunders-robotics/
│
├── 📂 .github/
│   ├── 📂 workflows/
│   │   ├── ci.yml
│   │   ├── build.yml
│   │   ├── test.yml
│   │   ├── lint.yml
│   │   └── security.yml
│   │
│   ├── 📂 ISSUE_TEMPLATE/
│   │   ├── bug_report.md
│   │   ├── feature_request.md
│   │   ├── hardware_issue.md
│   │   └── robotics_task.md
│   │
│   └── PULL_REQUEST_TEMPLATE.md
│
├── 📂 src/
│   ├── 📂 main/
│   │   ├── 📂 java/
│   │   │   └── 📂 frc/
│   │   │       ├── 📂 robot/
│   │   │       │   ├── Robot.java
│   │   │       │   ├── RobotContainer.java
│   │   │       │   └── RobotState.java
│   │   │       ├── 📂 commands/
│   │   │       │   ├── DriveCommand.java
│   │   │       │   ├── AutonomousCommand.java
│   │   │       │   ├── FollowPathCommand.java
│   │   │       │   ├── IntakeCommand.java
│   │   │       │   ├── ArmCommand.java
│   │   │       │   └── EmergencyStopCommand.java
│   │   │       ├── 📂 subsystems/
│   │   │       │   ├── Drivetrain.java
│   │   │       │   ├── Arm.java
│   │   │       │   ├── Elevator.java
│   │   │       │   ├── Intake.java
│   │   │       │   ├── Shooter.java
│   │   │       │   ├── Vision.java
│   │   │       │   └── Sensors.java
│   │   │       ├── 📂 autonomous/
│   │   │       │   ├── AutoRoutine.java
│   │   │       │   ├── PathPlanner.java
│   │   │       │   └── TrajectoryManager.java
│   │   │       ├── 📂 control/
│   │   │       │   ├── PIDController.java
│   │   │       │   ├── MotionController.java
│   │   │       │   └── BalanceController.java
│   │   │       ├── 📂 sensors/
│   │   │       │   ├── Gyroscope.java
│   │   │       │   ├── Encoder.java
│   │   │       │   ├── DistanceSensor.java
│   │   │       │   └── IMU.java
│   │   │       ├── 📂 constants/
│   │   │       │   ├── RobotConstants.java
│   │   │       │   ├── MotorConstants.java
│   │   │       │   └── SensorConstants.java
│   │   │       └── Main.java
│   │   │
│   │   ├── 📂 cpp/
│   │   │   └── 📂 thunders/
│   │   │       ├── robot/
│   │   │       ├── control/
│   │   │       ├── navigation/
│   │   │       └── sensors/
│   │   │
│   │   └── 📂 resources/
│   │       ├── robot.yaml
│   │       ├── motors.yaml
│   │       ├── sensors.yaml
│   │       ├── autonomous.yaml
│   │       └── logging.yaml
│   │
│   └── 📂 test/
│       ├── 📂 java/
│       │   └── 📂 frc/
│       │       ├── RobotTest.java
│       │       ├── DrivetrainTest.java
│       │       ├── PIDControllerTest.java
│       │       └── SensorTest.java
│       └── 📂 cpp/
│           ├── control_test.cpp
│           └── navigation_test.cpp
│
├── 📂 firmware/
│   ├── 📂 esp32/
│   │   ├── src/
│   │   ├── include/
│   │   ├── platformio.ini
│   │   └── README.md
│   ├── 📂 stm32/
│   │   ├── Core/
│   │   ├── Drivers/
│   │   ├── Middlewares/
│   │   └── README.md
│   ├── 📂 arduino/
│   │   ├── motor_controller/
│   │   ├── sensor_node/
│   │   └── communication/
│   └── README.md
│
├── 📂 ros/
│   ├── 📂 ros2_ws/
│   │   ├── 📂 src/
│   │   │   ├── thunders_robot/
│   │   │   ├── thunders_description/
│   │   │   ├── thunders_bringup/
│   │   │   ├── thunders_navigation/
│   │   │   ├── thunders_control/
│   │   │   ├── thunders_sensors/
│   │   │   └── thunders_interfaces/
│   │   └── README.md
│   └── 📂 launch/
│       ├── robot.launch.py
│       ├── simulation.launch.py
│       └── navigation.launch.py
│
├── 📂 robotics/
│   ├── 📂 kinematics/
│   │   ├── forward_kinematics.py
│   │   ├── inverse_kinematics.py
│   │   └── README.md
│   ├── 📂 dynamics/
│   │   ├── robot_dynamics.py
│   │   └── README.md
│   ├── 📂 navigation/
│   │   ├── path_planning.py
│   │   ├── obstacle_avoidance.py
│   │   └── localization.py
│   └── 📂 control/
│       ├── pid.py
│       ├── mpc.py
│       └── trajectory_control.py
│
├── 📂 autonomous/
│   ├── 📂 navigation/
│   ├── 📂 path_planning/
│   ├── 📂 localization/
│   ├── 📂 mapping/
│   └── 📂 obstacle_avoidance/
│
├── 📂 vision/
│   ├── 📂 camera/
│   │   ├── camera_config.yaml
│   │   └── camera_calibration.py
│   ├── 📂 detection/
│   │   ├── object_detection.py
│   │   ├── line_detection.py
│   │   └── marker_detection.py
│   ├── 📂 tracking/
│   │   ├── object_tracking.py
│   │   └── robot_tracking.py
│   ├── 📂 calibration/
│   │   ├── intrinsic.py
│   │   └── extrinsic.py
│   └── requirements.txt
│
├── 📂 ai/
│   ├── 📂 models/
│   │   ├── detection/
│   │   ├── classification/
│   │   └── segmentation/
│   ├── 📂 training/
│   │   ├── train.py
│   │   ├── config.yaml
│   │   └── checkpoints/
│   ├── 📂 datasets/
│   │   ├── raw/
│   │   ├── processed/
│   │   └── labels/
│   ├── 📂 inference/
│   │   ├── inference.py
│   │   └── realtime.py
│   └── requirements.txt
│
├── 📂 electronics/
│   ├── 📂 schematics/
│   │   ├── power_distribution.pdf
│   │   ├── motor_controller.pdf
│   │   └── sensor_system.pdf
│   ├── 📂 pcb/
│   │   ├── kicad/
│   │   ├── gerber/
│   │   └── bom/
│   ├── 📂 wiring/
│   │   ├── wiring_diagram.pdf
│   │   └── pinout.yaml
│   └── 📂 datasheets/
│       ├── motors/
│       ├── sensors/
│       └── controllers/
│
├── 📂 CAD/
│   ├── 📂 mechanical/
│   │   ├── chassis/
│   │   ├── drivetrain/
│   │   ├── arm/
│   │   ├── gripper/
│   │   └── brackets/
│   ├── 📂 assemblies/
│   │   ├── robot_assembly.step
│   │   └── robot_assembly.stl
│   ├── 📂 drawings/
│   │   ├── chassis.pdf
│   │   ├── arm.pdf
│   │   └── assembly.pdf
│   └── README.md
│
├── 📂 simulation/
│   ├── 📂 gazebo/
│   ├── 📂 webots/
│   ├── 📂 mujoco/
│   ├── 📂 unity/
│   ├── 📂 urdf/
│   │   └── thunders_robot.urdf
│   └── README.md
│
├── 📂 config/
│   ├── robot.yaml
│   ├── motors.yaml
│   ├── sensors.yaml
│   ├── network.yaml
│   ├── vision.yaml
│   └── autonomous.yaml
│
├── 📂 scripts/
│   ├── deploy.sh
│   ├── build.sh
│   ├── test.sh
│   ├── flash_firmware.sh
│   ├── calibrate_sensors.py
│   ├── calibrate_camera.py
│   ├── backup.sh
│   └── setup_environment.sh
│
├── 📂 tests/
│   ├── 📂 integration/
│   ├── 📂 hardware/
│   ├── 📂 software/
│   ├── 📂 simulation/
│   └── 📂 performance/
│
├── 📂 logs/
│   └── .gitkeep
│
├── 📂 data/
│   ├── 📂 telemetry/
│   ├── 📂 sensor_data/
│   ├── 📂 camera/
│   └── README.md
│
├── 📂 docs/
│   ├── architecture.md
│   ├── robot-overview.md
│   ├── software.md
│   ├── hardware.md
│   ├── electronics.md
│   ├── firmware.md
│   ├── ros2.md
│   ├── autonomous.md
│   ├── computer-vision.md
│   ├── artificial-intelligence.md
│   ├── simulation.md
│   ├── calibration.md
│   ├── testing.md
│   ├── deployment.md
│   ├── troubleshooting.md
│   ├── safety.md
│   ├── api.md
│   └── 📂 meeting-notes/
│       ├── 2026-01.md
│       ├── 2026-02.md
│       └── 2026-03.md
│
├── 📂 examples/
│   ├── basic-drive/
│   ├── autonomous-drive/
│   ├── object-detection/
│   ├── arm-control/
│   └── sensor-reading/
│
├── 📂 research/
│   ├── 📂 papers/
│   ├── 📂 experiments/
│   ├── 📂 datasets/
│   ├── 📂 benchmarks/
│   └── README.md
│
├── 📂 tools/
│   ├── dashboard/
│   ├── telemetry/
│   ├── robot-monitor/
│   └── diagnostics/
│
├── 📄 .gitignore
├── 📄 .gitattributes
├── 📄 .editorconfig
├── 📄 LICENSE
├── 📄 README.md
├── 📄 CONTRIBUTING.md
├── 📄 CODE_OF_CONDUCT.md
├── 📄 SECURITY.md
├── 📄 CHANGELOG.md
├── 📄 CITATION.cff
├── 📄 build.gradle
├── 📄 settings.gradle
├── 📄 gradle.properties
├── 📄 requirements.txt
├── 📄 pyproject.toml
└── 📄 docker-compose.yml
```

---

<div align="center">

## 🤝 Contributing

We welcome contributions from students, mentors, and the community!

</div>

1. **Fork the Project**
2. **Create your Feature Branch**

   ```bash
   git checkout -b feature/AmazingFeature
   ```

3. **Commit your Changes** (follow our commit convention below)

   ```bash
   git commit -m "feat: add some AmazingFeature"
   ```

4. **Push to the Branch**

   ```bash
   git push origin feature/AmazingFeature
   ```

5. **Open a Pull Request** — describe *what* changed and *why*.

<div align="center">

### Commit Convention

<table align="center">
  <thead>
    <tr>
      <th align="center">Prefix</th>
      <th align="center">Usage</th>
    </tr>
  </thead>
  <tbody>
    <tr><td align="center"><code>feat:</code></td><td align="center">New feature or subsystem</td></tr>
    <tr><td align="center"><code>fix:</code></td><td align="center">Bug fix</td></tr>
    <tr><td align="center"><code>docs:</code></td><td align="center">Documentation only</td></tr>
    <tr><td align="center"><code>refactor:</code></td><td align="center">Code change that neither fixes nor adds a feature</td></tr>
    <tr><td align="center"><code>test:</code></td><td align="center">Adding or fixing tests</td></tr>
    <tr><td align="center"><code>chore:</code></td><td align="center">Maintenance, CI, dependencies</td></tr>
  </tbody>
</table>

**Before you open a PR, please ensure:**

- ✅ Code follows the team's style guide (`./gradlew spotlessCheck` passes).
- ✅ All tests pass locally (`./gradlew build`).
- ✅ Complex logic is explained with comments.
- ✅ New constants live in the `constants/` package — no magic numbers.

</div>

---

<div align="center">

## 📜 Code of Conduct

Thunders Robotics is committed to providing a welcoming, inclusive, and safe environment for everyone — regardless of background, skill level, or identity. By participating in this project, you agree to uphold our standards of respectful collaboration.

Please read the full [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md) for details on our expectations for members and contributors, and how to report unacceptable behavior.

</div>

---

<div align="center">

## 🏆 Sponsors

A huge thank you to our sponsors who make our journey possible. Your support funds parts, tools, competition fees, and STEM outreach for students who would otherwise never touch a robot.

<table align="center">
  <thead>
    <tr>
      <th align="center">Platinum Sponsor</th>
      <th align="center">Gold Sponsor</th>
      <th align="center">Silver Sponsor</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td align="center"><i>[Your Logo Here]</i></td>
      <td align="center"><i>[Your Logo Here]</i></td>
      <td align="center"><i>[Your Logo Here]</i></td>
    </tr>
  </tbody>
</table>

*Interested in sponsoring us? Reach out at* [sponsorships@thundersrobotics.com](mailto:sponsorships@thundersrobotics.com) 📧

**Special thanks to the open-source robotics community:** WPILib, CTRE, REV Robotics, PhotonVision, and PathPlanner — you make student robotics possible.

</div>

---

<div align="center">

## 📞 Contact

- **Email:** [thursdersfoundation@gmail.com](mailto:thursdersfoundation@gmail.com)
- **GitHub:** [Thunders Foundation](https://github.com/ThundersRobotics)
- **Location:** *[Your City / School Name]*

</div>

---

<div align="center">

## 📄 License

Distributed under the **MIT License**. See [`LICENSE`](LICENSE) for more information.

</div>

<div align="center">

<sub>Made with ⚡ and far too much coffee by the Thunders Robotics software team.</sub>

</div>
<img width="800" height="450" alt="logo" src="https://github.com/user-attachments/assets/414f44b9-d29c-4769-8960-6405e8782c70" />
