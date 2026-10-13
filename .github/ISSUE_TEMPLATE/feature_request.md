name: Feature Request
about: Suggest a new feature or enhancement for the Thunders Robotics project
title: '[FEATURE] '
labels: enhancement, triage
assignees: ''
---

## 💡 Feature Description
A clear and concise description of the feature or enhancement you want to add.
*(e.g., Add a PID-controlled auto-alignment command for the shooter using Limelight vision data.)*

## 🎯 Motivation / Problem Statement
Why do you need this feature? What problem does it solve or what capability does it unlock?
*(e.g., During competitions, our drivers struggle to align the robot to the goal quickly under defense, which costs us valuable scoring time. An auto-align feature would improve our cycle times.)*

## 🛠️ Proposed Solution
Describe how you would like this feature to be implemented.
*(e.g., Create a new command that calculates the horizontal angle error from the Limelight, feeds it into a PID controller, and adjusts the drivetrain's rotation until the error is zero. Bind this to the 'X' button on the driver joystick.)*

## 🤖 Affected Subsystem / Scope
Which part of the robot or codebase does this feature primarily affect? (Check all that apply)

- [ ] Drivetrain / Controls
- [ ] Vision / Localization
- [ ] Manipulator / Mechanism (e.g., Intake, Shooter, Climber)
- [ ] Autonomous Routines
- [ ] Dashboard / UI (e.g., Shuffleboard, SmartDashboard)
- [ ] Hardware / Electrical Wiring
- [ ] Infrastructure / Build System
- [ ] Other: __________

**Complexity Estimate (Optional):** [e.g., Small tweak / Medium effort / Major feature / Requires hardware change]

## 🔄 Alternatives Considered
A clear description of any alternative solutions or features you've considered.
*(e.g., We could just give the driver a slower turning speed mode for fine adjustments, but that's still prone to human error and takes longer.)*

## 📌 Additional Context
Add any other context, screenshots, reference code, or diagrams about the feature request here.
*(e.g., Link to a CD thread discussing this approach, a photo of another robot with this mechanism, or mathematical proof of concept for a physics calculation.)*
