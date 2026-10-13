## 🚀 Description
Please include a summary of the changes and the related motivation. Why is this PR needed? What does it accomplish?

*(e.g., This PR adds a new PID-controlled auto-alignment command for the shooter using Limelight vision data, resolving the slow manual alignment issue during matches.)*

## 🔗 Related Issues
List any issues that this PR resolves or relates to. 
*(Use keywords like `Resolves`, `Closes`, or `Fixes` to automatically close the issue upon merge.)*

- Resolves # (issue number)

## 📋 Type of Change
Please delete options that are not relevant.

- [ ] 🐛 Bug fix (non-breaking change which fixes an issue)
- [ ] ✨ New feature (non-breaking change which adds functionality)
- [ ] 💥 Breaking change (fix or feature that would cause existing functionality to not work as expected)
- [ ] 🔄 Refactor (code restructuring without changing behavior)
- [ ] 📝 Documentation update
- [ ] 🏎️ Autonomous routine addition/update
- [ ] 📺 Dashboard/SmartDashboard update
- [ ] ⚙️ Build/CI/Dependencies update

## 🤖 Affected Subsystems
Which subsystems or parts of the robot are affected by this PR? (Check all that apply)

- [ ] Drivetrain
- [ ] Vision / Limelight
- [ ] Intake
- [ ] Shooter
- [ ] Climber / Hang
- [ ] Pneumatics
- [ ] Autonomous / Path Planning
- [ ] OI (Operator Interface / Controls)
- [ ] Infrastructure / Robot Container
- [ ] Other: __________

## 🧪 Testing & Validation
How did you test these changes? Robot code can be dangerous, so please be specific!

- [ ] **Simulation:** Tested in WPILib Sim / Gazebo / other simulator.
- [ ] **Hardware:** Tested on the Practice Bot / Chassis.
- [ ] **Competition Bot:** Tested on the Final Competition Robot.
- [ ] **Unit Tests:** Added or updated unit tests, and they pass.
- [ ] **No Testing:** Requires testing by someone else before merge.

*Describe the testing steps you took:*
*(e.g., "Ran the auto-align command 10 times in sim, then deployed to the practice bot and verified it locks onto the target within 0.5 seconds without oscillating.")*

## ✅ Author Checklist
Please ensure your PR meets the following requirements before requesting a review:

- [ ] My code follows the style guidelines of this project (Linting/Formatting passes).
- [ ] I have performed a self-review of my own code.
- [ ] I have commented my code, particularly in hard-to-understand areas or new PID constants.
- [ ] I have made corresponding changes to the documentation (if applicable).
- [ ] My changes generate no new compiler warnings.
- [ ] I have added tests that prove my fix is effective or that my feature works (if applicable).
- [ ] New and existing unit tests pass locally with my changes.

## 📸 Media (Optional but Recommended)
Add screenshots, videos, or Shuffleboard/SmartDashboard layouts to help reviewers visualize the changes. 
*(e.g., A video of the mechanism moving correctly, or a screenshot of the new networktables entry).*

## 💬 Additional Context
Add any other context about the pull request here. 
*(e.g., "The P values for the shooter PID might need to be tuned further on the competition bot since the battery voltage behaves differently under load.")*
