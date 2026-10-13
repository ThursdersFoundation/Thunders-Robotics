name: Hardware Issue
about: Report a physical, electrical, or mechanical malfunction on the robot
title: '[HARDWARE] '
labels: hardware, triage
assignees: ''
---

## 🔧 Hardware Issue Description
A clear and concise description of the physical issue. 
*(e.g., The left shooter motor is completely unresponsive and making a clicking sound.)*

## 🤖 Subsystem & Location
Which part of the robot is affected? Where is the component physically located?
*(e.g., Shooter / Top right NEO motor inside the flywheel gearbox)*

## 🔌 Component Details
Please provide the specific details of the hardware component(s) involved:
- **Part Name:** [e.g., NEO Brushless Motor, Talon SRX, PWM Servo, Limit Switch]
- **Device ID / Port:** [e.g., CAN ID 14, PWM Port 5, Pneumatics Hub Port 2]
- **Quantity Affected:** [e.g., 1 motor, 2 encoders]

## 🚨 Symptoms & Physical Behavior
Describe exactly what is happening physically. 
*(e.g., The motor spins freely without moving the shaft (stripped hex), the spark MAX is blinking red/yellow, there is a burning smell coming from the Victor SPX, or the chain is derailed.)*

## ⚠️ Safety Concern (CRITICAL)
Does this issue pose an immediate safety risk to the team or the robot?
- [ ] **YES** (Exposed live wires, fire hazard, structural instability, sharp edges) -> *IMMEDIATELY NOTIFY A LEAD/MENTOR*
- [ ] **NO**

## 🔍 Troubleshooting Steps Taken
What have you already checked or tried to fix the issue?
- [ ] Checked wiring connections (Power, CAN/PWM, Signal)
- [ ] Swapped wires with a known working component
- [ ] Checked circuit breakers / PDP/PDH channels
- [ ] Tested the component directly via hardware client (e.g., REV Hardware Client, Phoenix Tuner)
- [ ] Inspected for physical damage (frayed wires, stripped gears, broken mounts)
- [ ] Other: __________

*Describe the results of the steps you took:*
*(e.g., "I swapped the CAN wires with the right motor, and the right motor died, confirming it's a motor/controller issue, not a code issue.")*

## 🛠️ Proposed Fix / Action Plan
What do you think needs to be done to resolve this?
*(e.g., Need to replace the stripped 12T pinion gear, re-crimp the Anderson power connector, or order a new Spark MAX controller.)*

## 📸 Photos & Videos
Please attach photos or videos of the issue. This is **highly recommended** for hardware issues to show physical damage, wiring mistakes, or mechanical failure.
