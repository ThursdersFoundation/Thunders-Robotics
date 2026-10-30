# `docs/` — design & protocol documentation

Markdown-only documentation for the firmware: architecture diagrams, board
wiring guides, communication protocol specs, and tuning notes. Render with
your favourite Markdown previewer or use [mkdocs](https://www.mkdocs.org/)
for a hosted site.

## Suggested files

| File                  | Content                                            |
| --------------------- | -------------------------------------------------- |
| `architecture.md`     | Layer diagram, task tree, dependency rules.        |
| `board_wiring.md`     | ESP32 ↔ driver ↔ sensor wiring reference.          |
| `serial_protocol.md`  | Byte-level specification of the binary protocol.    |
| `mqtt_topics.md`      | Topic namespace, payload schemas, QoS choices.     |
| `ros_bridge.md`       | rosbridge_suite integration notes.                 |
| `tuning_pid.md`       | How to tune `kVelocityKp/Ki/Kd` on real hardware.   |
| `safety_model.md`     | E-stop state machine + recovery procedure.         |
| `ota_workflow.md`     | Over-the-air update workflow with two OTA slots.   |
