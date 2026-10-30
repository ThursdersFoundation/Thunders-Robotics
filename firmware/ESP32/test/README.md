# `test/` — unit tests

PlatformIO native unit tests using the Unity framework. Run with:

```bash
pio test -e native
```

Add one `test_*.cpp` file per module under test. Each file should `#include
<unity.h>` and implement `int runUnityTests()` (or rely on PlatformIO's
default entry point). Mocks for the ESP32 HAL live under `test/mock/`.

Convention:
* `test_pid_controller.cpp`     — tests for `src/control/pid_controller.cpp`
* `test_velocity_controller.cpp` — tests for `src/control/velocity_controller.cpp`
* `test_robot_state.cpp`        — tests for `src/app/robot_state.cpp`
* …
