# ------------------------------------------------------------------
# Host unit-test build for the Thunders Robotic STM32 firmware.
# Links the real middleware/control modules against mock hardware.
# Usage:  make [test_protocol|test_motion|test_sensors|test_rtos|test_system|all|test]
# ------------------------------------------------------------------
STM32 := ..
INC := -I$(STM32)/Core/Inc \
       -I$(STM32)/Drivers/CMSIS/Include \
       -I$(STM32)/Drivers/CMSIS/Device \
       -I$(STM32)/Drivers/STM32F4xx_HAL_Driver/Inc \
       -I$(STM32)/Drivers/Custom_Drivers/Inc \
       -I$(STM32)/Middlewares/FreeRTOS/Inc \
       -I$(STM32)/Middlewares/Communication/Inc \
       -I$(STM32)/Middlewares/Utilities/Inc \
       -Imocks
CFLAGS := -std=c11 -Wall -Wextra -Wno-unused-parameter -g -O1
LDLIBS := -lm

MOCKS := mocks/mock_hal.c
COMM_SRC := $(STM32)/Middlewares/Communication/Src/crc.c \
            $(STM32)/Middlewares/Communication/Src/packet_parser.c \
            $(STM32)/Middlewares/Communication/Src/uart_protocol.c \
            $(STM32)/Middlewares/Communication/Src/can_protocol.c \
            $(STM32)/Middlewares/Communication/Src/protocol.c
UTIL_SRC := $(STM32)/Middlewares/Utilities/Src/moving_average.c \
            $(STM32)/Middlewares/Utilities/Src/ring_buffer.c
RTOS_SRC := $(STM32)/Middlewares/FreeRTOS/Src/tasks.c \
            $(STM32)/Middlewares/FreeRTOS/Src/queue.c \
            $(STM32)/Middlewares/FreeRTOS/Src/port.c \
            $(STM32)/Drivers/CMSIS/Device/system_stm32f4xx.c
MOTION_SRC := $(STM32)/Core/Src/motor_control.c \
              $(STM32)/Core/Src/encoder.c \
              $(STM32)/Core/Src/safety_manager.c \
              $(STM32)/Drivers/Custom_Drivers/Src/drv_motor.c \
              $(STM32)/Drivers/Custom_Drivers/Src/drv_encoder.c \
              $(STM32)/Drivers/Custom_Drivers/Src/drv_ultrasonic.c \
              $(STM32)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
SENSOR_SRC := $(STM32)/Drivers/Custom_Drivers/Src/drv_battery.c \
              $(STM32)/Drivers/Custom_Drivers/Src/drv_ultrasonic.c \
              $(STM32)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
              $(STM32)/Drivers/Custom_Drivers/Src/drv_lidar.c \
              $(STM32)/Drivers/Custom_Drivers/Src/drv_imu.c

BINS := test_protocol test_motion test_sensors test_rtos test_system

all: $(BINS)

test_protocol: unit/test_protocol.c $(COMM_SRC) $(MOCKS)
	$(CC) $(CFLAGS) $(INC) $^ $(LDLIBS) -o $@

test_motion: unit/test_motion.c $(MOTION_SRC) $(RTOS_SRC) $(MOCKS)
	$(CC) $(CFLAGS) $(INC) $^ $(LDLIBS) -o $@

test_sensors: unit/test_sensors.c $(SENSOR_SRC) $(UTIL_SRC) $(COMM_SRC) $(MOCKS)
	$(CC) $(CFLAGS) $(INC) $^ $(LDLIBS) -o $@

test_rtos: unit/test_rtos.c $(RTOS_SRC) $(MOCKS)
	$(CC) $(CFLAGS) $(INC) $^ $(LDLIBS) -o $@

test_system: integration/test_system.c $(MOTION_SRC) $(COMM_SRC) $(RTOS_SRC) $(UTIL_SRC) $(MOCKS)
	$(CC) $(CFLAGS) $(INC) $^ $(LDLIBS) -o $@

test: all
	@pass=0; fail=0; \
	for t in $(BINS); do \
	    echo "--- $$t"; \
	    if ./$$t; then pass=$$((pass+1)); else fail=$$((fail+1)); echo "*** $$t FAILED ***"; fi; \
	done; \
	echo "=== $$pass suites passed, $$fail failed ==="; \
	test $$fail -eq 0

clean:
	rm -f $(BINS)

.PHONY: all test clean
