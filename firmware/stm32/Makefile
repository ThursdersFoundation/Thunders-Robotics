# =============================================================================
# Makefile wrapper for the Thunders Robotic STM32F407 firmware
# Usage:
#   make            # configure + build firmware.elf / .bin / .hex
#   make flash      # build, then flash firmware.elf via OpenOCD (ST-Link)
#   make test       # cross-compile firmware, then build+run host unit tests
#   make clean      # remove the build/ directory
#   make menuconfig # launch cmake-gui / ccmake for build option editing
#   make size       # show firmware size report
# =============================================================================

CMAKE          ?= cmake
BUILD_DIR      ?= build
TOOLCHAIN_FILE ?=
GENERATOR      ?= Unix Makefiles

CMAKE_ARGS      = -B $(BUILD_DIR) -G "$(GENERATOR)"
ifneq ($(TOOLCHAIN_FILE),)
    CMAKE_ARGS += -DCMAKE_TOOLCHAIN_FILE=$(TOOLCHAIN_FILE)
endif

.PHONY: all build flash test clean menuconfig size reconfigure

all: build

build:
	$(CMAKE) $(CMAKE_ARGS) -DCMAKE_BUILD_TYPE=Debug .
	$(CMAKE) --build $(BUILD_DIR) --parallel $$(nproc)

flash: build
	./Tools/scripts/flash_firmware.sh $(BUILD_DIR)/firmware.elf

test:
	$(CMAKE) -B $(BUILD_DIR)-test -DBUILD_UNIT_TESTS=ON -DCMAKE_BUILD_TYPE=Debug .
	$(CMAKE) --build $(BUILD_DIR)-test --parallel $$(nproc)
	ctest --test-dir $(BUILD_DIR)-test --output-on-failure

size: build
	@arm-none-eabi-size $(BUILD_DIR)/firmware.elf

clean:
	rm -rf $(BUILD_DIR) $(BUILD_DIR)-test

menuconfig:
	$(CMAKE) $(CMAKE_ARGS) .
	@command -v ccmake >/dev/null 2>&1 && ccmake $(BUILD_DIR) || cmake-gui $(BUILD_DIR)

reconfigure: clean build
