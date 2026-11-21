# QNX BrakeWire ADAS - QNX SDP 8.0 Makefile
# Usage:
#   source ~/qnx800/qnxsdp-env.sh
#   make                    # builds x86_64 binaries (default)
#   make ARCH=aarch64le     # builds ARM64 binaries (Raspberry Pi 4 / Snapdragon Ride)

ARCH ?= x86_64
CC := qcc
VARIANT := gcc_nto$(ARCH)
CFLAGS := -V$(VARIANT) -std=c11 -O2 -Wall -Wextra -Werror -D_QNX_SOURCE
LDFLAGS := -V$(VARIANT) -lslog2 -lm
BUILD_DIR := build/$(ARCH)

TARGETS := $(BUILD_DIR)/brakewire_adas $(BUILD_DIR)/canfd_sensor_rx

.PHONY: all clean ifs

all: $(TARGETS)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/brakewire_adas: brakewire_adas.c brakewire_ipc.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -o $@ brakewire_adas.c $(LDFLAGS)

$(BUILD_DIR)/canfd_sensor_rx: canfd_sensor_rx.c brakewire_ipc.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -o $@ canfd_sensor_rx.c $(LDFLAGS)

ifs: all
	mkifs -v -r $(QNX_TARGET)/$(ARCH) qnx8-qemu-x86_64.build $(BUILD_DIR)/qnx8-brakewire.ifs

clean:
	rm -rf build
