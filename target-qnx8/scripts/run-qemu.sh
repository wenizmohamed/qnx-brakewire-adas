#!/usr/bin/env bash
set -euo pipefail

# QNX BrakeWire ADAS - QEMU x86_64 SMP Target Launcher
# Prerequisites: QNX SDP 8.0 sourced (source ~/qnx800/qnxsdp-env.sh) + qemu-system-x86_64

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
IFS_IMG="${TARGET_DIR}/build/x86_64/qnx8-brakewire.ifs"

if [[ ! -f "${IFS_IMG}" ]]; then
    echo "[run-qemu] IFS image missing. Building QNX 8.0 binaries + IFS..."
    make -C "${TARGET_DIR}" ifs
fi

echo "[run-qemu] Booting QNX Neutrino 8.0 SMP (4 vCPUs, 1024 MiB RAM)..."
echo "[run-qemu] Inside QNX shell, run: cat /pps/adas/aeb && slog2info -w"

exec qemu-system-x86_64 \
    -smp 4 \
    -m 1024 \
    -nographic \
    -serial mon:stdio \
    -kernel "${IFS_IMG}" \
    -no-reboot
