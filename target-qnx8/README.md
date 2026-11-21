# QNX BrakeWire ADAS — QNX Neutrino 8.0 Target Codebase

Authentic C11 source and build artifacts targeting **QNX Neutrino RTOS 8.0** (`x86_64` QEMU SMP and `aarch64le` automotive SoCs).

## Architecture Overview

| Binary / File | Role | QNX 8.0 Primitives Used |
|---|---|---|
| `brakewire_adas.c` | ISO 26262 ASIL-D Brake-by-Wire & AEB Resource Manager | `name_attach("qnx/brakewire")`, `MsgReceivev()`, `MsgReplyv()`, `ClockCycles()`, `SYSPAGE_ENTRY(qtime)->cycles_per_sec`, `SCHED_FIFO` prio `63`, `ThreadCtl(_NTO_TCTL_RUNMASK)`, `PTHREAD_PRIO_INHERIT`, `/pps/adas/aeb`, `slog2f()` |
| `canfd_sensor_rx.c` | CAN-FD Frame Receiver, CRC-16 Validator & Plausibility Voter | `name_open("qnx/brakewire")`, zero-copy scatter-gather `MsgSendv()`, `SCHED_FIFO` prio `60`, CRC-16/CCITT-FALSE (`0x1021`), 4-bit rolling counter check, LiDAR-vs-Radar plausibility voting |
| `brakewire_ipc.h` | Shared Zero-Copy IOV & CAN-FD Structures | Packed `canfd_frame_t`, `bw_ipc_hdr_t`, `bw_sensor_fusion_t`, `bw_brake_reply_t`, inline `bw_crc16_ccitt()` |
| `qnx8-qemu-x86_64.build` | QNX 8.0 Instrumented SMP IFS Image | `procnto-smp-instr`, `slogger2`, `pps`, `tracelogger`, `brakewire_adas`, `canfd_sensor_rx` |

## Build with QNX SDP 8.0

```bash
# 1. Source QNX SDP 8.0 environment
source ~/qnx800/qnxsdp-env.sh

# 2. Build x86_64 binaries via Make (or pass ARCH=aarch64le for ARM64)
cd target-qnx8
make

# 3. Or build via CMake cross-compilation toolchain
cmake -B build-x86_64 -DCMAKE_TOOLCHAIN_FILE=qnx8-toolchain.cmake -DQNX_ARCH=x86_64
cmake --build build-x86_64
```

## Boot in QEMU & Inspect Live Telemetry

```bash
# Build bootable IFS image and launch 4-vCPU QNX 8.0 SMP in QEMU
./scripts/run-qemu.sh

# Inside the QNX target console:
cat /pps/adas/aeb
pidin -p brakewire_adas threads
slog2info -b asil_d_rt
```

## Capturing Kernel Traces (`tracelogger`)

Because `qnx8-qemu-x86_64.build` boots `procnto-smp-instr`, you can capture microsecond kernel event traces directly:

```bash
tracelogger -n 1 -s 5 -f /tmp/brakewire.kev
traceprinter -f /tmp/brakewire.kev > /tmp/brakewire_trace.txt
```
