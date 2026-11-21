# QNX BrakeWire ADAS

**ISO 26262 ASIL-D Brake-by-Wire, CAN-FD CRC-16 Validator & LiDAR/Radar Sensor Fusion Digital Twin for QNX Neutrino RTOS 8.0**

Built for **QNX Global Hackathon 2026 — Automotive & Mobility** | License: **MIT** | Zero Runtime Dependencies

- **Live Workbench:** `https://qnx-brakewire-adas.vercel.app`
- **Repository:** `https://github.com/wenizmohamed/qnx-brakewire-adas`

---

## Why QNX BrakeWire ADAS?

At `110 km/h` (`30.6 m/s`), a sudden highway cut-in with `1.42 s` Time-to-Collision leaves zero margin for operating-system jitter. Every `100 ms` of non-deterministic scheduling delay costs **`1.94 m`** of unbraked relative closing distance — turning a safe stop into a high-speed collision.

**QNX BrakeWire ADAS** pairs an authentic **QNX Neutrino 8.0 C11 target codebase (`target-qnx8/`)** with a **zero-dependency deterministic physics/timing engine (`src/`)**, a **SARIF 2.1.0 ISO 26262 CI safety gate CLI (`cli/brakewire-cli.mjs`)**, and a **60 FPS interactive Automotive Cockpit & Digital Twin (`public/`)**:

| Scenario | Ego / Target | QNX 8.0 (`SCHED_FIFO 63` + `MsgSendv()`) | Non-RT GPOS (CFS + Heap Copy) | Margin Saved |
|---|---|---|---|---|
| **01. Highway Cut-In** | 110 -> 40 km/h (`TTC 1.42s`) | `412 us` loop -> **`+4.82 m` SAFE STOP** | `370 ms` jitter -> **`-2.37 m` COLLISION** | **`+7.19 m`** |
| **02. Pedestrian Night Crosswalk** | 50 -> 0 km/h (`12 lux`) | `396 us` loop -> **`+3.85 m` SAFE STOP** | `340 ms` stall -> **`-0.87 m` COLLISION** | **`+4.72 m`** |
| **03. CAN-FD Spoof / Bad CRC-16** | 85 -> 15 km/h (`0x120` attack) | Rejected in **`18 us`**, LiDAR Ch-B -> **`+4.01 m` SAFE** | Spoof accepted -> **`-6.10 m` COLLISION** | **`+10.11 m`** |
| **04. Infotainment Priority Storm** | 100 -> 20 km/h (`98%` CPU storm) | `428 us`, `0` preempts -> **`+3.87 m` SAFE** | `685 ms` inversion -> **`-11.35 m` COLLISION** | **`+15.22 m`** |
| **05. Wet-Road Split-Mu ABS** | 75 -> 0 km/h (`mu=0.55`) | `1 kHz` ABS `96%` grip -> **`+3.12 m` SAFE** | `78%` slip lag -> **`-12.58 m` COLLISION** | **`+15.70 m`** |

---

## System Architecture

```text
  77 GHz Radar (0x120) ──┐
  LiDAR Point Cloud(0x124)┼─► canfd_sensor_rx (SCHED_FIFO 60)
  Wheel Speed IMU (0x130)┘      │ CRC-16/CCITT-FALSE (0x1021)
                                │ 4-bit Monotonic Counter + Plausibility Voter (18 us)
                                ▼  Zero-Copy Scatter-Gather MsgSendv() (2-part IOV)
                         brakewire_adas (SCHED_FIFO 63, CPU0 Pinned)
                           ├─ name_attach("qnx/brakewire")
                           ├─ ClockCycles() / SYSPAGE_ENTRY(qtime) 1,000 us Deadline Guard
                           ├─ ISO 26262 ASIL-D AEB Kinematics & Hydraulic Bar Law
                           └─ /pps/adas/aeb (20 Hz PPS Telemetry) + slog2 Diagnostics
```

---

## Quick Start (Local Engine, CLI & Tests)

```bash
# Run all 43 deterministic unit, physics, CRC-16, CLI, SARIF & byte-parity tests
npm test

# Run ISO 26262 ASIL-D Safety Gate CLI (exit 0 = PASS, exit 2 = SAFETY VIOLATION)
node cli/brakewire-cli.mjs --scenario highway-cutin --format markdown
node cli/brakewire-cli.mjs --scenario priority-storm --mode non-rt --format sarif
node cli/brakewire-cli.mjs --scenario canfd-spoof --format json
```

## QNX Neutrino 8.0 Target Build & QEMU SMP Boot

See [`target-qnx8/README.md`](target-qnx8/README.md) for full QNX SDP 8.0 instructions:

```bash
source ~/qnx800/qnxsdp-env.sh
make -C target-qnx8 ifs
./target-qnx8/scripts/run-qemu.sh
```

Inside QNX 8.0:

```sh
cat /pps/adas/aeb
pidin -p brakewire_adas threads
slog2info -b asil_d_rt
```

---

## Repository Structure

- `target-qnx8/` — Authentic QNX Neutrino 8.0 C11 target code (`brakewire_adas.c`, `canfd_sensor_rx.c`, `brakewire_ipc.h`, `Makefile`, `CMakeLists.txt`, `qnx8-toolchain.cmake`, `qnx8-qemu-x86_64.build`, `scripts/run-qemu.sh`)
- `src/` — Deterministic pure ESM engine (`canfdValidator.js`, `adasEngine.js`, `iso26262Audit.js`)
- `public/` — Automotive Cockpit & 60 FPS Digital Twin web workbench (with byte-identical `public/engine/`)
- `cli/brakewire-cli.mjs` — Zero-dependency ISO 26262 ASIL-D Safety Gate CLI (`json`, `markdown`, `sarif`)
- `test/brakewire.test.mjs` — 43 deterministic Node `node:test` suites

## License

MIT — see [`LICENSE`](LICENSE).
