import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';

import {
  CANFD_IDS,
  CANFD_FLAGS,
  crc16Ccitt,
  encodeCanFdFrame,
  validateCanFdFrame,
  runPlausibilityVoter,
  generateScenarioCanFdBurst,
} from '../src/canfdValidator.js';

import {
  GRAVITY_MPS2,
  DEADLINE_BUDGET_US,
  SCENARIOS,
  FRICTION_PRESETS,
  evaluateBrakingPass,
  runAdasSimulation,
} from '../src/adasEngine.js';

import {
  ISO_RULES,
  evaluateIso26262Audit,
  formatAuditJson,
  formatAuditMarkdown,
  formatAuditSarif,
} from '../src/iso26262Audit.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(__dirname, '..');

/* ------------------------------------------------------------------
 * 1. CRC-16/CCITT-FALSE & CAN-FD Frame Validation Tests
 * ------------------------------------------------------------------ */
test('CRC-16/CCITT-FALSE matches standard test vector "123456789" -> 0x29B1', () => {
  const bytes = new TextEncoder().encode('123456789');
  assert.equal(crc16Ccitt(bytes), 0x29b1);
});

test('CRC-16/CCITT-FALSE returns 0xFFFF for empty payload', () => {
  assert.equal(crc16Ccitt(new Uint8Array(0)), 0xffff);
});

test('encodeCanFdFrame encodes valid 16-byte chassis frame with matching CRC-16', () => {
  const frame = encodeCanFdFrame({
    canId: CANFD_IDS.RADAR_TARGET,
    rollingCounter: 5,
    rangeM: 27.61,
    speedMps: 30.56,
    mu: 0.9,
  });
  assert.equal(frame.dlc, 16);
  assert.equal(frame.canIdHex, '0x120');
  assert.equal(frame.crc16Expected, frame.crc16Received);
  assert.equal(frame.payload.length, 16);
});

test('validateCanFdFrame accepts valid monotonic rolling counter sequence', () => {
  let state = { initialized: false, expectedCounter: 0, consecutiveErrors: 0 };
  for (let i = 0; i < 18; i++) {
    const f = encodeCanFdFrame({ rollingCounter: i & 0x0f, rangeM: 25.0 });
    const res = validateCanFdFrame(f, state);
    assert.equal(res.valid, true);
    assert.equal(res.flags, CANFD_FLAGS.VALID);
    assert.equal(res.validationTimeUs, 14);
    state = res.nextState;
  }
  assert.equal(state.expectedCounter, 2);
});

test('validateCanFdFrame detects corrupted CRC-16 in 18 us', () => {
  const f = encodeCanFdFrame({ rollingCounter: 3, rangeM: 180.0, corruptCrc: true, spoofed: true });
  const res = validateCanFdFrame(f, { initialized: true, expectedCounter: 3, consecutiveErrors: 0 });
  assert.equal(res.valid, false);
  assert.ok((res.flags & CANFD_FLAGS.BAD_CRC16) !== 0);
  assert.equal(res.validationTimeUs, 18);
});

test('validateCanFdFrame detects rolling counter jump (replay/spoof)', () => {
  const f = encodeCanFdFrame({ rollingCounter: 11, rangeM: 25.0 });
  const res = validateCanFdFrame(f, { initialized: true, expectedCounter: 7, consecutiveErrors: 0 });
  assert.equal(res.valid, false);
  assert.ok((res.flags & CANFD_FLAGS.COUNTER_JUMP) !== 0);
});

test('validateCanFdFrame escalates to BUS_OFF after 3 consecutive frame faults', () => {
  let state = { initialized: true, expectedCounter: 1, consecutiveErrors: 0 };
  let lastRes;
  for (let i = 0; i < 3; i++) {
    const f = encodeCanFdFrame({ rollingCounter: 9, corruptCrc: true });
    lastRes = validateCanFdFrame(f, state);
    state = lastRes.nextState;
  }
  assert.ok((lastRes.flags & CANFD_FLAGS.BUS_OFF) !== 0);
  assert.ok(lastRes.flagNames.includes('BUS_OFF'));
});

/* ------------------------------------------------------------------
 * 2. LiDAR-vs-Radar Sensor Plausibility Voter Tests
 * ------------------------------------------------------------------ */
test('Plausibility voter fuses nominal Radar + LiDAR within 3.5 m tolerance', () => {
  const v = runPlausibilityVoter({
    radarRangeM: 27.7,
    radarFlags: CANFD_FLAGS.VALID,
    lidarRangeM: 27.5,
    lidarFlags: CANFD_FLAGS.VALID,
  });
  assert.equal(v.plausibilityOk, true);
  assert.equal(v.activeChannel, 0);
  assert.equal(v.fusedRangeM, 27.59);
});

test('Plausibility voter chooses conservative closer obstacle on valid sensor disagreement (> 3.5 m)', () => {
  const v = runPlausibilityVoter({
    radarRangeM: 45.0,
    radarFlags: CANFD_FLAGS.VALID,
    lidarRangeM: 18.2,
    lidarFlags: CANFD_FLAGS.VALID,
  });
  assert.equal(v.plausibilityOk, false);
  assert.equal(v.fusedRangeM, 18.2);
  assert.ok((v.faultMask & CANFD_FLAGS.SENSOR_DISAGREE) !== 0);
});

test('Plausibility voter switches to Redundant LiDAR Channel B when Radar fails CRC', () => {
  const v = runPlausibilityVoter({
    radarRangeM: 180.0,
    radarFlags: CANFD_FLAGS.BAD_CRC16 | CANFD_FLAGS.COUNTER_JUMP,
    lidarRangeM: 26.6,
    lidarFlags: CANFD_FLAGS.VALID,
  });
  assert.equal(v.activeChannel, 1);
  assert.equal(v.fusedRangeM, 26.6);
  assert.equal(v.voterLatencyUs, 18);
});

test('Plausibility voter engages Emergency Hydraulic Backup on dual-channel failure', () => {
  const v = runPlausibilityVoter({
    radarRangeM: 180.0,
    radarFlags: CANFD_FLAGS.BAD_CRC16,
    lidarRangeM: 180.0,
    lidarFlags: CANFD_FLAGS.BUS_OFF,
  });
  assert.equal(v.activeChannel, 2);
  assert.ok((v.faultMask & CANFD_FLAGS.BUS_OFF) !== 0);
});

test('generateScenarioCanFdBurst rejects spoofed frame in QNX mode but exposes vulnerability in Non-RT mode', () => {
  const qnxBurst = generateScenarioCanFdBurst({ rangeM: 26.8, egoSpeedMps: 23.61, frictionMu: 0.9, spoofInjected: true, decelCmdMps2: 8.83, mode: 'qnx' });
  const nonRtBurst = generateScenarioCanFdBurst({ rangeM: 26.8, egoSpeedMps: 23.61, frictionMu: 0.9, spoofInjected: true, decelCmdMps2: 8.83, mode: 'non-rt' });
  assert.equal(qnxBurst.rejectedCount, 1);
  assert.equal(qnxBurst.unvalidatedAdmittedCount, 0);
  assert.equal(nonRtBurst.unvalidatedAdmittedCount, 1);
});

/* ------------------------------------------------------------------
 * 3. All 5 Automotive Scenarios & Physics Verification
 * ------------------------------------------------------------------ */
test('Catalog contains exactly 5 distinct automotive ADAS scenarios', () => {
  assert.equal(SCENARIOS.length, 5);
  const ids = new Set(SCENARIOS.map((s) => s.id));
  assert.equal(ids.size, 5);
});

test('Scenario 1 (highway-cutin): 110 km/h, TTC 1.42s, QNX ~+4.8m safe vs Non-RT ~-2.4m collision', () => {
  const sim = runAdasSimulation('highway-cutin', { mode: 'qnx' });
  assert.equal(sim.qnx.ttcS, 1.42);
  assert.equal(sim.qnx.collision, false);
  assert.ok(sim.qnx.clearanceM >= 4.7 && sim.qnx.clearanceM <= 4.95, `expected ~4.8m got ${sim.qnx.clearanceM}`);
  assert.ok(sim.qnx.totalLatencyUs < DEADLINE_BUDGET_US);
  assert.equal(sim.nonRt.collision, true);
  assert.ok(sim.nonRt.clearanceM <= -2.2 && sim.nonRt.clearanceM >= -2.6, `expected ~-2.4m got ${sim.nonRt.clearanceM}`);
  assert.equal(sim.comparison.collisionPrevented, true);
});

test('Scenario 2 (pedestrian-night): 50 km/h crosswalk, QNX safe stop vs Non-RT collision', () => {
  const sim = runAdasSimulation('pedestrian-night', { mode: 'qnx' });
  assert.equal(sim.scenario.obstacleType, 'pedestrian');
  assert.equal(sim.qnx.collision, false);
  assert.ok(sim.qnx.clearanceM > 3.5);
  assert.equal(sim.nonRt.collision, true);
});

test('Scenario 3 (canfd-spoof): bad CRC-16 rejected in 18 us, redundant LiDAR keeps braking active', () => {
  const sim = runAdasSimulation('canfd-spoof', { mode: 'qnx' });
  assert.equal(sim.config.spoofInjected, true);
  assert.equal(sim.canfd.rejectedCount, 1);
  assert.equal(sim.canfd.unvalidatedAdmittedCount, 0);
  assert.equal(sim.canfd.voter.activeChannel, 1);
  assert.equal(sim.qnx.collision, false);
  assert.equal(sim.nonRt.collision, true);
});

test('Scenario 4 (priority-storm): QNX SCHED_FIFO 63 ~3.9m safe vs Non-RT ~11.4m collision', () => {
  const sim = runAdasSimulation('priority-storm', { mode: 'qnx' });
  assert.equal(sim.qnx.collision, false);
  assert.equal(sim.qnx.preemptionEvents, 0);
  assert.ok(sim.qnx.clearanceM >= 3.7 && sim.qnx.clearanceM <= 4.1, `expected ~3.9m got ${sim.qnx.clearanceM}`);
  assert.equal(sim.nonRt.collision, true);
  assert.ok(sim.nonRt.clearanceM <= -11.0 && sim.nonRt.clearanceM >= -11.7, `expected ~-11.4m got ${sim.nonRt.clearanceM}`);
});

test('Scenario 5 (wet-split-mu): 1 kHz ABS retains 96% adhesion & yaw stability <= 3.5 deg/s', () => {
  const sim = runAdasSimulation('wet-split-mu', { mode: 'qnx' });
  assert.equal(sim.qnx.absEfficiency, 0.96);
  assert.equal(sim.qnx.collision, false);
  assert.ok(sim.qnx.yawRateDegS <= 3.5);
  assert.equal(sim.nonRt.absEfficiency, 0.78);
  assert.equal(sim.nonRt.collision, true);
  assert.ok(sim.nonRt.yawRateDegS > 3.5);
});

test('Stopping distance scales monotonically with speed and inversely with friction mu', () => {
  const slow = evaluateBrakingPass({ egoSpeedKmh: 60, targetSpeedKmh: 0, initialDistanceM: 50, frictionMu: 0.9, mode: 'qnx' });
  const fast = evaluateBrakingPass({ egoSpeedKmh: 120, targetSpeedKmh: 0, initialDistanceM: 50, frictionMu: 0.9, mode: 'qnx' });
  const wet = evaluateBrakingPass({ egoSpeedKmh: 60, targetSpeedKmh: 0, initialDistanceM: 50, frictionMu: FRICTION_PRESETS.wet.mu, mode: 'qnx' });
  const ice = evaluateBrakingPass({ egoSpeedKmh: 60, targetSpeedKmh: 0, initialDistanceM: 50, frictionMu: FRICTION_PRESETS.ice.mu, mode: 'qnx' });

  assert.ok(fast.brakingDistanceM > slow.brakingDistanceM * 3.9);
  assert.ok(wet.brakingDistanceM > slow.brakingDistanceM);
  assert.ok(ice.brakingDistanceM > wet.brakingDistanceM * 2.0);
});

test('Stage breakdown microsecond sum equals totalLatencyUs in both QNX and Non-RT modes', () => {
  for (const sc of SCENARIOS) {
    for (const mode of ['qnx', 'non-rt']) {
      const sim = runAdasSimulation(sc.id, { mode });
      const b = sim.active.stageBreakdownUs;
      assert.equal(b.canfdRxUs + b.crcVoteUs + b.msgSendvUs + b.fusionLawUs + b.actuatorCmdUs, b.totalUs);
      assert.equal(b.totalUs, sim.active.totalLatencyUs);
    }
  }
});

test('Hydraulic caliper pressure stays bounded within [0, 180] bar and matches deceleration', () => {
  const pass = evaluateBrakingPass({ egoSpeedKmh: 110, targetSpeedKmh: 0, initialDistanceM: 80, frictionMu: 0.9, mode: 'qnx' });
  assert.ok(pass.hydraulicBar > 140 && pass.hydraulicBar <= 180);
  assert.ok(Math.abs(pass.maxDecelMps2 - 0.9 * GRAVITY_MPS2) < 0.05);
});

test('Trajectory array has 61 monotonic time points starting at t=0', () => {
  const sim = runAdasSimulation('highway-cutin', { mode: 'qnx' });
  assert.equal(sim.trajectory.length, 61);
  assert.equal(sim.trajectory[0].t, 0);
  assert.equal(sim.trajectory[0].gapM, sim.config.initialDistanceM);
  for (let i = 1; i < sim.trajectory.length; i++) {
    assert.ok(sim.trajectory[i].t > sim.trajectory[i - 1].t);
  }
});

test('PPS snapshot follows authentic QNX /pps/adas/aeb @aeb key:type:value format', () => {
  const sim = runAdasSimulation('highway-cutin', { mode: 'qnx' });
  assert.ok(sim.ppsSnapshot.startsWith('@aeb\n'));
  assert.ok(sim.ppsSnapshot.includes('ipc_latency_us:n:'));
  assert.ok(sim.ppsSnapshot.includes('deadline_misses:n:0'));
  assert.ok(sim.ppsSnapshot.includes('iso26262_asil::ASIL-D'));
});

/* ------------------------------------------------------------------
 * 4. ISO 26262 ASIL-D Audit & Exporters (JSON, Markdown, SARIF 2.1.0)
 * ------------------------------------------------------------------ */
test('All 5 scenarios PASS all 6 ISO 26262 rules (AEB001..AEB006) under QNX 8.0', () => {
  assert.equal(ISO_RULES.length, 6);
  for (const sc of SCENARIOS) {
    const sim = runAdasSimulation(sc.id, { mode: 'qnx' });
    const audit = evaluateIso26262Audit(sim);
    assert.equal(audit.status, 'PASS', `Scenario ${sc.id} failed under QNX: ${audit.summary}`);
    assert.equal(audit.exitCode, 0);
    assert.equal(audit.passedRules, 6);
    assert.equal(audit.score, 100);
  }
});

test('All 5 scenarios trigger SAFETY_VIOLATION (exitCode 2) under Non-RT mode', () => {
  for (const sc of SCENARIOS) {
    const sim = runAdasSimulation(sc.id, { mode: 'non-rt' });
    const audit = evaluateIso26262Audit(sim);
    assert.equal(audit.status, 'SAFETY_VIOLATION');
    assert.equal(audit.exitCode, 2);
    assert.ok(audit.failedRules >= 3);
  }
});

test('Black Ice override at 130 km/h triggers AEB002 clearance violation even under QNX', () => {
  const sim = runAdasSimulation('highway-cutin', { mode: 'qnx', egoSpeedKmh: 130, frictionMu: 0.25 });
  const audit = evaluateIso26262Audit(sim);
  assert.equal(audit.status, 'SAFETY_VIOLATION');
  assert.equal(audit.rules.find((r) => r.id === 'AEB002').passed, false);
});

test('formatAuditJson returns parseable JSON with audit, comparison, and PPS snapshot', () => {
  const sim = runAdasSimulation('highway-cutin', { mode: 'qnx' });
  const audit = evaluateIso26262Audit(sim);
  const parsed = JSON.parse(formatAuditJson(sim, audit));
  assert.equal(parsed.audit.status, 'PASS');
  assert.equal(parsed.qnxVsNonRtComparison.collisionPrevented, true);
  assert.ok(parsed.ppsSnapshot.includes('@aeb'));
});

test('formatAuditMarkdown includes ISO 26262 table and /pps/adas/aeb code block', () => {
  const sim = runAdasSimulation('priority-storm', { mode: 'qnx' });
  const audit = evaluateIso26262Audit(sim);
  const md = formatAuditMarkdown(sim, audit);
  assert.ok(md.includes('# QNX BrakeWire ADAS'));
  assert.ok(md.includes('`AEB001`'));
  assert.ok(md.includes('`AEB006`'));
  assert.ok(md.includes('/pps/adas/aeb'));
});

test('formatAuditSarif emits valid SARIF 2.1.0 with 6 rules and exact C11 artifact locations', () => {
  const sim = runAdasSimulation('canfd-spoof', { mode: 'non-rt' });
  const audit = evaluateIso26262Audit(sim);
  const sarif = JSON.parse(formatAuditSarif(sim, audit));
  assert.equal(sarif.version, '2.1.0');
  assert.equal(sarif.runs.length, 1);
  assert.equal(sarif.runs[0].tool.driver.rules.length, 6);
  assert.ok(sarif.runs[0].results.length >= 4);
  const ruleIds = sarif.runs[0].results.map((r) => r.ruleId);
  assert.ok(ruleIds.includes('AEB001'));
  assert.ok(ruleIds.includes('AEB004'));
  assert.ok(sarif.runs[0].results[0].locations[0].physicalLocation.artifactLocation.uri.startsWith('target-qnx8/'));
});

/* ------------------------------------------------------------------
 * 5. Zero-Dependency CLI Exit Codes & Formats
 * ------------------------------------------------------------------ */
test('CLI exits 0 (PASS) for QNX highway-cutin with valid JSON output', () => {
  const r = spawnSync(process.execPath, [path.join(ROOT, 'cli/brakewire-cli.mjs'), '--scenario', 'highway-cutin', '--format', 'json'], {
    encoding: 'utf-8',
  });
  assert.equal(r.status, 0);
  const out = JSON.parse(r.stdout);
  assert.equal(out.audit.status, 'PASS');
});

test('CLI exits 2 (SAFETY VIOLATION) for Non-RT priority-storm with valid SARIF 2.1.0 output', () => {
  const r = spawnSync(process.execPath, [path.join(ROOT, 'cli/brakewire-cli.mjs'), '--scenario', 'priority-storm', '--mode', 'non-rt', '--format', 'sarif'], {
    encoding: 'utf-8',
  });
  assert.equal(r.status, 2);
  const sarif = JSON.parse(r.stdout);
  assert.equal(sarif.version, '2.1.0');
  assert.equal(sarif.runs[0].invocations[0].exitCode, 2);
});

test('CLI --list prints all 5 scenarios and exits 0', () => {
  const r = spawnSync(process.execPath, [path.join(ROOT, 'cli/brakewire-cli.mjs'), '--list'], {
    encoding: 'utf-8',
  });
  assert.equal(r.status, 0);
  const list = JSON.parse(r.stdout);
  assert.equal(list.length, 5);
});

test('CLI --scenario canfd-spoof --format markdown exits 0 under QNX', () => {
  const r = spawnSync(process.execPath, [path.join(ROOT, 'cli/brakewire-cli.mjs'), '--scenario', 'canfd-spoof', '--format', 'markdown'], {
    encoding: 'utf-8',
  });
  assert.equal(r.status, 0);
  assert.ok(r.stdout.includes('**PASS**'));
});

/* ------------------------------------------------------------------
 * 6. Target QNX 8.0 Codebase & Engine Byte-Parity Verification
 * ------------------------------------------------------------------ */
test('src/ and public/engine/ modules are byte-for-byte identical', () => {
  for (const file of ['canfdValidator.js', 'adasEngine.js', 'iso26262Audit.js']) {
    const srcBuf = fs.readFileSync(path.join(ROOT, 'src', file));
    const pubBuf = fs.readFileSync(path.join(ROOT, 'public/engine', file));
    assert.ok(srcBuf.equals(pubBuf), `Parity mismatch for ${file}`);
  }
});

test('target-qnx8/brakewire_adas.c contains authentic QNX Neutrino 8.0 APIs', () => {
  const code = fs.readFileSync(path.join(ROOT, 'target-qnx8/brakewire_adas.c'), 'utf-8');
  for (const api of [
    '<sys/neutrino.h>',
    '<sys/dispatch.h>',
    '<sys/iofunc.h>',
    '<sys/slog2.h>',
    '<sys/syspage.h>',
    'name_attach(',
    'MsgReceivev(',
    'MsgReplyv(',
    'ClockCycles()',
    'SYSPAGE_ENTRY(qtime)->cycles_per_sec',
    'SCHED_FIFO',
    'ThreadCtl(_NTO_TCTL_RUNMASK',
    'PTHREAD_PRIO_INHERIT',
    '/pps/adas/aeb',
  ]) {
    assert.ok(code.includes(api), `Missing QNX API in brakewire_adas.c: ${api}`);
  }
});

test('target-qnx8/canfd_sensor_rx.c contains MsgSendv(), CRC-16, and plausibility voter', () => {
  const code = fs.readFileSync(path.join(ROOT, 'target-qnx8/canfd_sensor_rx.c'), 'utf-8');
  for (const sym of ['name_open(', 'MsgSendv(', 'bw_crc16_ccitt(', 'run_plausibility_voter(', 'CANFD_FLAG_BAD_CRC16', 'CANFD_FLAG_BUS_OFF']) {
    assert.ok(code.includes(sym), `Missing symbol in canfd_sensor_rx.c: ${sym}`);
  }
});

test('target-qnx8/ build system files exist (Makefile, CMakeLists, toolchain, IFS buildfile, QEMU script)', () => {
  for (const rel of [
    'target-qnx8/Makefile',
    'target-qnx8/CMakeLists.txt',
    'target-qnx8/qnx8-toolchain.cmake',
    'target-qnx8/qnx8-qemu-x86_64.build',
    'target-qnx8/scripts/run-qemu.sh',
    'target-qnx8/README.md',
  ]) {
    const p = path.join(ROOT, rel);
    assert.ok(fs.existsSync(p), `Missing ${rel}`);
    assert.ok(fs.statSync(p).size > 100, `Empty file ${rel}`);
  }
  const ifs = fs.readFileSync(path.join(ROOT, 'target-qnx8/qnx8-qemu-x86_64.build'), 'utf-8');
  assert.ok(ifs.includes('procnto-smp-instr'));
});

test('Root MIT LICENSE file is present and valid', () => {
  const lic = fs.readFileSync(path.join(ROOT, 'LICENSE'), 'utf-8');
  assert.ok(lic.startsWith('MIT License'));
  assert.ok(lic.includes('Permission is hereby granted, free of charge'));
});

test('Zero-speed ego vehicle reports STANDBY state, 99.0s TTC, and 0 braking distance', () => {
  const pass = evaluateBrakingPass({ egoSpeedKmh: 0, targetSpeedKmh: 0, initialDistanceM: 30, frictionMu: 0.9, mode: 'qnx' });
  assert.equal(pass.ttcS, 99);
  assert.equal(pass.brakingDistanceM, 0);
  assert.equal(pass.collision, false);
  assert.equal(pass.aebState, 'STANDBY');
});

test('Ego slower than leading vehicle yields 0 relative velocity and full headway clearance', () => {
  const pass = evaluateBrakingPass({ egoSpeedKmh: 60, targetSpeedKmh: 90, initialDistanceM: 25, frictionMu: 0.9, mode: 'qnx' });
  assert.equal(pass.vRelMps, 0);
  assert.equal(pass.clearanceM, 25);
  assert.equal(pass.collision, false);
});

test('Dynamic spoof toggle on highway-cutin keeps QNX PASS while failing Non-RT AEB004', () => {
  const qnxSim = runAdasSimulation('highway-cutin', { mode: 'qnx', spoofInjected: true });
  const nonRtSim = runAdasSimulation('highway-cutin', { mode: 'non-rt', spoofInjected: true });
  assert.equal(evaluateIso26262Audit(qnxSim).status, 'PASS');
  assert.equal(evaluateIso26262Audit(nonRtSim).rules.find((r) => r.id === 'AEB004').passed, false);
});

test('Dynamic CPU storm toggle on pedestrian-night keeps QNX < 500 us with 0 preemptions', () => {
  const sim = runAdasSimulation('pedestrian-night', { mode: 'qnx', stormInjected: true });
  assert.ok(sim.active.totalLatencyUs < 500);
  assert.equal(sim.active.preemptionEvents, 0);
  assert.equal(sim.active.cpuLoadPct, 98);
  assert.equal(sim.active.collision, false);
});

test('Unknown scenario ID gracefully falls back to highway-cutin default', () => {
  const sim = runAdasSimulation('non-existent-scenario', { mode: 'qnx' });
  assert.equal(sim.scenario.id, 'highway-cutin');
  assert.equal(sim.qnx.ttcS, 1.42);
});

