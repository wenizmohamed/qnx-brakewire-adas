/**
 * QNX BrakeWire ADAS - Deterministic ISO 26262 ASIL-D AEB Physics & QNX Timing Engine
 * Pure ESM (zero dependencies; byte-identically synced into public/engine/).
 */

import { generateScenarioCanFdBurst } from './canfdValidator.js';

export const GRAVITY_MPS2 = 9.80665;
export const DEADLINE_BUDGET_US = 1000;
export const CALIPER_HALF_RAMP_S = 0.07; // 140 ms electro-hydraulic build-up -> 70 ms effective kinematic delay

export const FRICTION_PRESETS = Object.freeze({
  dry: { id: 'dry', label: 'Dry Asphalt', mu: 0.9 },
  wet: { id: 'wet', label: 'Wet Road', mu: 0.55 },
  ice: { id: 'ice', label: 'Black Ice', mu: 0.25 },
});

export const SCENARIOS = Object.freeze([
  {
    id: 'highway-cutin',
    shortTitle: 'Highway Cut-In',
    title: 'Highway Cut-In at 110 km/h',
    subtitle: 'Aggressive lane cut-in ahead at 40 km/h (TTC 1.42 s); zero-copy MsgSendv() AEB stop',
    obstacleType: 'vehicle',
    egoSpeedKmh: 110,
    targetSpeedKmh: 40,
    initialDistanceM: 27.61,
    frictionMu: 0.9,
    splitMu: false,
    spoofInjected: false,
    stormInjected: false,
    qnxBaseIpcUs: 412,
    nonRtJitterMs: 370,
    fusionConfidence: 0.992,
    lidarPoints: 64,
    environment: 'Daylight 3-Lane Expressway',
  },
  {
    id: 'pedestrian-night',
    shortTitle: 'Pedestrian Night',
    title: 'Pedestrian Night Crosswalk at 50 km/h',
    subtitle: 'Unlit urban crosswalk at 50 km/h; LiDAR 3D cluster + 77 GHz Radar Doppler lock',
    obstacleType: 'pedestrian',
    egoSpeedKmh: 50,
    targetSpeedKmh: 0,
    initialDistanceM: 16.4,
    frictionMu: 0.85,
    splitMu: false,
    spoofInjected: false,
    stormInjected: false,
    qnxBaseIpcUs: 396,
    nonRtJitterMs: 340,
    fusionConfidence: 0.984,
    lidarPoints: 28,
    environment: 'Low-Lux Urban Crosswalk (12 lux)',
  },
  {
    id: 'canfd-spoof',
    shortTitle: 'CAN-FD Spoof',
    title: 'CAN-FD Spoofed Frame / Bad CRC-16 Attack',
    subtitle: 'Malicious 0x120 clear-road spoof rejected in 18 us; redundant LiDAR Channel B holds AEB',
    obstacleType: 'vehicle',
    egoSpeedKmh: 85,
    targetSpeedKmh: 15,
    initialDistanceM: 26.8,
    frictionMu: 0.9,
    splitMu: false,
    spoofInjected: true,
    stormInjected: false,
    qnxBaseIpcUs: 418,
    nonRtJitterMs: 520,
    fusionConfidence: 0.971,
    lidarPoints: 52,
    environment: 'Arterial Corridor (Active Bus Attack)',
  },
  {
    id: 'priority-storm',
    shortTitle: 'Priority Storm',
    title: 'Infotainment Priority Storm vs SCHED_FIFO 63',
    subtitle: '98% IVI/OTA CPU load vs QNX SCHED_FIFO 63 pinned CPU0; 3.9 m safe stop vs 11.4 m collision',
    obstacleType: 'vehicle',
    egoSpeedKmh: 100,
    targetSpeedKmh: 20,
    initialDistanceM: 33.4,
    frictionMu: 0.9,
    splitMu: false,
    spoofInjected: false,
    stormInjected: true,
    qnxBaseIpcUs: 428,
    nonRtJitterMs: 685,
    fusionConfidence: 0.989,
    lidarPoints: 58,
    environment: 'Highway + 8-Thread IVI/OTA CPU Storm',
  },
  {
    id: 'wet-split-mu',
    shortTitle: 'Wet Split-Mu ABS',
    title: 'Wet-Road ABS Split-Mu Slip Recovery',
    subtitle: 'Asymmetric grip (0.62 left / 0.48 right); 1 kHz QNX ABS loop maintains yaw stability',
    obstacleType: 'barrier',
    egoSpeedKmh: 75,
    targetSpeedKmh: 0,
    initialDistanceM: 46.5,
    frictionMu: 0.55,
    splitMu: true,
    spoofInjected: false,
    stormInjected: false,
    qnxBaseIpcUs: 434,
    nonRtJitterMs: 290,
    fusionConfidence: 0.981,
    lidarPoints: 74,
    environment: 'Rain-Slicked Split-Mu Highway',
  },
]);

export const SCENARIO_MAP = Object.freeze(Object.fromEntries(SCENARIOS.map((s) => [s.id, s])));

export function getScenarioById(id) {
  return SCENARIO_MAP[id] || SCENARIO_MAP['highway-cutin'];
}

function round2(v) {
  return Math.round(v * 100) / 100;
}

function round3(v) {
  return Math.round(v * 1000) / 1000;
}

/**
 * Evaluate deterministic kinematics, IPC microsecond budget, and AEB outcome
 * for either QNX Neutrino 8.0 (SCHED_FIFO 63 + MsgSendv) or Non-RT GPOS.
 */
export function evaluateBrakingPass({
  egoSpeedKmh,
  targetSpeedKmh,
  initialDistanceM,
  frictionMu,
  splitMu = false,
  spoofInjected = false,
  stormInjected = false,
  qnxBaseIpcUs = 412,
  nonRtJitterMs = 370,
  mode = 'qnx',
}) {
  const isQnx = mode === 'qnx';
  const vEgoMps = Math.max(0, egoSpeedKmh) / 3.6;
  const vTargetMps = Math.max(0, targetSpeedKmh) / 3.6;
  const vRelMps = Math.max(0, vEgoMps - vTargetMps);
  const d0 = Math.max(0.1, initialDistanceM);
  const mu = Math.max(0.12, Math.min(1.1, frictionMu));

  const ttcS = vRelMps > 0.05 ? round2(d0 / vRelMps) : 99.0;

  // Deterministic stage-by-stage timing breakdown (microseconds)
  let stageBreakdownUs;
  if (isQnx) {
    const canfdRxUs = 86;
    const crcVoteUs = spoofInjected ? 18 : 14;
    const msgSendvUs = stormInjected ? 74 : 62;
    const fusionLawUs = splitMu ? 168 : 154;
    const actuatorCmdUs = Math.max(70, qnxBaseIpcUs - (canfdRxUs + crcVoteUs + msgSendvUs + fusionLawUs));
    const stormOverheadUs = stormInjected ? 16 : 0; // Hardware IRQ + TLB noise only; SCHED_FIFO 63 never preempted
    const totalUs = canfdRxUs + crcVoteUs + msgSendvUs + fusionLawUs + actuatorCmdUs + stormOverheadUs;
    stageBreakdownUs = {
      canfdRxUs,
      crcVoteUs,
      msgSendvUs: msgSendvUs + stormOverheadUs,
      fusionLawUs,
      actuatorCmdUs,
      totalUs,
    };
  } else {
    // Non-RT GPOS: CFS scheduler preemption, socket/heap copies, unguarded spoof hold, priority inversion
    let jitterMs = nonRtJitterMs;
    if (stormInjected && jitterMs < 620) jitterMs += 265;
    if (spoofInjected && jitterMs < 480) jitterMs += 190;
    const totalUs = Math.round(jitterMs * 1000);
    stageBreakdownUs = {
      canfdRxUs: Math.round(totalUs * 0.14),
      crcVoteUs: spoofInjected ? Math.round(totalUs * 0.34) : Math.round(totalUs * 0.05),
      msgSendvUs: Math.round(totalUs * 0.42),
      fusionLawUs: Math.round(totalUs * 0.12),
      actuatorCmdUs: totalUs - Math.round(totalUs * 0.14) - (spoofInjected ? Math.round(totalUs * 0.34) : Math.round(totalUs * 0.05)) - Math.round(totalUs * 0.42) - Math.round(totalUs * 0.12),
      totalUs,
    };
  }

  // Split-mu ABS slip efficiency: QNX 1 kHz modulation retains 96% vs Non-RT jittery modulation 78%
  const absEfficiency = splitMu || mu <= 0.55 ? (isQnx ? 0.96 : 0.78) : 1.0;
  const maxDecelMps2 = mu * absEfficiency * GRAVITY_MPS2;

  const rtLatencyS = stageBreakdownUs.totalUs / 1e6;
  const effectiveDelayS = rtLatencyS + CALIPER_HALF_RAMP_S;
  const delayDistanceM = vRelMps * effectiveDelayS;
  const brakingDistanceM = maxDecelMps2 > 0 ? (vRelMps * vRelMps) / (2 * maxDecelMps2) : 999;
  const totalStoppingDistanceM = delayDistanceM + brakingDistanceM;
  const clearanceM = d0 - totalStoppingDistanceM;
  const collision = clearanceM < 0;

  // Residual impact speed if collision occurs
  let impactSpeedKmh = 0;
  if (collision) {
    const remainingBrakingDist = Math.max(0, d0 - delayDistanceM);
    const vImpactSq = Math.max(0, vRelMps * vRelMps - 2 * maxDecelMps2 * remainingBrakingDist);
    impactSpeedKmh = round2(Math.sqrt(vImpactSq) * 3.6);
  }

  const hydraulicBar = round2(Math.min(180, (maxDecelMps2 / GRAVITY_MPS2) * 165));
  const deadlineMisses = stageBreakdownUs.totalUs > DEADLINE_BUDGET_US ? Math.max(1, Math.floor(stageBreakdownUs.totalUs / DEADLINE_BUDGET_US)) : 0;
  const budgetUtilizationPct = round2((stageBreakdownUs.totalUs / DEADLINE_BUDGET_US) * 100);

  // Yaw stability (deg/s) during braking — elevated under split-mu or ice if non-RT ABS lags
  let yawRateDegS = 0.35;
  if (splitMu || mu <= 0.55) {
    yawRateDegS = isQnx ? (mu <= 0.3 ? 2.4 : 1.65) : (mu <= 0.3 ? 11.8 : 7.9);
  } else if (!isQnx && stormInjected) {
    yawRateDegS = 3.2;
  }

  let aebState = 'FULL_EMERGENCY';
  let aebStateCode = 3;
  if (ttcS > 3.4 && clearanceM > 12) {
    aebState = 'STANDBY';
    aebStateCode = 0;
  } else if (ttcS > 2.6 && clearanceM > 8) {
    aebState = 'PARTIAL_BRAKE';
    aebStateCode = 2;
  }

  const brakeDurationS = maxDecelMps2 > 0 ? vRelMps / maxDecelMps2 : 0;
  const totalStopTimeS = round2(effectiveDelayS + brakeDurationS);

  return {
    mode: isQnx ? 'qnx' : 'non-rt',
    schedulerLabel: isQnx ? 'QNX 8.0 SCHED_FIFO Prio 63 (CPU0 Pinned)' : 'Non-RT GPOS CFS (Unpinned)',
    ipcLabel: isQnx ? 'Zero-Copy MsgSendv() / MsgReceivev()' : 'Userspace Socket + Heap Copy',
    egoSpeedKmh: round2(egoSpeedKmh),
    targetSpeedKmh: round2(targetSpeedKmh),
    vEgoMps: round2(vEgoMps),
    vTargetMps: round2(vTargetMps),
    vRelMps: round2(vRelMps),
    initialDistanceM: round2(d0),
    frictionMu: round2(mu),
    ttcS,
    stageBreakdownUs,
    totalLatencyUs: stageBreakdownUs.totalUs,
    totalLatencyMs: round3(stageBreakdownUs.totalUs / 1000),
    effectiveDelayS: round3(effectiveDelayS),
    absEfficiency,
    maxDecelMps2: round2(maxDecelMps2),
    hydraulicBar,
    delayDistanceM: round2(delayDistanceM),
    brakingDistanceM: round2(brakingDistanceM),
    totalStoppingDistanceM: round2(totalStoppingDistanceM),
    clearanceM: round2(clearanceM),
    collision,
    impactSpeedKmh,
    aebState,
    aebStateCode,
    deadlineBudgetUs: DEADLINE_BUDGET_US,
    deadlineMisses,
    budgetUtilizationPct,
    yawRateDegS: round2(yawRateDegS),
    totalStopTimeS,
    cpuLoadPct: stormInjected ? 98 : 34,
    preemptionEvents: isQnx ? 0 : stormInjected ? 19 : 4,
  };
}

/**
 * Generate a 60-step kinematic deceleration trajectory for real-time visualization.
 */
function buildTrajectory(pass) {
  const steps = 60;
  const maxT = Math.max(2.2, pass.totalStopTimeS + 0.35);
  const points = [];

  for (let i = 0; i <= steps; i++) {
    const t = (i / steps) * maxT;
    let vRel = pass.vRelMps;
    let closedDist = 0;
    let decel = 0;

    if (t <= pass.effectiveDelayS) {
      closedDist = pass.vRelMps * t;
      decel = 0;
    } else {
      const tb = t - pass.effectiveDelayS;
      const stopTb = pass.maxDecelMps2 > 0 ? pass.vRelMps / pass.maxDecelMps2 : 0;
      if (tb < stopTb) {
        vRel = pass.vRelMps - pass.maxDecelMps2 * tb;
        closedDist = pass.vRelMps * pass.effectiveDelayS + pass.vRelMps * tb - 0.5 * pass.maxDecelMps2 * tb * tb;
        decel = pass.maxDecelMps2;
      } else {
        vRel = 0;
        closedDist = pass.totalStoppingDistanceM;
        decel = 0;
      }
    }

    const gapM = pass.initialDistanceM - closedDist;
    const egoSpeedKmh = Math.max(pass.targetSpeedKmh, (vRel + pass.vTargetMps) * 3.6);

    points.push({
      t: round2(t),
      egoSpeedKmh: round2(egoSpeedKmh),
      gapM: round2(gapM),
      decelMps2: round2(decel),
      collided: gapM < 0,
    });
  }
  return points;
}

/**
 * Format an authentic QNX 8.0 /pps/adas/aeb Persistent Publish/Subscribe object.
 */
export function buildPpsSnapshot({ scenario, active, canfd }) {
  const lines = [
    '@aeb',
    `scenario::${scenario.id}`,
    `os_mode::${active.mode === 'qnx' ? 'QNX_NEUTRINO_8.0_SCHED_FIFO_63' : 'NON_RT_GPOS_CFS'}`,
    `aeb_state:n:${active.aebStateCode}`,
    `aeb_label::${active.aebState}`,
    `ttc_s:n:${active.ttcS.toFixed(2)}`,
    `decel_mps2:n:${active.maxDecelMps2.toFixed(2)}`,
    `hydraulic_bar:n:${active.hydraulicBar.toFixed(1)}`,
    `clearance_m:n:${active.clearanceM.toFixed(2)}`,
    `ipc_latency_us:n:${active.totalLatencyUs}`,
    `deadline_budget_us:n:${active.deadlineBudgetUs}`,
    `deadline_misses:n:${active.deadlineMisses}`,
    `channel_active:n:${canfd.voter.activeChannel}`,
    `channel_label::${canfd.voter.activeChannelLabel}`,
    `fault_mask:n:${canfd.voter.faultMask}`,
    `spoofs_rejected:n:${canfd.rejectedCount}`,
    `ego_speed_mps:n:${active.vEgoMps.toFixed(2)}`,
    `fused_range_m:n:${canfd.voter.fusedRangeM.toFixed(2)}`,
    `yaw_rate_deg_s:n:${active.yawRateDegS.toFixed(2)}`,
    `iso26262_asil::ASIL-D`,
  ];
  return lines.join('\n');
}

/**
 * Run a complete deterministic QNX BrakeWire ADAS simulation.
 */
export function runAdasSimulation(scenarioId = 'highway-cutin', overrides = {}) {
  const base = getScenarioById(scenarioId);
  const mode = overrides.mode === 'non-rt' ? 'non-rt' : 'qnx';

  const config = {
    egoSpeedKmh: overrides.egoSpeedKmh ?? base.egoSpeedKmh,
    targetSpeedKmh: overrides.targetSpeedKmh ?? base.targetSpeedKmh,
    initialDistanceM: overrides.initialDistanceM ?? base.initialDistanceM,
    frictionMu: overrides.frictionMu ?? base.frictionMu,
    splitMu: overrides.splitMu ?? (base.splitMu || (overrides.frictionMu !== undefined && overrides.frictionMu <= 0.55)),
    spoofInjected: overrides.spoofInjected ?? base.spoofInjected,
    stormInjected: overrides.stormInjected ?? base.stormInjected,
    qnxBaseIpcUs: base.qnxBaseIpcUs,
    nonRtJitterMs: base.nonRtJitterMs,
  };

  const qnx = evaluateBrakingPass({ ...config, mode: 'qnx' });
  const nonRt = evaluateBrakingPass({ ...config, mode: 'non-rt' });
  const active = mode === 'qnx' ? qnx : nonRt;

  const canfd = generateScenarioCanFdBurst({
    rangeM: config.initialDistanceM,
    egoSpeedMps: active.vEgoMps,
    frictionMu: config.frictionMu,
    spoofInjected: config.spoofInjected,
    decelCmdMps2: active.maxDecelMps2,
    mode,
  });

  const comparison = {
    clearanceGainM: round2(qnx.clearanceM - nonRt.clearanceM),
    latencyReductionPct: round2(((nonRt.totalLatencyUs - qnx.totalLatencyUs) / Math.max(1, nonRt.totalLatencyUs)) * 100),
    latencySavedMs: round2((nonRt.totalLatencyUs - qnx.totalLatencyUs) / 1000),
    collisionPrevented: nonRt.collision && !qnx.collision,
  };

  const trajectory = buildTrajectory(active);
  const ppsSnapshot = buildPpsSnapshot({ scenario: base, active, canfd });

  return {
    scenario: base,
    config,
    mode,
    active,
    qnx,
    nonRt,
    comparison,
    canfd,
    trajectory,
    ppsSnapshot,
  };
}
