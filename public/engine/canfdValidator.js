/**
 * QNX BrakeWire ADAS - Deterministic CAN-FD E2E Validator & Plausibility Voter
 * Mirrors target-qnx8/canfd_sensor_rx.c & brakewire_ipc.h bit-for-bit.
 * Pure ESM (zero runtime dependencies, runs identically in Node.js and browser).
 */

export const CANFD_IDS = Object.freeze({
  RADAR_TARGET: 0x120,
  LIDAR_OBJECT: 0x124,
  WHEEL_SPEED: 0x130,
  BRAKE_CMD: 0x140,
  HYDRAULIC_BACKUP: 0x144,
});

export const CANFD_FLAGS = Object.freeze({
  VALID: 0x0000,
  BAD_CRC16: 0x0001,
  COUNTER_JUMP: 0x0002,
  BUS_OFF: 0x0004,
  SENSOR_DISAGREE: 0x0008,
  STALE_TIMESTAMP: 0x0010,
});

export const PLAUSIBILITY_MAX_DELTA_M = 3.5;

/**
 * CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF).
 * Exact match with bw_crc16_ccitt() in target-qnx8/brakewire_ipc.h.
 * @param {Uint8Array|number[]} bytes
 * @returns {number} 16-bit unsigned CRC
 */
export function crc16Ccitt(bytes) {
  let crc = 0xffff;
  for (let i = 0; i < bytes.length; i++) {
    crc ^= (bytes[i] & 0xff) << 8;
    for (let b = 0; b < 8; b++) {
      if (crc & 0x8000) {
        crc = ((crc << 1) ^ 0x1021) & 0xffff;
      } else {
        crc = (crc << 1) & 0xffff;
      }
    }
  }
  return crc;
}

export function decodeFlagNames(mask) {
  if (!mask) return ['VALID'];
  const names = [];
  if (mask & CANFD_FLAGS.BAD_CRC16) names.push('BAD_CRC16');
  if (mask & CANFD_FLAGS.COUNTER_JUMP) names.push('COUNTER_JUMP');
  if (mask & CANFD_FLAGS.BUS_OFF) names.push('BUS_OFF');
  if (mask & CANFD_FLAGS.SENSOR_DISAGREE) names.push('SENSOR_DISAGREE');
  if (mask & CANFD_FLAGS.STALE_TIMESTAMP) names.push('STALE_TIMESTAMP');
  return names;
}

export function toHex(num, width = 4) {
  return '0x' + (num >>> 0).toString(16).toUpperCase().padStart(width, '0');
}

export function bytesToHex(bytes) {
  return Array.from(bytes, (b) => (b & 0xff).toString(16).toUpperCase().padStart(2, '0')).join(' ');
}

/**
 * Encode a 16-byte CAN-FD automotive chassis payload with CRC-16 and rolling counter.
 */
export function encodeCanFdFrame({
  canId = CANFD_IDS.RADAR_TARGET,
  channel = 'RADAR_77GHZ',
  rollingCounter = 0,
  rangeM = 27.61,
  speedMps = 30.56,
  mu = 0.9,
  timestampUs = 1000,
  corruptCrc = false,
  spoofed = false,
}) {
  const payload = new Uint8Array(16);
  const mm = Math.max(0, Math.round(rangeM * 1000)) >>> 0;
  const cmps = Math.max(0, Math.round(speedMps * 100)) >>> 0;
  const muScaled = Math.max(0, Math.min(255, Math.round(mu * 200)));
  const cnt = rollingCounter & 0x0f;

  // Bytes 0..3: range in mm (big-endian)
  payload[0] = (mm >>> 24) & 0xff;
  payload[1] = (mm >>> 16) & 0xff;
  payload[2] = (mm >>> 8) & 0xff;
  payload[3] = mm & 0xff;
  // Byte 4: 4-bit rolling counter + channel nibble
  payload[4] = cnt;
  // Bytes 5..6: speed in cm/s
  payload[5] = (cmps >>> 8) & 0xff;
  payload[6] = cmps & 0xff;
  // Byte 7: road friction scaled
  payload[7] = muScaled;
  // Bytes 8..11: QNX ClockCycles timestamp (us truncated to uint32)
  const ts = timestampUs >>> 0;
  payload[8] = (ts >>> 24) & 0xff;
  payload[9] = (ts >>> 16) & 0xff;
  payload[10] = (ts >>> 8) & 0xff;
  payload[11] = ts & 0xff;
  // Bytes 12..15: E2E DataID signature
  payload[12] = (canId >>> 8) & 0xff;
  payload[13] = canId & 0xff;
  payload[14] = spoofed ? 0xff : 0xa5;
  payload[15] = 0x5a;

  const computedCrc = crc16Ccitt(payload);
  const receivedCrc = corruptCrc ? (computedCrc ^ 0xdead) & 0xffff : computedCrc;

  return {
    canId,
    canIdHex: toHex(canId, 3),
    channel,
    dlc: 16,
    rollingCounter: cnt,
    rangeM: Number(rangeM.toFixed(2)),
    speedMps: Number(speedMps.toFixed(2)),
    timestampUs,
    payload,
    hexPayload: bytesToHex(payload),
    crc16Expected: computedCrc,
    crc16Received: receivedCrc,
    crc16ExpectedHex: toHex(computedCrc, 4),
    crc16ReceivedHex: toHex(receivedCrc, 4),
    spoofed,
  };
}

/**
 * Validate a single CAN-FD frame against CRC-16/CCITT-FALSE and monotonic 4-bit counter.
 */
export function validateCanFdFrame(frame, channelState = { initialized: false, expectedCounter: 0, consecutiveErrors: 0 }) {
  let flags = CANFD_FLAGS.VALID;

  if (!frame || !frame.payload || frame.dlc <= 0 || frame.dlc > 64) {
    return {
      valid: false,
      flags: CANFD_FLAGS.BAD_CRC16,
      flagNames: ['BAD_CRC16'],
      computedCrc16: 0,
      receivedCrc16: frame?.crc16Received ?? 0,
      validationTimeUs: 18,
      nextState: {
        initialized: channelState.initialized,
        expectedCounter: channelState.expectedCounter,
        consecutiveErrors: (channelState.consecutiveErrors || 0) + 1,
      },
    };
  }

  const computedCrc16 = crc16Ccitt(frame.payload.slice(0, frame.dlc));
  if (computedCrc16 !== frame.crc16Received) {
    flags |= CANFD_FLAGS.BAD_CRC16;
  }

  if (channelState.initialized) {
    const delta = (frame.rollingCounter - channelState.expectedCounter) & 0x0f;
    if (delta !== 0) {
      flags |= CANFD_FLAGS.COUNTER_JUMP;
    }
  }

  const nextErrors = flags === CANFD_FLAGS.VALID ? 0 : (channelState.consecutiveErrors || 0) + 1;
  if (nextErrors >= 3) {
    flags |= CANFD_FLAGS.BUS_OFF;
  }

  const nextState =
    flags === CANFD_FLAGS.VALID
      ? {
          initialized: true,
          expectedCounter: (frame.rollingCounter + 1) & 0x0f,
          consecutiveErrors: 0,
        }
      : {
          initialized: channelState.initialized,
          expectedCounter: channelState.expectedCounter,
          consecutiveErrors: nextErrors,
        };

  return {
    valid: flags === CANFD_FLAGS.VALID,
    flags,
    flagNames: decodeFlagNames(flags),
    computedCrc16,
    receivedCrc16: frame.crc16Received,
    validationTimeUs: flags === CANFD_FLAGS.VALID ? 14 : 18,
    nextState,
  };
}

/**
 * Deterministic LiDAR-vs-Radar Sensor Plausibility Voter (mirrors target-qnx8/canfd_sensor_rx.c).
 */
export function runPlausibilityVoter({ radarRangeM, radarFlags, lidarRangeM, lidarFlags, maxDeltaM = PLAUSIBILITY_MAX_DELTA_M }) {
  const radarOk = radarFlags === CANFD_FLAGS.VALID;
  const lidarOk = lidarFlags === CANFD_FLAGS.VALID;
  const deltaM = Number(Math.abs(radarRangeM - lidarRangeM).toFixed(2));
  let faultMask = radarFlags | lidarFlags;
  let fusedRangeM = 0;
  let plausibilityOk = false;
  let activeChannel = 0;
  let activeChannelLabel = 'Primary Fused (LiDAR + 77 GHz Radar)';
  let fallbackReason = 'Sensors nominal & cross-validated within 3.5 m tolerance';

  if (radarOk && lidarOk) {
    if (deltaM <= maxDeltaM) {
      fusedRangeM = Number((radarRangeM * 0.45 + lidarRangeM * 0.55).toFixed(2));
      plausibilityOk = true;
      activeChannel = 0;
    } else {
      fusedRangeM = Number(Math.min(radarRangeM, lidarRangeM).toFixed(2));
      plausibilityOk = false;
      faultMask |= CANFD_FLAGS.SENSOR_DISAGREE;
      activeChannel = 1;
      activeChannelLabel = 'Conservative Min-Range Fail-Safe (Disagreement)';
      fallbackReason = `LiDAR/Radar delta ${deltaM} m exceeds ${maxDeltaM} m threshold`;
    }
  } else if (lidarOk) {
    fusedRangeM = Number(lidarRangeM.toFixed(2));
    plausibilityOk = false;
    activeChannel = 1;
    activeChannelLabel = 'Redundant LiDAR Channel B (Radar Rejected)';
    fallbackReason = '77 GHz Radar CAN-FD frame rejected (CRC-16/Counter fault); LiDAR Channel B locked in 18 us';
  } else if (radarOk) {
    fusedRangeM = Number(radarRangeM.toFixed(2));
    plausibilityOk = false;
    activeChannel = 1;
    activeChannelLabel = 'Redundant Radar Channel A (LiDAR Fault)';
    fallbackReason = 'LiDAR channel degraded; Radar Channel A active';
  } else {
    fusedRangeM = 0;
    plausibilityOk = false;
    faultMask |= CANFD_FLAGS.BUS_OFF;
    activeChannel = 2;
    activeChannelLabel = 'Emergency Hydraulic Backup Channel';
    fallbackReason = 'Dual-channel bus-off fault; direct hydraulic emergency pressure engaged';
  }

  return {
    radarRangeM: Number(radarRangeM.toFixed(2)),
    lidarRangeM: Number(lidarRangeM.toFixed(2)),
    fusedRangeM,
    deltaM,
    radarValid: radarOk,
    lidarValid: lidarOk,
    plausibilityOk,
    activeChannel,
    activeChannelLabel,
    fallbackReason,
    faultMask,
    faultNames: decodeFlagNames(faultMask),
    voterLatencyUs: radarOk && lidarOk ? 14 : 18,
  };
}

/**
 * Build an authentic multi-frame CAN-FD bus trace for the active scenario & fault toggles.
 */
export function generateScenarioCanFdBurst({ rangeM, egoSpeedMps, frictionMu, spoofInjected, decelCmdMps2, mode = 'qnx' }) {
  const frames = [];
  let radarState = { initialized: true, expectedCounter: 7, consecutiveErrors: 0 };
  let lidarState = { initialized: true, expectedCounter: 7, consecutiveErrors: 0 };
  let wheelState = { initialized: true, expectedCounter: 7, consecutiveErrors: 0 };

  const rawDescriptors = [
    {
      canId: CANFD_IDS.RADAR_TARGET,
      channel: 'RADAR_77GHZ',
      rollingCounter: 7,
      rangeM: rangeM + 0.08,
      speedMps: egoSpeedMps,
      mu: frictionMu,
      timestampUs: 100120,
      corruptCrc: false,
      spoofed: false,
      stateKey: 'radar',
    },
    {
      canId: CANFD_IDS.LIDAR_OBJECT,
      channel: 'LIDAR_CH_B',
      rollingCounter: 7,
      rangeM: rangeM - 0.06,
      speedMps: egoSpeedMps,
      mu: frictionMu,
      timestampUs: 100142,
      corruptCrc: false,
      spoofed: false,
      stateKey: 'lidar',
    },
    {
      canId: CANFD_IDS.WHEEL_SPEED,
      channel: 'WHEEL_IMU',
      rollingCounter: 7,
      rangeM: rangeM,
      speedMps: egoSpeedMps,
      mu: frictionMu,
      timestampUs: 100168,
      corruptCrc: false,
      spoofed: false,
      stateKey: 'wheel',
    },
    {
      canId: CANFD_IDS.RADAR_TARGET,
      channel: 'RADAR_77GHZ',
      rollingCounter: spoofInjected ? 12 : 8,
      rangeM: spoofInjected ? 180.0 : Math.max(0.5, rangeM - 0.18),
      speedMps: egoSpeedMps,
      mu: frictionMu,
      timestampUs: 101118,
      corruptCrc: Boolean(spoofInjected),
      spoofed: Boolean(spoofInjected),
      stateKey: 'radar',
    },
    {
      canId: CANFD_IDS.LIDAR_OBJECT,
      channel: 'LIDAR_CH_B',
      rollingCounter: 8,
      rangeM: Math.max(0.5, rangeM - 0.21),
      speedMps: egoSpeedMps,
      mu: frictionMu,
      timestampUs: 101136,
      corruptCrc: false,
      spoofed: false,
      stateKey: 'lidar',
    },
    {
      canId: CANFD_IDS.BRAKE_CMD,
      channel: 'ACTUATOR_BBW',
      rollingCounter: 8,
      rangeM: decelCmdMps2,
      speedMps: egoSpeedMps,
      mu: frictionMu,
      timestampUs: mode === 'qnx' ? 101532 : 471200,
      corruptCrc: false,
      spoofed: false,
      stateKey: 'wheel',
    },
  ];

  let latestRadar = { rangeM: rangeM, flags: CANFD_FLAGS.VALID };
  let latestLidar = { rangeM: rangeM, flags: CANFD_FLAGS.VALID };

  for (const d of rawDescriptors) {
    const encoded = encodeCanFdFrame(d);
    let validation;
    if (d.stateKey === 'radar') {
      validation = validateCanFdFrame(encoded, radarState);
      radarState = validation.nextState;
      latestRadar = { rangeM: encoded.rangeM, flags: validation.flags };
    } else if (d.stateKey === 'lidar') {
      validation = validateCanFdFrame(encoded, lidarState);
      lidarState = validation.nextState;
      latestLidar = { rangeM: encoded.rangeM, flags: validation.flags };
    } else {
      validation = validateCanFdFrame(encoded, wheelState);
      wheelState = validation.nextState;
    }

    const admittedByOs = mode === 'qnx' ? validation.valid : true;
    const action = validation.valid
      ? 'ADMITTED (MsgSendv)'
      : mode === 'qnx'
        ? `REJECTED in ${validation.validationTimeUs} us -> Redundant LiDAR`
        : 'VULNERABLE: Accepted without E2E check';

    frames.push({
      canId: encoded.canId,
      canIdHex: encoded.canIdHex,
      channel: encoded.channel,
      dlc: encoded.dlc,
      rollingCounter: encoded.rollingCounter,
      rangeM: encoded.rangeM,
      timestampUs: encoded.timestampUs,
      hexPayload: encoded.hexPayload,
      crc16ExpectedHex: encoded.crc16ExpectedHex,
      crc16ReceivedHex: encoded.crc16ReceivedHex,
      valid: validation.valid,
      flags: validation.flags,
      flagNames: validation.flagNames,
      validationTimeUs: validation.validationTimeUs,
      spoofed: encoded.spoofed,
      admittedByOs,
      action,
    });
  }

  const voter = runPlausibilityVoter({
    radarRangeM: latestRadar.rangeM,
    radarFlags: latestRadar.flags,
    lidarRangeM: latestLidar.rangeM,
    lidarFlags: latestLidar.flags,
  });

  const rejectedCount = frames.filter((f) => !f.valid && mode === 'qnx').length;
  const unvalidatedAdmittedCount = frames.filter((f) => !f.valid && f.admittedByOs).length;

  return {
    frames,
    voter,
    rejectedCount,
    unvalidatedAdmittedCount,
  };
}
