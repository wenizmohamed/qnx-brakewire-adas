/**
 * QNX BrakeWire ADAS - ISO 26262 ASIL-D & QNX 8.0 Safety Audit Engine
 * Evaluates rules AEB001..AEB006 and exports JSON, Markdown, and SARIF 2.1.0.
 * Pure ESM (zero runtime dependencies; synced to public/engine/).
 */

export const ISO_RULES = Object.freeze([
  {
    id: 'AEB001',
    name: 'HardRtLoopDeadlineCompliance',
    shortDescription: 'End-to-end AEB braking loop must complete within 1,000 us QNX ClockCycles() budget',
    asil: 'ASIL-D',
    artifactUri: 'target-qnx8/brakewire_adas.c',
    startLine: 151,
  },
  {
    id: 'AEB002',
    name: 'CollisionFreeStoppingClearance',
    shortDescription: 'Autonomous Emergency Braking must achieve >= 0.50 m safe stopping clearance',
    asil: 'ASIL-D',
    artifactUri: 'target-qnx8/brakewire_adas.c',
    startLine: 112,
  },
  {
    id: 'AEB003',
    name: 'SchedFifoPriorityIsolation',
    shortDescription: 'Brake actuator thread must maintain SCHED_FIFO priority 63 isolation with 0 preemptions',
    asil: 'ASIL-D',
    artifactUri: 'target-qnx8/brakewire_adas.c',
    startLine: 72,
  },
  {
    id: 'AEB004',
    name: 'CanFdE2eCrcAndCounterIntegrity',
    shortDescription: 'Spoofed or bad-CRC-16 CAN-FD frames must be rejected in <= 50 us with 0 unvalidated frames admitted',
    asil: 'ASIL-D',
    artifactUri: 'target-qnx8/canfd_sensor_rx.c',
    startLine: 45,
  },
  {
    id: 'AEB005',
    name: 'SensorPlausibilityVoterFailOperational',
    shortDescription: 'LiDAR + 77 GHz Radar Plausibility Voter must maintain verified range channel under faults',
    asil: 'ASIL-D',
    artifactUri: 'target-qnx8/canfd_sensor_rx.c',
    startLine: 86,
  },
  {
    id: 'AEB006',
    name: 'SplitMuAbsYawStabilityEnvelope',
    shortDescription: 'ABS wheel-slip loop must hold yaw rate <= 3.5 deg/s across wet or split-mu surfaces',
    asil: 'ASIL-C',
    artifactUri: 'target-qnx8/brakewire_adas.c',
    startLine: 118,
  },
]);

/**
 * Evaluate ISO 26262 ASIL-D compliance rules AEB001..AEB006 for a simulation result.
 */
export function evaluateIso26262Audit(sim) {
  const a = sim.active;
  const canfd = sim.canfd;
  const spoofRejectedFast =
    canfd.unvalidatedAdmittedCount === 0 &&
    canfd.frames.every((f) => f.valid || f.validationTimeUs <= 50);

  const evaluations = [
    {
      ...ISO_RULES[0],
      passed: a.totalLatencyUs <= a.deadlineBudgetUs && a.deadlineMisses === 0,
      measured: `${a.totalLatencyUs} us / ${a.deadlineBudgetUs} us budget (${a.deadlineMisses} misses)`,
      remediation:
        'Run brakewire_adas on QNX Neutrino 8.0 with MsgSendv()/MsgReceivev() zero-copy IPC and ClockCycles() budget guard.',
    },
    {
      ...ISO_RULES[1],
      passed: !a.collision && a.clearanceM >= 0.5,
      measured: a.collision
        ? `COLLISION (${a.clearanceM} m, impact ${a.impactSpeedKmh} km/h)`
        : `+${a.clearanceM} m safe clearance`,
      remediation:
        'Eliminate non-RT scheduling jitter via QNX SCHED_FIFO 63 or increase headway / reduce cruise velocity on low-mu surfaces.',
    },
    {
      ...ISO_RULES[2],
      passed: a.mode === 'qnx' && a.preemptionEvents === 0,
      measured:
        a.mode === 'qnx'
          ? `SCHED_FIFO prio 63 pinned CPU0 (0 preemptions @ ${a.cpuLoadPct}% load)`
          : `Non-RT CFS (${a.preemptionEvents} preemptions @ ${a.cpuLoadPct}% load)`,
      remediation:
        'Pin braking thread via ThreadCtl(_NTO_TCTL_RUNMASK, 0x01) and elevate to SCHED_FIFO priority 63 with PTHREAD_PRIO_INHERIT.',
    },
    {
      ...ISO_RULES[3],
      passed: spoofRejectedFast,
      measured: sim.config.spoofInjected
        ? canfd.unvalidatedAdmittedCount === 0
          ? `Spoofed 0x120 rejected in ${canfd.voter.voterLatencyUs} us (0 unvalidated admitted)`
          : `${canfd.unvalidatedAdmittedCount} unvalidated spoofed frame(s) admitted to actuator`
        : 'All CAN-FD CRC-16/CCITT-FALSE & rolling counters verified (0 faults)',
      remediation:
        'Enforce bw_crc16_ccitt() and 4-bit monotonic rolling-counter check inside canfd_sensor_rx before MsgSendv().',
    },
    {
      ...ISO_RULES[4],
      passed: canfd.voter.activeChannel !== 2 && (canfd.voter.radarValid || canfd.voter.lidarValid),
      measured: `${canfd.voter.activeChannelLabel} (fused ${canfd.voter.fusedRangeM} m)`,
      remediation:
        'Verify dual-sensor plausibility voter wiring and fallback routing to emergency hydraulic channel.',
    },
    {
      ...ISO_RULES[5],
      passed: a.yawRateDegS <= 3.5,
      measured: `${a.yawRateDegS} deg/s yaw rate (limit 3.50 deg/s, mu=${a.frictionMu})`,
      remediation:
        'Maintain 1 kHz deterministic ABS wheel-slip PID loop under QNX 8.0 to prevent split-mu yaw divergence.',
    },
  ];

  const passedRules = evaluations.filter((r) => r.passed).length;
  const failedRules = evaluations.length - passedRules;
  const status = failedRules === 0 ? 'PASS' : 'SAFETY_VIOLATION';
  const exitCode = failedRules === 0 ? 0 : 2;
  const score = Math.round((passedRules / evaluations.length) * 100);

  return {
    tool: 'QNX BrakeWire ADAS ISO 26262 Safety Auditor',
    version: '1.0.0',
    standard: 'ISO 26262-6 ASIL-D',
    scenarioId: sim.scenario.id,
    scenarioTitle: sim.scenario.title,
    mode: sim.mode,
    status,
    exitCode,
    score,
    totalRules: evaluations.length,
    passedRules,
    failedRules,
    rules: evaluations,
    summary:
      failedRules === 0
        ? `ASIL-D PASS (${passedRules}/${evaluations.length}): ${a.totalLatencyUs} us QNX loop, +${a.clearanceM} m clearance.`
        : `SAFETY VIOLATION (${failedRules} rule(s) failed): ${evaluations
            .filter((r) => !r.passed)
            .map((r) => r.id)
            .join(', ')}.`,
  };
}

export function formatAuditJson(sim, audit) {
  return JSON.stringify(
    {
      audit,
      activeTelemetry: sim.active,
      qnxVsNonRtComparison: sim.comparison,
      canfdSummary: {
        activeChannel: sim.canfd.voter.activeChannelLabel,
        rejectedCount: sim.canfd.rejectedCount,
        unvalidatedAdmittedCount: sim.canfd.unvalidatedAdmittedCount,
        faultNames: sim.canfd.voter.faultNames,
      },
      ppsSnapshot: sim.ppsSnapshot,
    },
    null,
    2
  );
}

export function formatAuditMarkdown(sim, audit) {
  const lines = [
    `# QNX BrakeWire ADAS — ISO 26262 ASIL-D Safety Audit`,
    '',
    `- **Scenario:** ${sim.scenario.title} (\`${sim.scenario.id}\`)`,
    `- **OS Runtime Mode:** \`${sim.active.schedulerLabel}\``,
    `- **Verdict:** **${audit.status}** (${audit.passedRules}/${audit.totalRules} rules passed, score ${audit.score}/100)`,
    `- **TTC:** ${sim.active.ttcS} s | **End-to-End Latency:** ${sim.active.totalLatencyUs} us | **Stopping Clearance:** ${sim.active.clearanceM >= 0 ? '+' : ''}${sim.active.clearanceM} m`,
    `- **QNX vs Non-RT Delta:** +${sim.comparison.clearanceGainM} m clearance gained, ${sim.comparison.latencyReductionPct}% latency reduction (${sim.comparison.latencySavedMs} ms saved)`,
    '',
    '| Rule | ASIL | Status | Evidence |',
    '|---|---|---|---|',
  ];

  for (const r of audit.rules) {
    lines.push(`| \`${r.id}\` ${r.name} | ${r.asil} | **${r.passed ? 'PASS' : 'FAIL'}** | ${r.measured} |`);
  }

  lines.push('', '## `/pps/adas/aeb` Telemetry Snapshot', '', '```text', sim.ppsSnapshot, '```');
  return lines.join('\n');
}

/**
 * Export valid SARIF 2.1.0 report for GitHub Code Scanning / ISO 26262 CI gates.
 */
export function formatAuditSarif(sim, audit) {
  const sarif = {
    $schema: 'https://json.schemastore.org/sarif-2.1.0.json',
    version: '2.1.0',
    runs: [
      {
        tool: {
          driver: {
            name: 'QNX BrakeWire ADAS Auditor',
            version: audit.version,
            informationUri: 'https://github.com/wenizmohamed/qnx-brakewire-adas',
            rules: ISO_RULES.map((r) => ({
              id: r.id,
              name: r.name,
              shortDescription: { text: r.shortDescription },
              defaultConfiguration: { level: 'error' },
              properties: {
                standard: 'ISO 26262-6',
                asil: r.asil,
                tags: ['automotive', 'qnx-8', 'iso-26262', 'asil-d'],
              },
            })),
          },
        },
        results: audit.rules
          .filter((r) => !r.passed)
          .map((r) => ({
            ruleId: r.id,
            level: 'error',
            message: {
              text: `[${r.asil}] ${r.shortDescription}. Measured: ${r.measured}. Remediation: ${r.remediation}`,
            },
            locations: [
              {
                physicalLocation: {
                  artifactLocation: { uri: r.artifactUri },
                  region: { startLine: r.startLine },
                },
              },
            ],
            properties: {
              scenarioId: sim.scenario.id,
              mode: sim.mode,
              measured: r.measured,
            },
          })),
        invocations: [
          {
            executionSuccessful: true,
            exitCode: audit.exitCode,
            exitCodeDescription: audit.status,
            properties: {
              scenarioId: sim.scenario.id,
              mode: sim.mode,
              passedRules: audit.passedRules,
              failedRules: audit.failedRules,
            },
          },
        ],
      },
    ],
  };

  return JSON.stringify(sarif, null, 2);
}
