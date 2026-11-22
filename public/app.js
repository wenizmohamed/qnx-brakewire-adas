/**
 * QNX BrakeWire ADAS — Interactive Automotive Cockpit & 60 FPS Digital Twin
 * Uses byte-identical deterministic engine modules from ./engine/*.js.
 */

import { SCENARIOS, runAdasSimulation } from './engine/adasEngine.js';
import {
  evaluateIso26262Audit,
  formatAuditJson,
  formatAuditSarif,
} from './engine/iso26262Audit.js';

const TARGET_SNIPPETS = {
  adas: `/* target-qnx8/brakewire_adas.c — QNX Neutrino 8.0 ASIL-D Brake Resource Manager */
#include <sys/neutrino.h>
#include <sys/dispatch.h>
#include <sys/iofunc.h>
#include <sys/slog2.h>
#include <sys/syspage.h>

g_cycles_per_sec = SYSPAGE_ENTRY(qtime)->cycles_per_sec;
name_attach_t *attach = name_attach(NULL, "qnx/brakewire", 0);

/* Lock brake actuator loop to SCHED_FIFO prio 63 on CPU0 (zero preemption) */
configure_rt_thread(BRAKEWIRE_ACTUATOR_PRIO /* 63 */, 0x01U);

iov_t rx_iov[2];
SETIOV(&rx_iov[0], &rx_hdr, sizeof(rx_hdr));
SETIOV(&rx_iov[1], &rx_fusion, sizeof(rx_fusion));

while (g_rt.running) {
    int rcvid = MsgReceivev(attach->chid, rx_iov, 2, NULL);
    uint64_t rx_now = ClockCycles();
    uint32_t ipc_us = cycles_to_us(rx_now - rx_hdr.send_cycles);
    compute_aeb_command(&rx_fusion, rx_hdr.seq_id, ipc_us, rx_now, &tx_reply);
    MsgReplyv(rcvid, EOK, tx_iov, 1); /* Zero-copy reply < 1,000 us deadline */
}`,
  canfd: `/* target-qnx8/canfd_sensor_rx.c — CRC-16/CCITT-FALSE & Plausibility Voter */
static uint32_t validate_canfd_frame(const canfd_frame_t *frame, canfd_channel_state_t *st) {
    uint32_t flags = CANFD_FLAG_VALID;
    uint16_t computed = bw_crc16_ccitt(frame->payload, frame->dlc);
    if (computed != frame->crc16_ccitt) flags |= CANFD_FLAG_BAD_CRC16;
    if (st->initialized && (((frame->rolling_cnt - st->expected_cnt) & 0x0FU) != 0U)) {
        flags |= CANFD_FLAG_COUNTER_JUMP;
    }
    return flags;
}

/* Zero-copy scatter-gather IPC transmission to brakewire_adas */
hdr.type = BW_MSG_SENSOR_FUSION;
hdr.send_cycles = ClockCycles();
MsgSendv(coid, send_iov, 2, recv_iov, 1);`,
  ifs: `# target-qnx8/qnx8-qemu-x86_64.build — Instrumented SMP QNX 8.0 Boot Image
[virtual=x86_64,bios +compress] boot = {
    startup-x86 -v
    PATH=/proc/boot:/bin:/usr/bin procnto-smp-instr -v
}
[+script] startup-script = {
    slogger2 -s 64k -c 8 &
    pps -m /pps &
    /proc/boot/brakewire_adas &
    waitfor /dev/name/local/qnx/brakewire 4
    /proc/boot/canfd_sensor_rx --spoof &
}`,
  cli: `# Zero-dependency ISO 26262 ASIL-D Safety Gate CLI (exit 0 PASS / exit 2 VIOLATION)
node cli/brakewire-cli.mjs --scenario highway-cutin --format json
node cli/brakewire-cli.mjs --scenario priority-storm --mode non-rt --format sarif
node cli/brakewire-cli.mjs --scenario canfd-spoof --format markdown

# QNX SDP 8.0 Cross-Compilation & QEMU SMP Boot
source ~/qnx800/qnxsdp-env.sh
make -C target-qnx8 ifs
./target-qnx8/scripts/run-qemu.sh`,
};

const state = {
  scenarioId: 'highway-cutin',
  mode: 'qnx',
  egoSpeedKmh: 110,
  frictionMu: 0.9,
  spoofInjected: false,
  stormInjected: false,
  codeTab: 'adas',
  playing: true,
  animProgress: 0,
  lastFrameTs: 0,
  sim: null,
  audit: null,
};

const el = {
  scenarioTabs: document.getElementById('scenarioTabs'),
  scenarioEnvBadge: document.getElementById('scenarioEnvBadge'),
  scenarioSubtitle: document.getElementById('scenarioSubtitle'),
  globalVerdictPill: document.getElementById('globalVerdictPill'),
  globalVerdictText: document.getElementById('globalVerdictText'),
  hudOsBadge: document.getElementById('hudOsBadge'),
  hudClockReadout: document.getElementById('hudClockReadout'),
  btnReplaySim: document.getElementById('btnReplaySim'),
  btnTogglePlay: document.getElementById('btnTogglePlay'),
  btnResetScenario: document.getElementById('btnResetScenario'),
  btnModeQnx: document.getElementById('btnModeQnx'),
  btnModeNonRt: document.getElementById('btnModeNonRt'),
  speedSlider: document.getElementById('speedSlider'),
  speedReadout: document.getElementById('speedReadout'),
  frictionGroup: document.getElementById('frictionGroup'),
  btnToggleSpoof: document.getElementById('btnToggleSpoof'),
  btnToggleStorm: document.getElementById('btnToggleStorm'),
  budgetMeasuredUs: document.getElementById('budgetMeasuredUs'),
  budgetStatusTag: document.getElementById('budgetStatusTag'),
  budgetBarFill: document.getElementById('budgetBarFill'),
  stageBreakdownStrip: document.getElementById('stageBreakdownStrip'),
  kpiTtc: document.getElementById('kpiTtc'),
  kpiHeadway: document.getElementById('kpiHeadway'),
  kpiClearanceTile: document.getElementById('kpiClearanceTile'),
  kpiClearance: document.getElementById('kpiClearance'),
  kpiOutcome: document.getElementById('kpiOutcome'),
  kpiDecel: document.getElementById('kpiDecel'),
  kpiPressure: document.getElementById('kpiPressure'),
  kpiYaw: document.getElementById('kpiYaw'),
  kpiPreemptions: document.getElementById('kpiPreemptions'),
  deltaHighlightPill: document.getElementById('deltaHighlightPill'),
  cmpNonRtOutcome: document.getElementById('cmpNonRtOutcome'),
  cmpNonRtLatency: document.getElementById('cmpNonRtLatency'),
  cmpNonRtDelayDist: document.getElementById('cmpNonRtDelayDist'),
  cmpNonRtStopDist: document.getElementById('cmpNonRtStopDist'),
  cmpNonRtImpact: document.getElementById('cmpNonRtImpact'),
  cmpNonRtPct: document.getElementById('cmpNonRtPct'),
  cmpNonRtBar: document.getElementById('cmpNonRtBar'),
  cmpQnxOutcome: document.getElementById('cmpQnxOutcome'),
  cmpQnxLatency: document.getElementById('cmpQnxLatency'),
  cmpQnxDelayDist: document.getElementById('cmpQnxDelayDist'),
  cmpQnxStopDist: document.getElementById('cmpQnxStopDist'),
  cmpQnxMisses: document.getElementById('cmpQnxMisses'),
  cmpQnxPct: document.getElementById('cmpQnxPct'),
  cmpQnxBar: document.getElementById('cmpQnxBar'),
  voterChannelBadge: document.getElementById('voterChannelBadge'),
  voterRadarVal: document.getElementById('voterRadarVal'),
  voterLidarVal: document.getElementById('voterLidarVal'),
  voterFusedVal: document.getElementById('voterFusedVal'),
  voterLatencyVal: document.getElementById('voterLatencyVal'),
  voterReason: document.getElementById('voterReason'),
  canTableBody: document.getElementById('canTableBody'),
  ppsOutput: document.getElementById('ppsOutput'),
  auditScoreBadge: document.getElementById('auditScoreBadge'),
  isoRuleList: document.getElementById('isoRuleList'),
  targetCodePreview: document.getElementById('targetCodePreview'),
  btnExportJson: document.getElementById('btnExportJson'),
  btnExportSarif: document.getElementById('btnExportSarif'),
  btnCopyPps: document.getElementById('btnCopyPps'),
  canvas: document.getElementById('highwayCanvas'),
};

function downloadFile(filename, content, mime = 'application/json') {
  const blob = new Blob([content], { type: mime });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  a.remove();
  URL.revokeObjectURL(url);
}

function selectScenario(id) {
  const sc = SCENARIOS.find((s) => s.id === id) || SCENARIOS[0];
  state.scenarioId = sc.id;
  state.egoSpeedKmh = sc.egoSpeedKmh;
  state.frictionMu = sc.frictionMu;
  state.spoofInjected = sc.spoofInjected;
  state.stormInjected = sc.stormInjected;
  state.animProgress = 0;
  recomputeAndRender();
}

const HUMAN_BRAKE_SCENARIOS = {
  'highway-cutin': {
    shortTitle: 'Highway Cut-In',
    summary: 'Highway Cut-In at 110 km/h: A slower car suddenly cuts into your lane with 1.42 s before impact. QNX locks the brakes in 412 us to stop safely with meters to spare.',
  },
  'pedestrian-night': {
    shortTitle: 'Night Pedestrian',
    summary: 'Night Crosswalk at 50 km/h: A pedestrian steps into an unlit crosswalk. Front radar and LiDAR lock on instantly so QNX stops the car short of the crosswalk.',
  },
  'canfd-spoof': {
    shortTitle: 'Fake Radar Attack',
    summary: 'Spoofed Radar Attack: A tampered radar packet tries to tell the car the road is empty. QNX rejects the fake packet in 18 us and brakes safely using the backup LiDAR.',
  },
  'priority-storm': {
    shortTitle: 'CPU Overload Storm',
    summary: 'Dashboard CPU Overload: Heavy infotainment and software updates spike CPU load to 98%. QNX isolates braking on a dedicated real-time core so the car stops cleanly instead of crashing.',
  },
  'wet-split-mu': {
    shortTitle: 'Wet Uneven Road',
    summary: 'Rain-Slicked Uneven Grip: One side of the road is wetter than the other. The 1,000 Hz QNX anti-lock braking loop keeps the car straight without spinning out.',
  },
};

function buildScenarioTabs() {
  el.scenarioTabs.innerHTML = '';
  SCENARIOS.forEach((sc, idx) => {
    const h = HUMAN_BRAKE_SCENARIOS[sc.id];
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'scenario-tab';
    btn.dataset.scenarioId = sc.id;
    btn.setAttribute('role', 'tab');
    const gripLabel = sc.frictionMu >= 0.8 ? 'Dry' : sc.frictionMu >= 0.45 ? 'Wet' : 'Ice';
    btn.innerHTML = `
      <span class="scenario-tab-num">0${idx + 1} / ${sc.obstacleType.toUpperCase()}</span>
      <span class="scenario-tab-name">${h ? h.shortTitle : sc.shortTitle}</span>
      <span class="scenario-tab-meta">${sc.egoSpeedKmh} km/h &bull; ${gripLabel}</span>
    `;
    btn.addEventListener('click', () => selectScenario(sc.id));
    el.scenarioTabs.appendChild(btn);
  });
}

function recomputeAndRender() {
  const sim = runAdasSimulation(state.scenarioId, {
    mode: state.mode,
    egoSpeedKmh: state.egoSpeedKmh,
    frictionMu: state.frictionMu,
    spoofInjected: state.spoofInjected,
    stormInjected: state.stormInjected,
  });
  const audit = evaluateIso26262Audit(sim);
  state.sim = sim;
  state.audit = audit;

  renderHeaderAndTabs();
  renderControls();
  renderBudgetBar();
  renderKpis();
  renderDeterminismCompare();
  renderCanFdAndVoter();
  renderPpsAndAudit();
  renderCodePreview();
}

function renderHeaderAndTabs() {
  const { sim, audit } = state;
  const h = HUMAN_BRAKE_SCENARIOS[sim.scenario.id];
  el.scenarioEnvBadge.textContent = sim.scenario.environment;
  el.scenarioSubtitle.textContent = h ? h.summary : `${sim.scenario.title} - ${sim.scenario.subtitle}`;

  document.querySelectorAll('.scenario-tab').forEach((btn) => {
    const active = btn.dataset.scenarioId === state.scenarioId;
    btn.classList.toggle('active', active);
    btn.setAttribute('aria-selected', String(active));
  });

  const passed = audit.status === 'PASS';
  el.globalVerdictPill.className = `verdict-pill ${passed ? 'verdict-pass' : 'verdict-fail'}`;
  el.globalVerdictText.textContent = passed
    ? `ALL SAFETY CHECKS PASS (${audit.passedRules}/${audit.totalRules})`
    : `SAFETY ALERT (${audit.failedRules} FAIL)`;

  el.hudOsBadge.textContent = state.mode === 'qnx' ? 'QNX Real-Time Braking' : 'Standard OS (Delayed)';
  el.hudOsBadge.className = `hud-badge ${state.mode === 'qnx' ? 'hud-badge-qnx' : 'hud-badge-nonrt'}`;
}

function renderControls() {
  el.btnModeQnx.classList.toggle('active', state.mode === 'qnx');
  el.btnModeNonRt.classList.toggle('active', state.mode === 'non-rt');

  el.speedSlider.value = String(state.egoSpeedKmh);
  el.speedReadout.textContent = `${state.egoSpeedKmh} km/h (${(state.egoSpeedKmh / 3.6).toFixed(1)} m/s)`;

  el.frictionGroup.querySelectorAll('.friction-btn').forEach((btn) => {
    const mu = Number(btn.dataset.mu);
    const active = Math.abs(mu - state.frictionMu) < 0.08;
    btn.classList.toggle('active', active);
  });

  el.btnToggleSpoof.classList.toggle('active', state.spoofInjected);
  el.btnToggleSpoof.setAttribute('aria-pressed', String(state.spoofInjected));
  el.btnToggleStorm.classList.toggle('active', state.stormInjected);
  el.btnToggleStorm.setAttribute('aria-pressed', String(state.stormInjected));
}

function renderBudgetBar() {
  const a = state.sim.active;
  const b = a.stageBreakdownUs;
  const overrun = a.totalLatencyUs > a.deadlineBudgetUs;

  el.budgetMeasuredUs.textContent = overrun
    ? `${(a.totalLatencyUs / 1000).toFixed(1)} ms (${a.totalLatencyUs.toLocaleString()} µs)`
    : `${a.totalLatencyUs} µs`;

  el.budgetStatusTag.textContent = overrun ? `LATE (${a.deadlineMisses} MISSED)` : '0 MISSED • ON TIME';
  el.budgetStatusTag.className = `mini-pill ${overrun ? 'mini-fail' : 'mini-pass'}`;

  const pct = Math.min(100, Math.max(4, (a.totalLatencyUs / a.deadlineBudgetUs) * 100));
  el.budgetBarFill.style.width = `${pct}%`;
  el.budgetBarFill.classList.toggle('overrun', overrun);

  const fmt = (us) => (us >= 2000 ? `${(us / 1000).toFixed(1)} ms` : `${us} µs`);
  const stages = [
    ['Sensor Read', fmt(b.canfdRxUs)],
    ['Tamper Check', fmt(b.crcVoteUs)],
    ['QNX Message', fmt(b.msgSendvUs)],
    ['Brake Decision', fmt(b.fusionLawUs)],
    ['Brake Caliper', fmt(b.actuatorCmdUs)],
  ];
  el.stageBreakdownStrip.innerHTML = stages
    .map(([label, val]) => `<div class="stage-chip"><span>${label}</span><strong>${val}</strong></div>`)
    .join('');
}

function renderKpis() {
  const a = state.sim.active;
  el.kpiTtc.textContent = `${a.ttcS.toFixed(2)} s`;
  el.kpiHeadway.textContent = `Starting gap ${a.initialDistanceM.toFixed(2)} m`;

  el.kpiClearance.textContent = `${a.clearanceM >= 0 ? '+' : ''}${a.clearanceM.toFixed(2)} m`;
  el.kpiOutcome.textContent = a.collision ? `CRASH @ ${a.impactSpeedKmh} km/h` : 'SAFE STOP';
  el.kpiClearanceTile.className = `kpi-tile ${a.collision ? 'tile-danger' : 'tile-safe'}`;

  el.kpiDecel.textContent = `${a.maxDecelMps2.toFixed(2)} m/s²`;
  el.kpiPressure.textContent = `${a.hydraulicBar.toFixed(1)} bar pressure`;

  el.kpiYaw.textContent = `${a.yawRateDegS.toFixed(2)} °/s`;
  el.kpiPreemptions.textContent = `${a.preemptionEvents} OS delays @ ${a.cpuLoadPct}% CPU`;
}

function renderDeterminismCompare() {
  const { qnx, nonRt, comparison, config } = state.sim;
  el.deltaHighlightPill.textContent = `+${comparison.clearanceGainM.toFixed(2)} m margin saved • ${comparison.latencyReductionPct.toFixed(2)}% faster braking`;

  el.cmpNonRtOutcome.textContent = nonRt.collision
    ? `COLLISION (${nonRt.clearanceM.toFixed(2)} m)`
    : `STOP (+${nonRt.clearanceM.toFixed(2)} m)`;
  el.cmpNonRtLatency.textContent = `${nonRt.totalLatencyMs.toFixed(1)} ms`;
  el.cmpNonRtDelayDist.textContent = `${nonRt.delayDistanceM.toFixed(2)} m`;
  el.cmpNonRtStopDist.textContent = `${nonRt.totalStoppingDistanceM.toFixed(2)} m`;
  el.cmpNonRtImpact.textContent = nonRt.collision ? `${nonRt.impactSpeedKmh} km/h` : '0 km/h';

  const nonRtPct = Math.round((nonRt.totalStoppingDistanceM / config.initialDistanceM) * 100);
  el.cmpNonRtPct.textContent = `${nonRtPct}% of ${config.initialDistanceM.toFixed(1)} m gap`;
  el.cmpNonRtBar.style.width = `${Math.min(100, nonRtPct)}%`;

  el.cmpQnxOutcome.textContent = qnx.collision
    ? `COLLISION (${qnx.clearanceM.toFixed(2)} m)`
    : `SAFE STOP (+${qnx.clearanceM.toFixed(2)} m)`;
  el.cmpQnxOutcome.className = `mono ${qnx.collision ? 'text-crimson' : 'text-emerald'}`;
  el.cmpQnxLatency.textContent = `${qnx.totalLatencyUs} µs`;
  el.cmpQnxDelayDist.textContent = `${qnx.delayDistanceM.toFixed(2)} m`;
  el.cmpQnxStopDist.textContent = `${qnx.totalStoppingDistanceM.toFixed(2)} m`;
  el.cmpQnxMisses.textContent = `${qnx.deadlineMisses} / 1 kHz`;

  const qnxPct = Math.round((qnx.totalStoppingDistanceM / config.initialDistanceM) * 100);
  el.cmpQnxPct.textContent = `${qnxPct}% of ${config.initialDistanceM.toFixed(1)} m gap`;
  el.cmpQnxBar.style.width = `${Math.min(100, qnxPct)}%`;
}

function renderCanFdAndVoter() {
  const { frames, voter } = state.sim.canfd;
  el.voterChannelBadge.textContent = voter.activeChannel === 0 ? 'Radar + LiDAR Verified' : voter.activeChannel === 1 ? 'Backup LiDAR Active' : 'Hydraulic Backup';
  el.voterChannelBadge.className = `mini-pill ${voter.activeChannel === 0 ? 'mini-pass' : voter.activeChannel === 1 ? 'mini-warn' : 'mini-fail'}`;

  el.voterRadarVal.textContent = `${voter.radarRangeM.toFixed(2)} m${voter.radarValid ? '' : ' (FAULT)'}`;
  el.voterLidarVal.textContent = `${voter.lidarRangeM.toFixed(2)} m${voter.lidarValid ? '' : ' (FAULT)'}`;
  el.voterFusedVal.textContent = `${voter.fusedRangeM.toFixed(2)} m`;
  el.voterLatencyVal.textContent = `${voter.voterLatencyUs} µs`;
  el.voterReason.textContent = voter.fallbackReason;

  el.canTableBody.innerHTML = frames
    .map((f) => {
      const badgeClass = f.valid ? 'mini-pass' : state.mode === 'qnx' ? 'mini-warn' : 'mini-fail';
      const crcText = f.valid
        ? `${f.crc16ReceivedHex} OK`
        : `${f.crc16ReceivedHex} != ${f.crc16ExpectedHex}`;
      return `
        <tr class="${f.valid ? '' : 'row-spoof'}">
          <td class="mono"><strong>${f.canIdHex}</strong></td>
          <td class="mono">${f.channel}</td>
          <td class="mono">${f.rollingCounter}</td>
          <td class="hex-cell">${f.hexPayload.slice(0, 23)}…</td>
          <td class="mono">${crcText}</td>
          <td><span class="mini-pill ${badgeClass}">${f.action}</span></td>
        </tr>
      `;
    })
    .join('');
}

function renderPpsAndAudit() {
  const { sim, audit } = state;
  el.ppsOutput.textContent = sim.ppsSnapshot;
  el.auditScoreBadge.textContent = `${audit.score} / 100 (${audit.status})`;
  el.auditScoreBadge.style.color = audit.status === 'PASS' ? '#047857' : '#b91c1c';

  el.isoRuleList.innerHTML = audit.rules
    .map(
      (r) => `
      <div class="iso-rule-item ${r.passed ? '' : 'rule-fail'}">
        <div class="rule-left">
          <strong>${r.id} • ${r.asil} — ${r.name}</strong>
          <span>${r.measured}</span>
        </div>
        <span class="mini-pill ${r.passed ? 'mini-pass' : 'mini-fail'}">${r.passed ? 'PASS' : 'FAIL'}</span>
      </div>
    `
    )
    .join('');
}

function renderCodePreview() {
  el.targetCodePreview.textContent = TARGET_SNIPPETS[state.codeTab] || TARGET_SNIPPETS.adas;
  document.querySelectorAll('.code-tab').forEach((btn) => {
    btn.classList.toggle('active', btn.dataset.code === state.codeTab);
  });
}

/* --------------------------------------------------------------------------
 * 60 FPS Interactive Carbon-Obsidian Highway AEB & LiDAR/Radar Canvas Engine
 * -------------------------------------------------------------------------- */
function drawHighwayFrame(ts) {
  const ctx = el.canvas.getContext('2d');
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const rect = el.canvas.getBoundingClientRect();
  const w = Math.max(320, Math.round(rect.width));
  const h = Math.max(260, Math.round(rect.height));

  if (el.canvas.width !== Math.round(w * dpr) || el.canvas.height !== Math.round(h * dpr)) {
    el.canvas.width = Math.round(w * dpr);
    el.canvas.height = Math.round(h * dpr);
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

  if (state.lastFrameTs === 0) state.lastFrameTs = ts;
  const dt = Math.min(0.05, (ts - state.lastFrameTs) / 1000);
  state.lastFrameTs = ts;

  const sim = state.sim;
  const active = sim.active;
  const cycleDuration = Math.max(3.6, active.totalStopTimeS + 1.4);

  if (state.playing) {
    state.animProgress = (state.animProgress + dt / cycleDuration) % 1;
  }

  const simTime = state.animProgress * (active.totalStopTimeS + 0.8);
  el.hudClockReadout.textContent = `t = ${simTime.toFixed(2)} s`;

  // Interpolate kinematic state at simTime
  const traj = sim.trajectory;
  let pt = traj[traj.length - 1];
  for (let i = 0; i < traj.length - 1; i++) {
    if (simTime >= traj[i].t && simTime <= traj[i + 1].t) {
      const f = (simTime - traj[i].t) / Math.max(0.0001, traj[i + 1].t - traj[i].t);
      pt = {
        t: simTime,
        egoSpeedKmh: traj[i].egoSpeedKmh + (traj[i + 1].egoSpeedKmh - traj[i].egoSpeedKmh) * f,
        gapM: traj[i].gapM + (traj[i + 1].gapM - traj[i].gapM) * f,
        decelMps2: traj[i].decelMps2 + (traj[i + 1].decelMps2 - traj[i].decelMps2) * f,
        collided: traj[i].gapM < 0 || traj[i + 1].gapM < 0,
      };
      break;
    }
  }

  // Background asphalt & weather lighting
  const isNight = sim.scenario.id === 'pedestrian-night';
  const isWet = state.frictionMu <= 0.6 && state.frictionMu > 0.35;
  const isIce = state.frictionMu <= 0.35;

  ctx.fillStyle = isNight ? '#050810' : '#0b0f17';
  ctx.fillRect(0, 0, w, h);

  // Subtle engineering grid backdrop
  ctx.strokeStyle = 'rgba(148, 163, 184, 0.06)';
  ctx.lineWidth = 1;
  for (let gx = 0; gx < w; gx += 40) {
    ctx.beginPath();
    ctx.moveTo(gx, 0);
    ctx.lineTo(gx, h);
    ctx.stroke();
  }

  // 3-Lane Highway Deck
  const roadTop = Math.round(h * 0.22);
  const roadH = Math.round(h * 0.56);
  const roadBot = roadTop + roadH;
  const laneH = roadH / 3;

  ctx.fillStyle = isIce ? '#132033' : isWet ? '#111826' : '#141b28';
  ctx.fillRect(0, roadTop, w, roadH);

  // Split-mu visual highlight on lower half of center lane when wet/ice
  if (isWet || isIce) {
    ctx.fillStyle = isIce ? 'rgba(56, 189, 248, 0.14)' : 'rgba(59, 130, 246, 0.10)';
    ctx.fillRect(0, roadTop + laneH * 1.5, w, laneH * 0.5);
  }

  // Road shoulders
  ctx.strokeStyle = '#334155';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(0, roadTop);
  ctx.lineTo(w, roadTop);
  ctx.moveTo(0, roadBot);
  ctx.lineTo(w, roadBot);
  ctx.stroke();

  // Animated dashed lane markings
  const scrollPx = ((ts * 0.04 * Math.max(10, pt.egoSpeedKmh)) / 100) % 56;
  ctx.strokeStyle = 'rgba(226, 232, 240, 0.26)';
  ctx.lineWidth = 2;
  ctx.setLineDash([26, 30]);
  ctx.lineDashOffset = scrollPx;
  for (let l = 1; l <= 2; l++) {
    const ly = roadTop + laneH * l;
    ctx.beginPath();
    ctx.moveTo(0, ly);
    ctx.lineTo(w, ly);
    ctx.stroke();
  }
  ctx.setLineDash([]);

  // Spatial mapping: ego car sits on left, obstacle sits ahead proportional to initial distance
  const egoX = Math.round(w * 0.17);
  const centerY = Math.round(roadTop + laneH * 1.5);
  const usableSpanPx = Math.round(w * 0.66);
  const pxPerMeter = usableSpanPx / Math.max(20, sim.config.initialDistanceM + 6);

  const egoLen = 74;
  const egoW = 34;
  const egoNoseX = egoX + egoLen / 2;

  const clampedGapM = Math.max(-3.5, pt.gapM);
  const obstacleX = Math.max(egoNoseX + 10, egoNoseX + clampedGapM * pxPerMeter);

  // Night Crosswalk Zebra Stripes for Pedestrian Scenario
  if (sim.scenario.obstacleType === 'pedestrian') {
    const cwX = obstacleX - 18;
    ctx.fillStyle = 'rgba(248, 250, 252, 0.18)';
    for (let sy = roadTop + 8; sy < roadBot - 12; sy += 22) {
      ctx.fillRect(cwX, sy, 36, 11);
    }
  }

  // Stopping Distance Envelope Overlay (Delay Zone + Braking Zone)
  const delayPx = active.delayDistanceM * pxPerMeter;
  const brakePx = active.brakingDistanceM * pxPerMeter;
  const envTop = centerY - 22;
  const envH = 44;

  // Delay/Reaction travel zone (amber)
  ctx.fillStyle = 'rgba(245, 158, 11, 0.20)';
  ctx.fillRect(egoNoseX, envTop, Math.min(w - egoNoseX - 12, delayPx), envH);

  // Braking zone (emerald if safe, crimson if collision)
  ctx.fillStyle = active.collision ? 'rgba(239, 68, 68, 0.22)' : 'rgba(16, 185, 129, 0.20)';
  const brakeStartX = egoNoseX + delayPx;
  ctx.fillRect(brakeStartX, envTop, Math.min(w - brakeStartX - 12, brakePx), envH);

  // Stop marker line
  const stopX = Math.min(w - 16, egoNoseX + active.totalStoppingDistanceM * pxPerMeter);
  ctx.strokeStyle = active.collision ? '#ef4444' : '#10b981';
  ctx.lineWidth = 2;
  ctx.setLineDash([5, 4]);
  ctx.beginPath();
  ctx.moveTo(stopX, roadTop + 6);
  ctx.lineTo(stopX, roadBot - 6);
  ctx.stroke();
  ctx.setLineDash([]);

  // 120-deg LiDAR Fan + 77 GHz Radar Narrow Beam
  const coneLen = Math.min(w - egoNoseX - 18, Math.max(120, obstacleX - egoNoseX + 42));
  const lidarGrad = ctx.createLinearGradient(egoNoseX, centerY, egoNoseX + coneLen, centerY);
  lidarGrad.addColorStop(0, 'rgba(245, 158, 11, 0.24)');
  lidarGrad.addColorStop(1, 'rgba(245, 158, 11, 0.02)');
  ctx.fillStyle = lidarGrad;
  ctx.beginPath();
  ctx.moveTo(egoNoseX, centerY);
  ctx.lineTo(egoNoseX + coneLen, centerY - 68);
  ctx.lineTo(egoNoseX + coneLen, centerY + 68);
  ctx.closePath();
  ctx.fill();

  // Narrow 77 GHz Radar Lock Beam
  const radarSpoofed = state.spoofInjected;
  ctx.fillStyle = radarSpoofed ? 'rgba(239, 68, 68, 0.20)' : 'rgba(16, 185, 129, 0.18)';
  ctx.beginPath();
  ctx.moveTo(egoNoseX, centerY);
  ctx.lineTo(obstacleX, centerY - 14);
  ctx.lineTo(obstacleX, centerY + 14);
  ctx.closePath();
  ctx.fill();

  // Animated LiDAR scan arcs
  const pulse = (ts % 1400) / 1400;
  ctx.strokeStyle = 'rgba(251, 191, 36, 0.35)';
  ctx.lineWidth = 1.25;
  ctx.beginPath();
  ctx.arc(egoNoseX, centerY, pulse * coneLen, -0.34, 0.34);
  ctx.stroke();

  // Draw Ego Vehicle (Precision Automotive Top-Down Silhouette)
  const carLeft = egoX - egoLen / 2;
  const carTop = centerY - egoW / 2;
  const brakingNow = pt.decelMps2 > 0.5;

  // Tail-light crimson glow when AEB active
  if (brakingNow) {
    const glow = ctx.createRadialGradient(carLeft, centerY, 2, carLeft, centerY, 28);
    glow.addColorStop(0, 'rgba(239, 68, 68, 0.85)');
    glow.addColorStop(1, 'rgba(239, 68, 68, 0)');
    ctx.fillStyle = glow;
    ctx.beginPath();
    ctx.arc(carLeft, centerY, 28, 0, Math.PI * 2);
    ctx.fill();
  }

  // Vehicle body
  ctx.fillStyle = '#e2e8f0';
  ctx.beginPath();
  ctx.roundRect(carLeft, carTop, egoLen, egoW, 8);
  ctx.fill();
  ctx.strokeStyle = '#94a3b8';
  ctx.lineWidth = 1.5;
  ctx.stroke();

  // Cabin glass canopy
  ctx.fillStyle = '#0f172a';
  ctx.beginPath();
  ctx.roundRect(carLeft + 18, carTop + 5, 34, egoW - 10, 5);
  ctx.fill();

  // Brake light bars
  ctx.fillStyle = brakingNow ? '#ff2a2a' : '#7f1d1d';
  ctx.fillRect(carLeft, carTop + 4, 4, 8);
  ctx.fillRect(carLeft, carTop + egoW - 12, 4, 8);

  // Headlights
  ctx.fillStyle = '#fef08a';
  ctx.fillRect(carLeft + egoLen - 3, carTop + 4, 3, 7);
  ctx.fillRect(carLeft + egoLen - 3, carTop + egoW - 11, 3, 7);

  // Ego telemetry tag
  ctx.fillStyle = '#f8fafc';
  ctx.font = '600 11px "JetBrains Mono", monospace';
  ctx.fillText(`EGO ${pt.egoSpeedKmh.toFixed(0)} km/h`, carLeft, carTop - 9);

  // Draw Target Obstacle (Vehicle, Pedestrian, or Barrier)
  const obsType = sim.scenario.obstacleType;
  const obsY =
    sim.scenario.id === 'highway-cutin' && state.animProgress < 0.22
      ? centerY - (1 - state.animProgress / 0.22) * (laneH * 0.85)
      : centerY;

  if (obsType === 'pedestrian') {
    ctx.fillStyle = '#f59e0b';
    ctx.beginPath();
    ctx.arc(obstacleX + 10, obsY, 9, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = '#fef3c7';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.fillStyle = '#fde68a';
    ctx.font = '600 11px "JetBrains Mono", monospace';
    ctx.fillText('PEDESTRIAN LOCK', obstacleX - 24, obsY - 16);
  } else if (obsType === 'barrier') {
    ctx.fillStyle = '#ef4444';
    ctx.fillRect(obstacleX, centerY - 26, 14, 52);
    ctx.fillStyle = '#f8fafc';
    for (let by = centerY - 22; by < centerY + 22; by += 14) {
      ctx.fillRect(obstacleX + 2, by, 10, 6);
    }
    ctx.fillStyle = '#fca5a5';
    ctx.font = '600 11px "JetBrains Mono", monospace';
    ctx.fillText('SPLIT-MU BARRIER', obstacleX - 28, centerY - 34);
  } else {
    ctx.fillStyle = '#475569';
    ctx.beginPath();
    ctx.roundRect(obstacleX, obsY - 16, 66, 32, 7);
    ctx.fill();
    ctx.strokeStyle = '#fbbf24';
    ctx.lineWidth = 1.5;
    ctx.stroke();
    ctx.fillStyle = '#0f172a';
    ctx.beginPath();
    ctx.roundRect(obstacleX + 15, obsY - 11, 30, 22, 4);
    ctx.fill();
    ctx.fillStyle = '#fbbf24';
    ctx.font = '600 11px "JetBrains Mono", monospace';
    ctx.fillText(`TARGET ${sim.config.targetSpeedKmh} km/h`, obstacleX - 4, obsY - 22);
  }

  // LiDAR Point-Cloud Lock Dots on Target Leading Edge
  ctx.fillStyle = '#fbbf24';
  for (let i = -3; i <= 3; i++) {
    ctx.beginPath();
    ctx.arc(obstacleX - 2, obsY + i * 4.2, 1.8, 0, Math.PI * 2);
    ctx.fill();
  }

  // Clearance / Impact Dimension Bracket along bottom of road
  const bracketY = roadBot + 24;
  ctx.strokeStyle = active.collision ? '#ef4444' : '#10b981';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(egoNoseX, bracketY);
  ctx.lineTo(obstacleX, bracketY);
  ctx.moveTo(egoNoseX, bracketY - 5);
  ctx.lineTo(egoNoseX, bracketY + 5);
  ctx.moveTo(obstacleX, bracketY - 5);
  ctx.lineTo(obstacleX, bracketY + 5);
  ctx.stroke();

  const liveGapLabel = pt.collided
    ? `IMPACT! (${active.clearanceM.toFixed(2)} m • ${active.impactSpeedKmh} km/h)`
    : `LIVE GAP: ${Math.max(0, pt.gapM).toFixed(2)} m  |  FINAL CLEARANCE: ${active.clearanceM >= 0 ? '+' : ''}${active.clearanceM.toFixed(2)} m`;
  ctx.fillStyle = active.collision ? '#f87171' : '#34d399';
  ctx.font = '700 11.5px "JetBrains Mono", monospace';
  ctx.fillText(liveGapLabel, egoNoseX, bracketY + 18);

  // Top-left & Top-right Engineering HUD Strips inside Canvas
  ctx.fillStyle = '#94a3b8';
  ctx.font = '600 11px "JetBrains Mono", monospace';
  ctx.fillText(
    `TTC ${active.ttcS.toFixed(2)}s  •  DECEL ${pt.decelMps2.toFixed(1)} m/s²  •  MU ${state.frictionMu.toFixed(2)}  •  LIDAR ${sim.scenario.lidarPoints} PTS`,
    16,
    24
  );

  // Fault Injection Status Banner inside Canvas (Non-overlapping top-right)
  if (state.spoofInjected || state.stormInjected) {
    const msg = state.spoofInjected
      ? state.mode === 'qnx'
        ? 'CAN-FD SPOOF 0x120 REJECTED (18 µs) → LIDAR CH-B ACTIVE'
        : 'CAN-FD SPOOF 0x120 ACCEPTED → BRAKING DELAYED'
      : state.mode === 'qnx'
        ? 'IVI CPU STORM 98% → SCHED_FIFO 63 ISOLATED (0 PREEMPTS)'
        : 'IVI CPU STORM 98% → CFS PRIORITY INVERSION (+685 ms)';
    ctx.font = '700 10.5px "JetBrains Mono", monospace';
    const tw = ctx.measureText(msg).width + 18;
    const bx = Math.max(14, w - tw - 14);
    ctx.fillStyle = state.mode === 'qnx' ? 'rgba(16, 185, 129, 0.16)' : 'rgba(239, 68, 68, 0.22)';
    ctx.strokeStyle = state.mode === 'qnx' ? '#10b981' : '#ef4444';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.roundRect(bx, 10, tw, 22, 6);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = state.mode === 'qnx' ? '#6ee7b7' : '#fca5a5';
    ctx.fillText(msg, bx + 9, 25);
  }

  // Impact Flash Ring if Collision Occurred
  if (pt.collided) {
    ctx.strokeStyle = 'rgba(239, 68, 68, 0.9)';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(obstacleX, obsY, 22 + (Math.sin(ts * 0.02) + 1) * 5, 0, Math.PI * 2);
    ctx.stroke();
  }

  requestAnimationFrame(drawHighwayFrame);
}

function wireEvents() {
  el.btnModeQnx.addEventListener('click', () => {
    state.mode = 'qnx';
    state.animProgress = 0;
    recomputeAndRender();
  });

  el.btnModeNonRt.addEventListener('click', () => {
    state.mode = 'non-rt';
    state.animProgress = 0;
    recomputeAndRender();
  });

  el.speedSlider.addEventListener('input', (e) => {
    state.egoSpeedKmh = Number(e.target.value);
    recomputeAndRender();
  });

  el.frictionGroup.querySelectorAll('.friction-btn').forEach((btn) => {
    btn.addEventListener('click', () => {
      state.frictionMu = Number(btn.dataset.mu);
      recomputeAndRender();
    });
  });

  el.btnToggleSpoof.addEventListener('click', () => {
    state.spoofInjected = !state.spoofInjected;
    recomputeAndRender();
  });

  el.btnToggleStorm.addEventListener('click', () => {
    state.stormInjected = !state.stormInjected;
    recomputeAndRender();
  });

  el.btnResetScenario.addEventListener('click', () => selectScenario(state.scenarioId));

  el.btnReplaySim.addEventListener('click', () => {
    state.animProgress = 0;
    state.playing = true;
    el.btnTogglePlay.textContent = 'Pause';
  });

  el.btnTogglePlay.addEventListener('click', () => {
    state.playing = !state.playing;
    el.btnTogglePlay.textContent = state.playing ? 'Pause' : 'Resume';
  });

  el.btnExportJson.addEventListener('click', () => {
    const content = formatAuditJson(state.sim, state.audit);
    downloadFile(`qnx-brakewire-${state.scenarioId}-${state.mode}.json`, content, 'application/json');
  });

  el.btnExportSarif.addEventListener('click', () => {
    const content = formatAuditSarif(state.sim, state.audit);
    downloadFile(`qnx-brakewire-${state.scenarioId}-${state.mode}.sarif`, content, 'application/sarif+json');
  });

  el.btnCopyPps.addEventListener('click', async () => {
    try {
      await navigator.clipboard.writeText(state.sim.ppsSnapshot);
      el.btnCopyPps.textContent = 'Copied!';
      setTimeout(() => {
        el.btnCopyPps.textContent = 'Copy PPS';
      }, 1500);
    } catch {
      el.btnCopyPps.textContent = 'Ready';
    }
  });

  document.querySelectorAll('.code-tab').forEach((btn) => {
    btn.addEventListener('click', () => {
      state.codeTab = btn.dataset.code;
      renderCodePreview();
    });
  });
}

// Initialize Cockpit
buildScenarioTabs();
wireEvents();
selectScenario('highway-cutin');
requestAnimationFrame(drawHighwayFrame);
