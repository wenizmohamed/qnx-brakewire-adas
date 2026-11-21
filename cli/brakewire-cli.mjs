#!/usr/bin/env node
/**
 * QNX BrakeWire ADAS - Zero-Dependency ISO 26262 ASIL-D Safety Gate CLI
 *
 * Examples:
 *   node cli/brakewire-cli.mjs --scenario highway-cutin --format json
 *   node cli/brakewire-cli.mjs --scenario priority-storm --mode non-rt --format sarif
 *   node cli/brakewire-cli.mjs --scenario canfd-spoof --format markdown
 *   node cli/brakewire-cli.mjs --list
 *
 * Exit codes:
 *   0 = ASIL-D PASS (all AEB001..AEB006 rules pass)
 *   2 = SAFETY VIOLATION (one or more ISO 26262 rules failed)
 */

import { SCENARIOS, runAdasSimulation } from '../src/adasEngine.js';
import {
  evaluateIso26262Audit,
  formatAuditJson,
  formatAuditMarkdown,
  formatAuditSarif,
} from '../src/iso26262Audit.js';

function parseArgs(argv) {
  const opts = {
    scenario: 'highway-cutin',
    mode: 'qnx',
    format: 'markdown',
    list: false,
    help: false,
  };

  for (let i = 0; i < argv.length; i++) {
    const arg = argv[i];
    if (arg === '--list') opts.list = true;
    else if (arg === '--help' || arg === '-h') opts.help = true;
    else if (arg === '--scenario' && argv[i + 1]) opts.scenario = argv[++i];
    else if (arg === '--mode' && argv[i + 1]) opts.mode = argv[++i];
    else if (arg === '--format' && argv[i + 1]) opts.format = argv[++i];
    else if (arg === '--speed' && argv[i + 1]) opts.egoSpeedKmh = Number(argv[++i]);
    else if (arg === '--mu' && argv[i + 1]) opts.frictionMu = Number(argv[++i]);
    else if (arg === '--spoof') opts.spoofInjected = true;
    else if (arg === '--storm') opts.stormInjected = true;
  }
  return opts;
}

const opts = parseArgs(process.argv.slice(2));

if (opts.help) {
  console.log(`QNX BrakeWire ADAS CLI v1.0.0 — ISO 26262 ASIL-D Safety Gate

Usage:
  brakewire-cli [--scenario <id>] [--mode qnx|non-rt] [--format json|markdown|sarif]
                [--speed <km/h>] [--mu <0.25..0.95>] [--spoof] [--storm] [--list]

Scenarios:
  ${SCENARIOS.map((s) => `${s.id.padEnd(18)} ${s.title}`).join('\n  ')}

Exit codes:
  0  ASIL-D PASS
  2  SAFETY VIOLATION
`);
  process.exit(0);
}

if (opts.list) {
  console.log(JSON.stringify(SCENARIOS.map(({ id, title, egoSpeedKmh, frictionMu }) => ({ id, title, egoSpeedKmh, frictionMu })), null, 2));
  process.exit(0);
}

const overrides = { mode: opts.mode };
if (Number.isFinite(opts.egoSpeedKmh)) overrides.egoSpeedKmh = opts.egoSpeedKmh;
if (Number.isFinite(opts.frictionMu)) overrides.frictionMu = opts.frictionMu;
if (opts.spoofInjected !== undefined) overrides.spoofInjected = opts.spoofInjected;
if (opts.stormInjected !== undefined) overrides.stormInjected = opts.stormInjected;

const sim = runAdasSimulation(opts.scenario, overrides);
const audit = evaluateIso26262Audit(sim);

if (opts.format === 'json') {
  console.log(formatAuditJson(sim, audit));
} else if (opts.format === 'sarif') {
  console.log(formatAuditSarif(sim, audit));
} else {
  console.log(formatAuditMarkdown(sim, audit));
}

process.exit(audit.exitCode);
