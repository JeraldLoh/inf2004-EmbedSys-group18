// Humps recorded in the current run, from START until the end of run.
//
// Buddy 4's telemetry carries only the LAST completed hump plus a 3-bit
// counter (hump_seq) that changes once per hump. Telemetry arrives every
// 200 ms, far faster than the car can finish humps, so each change of the
// counter is one new hump and its measurements are copied into the list.
//
// A run starts when the controller enters AUTO (system mode 2). It ends when
// Buddy 4 reports END OF RUN (car lifted or overturned): the list is then
// frozen, so the lift itself cannot add a hump. Leaving AUTO any other way
// (STOP button, MQTT STOP, a fault) also freezes it. The next START clears it.
const AUTO = 2;

export function createHumpLog() {
 return {phase: 'idle', humps: [], lastSeq: null, lastUptime: null, prevMode: null,
  startedMs: null, endedMs: null, endCause: 0, liftCm: 0};
}

const record = (s, n, ms) => ({n, mm: s.hump_mm, uncertainty_mm: s.uncertainty_mm,
 peak_pitch_deg: s.peak_pitch_deg, length_cm: s.path_cm, valid: !!s.hump_valid, at_ms: ms});

// Returns {log, events}: a new log (the old one is not modified) and log-panel
// messages ({text, type}) for the dashboard event list.
export function updateHumpLog(previous, packet) {
 const log = {...previous, humps: previous.humps}, events = [];
 const s = packet?.buddies?.[4], mode = packet?.buddies?.[2]?.system_mode;
 const ms = packet?.uptime_ms ?? 0;
 if (!s) return {log, events};

 // The board restarted: its hump counter starts again from 0.
 if (log.lastUptime !== null && ((ms - log.lastUptime) >>> 0) >= 0x80000000) {
  log.lastSeq = s.hump_seq;
  if (log.phase === 'running') { log.phase = 'stopped'; log.endedMs = log.lastUptime; }
 }
 log.lastUptime = ms;

 const enteredAuto = mode === AUTO && log.prevMode !== AUTO && log.prevMode !== null;
 if (mode !== undefined) log.prevMode = mode;
 if (enteredAuto || (log.phase === 'idle' && mode === AUTO)) {
  Object.assign(log, {phase: 'running', humps: [], lastSeq: s.hump_seq, startedMs: ms,
   endedMs: null, endCause: 0, liftCm: 0});
  events.push({text: 'Run started: hump counter reset to 0.', type: 'system'});
  return {log, events};
 }
 if (log.phase !== 'running') { log.lastSeq = s.hump_seq; return {log, events}; }

 if (s.run_ended) {
  Object.assign(log, {phase: 'ended', endedMs: ms, endCause: s.run_end_cause, liftCm: s.lift_cm});
  events.push({text: `End of run: ${log.humps.length} hump${log.humps.length === 1 ? '' : 's'} recorded.`, type: 'warning'});
  return {log, events};
 }
 if (s.hump_seq !== log.lastSeq) {
  const h = record(s, log.humps.length + 1, ms);
  log.humps = [...log.humps, h];
  log.lastSeq = s.hump_seq;
  events.push({text: `Hump #${h.n}: ${h.mm} mm ±${h.uncertainty_mm}, peak ${h.peak_pitch_deg}°, ${h.length_cm} cm long${h.valid ? '' : ' (INVALID)'}.`,
   type: h.valid ? 'system' : 'warning'});
 }
 if (mode !== undefined && mode !== AUTO) {
  Object.assign(log, {phase: 'stopped', endedMs: ms});
  events.push({text: `Run stopped: ${log.humps.length} hump${log.humps.length === 1 ? '' : 's'} recorded.`, type: 'system'});
 }
 return {log, events};
}

export function humpSummary(log) {
 const valid = log.humps.filter(h => h.valid).length;
 const state = {idle: 'Waiting for START', running: 'Recording · run in progress',
  ended: log.endCause === 1 ? `Run ended · car lifted ${log.liftCm} cm` : 'Run ended · car overturned',
  stopped: 'Run stopped before end of run'}[log.phase];
 const secs = log.startedMs === null ? null :
  ((((log.endedMs ?? log.lastUptime ?? log.startedMs) - log.startedMs) >>> 0) / 1000);
 return {count: log.humps.length, valid, state, seconds: secs};
}

export function renderHumpLog(root, log) {
 if (!root) return;
 const sum = humpSummary(log);
 root.replaceChildren();
 const head = document.createElement('div'); head.className = 'calibration-heading';
 head.textContent = 'HUMPS THIS RUN';
 const count = document.createElement('div'); count.className = 'hump-count';
 const big = document.createElement('strong'); big.textContent = String(sum.count);
 const detail = document.createElement('span');
 detail.textContent = (sum.count ? `${sum.valid} valid · ` : '') + sum.state +
  (sum.seconds !== null && log.phase !== 'idle' ? ` · ${sum.seconds.toFixed(1)} s` : '');
 count.append(big, detail);
 root.append(head, count);
 if (!log.humps.length) {
  const note = document.createElement('p'); note.className = 'calibration-note';
  note.textContent = log.phase === 'idle' ? 'Humps are counted from START until the car is lifted (end of run).'
   : 'No hump measured yet.';
  root.append(note);
  return;
 }
 const list = document.createElement('ol'); list.className = 'hump-list';
 for (const h of log.humps) {
  const li = document.createElement('li'); li.className = 'calibration-row' + (h.valid ? '' : ' invalid');
  const name = document.createElement('span');
  name.textContent = `#${h.n} · ${h.mm} mm ±${h.uncertainty_mm}`;
  const meta = document.createElement('span'); meta.className = 'calibration-badge';
  meta.textContent = `${h.peak_pitch_deg}° · ${h.length_cm} cm` + (h.valid ? '' : ' · invalid');
  li.append(name, meta);
  list.append(li);
 }
 root.append(list);
}
