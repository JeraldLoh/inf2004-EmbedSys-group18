import {barcodeView} from './barcode-status.mjs';
import {sectionAgeMs} from './binary.mjs';
import {bypassProgress, lineHeadline, liveGroup} from './live-status.mjs';
import {obstacleOutputs} from './obstacle-outputs.mjs';
import {stopNames} from './robot-events.mjs';
import {avoidanceNames, motionSpeed, rangeNames, systemNames, terrainNames} from './telemetry-view.mjs';

export function roverOverview(packet, {fresh, elapsed, mapRevision}) {
 const buddies = packet?.buddies ?? {};
 const cards = {};
 const readings = {};
 for (const id of [2, 3, 4, 5]) {
  const sensor = buddies[id];
  const age = sensor ? sectionAgeMs(packet, sensor, elapsed) : null;
  const freshness = !sensor ? 'Waiting' : !fresh || age === null || age >= 1500 ? 'Stale' : packet.demo ? 'Sample' : 'Live';
  cards[id] = {freshness, primary: 'Unavailable', secondary: freshness === 'Stale' ? 'Waiting for fresh readings' : 'Waiting for readings'};
  readings[id] = freshness === 'Live' || freshness === 'Sample' ? sensor : null;
 }

 const motion = readings[2];
 const line = readings[3];
 const terrain = readings[4];
 const obstacle = readings[5];
 const halted = !!motion && ([5, 6].includes(motion.system_mode) || !!motion.stop_reasons);
 let run = motion ? systemNames[motion.system_mode] ?? 'Unknown mode' : 'Waiting for rover data';
 if (motion?.system_mode === 2) run = motion.motion_active ? 'Driving' : 'Auto · waiting for motion';
 if (halted) run = motion.stop_reasons ? `Stopped · ${stopNames(motion.stop_reasons)}` : 'Stopped';
 if (halted && motion.stop_reasons === 1 && terrain?.data_valid && !terrain.run_ended) run = 'Stopped · waiting for START';
 if (terrain?.run_ended) run = terrain.run_end_cause === 1 ? 'Halted · picked up' : 'Halted · overturned';
 if (cards[2].freshness === 'Stale' && !terrain?.run_ended) run = 'Motion status stale';

 if (motion) {
  const speed = motionSpeed(motion);
  const targets = liveGroup(packet, 2);
  const pivot = targets?.targets_valid && targets.left_target * targets.right_target < 0;
  cards[2].primary = speed === null ? 'Speed unavailable' : `${Math.round(speed)} mm/s${pivot ? ' · pivoting' : ''}`;
  const left = motion.left_speed_valid ? motion.left_speed_mm_s : '—';
  const right = motion.right_speed_valid ? motion.right_speed_mm_s : '—';
  cards[2].secondary = `Wheels L ${left} / R ${right} mm/s`;
 }
 if (line) {
  cards[3].primary = motion?.mission_state === 3 ? 'Junction maneuver' : lineHeadline({...packet, buddies: {...buddies, 2: motion}}, line);
  const barcode = barcodeView(packet, {mapRevision});
  cards[3].secondary = barcode.state === 'decoded' ? `${barcode.headline} · mapped` : barcode.headline;
 }
 if (terrain) {
  cards[4].primary = terrain.hump_valid ? `Last hump ${terrain.hump_mm} ± ${terrain.uncertainty_mm} mm` : 'No valid hump yet';
  cards[4].secondary = terrain.run_ended
   ? (terrain.run_end_cause === 1 ? `Picked up · ${terrain.lift_cm} cm` : 'Overturned')
   : terrain.data_valid ? terrainNames[terrain.terrain] ?? 'Terrain unknown' : 'Terrain unavailable';
 }
 if (obstacle) {
  const age = ((packet.uptime_ms - obstacle.front_at_ms) >>> 0) + elapsed;
  cards[5].primary = age >= 1500 ? 'Front distance unavailable' : obstacle.front_status === 0
   ? `${(obstacle.front_mm / 10).toFixed(1)} cm ahead` : rangeNames[obstacle.front_status] ?? 'Front distance unavailable';
  const outputs = obstacleOutputs(obstacle);
  const bypass = liveGroup(packet, 5);
  const stage = bypass?.backing_off ? bypassProgress(bypass) : obstacle.state === 9 && obstacle.decision_valid && obstacle.side > 0
   ? `Bypassing ${outputs.direction.toLowerCase()}` : obstacle.state === 10 ? outputs.rejoin : avoidanceNames[obstacle.state] ?? 'State unknown';
  const width = obstacle.profile_valid && obstacle.left_found && obstacle.right_found
   ? `${obstacle.state === 0 ? 'Last width' : 'Width'} ${(obstacle.width_mm / 10).toFixed(1)} cm`
   : 'Width unavailable';
  cards[5].secondary = `${width} · ${stage}`;
 }
 return {cards, run: packet?.demo ? `Sample · ${run}` : run, halted: halted || !!terrain?.run_ended};
}

export function renderRoverOverview(packet, options) {
 const overview = roverOverview(packet, options);
 const run = document.getElementById('overview-run');
 if (run.textContent !== overview.run) run.textContent = overview.run;
 run.classList.toggle('is-halted', overview.halted);
 for (const [id, card] of Object.entries(overview.cards)) {
  const element = document.getElementById(`overview-${id}`);
  element.dataset.freshness = card.freshness;
  for (const [field, value] of Object.entries(card)) element.querySelector(`[data-${field}]`).textContent = value;
 }
}
