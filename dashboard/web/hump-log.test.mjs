import test from 'node:test';
import assert from 'node:assert/strict';
import {createHumpLog,updateHumpLog,humpSummary} from './hump-log.mjs';

// Telemetry packet: controller mode, Buddy 4 hump word and end-of-run flag.
const packet=(ms,mode,seq,{mm=9,unc=7,pitch=4,cm=19,valid=1,ended=false,cause=1,lift=27}={})=>({demo:false,uptime_ms:ms,
 buddies:{2:{system_mode:mode},4:{hump_seq:seq,hump_mm:mm,uncertainty_mm:unc,peak_pitch_deg:pitch,path_cm:cm,hump_valid:valid,
  run_ended:ended,run_end_cause:cause,lift_cm:lift}}});
const feed=(packets)=>{let log=createHumpLog(),events=[];for(const p of packets){const r=updateHumpLog(log,p);log=r.log;events.push(...r.events);}return {log,events};};

test('counts each new hump from START, with its measurements',()=>{
 const {log,events}=feed([packet(1000,1,5),packet(1200,2,5),packet(1400,2,5),
  packet(5000,2,6,{mm:9,unc:7,pitch:4,cm:19}),packet(5200,2,6,{mm:9}),packet(9000,2,7,{mm:11,unc:5,pitch:5,cm:20,valid:0})]);
 assert.equal(log.phase,'running');
 assert.equal(log.humps.length,2);   // the hump seen before START is not counted
 assert.deepEqual(log.humps[0],{n:1,mm:9,uncertainty_mm:7,peak_pitch_deg:4,length_cm:19,valid:true,at_ms:5000});
 assert.equal(log.humps[1].valid,false);
 assert.deepEqual(humpSummary(log).valid,1);
 assert.ok(events.some(e=>e.text.startsWith('Hump #2: 11 mm')));
});

test('freezes at end of run, so the lift cannot add a hump',()=>{
 const {log,events}=feed([packet(1000,2,0),packet(3000,2,1),packet(6000,2,1,{ended:true,lift:27}),
  packet(6200,5,2,{ended:true}),packet(6400,5,3)]);
 assert.equal(log.phase,'ended');
 assert.equal(log.humps.length,1);
 assert.equal(log.liftCm,27);
 assert.match(humpSummary(log).state,/lifted 27 cm/);
 assert.ok(events.some(e=>e.text==='End of run: 1 hump recorded.'));
});

test('a new START clears the previous run; STOP freezes it',()=>{
 let r=feed([packet(1000,2,0),packet(2000,2,1),packet(3000,5,1)]);
 assert.equal(r.log.phase,'stopped');assert.equal(r.log.humps.length,1);
 let log=r.log;
 log=updateHumpLog(log,packet(4000,5,2)).log;           // no counting while stopped
 log=updateHumpLog(log,packet(5000,2,2)).log;           // START again
 assert.equal(log.phase,'running');assert.equal(log.humps.length,0);
 log=updateHumpLog(log,packet(6000,2,3)).log;
 assert.equal(log.humps.length,1);
});

test('counter wrap mod 8 still counts, and a board restart re-baselines',()=>{
 const seqs=[7,0,1,2,3,4,5,6,7,0];
 const {log}=feed([packet(100,2,6),...seqs.map((s,i)=>packet(1000+i*1000,2,s))]);
 assert.equal(log.humps.length,10);
 let l=updateHumpLog(log,packet(50,1,0)).log;           // uptime went backwards: rebooted
 assert.equal(l.phase,'stopped');
 l=updateHumpLog(l,packet(500,2,0)).log;
 assert.equal(l.phase,'running');assert.equal(l.humps.length,0);
});

test('joining mid-run starts counting from now; no Buddy 4 data is ignored',()=>{
 const {log}=feed([packet(1000,2,3),packet(2000,2,4)]);
 assert.equal(log.humps.length,1);
 const none=updateHumpLog(createHumpLog(),{uptime_ms:5,buddies:{2:{system_mode:2}}});
 assert.equal(none.log.phase,'idle');assert.deepEqual(none.events,[]);
});
