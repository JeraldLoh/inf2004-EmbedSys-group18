import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {decodeLiveStatus,encodeLiveStatus,liveGroup,lineHeadline,bypassProgress} from './live-status.mjs';
import {RobotState} from './state.mjs';
const golden=new Uint8Array(Buffer.from(readFileSync(new URL('../../host_tests/live-status-golden.hex',import.meta.url),'utf8').trim(),'hex'));
test('live status golden vector preserves signed targets and progress',()=>{
 const s=decodeLiveStatus(golden);assert.equal(s.motion.left_target,-40);assert.equal(s.motion.right_target,80);assert.equal(s.motion.rejected,2);assert.equal(s.line.paused,true);assert.equal(s.bypass.segment,2);assert.equal(s.bypass.segments,5);assert.deepEqual(encodeLiveStatus(s),golden);
 assert.equal(bypassProgress(s.bypass),'S-curve segment 3 of 5');
});
test('invalid packet never overwrites last good status or refreshes heartbeat',()=>{
 const s=new RobotState();s.connected=true;s.receive('live_status',golden,1);assert.equal(s.lastHeartbeat,null);assert.equal(s.status(1),'waiting');
 const good=s.liveStatus;for(const [offset,value] of [[0,0],[1,3],[2,1],[3,2],[8,8],[9,8],[10,2],[11,8],[40,46],[41,3],[42,1],[43,1]]){const p=golden.slice();p[offset]=value;assert.throws(()=>s.receive('live_status',p,2));assert.equal(s.liveStatus,good);}
 assert.throws(()=>s.receive('live_status',golden.slice(1),2));assert.equal(s.liveStatusAt,1);assert.equal(s.receive('live_status',golden,5,true),false);s.disconnect();assert.equal(s.liveStatus,null);
});
test('group freshness uses independent age, handles wrap and future stamps',()=>{
 const s=decodeLiveStatus(golden),packet={demo:false,live_status:s,live_status_elapsed_ms:1200};assert.ok(liveGroup(packet,2));packet.live_status_elapsed_ms=1300;assert.equal(liveGroup(packet,2),null);packet.live_status_elapsed_ms=0;
 s.uptime_ms=20;s.motion.at_ms=0xfffffff0;assert.ok(liveGroup(packet,2));s.motion.at_ms=21;assert.equal(liveGroup(packet,2),null);s.demo=true;assert.equal(liveGroup(packet,3),null);
});
test('line recovery and hump pause are explicit and expire',()=>{
 const packet={demo:false,live_status:decodeLiveStatus(golden)},s={line_state:0};assert.equal(lineHeadline(packet,s),'Sensing paused · hump');packet.live_status.line.paused=false;assert.equal(lineHeadline(packet,s),'Searching for line');packet.live_status_elapsed_ms=1600;assert.equal(lineHeadline(packet,s),'Centred');assert.equal(bypassProgress(null),'Not reported / stale');
});


test('fallback bypass and backing-off captions do not invent segment names',()=>{
 assert.equal(bypassProgress({active:true,fast:false,segment:1,segments:7}),'Leg 2 of 7');
 assert.equal(bypassProgress({active:true,fast:true,segment:5,segments:5}),'Finishing bypass');
 assert.equal(bypassProgress({active:false,backing_off:true}),'Backing off for clearance');
 assert.equal(bypassProgress({active:false,backing_off:false}),'No bypass in progress');
});

test('sensor cards render targets, search, hump pause and preserve failed avoidance',async()=>{
 const {renderSensorHighlights}=await import('./sensor-panels.mjs');
 const elements=new Map(),element=()=>({textContent:'',lastChild:{textContent:''},classList:{toggle(){}},style:{},setAttribute(){},replaceChildren(){},append(){}});
 const previous=globalThis.document;
 globalThis.document={createElement:element,getElementById:id=>{if(!elements.has(id))elements.set(id,element());return elements.get(id);}};
 try{
  const packet={demo:false,uptime_ms:1200,live_status:decodeLiveStatus(golden),buddies:{2:{},3:{line_state:0,steering:0},5:{state:11,front_status:4,last_status:4}}};
  const text=id=>elements.get(id).textContent;
  renderSensorHighlights(2,packet,{stale:false},true,0);assert.equal(text('motion-target'),'-40 / 80 mm/s');
  renderSensorHighlights(3,packet,{stale:false},true,0);assert.equal(text('line-primary'),'Sensing paused · hump');
  packet.live_status.line.paused=false;renderSensorHighlights(3,packet,{stale:false},true,0);assert.equal(text('line-primary'),'Searching for line');
  packet.live_status.bypass.active=false;packet.live_status.bypass.backing_off=false;
  renderSensorHighlights(5,packet,{stale:false},true,0);assert.equal(text('obstacle-state'),'Failed');
  packet.live_status_elapsed_ms=1600;renderSensorHighlights(2,packet,{stale:false},true,0);assert.equal(text('motion-target'),'—');
 }finally{if(previous===undefined)delete globalThis.document;else globalThis.document=previous;}
});

test('legacy live-status v1 remains readable without invented calibration',()=>{
 const old=golden.slice(0,64);old[1]=1;const s=decodeLiveStatus(old);
 assert.equal(s.motion.encoder_left_counts,0);assert.deepEqual(encodeLiveStatus(s),old);
 const current=decodeLiveStatus(golden);assert.equal(current.motion.encoder_left_counts,1271);assert.equal(current.motion.encoder_right_counts,1264);assert.equal(current.motion.wheel_diameter_mm,65);
});

test('line card distinguishes reacquiring and stopped recovery from centred sensors',()=>{
 const packet={demo:false,live_status:decodeLiveStatus(golden),buddies:{2:{system_mode:2,stop_reasons:0}}};packet.live_status.line.paused=false;
 assert.equal(lineHeadline(packet,{line_state:0}),'Searching for line');
 packet.buddies[2].system_mode=5;packet.buddies[2].stop_reasons=128;
 assert.equal(lineHeadline(packet,{line_state:0}),'Stopped · recovery failed');
 delete packet.live_status;delete packet.buddies[2];assert.equal(lineHeadline(packet,{line_state:4}),'Line lost · reacquiring');
});
