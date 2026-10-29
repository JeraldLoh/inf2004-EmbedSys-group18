import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {decodeTelemetry,encodeTelemetry} from './binary.mjs';
import {RobotState} from './state.mjs';
import {decodePublish,mqttString,packet,extractPackets} from './mqtt.mjs';
const fixture=()=>new Uint8Array(Buffer.from(readFileSync(new URL('../../host_tests/binary-golden.hex',import.meta.url),'utf8').trim(),'hex'));
test('C-produced bytes decode with signed fields, embedded zeros and original Buddy 4 frame',()=>{
 const v=decodeTelemetry(fixture());
 assert.equal(v.seq,7);assert.equal(v.uptime_ms,1000);assert.equal(v.demo,true);
 assert.equal(v.speed_percent,42);assert.equal(v.distance_cm,93);
 assert.equal(v.buddies[3].steering,-100);assert.equal(v.buddies[3].adc,4095);
 assert.equal(v.buddies[4].data_valid,true);
 assert.equal(v.buddies[5].direction_halfdeg,-160);assert.equal(v.buddies[5].event_seq,0xffffffff);
 assert.equal(v.buddies[5].last_status,4);assert.equal(v.buddies[5].front_mm,930);
 assert.equal(v.buddies[2],undefined);
});
test('JS encoder agrees byte for byte with independently populated C fixture',()=>{
 const f=fixture(),v=decodeTelemetry(f);assert.deepEqual(encodeTelemetry(v),f);
});
test('malformed, duplicate, invalid and unknown-version packets are rejected atomically',()=>{
 const f=fixture();for(let n=0;n<f.length;n++)assert.throws(()=>decodeTelemetry(f.slice(0,n)));
 for(const [offset,value] of [[1,99],[2,99],[3,255],[14,101]]){
  const bad=f.slice();bad[offset]=value;assert.throws(()=>decodeTelemetry(bad));
 }
 const first=f.slice(18,18+6+f[19]);
 assert.throws(()=>decodeTelemetry(new Uint8Array([...f,...first])));
 const s=new RobotState();s.receive('telemetry',f,1);const previous=s.telemetry;
 assert.throws(()=>s.receive('telemetry',f.slice(0,-1),2));assert.equal(s.telemetry,previous);
});
test('MQTT preserves raw bytes, retained binary is ignored, and text heartbeat still works',()=>{
 const f=fixture(),p=packet(0x30,new Uint8Array([...mqttString('test/telemetry'),...f]));
 const m=decodePublish(extractPackets(p).packets[0]);assert.deepEqual(m.payload,f);
 const s=new RobotState();s.connected=true;
 assert.equal(s.receive('telemetry',f,0,true),false);
 s.receive('heartbeat',new TextEncoder().encode('{"schema":1,"demo":true,"seq":1}'),0);
 s.receive('telemetry',m.payload,0);assert.equal(s.telemetryView(0).fresh,true);
 assert.equal(s.telemetryView(3).fresh,false);
});
test('no snapshot leaves buddies unavailable, and bad values cannot silently wrap',()=>{
 const v={demo:true,seq:0,uptime_ms:0,fault_mask:0,speed_percent:0,distance_cm:0,buddies:{}};
 assert.deepEqual(decodeTelemetry(encodeTelemetry(v)).buddies,{});
 assert.throws(()=>encodeTelemetry({...v,buddies:{3:{steering:101}}}));
 assert.throws(()=>encodeTelemetry({...v,buddies:{5:{front_status:5}}}));
});
