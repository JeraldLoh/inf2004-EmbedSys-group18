import test from 'node:test';import assert from 'node:assert/strict';
import {encodeTelemetry,decodeTelemetry} from './binary.mjs';
import {encodeCommand,decodeCommand} from './commands.mjs';
import {motionSpeed,motionView,buddyView} from './telemetry-view.mjs';
import {readFileSync} from 'node:fs';
import {MqttConnection,decodePublish,extractPackets} from './mqtt.mjs';
test('START/STOP command bytes preserve command ID and reject speed commands',()=>{
 const b=encodeCommand('START',0x12345678);assert.deepEqual([...b],[0xb1,2,3,1,0x78,0x56,0x34,0x12]);
 assert.deepEqual(decodeCommand(b),{command:'START',command_id:0x12345678});
 assert.equal(decodeCommand(encodeCommand('STOP',1)).command,'STOP');
 assert.throws(()=>encodeCommand('SET_SPEED 30',1));assert.throws(()=>encodeCommand('START',0));
 for(let n=0;n<8;n++)assert.throws(()=>decodeCommand(b.slice(0,n)));
 const bad=b.slice();bad[3]=3;assert.throws(()=>decodeCommand(bad));
});
test('Buddy 2 signed values, validity and 20-bit faults survive v3',()=>{
 const p={demo:false,seq:1,uptime_ms:20,fault_mask:0x80000,buddies:{2:{updated_ms:10,target_speed_mm_s:30,left_speed_mm_s:-10,right_speed_mm_s:20,distance_mm:-500,target_speed_valid:1,left_speed_valid:1,right_speed_valid:1,distance_valid:1,sequence:255,hardware_faults:0x80000,system_mode:2,mission_state:1}}};
 const b=encodeTelemetry(p),v=decodeTelemetry(b);assert.equal(b[1],3);assert.equal(v.fault_mask,0x80000);assert.equal(v.buddies[2].distance_mm,-500);
 assert.equal(motionSpeed(v.buddies[2]),5);assert.equal(motionSpeed({...v.buddies[2],left_speed_valid:0}),null);assert.equal(motionSpeed(),null);
 assert.throws(()=>encodeTelemetry({...p,fault_mask:0x100000}));
});
test('default result zero is not displayed as motor acceptance',()=>{
 const v=buddyView({uptime_ms:20,buddies:{2:{updated_ms:10,last_motion_result:0,result_pending:0}}},2);
 assert(v.available);assert(v.rows.some(([k,v])=>k==='Latest result'&&v==='No pending result'));
});
test('fresh packet does not refresh an old wheel measurement',()=>{
 const p={uptime_ms:3000,buddies:{2:{updated_ms:1000,left_speed_valid:1,right_speed_valid:1,left_speed_mm_s:30,right_speed_mm_s:30}}};
 const v=motionView(p,0);assert.equal(v.speed,30);assert.equal(v.fresh,false);assert.equal(v.age,2000);
 p.buddies[2].updated_ms=3000;assert.equal(motionView(p,0).fresh,true);assert.equal(motionView(p,1500).fresh,false);
});
test('independent C v3 packet decodes and round trips without losing signed values',()=>{
 const bytes=new Uint8Array(Buffer.from(readFileSync(new URL('../../host_tests/binary-golden-v3.hex',import.meta.url),'utf8').trim(),'hex'));
 const v=decodeTelemetry(bytes);assert.equal(bytes.length,252);assert.equal(v.buddies[2].distance_mm,-500);
 assert.equal(v.buddies[2].left_speed_mm_s,-10);assert.equal(v.fault_mask,0x80000);assert.deepEqual(encodeTelemetry(v),bytes);
 for(let n=0;n<bytes.length;n++)assert.throws(()=>decodeTelemetry(bytes.slice(0,n)));
 const reserved=bytes.slice();reserved[14]|=0x80;assert.throws(()=>decodeTelemetry(reserved));
});
test('MQTT publishes binary START/STOP payloads and never queues speed commands',()=>{
 const sent=[],client=new MqttConnection({onState:()=>{},onLog:()=>{},onMessage:()=>{}});
 client.ready=true;client.prefix='test';client.socket={readyState:WebSocket.OPEN,bufferedAmount:0,send:bytes=>sent.push(bytes)};
 client.publish('START',7);client.publish('STOP',8);
 assert.throws(()=>client.publish('SET_SPEED 30',9));assert.equal(sent.length,2);
 assert.deepEqual(decodeCommand(decodePublish(extractPackets(sent[0]).packets[0]).payload),{command:'START',command_id:7});
 assert.deepEqual(decodeCommand(decodePublish(extractPackets(sent[1]).packets[0]).payload),{command:'STOP',command_id:8});
});
