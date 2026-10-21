import test from 'node:test';
import assert from 'node:assert/strict';
import {RobotState, validateCommand} from './state.mjs';
import {mqttString, packet, extractPackets, decodePublish, validateConnection} from './mqtt.mjs';
test('heartbeat expires exactly at 15 seconds; telemetry cannot refresh it',()=>{
 const s=new RobotState();assert.equal(s.status(0),'disconnected');s.connected=true;
 assert.equal(s.status(0),'waiting');
 s.receive('heartbeat','{"schema":1,"demo":true,"seq":1}',10);
 assert.equal(s.status(24.999),'online');
 s.receive('telemetry','{"schema":1,"demo":true,"speed_percent":30,"distance_cm":80}',24);
 assert.equal(s.status(25),'offline');s.disconnect();s.connected=true;assert.equal(s.status(26),'waiting');
});
test('reject malformed data and ignore retained heartbeats',()=>{
 const s=new RobotState();
 for(const p of ['[]','bad','{"schema":1}','{"schema":1,"demo":true,"seq":false}']) assert.throws(()=>s.receive('heartbeat',p,0));
 assert.equal(s.receive('heartbeat','{"schema":1,"demo":true,"seq":1}',0,true),false);
 assert.equal(s.lastHeartbeat,null);
 assert.throws(()=>s.receive('telemetry','{"schema":1,"demo":false,"speed_percent":101,"distance_cm":1}',0));
});
test('strict commands with bounded speed',()=>{
 for(const c of ['START','STOP','SET_SPEED 0','SET_SPEED 100']) assert.equal(validateCommand(c),c);
 for(const c of ['START\n','START\0','SET_SPEED -1','SET_SPEED 101','SET_SPEED 2x']) assert.throws(()=>validateCommand(c));
});
test('MQTT framing tolerates a split packet and multiple packets',()=>{
 const p=packet(0x30,new Uint8Array(150));assert.equal(p[1],150%128+128);
 let r=extractPackets(p.slice(0,10));assert.equal(r.packets.length,0);assert.equal(r.rest.length,10);
 r=extractPackets(new Uint8Array([...p,0xd0,0]));assert.equal(r.packets.length,2);assert.equal(r.rest.length,0);
 assert.throws(()=>extractPackets(new Uint8Array([0x30,255,255,255,255])));
});
test('UTF-8 topics and retained flag decode correctly',()=>{
 const topic=mqttString('group18/robot1/heartbeat');
 const body=new Uint8Array([...topic,...new TextEncoder().encode('{"schema":1}')]);
 const p=extractPackets(packet(0x31,body)).packets[0];
 const m=decodePublish(p);assert.equal(m.topic,'group18/robot1/heartbeat');assert.equal(m.retained,true);
 assert.throws(()=>decodePublish({header:0x30,body:new Uint8Array([0,20,65])}));
});

test('connection settings validate before changing a session',()=>{
 assert.doesNotThrow(()=>validateConnection('ws://localhost:9001','group18/robot1'));
 for(const [url,prefix] of [['http://localhost:9001','group18/robot1'],['ws://localhost:9001','bad/#'],['ws://localhost:9001','bad\x00topic'],['ws://user:pass@localhost:9001','group18/robot1']]) assert.throws(()=>validateConnection(url,prefix));
});
