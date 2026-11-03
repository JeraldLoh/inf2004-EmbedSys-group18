import test from 'node:test';
import assert from 'node:assert/strict';
import {robotEvents,lineSearchEvents} from './robot-events.mjs';
const packet=(mode=2,stops=0,faults=0,ms=10000,mission=1)=>({demo:false,uptime_ms:ms,buddies:{2:{system_mode:mode,stop_reasons:stops,hardware_faults:faults,mission_state:mission}}});
test('logs named stops once, and never labels a queued request executed',()=>{
 const a=packet(),b=packet(5,8);const events=robotEvents(a,b);assert.ok(events.some(e=>e.text.includes('Motion fault')));assert.deepEqual(robotEvents(b,b),[]);assert.ok(!events.some(e=>e.text.includes('executed')));
});
test('reports faults clearing and uptime restart, but not normal counter wrap',()=>{
 assert.ok(robotEvents(packet(5,8,4),packet()).some(e=>e.text.includes('cleared')));
 assert.ok(robotEvents(packet(2,0,0,15000),packet(0,0,0,200)).some(e=>e.text.includes('restarted')));
 assert.ok(!robotEvents(packet(2,0,0,0xfffffff0),packet(2,0,0,20)).some(e=>e.text.includes('restarted')));
});
test('preview and absent sections cannot create real stop messages',()=>{
 assert.deepEqual(robotEvents(null,{...packet(5,8),demo:true}),[]);assert.deepEqual(robotEvents(packet(),{uptime_ms:20,buddies:{}}),[]);
});

test('search events explain transitions and failures without claiming successful recovery',()=>{
 const status=(searching,searches=1,search_fails=0)=>({demo:false,uptime_ms:1000,present:1,motion:{at_ms:1000,searching,searches,search_fails}});
 assert.ok(lineSearchEvents(status(false,0),status(true)).some(e=>e.text.includes('Searching for line')));
 assert.deepEqual(lineSearchEvents(status(true),status(true)),[]);
 assert.ok(lineSearchEvents(status(true),status(false,1,1)).some(e=>e.text.includes('failed')));
 assert.ok(!lineSearchEvents(status(true),status(false)).some(e=>e.text.includes('found')));
 const stale=status(true);stale.motion.at_ms=0;stale.uptime_ms=3000;assert.deepEqual(lineSearchEvents(null,stale),[]);
});
