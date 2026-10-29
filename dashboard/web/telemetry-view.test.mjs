import test from 'node:test';import assert from 'node:assert/strict';
import {buddyView} from './telemetry-view.mjs';import {sectionAgeMs,decodeBuddy4} from './binary.mjs';
test('missing buddies are unavailable and snapshot age survives kernel counter wrap',()=>{
 assert.equal(buddyView(null,3).available,false);
 assert.equal(sectionAgeMs({uptime_ms:50},{updated_ms:0xfffffff0},20),86);
 assert.equal(sectionAgeMs({uptime_ms:50},{updated_ms:51}),null);
});
test('Buddy 4 invalid data and missing MAG calibration never become usable angles',()=>{
 const raw=new Uint8Array(17);raw[0]=0x21;
 let s=decodeBuddy4(raw);assert.equal(s.pitch_deg,null);assert.equal(s.heading_deg,null);
 raw[4]=0x80;s=decodeBuddy4(raw);assert.equal(s.pitch_deg,0);assert.equal(s.heading_deg,null);
 raw[3]=0x30;s=decodeBuddy4(raw);assert.equal(s.heading_deg,0);
 raw[2]=0x40;s=decodeBuddy4(raw);assert.equal(s.heading_deg,null);
});
test('Buddy 5 no-echo never appears as a valid zero or maximum distance',()=>{
 const p={uptime_ms:1000,demo:false,buddies:{5:{updated_ms:900,state:0,front_status:1,front_mm:4000,front_at_ms:500,last_status:4,servo_valid:0}}};
 const v=buddyView(p,5,200);assert.match(v.rows[1][1],/No echo/);assert.doesNotMatch(v.rows[1][1],/400/);
 assert.match(v.rows[1][1],/700 ms/);assert.equal(buddyView(p,5,2000).stale,true);
});


test('probe routes and missing durations never invent a barcode bar measurement',async()=>{
 const {barcodeDurationText}=await import('./telemetry-view.mjs');
 assert.equal(barcodeDurationText({scan_valid:1,barcode_source:1,bar_ms:25}),'Unavailable');
 assert.equal(barcodeDurationText({scan_valid:1,barcode_source:0,bar_ms:0}),'Unavailable');
 assert.equal(barcodeDurationText({scan_valid:1,barcode_source:0,bar_ms:25}),'25 ms');
});
