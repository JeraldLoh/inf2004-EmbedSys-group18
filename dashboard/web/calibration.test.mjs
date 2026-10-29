import test from 'node:test';
import assert from 'node:assert/strict';
import {calibrationView} from './calibration.mjs';
test('missing telemetry never claims calibration passed or failed',()=>{
 for(const id of [3,4,5])for(const item of calibrationView(id).items)assert.equal(item.state,'unknown');
});
test('Buddy 3 false calibration cannot distinguish running from failure',()=>{
 assert.equal(calibrationView(3,{calibrated:1}).items[0].state,'passed');
 const v=calibrationView(3,{calibrated:0});assert.equal(v.items[0].state,'pending');
 assert.match(v.note,/failure.*not reported/i);
});
test('Buddy 4 separates level and magnetic calibration without inferring failure',()=>{
 const states=code=>calibrationView(4,{calibration:code}).items.map(x=>x.state);
 assert.deepEqual(states(0),['pending','unknown']);
 assert.deepEqual(states(1),['running','unknown']);
 assert.deepEqual(states(2),['passed','unknown']);
 assert.deepEqual(states(3),['passed','passed']);
 assert.match(calibrationView(4,{calibration:2}).note,/failure.*not reported/i);
});
test('sensor failure does not erase a successful calibration',()=>{
 const v=calibrationView(4,{calibration:3,data_valid:false,detail_faults:3});
 assert.deepEqual(v.items.map(x=>x.state),['passed','passed']);
});
test('Buddy 5 commanded servo position is not calibration evidence',()=>{
 const v=calibrationView(5,{servo_valid:1,servo_deg:90,front_status:0});
 assert(v.items.every(x=>x.state==='unknown'));
});
