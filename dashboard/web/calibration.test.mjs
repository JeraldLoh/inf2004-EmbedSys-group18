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

test('wheel settings are configured, not passed, and qualification alone proves nothing',()=>{
 const configured=calibrationView(2,{}, {encoder_left_counts:1271,encoder_right_counts:1264,wheel_diameter_mm:65});
 assert.equal(configured.items[0].label,'Configured');assert.equal(configured.items[0].state,'configured');assert.match(configured.note,/1271.*1264.*65/);
 for(const live of [null,{qualified:true},{encoder_left_counts:1271,encoder_right_counts:0,wheel_diameter_mm:65}])assert.equal(calibrationView(2,{},live).items[0].state,'unknown');
 assert.equal(calibrationView(2,null,{encoder_left_counts:1271,encoder_right_counts:1264,wheel_diameter_mm:65}).items[0].label,'Awaiting data');
});

test('explicit Buddy 3 calibration states override the legacy ready flag',()=>{
 assert.equal(calibrationView(3,{calibrated:0},{calibration_state:1}).items[0].label,'Calibrating');
 assert.equal(calibrationView(3,{calibrated:0},{calibration_state:3,calibration_reason:1}).items[0].label,'Failed');
 assert.match(calibrationView(3,{calibrated:0},{calibration_state:3,calibration_reason:1}).note,/contrast/i);
 assert.equal(calibrationView(3,{calibrated:1},{calibration_state:2}).items[0].label,'Passed');
});
