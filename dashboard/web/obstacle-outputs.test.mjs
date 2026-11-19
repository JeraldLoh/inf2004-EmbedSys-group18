import test from 'node:test';
import assert from 'node:assert/strict';
import {obstacleOutputs} from './obstacle-outputs.mjs';
test('missing data and failed avoidance never claim a clear path',()=>{
 assert.equal(obstacleOutputs(null).detection,'Awaiting data');
 assert.equal(obstacleOutputs({state:11}).detection,'Unknown · avoidance failed');
 assert.equal(obstacleOutputs({state:0,front_status:1}).detection,'No active encounter');
});
test('reported profile and decision keep units, lower-bound and encounter context',()=>{
 const s={state:9,profile_valid:1,width_mm:125,left_found:0,right_found:1,decision_valid:1,side:2};
 assert.equal(obstacleOutputs(s).width,'At least 12.5 cm');
 assert.equal(obstacleOutputs(s).direction,'Right');
 s.state=0;
 assert.equal(obstacleOutputs(s).width,'At least 12.5 cm · last encounter');
 assert.equal(obstacleOutputs(s).direction,'Right · last encounter');
});
test('rejoin is a reported state, not an invented command or success',()=>{
 assert.equal(obstacleOutputs({state:10}).rejoin,'Searching for line');
 assert.equal(obstacleOutputs({state:9}).rejoin,'Not active · bypassing');
 assert.equal(obstacleOutputs({state:0,line_search_ok:5}).rejoin,'Not active');
 assert.equal(obstacleOutputs({state:11}).rejoin,'Avoidance failed');
});
test('absent profile or decision stays unavailable',()=>{
 const v=obstacleOutputs({state:1,profile_valid:0,width_mm:100,decision_valid:0,side:1});
 assert.equal(v.detection,'Yes · encounter active');
 assert.equal(v.width,'Not reported yet');assert.equal(v.direction,'Not decided yet');
});
