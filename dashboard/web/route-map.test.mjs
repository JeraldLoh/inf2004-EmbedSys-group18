import test from 'node:test';
import assert from 'node:assert/strict';
import {DEFAULT_MAP,encodeMapSet,encodeMapShow,decodeMapCommand,mapFromHex,mapToHex,mapSummary,packMap,sameMap,charIndex} from './route-map.mjs';
import {RobotState} from './state.mjs';

test('power-on map matches the firmware reply bytes',()=>{
 // Same value the C bridge test expects for barcode_map_defaults().
 assert.equal(mapToHex(DEFAULT_MAP),'d108'+'0'.repeat(24));
 assert.deepEqual(mapFromHex('d108'+'0'.repeat(24)),{A:1,B:2,C:3,D:4});
});
test('MAP_SET is 22 bytes and round-trips; MAP_SHOW is 8',()=>{
 const map={K:1,Q:2,Z:3,7:4};
 const bytes=encodeMapSet(map,6);
 assert.equal(bytes.length,22);assert.deepEqual([...bytes.slice(0,8)],[0xb1,2,3,3,6,0,0,0]);
 assert.deepEqual(decodeMapCommand(bytes),{kind:'SET',command_id:6,map:{K:1,Q:2,Z:3,7:4}});
 assert.deepEqual([...encodeMapShow(5)],[0xb1,2,3,4,5,0,0,0]);
 assert.equal(mapSummary(map),'K=L Q=R Z=S 7=U');
});
test('bad letters, turns, ids and padding are rejected',()=>{
 assert.equal(charIndex('k'),10);assert.equal(charIndex('9'),35);assert.equal(charIndex('#'),-1);
 assert.throws(()=>packMap({'#':1}));assert.throws(()=>packMap({A:5}));assert.throws(()=>encodeMapSet({A:1},0));
 assert.throws(()=>mapFromHex('d108'));assert.throws(()=>mapFromHex('d108'+'0'.repeat(22)+'f0'));
 const bad=encodeMapSet({},1);bad[8]=6;assert.throws(()=>decodeMapCommand(bad));
});
test('command_result with map updates the confirmed rover map',()=>{
 const s=new RobotState();
 s.receive('command_result',JSON.stringify({schema:1,demo:false,status:'applied',command_id:6,map:mapToHex({K:1})}),3);
 assert.ok(sameMap(s.routeMap,{K:1}));assert.equal(s.routeMapAt,3);
 assert.throws(()=>s.receive('command_result','{"schema":1,"demo":false,"status":"applied","command_id":7,"map":"zz"}',4));
 assert.ok(sameMap(s.routeMap,{K:1}));
});
