import test from 'node:test';
import assert from 'node:assert/strict';
import {DEFAULT_MAP,encodeMapSet,encodeMapShow,decodeMapCommand,mapFromHex,mapToHex,mapSummary,packMap,sameMap,charIndex,assignDirection,lettersForDirection} from './route-map.mjs';
import {RobotState} from './state.mjs';
import {barcodeView} from './barcode-status.mjs';
import {encodeLiveStatus,decodeLiveStatus} from './live-status.mjs';
import {ASSIGNABLE_ACTIONS,editableRouteMap} from './route-map.mjs';

test('route editor leaves straight to the default junction behavior',()=>{
 const oldMap={A:1,B:2,C:3,D:4};
 assert.deepEqual(ASSIGNABLE_ACTIONS.map(action=>action.name),['Left','Right','U-turn']);
 const command=decodeMapCommand(encodeMapSet(editableRouteMap(oldMap),1));
 assert.deepEqual(command.map,{A:1,B:2,D:4});
 assert.equal(oldMap.C,3);
});

test('power-on map matches the firmware reply bytes',()=>{
 assert.equal(mapToHex(DEFAULT_MAP),'0'.repeat(28));
 assert.deepEqual(mapFromHex('0'.repeat(28)),{});
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
test('new assignments replace a slot and move duplicate letters',()=>{
 assert.deepEqual(assignDirection({A:1,B:2},1,'K'),{K:1,B:2});
 assert.deepEqual(assignDirection({A:1,B:2},1,'B'),{B:1});
 assert.deepEqual(assignDirection({A:1,B:2},1,''),{B:2});
});
test('confirmed direction rows show the live map, including older maps with duplicate directions',()=>{
 const map={A:1,K:1,B:2,D:4};
 assert.deepEqual(lettersForDirection(map,1),['A','K']);
 assert.deepEqual(lettersForDirection(map,2),['B']);
 assert.deepEqual(lettersForDirection(map,3),[]);
 assert.deepEqual(lettersForDirection(map,4),['D']);
});
test('reader shows only scans from the current confirmed map',()=>{
 const packet={demo:false,live_status:{demo:false,present:2,uptime_ms:1500,line:{
  at_ms:1500,barcode_event_ms:1000,barcode_state:2,barcode_character:65,barcode_route:1,barcode_failures:0,map_revision:4
 }}};
 const view=barcodeView(packet,{mapRevision:6});
 assert.equal(view.headline,'Ready for a fresh scan');
 packet.live_status.line.map_revision=6;
 packet.live_status.line.barcode_route=2;
 assert.equal(barcodeView(packet,{mapRevision:6}).headline,'Decoded A → Right');
});
test('reader status v5 carries its map revision while v4 still decodes',()=>{
 const line={map_revision:6,barcode_state:2,barcode_character:65,barcode_route:2};
 assert.equal(decodeLiveStatus(encodeLiveStatus({version:5,line})).line.map_revision,6);
 assert.equal(decodeLiveStatus(encodeLiveStatus({version:4,line})).line.map_revision,undefined);
});
test('a reader sample just newer than the packet does not look stalled',()=>{
 const status={demo:false,present:2,uptime_ms:1000,line:{
  at_ms:1001,barcode_state:0,map_revision:6
 }};
 const packet={demo:false,live_status:status};
 assert.equal(barcodeView(packet,{mapRevision:6}).headline,'Ready · waiting for sensor activity');
 status.uptime_ms=8000;
 assert.match(barcodeView(packet,{mapRevision:6}).detail,/reader data is 7\.0 s old/);
});
