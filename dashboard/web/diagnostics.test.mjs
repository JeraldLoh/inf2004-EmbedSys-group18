import test from 'node:test';import assert from 'node:assert/strict';
import {decodeDiagnostics,encodeDiagnostics,validationLabel,equivalentJsonBytes} from './diagnostics.mjs';
import {readFileSync} from 'node:fs';
import {decodeTelemetry} from './binary.mjs';
import {RobotState} from './state.mjs';
test('diagnostics preserves unsigned counters and signed offending value',()=>{
 const v={demo:false,uptime_ms:1000,rejected:3,out_of_range:2,busy:1,last_at_ms:900,section:3,field:5,reason:2,value:-101,min:-100,max:100};
 const wire=encodeDiagnostics(v);assert.equal(wire.length,52);
 assert.deepEqual(decodeDiagnostics(wire),v);assert.equal(validationLabel(v),'Buddy 3 · steering');
 for(let n=0;n<52;n++)assert.throws(()=>decodeDiagnostics(wire.slice(0,n)));
});
test('text-size comparison uses the same represented fields without duplicating Buddy 4 derivations',()=>{
 const bytes=new Uint8Array(Buffer.from(readFileSync(new URL('../../host_tests/binary-golden.hex',import.meta.url),'utf8').trim(),'hex'));
 const v=decodeTelemetry(bytes);assert(equivalentJsonBytes(v)>bytes.length);
 assert.equal(equivalentJsonBytes(null),null);
});
test('diagnostic receive does not refresh heartbeat or replace sensor data',()=>{
 const s=new RobotState();s.connected=true;
 const wire=encodeDiagnostics({demo:true,uptime_ms:0,rejected:0,out_of_range:0,busy:0,last_at_ms:0,section:0,field:0,reason:0,value:0,min:0,max:0});
 assert.equal(s.receive('diagnostics',wire,5),true);assert.equal(s.lastHeartbeat,null);assert.equal(s.telemetry,null);
 assert.equal(s.diagnostics.device.out_of_range,0);assert.equal(s.diagnostics.deviceAt,5);
 assert.equal(s.receive('diagnostics',wire,6,true),false);
});
test('malformed inbound packets count as browser rejections and preserve last good readings',()=>{
 const s=new RobotState();s.receive('telemetry','{"schema":1,"demo":true,"speed_percent":42,"distance_cm":80}',1);
 const before=s.telemetry;assert.throws(()=>s.receive('telemetry','bad',2));
 assert.equal(s.telemetry,before);assert.equal(s.diagnostics.rxRejected,1);assert.match(s.diagnostics.lastRxError.reason,/JSON/i);
 assert.equal(s.diagnostics.packetBytes,new TextEncoder().encode('{"schema":1,"demo":true,"speed_percent":42,"distance_cm":80}').length);
});
test('invalid diagnostic field IDs and inconsistent limits preserve the last report',()=>{
 const s=new RobotState(),good=encodeDiagnostics({demo:false,uptime_ms:1000,rejected:3,out_of_range:2,busy:1,last_at_ms:900,section:3,field:5,reason:2,value:-101,min:-100,max:100});
 s.receive('diagnostics',good,1);const previous=s.diagnostics.device;
 for(const [offset,value] of [[24,255],[25,255],[26,3],[36,0]]){
  const bad=good.slice();bad[offset]=value;assert.throws(()=>s.receive('diagnostics',bad,2));
  assert.equal(s.diagnostics.device,previous);assert.equal(s.diagnostics.deviceAt,1);
 }
 assert.equal(s.diagnostics.rxRejected,4);
});
