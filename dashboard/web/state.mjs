import {decodeTelemetry} from './binary.mjs';
import {decodeDiagnostics,newDiagnostics} from './diagnostics.mjs';
export function validateCommand(command) {
 if(command==='START'||command==='STOP') return command;
 throw new Error('Only START and STOP are supported');
}
export class RobotState {
 constructor(){this.disconnect();}
 disconnect(){this.connected=false;this.lastHeartbeat=null;this.telemetry=null;this.telemetryAt=null;this.demo=null;this.result=null;this.received=0;this.diagnostics=newDiagnostics();}
 status(now){return !this.connected?'disconnected':this.lastHeartbeat===null?'waiting':now-this.lastHeartbeat<15?'online':'offline';}
 connectionLost(){this.connected=false;this.result=null;}
 connectionRestored(){this.connected=true;this.lastHeartbeat=null;this.result=null;}
 telemetryView(now){
  const age=this.telemetryAt===null?null:Math.max(0,now-this.telemetryAt);
  return {values:this.telemetry,age,fresh:age!==null&&age<3&&this.status(now)==='online'};
 }

 receive(topic,payload,now,retained=false){
  if(retained)return false;
  try{return this.receiveValidated(topic,payload,now);}catch(error){
   this.diagnostics.rxRejected++;this.diagnostics.lastRxError={reason:error.message,at:now};throw error;
  }
 }
 receiveValidated(topic,payload,now){
  if(payload.length>1024)throw new Error('Message too large');
  if(topic==='diagnostics'){
   const d=decodeDiagnostics(payload);this.diagnostics.device=d;this.diagnostics.deviceAt=now;this.received++;return true;
  }
  const binary=topic==='telemetry'&&payload instanceof Uint8Array&&payload[0]===0xb1;
  const text=typeof payload==='string'?payload:new TextDecoder('utf-8',{fatal:true}).decode(binary?new Uint8Array():payload);
  const v=binary?decodeTelemetry(payload):JSON.parse(text);
  if(!v||Array.isArray(v)||!(binary?[2,3]:[1]).includes(v.schema)||typeof v.demo!=='boolean')throw new Error('Invalid schema or demo flag');
  if(topic==='heartbeat'){
   if(!Number.isSafeInteger(v.seq)||v.seq<0)throw new Error('Invalid heartbeat sequence');
   this.lastHeartbeat=now;
  }else if(topic==='telemetry'){
   for(const [key,max] of [['speed_percent',100],['distance_cm',100000]])if(!Number.isInteger(v[key])||v[key]<0||v[key]>max)throw new Error('Invalid '+key);
   this.telemetry=v;this.telemetryAt=now;
   this.diagnostics.packetBytes=typeof payload==='string'?new TextEncoder().encode(payload).length:payload.byteLength;
  }else if(topic==='command_result'){
   if(!['simulated','queued','rejected','queue_full'].includes(v.status))throw new Error('Invalid command result');
   this.result=v;
  }else throw new Error('Unexpected topic');
  this.demo=v.demo;this.received++;return true;
 }
}
