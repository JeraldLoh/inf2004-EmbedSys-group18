export function validateCommand(command) {
 if(command==='START'||command==='STOP') return command;
 if(/^SET_SPEED [0-9]{1,3}$/.test(command) && Number(command.slice(10))<=100) return command;
 throw new Error('Use START, STOP or SET_SPEED 0..100');
}
export class RobotState {
 constructor(){this.disconnect();}
 disconnect(){this.connected=false;this.lastHeartbeat=null;this.telemetry=null;this.telemetryAt=null;this.demo=null;this.result=null;this.received=0;}
 status(now){return !this.connected?'disconnected':this.lastHeartbeat===null?'waiting':now-this.lastHeartbeat<15?'online':'offline';}
 connectionLost(){this.connected=false;this.result=null;}
 connectionRestored(){this.connected=true;this.lastHeartbeat=null;this.result=null;}
 telemetryView(now){
  const age=this.telemetryAt===null?null:Math.max(0,now-this.telemetryAt);
  return {values:this.telemetry,age,fresh:age!==null&&age<3&&this.status(now)==='online'};
 }

 receive(topic,payload,now,retained=false){
  if(retained)return false;
  if(payload.length>1024)throw new Error('Message too large');
  const v=JSON.parse(payload);
  if(!v||Array.isArray(v)||v.schema!==1||typeof v.demo!=='boolean')throw new Error('Invalid schema or demo flag');
  if(topic==='heartbeat'){
   if(!Number.isSafeInteger(v.seq)||v.seq<0)throw new Error('Invalid heartbeat sequence');
   this.lastHeartbeat=now;
  }else if(topic==='telemetry'){
   for(const [key,max] of [['speed_percent',100],['distance_cm',100000]])if(!Number.isInteger(v[key])||v[key]<0||v[key]>max)throw new Error('Invalid '+key);
   this.telemetry=v;this.telemetryAt=now;
  }else if(topic==='command_result'){
   if(!['simulated','queued','rejected','queue_full'].includes(v.status))throw new Error('Invalid command result');
   this.result=v;
  }else throw new Error('Unexpected topic');
  this.demo=v.demo;this.received++;return true;
 }
}
