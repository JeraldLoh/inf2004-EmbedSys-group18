import {encodeCommand} from './commands.mjs';
/* MQTT 3.1.1 over WebSocket: clean sessions, QoS 0, no offline command queue. */
const encoder=new TextEncoder(),decoder=new TextDecoder('utf-8',{fatal:true});
const join=(...arrays)=>new Uint8Array(arrays.flatMap(a=>Array.from(a)));
export function mqttString(value){const b=encoder.encode(value);if(b.length>65535)throw Error('String too long');return join([b.length>>8,b.length&255],b);}
export function packet(header,body=new Uint8Array()){
 if(body.length>65536)throw Error('Packet too large');
 let n=body.length,remaining=[];do{let b=n%128;n=Math.floor(n/128);remaining.push(b|(n?128:0));}while(n);
 return join([header,...remaining],body);
}
export function extractPackets(input){
 let offset=0;const packets=[];
 while(offset<input.length){
  const start=offset,header=input[offset++];let length=0,multiplier=1,count=0,byte;
  do{
   if(offset>=input.length)return {packets,rest:input.slice(start)};
   byte=input[offset++];length+=(byte&127)*multiplier;multiplier*=128;count++;
   if(count===4&&(byte&128))throw Error('Invalid MQTT length');
  }while(byte&128);
  if(length>65536)throw Error('MQTT packet too large');
  if(offset+length>input.length)return {packets,rest:input.slice(start)};
  packets.push({header,body:input.slice(offset,offset+length)});offset+=length;
 }
 return {packets,rest:new Uint8Array()};
}
export function decodePublish({header,body}){
 if(((header>>1)&3)!==0)throw Error('Unsupported publish QoS');
 if(body.length<2)throw Error('Missing topic');
 const n=(body[0]<<8)|body[1];if(!n||n+2>body.length)throw Error('Invalid topic length');
 return {topic:decoder.decode(body.slice(2,2+n)),payload:body.slice(2+n),retained:!!(header&1)};
}
export function validateConnection(url,prefix){
  const parsed=new URL(url);if(!['ws:','wss:'].includes(parsed.protocol)||parsed.username||parsed.password)throw Error('Use ws://host:9001 or wss://host:port');
  if(!prefix||/[#+\u0000-\u001f]/.test(prefix)||prefix.length>100)throw Error('Use a plain MQTT topic prefix');
}
export class MqttConnection {
 constructor({onState,onMessage,onLog}){Object.assign(this,{onState,onMessage,onLog});this.generation=0;this.wanted=false;this.ready=false;this.retry=1000;}
 connect(url,prefix){
  validateConnection(url,prefix);
  this.disconnect();this.url=url;this.prefix=prefix.replace(/\/$/,'');this.wanted=true;this.retry=1000;this.open();
 }
 disconnect(){
  this.wanted=false;this.generation++;clearTimeout(this.retryTimer);clearTimeout(this.deadline);clearInterval(this.pinger);
  this.ready=false;if(this.socket){this.socket.onclose=null;this.socket.close();this.socket=null;}this.onState('disconnected');
 }
 open(){
  const generation=++this.generation;this.ready=false;this.pending=new Uint8Array();this.pingAt=0;
  this.onState('connecting');
  const ws=this.socket=new WebSocket(this.url,['mqtt']);ws.binaryType='arraybuffer';
  const fail=()=>{if(generation===this.generation)ws.close();};
  this.deadline=setTimeout(()=>{this.onLog('Connection timed out','warning');fail();},12000);
  ws.onopen=()=>{
   if(generation!==this.generation)return;
   const id='rover-web-'+Math.random().toString(16).slice(2,10);
   ws.send(packet(0x10,join(mqttString('MQTT'),[4,2,0,30],mqttString(id))));
  };
  ws.onmessage=e=>{
   if(generation!==this.generation)return;
   try{
    if(!(e.data instanceof ArrayBuffer))throw Error('Expected binary MQTT');
    const buffer=join(this.pending,new Uint8Array(e.data));
    if(buffer.length>131072)throw Error('Input buffer too large');
    const result=extractPackets(buffer);this.pending=result.rest;
    for(const frame of result.packets)this.handle(frame,ws);
   }catch(error){this.onLog(error.message,'warning');fail();}
  };
  ws.onerror=()=>{if(generation===this.generation)this.onLog('WebSocket unavailable. Check broker port 9001.','warning');};
  ws.onclose=()=>{
   if(generation!==this.generation)return;
   clearTimeout(this.deadline);clearInterval(this.pinger);this.ready=false;this.onState('disconnected');
   if(this.wanted){const delay=this.retry;this.retry=Math.min(delay*2,10000);this.retryTimer=setTimeout(()=>this.open(),delay);}
  };
 }
 handle(frame,ws){
  const type=frame.header>>4,b=frame.body;
  if(type===2){
   if(b.length!==2||b[0]!==0||b[1]!==0)throw Error('Broker rejected clean-session connection');
   const topics=['heartbeat','telemetry','command_result','diagnostics'].map(s=>join(mqttString(this.prefix+'/'+s),[0]));
   ws.send(packet(0x82,join([0,1],...topics)));
  }else if(type===9){
   if(this.ready)throw Error('Unexpected duplicate subscription acknowledgement');
   if(b.length!==6||b[0]!==0||b[1]!==1||!Array.from(b.slice(2)).every(v=>v===0))throw Error('Subscription rejected');
   clearTimeout(this.deadline);this.ready=true;this.retry=1000;this.onState('connected');
   this.pinger=setInterval(()=>{
    if(this.pingAt){this.onLog('Broker heartbeat timed out','warning');ws.close();return;}
    this.pingAt=performance.now();ws.send(packet(0xc0));
   },15000);
  }else if(type===3){if(this.ready)this.onMessage(decodePublish(frame));}
  else if(type===13){if(b.length!==0)throw Error('Invalid PINGRESP');this.pingAt=0;}
 }
 publish(command,id){
  if(!this.ready||!this.socket||this.socket.readyState!==WebSocket.OPEN)throw Error('Broker is disconnected');
  if(this.socket.bufferedAmount>4096)throw Error('Network busy; command was not queued');
  this.socket.send(packet(0x30,join(mqttString(this.prefix+'/command'),encodeCommand(command,id))));
 }
}
