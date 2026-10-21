// Optional local integration check. Isolated topic/port; never talks to the Pico.
import assert from 'node:assert/strict';
import {spawn} from 'node:child_process';
import {fileURLToPath} from 'node:url';
import {MqttConnection,mqttString,packet} from './mqtt.mjs';
import {RobotState} from './state.mjs';
const broker=spawn(process.env.MOSQUITTO_EXE||'C:\\Program Files\\mosquitto\\mosquitto.exe',['-c',fileURLToPath(new URL('./integration-broker.conf',import.meta.url)),'-v'],{windowsHide:true});
let client;
try{
 await new Promise((resolve,reject)=>{
  const timer=setTimeout(()=>reject(Error('Test broker startup timed out')),5000);
  broker.once('error',error=>{clearTimeout(timer);reject(error);});
  broker.once('exit',code=>{clearTimeout(timer);reject(Error('Test broker exited '+code));});
  broker.stderr.on('data',data=>{if(data.toString().includes('running')){clearTimeout(timer);resolve();}});
 });
 const model=new RobotState(),seen=[];
 await new Promise((resolve,reject)=>{
  const timer=setTimeout(()=>reject(Error('MQTT roundtrip timed out')),7000);
  const prefix='group18/website-integration-test';
  client=new MqttConnection({
   onLog:(text,type)=>{if(type==='warning'){clearTimeout(timer);reject(Error(text));}},
   onState:status=>{
    if(status==='connected'){
     model.connected=true;
     const encode=new TextEncoder();
     for(const [topic,value] of [['heartbeat',{schema:1,demo:true,seq:1}],['telemetry',{schema:1,demo:true,speed_percent:42,distance_cm:85}]]){
      const t=mqttString(prefix+'/'+topic),body=encode.encode(JSON.stringify(value));
      client.socket.send(packet(0x30,new Uint8Array([...t,...body])));
     }
    }
   },
   onMessage:message=>{
    try{
     const topic=message.topic.slice(prefix.length+1);model.receive(topic,message.payload,0,message.retained);seen.push(topic);
     if(seen.length===2){assert.equal(model.status(0),'online');assert.equal(model.telemetry.speed_percent,42);clearTimeout(timer);resolve();}
    }catch(error){clearTimeout(timer);reject(error);}
   }
  });
  client.connect('ws://127.0.0.1:19001',prefix);
 });
 console.log('PASS: real Mosquitto WebSocket CONNECT, SUBACK and heartbeat/telemetry roundtrip on isolated port 19001.');
}finally{client?.disconnect();broker.kill();}
