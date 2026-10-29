// Optional local integration check. Isolated topic/port; never talks to the Pico.
import assert from 'node:assert/strict';
import {spawn} from 'node:child_process';
import {fileURLToPath} from 'node:url';
import {readFileSync} from 'node:fs';
import {MqttConnection,mqttString,packet} from './mqtt.mjs';
import {RobotState} from './state.mjs';
import {encodeDiagnostics} from './diagnostics.mjs';
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
     const binary=new Uint8Array(Buffer.from(readFileSync(new URL('../../host_tests/binary-golden-v3.hex',import.meta.url),'utf8').trim(),'hex'));
     const diagnostics=encodeDiagnostics({demo:true,uptime_ms:1000,rejected:1,out_of_range:1,busy:0,last_at_ms:900,section:3,field:5,reason:2,value:-101,min:-100,max:100});
     for(const [topic,value] of [['heartbeat',{schema:1,demo:true,seq:1}],['telemetry',binary],['diagnostics',diagnostics]]){
      const t=mqttString(prefix+'/'+topic),body=value instanceof Uint8Array?value:encode.encode(JSON.stringify(value));
      client.socket.send(packet(0x30,new Uint8Array([...t,...body])));
     }
    }
   },
   onMessage:message=>{
    try{
     const topic=message.topic.slice(prefix.length+1);model.receive(topic,message.payload,0,message.retained);seen.push(topic);
     if(seen.length===3){assert.equal(model.status(0),'online');assert.equal(model.telemetry.schema,3);assert.equal(model.telemetry.buddies[2].left_speed_mm_s,-10);assert.equal(model.telemetry.buddies[3].steering,-100);assert.equal(model.telemetry.buddies[5].event_seq,0xffffffff);assert.equal(model.diagnostics.device.value,-101);assert.equal(model.diagnostics.device.out_of_range,1);clearTimeout(timer);resolve();}
    }catch(error){clearTimeout(timer);reject(error);}
   }
  });
  client.connect('ws://127.0.0.1:19001',prefix);
 });
 console.log('PASS: real Mosquitto WebSocket CONNECT, SUBACK and heartbeat/telemetry/diagnostics roundtrip on isolated port 19001.');
}finally{client?.disconnect();broker.kill();}
