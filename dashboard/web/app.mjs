import {RobotState,validateCommand} from './state.mjs';
import {MqttConnection,validateConnection} from './mqtt.mjs';
const $=id=>document.getElementById(id),now=()=>performance.now()/1000;
let state=new RobotState(),preview=false,connecting=false,history=[],previewTimer=null,simRunning=true,simSpeed=30,simSeq=0,previousStatus='';
let prefix='group18/robot1',endpoint='ws://localhost:9001';
try{const saved=JSON.parse(localStorage.getItem('rover18.connection'));if(saved){prefix=saved.prefix||prefix;endpoint=saved.endpoint||endpoint;}}catch{}
function log(message,type='system'){
 const li=document.createElement('li'),time=document.createElement('time'),tag=document.createElement('span'),text=document.createElement('span');
 time.textContent=new Date().toLocaleTimeString('en-GB',{hour12:false});tag.className='event-type'+(type==='warning'?' warning':'');tag.textContent=type.toUpperCase();text.className='event-message';text.textContent=message;
 li.append(time,tag,text);$('events').prepend(li);while($('events').children.length>50)$('events').lastChild.remove();$('event-count').textContent=String($('events').children.length).padStart(2,'0');
}
const client=new MqttConnection({
 onState(status){connecting=status==='connecting';if(status==='connected'){state.disconnect();state.connected=true;history=[];log('Subscribed. Listening for the rover heartbeat.','broker');}else if(status==='disconnected'){if(state.connected&&!preview)log('Broker disconnected. Commands are disabled.','warning');state.disconnect();}render();},
 onLog:log,
 onMessage(message){
  if(preview||!message.topic.startsWith(prefix+'/'))return;
  receive(message.topic.slice(prefix.length+1),message.payload,now(),message.retained);
 }
});
function receive(topic,payload,time,retained=false){
 try{
  if(!state.receive(topic,payload,time,retained))return;
  if(topic==='telemetry'){history.push({time,speed:state.telemetry.speed_percent});if(history.length>180)history.shift();}
  if(topic==='command_result'){
   const result=state.result.status;
   $('command-feedback').textContent=preview?'Preview only · '+result:'Robot result: '+result+' · physical execution unconfirmed.';
   log((preview?'Preview':'Robot')+' result: '+result,'command');
  }
 }catch(error){log('Ignored message: '+error.message,'warning');}
 render();
}
function reset(){state=new RobotState();history=[];previousStatus='';$('command-feedback').textContent='Controls unlock after a fresh heartbeat.';}
function setPreview(value){
 clearInterval(previewTimer);client.disconnect();preview=value;reset();
 document.body.classList.toggle('preview',preview);
 $('preview-toggle').classList.toggle('active',preview);$('preview-toggle').textContent=preview?'◈  Exit preview':'◈  Explore preview';
 if(preview){
  state.connected=true;simRunning=true;simSpeed=30;simSeq=0;
  log('Preview mode. Generated readings only; no broker connection or robot commands.','preview');
  const sample=()=>{
   const t=now();if(simSeq%5===0)receive('heartbeat',JSON.stringify({schema:1,demo:true,seq:simSeq}),t);
   receive('telemetry',JSON.stringify({schema:1,demo:true,speed_percent:simRunning?Math.max(0,Math.min(100,simSpeed+Math.round(Math.sin(simSeq/3)*3))):0,distance_cm:82+Math.round(Math.sin(simSeq/5)*9)}),t);
   simSeq++;
  };
  sample();previewTimer=setInterval(sample,1000);
 }else log('Preview ended. Ready for a live broker connection.');
 render();
}
function send(command){
 try{
  validateCommand(command);
  if(state.status(now())!=='online')throw Error('Wait for a fresh rover heartbeat');
  if(preview){
   if(command==='START')simRunning=true;
   if(command==='STOP')simRunning=false;
   if(command.startsWith('SET_SPEED '))simSpeed=Number(command.slice(10));
   receive('command_result',JSON.stringify({schema:1,demo:true,status:'simulated'}),now());
  }else{client.publish(command);$('command-feedback').textContent='Sent to MQTT · waiting for a robot result. Not an execution confirmation.';}
  log((preview?'Preview only: ':'Sent: ')+command,'command');
 }catch(error){$('command-feedback').textContent=error.message;log(error.message,'warning');}
}
function render(){
 const t=now(),status=state.status(t),fresh=state.telemetryAt!==null&&t-state.telemetryAt<3&&status==='online';
 const names={disconnected:connecting?'Connecting to broker…':'Ready when you are',waiting:'Broker connected · waiting for rover',online:preview?'Preview running · sample data':'Rover heartbeat received',offline:'Rover heartbeat lost'};
 $('session-status').textContent=names[status];
 $('session-detail').textContent=preview?'Visual demo only. Your robot is not being controlled.':status==='online'?'Telemetry link is active.':status==='offline'?'No heartbeat for 15 seconds. Commands locked.':status==='waiting'?'The Pico has not sent a fresh heartbeat yet.':'Connect your broker to bring the rover online.';
 $('main-dot').className='status-dot '+(status==='online'?'online':status==='offline'?'offline':'');
 $('mode-badge').textContent=preview?'PREVIEW / SIMULATED':state.demo===true?'FIRMWARE / SIMULATED':state.demo===false?'LIVE / ROBOT DATA':'NO LIVE FEED';
 $('speed-value').textContent=fresh?state.telemetry.speed_percent:'—';$('distance-value').textContent=fresh?state.telemetry.distance_cm:'—';
 $('speed-note').textContent=fresh?(preview?'Generated preview readings':state.demo?'Sample firmware readings':'Fresh rover telemetry'):state.telemetry?'Telemetry stale · values hidden':'Awaiting telemetry';
 $('heartbeat-value').textContent=state.lastHeartbeat===null?'—':(t-state.lastHeartbeat).toFixed(1);
 $('heartbeat-note').textContent=status==='offline'?'15 s timeout reached':'5 s cadence / 15 s timeout';
 $('message-value').textContent=String(state.received).padStart(2,'0');
 $('rover-caption').textContent=preview?'Exploring a simulated rover.':status==='online'?'Receiving heartbeat from rover.':status==='offline'?'Signal lost. Waiting for recovery.':'Standing by for a signal.';
 for(const id of ['start','stop','set-speed'])$(id).disabled=status!=='online';
 $('speed-slider').disabled=status!=='online';
 $('endpoint-label').textContent=preview?'Preview · no broker':endpoint;$('prefix-label').textContent=prefix;
 const samples=history.filter(s=>t-s.time<=60);history=samples;let d='',last=null;
 for(const s of samples){const x=Math.max(0,600-(t-s.time)*10),y=118-s.speed*1.16;d+=(last&&s.time-last.time<3?'L':'M')+x.toFixed(1)+','+y.toFixed(1);last=s;}
 $('chart-line').setAttribute('d',d);
 // Disconnected runs remain separate; the line does not interpolate missing packets.
 $('chart-area').setAttribute('d','');
 $('chart-empty').hidden=samples.length>0;
 $('chart-freshness').textContent=fresh?(preview?'SIMULATED SAMPLES':'RECEIVING TELEMETRY'):samples.length?'LAST RECEIVED SAMPLES':'NO SAMPLES YET';
 $('clock').textContent=new Date().toLocaleTimeString('en-SG',{hour12:false,timeZone:'Asia/Singapore'})+' SGT';
 if(status!==previousStatus){if(status==='offline')log('Heartbeat timeout: 15 seconds without a fresh heartbeat.','warning');previousStatus=status;}
}
function settings(){$('broker-url').value=endpoint;$('topic-prefix').value=prefix;$('form-error').textContent='';$('connection-dialog').showModal();}
$('preview-toggle').onclick=()=>setPreview(!preview);
$('connection-open').onclick=settings;$('connection-edit').onclick=settings;
$('dialog-close').onclick=()=>$('connection-dialog').close();
$('disconnect').onclick=()=>{if(preview)setPreview(false);client.disconnect();reset();$('connection-dialog').close();log('Disconnected by operator.');render();};
$('connection-form').onsubmit=e=>{
 e.preventDefault();
 try{
  const url=$('broker-url').value.trim(),topic=$('topic-prefix').value.trim();
  validateConnection(url,topic);
  if(preview)setPreview(false);
  reset();client.connect(url,topic);endpoint=url;prefix=topic.endsWith('/')?topic.slice(0,-1):topic;
  try{localStorage.setItem('rover18.connection',JSON.stringify({endpoint,prefix}));}catch{}
  $('connection-dialog').close();log('Connecting to '+endpoint,'broker');render();
 }catch(error){$('form-error').textContent=error.message;}
};
$('speed-slider').oninput=e=>$('requested-speed').textContent=e.target.value;
$('start').onclick=()=>send('START');$('stop').onclick=()=>send('STOP');$('set-speed').onclick=()=>send('SET_SPEED '+$('speed-slider').value);
$('clear-log').onclick=()=>{$('events').replaceChildren();$('event-count').textContent='00';};
window.addEventListener('pagehide',()=>{client.disconnect();clearInterval(previewTimer);});
log('Dashboard ready. Connect your broker or explore a simulated preview.');render();setInterval(render,200);
