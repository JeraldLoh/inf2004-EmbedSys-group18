import {renderDiagnostics,encodeDiagnostics} from './diagnostics.mjs';
import {encodeTelemetry} from './binary.mjs';
import {encodeCommand,decodeCommand} from './commands.mjs';
import {buddyView,motionSpeed,motionView} from './telemetry-view.mjs';
import {renderSensorHighlights} from './sensor-panels.mjs';
import {RobotState,validateCommand} from './state.mjs';
import {MqttConnection,validateConnection} from './mqtt.mjs';
const $=id=>document.getElementById(id),now=()=>performance.now()/1000;
let state=new RobotState(),preview=false,connecting=false,history=[],previewTimer=null,simRunning=false,simSeq=0,simDistance=0,commandSeq=0,previousStatus='',lastUpdateText='',lastMotorStamp='';
let buildEvidence=null;
fetch('./build-memory.json',{cache:'no-store'}).then(r=>r.ok?r.json():null).then(v=>{buildEvidence=v;render();}).catch(()=>{});
let prefix='group18/robot1',endpoint='ws://localhost:9001';
try{const saved=JSON.parse(localStorage.getItem('rover18.connection'));if(saved){prefix=saved.prefix||prefix;endpoint=saved.endpoint||endpoint;}}catch{}
function log(message,type='system'){
 const li=document.createElement('li'),time=document.createElement('time'),tag=document.createElement('span'),text=document.createElement('span');
 time.textContent=new Date().toLocaleTimeString('en-GB',{hour12:false});tag.className='event-type'+(type==='warning'?' warning':'');tag.textContent=type.toUpperCase();text.className='event-message';text.textContent=message;
 li.append(time,tag,text);$('events').prepend(li);while($('events').children.length>50)$('events').lastChild.remove();$('event-count').textContent=String($('events').children.length).padStart(2,'0');
}
const client=new MqttConnection({
 onState(status){connecting=status==='connecting';if(status==='connected'){state.connectionRestored();log('Subscribed. Listening for the rover heartbeat.','broker');}else if(status==='disconnected'){if(state.connected&&!preview)log('Broker disconnected. Commands are disabled.','warning');state.connectionLost();}render();},
 onLog:log,
 onMessage(message){
  if(preview||!message.topic.startsWith(prefix+'/'))return;
  receive(message.topic.slice(prefix.length+1),message.payload,now(),message.retained);
 }
});
function receive(topic,payload,time,retained=false){
 try{
  if(!state.receive(topic,payload,time,retained))return;
  if(topic==='telemetry'){lastUpdateText=new Date().toLocaleTimeString('en-SG',{hour12:false,timeZone:'Asia/Singapore'})+' SGT';const m=motionView(state.telemetry),b=state.telemetry.buddies?.[2],stamp=b?`${b.updated_ms}:${b.sequence}`:'';if(m.fresh&&stamp!==lastMotorStamp){history.push({time:time-m.age/1000,speed:m.speed});lastMotorStamp=stamp;}if(history.length>305)history.shift();}
  if(topic==='command_result'){
   const result=state.result.status;
   $('command-feedback').textContent=preview?'Preview only · '+result:'Robot result: '+result+' · physical execution unconfirmed.';
   log((preview?'Preview':'Robot')+' result: '+result,'command');
  }
 }catch(error){log('Ignored message: '+error.message,'warning');}
 render();
}
function reset(){state=new RobotState();history=[];previousStatus='';lastUpdateText='';lastMotorStamp='';$('command-feedback').textContent='Controls unlock after a fresh heartbeat.';}
function setPreview(value){
 clearInterval(previewTimer);client.disconnect();preview=value;reset();
 document.body.classList.toggle('preview',preview);
 $('preview-toggle').classList.toggle('active',preview);$('preview-toggle').textContent=preview?'◈  Exit preview':'◈  Explore preview';
 if(preview){
  state.connected=true;simRunning=false;simSeq=0;simDistance=0;
  log('Preview mode. Generated readings only; no broker connection or robot commands.','preview');
  const sample=()=>{
   const t=now(),ms=Math.floor(t*1000)>>>0;
   if(simSeq%25===0)receive('heartbeat',JSON.stringify({schema:1,demo:true,seq:simSeq}),t);
   const raw=new Uint8Array(17);raw[0]=0x21;const w=new DataView(raw.buffer);
   w.setUint32(1,(0x80000000|1|(3<<20))>>>0,true);
   w.setUint32(5,((simSeq%32)<<27)>>>0,true);
   const distance=82+Math.round(Math.sin(simSeq/5)*9);
   if(simRunning)simDistance+=6;
   receive('telemetry',encodeTelemetry({demo:true,seq:simSeq,uptime_ms:ms,fault_mask:0,
    base_valid:false,speed_percent:0,distance_cm:0,
    buddies:{2:{updated_ms:ms,target_speed_mm_s:simRunning?30:0,left_speed_mm_s:simRunning?30:0,right_speed_mm_s:simRunning?30:0,distance_mm:simDistance,target_speed_valid:1,left_speed_valid:1,right_speed_valid:1,distance_valid:1,odometry_valid:1,motion_active:simRunning?1:0,sequence:simSeq%256,uptime_ms:ms,system_mode:simRunning?2:5,mission_state:simRunning?1:0},3:{updated_ms:ms,line_state:Math.sin(simSeq/5)>.1?2:Math.sin(simSeq/5)<-.1?1:0,steering:Math.round(Math.sin(simSeq/5)*30),adc_valid:1,adc:1800,calibration_low:1400,calibration_high:2000,calibrated:1},
     4:{updated_ms:ms,raw},5:{updated_ms:ms,state:0,front_status:0,front_mm:distance*10,front_at_ms:ms,last_status:0,last_mm:distance*10,last_at_ms:ms,servo_valid:1,servo_deg:90}}
   }),t);
   if(simSeq%5===0)receive('diagnostics',encodeDiagnostics({demo:true,uptime_ms:ms,rejected:0,out_of_range:0,busy:0,last_at_ms:0,section:0,field:0,reason:0,value:0,min:0,max:0}),t);
   simSeq++;
  };
  sample();previewTimer=setInterval(sample,200);
 }else log('Preview ended. Ready for a live broker connection.');
 render();
}
function send(command){
 try{
  validateCommand(command);
  if(state.status(now())!=='online')throw Error('Wait for a fresh rover heartbeat');
  const id=commandSeq=(commandSeq+1)>>>0||1;
  if(preview){
   const decoded=decodeCommand(encodeCommand(command,id));
   command=decoded.command;
   if(command==='START')simRunning=true;
   if(command==='STOP')simRunning=false;
   receive('command_result',JSON.stringify({schema:1,demo:true,status:'simulated',command_id:id}),now());
  }else{client.publish(command,id);$('command-feedback').textContent='Sent to MQTT · waiting for a robot result. Not an execution confirmation.';}
  log((preview?'Preview only: ':'Sent: ')+command,'command');
 }catch(error){$('command-feedback').textContent=error.message;log(error.message,'warning');}
}
function render(){
 const t=now(),status=state.status(t),view=state.telemetryView(t),fresh=view.fresh;
 const names={disconnected:connecting?'Connecting to broker…':view.values?'Communication Unavailable':'Ready when you are',waiting:'Broker connected · waiting for rover',online:preview?'Preview running · sample data':'Rover heartbeat received',offline:'Communication Unavailable'};
 $('session-status').textContent=names[status];
 $('session-detail').textContent=preview?'Visual demo only. Your robot is not being controlled.':status==='online'?'Telemetry link is active.':status==='offline'?'No heartbeat for 15 seconds. Last readings are stale. Commands locked.':status==='waiting'?'The Pico has not sent a fresh heartbeat yet.':view.values?'Broker unavailable. Last readings are stale. Commands locked.':'Connect your broker to bring the rover online.';
 $('main-dot').className='status-dot '+(status==='online'?'online':status==='offline'?'offline':'');
 $('mode-badge').textContent=preview?'PREVIEW / SIMULATED':state.demo===true?'FIRMWARE / SIMULATED':state.demo===false?'LIVE / ROBOT DATA':'NO LIVE FEED';
 const motor=motionView(view.values,view.age===null?0:view.age*1000),speed=motor.speed,sonar=view.values?.buddies?.[5];
 $('speed-value').textContent=speed===null?'—':String(speed);
 $('distance-value').textContent=sonar?.front_status===0?(sonar.front_mm/10).toFixed(1):'—';
 $('speed-note').textContent=speed===null?'Awaiting valid wheel-speed readings':(fresh&&motor.fresh?'':'STALE · ')+(preview||state.demo?'Simulated wheel feedback':'Mean left/right wheel feedback')+' · '+(motor.age===null?'unknown sample age':(motor.age/1000).toFixed(1)+' s sample age');
 $('distance-note').textContent=sonar?.front_status===0?(fresh?'':'STALE · ')+'Front at scan-start heading · see reading age below':'Awaiting valid front distance';
 for(const id of ['speed-value','distance-value'])$(id).closest('.metric').classList.toggle('stale',!!view.values&&(!fresh||(id==='speed-value'&&!motor.fresh)));
 $('heartbeat-value').textContent=state.lastHeartbeat===null?'—':(t-state.lastHeartbeat).toFixed(1);
 $('heartbeat-note').textContent=status==='offline'?'15 s timeout reached':'5 s cadence / 15 s timeout';
 $('message-value').textContent=String(state.received).padStart(2,'0');
 $('rover-caption').textContent=preview?'Exploring a simulated rover.':status==='online'?'Receiving heartbeat from rover.':status==='offline'?'Signal lost. Waiting for recovery.':'Standing by for a signal.';
 for(const id of ['start','stop'])$(id).disabled=status!=='online';
 $('endpoint-label').textContent=preview?'Preview · no broker':endpoint;$('prefix-label').textContent=prefix;
 const samples=history.filter(s=>t-s.time<=60);history=samples;let d='',last=null;
 const max=Math.ceil(Math.max(50,...samples.map(s=>s.speed))/50)*50,min=Math.floor(Math.min(0,...samples.map(s=>s.speed))/50)*50;
 $('chart-max').textContent=max;$('chart-mid').textContent=(max+min)/2;$('chart-min').textContent=min;
 for(const s of samples){const x=Math.max(0,600-(t-s.time)*10),y=118-(s.speed-min)/(max-min)*116;d+=(last&&s.time-last.time<3?'L':'M')+x.toFixed(1)+','+y.toFixed(1);last=s;}
 $('chart-line').setAttribute('d',d);
 // Disconnected runs remain separate; the line does not interpolate missing packets.
 $('chart-area').setAttribute('d','');
 $('chart-empty').hidden=samples.length>0;
 $('chart-freshness').textContent=fresh&&motor.fresh?(preview?'SIMULATED SAMPLES':'RECEIVING TELEMETRY'):samples.length?'LAST RECEIVED SAMPLES':'NO SAMPLES YET';
 renderBuddies(view,t);
 renderDiagnostics(state,t,buildEvidence);
 $('clock').textContent=new Date().toLocaleTimeString('en-SG',{hour12:false,timeZone:'Asia/Singapore'})+' SGT';
 if(status!==previousStatus){if(status==='offline')log('Heartbeat timeout: 15 seconds without a fresh heartbeat.','warning');previousStatus=status;}
}
function renderBuddies(view,t){
 const packet=view.values,elapsed=view.age===null?0:view.age*1000;
 const count=[2,3,4,5].filter(id=>packet?.buddies?.[id]).length;
 $('binary-summary').textContent=count?`${count} subsystems · ${packet.demo?'sample readings':view.fresh?'receiving data':'last received readings'}`:'Waiting for sensor readings';
 for(const id of [2,3,4,5]){
  const v=buddyView(packet,id,elapsed),host=$('buddy-'+id),status=$('buddy-status-'+id);
  status.textContent=v.available?(!view.fresh?'STALE LINK · ':'')+v.label:v.label;
  host.classList.toggle('stale',v.available&&(!!v.stale||!view.fresh));
  renderSensorHighlights(id,packet,v,view.fresh,elapsed);
  const list=$('buddy-fields-'+id);list.replaceChildren();
  for(const [name,value] of v.rows){const dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent=name;dd.textContent=String(value??'Unavailable');list.append(dt,dd);}
  if(!v.available){const dd=document.createElement('dd');dd.textContent='No real snapshot connected. Preview uses generated sample values.';list.append(dd);}
  $('buddy-raw-'+id).textContent=v.raw?JSON.stringify(v.raw,null,2):'Unavailable';
 }
}
function settings(){$('broker-url').value=endpoint;$('topic-prefix').value=prefix;$('form-error').textContent='';$('connection-dialog').showModal();}
$('preview-toggle').onclick=()=>setPreview(!preview);
$('sensor-preview').onclick=()=>setPreview(true);
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
$('start').onclick=()=>send('START');$('stop').onclick=()=>send('STOP');
$('clear-log').onclick=()=>{$('events').replaceChildren();$('event-count').textContent='00';};
window.addEventListener('pagehide',()=>{client.disconnect();clearInterval(previewTimer);});
log('Dashboard ready. Connect your broker or explore a simulated preview.');render();setInterval(render,200);
