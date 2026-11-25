import {renderBarcodeStatus} from './barcode-status.mjs?v=barcode-retry-1';
import {robotEvents,lineSearchEvents,calibrationEvents} from './robot-events.mjs?v=barcode-digital-4';
import {renderDiagnostics,encodeDiagnostics} from './diagnostics.mjs?v=recorded-static-ram-1';
import {encodeLiveStatus} from './live-status.mjs?v=barcode-digital-4';
import {encodeTelemetry} from './binary.mjs?v=barcode-digital-4';
import {encodeCommand,decodeCommand} from './commands.mjs?v=barcode-digital-4';
import {buddyView,motionView} from './telemetry-view.mjs?v=route-slots-1';
import {renderSensorHighlights} from './sensor-panels.mjs?v=route-slots-1';
import {renderRoverOverview} from './rover-overview.mjs?v=overview-1';
import {createHumpLog,updateHumpLog,renderHumpLog} from './hump-log.mjs?v=humps-1';
let humpLog=createHumpLog();
import {RobotState,validateCommand} from './state.mjs?v=compact-control-1';
import {MqttConnection,validateConnection} from './mqtt.mjs?v=combined-1';
import {ASSIGNABLE_ACTIONS,editableRouteMap,charIndex,encodeMapSet,encodeMapShow,decodeMapCommand,mapToHex,sameMap,assignDirection,lettersForDirection} from './route-map.mjs?v=route-no-straight-1';
const $=id=>document.getElementById(id),now=()=>performance.now()/1000;
let state=new RobotState(),preview=false,connecting=false,previewTimer=null,simRunning=false,simSeq=0,simDistance=0,simHumps=0,commandSeq=0,previousStatus='';
let buildEvidence=null;
fetch('./build-memory.json',{cache:'no-store'}).then(r=>r.ok?r.json():null).then(v=>{buildEvidence=v;render();}).catch(()=>{});
let prefix='group18/robot1',endpoint='ws://localhost:9001';
const diagnosticLimit=12000,diagnosticChunks=[],diagnosticDecoder=new TextDecoder();
let diagnosticCount=0,diagnosticLast=null;
function receiveDiagnostic(payload){
 const chunk=diagnosticDecoder.decode(payload),split=chunk.indexOf('\n');
 if(split<0)return;
 const match=/^fw=(\S+) seq=(\d+) uptime_ms=(\d+) dropped_bytes=(\d+)$/.exec(chunk.slice(0,split));
 if(!match)return;
 const record={received_at:new Date().toISOString(),firmware:match[1],sequence:Number(match[2]),uptime_ms:Number(match[3]),pico_dropped_bytes:Number(match[4]),text:chunk.slice(split+1)};
 if(diagnosticLast){
  if(record.uptime_ms<diagnosticLast.uptime_ms||record.sequence<diagnosticLast.sequence)record.reboot=true;
  else if(record.sequence!==diagnosticLast.sequence+1)record.sequence_gap=record.sequence-diagnosticLast.sequence-1;
 }
 diagnosticLast=record;
 diagnosticChunks[diagnosticCount%diagnosticLimit]=record;
 diagnosticCount++;
 $('diagnostic-log-status').textContent=`${diagnosticCount} batches received · ${record.pico_dropped_bytes} bytes dropped by Pico`;
 $('download-diagnostics').disabled=false;
}
function downloadDiagnostics(){
 const first=Math.max(0,diagnosticCount-diagnosticLimit);
 const lines=[JSON.stringify({topic:prefix+'/log',exported_at:new Date().toISOString(),browser_batches_discarded:first})+'\n'];
 for(let i=first;i<diagnosticCount;i++)lines.push(JSON.stringify(diagnosticChunks[i%diagnosticLimit])+'\n');
 const url=URL.createObjectURL(new Blob(lines,{type:'application/x-ndjson'}));
 const link=document.createElement('a');link.href=url;
 link.download='group18-serial-'+new Date().toISOString().replaceAll(':','-')+'.ndjson';
 link.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
}
$('download-diagnostics').onclick=downloadDiagnostics;
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
  if(message.topic===prefix+'/log'){
   if(!message.retained)receiveDiagnostic(message.payload);
   return;
  }
  receive(message.topic.slice(prefix.length+1),message.payload,now(),message.retained);
 }
});
function receive(topic,payload,time,retained=false){
 try{
  const previousTelemetry=state.telemetry,previousLiveStatus=state.liveStatus;
  if(!state.receive(topic,payload,time,retained))return;
  if(!preview&&!pendingRoute&&
     (topic==='heartbeat'&&!state.routeMap||
      topic==='live_status'&&state.liveStatus.line.map_revision!==undefined&&
      state.liveStatus.line.map_revision!==state.routeMapRevision)&&
     state.status(time)==='online'&&time>=routeReadAt){
   routeReadAt=time+1;sendRoutes(true);
  }
  if(topic==='live_status')for(const event of [...lineSearchEvents(previousLiveStatus,state.liveStatus),...calibrationEvents(previousLiveStatus,state.liveStatus)])log(event.text,event.type);
  if(topic==='telemetry')for(const event of robotEvents(previousTelemetry,state.telemetry))log(event.text,event.type);
  if(topic==='telemetry'){const r=updateHumpLog(humpLog,state.telemetry);humpLog=r.log;for(const event of r.events)log(event.text,event.type);}
  if(topic==='command_result'&&state.result.map!==undefined){
   if(pendingRoute===state.result.command_id){
    pendingRoute=0;
    if(pendingMapSet)routeDirty=false;
   }
   if(!routeDirty){routeDraft=editableRouteMap(state.routeMap);drawDirectionSlots();}
   $('route-feedback').textContent=preview?'Preview map confirmed.':'Rover confirmed the current map.';
  }else if(topic==='command_result'&&state.result.status==='rejected'&&pendingRoute===state.result.command_id){
   pendingRoute=0;
   $('route-feedback').textContent='Rover rejected map #'+state.result.command_id+'. Nothing changed on the rover.';
   log('Robot rejected route map #'+state.result.command_id,'warning');
  }else if(topic==='command_result'){
   const result=state.result.status;
   $('command-feedback').textContent=preview?'Preview only · simulated.':({queued:'Request queued · movement not confirmed.',rejected:'Request rejected.',queue_full:'Robot busy · request not queued.',simulated:'Simulated response · no movement confirmed.'}[result]??'Robot replied.');
   log((preview?'Preview':'Robot')+' command #'+state.result.command_id+': '+result+(result==='queued'?' · request queued, movement not confirmed.':''),'command');
  }
 }catch(error){log('Ignored message: '+error.message,'warning');}
 render();
}
function reset(){state=new RobotState();humpLog=createHumpLog();pendingRoute=0;routeDraft={};routeDirty=false;drawDirectionSlots();if($('route-feedback'))$('route-feedback').textContent='Waiting for rover connection.';previousStatus='';$('command-feedback').textContent='Waiting for rover connection.';}
function setPreview(value){
 clearInterval(previewTimer);client.disconnect();preview=value;reset();
 document.body.classList.toggle('preview',preview);
 $('preview-toggle').classList.toggle('active',preview);$('preview-toggle').textContent=preview?'◈  Exit preview':'◈  Explore preview';
 if(preview){
  state.connected=true;simRunning=false;simSeq=0;simDistance=0;previewRouteMap={};
  log('Preview mode. Generated readings only; no broker connection or robot commands.','preview');
  const sample=()=>{
   const t=now(),ms=Math.floor(t*1000)>>>0;
   if(simSeq%25===0)receive('heartbeat',JSON.stringify({schema:1,demo:true,seq:simSeq}),t);
   const raw=new Uint8Array(17);raw[0]=0x21;const w=new DataView(raw.buffer);
   w.setUint32(1,(0x80000000|1|(3<<20))>>>0,true);
   w.setUint32(5,((simSeq%32)<<27)>>>0,true);
   if(simRunning&&simSeq%40===39)simHumps++;   /* preview: a sample hump every 8 s */
   {const mm=8+simHumps%5,unc=4+simHumps%3,pitch=3+simHumps%2,cm=18+simHumps%4;
    w.setUint32(13,(mm|(unc<<8)|(pitch<<14)|(cm<<20)|(1<<28)|((simHumps%8)<<29))>>>0,true);}
   const distance=82+Math.round(Math.sin(simSeq/5)*9);
   if(simRunning)simDistance+=6;
   receive('telemetry',encodeTelemetry({demo:true,seq:simSeq,uptime_ms:ms,fault_mask:0,
    base_valid:false,speed_percent:0,distance_cm:0,
    buddies:{2:{updated_ms:ms,target_speed_mm_s:simRunning?30:0,left_speed_mm_s:simRunning?30:0,right_speed_mm_s:simRunning?30:0,distance_mm:simDistance,target_speed_valid:1,left_speed_valid:1,right_speed_valid:1,distance_valid:1,odometry_valid:1,motion_active:simRunning?1:0,sequence:simSeq%256,uptime_ms:ms,system_mode:simRunning?2:5,mission_state:simRunning?1:0},3:{updated_ms:ms,line_state:Math.sin(simSeq/5)>.1?2:Math.sin(simSeq/5)<-.1?1:0,steering:Math.round(Math.sin(simSeq/5)*30),adc_valid:1,adc:1800,calibration_low:1400,calibration_high:2000,calibrated:1},
     4:{updated_ms:ms,raw},5:{updated_ms:ms,state:simRunning?(simSeq%60<20?9:simSeq%60<30?1:0):0,front_status:0,front_mm:distance*10,front_at_ms:ms,last_status:0,last_mm:distance*10,last_at_ms:ms,servo_valid:1,servo_deg:90}}
   }),t);
   receive('live_status',encodeLiveStatus({demo:true,uptime_ms:ms,present:5,
    motion:{at_ms:ms,left_target:simRunning?30:0,right_target:simRunning?30:0,targets_valid:true,qualified:true,encoder_left_counts:1271,encoder_right_counts:1264,wheel_diameter_mm:65,searching:simRunning&&simSeq%60>=50,searches:Math.floor(simSeq/60),accepted:simSeq},
    line:{at_ms:ms,calibration_state:2,paused:false},
    bypass:{at_ms:ms,fast:false,active:simRunning&&simSeq%60<20,segment:simRunning?Math.floor(simSeq%20/3):0,segments:7,side:1}}),t);
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
  }else{client.publish(command,id);$('command-feedback').textContent='Request sent · waiting for robot reply.';}
  log((preview?'Preview only: ':'Sent: ')+command+' · command #'+id,'command');
 }catch(error){$('command-feedback').textContent=error.message;log(error.message,'warning');}
}
// ---- Barcode routes (Buddy 3) -------------------------------------------
let routeDraft={},pendingRoute=0,routeDeadline=0,routeReadAt=0,routeDirty=false,pendingMapSet=false,previewRouteMap={};
const directionInputs=new Map();
const confirmedLetters=new Map();
for(const direction of ASSIGNABLE_ACTIONS){
 const row=document.createElement('div'),name=document.createElement('dt'),mapped=document.createElement('dd');
 name.textContent=direction.icon+' '+direction.name;
 mapped.textContent='—';
 row.append(name,mapped);$('route-onboard').append(row);confirmedLetters.set(direction.value,mapped);
 const label=document.createElement('label'),input=document.createElement('input');
 label.className='route-letter-label';
 label.append(direction.icon+' '+direction.name);
 input.maxLength=1;input.spellcheck=false;input.setAttribute('aria-describedby','route-error');
 input.oninput=()=>{
  const letter=input.value.trim().toUpperCase();
  if(letter&&charIndex(letter)<0){
   $('route-error').textContent='Use one letter A–Z or digit 0–9.';
   drawDirectionSlots();
   return;
  }
  $('route-error').textContent='';
  routeDraft=assignDirection(routeDraft,direction.value,letter);
  routeDirty=true;drawDirectionSlots();renderRouteSync();
 };
 label.append(input);$('route-form').append(label);directionInputs.set(direction.value,input);
}
function drawDirectionSlots(){
 for(const [direction,input] of directionInputs){
  input.value=lettersForDirection(routeDraft,direction)[0]??'';
 }
}
function renderRouteSync(){
 if(pendingRoute&&now()>=routeDeadline){
  pendingRoute=0;$('route-feedback').textContent='No map response. Try again.';
 }
 const sync=$('route-sync'),revision=state.liveStatus?.line.map_revision;
 const online=state.status(now())==='online';
 const onboard=online&&(revision===undefined||revision===state.routeMapRevision)?state.routeMap:null;
 const synced=onboard&&sameMap(onboard,routeDraft);
 const oldStraight=onboard&&!synced&&sameMap(editableRouteMap(onboard),routeDraft);
 for(const [direction,label] of confirmedLetters){
  label.textContent=onboard?lettersForDirection(onboard,direction).join(', ')||'Unassigned':'—';
 }
 sync.textContent=pendingRoute?'Waiting for rover':!online?'Offline':synced?'In sync':oldStraight?'Send to clear old Straight assignment':onboard?'Changes not sent':'Reading map';
 sync.className='route-sync'+(synced?' synced':'');
 $('route-send').disabled=!online||!!pendingRoute;
 $('route-read').disabled=!online||!!pendingRoute;
 for(const input of directionInputs.values())input.disabled=!!pendingRoute&&pendingMapSet;
 $('route-clear').disabled=!!pendingRoute&&pendingMapSet;
}
function sendRoutes(show){
 try{
  if(state.status(now())!=='online')throw Error('Wait for a fresh rover heartbeat');
  const id=commandSeq=(commandSeq+1)>>>0||1,bytes=show?encodeMapShow(id):encodeMapSet(routeDraft,id);
  pendingRoute=id;routeDeadline=now()+4;pendingMapSet=!show;
  if(preview){
   const cmd=decodeMapCommand(bytes);if(cmd.kind==='SET')previewRouteMap=cmd.map;
   receive('command_result',JSON.stringify({schema:1,demo:true,status:'applied',command_id:id,map:mapToHex(previewRouteMap)}),now());
  }else{
   client.publishBytes(bytes);
   $('route-feedback').textContent=show?'Reading rover map…':'Sending the full map…';
  }
 }catch(error){pendingRoute=0;$('route-feedback').textContent=error.message;log(error.message,'warning');}
 renderRouteSync();
}
$('route-form').onsubmit=e=>{e.preventDefault();if(!$('route-send').disabled)sendRoutes(false);};
$('route-send').onclick=()=>sendRoutes(false);
$('route-read').onclick=()=>{routeDirty=false;sendRoutes(true);};
$('route-clear').onclick=()=>{routeDraft={};routeDirty=true;drawDirectionSlots();renderRouteSync();};
function render(){
 const t=now(),status=state.status(t),view=state.telemetryView(t),fresh=view.fresh;
 // Reader updates independently of the other sensor cards and telemetry.
 renderBarcodeStatus({...view.values,demo:state.demo,live_status:state.liveStatus,live_status_elapsed_ms:state.liveStatusAt===null?Infinity:(t-state.liveStatusAt)*1000},{stale:status!=='online'&&!!state.liveStatus,mapRevision:state.routeMapRevision});
 if(['Waiting for rover connection.','Ready · choose Start rover or Stop.'].includes($('command-feedback').textContent))$('command-feedback').textContent=status==='online'?'Ready · choose Start rover or Stop.':'Waiting for rover connection.';
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
 renderBuddies(view,t);
 renderHumpLog($('hump-log-4'),humpLog);
 renderRouteSync();
 renderDiagnostics(state,t,buildEvidence);
 $('clock').textContent=new Date().toLocaleTimeString('en-SG',{hour12:false,timeZone:'Asia/Singapore'})+' SGT';
 if(status!==previousStatus){if(status==='offline')log('Heartbeat timeout: 15 seconds without a fresh heartbeat.','warning');if(status==='online'&&!preview)log(previousStatus==='offline'?'Rover heartbeat recovered.':'Fresh rover heartbeat received.','broker');previousStatus=status;}
}
function renderBuddies(view,t){
 const packet=view.values?{...view.values,live_status:state.liveStatus,live_status_elapsed_ms:state.liveStatusAt===null?Infinity:(t-state.liveStatusAt)*1000}:null,elapsed=view.age===null?0:view.age*1000;
 renderRoverOverview(packet,{fresh:view.fresh,elapsed,mapRevision:state.routeMapRevision});
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
$('connection-open').onclick=settings;
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
