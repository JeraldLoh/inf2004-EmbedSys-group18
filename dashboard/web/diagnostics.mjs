import {fields} from './binary-fields.mjs';
const bad=()=>{throw Error('Invalid diagnostics packet');};
export function validationLabel(d){
 if(!d||!d.reason)return 'No rejected field';
 const name=d.section===1?['snapshot','base_valid','speed_percent','distance_cm','fault_mask','present'][d.field]:d.section===4?'frame_header':fields[d.section]?.[d.field-1]?.[0];
 return `${d.section===1?'Buddy 1':`Buddy ${d.section}`} · ${name??'unknown field'}`;
}
export function decodeDiagnostics(bytes){
 if(!(bytes instanceof Uint8Array)||bytes.length!==52||bytes[0]!==0xb1||bytes[1]!==2||bytes[2]!==2||bytes[3]>1||bytes[27]!==0||bytes[26]>3)bad();
 const v=new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength),signed=at=>{
  const n=Number(v.getBigInt64(at,true));if(!Number.isSafeInteger(n))bad();return n;
 };
 const d={demo:!!bytes[3],uptime_ms:v.getUint32(4,true),rejected:v.getUint32(8,true),out_of_range:v.getUint32(12,true),busy:v.getUint32(16,true),last_at_ms:v.getUint32(20,true),section:bytes[24],field:bytes[25],reason:bytes[26],value:signed(28),min:signed(36),max:signed(44)};
 if(d.min>d.max)bad();
 if(d.reason===0){if(d.section||d.field||d.value||d.min||d.max)bad();}
 else if(d.reason===1){if(d.section!==1||d.field!==0||d.value||d.min||d.max)bad();}
 else if(d.reason===3){if(d.section!==4||d.field!==1||d.min!==33||d.max!==33||d.value===33||d.value<0||d.value>255)bad();}
 else{
  const base=[null,['base_valid',1,0,1],['speed_percent',7,0,100],['distance_cm',17,0,100000],['fault_mask',20,0,1048575],['present',4,0,15]];
  const field=d.section===1?base[d.field]:fields[d.section]?.[d.field-1];
  if(!field||d.min!==field[2]||d.max!==field[3]||(d.value>=d.min&&d.value<=d.max))bad();
 }
 return d;
}
export function encodeDiagnostics(d){
 const bytes=new Uint8Array(52),v=new DataView(bytes.buffer);bytes.set([0xb1,2,2,d.demo?1:0]);
 for(const key of ['section','field','reason'])if(!Number.isInteger(d[key])||d[key]<0||d[key]>255)bad();
 for(const [at,key] of [[4,'uptime_ms'],[8,'rejected'],[12,'out_of_range'],[16,'busy'],[20,'last_at_ms']]){
  const n=d[key];if(!Number.isInteger(n)||n<0||n>0xffffffff)bad();v.setUint32(at,n,true);
 }
 bytes[24]=d.section;bytes[25]=d.field;bytes[26]=d.reason;
 for(const [at,key] of [[28,'value'],[36,'min'],[44,'max']]){if(!Number.isSafeInteger(d[key]))bad();v.setBigInt64(at,BigInt(d[key]),true);}
 decodeDiagnostics(bytes);return bytes;
}
export function newDiagnostics(){return {packetBytes:null,rxRejected:0,lastRxError:null,device:null,deviceAt:null};}
export function equivalentJsonBytes(packet){
 if(!packet)return null;
 const buddies={};for(const [id,s] of Object.entries(packet.buddies??{}))buddies[id]=id==='4'?{updated_ms:s.updated_ms,raw:s.raw}:s;
 const value={schema:packet.schema,demo:packet.demo,base_valid:packet.base_valid,seq:packet.seq,uptime_ms:packet.uptime_ms,fault_mask:packet.fault_mask,speed_percent:packet.speed_percent,distance_cm:packet.distance_cm,buddies};
 return new TextEncoder().encode(JSON.stringify(value)).length;
}
export function renderDiagnostics(state,time,build){
 const $=id=>document.getElementById(id),d=state.diagnostics,device=d.device,age=d.deviceAt===null?null:Math.max(0,time-d.deviceAt);
 $('diagnostic-packet').textContent=d.packetBytes===null?'—':String(d.packetBytes);
 $('diagnostic-packet-note').textContent=d.packetBytes===null?'Awaiting telemetry payload':'Last accepted payload · MQTT overhead excluded';
 const reference=[2,3].includes(state.telemetry?.schema)?equivalentJsonBytes(state.telemetry):null;
 $('diagnostic-text-size').textContent=reference===null?'Equivalent JSON comparison awaits binary data':`Same represented values as JSON: ${reference} bytes`;
 $('diagnostic-range').textContent=device?String(device.out_of_range):'—';
 $('diagnostic-rejected').textContent=device?String(device.rejected):'—';
 $('diagnostic-rx').textContent=String(d.rxRejected);
 $('diagnostic-status').textContent=!device?'Awaiting firmware diagnostics':`${device.demo?'Simulated firmware':'Firmware'} · ${age.toFixed(1)} s since reception${!state.connected||age>=3?' · STALE':''}`;
 const e=device?.reason?device:null;
 $('diagnostic-field').textContent=e?validationLabel(e):device?'No validation rejection reported':'Awaiting device report';
 $('diagnostic-reason').textContent=e?(e.reason===2?`Rejected ${e.value} · allowed ${e.min} to ${e.max}`:e.reason===1?'Missing snapshot input':'Invalid frame format'):'Rejected samples do not replace the last valid readings.';
 $('diagnostic-browser-note').textContent=d.lastRxError?`Last browser rejection: ${d.lastRxError.reason}`:'Browser counters cover this dashboard session only.';
 const terrain=state.telemetry?.buddies?.[4];
 $('diagnostic-clamped').textContent=terrain?`Last Buddy 4 frame: ${terrain.clamped?'one or more fields saturated to fit':'no saturation flag'}.`:'No Buddy 4 saturation report available.';
 $('diagnostic-build').textContent=build?.verified?'Verified static RAM comparison':'Build comparison pending';
 $('diagnostic-before').textContent=build?.verified?`${build.before.static_ram_bytes.toLocaleString()} B`:'—';
 $('diagnostic-after').textContent=build?.verified?`${build.after.static_ram_bytes.toLocaleString()} B`:'—';
 $('diagnostic-delta').textContent=build?.verified?`${build.after.static_ram_bytes-build.before.static_ram_bytes>=0?'+':''}${(build.after.static_ram_bytes-build.before.static_ram_bytes).toLocaleString()} B`:'—';
 $('diagnostic-ram-note').textContent=build?.verified?`${build.profile} · .data + .bss from matched builds. Excludes runtime stack/heap usage; not live RAM.`:'No live stack/heap measurement is available. RAM will appear after matching builds.';
}
