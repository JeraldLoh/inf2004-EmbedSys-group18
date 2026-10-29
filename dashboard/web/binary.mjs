import {fields} from './binary-fields.mjs';
const size=id=>Math.ceil(fields[id].reduce((n,f)=>n+f[1],0)/8);
const fail=()=>{throw Error('Invalid binary telemetry');};
function uint(value,max=0xffffffff){if(!Number.isInteger(value)||value<0||value>max)fail();return value;}
function unpack(id,bytes){
 if(bytes.length!==size(id))fail();let at=0;const result={};
 for(const [name,width,min,max] of fields[id]){
  let value=0;for(let i=0;i<width;i++,at++)value+=((bytes[at>>3]>>(at%8))&1)*2**i;
  if(min<0&&value>=2**(width-1))value-=2**width;
  if(value<min||value>max)throw new RangeError(`Buddy ${id} · ${name}: ${value} outside ${min} to ${max}`);result[name]=value;
 }
 while(at<bytes.length*8){if((bytes[at>>3]>>(at%8))&1)fail();at++;}
 return result;
}
function pack(id,values){
 const bytes=new Uint8Array(size(id));let at=0;
 for(const [name,width,min,max] of fields[id]){
  let value=values[name]??0;
  if(!Number.isInteger(value)||value<min||value>max)fail();
  if(value<0)value+=2**width;
  for(let i=0;i<width;i++,at++)if(Math.floor(value/2**i)%2)bytes[at>>3]|=1<<(at%8);
 }
 return bytes;
}
export function decodeBuddy4(bytes){
 if(bytes.length!==17||bytes[0]!==0x21)fail();
 const d=new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength),w=Array.from({length:4},(_,i)=>d.getUint32(1+i*4,true));
 const get=(word,at,n)=>Math.floor(w[word]/2**at)%2**n;
 const signed=(v,n)=>v>=2**(n-1)?v-2**n:v;
 const calibration=get(0,20,2),detail_faults=get(0,13,7),data_valid=!!get(0,31,1);
 const heading_valid=data_valid&&calibration===3&&!(detail_faults&2);
 return {raw:Array.from(bytes),terrain:get(0,0,3),speed_cap_mm_s:get(0,3,7)*10,motion_class:get(0,10,3),detail_faults,
  calibration,odometry_used:!!get(0,22,1),hump_active:!!get(0,23,1),collision_active:!!get(0,24,1),collision_counter:get(0,25,3),collision_side:get(0,28,2),
  clamped:!!get(0,30,1),data_valid,heading_valid,
  pitch_deg:data_valid?signed(get(1,0,9),9)/2:null,roll_deg:data_valid?signed(get(1,9,9),9)/2:null,
  heading_deg:heading_valid?get(1,18,9):null,sample_counter:get(1,27,5),
  turn_rate_deg_s:heading_valid?signed(get(2,0,8),8)*2:null,live_hump_mm:data_valid?get(2,8,8):null,
  shock_mg:get(2,16,8)*16,vibration_mg:data_valid?get(2,24,6)*8:null,
  hump_mm:get(3,0,8),uncertainty_mm:get(3,8,6),peak_pitch_deg:get(3,14,6),path_cm:get(3,20,8),hump_valid:!!get(3,28,1),hump_seq:get(3,29,3)};
}
export function decodeTelemetry(input){
 const bytes=input instanceof Uint8Array?input:new Uint8Array(input),v3=bytes[1]===3,header=v3?19:18;
 if(bytes.length<header||bytes.length>512||bytes[0]!==0xb1||![2,3].includes(bytes[1])||bytes[2]!==1||(bytes[3]&(v3?0xc0:0xe0)))fail();
 const d=new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength),mask=(bytes[3]>>2)&(v3?15:7),base=v3?15:14;
 if(v3&&(bytes[14]&0xf0))fail();
 const result={schema:bytes[1],demo:!!(bytes[3]&1),base_valid:!!(bytes[3]&2),seq:d.getUint32(4,true),uptime_ms:d.getUint32(8,true),fault_mask:d.getUint16(12,true)+(v3?bytes[14]*65536:0),speed_percent:bytes[base],distance_cm:bytes[base+1]+bytes[base+2]*256+bytes[base+3]*65536,buddies:{}};
 if(result.speed_percent>100||result.distance_cm>100000)fail();let at=header,seen=0;
 while(at<bytes.length){
  if(at+6>bytes.length)fail();const id=bytes[at],n=bytes[at+1],flag=({2:v3?8:0,3:1,4:2,5:4})[id];
  if(!flag||!(mask&flag)||(seen&flag)||at+6+n>bytes.length)fail();
  const payload=bytes.slice(at+6,at+6+n),updated_ms=d.getUint32(at+2,true);
  result.buddies[id]={...(id===4?decodeBuddy4(payload):unpack(id,payload)),updated_ms};seen|=flag;at+=6+n;
 }
 if(seen!==mask)fail();return result;
}
export function encodeTelemetry(value){
 const sections=[];let mask=0;const v3=value.schema!==2,header=v3?19:18;
 for(const id of Object.keys(value.buddies??{})){
  const flag=({2:v3?8:0,3:1,4:2,5:4})[id];if(!flag)fail();const v=value.buddies[id];
  const body=id==='4'?Uint8Array.from(v.raw??[]):pack(id,v);if(id==='4')decodeBuddy4(body);
  const section=new Uint8Array(6+body.length);section[0]=Number(id);section[1]=body.length;
  new DataView(section.buffer).setUint32(2,uint(v.updated_ms??value.uptime_ms),true);section.set(body,6);sections.push(section);mask|=flag;
 }
 const bytes=new Uint8Array(header+sections.reduce((n,s)=>n+s.length,0)),d=new DataView(bytes.buffer),base=v3?15:14;
 bytes.set([0xb1,v3?3:2,1,(value.demo?1:0)|((value.base_valid===false?0:1)<<1)|(mask<<2)]);
 const faults=uint(value.fault_mask??0,v3?1048575:65535);
 d.setUint32(4,uint(value.seq),true);d.setUint32(8,uint(value.uptime_ms),true);d.setUint16(12,faults&65535,true);if(v3)bytes[14]=faults>>16;
 bytes[base]=uint(value.speed_percent??0,100);const distance=uint(value.distance_cm??0,100000);
 for(let i=0;i<3;i++)bytes[base+1+i]=Math.floor(distance/2**(8*i))&255;
 let at=header;for(const section of sections){bytes.set(section,at);at+=section.length;}return bytes;
}
export function sectionAgeMs(packet,section,elapsedMs=0){
 const age=(packet.uptime_ms-section.updated_ms)>>>0;
 return age>=0x80000000?null:age+Math.max(0,elapsedMs);
}
