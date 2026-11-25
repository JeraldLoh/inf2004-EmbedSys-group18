import {unpackMap} from './route-map.mjs?v=barcode-digital-4';
const resultNames=['simulated','queued','rejected','queue_full','applied'];

export function decodeControlWire(topic,bytes){
 const heartbeat=topic==='heartbeat';
 if(!heartbeat&&topic!=='command_result')throw Error('Unexpected topic');
 if(bytes.length<4||bytes[0]!==0xb1||bytes[1]!==1||bytes[2]!==(heartbeat?5:6))
  throw Error('Invalid binary control header');
 const view=new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength);
 const flags=bytes[3];
 if(heartbeat){
  if(bytes.length!==12||flags>1)throw Error('Invalid binary heartbeat');
  return {schema:1,demo:!!flags,seq:view.getUint32(4,true),uptime_ms:view.getUint32(8,true)};
 }
 const status=resultNames[flags&7], hasMap=(flags&16)!==0;
 if(flags&0xe0||!status||hasMap!==(status==='applied')||bytes.length!==(hasMap?26:8))
  throw Error('Invalid binary command result');
 const result={schema:1,demo:!!(flags&8),status,command_id:view.getUint32(4,true)};
 if(hasMap){
  result.map_revision=view.getUint32(8,true);
  if(!result.command_id||!result.map_revision||(result.map_revision&1)||bytes[25]&0xf0)
   throw Error('Invalid binary map result');
  const map=bytes.subarray(12),actions=Object.values(unpackMap(map));
  if(actions.length!==new Set(actions).size)throw Error('Duplicate binary map action');
  result.map=Array.from(map,b=>b.toString(16).padStart(2,'0')).join('');
 }
 return result;
}
