// Barcode route map (Buddy 3): which barcode letter means which turn.
// Wire format matches app_program/subsystem1/buddy1_protocol.h:
//   MAP_SET  22 bytes: B1 02 03 03 id(u32 LE) + 14 map bytes
//   MAP_SHOW  8 bytes: B1 02 03 04 id(u32 LE)
// 36 entries (A-Z then 0-9) x 3 bits, LSB-first. 0 none, 1 left, 2 right,
// 3 straight, 4 U-turn. The car replies on command_result with "map" as
// 28 hex characters of the table it now holds.
export const ACTIONS=[
 {value:1,key:'L',name:'Left',icon:'↰'},
 {value:2,key:'R',name:'Right',icon:'↱'},
 {value:3,key:'S',name:'Straight',icon:'↑'},
 {value:4,key:'U',name:'U-turn',icon:'↶'}
];
export const MAP_BYTES=14,ENTRIES=36;
export const DEFAULT_MAP=Object.freeze({A:1,B:2,C:3,D:4});   // firmware power-on map
export function charIndex(c){
 if(typeof c!=='string'||c.length!==1)return -1;
 c=c.toUpperCase();
 if(c>='A'&&c<='Z')return c.charCodeAt(0)-65;
 if(c>='0'&&c<='9')return 26+c.charCodeAt(0)-48;
 return -1;
}
export const indexChar=i=>i<26?String.fromCharCode(65+i):String(i-26);
export function packMap(map){
 const bytes=new Uint8Array(MAP_BYTES);
 for(const [c,v] of Object.entries(map)){
  const i=charIndex(c);
  if(i<0)throw Error('Bad barcode character: '+c);
  if(!Number.isInteger(v)||v<0||v>4)throw Error('Bad turn for '+c);
  for(let b=0;b<3;b++)if((v>>b)&1){const at=i*3+b;bytes[at>>3]|=1<<(at&7);}
 }
 return bytes;
}
export function unpackMap(bytes){
 if(!(bytes instanceof Uint8Array)||bytes.length!==MAP_BYTES)throw Error('Bad map length');
 if(bytes[MAP_BYTES-1]&0xf0)throw Error('Bad map padding');
 const map={};
 for(let i=0;i<ENTRIES;i++){
  let v=0;for(let b=0;b<3;b++){const at=i*3+b;if(bytes[at>>3]&(1<<(at&7)))v|=1<<b;}
  if(v>4)throw Error('Bad map entry');
  if(v)map[indexChar(i)]=v;
 }
 return map;
}
export function mapFromHex(hex){
 if(typeof hex!=='string'||!/^[0-9a-f]{28}$/.test(hex))throw Error('Bad map field');
 return unpackMap(Uint8Array.from(hex.match(/../g),h=>parseInt(h,16)));
}
export const mapToHex=map=>[...packMap(map)].map(b=>b.toString(16).padStart(2,'0')).join('');
function header(opcode,id,length){
 if(!Number.isInteger(id)||id<1||id>0xffffffff)throw Error('Invalid command ID');
 const bytes=new Uint8Array(length);bytes.set([0xb1,2,3,opcode]);
 new DataView(bytes.buffer).setUint32(4,id,true);return bytes;
}
export function encodeMapSet(map,id){const bytes=header(3,id,8+MAP_BYTES);bytes.set(packMap(map),8);return bytes;}
export const encodeMapShow=id=>header(4,id,8);
export function decodeMapCommand(bytes){
 if(!(bytes instanceof Uint8Array)||bytes.length<8||bytes[0]!==0xb1||bytes[1]!==2||bytes[2]!==3)throw Error('Invalid map command');
 const id=new DataView(bytes.buffer,bytes.byteOffset,8).getUint32(4,true);if(!id)throw Error('Invalid command ID');
 if(bytes[3]===4&&bytes.length===8)return {kind:'SHOW',command_id:id};
 if(bytes[3]===3&&bytes.length===8+MAP_BYTES)return {kind:'SET',command_id:id,map:unpackMap(bytes.slice(8))};
 throw Error('Invalid map command');
}
export const sortedEntries=map=>Object.entries(map).filter(([,v])=>v).sort(([a],[b])=>charIndex(a)-charIndex(b));
export const mapSummary=map=>sortedEntries(map).map(([c,v])=>c+'='+ACTIONS[v-1].key).join(' ')||'(empty)';
export function sameMap(a,b){return !!a&&!!b&&mapSummary(a)===mapSummary(b);}
