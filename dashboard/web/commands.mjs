export function encodeCommand(command,id){
 if(!['START','STOP'].includes(command))throw Error('Only START and STOP are supported');
 if(!Number.isInteger(id)||id<1||id>0xffffffff)throw Error('Invalid command ID');
 const bytes=new Uint8Array([0xb1,2,3,command==='START'?1:2,0,0,0,0]);new DataView(bytes.buffer).setUint32(4,id,true);return bytes;
}
export function decodeCommand(bytes){
 if(!(bytes instanceof Uint8Array)||bytes.length!==8||bytes[0]!==0xb1||bytes[1]!==2||bytes[2]!==3||![1,2].includes(bytes[3]))throw Error('Invalid START/STOP command');
 const id=new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength).getUint32(4,true);if(!id)throw Error('Invalid command ID');
 return {command:bytes[3]===1?'START':'STOP',command_id:id};
}
