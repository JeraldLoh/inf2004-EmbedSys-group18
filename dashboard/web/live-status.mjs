// Supplementary binary status v1: independent of legacy telemetry v2/v3.
export function decodeLiveStatus(p){
 if(!(p instanceof Uint8Array)||!((p[1]===1&&p.length===64)||(p[1]===2&&p.length===76)))throw Error('Invalid live status length');
 const d=new DataView(p.buffer,p.byteOffset,p.byteLength),u=o=>d.getUint32(o,true);
 if(p[0]!==0xb1||![1,2].includes(p[1])||p[2]!==4||p[3]>1||p[8]>7||p[9]>7||p[10]>1||p[11]>7||p[40]>63||p[41]>2||p[42]||p[43])throw Error('Invalid live status header or flags');
 const segment=p[40]&7,segments=p[40]>>3;if(segment>segments)throw Error('Invalid bypass progress');
 return {version:p[1],demo:!!p[3],uptime_ms:u(4),present:p[8],motion:{at_ms:u(12),left_target:d.getInt32(16,true),right_target:d.getInt32(20,true),targets_valid:!!(p[9]&1),qualified:!!(p[9]&2),searching:!!(p[9]&4),searches:u(24),search_fails:u(28),accepted:u(52),rejected:u(56),driver_errors:u(60),encoder_left_counts:p[1]===2?u(64):0,encoder_right_counts:p[1]===2?u(68):0,wheel_diameter_mm:p[1]===2?u(72):0},line:{at_ms:u(32),paused:!!p[10]},bypass:{at_ms:u(36),fast:!!(p[11]&1),active:!!(p[11]&2),backing_off:!!(p[11]&4),segment,segments,side:p[41],backoffs:u(44),elapsed_ms:u(48)}};
}
export function encodeLiveStatus(s){
 const version=s.version??2,p=new Uint8Array(version===1?64:76),d=new DataView(p.buffer),u=(o,v)=>d.setUint32(o,v??0,true),m=s.motion??{},l=s.line??{},b=s.bypass??{};
 p.set([0xb1,version,4,s.demo?1:0]);u(4,s.uptime_ms);p[8]=s.present??0;
 p[9]=(m.targets_valid?1:0)|(m.qualified?2:0)|(m.searching?4:0);p[10]=l.paused?1:0;p[11]=(b.fast?1:0)|(b.active?2:0)|(b.backing_off?4:0);
 u(12,m.at_ms);d.setInt32(16,m.left_target??0,true);d.setInt32(20,m.right_target??0,true);u(24,m.searches);u(28,m.search_fails);u(32,l.at_ms);u(36,b.at_ms);
 p[40]=(b.segment??0)|((b.segments??0)<<3);p[41]=b.side??0;u(44,b.backoffs);u(48,b.elapsed_ms);u(52,m.accepted);u(56,m.rejected);u(60,m.driver_errors);
 if(version===2){u(64,m.encoder_left_counts);u(68,m.encoder_right_counts);u(72,m.wheel_diameter_mm);}
 decodeLiveStatus(p);return p;
}
// Freshness uses this feed's own clock and reception time; unsigned subtraction handles wrap.
export function liveGroup(packet,id){
 const s=packet?.live_status,key={2:'motion',3:'line',5:'bypass'}[id],bit={2:1,3:2,5:4}[id];
 if(!s||s.demo!==packet.demo||!(s.present&bit))return null;
 const g=s[key],age=((s.uptime_ms-g.at_ms)>>>0)+Math.max(0,packet.live_status_elapsed_ms??0);
 return age<1500?g:null;
}
export function lineSearching(packet){
 const motion=packet?.buddies?.[2];
 return !!liveGroup(packet,2)?.searching && !(motion&&(motion.stop_reasons||[5,6].includes(motion.system_mode)));
}
export function lineHeadline(packet,s){
 const motion=packet?.buddies?.[2];
 if(motion?.stop_reasons&128)return 'Stopped · recovery failed';
 if([5,6].includes(motion?.system_mode))return 'Stopped';
 if(liveGroup(packet,3)?.paused)return 'Sensing paused · hump';
 if(lineSearching(packet))return 'Searching for line';
 return ['Centred','Steer left','Steer right','Junction','Line lost · reacquiring'][s.line_state]??'Unknown';
}
export function bypassProgress(b){
 if(!b)return 'Not reported / stale';
 if(b.backing_off)return 'Backing off for clearance';
 if(!b.active)return 'No bypass in progress';
 return b.segment>=b.segments?'Finishing bypass':`${b.fast?'S-curve segment':'Leg'} ${b.segment+1} of ${b.segments}`;
}
