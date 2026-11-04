import {systemNames,avoidanceNames} from './telemetry-view.mjs';
const missions=['Idle','Following line','Junction approach','Junction movement','Verify line','Obstacle stop','Scanning','Planning bypass','Bypassing','Recovery','Safe stop'];
const stops=['Physical stop','Remote STOP','Obstacle stop','Motion fault','Manual request expired','Controller fault / watchdog','Critical sensor fault','Recovery failed','Barcode route required'];
export function stopNames(mask){const names=stops.filter((_,i)=>mask&(1<<i));const other=(mask&~511)>>>0;if(other)names.push('Unknown bits 0x'+other.toString(16));return names.join(', ')||'None';}
// Observations from latest snapshots, not a complete device event history.
export function robotEvents(previous,packet){
 if(packet?.demo||!packet?.buddies?.[2])return [];
 const old=previous?.demo?null:previous?.buddies?.[2],s=packet.buddies[2],events=[];
 const add=(text,type='system')=>events.push({text:`Robot ${(packet.uptime_ms/1000).toFixed(1)} s: ${text}`,type});
 if(previous&&!previous.demo&&((packet.uptime_ms-previous.uptime_ms)>>>0)>=0x80000000)add('uptime reset; rover may have restarted.','warning');
 if(!old||s.system_mode!==old.system_mode)add(`controller ${systemNames[s.system_mode]??s.system_mode}; movement ${s.motion_active?'reported active':'reported inactive'}.`,s.system_mode>=5?'warning':'system');
 if((!old&&s.stop_reasons)||(old&&s.stop_reasons!==old.stop_reasons))add(s.stop_reasons?'Stop reasons: '+stopNames(s.stop_reasons)+'.':'Stop reasons cleared.',s.stop_reasons?'warning':'system');
 if((!old&&s.hardware_faults)||(old&&s.hardware_faults!==old.hardware_faults))add(s.hardware_faults?'Hardware fault mask: 0x'+s.hardware_faults.toString(16)+'.':'Hardware fault mask cleared.',s.hardware_faults?'warning':'system');
 if(old&&s.mission_state!==old.mission_state)add(`mission state ${missions[old.mission_state]??old.mission_state} -> ${missions[s.mission_state]??s.mission_state}.`);
 const b=packet.buddies[5],a=previous?.buddies?.[5];if(b&&a&&b.state!==a.state)add('Avoidance: '+(avoidanceNames[b.state]??b.state)+'.',b.state===11?'warning':'system');
 return events;
}

export function lineSearchEvents(previous,current){
 if(!current||current.demo||!(current.present&1))return [];
 const m=current.motion;if(((current.uptime_ms-m.at_ms)>>>0)>=1500)return [];
 const before=previous&&!previous.demo&&(previous.present&1)&&((previous.uptime_ms-previous.motion.at_ms)>>>0)<1500?previous.motion:null;
 const events=[],add=(text,type='system')=>events.push({text:`Robot ${(current.uptime_ms/1000).toFixed(1)} s: ${text}`,type});
 const sameBoot=previous&&((current.uptime_ms-previous.uptime_ms)>>>0)<0x80000000;
 if(m.searching&&(!before||!sameBoot||!before.searching))add(`Searching for line · attempt ${m.searches}.`);
 if(before&&sameBoot&&m.search_fails>before.search_fails)add(`Line search failed · total ${m.search_fails}.`,'warning');
 else if(before&&sameBoot&&before.searching&&!m.searching)add('Line search ended; check controller state for the outcome.');
 if(before&&sameBoot&&m.searches>before.searches&&!m.searching)add(`Line search attempt reported · total ${m.searches}.`);
 return events;
}

export function calibrationEvents(previous,current){
 if(!current||current.demo||!(current.present&2))return [];
 const line=current.line;if(((current.uptime_ms-line.at_ms)>>>0)>=1500||line.calibration_state===4)return [];
 const same=previous&&!previous.demo&&(previous.present&2)&&((current.uptime_ms-previous.uptime_ms)>>>0)<0x80000000;
 if(same&&previous.line.calibration_state===line.calibration_state&&previous.line.calibration_reason===line.calibration_reason)return [];
 const text=['not started','running; sweep over black and white','passed','failed'+(line.calibration_reason===1?': insufficient black/white contrast':'; reason unavailable')][line.calibration_state];
 return [{text:`Robot ${(current.uptime_ms/1000).toFixed(1)} s: Barcode calibration ${text}.`,type:line.calibration_state===3?'warning':'system'}];
}
