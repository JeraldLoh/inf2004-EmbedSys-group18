import {sectionAgeMs} from './binary.mjs';
export const lineNames=['Centred','Steer left','Steer right','Junction','Reacquire'];
export const terrainNames=['Unknown','Level','Ascending','Crest','Descending','Settling','Invalid'];
export const avoidanceNames=['Monitor','Coarse scan','Fine scan right','Fine scan left','Turn scan right','Turn scan left','Decide','Wait rescan','Reverse','Bypass','Reacquire','Failed'];
export const rangeNames=['OK','No echo','Too close','Sensor fault','No reading yet'];
export const systemNames=['Startup','Idle','Auto','Manual','Calibration','Stopped','Fault'];
export const resultNames=['Accepted','Rejected','In progress','Completed','Cancelled','Timeout','Aborted','Fault'];
export function barcodeDurationText(s){return s?.scan_valid&&s.barcode_source===0&&s.bar_ms>0?s.bar_ms+' ms':'Unavailable';}
export function motionSpeed(s){return s?.left_speed_valid&&s?.right_speed_valid?(s.left_speed_mm_s+s.right_speed_mm_s)/2:null;}
export function motionView(packet,elapsedMs=0){
 const s=packet?.buddies?.[2],speed=motionSpeed(s),age=s?sectionAgeMs(packet,s,elapsedMs):null;
 return {speed,age,fresh:speed!==null&&age!==null&&age<1500};
}
export function buddyView(packet,id,elapsedMs=0){
 const s=packet?.buddies?.[id];if(!s)return {available:false,label:'Unavailable Â· awaiting integration',rows:[]};
 const age=sectionAgeMs(packet,s,elapsedMs),stale=age===null||age>=1500;
 const label=(packet.demo?'Simulated Â· ':'')+(stale?'STALE Â· ':'')+(age===null?'unknown age':Math.round(age)+' ms since snapshot');
 let rows=[];
 if(id===2)rows=[['Target speed',s.target_speed_valid?s.target_speed_mm_s+' mm/s':'Unavailable'],['Left wheel',s.left_speed_valid?s.left_speed_mm_s+' mm/s':'Unavailable'],['Right wheel',s.right_speed_valid?s.right_speed_mm_s+' mm/s':'Unavailable'],['Distance',s.distance_valid?s.distance_mm+' mm':'Unavailable'],['System mode',systemNames[s.system_mode]??'Unknown'],['Movement',s.motion_active?'Active (producer reported)':'Inactive (producer reported)'],['Odometry valid',s.odometry_valid?'Yes':'No'],['Latest result',s.result_pending?resultNames[s.last_motion_result]:'No pending result'],['Stop reasons','0x'+(s.stop_reasons??0).toString(16)],['Hardware faults','0x'+(s.hardware_faults??0).toString(16)],['Command ID',s.last_command_id??'Unavailable'],['Authority generation',s.authority_generation??'Unavailable']];
 if(id===3)rows=[['Line state',lineNames[s.line_state]],['Steering',s.steering+' (âˆ’100 left / +100 right)'],['IR left / right',`${s.left_black} / ${s.right_black} (1 = black)`],['Barcode ADC',s.adc_valid?s.adc:'Unavailable'],['Calibration',s.calibrated?`${s.calibration_low}â€“${s.calibration_high}`:'Not calibrated'],['Route',s.scan_valid?['None','Left','Right','Straight','U-turn'][s.route]:'No retained scan'],['Last bar',barcodeDurationText(s)],['Wide/narrow pattern',s.pattern_valid?s.pattern.toString(2).padStart(9,'0'):'Unavailable'],['Event counters',`Route ${s.route_counter} Â· junction ${s.junction_counter} Â· scan ${s.scan_counter}`]];
 if(id===4)rows=[['Terrain',terrainNames[s.terrain]??'Unknown'],['Data valid',s.data_valid?'Yes':'No Â· attitude unavailable'],['Pitch / roll',s.data_valid?`${s.pitch_deg}Â° / ${s.roll_deg}Â°`:'Unavailable'],['Heading',s.heading_valid?s.heading_deg+'Â° magnetic':'Unavailable Â· MAG/calibration required'],['Speed cap',s.speed_cap_mm_s?s.speed_cap_mm_s+' mm/s':'No cap'],['Last hump',s.hump_valid?`${s.hump_mm} mm Â± ${s.uncertainty_mm} mm (heuristic)`:'No valid result'],['Collision active',s.collision_active?'Yes':'No'],['Calibration',['None','Running','Level','Level + MAG'][s.calibration]],['Clamped',s.clamped?'Yes Â· a packed value saturated':'No'],['Sensor freshness','Reception age only; measurement timestamp not supplied']];
 if(id===5){
  const reading=(status,mm,at)=>{
   const age=(packet.uptime_ms-at)>>>0;
   return rangeNames[status]+(status===0?` Â· ${mm/10} cm`:'')+(status!==4?` Â· ${age>=0x80000000?'unknown':Math.round(age+elapsedMs)+' ms'} old`:'');
  };
  rows=[['Avoidance',avoidanceNames[s.state]],['Front at scan-start heading',reading(s.front_status,s.front_mm,s.front_at_ms)],['Latest reading',reading(s.last_status,s.last_mm,s.last_at_ms)],['Latest direction',s.last_status===4?'Unavailable':`${s.direction_halfdeg/2}Â° relative to scan start (+left)`],['Commanded heading',s.heading_deg+'Â° Â· not measured'],['Commanded servo',s.servo_valid?s.servo_deg+'Â° Â· no feedback':'Unavailable'],['Encounter',s.encounter],['Profile',s.profile_valid?`Width ${s.width_mm} mm${!s.left_found||!s.right_found?' (minimum only)':''}`:'Unavailable'],['Decision',s.decision_valid?['None / no usable side','Left','Right'][s.side]:'Unavailable'],['Bypass plan',s.plan_valid?`${s.turn_deg}Â° Â· diagonal ${s.diagonal_mm} mm Â· alongside ${s.alongside_mm} mm`:'Unavailable'],['Events dropped',s.dropped_events],['Totals',`${s.detections} detections Â· ${s.cleared} cleared Â· ${s.sensor_faults} sensor faults`]];
 }
 return {available:true,stale,label,rows,raw:s};
}
