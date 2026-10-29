import {lineNames,terrainNames,avoidanceNames,rangeNames,motionSpeed,systemNames} from './telemetry-view.mjs';
import {renderCalibration} from './calibration.mjs';
// Only presentation; all data validity and age remain visible in sensor details.
export function renderSensorHighlights(id,packet,view,fresh,elapsed){
 const $=key=>document.getElementById(key),s=packet?.buddies?.[id],available=!!s,stale=available&&(!fresh||view.stale);
 $('buddy-'+id).classList.toggle('is-empty',!available);
 renderCalibration(id,s,{stale,demo:!!packet?.demo});
 const badge=$('sensor-badge-'+id);badge.lastChild.textContent=!available?'Waiting':stale?'Stale':packet.demo?'Sample':'Live';
 const put=(key,value)=>{$(key).textContent=value;};
 if(id===2){
  const speed=motionSpeed(s);put('motion-primary',speed===null?'—':String(speed));
  put('motion-target',s?.target_speed_valid?s.target_speed_mm_s+' mm/s':'—');
  put('motion-left',s?.left_speed_valid?s.left_speed_mm_s+' mm/s':'—');put('motion-right',s?.right_speed_valid?s.right_speed_mm_s+' mm/s':'—');
  put('motion-mode',s?systemNames[s.system_mode]??'Unknown':'Awaiting motion data');
  put('sensor-caption-2',s?`${s.distance_valid?'Travel '+s.distance_mm+' mm':'Distance unavailable'} · ${s.motion_active?'movement reported active':'movement reported inactive'}`:'Wheel speeds and distance appear when the motor task supplies them.');
  $('motion-wheels').classList.toggle('is-moving',!!s?.motion_active&&speed!==null&&!stale);
 }
 if(id===3){
  put('line-primary',s?lineNames[s.line_state]:'Waiting for the line');
  put('line-steering',s?(s.steering===0?'Straight':`${Math.abs(s.steering)}% ${s.steering<0?'left':'right'}`):'—');
  put('line-route',s?.scan_valid?['None','Left','Right','Straight','U-turn'][s.route]:'—');
  $('sensor-left').classList.toggle('is-black',!!s?.left_black);$('sensor-right').classList.toggle('is-black',!!s?.right_black);
  $('line-correction').setAttribute('d',`M140 118Q140 65 ${140+(s?.steering??0)*.55} 24`);
  put('sensor-caption-3',s?'Two line sensors · illuminated dots indicate black.':'Your line position and last barcode route will appear here.');
 }
 if(id===4){
  const valid=!!s?.data_valid;
  put('terrain-primary',s?terrainNames[s.terrain]??'Unknown terrain':'Waiting for terrain');
  put('terrain-pitch',valid?`${s.pitch_deg}°`:'—');put('terrain-roll',valid?`${s.roll_deg}°`:'—');put('terrain-heading',s?.heading_valid?`${s.heading_deg}°`:'—');
  $('attitude-horizon').style.transform=`translateY(${valid?Math.max(-25,Math.min(25,s.pitch_deg)):0}px) rotate(${valid?-s.roll_deg:0}deg)`;
  $('attitude-horizon').style.opacity=valid?'1':'.25';
  put('sensor-caption-4',s?.hump_valid?`Last hump ${s.hump_mm} mm · ±${s.uncertainty_mm} mm estimate`:s?(valid?(s.heading_valid?'Attitude and magnetic heading available.':'Attitude available · heading needs magnetometer calibration.'):'Awaiting valid calibration and sensor readings.'):'Pitch, roll and terrain bring the road into view.');
 }
 if(id===5){
  const ok=s?.front_status===0,lastOK=s?.last_status===0;
  put('obstacle-primary',ok?(s.front_mm/10).toFixed(1):'—');
  put('obstacle-unit',!s?'cm ahead':ok?'cm · scan-start front':rangeNames[s.front_status]);
  put('obstacle-state',s?avoidanceNames[s.state]:'—');put('obstacle-angle',s&&s.last_status!==4?`${s.direction_halfdeg/2}°`:'—');
  $('radar-direction').style.transform=`rotate(${s&&s.last_status!==4?-s.direction_halfdeg/2:0}deg)`;
  $('radar-point').style.visibility=ok?'visible':'hidden';
  $('radar-point').setAttribute('cy',String(ok?126-Math.min(95,Math.max(12,s.front_mm/4000*95)):65));
  const age=s?((packet.uptime_ms-s.front_at_ms)>>>0)+Math.max(0,elapsed):0;
  put('sensor-caption-5',!s?'Obstacle distance and scanning activity will appear here.':ok?`Front reading ${age>=0x80000000?'age unknown':(age/1000).toFixed(1)+' s old'} · direction relative to scan start.`:lastOK?'A side reading is available in sensor details.':'Waiting for a valid distance · no echo is not a measured distance.');
 }
}
