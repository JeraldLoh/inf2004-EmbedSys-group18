const item=(name,state,label)=>({name,state,label});
// Interpret only fields actually carried by the current binary specification.
// Neither an invalid reading nor an unset flag proves calibration failed.
export function calibrationView(id,s,live){
 if(id===2){
  const configured=!!s&&live?.encoder_left_counts>0&&live?.encoder_right_counts>0&&live?.wheel_diameter_mm>0;
  return {items:[item('Encoder calibration',configured?'configured':'unknown',configured?'Configured':s?'Not reported':'Awaiting data')],note:configured?`L ${live.encoder_left_counts} / R ${live.encoder_right_counts} counts/rev · wheel ${live.wheel_diameter_mm} mm. Stored conversion settings; no live calibration test result.`:'Waiting for active encoder settings; motion qualification is separate.'};
 }
 if(!s)return {items:(id===4?['Level','Magnetometer']:id===5?['Sonar','Scan servo']:['Barcode']).map(name=>item(name,'unknown','Awaiting data')),note:'Calibration results appear when sensor telemetry arrives.'};
 if(id===3)return {items:[s.calibrated?item('Barcode','passed','Passed'):item('Barcode','pending','Not ready')],
  note:s.calibrated?'Barcode thresholds are calibrated.':'Progress and failure results are not reported by this feed.'};
 if(id===4){
  const c=s.calibration;
  return {items:[c===1?item('Level','running','Calibrating'):c===2||c===3?item('Level','passed','Passed'):item('Level','pending','Not ready'),
   c===3?item('Magnetometer','passed','Passed'):item('Magnetometer','unknown','Not confirmed')],
   note:c===1?'Keep the car still for level calibration.':c===3?'Both calibrations are available. Sensor validity is checked separately.':'Magnetometer progress and calibration failure results are not reported.'};
 }
 return {items:[item('Sonar','unknown','Not reported'),item('Scan servo','unknown','Not reported')],note:'This feed does not include calibration results.'};
}
export function renderCalibration(id,s,{stale=false,demo=false,live=null}={}){
 const root=document.getElementById('calibration-'+id),v=calibrationView(id,s,live);
 root.replaceChildren();
 const heading=document.createElement('div');heading.className='calibration-heading';
 heading.textContent='CALIBRATION'+(s?(demo?' · SAMPLE':stale?' · LAST REPORTED':''):'');root.append(heading);
 for(const result of v.items){
  const row=document.createElement('div'),name=document.createElement('span'),badge=document.createElement('span');
  row.className='calibration-row';name.textContent=result.name;
  badge.className='calibration-badge '+result.state;badge.textContent=(result.state==='passed'?'✓ ':result.state==='running'?'◌ ':'')+result.label;
  row.append(name,badge);root.append(row);
 }
 const note=document.createElement('p');note.className='calibration-note';note.textContent=(stale&&s?'Last received result · ':'')+v.note;root.append(note);
}
