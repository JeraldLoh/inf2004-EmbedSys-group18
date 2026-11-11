import {liveGroup} from './live-status.mjs?v=barcode-digital-4';
const routes=['No action assigned','Left','Right','Straight','U-turn'];
function result(line){
 const letter=line.barcode_character?String.fromCharCode(line.barcode_character):'?';
 switch(line.barcode_state){
  case 1:return {state:'scanning',headline:'Reading…',detail:'Sensor activity detected. Capturing the pattern; waiting for a complete barcode.'};
  case 2:return {state:'decoded',headline:`Decoded ${letter} → ${routes[line.barcode_route]}`,detail:'Barcode read successfully. The controller checks the pending instruction at the next junction; this does not confirm a turn.'};
  case 3:return {state:'failed',headline:'Unable to read barcode',detail:'The captured pattern did not decode. It may be an incomplete barcode or a tape crossing. No new turn instruction was produced.'};
  case 4:return {state:'unmapped',headline:`Decoded ${letter} · no action assigned`,detail:'The barcode was readable. Assign this letter in Barcode routes, send the map, then scan again.'};
  default:return {state:'waiting',headline:'Ready · waiting for sensor activity',detail:'Move the barcode across the sensor. Reading starts at the first captured bar.'};
 }
}
// The reader status is a low-rate side packet. Accept it up to 5 s old (not the
// 1.5 s other panels use) so a brief Wi-Fi/MQTT hiccup does not flip the panel to
// "Connect the Pico" mid-read; say so when it is late.
function readerGroup(packet){
 const s=packet?.live_status;
 if(!s||s.demo!==packet.demo||!(s.present&2))return null;
 const g=s.line,age=((s.uptime_ms-g.at_ms)>>>0)+Math.max(0,packet.live_status_elapsed_ms??0);
 return age<5000?{group:g,late:age>=1500}:null;
}
export function barcodeView(packet,{stale=false}={}){
 const found=readerGroup(packet),line=found?found.group:null;
 if(stale)return {state:'stale',headline:'Barcode reader · readings stale',detail:'Waiting for fresh Pico data. Previous results do not describe the current scan.'};
 if(!line)return {state:'waiting',headline:'Waiting for barcode reader',detail:'Connect the Pico and wait for fresh reader status.'};
 if(line.barcode_state===null||line.barcode_state===undefined)return {state:'waiting',headline:'Reader status update needed',detail:'Flash the firmware supplied with this dashboard to report scanning, decoded letters and failed reads.'};
 if(line.calibration_state===1)return {state:'waiting',headline:'Calibrating barcode reader',detail:'Sweep the sensor across black and white as prompted before scanning.'};
 if(line.calibration_state===3)return {state:'failed',headline:'Barcode reader not ready',detail:'Calibration failed. Recalibrate before scanning barcodes.'};
 const view=result(line),age=((packet.live_status.uptime_ms-line.barcode_event_ms)>>>0)+Math.max(0,packet.live_status_elapsed_ms??0);
 if([2,3,4].includes(line.barcode_state))view.detail+=` Last result ${(age/1000).toFixed(1)} s ago · ${line.barcode_failures} unreadable attempts this boot.`;
 const sensor=packet?.buddies?.[3];
 if(line.barcode_digital&&sensor)view.detail+=` Digital D0 on GP26: ${sensor.black?'black':'white'} detected.`;
 if(sensor?.adc_valid)view.detail+=` Sensor: ${sensor.black?'black':'white'} · ADC ${sensor.adc}.`;
 if(found.late)view.detail+=' (update delayed: Wi-Fi/MQTT is slow, this may be a few seconds old)';
 if(packet.demo)view.detail='Simulated preview. '+view.detail;
 return view;
}
export function barcodeEvents(previous,current){
 if(!current||current.demo||current.version<4||!(current.present&2)||((current.uptime_ms-current.line.at_ms)>>>0)>=1500)return [];
 const line=current.line,sameBoot=previous&&!previous.demo&&previous.version>=4&&(previous.present&2)&&((current.uptime_ms-previous.uptime_ms)>>>0)<0x80000000;
 const before=sameBoot?previous.line:null;
 const scanning=line.barcode_state===1&&(!before||before.barcode_state!==1);
 const completed=[2,3,4].includes(line.barcode_state)&&(!before||line.barcode_sequence!==before.barcode_sequence);
 if(!scanning&&!completed)return [];
 return [{text:`Barcode: ${result(line).headline}${!before&&completed?' (last reported result)':''}`,type:line.barcode_state>=3?'warning':'system'}];
}

export function renderBarcodeStatus(packet,{stale=false}={}){
 const root=document.getElementById('barcode-status');
 if(!root)return;
 const view=barcodeView(packet,{stale});
 document.getElementById('barcode-headline').textContent=view.headline;
 document.getElementById('barcode-detail').textContent=view.detail;
 root.dataset.state=view.state;
}
