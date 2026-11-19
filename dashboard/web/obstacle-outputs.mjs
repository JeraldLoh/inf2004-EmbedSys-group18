// Display only producer-reported encounter/profile/decision states.
export function obstacleOutputs(s) {
 if(!s)return {detection:'Awaiting data',width:'Awaiting data',direction:'Awaiting data',rejoin:'Awaiting data'};
 const last=s.state===0?' · last encounter':'';
 return {
  detection:s.state===11?'Unknown · avoidance failed':s.state===0?'No active encounter':'Yes · encounter active',
  width:s.profile_valid?`${!s.left_found||!s.right_found?'At least ':''}${(s.width_mm/10).toFixed(1)} cm${last}`:'Not reported yet',
  direction:s.decision_valid?(['No usable side','Left','Right'][s.side]??'Unknown')+last:'Not decided yet',
  rejoin:s.state===10?'Searching for line':s.state===11?'Avoidance failed':s.state===9?'Not active · bypassing':'Not active'
 };
}
