'use strict';
const el=id=>document.getElementById(id);
let control={yaw:50,pitch:-25,distance:4,layers:{scene:true,lidar:false,cameras:false,radiation:false}},drag=null,pending=false;
el('halo').href=`http://${location.hostname}:8080/`;
async function api(path,body){const r=await fetch(path,{method:body?'POST':'GET',headers:body?{'Content-Type':'application/json','X-Sim-Request':'1'}:{},body:body?JSON.stringify(body):undefined});if(r.status===401){show(false);throw Error(await r.text());}if(!r.ok)throw Error(await r.text());return r.json();}
function show(ok){el('login').hidden=ok;el('console').hidden=!ok;el('logout').hidden=!ok;if(ok&&!el('view').getAttribute('src'))el('view').src='/stream.mjpg';if(!ok)el('view').removeAttribute('src');document.querySelectorAll('img[data-src]').forEach(img=>{if(ok)img.src=img.dataset.src;else img.removeAttribute('src');});}
let initialized=false;
async function initialize(){const saved=await api('/view');control={...control,...saved,layers:{...control.layers,...saved.layers}};el('distance').value=control.distance;document.querySelectorAll('[data-layer]').forEach(input=>input.checked=control.layers[input.dataset.layer]);initialized=true;}
async function refresh(){try{const s=await api('/status');show(true);if(!initialized)await initialize();const l=s.layers||{};const reading=l.radiation_readings?.[0];if(reading){el('dose-rate').textContent=`${reading.dose_rate_usv_h.toFixed(3)} µSv/h`;el('radiation-details').replaceChildren(...reading.contributions.map(source=>{const row=document.createElement('tr');for(const value of [source.id,`${source.distance_m.toFixed(2)} m`,`${source.dose_rate_usv_h.toFixed(3)} µSv/h`]){const cell=document.createElement('td');cell.textContent=value;row.append(cell);}return row;}));}const messages=[];if(control.layers.lidar)messages.push(l.lidar_age_seconds==null?'LiDAR scan unavailable':`${l.lidar_points} live LiDAR points (green / orange)`);if(control.layers.cameras)messages.push(`${l.camera_frustums||0} camera frustums · 2 m display length`);if(control.layers.radiation)messages.push(l.radiation_sources?.length?`${l.radiation_sources.length} radiation sources (amber) · ${(l.radiation_readings?.[0]?.dose_rate_usv_h||0).toFixed(3)} µSv/h at arm detector (cyan)`:'No radiation sources configured');el('layer-status').textContent=messages.join(' · ');if(s.fps_per_camera)el('status').textContent=`${s.fps_per_camera.toFixed(1)} fps · cameras ${s.width}×${s.height}`;}catch(e){if(e.message!=='Login required')el('status').textContent='Renderer unavailable';}}
el('login').onsubmit=async e=>{e.preventDefault();try{await api('/login',{password:el('password').value});el('password').value='';el('error').textContent='';await refresh();}catch(e){el('error').textContent=e.message;}};
el('logout').onclick=async()=>{await api('/logout',{});show(false);initialized=false;};
function send(){if(pending)return;pending=true;setTimeout(async()=>{pending=false;try{await api('/view',control);}catch(e){el('status').textContent=e.message;}},40);}
el('reset').onclick=()=>{control={...control,yaw:50,pitch:-25,distance:4};el('distance').value=4;send();};
document.querySelectorAll('[data-layer]').forEach(input=>input.onchange=()=>{control.layers[input.dataset.layer]=input.checked;send();refresh();});
el('distance').oninput=e=>{control.distance=Number(e.target.value);send();};
el('viewport').onpointerdown=e=>{drag=[e.clientX,e.clientY];e.currentTarget.setPointerCapture(e.pointerId);};
el('viewport').onpointermove=e=>{if(!drag)return;control.yaw=(control.yaw+(e.clientX-drag[0])*.3)%360;control.pitch=Math.max(-85,Math.min(-5,control.pitch+(e.clientY-drag[1])*.25));drag=[e.clientX,e.clientY];send();};
el('viewport').onpointerup=()=>{drag=null;};el('viewport').onpointercancel=()=>{drag=null;};
el('viewport').onwheel=e=>{e.preventDefault();control.distance=Math.max(1,Math.min(30,control.distance*Math.exp(e.deltaY*.001)));el('distance').value=control.distance;send();};
refresh();setInterval(refresh,3000);
