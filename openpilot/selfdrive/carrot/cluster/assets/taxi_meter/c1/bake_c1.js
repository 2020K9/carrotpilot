// Bake C1 (variants/C1.html, read-only) into PNG layers for the raylib K9ExtHud=1 renderer.
// node bake_c1.js <variantsDirUrl> <outDir>   (WORKER only)
// Layers: back_{season}_{tod}.png, front_{season}_{tod}.png (776x456, transparent outside the card),
//         ui_{tod}.png (static text/frame overlay), sprites (porori_*, bono_body, shell) 240x240 anchor at (120,120).
const {spawn}=require('child_process');const fs=require('fs');const path=require('path');
const [,,base,outDir]=process.argv;const PORT=process.env.PORT||9371,PROF='/tmp/cdp_prof_c1bake';
const C=process.env.HOME+'/.cache/ms-playwright/chromium-1243/chrome-linux64/chrome';
fs.mkdirSync(outDir,{recursive:true});
const ch=spawn(C,['--headless=new','--no-sandbox','--disable-gpu','--hide-scrollbars','--force-device-scale-factor=1','--remote-debugging-port='+PORT,'--window-size=796,480','--user-data-dir='+PROF,'about:blank'],{stdio:'ignore'});
const sleep=ms=>new Promise(r=>setTimeout(r,ms));
const SEAS={spring:4,summer:7,autumn:10,winter:1},TOD={day:'12:00:00',dusk:'17:00:00',night:'21:00:00'};
const S=240,A=120; // sprite canvas / anchor
(async()=>{
 let tabs;for(let i=0;i<60;i++){try{tabs=await (await fetch('http://127.0.0.1:'+PORT+'/json')).json();if(tabs.find(t=>t.type==='page'))break}catch(e){}await sleep(200)}
 const pg=tabs.find(t=>t.type==='page');const ws=new WebSocket(pg.webSocketDebuggerUrl);
 await new Promise(r=>ws.onopen=r);let id=0;const pend={};
 ws.onmessage=e=>{const m=JSON.parse(e.data);if(m.method==='Runtime.exceptionThrown')console.error('JSERR',JSON.stringify(m.params.exceptionDetails).slice(0,400));if(m.id&&pend[m.id]){pend[m.id](m);delete pend[m.id]}};
 const send=(method,params={})=>new Promise(r=>{const i=++id;pend[i]=r;ws.send(JSON.stringify({id:i,method,params}))});
 const ev=async e=>{const r=await send('Runtime.evaluate',{expression:e,returnByValue:true});if(r.result.exceptionDetails)console.error('EVERR',JSON.stringify(r.result.exceptionDetails).slice(0,300));return r.result.result&&r.result.result.value};
 await send('Emulation.setDeviceMetricsOverride',{width:796,height:480,deviceScaleFactor:1,mobile:false});
 await send('Emulation.setDefaultBackgroundColorOverride',{color:{r:0,g:0,b:0,a:0}});
 await send('Page.enable');await send('Runtime.enable');
 const shot=async(file,clip)=>{const r=await send('Page.captureScreenshot',{format:'png',clip:Object.assign({scale:1},clip)});fs.writeFileSync(path.join(outDir,file),Buffer.from(r.result.data,'base64'));};
 const load=async(month,clock)=>{await send('Page.navigate',{url:`${base}/C1.html?month=${month}&clock=${clock}#cap`});await sleep(1400);
  for(let k=0;k<100;k++){const ok=await ev(`document.readyState==='complete'&&typeof render==='function'&&document.fonts.status==='loaded'&&location.search.includes('month=${month}&')`);if(ok)break;await sleep(200)}await sleep(600);
  await ev(`render(0);document.querySelector('#bgF').style.removeProperty('display');1`);};
 const style=css=>ev(`(()=>{let s=document.getElementById('bakeCss');if(!s){s=document.createElement('style');s.id='bakeCss';document.head.appendChild(s)}s.textContent=${JSON.stringify(css)};return 1})()`);
 const CARD={x:10,y:12,width:776,height:456};
 const T0='html,body{background:transparent!important}';
 for(const [sn,mo] of Object.entries(SEAS))for(const [tn,ck] of Object.entries(TOD)){
  await load(mo,ck);
  await style(T0+'#ptcl,#trk0,#trkF,#goal,#poro,#bono,#bgF,.trim,.top,.lbl,.fare,#plus,.stats,.next,.bub,#tak,.foot,#mock{visibility:hidden!important}');
  await shot(`back_${sn}_${tn}.png`,CARD);
  await style(T0+'.card{background:transparent!important;border-color:transparent!important}#bgB,#ptcl,#trk0,#trkF,#goal,#poro,#bono,.trim,.top,.lbl,.fare,#plus,.stats,.next,.bub,#tak,.foot,#mock{visibility:hidden!important}');
  await shot(`front_${sn}_${tn}.png`,CARD);
  if(sn==='summer'){
   await style(T0+'.card{background:transparent!important;border-color:transparent!important}#scene,.lbl,.fare,#plus,.next,.bub,#tak,#mock,#pill,.stats .x{visibility:hidden!important}');
   await shot(`ui_${tn}.png`,CARD);
  }
 }
 // sprites (summer night page; characters do not depend on the background)
 await load(7,'21:00:00');
 const base0=T0+'.card{background:transparent!important;border-color:transparent!important}#bgB,#bgF,#ptcl,#trk0,#trkF,#goal,.trim,.top,.lbl,.fare,#plus,.stats,.next,.bub,#tak,.foot,#mock{visibility:hidden!important}';
 // neutral pose; anchor sprite at card (A,A) -> page (10+A,12+A)
 await ev(`(()=>{const P=document.getElementById('poro');P.setAttribute('transform','translate(${A} ${A}) scale(${0.72})');
  for(const i of ['ptail','pfL','pfR','paL','paR','peyes','pepu','pmouth'])document.getElementById(i)&&document.getElementById(i).setAttribute('transform','');
  const h=document.getElementById('phead');h.setAttribute('transform','translate(0 4) scale(1.3) translate(0 -4)');
  const B=document.getElementById('bono');B.setAttribute('transform','translate(${A+300} ${A}) scale(0.82)');
  document.getElementById('bsteer').style.display='none';document.getElementById('beyes').setAttribute('transform','');
  for(const i of ['barm','barmL'])document.getElementById(i).style.display='none';
  const g=document.createElementNS('http://www.w3.org/2000/svg','g');g.id='bakeShell';g.setAttribute('transform','translate(${A} ${A}) scale(0.82)');g.innerHTML=shell(ST,0,0,0,1.32);document.getElementById('scene').appendChild(g);
  document.querySelector('#scene g[clip-path]').removeAttribute('clip-path');
  return 1})()`);
 const SP={x:10,y:12,width:S,height:S},SB={x:10+300,y:12,width:S,height:S};
 const parts=['ptail','pfL','pfR','paL','paR'];
 const only=sel=>`#poro *{visibility:hidden!important}${sel},${sel} *{visibility:visible!important}`;
 await style(base0+'#bono,#bakeShell{visibility:hidden!important}'+parts.map(p=>`#${p},#${p} *`).join(',')+'{visibility:hidden!important}');await shot('porori_body.png',SP);
 for(const p of parts){await style(base0+'#bono,#bakeShell{visibility:hidden!important}'+only('#'+p));await shot(`porori_${p.slice(1)}.png`,SP);}
 await style(base0+'#poro,#bakeShell{visibility:hidden!important}');await shot('bono_body.png',SB);
 await style(base0+'#poro,#bono{visibility:hidden!important}');await shot('shell.png',SP);
 const pal=await ev(`JSON.stringify({bono:pal(ST,'bono'),poro:pal(ST,'poro')})`);fs.writeFileSync(path.join(outDir,'palette.json'),pal);
 const th=await ev(`JSON.stringify({theme:C1_THEME})`);fs.writeFileSync(path.join(outDir,'theme.json'),th||'{}');
 ws.close();ch.kill();process.exit(0);
})().catch(e=>{console.error(e);ch.kill();process.exit(1)});
