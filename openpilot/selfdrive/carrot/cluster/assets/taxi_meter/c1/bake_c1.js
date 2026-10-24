// Bake C1 (variants/C1.html, read-only) into PNG layers for the raylib K9ExtHud=1 renderer.
// node bake_c1.js <variantsDirUrl> <outDir>   (WORKER only)
// Layers: back_{season}_{tod}.png, front_{season}_{tod}.png (776x456, transparent outside the card),
//         ui_{tod}.png (static text/frame overlay), sprites (porori_*, bono_body, shell, acorn) 240x240 anchor at (120,120).
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
 const ANIM=[];const TK={day:'D',dusk:'S',night:'N'};
 const HIDEA='.c1anim,.c1anim *{visibility:hidden!important}';
 for(const [sn,mo] of Object.entries(SEAS))for(const [tn,ck] of Object.entries(TOD)){
  await load(mo,ck);
  // r3: the elements c1bg.js draw() animates (first 14 data-tw = twinkle opacity, all data-dr = drift x) are left out
  // of back_/front_ and baked one by one (opacity 1, no transform) so the renderer can animate them.
  const info=JSON.parse(await ev(`(()=>{const k='${TK[tn]}',g=document.getElementById('bg'+k),f=document.getElementById('f'+k);
   const tw=[...g.querySelectorAll('[data-tw]'),...f.querySelectorAll('[data-tw]')].slice(0,14),dr=[...g.querySelectorAll('[data-dr]'),...f.querySelectorAll('[data-dr]')];
   const cr=document.getElementById('card').getBoundingClientRect();const out=[];
   const add=(e,kind,i,v)=>{e.classList.add('c1anim');e.setAttribute('data-ai',out.length);if(kind==='tw')e.setAttribute('opacity','1');else e.removeAttribute('transform');
    const r=e.getBoundingClientRect();out.push({kind,i,v,part:g.contains(e)?'back':'front',tag:e.tagName,x:r.left-cr.left,y:r.top-cr.top,w:r.width,h:r.height})};
   tw.forEach((e,i)=>add(e,'tw',i,+e.dataset.tw));dr.forEach((e,i)=>add(e,'dr',i,+e.dataset.dr));return JSON.stringify(out)})()`));
  await style(T0+HIDEA+'#ptcl,#trk0,#trkF,#goal,#poro,#bono,#bgF,.trim,.top,.lbl,.fare,#plus,.stats,.next,.bub,#tak,.foot,#mock{visibility:hidden!important}');
  await shot(`back_${sn}_${tn}.png`,CARD);
  await style(T0+HIDEA+'.card{background:transparent!important;border-color:transparent!important}#bgB,#ptcl,#trk0,#trkF,#goal,#poro,#bono,.trim,.top,.lbl,.fare,#plus,.stats,.next,.bub,#tak,.foot,#mock{visibility:hidden!important}');
  await shot(`front_${sn}_${tn}.png`,CARD);
  for(let j=0;j<info.length;j++){const a=info[j];
   const x0=Math.max(0,Math.floor(a.x-3)),y0=Math.max(0,Math.floor(a.y-3)),x1=Math.min(776,Math.ceil(a.x+a.w+3)),y1=Math.min(456,Math.ceil(a.y+a.h+3));
   if(x1<=x0||y1<=y0)continue;
   await style(T0+`.card{background:transparent!important;border-color:transparent!important}#card *{visibility:hidden!important}#card [data-ai="${j}"],#card [data-ai="${j}"] *{visibility:visible!important}`);
   const file=`anim_${sn}_${tn}_${j}.png`;await shot(file,{x:CARD.x+x0,y:CARD.y+y0,width:x1-x0,height:y1-y0});
   ANIM.push(Object.assign({season:sn,tod:tn,file,cx:x0,cy:y0,cw:x1-x0,ch:y1-y0},a));}
  if(sn==='summer'){
   await style(T0+'.card{background:transparent!important;border-color:transparent!important}#scene,.lbl,.fare,#plus,.next,.bub,#tak,#mock,#pill,.stats .x{visibility:hidden!important}');
   await shot(`ui_${tn}.png`,CARD);
  }
 }
 fs.writeFileSync(path.join(outDir,'anim_raw.json'),JSON.stringify(ANIM,null,1));
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
 // res 10/24: goal 도토리 = acorn() of C1.html, scale 1 (drawn at scale 1 at GOAL)
 await ev(`(()=>{const g=document.createElementNS('http://www.w3.org/2000/svg','g');g.id='bakeAcorn';g.innerHTML=acorn(ST,${A},${A},0,1);document.getElementById('scene').appendChild(g);return 1})()`);
 await style(T0+'.card{background:transparent!important;border-color:transparent!important}#card *{visibility:hidden!important}#bakeAcorn,#bakeAcorn *{visibility:visible!important}');await shot('acorn.png',SP);
 const pal=await ev(`JSON.stringify({bono:pal(ST,'bono'),poro:pal(ST,'poro')})`);fs.writeFileSync(path.join(outDir,'palette.json'),pal);
 const th=await ev(`JSON.stringify({theme:C1_THEME})`);fs.writeFileSync(path.join(outDir,'theme.json'),th||'{}');
 ws.close();ch.kill();process.exit(0);
})().catch(e=>{console.error(e);ch.kill();process.exit(1)});
