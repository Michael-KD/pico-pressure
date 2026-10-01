'use strict';
/* ============ ui plumbing ============ */
const $ = id => document.getElementById(id);
const ui = {
  map: $('map'), run: $('b-run'), speed: $('b-speed'), loss: $('b-loss'),
  sched: $('b-sched'), kill: $('b-kill'), key: $('b-key'), sel: $('kv-sel'),
  mode: $('kv-mode'), replayBtn: $('b-replay'), logbox: $('logbox'),
};
let selected = null;
function log(msg, cls){
  const d = document.createElement('div');
  d.innerHTML = `<span class="t">${(SIM.t/1000).toFixed(1)}s</span> <span class="${cls||''}">${msg}</span>`;
  ui.logbox.appendChild(d);
  while (ui.logbox.childElementCount > 120) ui.logbox.firstChild.remove();
  ui.logbox.scrollTop = ui.logbox.scrollHeight;
}
function renderChecklist(){
  const el = $('checklist'); el.innerHTML = '';
  for (const [id, ok] of ackChecklist){
    const s = document.createElement('span');
    const dead = !nodes[id].alive;
    s.className = 'ck' + (ok ? ' yes' : '') + (dead ? ' dead' : '');
    s.textContent = id + (ok ? ' ✓' : '');
    el.appendChild(s);
  }
}
function renderBitmap(){
  const el = $('bitmap'); el.innerHTML = '';
  if (!fx.active) return;
  for (let i = 0; i < fx.nchunks; i++){
    const b = document.createElement('span');
    b.className = 'bit' + (fx.got.has(i) ? ' got' : '');
    el.appendChild(b);
  }
  $('kv-fetch').innerHTML = `<b>node ${fx.target}: ${fx.got.size}/${fx.nchunks}</b>`;
}
/* timeline */
function renderTimeline(){
  const bar = $('tlbar'); bar.innerHTML = '';
  const L = layout(); const P = L.period;
  const segs = [['beacon','--f-beacon',0,L.beacon],['cmd','--f-cmd',L.beacon,L.cmd],
    ['telem','--f-telem',L.beacon+L.cmd,L.telem],['bulk','--f-data',L.beacon+L.cmd+L.telem,L.bulk]];
  const css = getComputedStyle(document.documentElement);
  for (const [,v,off,w] of segs){
    if (!w) continue;
    const d = document.createElement('div');
    d.className = 'tl-seg';
    d.style.left = (off/P*100)+'%'; d.style.width = (w/P*100)+'%';
    d.style.background = css.getPropertyValue(v); d.style.opacity = .55;
    bar.appendChild(d);
  }
  if (SIM.schedOn){
    const c = document.createElement('div');
    c.className = 'tl-cursor'; c.style.left = (sfPhase(SIM.t)/P*100)+'%';
    bar.appendChild(c);
  }
}

/* ============ canvas ============ */
const ctx = ui.map.getContext('2d');
function cssv(name){ return getComputedStyle(document.documentElement).getPropertyValue(name).trim(); }
function draw(){
  const asleep = SIM.schedOn && segAt(SIM.t) === 'sleep';
  ctx.clearRect(0,0,W,H);
  /* faint dune contours */
  ctx.save(); ctx.globalAlpha = .07; ctx.strokeStyle = cssv('--accent'); ctx.lineWidth = 1;
  for (let i=0;i<5;i++){ ctx.beginPath();
    for (let x=0;x<=W;x+=12){ const y = 80+i*85 + Math.sin(x*0.011+i*2.1)*16 + Math.sin(x*0.027+i)*7;
      x?ctx.lineTo(x,y):ctx.moveTo(x,y); }
    ctx.stroke(); }
  ctx.restore();
  /* links */
  for (const a of nodes) for (const b of nodes){
    if (a.id >= b.id || !a.alive || !b.alive) continue;
    const lp = lossPct(a,b); if (lp > 100) continue;
    ctx.strokeStyle = cssv('--line'); ctx.globalAlpha = .8 - lp/160; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(a.x,a.y); ctx.lineTo(b.x,b.y); ctx.stroke();
    ctx.globalAlpha = 1;
  }
  /* tree edges */
  for (const n of nodes){
    if (n.parent < 0 || !n.alive) continue;
    const p = nodes[n.parent]; if (!p || !p.alive) continue;
    ctx.strokeStyle = cssv('--accent'); ctx.lineWidth = 1.8; ctx.globalAlpha = .9;
    ctx.beginPath(); ctx.moveTo(n.x,n.y); ctx.lineTo(p.x,p.y); ctx.stroke();
    ctx.globalAlpha = 1;
    const mx = (n.x+p.x)/2, my = (n.y+p.y)/2, ang = Math.atan2(p.y-n.y, p.x-n.x);
    ctx.save(); ctx.translate(mx,my); ctx.rotate(ang);
    ctx.fillStyle = cssv('--accent');
    ctx.beginPath(); ctx.moveTo(5,0); ctx.lineTo(-3,-4); ctx.lineTo(-3,4); ctx.fill();
    ctx.restore();
  }
  /* frames */
  for (const f of flights){
    const k = Math.min(1, (SIM.t - f.t0) / (f.eta - f.t0));
    const x = f.from.x + (f.to.x - f.from.x)*k, y = f.from.y + (f.to.y - f.from.y)*k;
    ctx.fillStyle = cssv(FCOL[f.type] || '--muted');
    ctx.globalAlpha = f.drop ? .35 : 1;
    ctx.beginPath(); ctx.arc(x,y,f.type==='hopack'?2.2:3.4,0,7); ctx.fill();
    if (f.drop && k > .55){ ctx.strokeStyle = cssv('--bad'); ctx.lineWidth=1; ctx.globalAlpha=.6;
      ctx.beginPath(); ctx.moveTo(x-4,y-4); ctx.lineTo(x+4,y+4); ctx.moveTo(x+4,y-4); ctx.lineTo(x-4,y+4); ctx.stroke(); }
    ctx.globalAlpha = 1;
  }
  /* nodes */
  for (const n of nodes){
    const dim = !n.alive ? .28 : (asleep && n.id !== 0 ? .45 : 1);
    ctx.globalAlpha = dim;
    const r = 13;
    ctx.fillStyle = cssv('--panel');
    ctx.strokeStyle = n === selected ? cssv('--ink') :
      (n.flash && SIM.t < n.flash.until ? cssv('--'+n.flash.color) :
       (n.wrongKey ? cssv('--bad') : (n.armed ? cssv('--warn') : cssv('--muted'))));
    ctx.lineWidth = n === selected ? 2.4 : 1.6;
    ctx.beginPath();
    if (n.id === 0) ctx.rect(n.x-r, n.y-r, r*2, r*2); else ctx.arc(n.x, n.y, r, 0, 7);
    ctx.fill(); ctx.stroke();
    ctx.fillStyle = cssv('--ink');
    ctx.font = '600 11px ' + cssv('--mono');
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText(n.id === 0 ? 'B' : String(n.id), n.x, n.y+0.5);
    if (!n.alive){ ctx.strokeStyle = cssv('--bad'); ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(n.x-9,n.y-9); ctx.lineTo(n.x+9,n.y+9);
      ctx.moveTo(n.x+9,n.y-9); ctx.lineTo(n.x-9,n.y+9); ctx.stroke(); }
    ctx.globalAlpha = 1;
  }
}
