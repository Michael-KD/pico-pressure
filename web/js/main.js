'use strict';
/* ============ main loop ============ */
let lastEpochAt = -99999, lastWall = performance.now();
function step(dt){
  SIM.t += dt;
  /* beacon origination: window start each superframe (or timer when sched off).
     the phase test is widened by dt so a large step can't hop over the window */
  const due = SIM.schedOn
    ? (sfPhase(SIM.t) < Math.max(400, dt) && SIM.t - lastEpochAt > layout().period * .6)
    : (SIM.t - lastEpochAt > 6000);
  if (due){ lastEpochAt = SIM.t; beaconEpoch(); }
  /* queued window-gated sends */
  for (let i = queue.length - 1; i >= 0; i--){
    const q = queue[i];
    const g = gate(q.seg, q.due);
    if (g === null){ queue.splice(i,1); continue; }
    if (g > q.due) q.due = g + Math.random()*300;
    if (SIM.t >= q.due){ queue.splice(i,1); q.fire(SIM.t); }
  }
  /* flights */
  for (let i = flights.length - 1; i >= 0; i--){
    const f = flights[i];
    if (SIM.t >= f.eta){ flights.splice(i,1); if (!f.drop) deliver(f); }
  }
  pumpAcks();
  campaignWatch();
}
function frame(){
  const now = performance.now();
  const wall = Math.min(64, now - lastWall); lastWall = now;
  if (SIM.running) step(wall * SIM.speed);
  draw(); renderTimeline();
  $('chip-time').innerHTML = `t <b>${(SIM.t/1000).toFixed(1)}s</b>`;
  $('chip-sf').innerHTML = `sf <b>${SIM.schedOn ? sfIndex(SIM.t) : 'off'}</b>`;
  const attached = nodes.filter(n => n.id && n.alive && n.parent >= 0).length;
  const alive = nodes.filter(n => n.id && n.alive).length;
  $('chip-tree').innerHTML = `tree <b>${attached}/${alive}</b>`;
  $('chip-tree').className = 'chip ' + (attached === alive && alive ? 'on' : 'off');
  $('chip-drops').innerHTML = `mic drops <b>${SIM.micDrops}</b>`;
  requestAnimationFrame(frame);
}

/* ============ events ============ */
ui.run.onclick = () => { SIM.running = !SIM.running; ui.run.textContent = SIM.running?'Pause':'Run'; };
ui.speed.oninput = () => SIM.speed = Number(ui.speed.value);
ui.loss.oninput = () => $('kv-loss').innerHTML = `<b>${ui.loss.value}%</b>`;
ui.sched.onclick = () => {
  SIM.schedOn = !SIM.schedOn;
  ui.sched.textContent = 'Scheduler: ' + (SIM.schedOn ? 'on' : 'off');
  log(SIM.schedOn ? 'scheduler ON: windows gate traffic, radios sleep, MIC binds the superframe'
                  : 'scheduler OFF: free-running, replay protection dormant', 'hi');
};
ui.kill.onclick = () => {
  if (!selected || selected.id === 0){ log('select a field node first', 'err'); return; }
  selected.alive = !selected.alive;
  if (!selected.alive){ selected.parent = -1; selected.cost = Infinity; selected.nbr.clear();
    log(`node ${selected.id} killed — watch its children heal`, 'err'); }
  else log(`node ${selected.id} revived — re-probed fresh, hysteresis prevents flapping`, 'ok');
  ui.kill.textContent = selected.alive ? 'Kill node' : 'Revive node';
  renderChecklist();
};
ui.key.onclick = () => {
  if (!selected || selected.id === 0){ log('select a field node first', 'err'); return; }
  selected.wrongKey = !selected.wrongKey;
  log(`node ${selected.id} now has the ${selected.wrongKey?'WRONG':'fleet'} key` +
      (selected.wrongKey ? ' — its frames fail every MIC check' : ''), selected.wrongKey?'err':'ok');
};
$('b-arm').onclick = () => floodCmd('ARM', null);
$('b-disarm').onclick = () => floodCmd('DISARM', null);
$('b-status').onclick = () => floodCmd('STATUS', null);
$('b-resend').onclick = () => { if (lastCmd) floodCmd(lastCmd.op, lastCmd.target, lastCmd.id); };
$('b-fetch').onclick = () => {
  const t = selected && selected.id ? selected.id : 8;
  fx.auto = false; startFetch(t);
};
$('b-refetch').onclick = refetch;
$('b-campaign').onclick = startCampaign;
$('b-capture').onclick = captureFrame;
ui.replayBtn.onclick = replayFrame;

/* canvas interaction */
let dragging = null, downAt = null;
function evPos(ev){
  const r = ui.map.getBoundingClientRect();
  return { x: (ev.clientX - r.left) * (W / r.width), y: (ev.clientY - r.top) * (H / r.height) };
}
ui.map.addEventListener('pointerdown', ev => {
  const p = evPos(ev); downAt = p;
  dragging = nodes.find(n => Math.hypot(n.x-p.x, n.y-p.y) < 16) || null;
  if (dragging) ui.map.setPointerCapture(ev.pointerId);
});
ui.map.addEventListener('pointermove', ev => {
  if (!dragging) return;
  const p = evPos(ev);
  dragging.x = Math.max(18, Math.min(W-18, p.x));
  dragging.y = Math.max(18, Math.min(H-18, p.y));
});
ui.map.addEventListener('pointerup', ev => {
  const p = evPos(ev);
  if (dragging && downAt && Math.hypot(p.x-downAt.x, p.y-downAt.y) < 5){
    selected = dragging;
    ui.sel.innerHTML = 'selected: <b>' + (selected.id === 0 ? 'base' :
      'node ' + selected.id + (selected.parent>=0 ? ` (parent ${selected.parent}, ${selected.cost===Infinity?'∞':selected.cost.toFixed(2)} ETX, hop ${selected.hops})` : ' (unattached)')) + '</b>';
    ui.kill.textContent = selected.alive ? 'Kill node' : 'Revive node';
  }
  dragging = null;
});
ui.map.addEventListener('pointercancel', () => { dragging = null; downAt = null; });

/* boot */
log('fleet booted — beacons build the collection tree; try the buttons on the right', 'hi');
log('tip: kill a relay (click it, then Kill) and watch its branch re-route', '');
renderChecklist();
requestAnimationFrame(frame);
