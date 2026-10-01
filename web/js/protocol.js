'use strict';
/* ============ protocol: beacons & tree (§5) ============ */
let epochSeq = 0;
function beaconEpoch(){
  epochSeq++;
  /* every node scores what it heard LAST epoch before the new wave */
  for (const n of nodes){ if (n.id !== 0) scoreEpoch(n); }
  enqueue('beacon', () => {
    tx(nodes[0], null, 'beacon', { seq: epochSeq, cost: 0, hops: 0, origin: 0 });
  }, 0);
}
function scoreEpoch(n){
  for (const [id, e] of n.nbr){
    const ewma = 0.30;
    e.etx = e.heard ? e.etx*(1-ewma) + 1*ewma : Math.min(16, e.etx*(1-ewma) + 8*ewma);
    if (!e.heard) e.missed++; else e.missed = 0;
    e.heard = false;
    if (e.missed >= 4){ n.nbr.delete(id); if (n.parent === id) parentDead(n); }
  }
  evaluate(n);
}
function onBeacon(n, f){
  let e = n.nbr.get(f.from.id);
  if (!e){ e = { etx: 1.3, adv: f.payload.cost, hops: f.payload.hops, heard: true, missed: 0 };
           n.nbr.set(f.from.id, e); }
  e.heard = true; e.adv = f.payload.cost; e.hops = f.payload.hops;
  evaluate(n);
  /* best-beacon rebroadcast: once per seq, only if attached */
  if (n.rebc < f.payload.seq && n.cost < Infinity){
    n.rebc = f.payload.seq;
    const seq = f.payload.seq;
    enqueue('beacon', () => {
      if (n.alive && n.cost < Infinity)
        tx(n, null, 'beacon', { seq, cost: n.cost, hops: n.hops, origin: n.id });
    }, 120 + Math.random()*900);
  }
}
function evaluate(n){
  if (n.id === 0) return;
  let best = -1, bestTotal = Infinity;
  for (const [id, e] of n.nbr){
    if (e.adv >= n.cost) continue;               /* loop guard: never pick downstream */
    const total = e.adv + e.etx;
    if (total < bestTotal){ bestTotal = total; best = id; }
  }
  const cur = n.nbr.get(n.parent);
  const curTotal = cur ? cur.adv + cur.etx : Infinity;
  if (n.parent < 0 || !cur){
    if (best >= 0) attach(n, best, bestTotal);
    return;
  }
  /* hysteresis: challenger must win by 0.5 ETX for 2 epochs */
  if (best >= 0 && best !== n.parent && bestTotal + 0.5 < curTotal){
    if (n.cand === best) n.candEpochs++; else { n.cand = best; n.candEpochs = 1; }
    if (n.candEpochs >= 2) attach(n, best, bestTotal);
  } else { n.cand = -1; n.candEpochs = 0; n.cost = curTotal; n.hops = (cur.hops||0)+1; }
}
function attach(n, pid, total){
  const old = n.parent;
  n.parent = pid; n.cost = total;
  n.hops = (n.nbr.get(pid).hops || 0) + 1;
  n.parentFail = 0; n.cand = -1; n.candEpochs = 0;
  if (old !== pid) log(`node ${n.id}: parent ${old<0?'—':old} → ${pid} (${total.toFixed(2)} ETX)`);
}
function parentDead(n){
  if (n.parent < 0) return;
  log(`node ${n.id}: parent ${n.parent} dead — re-selecting`, 'err');
  n.nbr.delete(n.parent); n.parent = -1; n.cost = Infinity; n.parentFail = 0;
  evaluate(n);
}

/* ============ per-hop reliable unicast up the tree (§6.4) ============ */
const pendingAcks = [];
function upSend(n, type, payload, seg){
  if (n.parent < 0){ return false; }
  enqueue(seg || 'any', () => {
    if (!n.alive || n.parent < 0) return;
    const parent = nodes[n.parent];
    const fr = { seq: seqCounter++ };
    tx(n, parent, type, payload, fr);
    pendingAcks.push({ from: n, to: parent, seq: fr.seq, type, payload, seg,
                       deadline: SIM.t + 1400, tries: 0 });
  }, Math.random()*250);
  return true;
}
function pumpAcks(){
  for (let i = pendingAcks.length - 1; i >= 0; i--){
    const pa = pendingAcks[i];
    if (SIM.t < pa.deadline) continue;
    pendingAcks.splice(i, 1);
    if (pa.tries < 3){
      const g = gate(pa.seg || 'any', SIM.t);
      if (g !== null){
        queue.push({ seg: pa.seg||'any', due: g + Math.random()*220, fire: () => {
          if (!pa.from.alive || pa.from.parent < 0) return;
          const parent = nodes[pa.from.parent];
          const fr = { seq: seqCounter++ };
          tx(pa.from, parent, pa.type, pa.payload, fr);
          pendingAcks.push({ ...pa, to: parent, seq: fr.seq,
                             deadline: SIM.t + 1400, tries: pa.tries+1 });
        }});
      }
    } else {
      pa.from.parentFail++;
      log(`node ${pa.from.id}: hop to ${pa.to.id} exhausted (${pa.from.parentFail}/3)`, 'err');
      if (pa.from.parentFail >= 3) parentDead(pa.from);
      /* a lost chunk leaves a gap in the bitmap; keep the rest of the session moving */
      if (pa.type === 'data' && pa.from.bulk){ pa.from.bulk.pending = false; bulkPump(pa.from); }
    }
  }
}
function onHopAck(f){
  for (let i = pendingAcks.length - 1; i >= 0; i--){
    const pa = pendingAcks[i];
    if (pa.seq === f.payload.ackSeq && pa.to === f.from){
      pendingAcks.splice(i, 1);
      pa.from.parentFail = 0;
      if (pa.type === 'data' && pa.from.bulk){ pa.from.bulk.pending = false; bulkPump(pa.from); }
    }
  }
}

/* ============ commands (§6) ============ */
let cmdId = 0, lastCmd = null;
const ackChecklist = new Map();
function floodCmd(op, target, reuse){
  const id = reuse || ++cmdId;
  const seq = seqCounter++;
  lastCmd = { op, target, id };
  if (!reuse){ ackChecklist.clear(); for (const n of nodes) if (n.id) ackChecklist.set(n.id, false); }
  enqueue('cmd', () => tx(nodes[0], null, 'cmd', { op, target, id, seq }), 0);
  log(`flood ${op}${target!=null?(' → node '+target):''} (cmd ${id})`, 'hi');
  renderChecklist();
}
function onCmd(n, f){
  const pl = f.payload;
  if (n.seenFloods.has(pl.seq)) return;
  n.seenFloods.add(pl.seq);
  /* relay */
  enqueue('cmd', () => { if (n.alive) tx(n, null, 'cmd', pl, { seq: seqCounter++ }); },
          150 + Math.random()*800);
  if (pl.target != null && pl.target !== n.id) return;
  if (!n.executed.has(pl.id)){
    n.executed.add(pl.id);
    if (pl.op === 'ARM') n.armed = true;
    if (pl.op === 'DISARM') n.armed = false;
  }
  upSend(n, 'ack', { origin: n.id, id: pl.id }, 'cmd');
  if (pl.op === 'STATUS') upSend(n, 'telem', { origin: n.id, parent: n.parent, armed: n.armed }, 'telem');
}
function onUpFrame(rx, f){
  /* hop-ack the link, dedup, then deliver or relay (§6.4) */
  tx(rx, f.from, 'hopack', { ackSeq: f.seq });
  if (rx.id === 0){
    if (f.type === 'ack'){
      if (ackChecklist.has(f.payload.origin)){ ackChecklist.set(f.payload.origin, true); renderChecklist(); }
    } else if (f.type === 'telem'){
      log(`TELEM node ${f.payload.origin} (parent ${f.payload.parent}, ${f.payload.armed?'ARMED':'standby'})`);
    } else if (f.type === 'data'){
      bulkDeliver(f.payload);
    }
    return;
  }
  upSend(rx, f.type, f.payload, f.type === 'data' ? 'bulk' : (f.type === 'telem' ? 'telem' : 'cmd'));
}

/* ============ bulk (§7) ============ */
const EVENT_LEN = 1152, CHUNK = 96;
const fx = { active:false, target:-1, session:0, nchunks:0, got:new Set(), auto:false };
let nextSession = 1;
function rootFetch(target, offset, len){
  if (SIM.schedOn && !layout().bulk){ log(`fetch: no bulk window in '${SIM.mode}' mode — fails fast`, 'err'); return false; }
  const seq = seqCounter++;
  enqueue('cmd', () => tx(nodes[0], null, 'session',
    { target, session: fx.session, offset, len, chunk: CHUNK, seq }), 0);
  return true;
}
function startFetch(target){
  fx.active = true; fx.target = target; fx.session = nextSession++;
  fx.nchunks = Math.ceil(EVENT_LEN / CHUNK); fx.got.clear();
  renderBitmap();
  log(`fetch: session ${fx.session}, ${EVENT_LEN} B in ${fx.nchunks}×${CHUNK} B from node ${target}`, 'hi');
  return rootFetch(target, 0, EVENT_LEN);
}
function refetch(){
  if (!fx.active) return;
  let first = 0; while (fx.got.has(first)) first++;
  if (first >= fx.nchunks){ log('fetch already complete', 'ok'); return; }
  log(`refetch from chunk ${first}`, 'hi');
  rootFetch(fx.target, first*CHUNK, EVENT_LEN - first*CHUNK);
}
function onSession(n, f){
  const pl = f.payload;
  if (n.seenFloods.has(pl.seq)) return;
  n.seenFloods.add(pl.seq);
  enqueue('cmd', () => { if (n.alive) tx(n, null, 'session', pl, { seq: seqCounter++ }); },
          150 + Math.random()*800);
  if (pl.target !== n.id) return;
  n.bulk = { session: pl.session, cursor: pl.offset, end: pl.offset + pl.len,
             chunk: pl.chunk, pending: false };
  bulkPump(n);
}
function bulkPump(n){
  const b = n.bulk;
  if (!b || b.pending) return;
  if (b.cursor >= b.end || b.cursor >= EVENT_LEN){ n.bulk = null; return; }
  const idx = Math.floor(b.cursor / b.chunk);
  const len = Math.min(b.chunk, EVENT_LEN - b.cursor, b.end - b.cursor);
  if (!upSend(n, 'data', { origin: n.id, session: b.session, idx, len }, 'bulk')) return;
  b.pending = true;
  b.cursor += len;
}
function bulkDeliver(pl){
  if (!fx.active || pl.session !== fx.session) return;
  if (!fx.got.has(pl.idx)){
    fx.got.add(pl.idx); renderBitmap();
    if (fx.got.size === fx.nchunks){
      log(`fetch complete: ${EVENT_LEN} B from node ${fx.target}, CRC verified`, 'ok');
      if (fx.auto) campaignNext();
    }
  }
}
/* campaign (M6) */
const camp = { on:false, targets:[], i:0, timer:0, stalls:0 };
function startCampaign(){
  if (camp.on) return;
  camp.targets = nodes.filter(n => n.id && n.alive).map(n => n.id)
    .sort((a,b) => nodes[a].hops - nodes[b].hops);
  if (!camp.targets.length) return;
  camp.on = true; camp.i = 0;
  if (SIM.schedOn && !layout().bulk){ setMode('retrieval'); log('campaign: fleet → retrieval mode', 'hi'); }
  else if (SIM.schedOn) log('campaign: bulk window present — walking nodes', 'hi');
  fx.auto = true;
  startFetch(camp.targets[0]);
  camp.timer = SIM.t;
}
function campaignNext(){
  camp.i++; camp.stalls = 0;
  if (camp.i >= camp.targets.length){
    camp.on = false; fx.auto = false;
    if (SIM.schedOn && SIM.mode === 'retrieval'){ setMode('active'); log('campaign done: fleet → active mode', 'ok'); }
    else log('campaign done', 'ok');
    return;
  }
  startFetch(camp.targets[camp.i]);
  camp.timer = SIM.t;
}
function campaignWatch(){
  if (!camp.on) return;
  if (SIM.t - camp.timer > 2.2 * layout().period){
    camp.timer = SIM.t;
    if (fx.got.size < fx.nchunks){
      if (++camp.stalls > 6){
        log(`campaign: node ${camp.targets[camp.i]} INCOMPLETE after retry budget — moving on`, 'err');
        campaignNext();
      } else refetch();
    }
  }
}
function setMode(m){ SIM.mode = m; ui.mode.innerHTML = 'mode: <b>'+m+'</b>'; }

/* ============ replay capture (§10.2) ============ */
let captured = null;
function captureFrame(){
  const cand = flights.filter(f => !f.drop && f.type !== 'hopack');
  const f = cand[cand.length-1];
  if (!f){ log('nothing on air to capture — try during a flood', 'err'); return; }
  captured = { ...f };
  ui.replayBtn.disabled = false;
  log(`captured a ${f.type.toUpperCase()} frame sealed in sf ${f.sfSealed} — replay it later`, 'hi');
}
function replayFrame(){
  if (!captured) return;
  const f = { ...captured, t0: SIM.t, eta: SIM.t + 400, drop: false, replayed: true };
  flights.push(f);
  const d = SIM.schedOn ? Math.abs(sfIndex(SIM.t) - f.sfSealed) : 0;
  log(`replaying the ${f.type.toUpperCase()} frame (${d} superframes stale)…`, 'hi');
}

/* ============ delivery ============ */
function deliver(f){
  const rx = f.to;
  if (!rx.alive) return;
  if (!listening(SIM.t)) { return; }             /* radio asleep */
  if (!micOk(f, SIM.t)){
    SIM.micDrops++;
    rx.flash = { color: 'bad', until: SIM.t + 900 };
    log(`node ${rx.id}: MIC drop (${f.replayed ? 'stale superframe — replay rejected' : 'wrong key'})`, f.replayed ? 'ok' : 'err');
    return;
  }
  if (f.replayed){
    log(`node ${rx.id}: replayed frame ACCEPTED — this is the §10.2 gap when the scheduler is off`, 'err');
    rx.flash = { color: 'warn', until: SIM.t + 900 };
    return; /* don't actually re-execute in the model */
  }
  switch (f.type){
    case 'beacon': if (rx.id !== 0) onBeacon(rx, f); break;
    case 'cmd': if (rx.id !== 0) onCmd(rx, f); break;
    case 'session': if (rx.id !== 0) onSession(rx, f); break;
    case 'hopack': onHopAck(f); break;
    case 'ack': case 'telem': case 'data': onUpFrame(rx, f); break;
  }
}
