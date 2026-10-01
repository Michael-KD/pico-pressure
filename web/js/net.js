'use strict';
/* ============ nodes ============ */
const W = 720, H = 470, RANGE = 205;
const nodes = [];
function mkNode(id, x, y){
  return {
    id, x, y, alive: true, wrongKey: false, armed: false,
    parent: id === 0 ? -2 : -1, cost: id === 0 ? 0 : Infinity,
    hops: id === 0 ? 0 : 99,
    nbr: new Map(),          /* id -> {etx, adv, hops, heard, missed} */
    cand: -1, candEpochs: 0, parentFail: 0,
    seenFloods: new Set(), executed: new Set(), rebc: -1,
    bulk: null,              /* {session, cursor, end, chunk, pending} */
  };
}
/* a desert-ish default layout: base west, two branches east */
[[0,70,235],[1,205,130],[2,210,345],[3,360,90],[4,355,255],[5,352,415],
 [6,505,150],[7,510,330],[8,650,240]].forEach(([i,x,y]) => nodes.push(mkNode(i,x,y)));

function dist(a,b){ return Math.hypot(a.x-b.x, a.y-b.y); }
function lossPct(a,b){
  const d = dist(a,b);
  if (d > RANGE) return 101;
  const base = Math.max(0, (d - 70) / (RANGE - 70)) * 30;   /* 0..30 by distance */
  return Math.min(97, base + Number(ui.loss.value));
}
function neighborsOf(n){ return nodes.filter(o => o !== n && o.alive && lossPct(n,o) <= 100); }

/* ============ frames in flight ============ */
const FCOL = { beacon:'--f-beacon', cmd:'--f-cmd', ack:'--f-ack', telem:'--f-telem',
               hopack:'--f-hop', data:'--f-data', session:'--f-session' };
const flights = [];   /* {from,to,type,payload,seq,sfSealed,wrongKey,t0,eta,drop} */
let seqCounter = 1;
function micOk(frame, rxT){
  /* modeled MIC: fleet key + superframe index sealed at tx (§10.2) */
  const key = frame.wrongKey ? 'K1' : 'K0';
  if (key !== 'K0') return false;
  if (!SIM.schedOn) return true;
  const now = sfIndex(rxT);
  return Math.abs(now - frame.sfSealed) <= 1;
}
function tx(from, to, type, payload, opts){
  /* to === null => broadcast to all in range */
  const o = opts || {};
  const targets = to === null ? neighborsOf(from) : [to];
  SIM.framesTx++;
  for (const t of targets){
    const lp = lossPct(from, t);
    const fl = {
      from, to: t, type, payload, seq: o.seq ?? seqCounter++,
      sfSealed: sfIndex(SIM.t), wrongKey: from.wrongKey,
      t0: SIM.t, eta: SIM.t + 260 + dist(from,t) * 0.55,
      drop: Math.random()*100 < lp,
    };
    flights.push(fl);
  }
  return true;
}

/* queued sends waiting for their window */
const queue = [];   /* {seg, fire(t), due} */
function enqueue(seg, fire, jitter){
  const g = gate(seg, SIM.t);
  if (g === null){ log(`window '${seg}' never opens in ${SIM.mode} — dropped`, 'err'); return false; }
  queue.push({ seg, fire, due: g + (jitter || 0) });
  return true;
}
