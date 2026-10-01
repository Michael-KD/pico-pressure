'use strict';
/* ============ virtual time & superframe schedule ============ */
const SIM = {
  t: 0, running: true, speed: 3,
  schedOn: true, mode: 'active',
  micDrops: 0, framesTx: 0,
};
/* superframe layouts, ms of virtual time (scaled-down §4) */
const MODES = {
  active:    { period: 12000, beacon: 2200, cmd: 2600, telem: 3000, bulk: 1800 },
  retrieval: { period: 12000, beacon: 2000, cmd: 2000, telem: 1000, bulk: 6600 },
  standby:   { period: 36000, beacon: 2200, cmd: 2600, telem: 3000, bulk: 0 },
};
function layout(){ return MODES[SIM.mode]; }
function sfIndex(t){ return Math.floor(t / layout().period); }
function sfPhase(t){ return t % layout().period; }
function segAt(t){
  const L = layout(); let p = sfPhase(t);
  if (p < L.beacon) return 'beacon'; p -= L.beacon;
  if (p < L.cmd) return 'cmd'; p -= L.cmd;
  if (p < L.telem) return 'telem'; p -= L.telem;
  if (p < L.bulk) return 'bulk';
  return 'sleep';
}
/* earliest time >= t when segment `seg` is open; null if zero-width */
function gate(seg, t){
  if (!SIM.schedOn || seg === 'any') return t;
  const L = layout();
  const offs = { beacon: 0, cmd: L.beacon, telem: L.beacon + L.cmd,
                 bulk: L.beacon + L.cmd + L.telem };
  const width = L[seg];
  if (!width) return null;                       /* never opens: drop */
  const base = Math.floor(t / L.period) * L.period + offs[seg];
  if (t >= base && t < base + width) return t;
  return t < base ? base : base + L.period;
}
function listening(t){ return !SIM.schedOn || segAt(t) !== 'sleep'; }
