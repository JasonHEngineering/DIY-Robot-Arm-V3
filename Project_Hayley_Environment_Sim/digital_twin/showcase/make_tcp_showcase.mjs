// TCP showcase path generator for Project Hayley V3
//   node make_tcp_showcase.mjs [--L 30] [--A 30] [--step 5] [--astep 5] [--minx 60] [--minz 80] [--q0 a1,a2,a3,a4,a5,a6]
//
// Sequence (all about the start TCP pose):
//   1-3. translate along base X, Y, Z:   0 -> +L -> -L -> 0   (tool orientation held)
//   4-6. rotate about the TCP point:     roll, pitch, yaw  0 -> +A -> -A -> 0  (tool tip held still)
// Roll/pitch/yaw are about the TOOL's own axes (x = approach along axis 6, y, z), like the
// Euler (Roll/Pitch/Yaw) columns in the Dash recordings.
//
// Every waypoint is solved with the twin's IK (kin.js, full 6D pose, continuing from the previous
// solution) and checked for: IK error, joint limits, singularity margins (smallest singular value of
// the Jacobian, |A5| wrist singularity, wrist-centre distance from the A1 axis), joint jump per row,
// and how far the TCP strays between rows (the ESP32 interpolates joints, not Cartesian).
// Writes recorded-table CSVs for the Dash Path Plan page + a JSON report.
import { writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

const HERE = dirname(fileURLToPath(import.meta.url));
const K = await import(new URL('../../Python%20program%20loaded%20on%20Raspberry%20Pi%204/twin_web/kin.js', import.meta.url).href);
const LIM = [[-100, 100], [-90, 90], [-90, 90], [-120, 120], [-100, 100], [-179.9, 179.9]];
const arg = (n, d) => { const i = process.argv.indexOf('--' + n); return i > 0 ? parseFloat(process.argv[i + 1]) : d; };
const L = arg('L', 30), A = arg('A', 30), STEP = arg('step', 5), ASTEP = arg('astep', 5);
const MINX = arg('minx', 60), MINZ = arg('minz', 80);
const q0Arg = (() => { const i = process.argv.indexOf('--q0'); return i > 0 ? process.argv[i + 1].split(',').map(Number) : null; })();

// ---------- small 3x3 / 4x4 helpers (row-major Float64Array(16) like kin.js) ----------
const rotAxis = (u, deg) => {
  const t = deg * Math.PI / 180, c = Math.cos(t), s = Math.sin(t), C = 1 - c, [x, y, z] = u;
  return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s], [y * x * C + z * s, c + y * y * C, y * z * C - x * s], [z * x * C - y * s, z * y * C + x * s, c + z * z * C]];
};
const R3 = T => [[T[0], T[1], T[2]], [T[4], T[5], T[6]], [T[8], T[9], T[10]]];
const mul3 = (a, b) => a.map((r, i) => [0, 1, 2].map(j => r[0] * b[0][j] + r[1] * b[1][j] + r[2] * b[2][j]));
const toT = (R, p) => Float64Array.from([R[0][0], R[0][1], R[0][2], p[0], R[1][0], R[1][1], R[1][2], p[1], R[2][0], R[2][1], R[2][2], p[2], 0, 0, 0, 1]);
const angBetween = (Ra, Rb) => {         // rotation angle between two orientations (deg)
  const tr = [0, 1, 2].reduce((s, i) => s + [0, 1, 2].reduce((t, k) => t + Ra[k][i] * Rb[k][i], 0), 0);
  return Math.acos(Math.max(-1, Math.min(1, (tr - 1) / 2))) * 180 / Math.PI;
};

// ---------- singularity metrics ----------
function jac(q) {
  const k = K.fk(q), p = K.pos(k.tcp), zs = k.frames.map(F => K.col(F, 2)), os = k.frames.map(K.pos);
  const cr = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const J = [[], [], [], [], [], []];
  for (let j = 0; j < 6; j++) { const v = cr(zs[j], [p[0] - os[j][0], p[1] - os[j][1], p[2] - os[j][2]]); for (let r = 0; r < 3; r++) { J[r][j] = v[r]; J[r + 3][j] = zs[j][r] * 100; } }
  return J;
}
function sminOf(q) {                     // smallest singular value (mm per rad, orientation scaled 100 mm/rad)
  const J = jac(q), n = 6;
  const Am = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => J.reduce((s, row) => s + row[i] * row[j], 0)));
  for (let sw = 0; sw < 60; sw++) for (let p = 0; p < n; p++) for (let r = p + 1; r < n; r++) {
    if (Math.abs(Am[p][r]) < 1e-12) continue;
    const th = (Am[r][r] - Am[p][p]) / (2 * Am[p][r]), t = Math.sign(th || 1) / (Math.abs(th) + Math.sqrt(th * th + 1));
    const c = 1 / Math.sqrt(t * t + 1), s = t * c;
    for (let k = 0; k < n; k++) { const a = Am[k][p], b = Am[k][r]; Am[k][p] = c * a - s * b; Am[k][r] = s * a + c * b; }
    for (let k = 0; k < n; k++) { const a = Am[p][k], b = Am[r][k]; Am[p][k] = c * a - s * b; Am[r][k] = s * a + c * b; }
  }
  return Math.sqrt(Math.max(0, Math.min(...Am.map((r, i) => r[i]))));
}
const wristR = q => { const w = K.pos(K.fk(q).frames[4]); return Math.hypot(w[0], w[1]); };   // J5 = wrist centre
const margin = q => Math.min(...q.map((v, i) => Math.min(v - LIM[i][0], LIM[i][1] - v)));

// ---------- path definition ----------
function ramp(amp, step) {               // 0 -> +amp -> -amp -> 0, sampled every `step`
  const out = [], n1 = Math.round(amp / step);
  for (let k = 1; k <= n1; k++) out.push(k * step);
  for (let k = n1 - 1; k >= -n1; k--) out.push(k * step);
  for (let k = -n1 + 1; k <= 0; k++) out.push(k * step);
  return out;
}
function buildTargets(q0) {
  const T0 = K.fk(q0).tcp, p0 = K.pos(T0), R0 = R3(T0), out = [];
  ['X', 'Y', 'Z'].forEach((ax, i) => ramp(L, STEP).forEach(d => {
    const p = p0.slice(); p[i] += d; out.push({ seg: `translate ${ax}`, val: d, p, R: R0 });
  }));
  const toolAxes = { roll: [1, 0, 0], pitch: [0, 1, 0], yaw: [0, 0, 1] };
  Object.entries(toolAxes).forEach(([name, u]) => ramp(A, ASTEP).forEach(d => {
    out.push({ seg: name, val: d, p: p0.slice(), R: mul3(R0, rotAxis(u, d)) });   // intrinsic (tool frame)
  }));
  return { p0, R0, targets: out };
}
function solve(q0) {
  const { p0, R0, targets } = buildTargets(q0);
  let q = q0.slice();
  const rows = [], worst = { ikPos: 0, ikOri: 0, jump: 0, jumpAt: '', smin: Infinity, sminAt: '', wrist: Infinity, wristAt: '', wristR: Infinity, margin: Infinity, strayPos: 0, strayOri: 0, strayAt: '' };
  for (const t of targets) {
    const tgt = { p: t.p, R: toT(t.R, t.p) };
    const qn = K.correct(q, 'pose', tgt, LIM, 300);
    const k = K.fk(qn), e = K.taskError(k, 'pose', tgt);
    // joint-space interpolation between rows: how far does the TCP stray from the intended motion?
    for (const f of [0.25, 0.5, 0.75]) {
      const qm = q.map((v, i) => v + (qn[i] - v) * f), Tm = K.fk(qm).tcp, pm = K.pos(Tm);
      const prev = rows.length ? rows[rows.length - 1].tgt : { p: p0, R: R0 };
      const pIdeal = prev.p.map((v, i) => v + (t.p[i] - v) * f);
      const sp = Math.hypot(...pm.map((v, i) => v - pIdeal[i]));
      const so = Math.max(0, Math.min(angBetween(R3(Tm), prev.R), angBetween(R3(Tm), t.R)) - 0);   // ori between the two targets
      if (sp > worst.strayPos) { worst.strayPos = sp; worst.strayAt = `${t.seg} ${t.val}`; }
      worst.strayOri = Math.max(worst.strayOri, so > angBetween(prev.R, t.R) ? so - angBetween(prev.R, t.R) : 0);
    }
    const jump = Math.max(...qn.map((v, i) => Math.abs(v - q[i]))), sm = sminOf(qn);
    if (e.posErr > worst.ikPos) worst.ikPos = e.posErr;
    if (e.oriErr > worst.ikOri) worst.ikOri = e.oriErr;
    if (jump > worst.jump) { worst.jump = jump; worst.jumpAt = `${t.seg} ${t.val}`; }
    if (sm < worst.smin) { worst.smin = sm; worst.sminAt = `${t.seg} ${t.val}`; }
    if (Math.abs(qn[4]) < worst.wrist) { worst.wrist = Math.abs(qn[4]); worst.wristAt = `${t.seg} ${t.val}`; }
    worst.wristR = Math.min(worst.wristR, wristR(qn)); worst.margin = Math.min(worst.margin, margin(qn));
    rows.push({ seg: t.seg, val: t.val, q: qn, tgt: { p: t.p, R: t.R }, jump, smin: sm });
    q = qn;
  }
  return { rows, worst, p0 };
}

// ---------- choose the start pose ----------
const candidates = [];
// tool pointing straight down (classic demo); the arm is small (70 mm upper arm, ~114 mm to the wrist
// centre), so far-forward tool-down poses are near the stretched-arm (elbow) singularity.
if (q0Arg) candidates.push(q0Arg);
else for (let a2 = -40; a2 <= 40; a2 += 5) for (let a3 = -30; a3 <= 85; a3 += 5) for (const a5 of [45, 60, 75, 90, -45, -60, -75, -90]) {
  const q = [0, a2, a3, 0, a5, 0], k = K.fk(q), p = K.pos(k.tcp), down = -K.col(k.frames[5], 2)[2];
  if (down > 0.97 && p[0] >= MINX && p[2] >= MINZ && margin(q) >= 25 && sminOf(q) > 15) candidates.push(q);
}
let best = null;
for (const q0 of candidates) {
  const r = solve(q0), w = r.worst;
  const ok = w.ikPos < 0.05 && w.ikOri < 0.05 && w.margin > 3 && w.wrist > 10;
  const score = ok ? Math.min(w.smin, 5 * w.wrist) - 2 * w.jump : -1e9;
  if (!best || score > best.score) best = { q0, r, score, ok };
}
if (!best || !best.ok) { console.log('No candidate start pose satisfied the checks - reduce --L / --A'); process.exit(1); }

// ---------- write outputs ----------
const { q0, r } = best, w = r.worst;
const all = [q0, ...r.rows.map(x => x.q)];
const segs = [...new Set(r.rows.map(x => x.seg))];
writeFileSync(join(HERE, 'tcp_showcase_full.csv'), K.recordedTableCSV(all, { timeStay: 0 }));
writeFileSync(join(HERE, 'tcp_showcase_1_translate.csv'), K.recordedTableCSV([q0, ...r.rows.filter(x => x.seg.startsWith('translate')).map(x => x.q)], { timeStay: 0 }));
writeFileSync(join(HERE, 'tcp_showcase_2_rotate.csv'), K.recordedTableCSV([q0, ...r.rows.filter(x => !x.seg.startsWith('translate')).map(x => x.q)], { timeStay: 0 }));
const perSeg = segs.map(s => {
  const rs = r.rows.filter(x => x.seg === s);
  const range = i => [Math.min(...rs.map(x => x.q[i])), Math.max(...rs.map(x => x.q[i]))].map(v => +v.toFixed(1));
  return { seg: s, rows: rs.length, A1: range(0), A2: range(1), A3: range(2), A4: range(3), A5: range(4), A6: range(5),
    minSmin: +Math.min(...rs.map(x => x.smin)).toFixed(1), maxJump: +Math.max(...rs.map(x => x.jump)).toFixed(2) };
});
const report = { params: { L, A, step: STEP, astep: ASTEP, minx: MINX, minz: MINZ }, start_q: q0, start_tcp_mm: r.p0.map(v => +v.toFixed(1)),
  rows_total: all.length, worst: Object.fromEntries(Object.entries(w).map(([k, v]) => [k, typeof v === 'number' ? +v.toFixed(3) : v])),
  start_smin: +sminOf(q0).toFixed(1), per_segment: perSeg, candidates_tried: candidates.length };
writeFileSync(join(HERE, 'tcp_showcase_report.json'), JSON.stringify(report, null, 1));
console.log(JSON.stringify(report, null, 1));
