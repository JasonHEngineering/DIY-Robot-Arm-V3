// Wrist-singularity demonstration (twin preview first; run on the arm only at LOW speed, if at all)
// From the showcase start pose (tool down, A5 = 60): pitch the tool about its own y axis until A5
// approaches 0 (J4 and J6 axes line up), then yaw +-10 deg there. Near A5 = 0 a small tool rotation
// needs large, fast A4/A6 counter-rotations - the classic wrist singularity.
import { writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
const HERE = dirname(fileURLToPath(import.meta.url));
const K = await import(new URL('../../Python%20program%20loaded%20on%20Raspberry%20Pi%204/twin_web/kin.js', import.meta.url).href);
const LIM = [[-100, 100], [-90, 90], [-90, 90], [-120, 120], [-100, 100], [-179.9, 179.9]];
const rotAxis = (u, deg) => { const t = deg * Math.PI / 180, c = Math.cos(t), s = Math.sin(t), C = 1 - c, [x, y, z] = u;
  return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s], [y * x * C + z * s, c + y * y * C, y * z * C - x * s], [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]; };
const R3 = T => [[T[0], T[1], T[2]], [T[4], T[5], T[6]], [T[8], T[9], T[10]]];
const mul3 = (a, b) => a.map(r => [0, 1, 2].map(j => r[0] * b[0][j] + r[1] * b[1][j] + r[2] * b[2][j]));
const toT = (R, p) => Float64Array.from([R[0][0], R[0][1], R[0][2], p[0], R[1][0], R[1][1], R[1][2], p[1], R[2][0], R[2][1], R[2][2], p[2], 0, 0, 0, 1]);
// Start: forearm horizontal (A2 = A3 keeps it level), tool pitched 25 deg down -> A5 = 25, TCP well clear of the table.
const q0 = [0, -15, -15, 0, 25, 0], T0 = K.fk(q0).tcp, p0 = K.pos(T0), R0 = R3(T0);
let q = q0.slice(); const rows = [q0], log = [];
const step = (pitch, yaw, label) => {
  const R = mul3(mul3(R0, rotAxis([0, 0, 1], yaw)), rotAxis([0, 1, 0], pitch));
  const tgt = { p: p0, R: toT(R, p0) }, qn = K.correct(q, 'pose', tgt, LIM, 400), e = K.taskError(K.fk(qn), 'pose', tgt);
  log.push({ label, pitch, yaw, A4: +qn[3].toFixed(1), A5: +qn[4].toFixed(1), A6: +qn[5].toFixed(1),
             jump: +Math.max(...qn.map((v, i) => Math.abs(v - q[i]))).toFixed(1), ikErr: +e.posErr.toFixed(2), oriErr: +e.oriErr.toFixed(2) });
  rows.push(qn); q = qn;
};
// 1) small yaw offset away from the plane of the arm, 2) pitch the tool UP through the singular (tool along forearm) orientation, 3) back
for (const y of [1, 2, 3]) step(0, y, 'yaw offset');
for (let p = -5; p >= -50; p -= 5) step(p, 3, 'pitch through');      // negative pitch about tool y raises the tool here
for (let p = -45; p <= 0; p += 5) step(p, 3, 'pitch back');
for (const y of [2, 1, 0]) step(0, y, 'yaw back');
// rows the arm cannot actually reach (IK error > 0.1 mm / 0.1 deg, A4 pinned at its limit) -> Solve Status FALSE,
// so the Dash Path Plan runner and the twin both skip them
const NL = String.fromCharCode(10);
const csv = K.recordedTableCSV(rows, { timeStay: 0 }).trimEnd().split(NL);
log.forEach((r, k) => { if (r.ikErr > 0.1 || r.oriErr > 0.1) csv[k + 2] = csv[k + 2].replace(',TRUE,', ',FALSE,'); });
writeFileSync(join(HERE, 'tcp_showcase_3_singularity_demo.csv'), csv.join(NL) + NL);
console.log('rows', rows.length, 'unreachable (FALSE)', log.filter(r => r.ikErr > 0.1 || r.oriErr > 0.1).length);
console.table(log);
console.log('start tcp', p0.map(v => v.toFixed(1)).join(', '), 'mm');
