// Kinematics for Project Hayley V3 - JS port of input_linkage_angles() in the Dash UI.
// Units: mm and degrees at the API, radians internally. Pure math, no three.js.
//
// Rows: [axis, theta(deg about Z), alpha(deg about X), delta(mm along Z), rho(mm along X)],
// applied as Rx(alpha)*Rz(theta) then translate (rho, 0, delta)  (same as the Dash app).
export const LINKAGE = [
  [0, 0, 0, 0, 0], [1, 0, 0, 54.5, 0], [1, 0, 0, 0, 0], [2, 0, 0, 84, 0], [2, 0, -90, 0, 0],
  [2, 0, 0, 61, 0], [2, 0, 180, 0, 0], [2, 90, 0, 0, 0], [2, 0, 0, 0, 0], [3, 0, -180, 0, 0],
  [3, 0, 0, 0, 70], [3, 0, 0, 0, 0], [4, 0, 0, -61, 0], [4, 0, 0, 0, 52], [4, 90, 0, 0, 0],
  [4, 0, 0, 0, 57], [4, 90, 0, 0, 0], [4, 0, 90, 0, 0], [4, 0, 0, 0, 0], [5, 90, 0, 0, 0],
  [5, 0, 0, 0, 47], [5, 0, 0, 44.5, 0], [5, -90, 0, 0, 0], [5, 0, -90, 0, 0], [5, -90, 0, 0, 0],
  [5, 0, 0, -47, 0], [5, 0, 0, 0, 0], [6, 0, 0, 0, 93], [6, -90, 0, 0, 0], [6, 0, -90, 0, 0],
  [6, 0, 0, 0, 0], [7, 0, 90, 0, 0], [7, 90, 0, 0, 0], [7, 0, 90, 0, 0],
];
export const JOINT_ROWS = [2, 8, 11, 18, 26, 30];
const D2R = Math.PI / 180, R2D = 180 / Math.PI;

// 4x4 row-major matrices as Float64Array(16)
const I4 = () => { const m = new Float64Array(16); m[0] = m[5] = m[10] = m[15] = 1; return m; };
function mul(A, B) {
  const C = new Float64Array(16);
  for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) {
    let s = 0; for (let k = 0; k < 4; k++) s += A[i * 4 + k] * B[k * 4 + j]; C[i * 4 + j] = s;
  }
  return C;
}
function rowMat(thetaDeg, alphaDeg, delta, rho) {
  const t = thetaDeg * D2R, a = alphaDeg * D2R, ct = Math.cos(t), st = Math.sin(t), ca = Math.cos(a), sa = Math.sin(a);
  return Float64Array.from([ct, -st, 0, rho, st * ca, ct * ca, -sa, -sa * delta, st * sa, ct * sa, ca, ca * delta, 0, 0, 0, 1]);
}
export function invRigid(T) {
  const R = [T[0], T[1], T[2], T[4], T[5], T[6], T[8], T[9], T[10]], p = [T[3], T[7], T[11]];
  const m = I4();
  m[0] = R[0]; m[1] = R[3]; m[2] = R[6]; m[4] = R[1]; m[5] = R[4]; m[6] = R[7]; m[8] = R[2]; m[9] = R[5]; m[10] = R[8];
  m[3] = -(m[0] * p[0] + m[1] * p[1] + m[2] * p[2]);
  m[7] = -(m[4] * p[0] + m[5] * p[1] + m[6] * p[2]);
  m[11] = -(m[8] * p[0] + m[9] * p[1] + m[10] * p[2]);
  return m;
}
export const pos = T => [T[3], T[7], T[11]];
export const col = (T, c) => [T[c], T[4 + c], T[8 + c]];
export function apply(T, p) {
  return [T[0] * p[0] + T[1] * p[1] + T[2] * p[2] + T[3], T[4] * p[0] + T[5] * p[1] + T[6] * p[2] + T[7], T[8] * p[0] + T[9] * p[1] + T[10] * p[2] + T[11]];
}

/** Forward kinematics. q in degrees. Returns { frames: [F1..F6], tcp } (mm). */
export function fk(q) {
  let T = I4();
  const frames = [];
  for (let i = 0; i < LINKAGE.length; i++) {
    const r = LINKAGE[i], j = JOINT_ROWS.indexOf(i);
    T = mul(T, rowMat(j >= 0 ? q[j] : r[1], r[2], 0, 0));
    if (j >= 0) frames.push(T);
    T = mul(T, rowMat(0, 0, r[3], r[4]));
  }
  return { frames, tcp: T };
}
const HOME = fk([0, 0, 0, 0, 0, 0]);
const HOME_INV = HOME.frames.map(invRigid);
// URDF link frame i at q = F_i(q) * F_i(0)^-1 * Translate(origin_i);  origin_i = home joint position
export const LINK_ORIGINS = HOME.frames.map(pos);
export function linkPointWorld(kin, linkIdx, localMm) {
  // linkIdx 1..6, localMm = point in the URDF link frame (mm)
  const o = LINK_ORIGINS[linkIdx - 1];
  const pHome = [localMm[0] + o[0], localMm[1] + o[1], localMm[2] + o[2]];
  return apply(mul(kin.frames[linkIdx - 1], HOME_INV[linkIdx - 1]), pHome);
}

// ------------------------------------------------------------------ small linear algebra
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const norm = a => Math.hypot(a[0], a[1], a[2]);
function solveSPD(A, b) { // Gauss-Jordan, A k x k (array of rows), returns x
  const n = b.length, M = A.map((r, i) => [...r, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c; for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    [M[c], M[p]] = [M[p], M[c]];
    const d = M[c][c] || 1e-12;
    for (let k = c; k <= n; k++) M[c][k] /= d;
    for (let r = 0; r < n; r++) if (r !== c) { const f = M[r][c]; if (f) for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k]; }
  }
  return M.map(r => r[n]);
}
// damped pseudo-inverse applied to a vector: J^T (J J^T + l^2 I)^-1 e
function dlsApply(J, e, lambda) {
  const k = J.length, n = J[0].length;
  const A = Array.from({ length: k }, (_, i) => Array.from({ length: k }, (_, j) => {
    let s = 0; for (let c = 0; c < n; c++) s += J[i][c] * J[j][c]; return s + (i === j ? lambda * lambda : 0);
  }));
  const y = solveSPD(A, e);
  return Array.from({ length: n }, (_, c) => { let s = 0; for (let i = 0; i < k; i++) s += J[i][c] * y[i]; return s; });
}
// explicit damped pseudo-inverse matrix (n x k)
function dlsMatrix(J, lambda) {
  const k = J.length, n = J[0].length, P = Array.from({ length: n }, () => new Array(k).fill(0));
  for (let i = 0; i < k; i++) { const e = new Array(k).fill(0); e[i] = 1; const c = dlsApply(J, e, lambda); for (let r = 0; r < n; r++) P[r][i] = c[r]; }
  return P;
}
const matVec = (M, v) => M.map(r => r.reduce((s, x, i) => s + x * v[i], 0));
const matMul = (A, B) => A.map(r => B[0].map((_, j) => r.reduce((s, x, k) => s + x * B[k][j], 0)));

// ------------------------------------------------------------------ TCP lock (primary task)
export const ORI_SCALE = 100; // mm per rad - weights orientation vs position errors
/**
 * mode: 'pos' (3D), 'pos_axis' (5D: position + tool axis direction), 'pose' (6D)
 * target: { p:[mm], R: tcp matrix (Float64Array) }
 * returns { e, J } with J 6 columns, units mm (orientation scaled by ORI_SCALE)
 */
export function primaryTask(kin, mode, target) {
  const T = kin.tcp, p = pos(T);
  const e = [], J = [];
  const zs = kin.frames.map(F => col(F, 2)), os = kin.frames.map(pos);
  const ep = sub(target.p, p);
  for (let r = 0; r < 3; r++) { e.push(ep[r]); J.push(zs.map((z, j) => cross(z, sub(p, os[j]))[r])); }
  if (mode === 'pose') {
    let ew = [0, 0, 0];
    for (let c = 0; c < 3; c++) { const x = cross(col(T, c), col(target.R, c)); ew = [ew[0] + x[0] / 2, ew[1] + x[1] / 2, ew[2] + x[2] / 2]; }
    for (let r = 0; r < 3; r++) { e.push(ew[r] * ORI_SCALE); J.push(zs.map(z => z[r] * ORI_SCALE)); }
  } else if (mode === 'pos_axis') {
    const a = col(kin.frames[5], 2), at = col(target.F6 || target.R, 2);
    const ew = cross(a, at);
    let u = cross(at, Math.abs(at[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0]); const un = norm(u); u = u.map(x => x / un);
    const v = cross(at, u);
    for (const b of [u, v]) { e.push(dot(b, ew) * ORI_SCALE); J.push(zs.map(z => dot(b, z) * ORI_SCALE)); }
  }
  return { e, J };
}
export function taskError(kin, mode, target) {
  const { e } = primaryTask(kin, mode, target);
  const posErr = Math.hypot(e[0], e[1], e[2]);
  let oriErr = 0; for (let i = 3; i < e.length; i++) oriErr = Math.max(oriErr, Math.abs(e[i]) / ORI_SCALE * R2D);
  return { posErr, oriErr };
}
export function makeTarget(q) {
  const k = fk(q);
  return { p: pos(k.tcp), R: k.tcp, F6: k.frames[5] };
}

const clampLimits = (q, limits) => q.map((v, i) => Math.max(limits[i][0], Math.min(limits[i][1], v)));
function scaleStep(dq, maxDeg) {
  const m = Math.max(...dq.map(Math.abs)) * R2D;
  return m > maxDeg ? dq.map(x => x * maxDeg / m) : dq;
}

/** Pull q back onto the TCP constraint (primary task only). */
export function correct(q, mode, target, limits, iters = 30) {
  let cur = q.slice();
  for (let it = 0; it < iters; it++) {
    const kin = fk(cur);
    const { e, J } = primaryTask(kin, mode, target);
    if (Math.hypot(...e) < 0.01) break;
    const dq = scaleStep(dlsApply(J, e, 0.5), 5);
    cur = clampLimits(cur.map((v, i) => v + dq[i] * R2D), limits);
  }
  return cur;
}

/**
 * One drag step: keep the TCP constraint, move point `localMm` of link `linkIdx` toward `goalMm`
 * using task-priority control (secondary task in the null space of the TCP lock).
 */
export function dragStep(q, mode, target, linkIdx, localMm, goalMm, limits, maxStepDeg = 1.5) {
  const kin = fk(q);
  const { e: e1, J: J1 } = primaryTask(kin, mode, target);
  const p = linkPointWorld(kin, linkIdx, localMm);
  const zs = kin.frames.map(F => col(F, 2)), os = kin.frames.map(pos);
  const J2 = [0, 1, 2].map(r => zs.map((z, j) => j < linkIdx ? cross(z, sub(p, os[j]))[r] : 0));
  const e2 = sub(goalMm, p);
  const J1p = dlsMatrix(J1, 0.5);                       // 6 x k
  const dq1 = matVec(J1p, e1);
  const N = Array.from({ length: 6 }, (_, i) => Array.from({ length: 6 }, (_, j) => (i === j ? 1 : 0) - J1.reduce((s, row, r) => s + J1p[i][r] * row[j], 0)));
  const J2N = matMul(J2, N);
  const r2 = sub(e2, matVec(J2, dq1));
  const dq2 = matVec(N, dlsApply(J2N, r2, 20));        // heavier damping: secondary task is a wish
  let dq = scaleStep(dq1.map((v, i) => v + dq2[i]), maxStepDeg);
  let next = clampLimits(q.map((v, i) => v + dq[i] * R2D), limits);
  next = correct(next, mode, target, limits, 8);
  const err = taskError(fk(next), mode, target);
  return { q: next, err, pointErr: norm(e2) };
}

/** Wrist flip for a spherical wrist: same TCP pose, other wrist branch (exact). */
export function wristFlip(q) {
  const w = a => ((a + 180) % 360 + 360) % 360 - 180;
  return [q[0], q[1], q[2], w(q[3] + 180), -q[4], w(q[5] + 180)];
}

/** Search for other joint configurations reaching the same full TCP pose. */
export function alternatives(q, limits) {
  const target = makeTarget(q);
  const seeds = [];
  for (const d1 of [0, 180, -180]) for (const s2 of [-60, 0, 60]) for (const s3 of [-70, 0, 70]) for (const d4 of [0, 180])
    seeds.push([q[0] + d1, s2, s3, q[3] + d4, d4 ? -q[4] : q[4], q[5] + d4]);
  seeds.push(wristFlip(q));
  const found = [];
  const inLim = s => s.every((v, i) => v >= limits[i][0] - 1e-6 && v <= limits[i][1] + 1e-6);
  for (const s0 of seeds) {
    let s = clampLimits(s0.map(v => ((v + 180) % 360 + 360) % 360 - 180), limits);
    s = correct(s, 'pose', target, limits, 120);
    const err = taskError(fk(s), 'pose', target);
    if (err.posErr > 0.2 || err.oriErr > 0.2 || !inLim(s)) continue;
    if (found.some(f => same(f, s))) continue;
    found.push(s.map(v => Math.round(v * 100) / 100));
  }
  // current configuration first
  found.sort((a, b) => dist(a, q) - dist(b, q));
  return found;
}
// two solutions are the same if equal, or both on the A5=0 wrist singularity with equal A4+A6
const same = (a, b) => a.every((v, i) => Math.abs(v - b[i]) < 1.0) ||
  (Math.abs(a[4]) < 0.5 && Math.abs(b[4]) < 0.5 && [0, 1, 2].every(i => Math.abs(a[i] - b[i]) < 1.0) &&
   Math.abs((((a[3] + a[5]) - (b[3] + b[5])) % 360 + 540) % 360 - 180) < 1.0);
export const isWristSingular = q => Math.abs(q[4]) < 2;
const dist = (a, b) => Math.max(...a.map((v, i) => Math.abs(v - b[i])));

// ------------------------------------------------------------------ Dash-compatible recorded table
// Exact ports of rotation_matrix_to_euler() and matrix_to_quaternion() in the Dash UI V09.
export function eulerRPY(T) {          // degrees [roll, pitch, yaw]
  const R11 = T[0], R12 = T[1], R21 = T[4], R31 = T[8], R32 = T[9], R33 = T[10], R13 = T[2];
  const r4 = v => Math.round(v * 1e4) / 1e4;
  let roll, pitch, yaw;
  if (r4(R31) !== 1 && r4(R31) !== -1) {
    pitch = -Math.asin(R31);
    roll = Math.atan2(R32 / Math.cos(pitch), R33 / Math.cos(pitch));
    yaw = Math.atan2(R21 / Math.cos(pitch), R11 / Math.cos(pitch));
  } else {
    yaw = 0;
    if (R31 === -1) { pitch = Math.PI / 2; roll = yaw + Math.atan2(R12, R13); }
    else { pitch = -Math.PI / 2; roll = -yaw + Math.atan2(-R12, -R13); }
  }
  return [roll * R2D, pitch * R2D, yaw * R2D];
}
export function quaternion(T) {        // [q0(w), q1, q2, q3], normalised
  const q0 = Math.sqrt(1 + T[0] + T[5] + T[10]) / 2;
  const q = [q0, (T[9] - T[6]) / (4 * q0), (T[2] - T[8]) / (4 * q0), (T[4] - T[1]) / (4 * q0)];
  const n = Math.hypot(...q);
  return q.map(v => v / n);
}
/**
 * CSV in the same structure as the Dash "Recording Mode" download (recorded_joints_table_*.csv),
 * which the Path Plan page loads directly:
 * Rows, Coordinate_X..Z, Euler (Roll/Pitch/Yaw), Quaternion (q1..q4), theta1..6, time_stay, Solve Status, Servo Status
 */
export const RECORDED_TABLE_COLUMNS = ['Rows', 'Coordinate_X', 'Coordinate_Y', 'Coordinate_Z',
  'Euler (Roll)', 'Euler (Pitch)', 'Euler (Yaw)',
  'Quaternion (q1)', 'Quaternion (q2)', 'Quaternion (q3)', 'Quaternion (q4)',
  'theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6', 'time_stay', 'Solve Status', 'Servo Status'];
export function recordedTableCSV(poses, { timeStay = 0, servo = 2.0 } = {}) {
  const r1 = v => (Math.round(v * 10) / 10 + 0).toString();          // like round(x, 1) / np.round(x, 1)
  const lines = poses.map((q, k) => {
    const T = fk(q).tcp, e = eulerRPY(T), qt = quaternion(T);
    return [k + 1, r1(T[3]), r1(T[7]), r1(T[11]), ...e.map(r1), ...qt.map(v => String(v + 0)),
      ...q.map(v => (Math.round(v * 100) / 100 + 0).toString()), timeStay, 'TRUE', servo.toFixed(1)].join(',');
  });
  return RECORDED_TABLE_COLUMNS.join(',') + '\n' + lines.join('\n') + '\n';
}

export { R2D, D2R };
