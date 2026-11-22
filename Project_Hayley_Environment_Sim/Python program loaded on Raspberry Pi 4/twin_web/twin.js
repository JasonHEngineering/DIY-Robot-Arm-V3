// Project Hayley - digital twin front end
// Robots sharing one set of meshes:
//   live    - pose from AS5048A encoders (via /twin/api/state)
//   ghost   - last commanded target sent over I2C
//   preview - CSV playback / pose sandbox / TCP-lock dragging (never sent to the robot)
//   checker - hidden, used for collision sweeps
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { acceleratedRaycast, MeshBVH } from 'three-mesh-bvh';
import URDFLoader from './vendor/urdf-loader/URDFLoader.js';
import { CollisionModel, Highlighter, describe, owningLink } from './collision.js';
import * as K from './kin.js';
import { createMotion } from './motion.js';

THREE.Mesh.prototype.raycast = acceleratedRaycast;

const N = 6;
const JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'];
const DEG = Math.PI / 180;
const POLL_MS = 100;
const SWEEP_STEP_DEG = 1.0;     // collision sweep resolution along a move
const cfg = {
  limits: [[-100, 100], [-90, 90], [-90, 90], [-120, 120], [-100, 100], [-179.9, 179.9]],
  speeds: [7.5, 7.5, 7.5, 7.5, 7.5, 225],
};
let motionCfg = null;                 // firmware timing model params from /twin/api/config
let motion = createMotion();          // default: V22 profile
const el = id => document.getElementById(id);
const mmss = s => `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, '0')}`;
const fmtP = p => p ? `${p[0].toFixed(1)}, ${p[1].toFixed(1)}, ${p[2].toFixed(1)} mm` : '–';
const maxAbsDiff = (a, b) => Math.max(...a.map((v, i) => Math.abs(v - b[i])));

// ================================================================ scene
const vp = el('viewport');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
vp.prepend(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x12151a);
const camera = new THREE.PerspectiveCamera(40, 1, 0.005, 20);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

scene.add(new THREE.HemisphereLight(0xdde6ff, 0x30343a, 1.6));
const sun = new THREE.DirectionalLight(0xffffff, 2.2);
sun.position.set(0.6, 1.2, 0.5);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -0.5, right: 0.5, top: 0.5, bottom: -0.5, near: 0.1, far: 3 });
scene.add(sun);
const fill = new THREE.DirectionalLight(0x9fb4ff, 0.6);
fill.position.set(-0.8, 0.4, -0.6);
scene.add(fill);

const TABLE_Z = -0.010; // underside of the base plate = table top (m)
const ground = new THREE.Mesh(new THREE.PlaneGeometry(3, 3), new THREE.ShadowMaterial({ opacity: 0.35 }));
ground.rotation.x = -Math.PI / 2; ground.position.y = TABLE_Z - 0.0005; ground.receiveShadow = true;
scene.add(ground);
const grid = new THREE.GridHelper(1.0, 20, 0x3a4250, 0x262c35); // 50 mm squares
grid.position.y = TABLE_Z;
scene.add(grid);

// URDF is Z-up; three.js is Y-up
const world = new THREE.Group();
world.rotation.x = -Math.PI / 2;
scene.add(world);
world.add(new THREE.AxesHelper(0.06));

function setView(name) {
  const views = { iso: [0.55, 0.42, 0.55], front: [0.75, 0.2, 0.0], side: [0.08, 0.2, 0.8], top: [0.08, 0.95, 0.001] };
  const v = views[name] || views.iso;
  camera.position.set(v[0], v[1], v[2]);
  controls.target.set(0.08, 0.17, 0);
  controls.update();
}
setView('iso');
document.querySelectorAll('#viewbtns button').forEach(b => b.onclick = () => setView(b.dataset.view));
function resize() {
  const w = vp.clientWidth, h = vp.clientHeight;
  renderer.setSize(w, h); camera.aspect = w / h; camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize); resize();

// ================================================================ robots
let live = null, ghost = null, preview = null, checker = null, coll = null;
const HL = {};
const manager = new THREE.LoadingManager();
const loader = new URDFLoader(manager);
loader.packages = { hayley_description: './description' };
let liveUrdf = null;
loader.load('./description/hayley_v3.urdf', r => { liveUrdf = r; });

const ghostMat = new THREE.MeshBasicMaterial({ color: 0x36c5f0, transparent: true, opacity: 0.22, depthWrite: false });
const previewMat = new THREE.MeshLambertMaterial({ color: 0xff9a3c, transparent: true, opacity: 0.45, depthWrite: false });
const hitSolid = new THREE.MeshStandardMaterial({ color: 0xff3b3b, emissive: 0x661010, metalness: 0.2, roughness: 0.5 });
const hitGhost = new THREE.MeshBasicMaterial({ color: 0xff3b3b, transparent: true, opacity: 0.45, depthWrite: false });
const hitPreview = new THREE.MeshLambertMaterial({ color: 0xff3b3b, emissive: 0x501010, transparent: true, opacity: 0.75, depthWrite: false });

function restyle(robot, mat) {
  robot.traverse(o => { if (o.isMesh) { o.material = mat; o.castShadow = false; o.renderOrder = 2; } });
}

manager.onLoad = () => {
  if (!liveUrdf || live) return;
  live = liveUrdf;
  live.traverse(o => {
    if (o.isMesh) {
      o.castShadow = true; o.receiveShadow = true;
      const m = o.material;
      if (m && m.color) o.material = new THREE.MeshStandardMaterial({ color: m.color, metalness: 0.35, roughness: 0.55 });
    }
  });
  ghost = live.clone(); restyle(ghost, ghostMat);
  preview = live.clone(); restyle(preview, previewMat);
  checker = live.clone(); checker.visible = false;
  world.add(live, ghost, preview, checker);
  ghost.visible = false; preview.visible = false;
  // live TCP marker: small dot on tool0, always visible (added after cloning, so only the live arm has it)
  const tcpDot = new THREE.Mesh(new THREE.SphereGeometry(0.0015, 16, 12),
    new THREE.MeshBasicMaterial({ color: 0xf2c14e, depthTest: false, depthWrite: false, transparent: true }));
  tcpDot.renderOrder = 11;
  live.links.tool0.add(tcpDot);
  el('show-trail').addEventListener('change', e => { tcpDot.visible = e.target.checked; });
  HL.live = new Highlighter(live, hitSolid);
  HL.ghost = new Highlighter(ghost, hitGhost);
  HL.preview = new Highlighter(preview, hitPreview);

  // collision model (built at kinematic zero, where the CAD was modelled)
  const t0 = performance.now();
  coll = new CollisionModel(live, { tableZ: TABLE_Z });
  setPose(checker, [0, 0, 0, 0, 0, 0]);
  const touching = coll.calibrate(checker);
  el('coll-note').textContent = `ready in ${Math.round(performance.now() - t0)} ms` +
    (touching.length ? ` · ignoring pairs touching at zero pose: ${touching.join(', ')}` : '');
  // BVHs on the visual meshes make picking for drag mode fast
  const seen = new Set();
  preview.traverse(o => { if (o.isMesh && !seen.has(o.geometry)) { seen.add(o.geometry); o.geometry.boundsTree = new MeshBVH(o.geometry); } });

  el('loading').remove();
  buildSliders();
  setPose(preview, sandbox);
};

function setPose(robot, deg) {
  if (!robot || !deg) return;
  for (let i = 0; i < N; i++) robot.setJointValue(JOINTS[i], deg[i] * DEG);
}
const _v = new THREE.Vector3();
function tcpOf(robot) {
  if (!robot) return null;
  robot.updateMatrixWorld(true);
  robot.links.tool0.getWorldPosition(_v);
  world.worldToLocal(_v);
  return [_v.x * 1000, _v.y * 1000, _v.z * 1000];
}

// ================================================================ collisions
const collOn = () => el('coll-on').checked && coll;
function checkPose(deg) {          // check an arbitrary joint vector with the hidden checker
  setPose(checker, deg);
  return coll.check(checker);
}
function showColl(id, res, extra = '') {
  const e = el(id);
  if (!collOn()) { e.textContent = 'off'; e.className = ''; return; }
  if (!res) { e.textContent = '–'; e.className = ''; return; }
  e.textContent = (res.any ? describe(res) : 'clear') + extra;
  e.className = res.any ? 'bad' : 'ok';
}
const collState = { live: null, ghost: null, preview: null, move: null, liveSig: '', ghostSig: '', previewSig: '' };
function updateCollisions() {
  if (!coll) return;
  const on = collOn();
  for (const [name, robot] of [['live', live], ['ghost', ghost], ['preview', preview]]) {
    if (!on || !robot.visible) { collState[name] = null; HL[name].apply(null); continue; }
    const sig = JOINTS.map(j => robot.joints[j].angle.toFixed(3)).join(',');
    if (sig !== collState[name + 'Sig']) {
      collState[name + 'Sig'] = sig;
      collState[name] = coll.check(robot);
    }
    HL[name].apply(collState[name] && collState[name].any ? collState[name].links : null);
  }
  showColl('coll-live', collState.live);
  showColl('coll-preview', collState.preview);
  const anyLive = collState.live && collState.live.any;
  pill('pill-coll', !on ? 'collision check off' : anyLive ? 'LIVE COLLISION' : 'live: no collision', !on ? '' : anyLive ? 'hit' : 'ok');
  updateBanner();
}

// sweep a joint-space move (synchronised, like the ESP32) and return first hit
async function sweepMove(from, to, budget) {
  const steps = Math.max(1, Math.ceil(maxAbsDiff(from, to) / SWEEP_STEP_DEG));
  const seg = motion.plan([{ deg: to, servo: 2 }], from).segs[0];
  const timeOf = f => {            // invert the firmware profile: time when the move is f done
    let lo = 0, hi = seg.T;
    for (let k = 0; k < 30; k++) { const mid = (lo + hi) / 2; if (seg.progress(mid) < f) lo = mid; else hi = mid; }
    return hi;
  };
  for (let s = 1; s <= steps; s++) {
    const f = s / steps;
    const q = from.map((v, i) => v + (to[i] - v) * f);
    const res = checkPose(q);
    if (res.any) return { f, t: timeOf(f), res, q };
    if (budget && ++budget.n % 25 === 0) await new Promise(r => setTimeout(r));
  }
  return null;
}

// commanded-move prediction: when a new command arrives, sweep live -> target
let lastCmdT = null;
async function onNewCommand(cmd) {
  if (!collOn() || !state || !state.measured) { collState.move = null; return; }
  const from = state.measured.slice();
  const hit = await sweepMove(from, cmd, { n: 0 });
  collState.move = hit ? { hit, at: Date.now() } : { hit: null };
  const e = el('coll-move');
  if (hit) { e.textContent = `${describe(hit.res)} after ~${hit.t.toFixed(1)} s`; e.className = 'bad'; }
  else { e.textContent = 'clear'; e.className = 'ok'; }
  updateBanner();
}
function updateBanner() {
  const b = el('banner');
  let msg = '';
  if (collOn()) {
    if (collState.live && collState.live.any) msg = `⚠ Live pose in collision: ${describe(collState.live)}`;
    else if (collState.move && collState.move.hit && state && state.commanded &&
             maxAbsDiff(state.measured || state.commanded, state.commanded) > 0.3)
      msg = `⚠ Current move predicted to collide (${describe(collState.move.hit.res)}) ~${collState.move.hit.t.toFixed(1)} s after it started - consider E-stop / disabling motors`;
  }
  b.hidden = !msg; b.textContent = msg;
}

// ================================================================ TCP accuracy (live vs commanded)
// Path Plan sends every row at once, so "target" jumps to the LAST row immediately. Besides the
// plain live<->target distance, the twin rebuilds the queued path from /twin/api/commands and
// measures how far the live TCP is from the path the TCP should follow (joint-space interpolation
// between rows, exactly what the ESP32 does). Max is kept per run; a run ends when the arm settles.
const acc = { init: false, lastSeq: 0, fetching: false, run: null, samples: [] };   // samples: distinct encoder readings [t, q]
const tcpMm = q => K.pos(K.fk(q).tcp);
const dist3 = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
function segDist(p, a, b) {
  const ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
  const L2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
  const t = L2 > 0 ? Math.max(0, Math.min(1, (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / L2)) : 0;
  return dist3(p, [a[0] + ab[0] * t, a[1] + ab[1] * t, a[2] + ab[2] * t]);
}
const planGeo = new THREE.BufferGeometry();
const planLine = new THREE.Line(planGeo, new THREE.LineDashedMaterial({ color: 0x36c5f0, dashSize: 0.004, gapSize: 0.003, depthTest: false, transparent: true }));
planLine.renderOrder = 12;   // dashed, drawn over the yellow live trail so both stay visible
world.add(planLine);
function updatePlanLine() {
  const pts = acc.run ? acc.run.poly : [];
  planGeo.setAttribute('position', new THREE.Float32BufferAttribute(pts.flatMap(p => [p[0] / 1000, p[1] / 1000, p[2] / 1000]), 3));
  planLine.computeLineDistances();
  planLine.visible = pts.length > 1 && el('show-plan').checked;
}
el('show-plan').onchange = updatePlanLine;
function extendPlan(run, a, b) {
  const n = Math.max(1, Math.ceil(maxAbsDiff(a, b) / 0.5));       // sample every 0.5 deg of joint motion
  for (let k = 1; k <= n; k++) run.poly.push(tcpMm(a.map((v, i) => v + (b[i] - v) * k / n)));
}
async function fetchCommands() {
  if (acc.fetching) return;
  acc.fetching = true;
  try {
    const j = await (await fetch(`./api/commands?since=${acc.lastSeq}`, { cache: 'no-store' })).json();
    const cmds = j.commands || [];
    if (cmds.length) {
      if (!acc.run || !acc.run.active) {                      // new run starts at the live pose
        const q0 = (state && state.measured) ? state.measured.slice() : cmds[0].deg.slice();
        acc.run = { qs: [q0], poly: [tcpMm(q0)], max: 0, now: 0, settled: null, active: true, rows: 0 };
      }
      for (const c of cmds) {
        extendPlan(acc.run, acc.run.qs[acc.run.qs.length - 1], c.deg);
        acc.run.qs.push(c.deg.slice()); acc.run.rows++;
      }
      acc.run.lastCmdAt = performance.now();
      updatePlanLine();
    }
    acc.lastSeq = j.latest;
  } catch (e) { /* retry on next poll */ }
  acc.fetching = false;
}
function setAcc(id, mm, warn, bad) {
  const e = el(id);
  if (mm == null) { e.textContent = '–'; e.className = ''; return; }
  e.textContent = typeof mm === 'string' ? mm : `${mm.toFixed(2)} mm`;
  const v = typeof mm === 'string' ? 0 : mm;
  e.className = v > bad ? 'bad' : v > warn ? 'warn' : 'ok';
}
function accuracyTick() {
  const m = state && state.measured;
  if (!m) return;
  // keep distinct encoder samples (the server reads at ~5 Hz, this runs at ~10 Hz)
  const nowMs = performance.now(), last = acc.samples[acc.samples.length - 1];
  if (!last || maxAbsDiff(last[1], m) > 1e-6) acc.samples.push([nowMs, m.slice()]);
  while (acc.samples.length > 2 && nowMs - acc.samples[1][0] > 2000) acc.samples.shift();
  const old = acc.samples.findLast(x => nowMs - x[0] >= 400);   // newest sample that is at least 0.4 s old
  const still = !!old && acc.samples.filter(x => x[0] >= old[0]).every(x => maxAbsDiff(x[1], m) < 0.02);
  const p = tcpMm(m);
  const lt = state.commanded ? dist3(p, tcpMm(state.commanded)) : null;
  setAcc('acc-lt', lt, 0.5, 5);
  const run = acc.run;
  if (run) {
    if (run.active) {
      let d = Infinity;
      for (let k = 1; k < run.poly.length; k++) d = Math.min(d, segDist(p, run.poly[k - 1], run.poly[k]));
      if (run.poly.length === 1) d = dist3(p, run.poly[0]);
      run.now = d; run.max = Math.max(run.max, d);
      // settled: at the last row, no new rows for 1.5 s and the encoders unchanged for 0.4 s
      if (state.commanded && maxAbsDiff(m, state.commanded) < 0.3 && still && performance.now() - run.lastCmdAt > 1500) {
        run.active = false;
      }
    }
    if (!run.active) run.settled = lt;      // keeps updating at rest (e.g. while the ESP32 corrects)
    el('acc-dev').textContent = run.active ? `${run.now.toFixed(2)} / ${run.max.toFixed(2)} mm` : `– / ${run.max.toFixed(2)} mm`;
    el('acc-dev').className = run.max > 3 ? 'bad' : run.max > 1 ? 'warn' : 'ok';
    setAcc('acc-set', run.settled, 0.2, 1);
    el('acc-run').textContent = `${run.rows} row${run.rows === 1 ? '' : 's'}${run.active ? ' · running' : ' · settled'}`;
  }
}
el('acc-reset').onclick = () => { if (acc.run) { acc.run.max = acc.run.now || 0; } };

// ================================================================ TCP trail
const TRAIL_MAX = 4000;
const trailPos = new Float32Array(TRAIL_MAX * 3);
const trailGeo = new THREE.BufferGeometry();
trailGeo.setAttribute('position', new THREE.BufferAttribute(trailPos, 3));
trailGeo.setDrawRange(0, 0);
// drawn on top of the model: the TCP sits on the axis-6 shaft tip, so a depth-tested line hides inside it
const trail = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ color: 0xf2c14e, depthTest: false, depthWrite: false, transparent: true }));
trail.renderOrder = 10;
world.add(trail);
let trailN = 0;
function pushTrail(p) {
  if (!p) return;
  if (trailN > 0) {
    const k = (trailN - 1) * 3;
    if (Math.hypot(trailPos[k] - p[0] / 1000, trailPos[k + 1] - p[1] / 1000, trailPos[k + 2] - p[2] / 1000) < 0.001) return;
  }
  if (trailN >= TRAIL_MAX) { trailPos.copyWithin(0, 3); trailN--; }
  trailPos.set([p[0] / 1000, p[1] / 1000, p[2] / 1000], trailN * 3); trailN++;
  trailGeo.setDrawRange(0, trailN); trailGeo.attributes.position.needsUpdate = true;
}
el('clear-trail').onclick = () => { trailN = 0; trailGeo.setDrawRange(0, 0); };
el('show-trail').onchange = e => trail.visible = e.target.checked;

// ================================================================ live state
let state = null, liveShown = null;
const tbody = document.querySelector('#joints tbody');
for (let i = 0; i < N; i++) {
  tbody.insertAdjacentHTML('beforeend',
    `<tr id="j${i}"><td>A${i + 1}</td><td class="live">–</td><td class="ghost">–</td><td class="err">–</td>` +
    `<td><div class="bar"><i class="g"></i><i class="l"></i></div></td></tr>`);
}
function pill(id, text, cls) { const e = el(id); e.textContent = text; e.className = 'pill ' + (cls || ''); }
const barPct = (i, v) => Math.max(0, Math.min(100, (v - cfg.limits[i][0]) / (cfg.limits[i][1] - cfg.limits[i][0]) * 100));

async function poll() {
  try {
    const r = await fetch('./api/state', { cache: 'no-store' });
    state = await r.json();
    const ratio = state.speed_mode === 'low' ? 0.2 : 1.0;
    if (motion.params.ratio !== ratio) motion = createMotion(Object.assign({}, motionCfg || {}, { ratio }));
    if (state.command_seq != null) {
      if (!acc.init) { acc.init = true; acc.lastSeq = state.command_seq; }      // ignore history from before page load
      else if (state.command_seq > acc.lastSeq) fetchCommands();
    }
    if (state.commanded && state.commanded_t !== lastCmdT) {
      lastCmdT = state.commanded_t;
      if (coll) onNewCommand(state.commanded.slice());
    }
  } catch (e) { state = null; }
  setTimeout(poll, POLL_MS);
}
async function loadConfig() {
  try {
    const c = await (await fetch('./api/config')).json();
    if (c.limits_deg && c.limits_deg.length === N) cfg.limits = c.limits_deg;
    if (c.joint_speed_deg_s) cfg.speeds = c.joint_speed_deg_s;
    if (c.motion) { motionCfg = c.motion; motion = createMotion(motionCfg); }
  } catch (e) { /* defaults */ }
}

function updatePanel() {
  const s = state;
  if (!s) { pill('pill-link', 'server offline', 'bad'); return; }
  pill('pill-mode', s.mode === 'sim' ? 'SIMULATION' : 'HARDWARE', s.mode === 'sim' ? 'warn' : 'info');
  const age = s.measured_age_s;
  pill('pill-link', age == null ? 'no encoder data' : (age < 2 ? 'encoders live' : `encoders stale ${age.toFixed(0)}s`),
    age == null ? 'bad' : (age < 2 ? 'ok' : 'warn'));
  pill('pill-motor', s.stepper_enabled == null ? 'motors ?' : (s.stepper_enabled ? 'motors on' : 'motors off'),
    s.stepper_enabled ? 'ok' : 'warn');
  pill('pill-ps', s.ps_enabled ? 'PS controller' : 'PS off', s.ps_enabled ? 'info' : '');
  pill('pill-claw', s.servo == null ? 'claw ?' : (s.servo === 1 ? 'claw closed' : 'claw open'), '');
  for (let i = 0; i < N; i++) {
    const tr = el('j' + i), td = tr.children;
    const m = s.measured ? s.measured[i] : null, c = s.commanded ? s.commanded[i] : null;
    td[1].textContent = m == null ? '–' : m.toFixed(2);
    td[2].textContent = c == null ? '–' : c.toFixed(2);
    const d = (m != null && c != null) ? c - m : null;
    td[3].textContent = d == null ? '–' : d.toFixed(2);
    td[3].className = 'err ' + (d == null ? '' : Math.abs(d) > 2 ? 'bad' : Math.abs(d) > 0.5 ? 'warn' : '');
    const bars = td[4].firstChild.children;
    bars[0].style.display = c == null ? 'none' : ''; bars[0].style.left = `calc(${barPct(i, c ?? 0)}% - 1px)`;
    bars[1].style.display = m == null ? 'none' : ''; bars[1].style.left = `calc(${barPct(i, m ?? 0)}% - 1px)`;
    tr.classList.toggle('limit', m != null && (m < cfg.limits[i][0] || m > cfg.limits[i][1]));
  }
}

// ================================================================ sequence preview
let seq = null;            // {rows:[...], T, start}
let seqT = -1, playing = false;   // seqT < 0 => sandbox/drag drives the preview

function parseCSV(text) {
  const lines = text.split(/\r?\n/).filter(l => l.trim().length);
  const head = lines[0].split(',').map(h => h.trim());
  const ti = [1, 2, 3, 4, 5, 6].map(k => head.indexOf('theta' + k));
  if (ti.some(i => i < 0)) throw new Error('CSV needs columns theta1..theta6');
  const si = head.indexOf('Solve Status'), vi = head.indexOf('Servo Status');
  return lines.slice(1).map((l, n) => {
    const c = l.split(',');
    return makeRow(n + 1, ti.map(i => parseFloat(c[i])),
      si < 0 ? true : /^(true|1)$/i.test((c[si] || '').trim()), vi < 0 ? 2 : parseFloat(c[vi]));
  });
}
function makeRow(n, deg, solved, servo) {
  const inRange = deg.every((v, i) => v >= cfg.limits[i][0] && v <= cfg.limits[i][1]);
  const valid = deg.every(Number.isFinite);
  return { n, deg, servo, ok: solved && inRange && valid, why: !valid ? 'bad value' : !solved ? 'not solved' : !inRange ? 'out of range' : '' };
}
function buildTimeline(rows, startDeg) {
  // firmware timing model (motion.js): V22 trapezoid + row blending, or V21 launch zones
  const okRows = rows.filter(r => r.ok);
  const plan = motion.plan(okRows, startDeg);
  let prev = startDeg.slice(), t = 0, k = 0;
  for (const r of rows) {
    r.hit = null;
    if (r.ok) {
      const seg = plan.segs[k++];
      r.seg = seg; r.start = seg.from; r.t0 = seg.t0; r.tm = seg.t0 + seg.T;
      t = (k < plan.segs.length) ? plan.segs[k].t0 : plan.T;
      prev = r.deg.slice();
    } else { r.start = prev.slice(); r.t0 = t; r.tm = t; r.seg = null; }
    r.t1 = t;
  }
  return { rows, T: plan.T, start: startDeg.slice() };
}
function poseAt(t) {
  if (!seq) return null;
  for (const r of seq.rows) {
    if (!r.ok) continue;
    if (t <= r.t1) {
      const s = r.seg ? r.seg.progress(Math.min(Math.max(t - r.t0, 0), r.seg.T)) : 1;
      return { deg: r.deg.map((v, i) => r.start[i] + (v - r.start[i]) * Math.max(0, Math.min(1, s))), row: r };
    }
  }
  const last = [...seq.rows].reverse().find(r => r.ok);
  return { deg: last ? last.deg : seq.start, row: last };
}
let sweepToken = 0;
async function loadSequence(rows, label, start, startLabel) {
  seq = buildTimeline(rows, start);
  const bad = rows.filter(r => !r.ok).length;
  const head = `<b>${label}</b>: ${rows.length} rows, ${bad ? `<span style="color:var(--bad)">${bad} skipped</span>, ` : ''}` +
    `est. run time <b>${mmss(seq.T)}</b> from the ${startLabel} pose (firmware ${motion.params.firmware} timing).` +
    (rows.filter(r => r.ok).length > 200 ? ' <span style="color:var(--bad)">More than 200 moves: the ESP32 queue holds 200, extra rows sent back-to-back are dropped.</span>' : '');
  el('seq-info').innerHTML = head + (collOn() ? ' <span id="sweep-msg">Checking collisions…</span>' : '');
  renderRows();
  ['play', 'restart', 'scrub'].forEach(id => el(id).disabled = false);
  seqT = 0; playing = false; updatePlayBtn();
  preview.visible = true; el('show-preview').checked = true;
  el('scrub-marks').innerHTML = '';
  if (!collOn()) return;
  const token = ++sweepToken, budget = { n: 0 };
  let hits = 0;
  for (const r of seq.rows) {
    if (!r.ok) continue;
    const h = await sweepMove(r.start, r.deg, budget);
    if (token !== sweepToken) return;                       // a newer sequence was loaded
    if (h) { r.hit = h; hits++; el('scrub-marks').insertAdjacentHTML('beforeend', `<i style="left:${(r.t0 + h.t) / (seq.T || 1) * 100}%"></i>`); }
  }
  renderRows();
  const m = el('sweep-msg');
  if (m) m.innerHTML = hits ? `<span style="color:var(--hit)">⚠ ${hits} move${hits > 1 ? 's' : ''} collide (red marks).</span>` : '<span style="color:var(--ok)">No collisions along the path.</span>';
}
function renderRows() {
  el('seq-rows').innerHTML = seq.rows.map(r =>
    `<div data-n="${r.n}" class="${r.ok ? '' : 'skip'} ${r.hit ? 'hitrow' : ''}" title="${r.why}">#${r.n} ${r.deg.map(v => v.toFixed(1)).join(', ')}` +
    `${r.servo === 0 ? ' · open' : r.servo === 1 ? ' · close' : ''}${r.why ? ' · ' + r.why : ''}${r.hit ? ' · ⚠ ' + describe(r.hit.res) : ''}</div>`).join('');
  el('seq-rows').querySelectorAll('div').forEach(d => d.onclick = () => {
    const r = seq.rows[d.dataset.n - 1];
    seqT = r.hit ? r.t0 + r.hit.t : r.t1; playing = false; updatePlayBtn();
  });
}
el('csv').onchange = async e => {
  const f = e.target.files[0]; if (!f) return;
  try {
    const rows = parseCSV(await f.text());
    const live0 = state && state.measured;
    if (locked) unlock();
    await loadSequence(rows, f.name, live0 ? live0 : [0, 0, 0, 0, 0, 0], live0 ? 'live' : 'zero');
  } catch (err) { el('seq-info').textContent = 'Could not read CSV: ' + err.message; }
};
function updatePlayBtn() { el('play').textContent = playing ? '❚❚ Pause' : '▶ Play'; }
el('play').onclick = () => { if (seq) { if (seqT < 0 || seqT >= seq.T) seqT = 0; playing = !playing; updatePlayBtn(); } };
el('restart').onclick = () => { seqT = 0; };
el('scrub').oninput = e => { if (seq) { seqT = e.target.value / 1000 * seq.T; playing = false; updatePlayBtn(); } };

// ================================================================ sandbox
const sandbox = [0, 0, 0, 0, 0, 0];
function buildSliders() {
  const box = el('sliders');
  box.innerHTML = '';
  for (let i = 0; i < N; i++) {
    const [lo, hi] = cfg.limits[i];
    box.insertAdjacentHTML('beforeend',
      `<div class="slider"><span>A${i + 1}</span><input type="range" id="s${i}" min="${lo}" max="${hi}" step="0.1" value="${sandbox[i]}">` +
      `<input type="number" id="sv${i}" min="${lo}" max="${hi}" step="0.1" value="${sandbox[i].toFixed(1)}" title="type an angle (deg), Enter to apply"></div>`);
  }
  for (let i = 0; i < N; i++) {
    el('s' + i).oninput = e => { sandbox[i] = parseFloat(e.target.value); sandboxActive(); };
    const box = el('sv' + i), [lo, hi] = cfg.limits[i];
    box.oninput = () => {                       // live while typing, only when the value is complete and in range
      const v = parseFloat(box.value);
      box.classList.toggle('bad', !Number.isFinite(v) || v < lo || v > hi);
      if (Number.isFinite(v) && v >= lo && v <= hi) { sandbox[i] = v; sandboxActive(); }
    };
    box.onchange = () => {                      // Enter / leaving the box: clamp and apply
      let v = parseFloat(box.value);
      if (!Number.isFinite(v)) v = sandbox[i];
      v = Math.min(hi, Math.max(lo, v));
      box.classList.remove('bad');
      sandbox[i] = Math.round(v * 100) / 100; sandboxActive(); box.value = sandbox[i].toFixed(1);
    };
    box.onkeydown = e => { if (e.key === 'Enter') box.blur(); };
  }
}
function syncSliders() {
  for (let i = 0; i < N; i++) {
    el('s' + i).value = sandbox[i];
    const box = el('sv' + i);
    if (document.activeElement !== box) box.value = sandbox[i].toFixed(1);   // don't fight the user's typing
  }
}
function sandboxActive() {
  playing = false; seqT = -1; updatePlayBtn();
  syncSliders();
  setPose(preview, sandbox);
  if (preview) preview.visible = true;
  el('show-preview').checked = true;
}
el('snap-live').onclick = () => { if (state && state.measured && !locked) { state.measured.forEach((v, i) => sandbox[i] = v); sandboxActive(); } };
el('zero').onclick = () => { if (!locked) { sandbox.fill(0); sandboxActive(); } };
el('copy-angles').onclick = () => navigator.clipboard && navigator.clipboard.writeText(sandbox.map(v => v.toFixed(2)).join(', '));
el('show-preview').onchange = e => { if (preview) preview.visible = e.target.checked; };
el('show-live').onchange = e => { if (live) live.visible = e.target.checked; };
el('coll-on').onchange = () => { for (const k of ['liveSig', 'ghostSig', 'previewSig']) collState[k] = ''; collState.move = null; el('coll-move').textContent = '–'; el('coll-move').className = ''; };

// ================================================================ TCP lock & drag
let locked = null;          // { mode, target }
let path = [];              // recorded waypoints (deg)
let undoStack = [];
const marker = new THREE.Group();
{
  const s = new THREE.Mesh(new THREE.SphereGeometry(0.004, 16, 12), new THREE.MeshBasicMaterial({ color: 0xff9a3c, depthTest: false }));
  s.renderOrder = 5; marker.add(s);
  const ax = new THREE.AxesHelper(0.035); ax.material.depthTest = false; ax.renderOrder = 5; marker.add(ax);
  marker.matrixAutoUpdate = false; marker.visible = false; world.add(marker);
}
function placeMarker(T) {
  const m = new THREE.Matrix4().set(T[0], T[1], T[2], T[3] / 1000, T[4], T[5], T[6], T[7] / 1000, T[8], T[9], T[10], T[11] / 1000, 0, 0, 0, 1);
  marker.matrix.copy(m); marker.visible = true;
}
function lockStatus(msg, cls) { const e = el('lock-status'); e.textContent = msg; e.className = cls || 'muted'; }
function setLockedUI(on) {
  el('lock').classList.toggle('on', on);
  el('lock').textContent = on ? '🔓 Unlock TCP' : '🔒 Lock TCP at preview pose';
  el('lock-mode').disabled = on;
  el('alts').disabled = !on;
  for (let i = 0; i < N; i++) { const s = el('s' + i), b = el('sv' + i); if (s) s.disabled = on; if (b) b.disabled = on; }
  el('draghint').hidden = !on || el('lock-mode').value === 'pose';
  updatePathUI();
}
function lock() {
  if (!preview) return;
  if (seqT >= 0 && seq) { const p = poseAt(seqT); if (p) p.deg.forEach((v, i) => sandbox[i] = v); }
  sandboxActive();
  const mode = el('lock-mode').value;
  locked = { mode, target: K.makeTarget(sandbox) };
  placeMarker(locked.target.R);
  path = [sandbox.slice()]; undoStack = [];
  el('alt-list').innerHTML = '';
  setLockedUI(true);
  const tcp = K.pos(locked.target.R);
  lockStatus(mode === 'pose'
    ? `Locked full pose at ${fmtP(tcp)}. A 6-axis arm has no free motion with all 6 TCP DOF fixed - use "find alternative configurations".`
    : `Locked ${mode === 'pos' ? 'position' : 'position + tool direction'} at ${fmtP(tcp)}. Drag a link of the orange arm.`);
}
function unlock() {
  locked = null; marker.visible = false; dragging = null;
  setLockedUI(false);
  lockStatus('Pose the preview with the sandbox sliders (or "copy live pose"), then lock.');
}
el('lock').onclick = () => (locked ? unlock() : lock());

function recordWaypoint(q, force) {
  const res = parseFloat(el('wp-res').value);
  if (force ? maxAbsDiff(q, path[path.length - 1]) > 0.05 : maxAbsDiff(q, path[path.length - 1]) >= res) path.push(q.slice());
}
function pathStats() {
  let drift = 0, T = 0;
  const t = K.pos(locked ? locked.target.R : K.fk(path[0]).tcp);
  for (let k = 1; k < path.length; k++) {
    const a = path[k - 1], b = path[k];
    for (const f of [0.25, 0.5, 0.75]) {
      const q = a.map((v, i) => v + (b[i] - v) * f);
      const p = K.pos(K.fk(q).tcp);
      drift = Math.max(drift, Math.hypot(p[0] - t[0], p[1] - t[1], p[2] - t[2]));
    }
  }
  T = path.length > 1 ? motion.plan(path.slice(1).map(q => ({ deg: q, servo: 2 })), path[0]).T : 0;
  return { drift, T };
}
function updatePathUI() {
  const has = path.length > 1;
  ['path-preview', 'path-export', 'path-clear'].forEach(id => el(id).disabled = !has);
  el('path-undo').disabled = !undoStack.length;
  if (!has) { el('path-info').textContent = locked ? 'Path: start pose recorded - drag to add waypoints.' : ''; return; }
  const s = pathStats();
  el('path-info').innerHTML = `Path: <b>${path.length}</b> waypoints · est. run <b>${mmss(s.T)}</b> · max TCP drift between waypoints <b>${s.drift.toFixed(2)} mm</b>` +
    (path.length > 200 ? ' <span style="color:var(--bad)">· over 200 rows: the ESP32 V20 queue (200) would drop the rest - use a coarser step</span>' : '') +
    (s.drift > 2 ? ' <span style="color:var(--warn)">(TCP moves between some waypoints: a configuration switch, or use a finer waypoint step)</span>' : '');
}

// --- picking & dragging
const raycaster = new THREE.Raycaster();
raycaster.firstHitOnly = true;
const ndc = new THREE.Vector2();
let dragging = null;   // { linkIdx, localMm, plane, goalMm }
function setNdc(ev) {
  const r = renderer.domElement.getBoundingClientRect();
  ndc.set(((ev.clientX - r.left) / r.width) * 2 - 1, -((ev.clientY - r.top) / r.height) * 2 + 1);
}
renderer.domElement.addEventListener('pointerdown', ev => {
  if (!locked || locked.mode === 'pose' || ev.button !== 0 || !preview || !preview.visible) return;
  setNdc(ev);
  raycaster.setFromCamera(ndc, camera);
  const hit = raycaster.intersectObject(preview, true)[0];
  if (!hit) return;
  const link = owningLink(hit.object);
  const idx = link && /^link(\d)$/.test(link.name) ? parseInt(link.name.slice(4)) : 0;
  if (!idx) { lockStatus('That part is on the fixed base - grab one of the moving links.', 'warn'); return; }
  const local = link.worldToLocal(hit.point.clone());
  const plane = new THREE.Plane().setFromNormalAndCoplanarPoint(camera.getWorldDirection(new THREE.Vector3()), hit.point);
  const wp = world.worldToLocal(hit.point.clone());
  dragging = { linkIdx: idx, localMm: [local.x * 1000, local.y * 1000, local.z * 1000], plane, goalMm: [wp.x * 1000, wp.y * 1000, wp.z * 1000] };
  undoStack.push(path.length);
  if (seqT >= 0) { seqT = -1; playing = false; updatePlayBtn(); setPose(preview, sandbox); }   // leave path playback
  controls.enabled = false;
  renderer.domElement.setPointerCapture(ev.pointerId);
  lockStatus(`Dragging link ${idx}…`);
});
renderer.domElement.addEventListener('pointermove', ev => {
  if (!dragging) return;
  setNdc(ev);
  raycaster.setFromCamera(ndc, camera);
  const p = new THREE.Vector3();
  if (raycaster.ray.intersectPlane(dragging.plane, p)) { world.worldToLocal(p); dragging.goalMm = [p.x * 1000, p.y * 1000, p.z * 1000]; }
});
function endDrag(ev) {
  if (!dragging) return;
  recordWaypoint(sandbox, true);
  dragging = null; controls.enabled = true;
  if (ev && ev.pointerId != null && renderer.domElement.hasPointerCapture(ev.pointerId)) renderer.domElement.releasePointerCapture(ev.pointerId);
  if (undoStack.length && undoStack[undoStack.length - 1] === path.length) undoStack.pop();   // no-op drag
  updatePathUI();
}
renderer.domElement.addEventListener('pointerup', endDrag);
renderer.domElement.addEventListener('pointercancel', endDrag);

function dragTick() {
  if (!dragging || !locked) return;
  let q = sandbox.slice(), why = '';
  const prev = q.slice();
  for (let it = 0; it < 4; it++) {
    const r = K.dragStep(q, locked.mode, locked.target, dragging.linkIdx, dragging.localMm, dragging.goalMm, cfg.limits);
    if (r.err.posErr > 0.3 || r.err.oriErr > 0.3) { why = 'joint limit / reach: cannot move further while holding the TCP'; break; }
    q = r.q;
    if (r.pointErr < 0.5) break;
  }
  if (maxAbsDiff(q, prev) < 1e-4) {
    lockStatus(why || (locked.mode === 'pos_axis'
      ? 'No free motion for this link: with the TCP on the axis-6 centreline, the 5D lock only leaves A6 spinning. Unlock and use Position only (3D).'
      : 'This link cannot move that way while the TCP is held - try another direction or link.'), 'warn');
    return;
  }
  if (collOn() && el('avoid').checked) {
    const res = checkPose(q);
    if (res.any) { lockStatus(`Stopped: would collide (${describe(res)})`, 'bad'); return; }
  }
  q.forEach((v, i) => sandbox[i] = v);
  syncSliders();
  setPose(preview, sandbox);
  recordWaypoint(sandbox, false);
  const atLim = q.map((v, i) => (Math.abs(v - cfg.limits[i][0]) < 0.05 || Math.abs(v - cfg.limits[i][1]) < 0.05) ? `A${i + 1}` : null).filter(Boolean);
  lockStatus(atLim.length ? `At joint limit: ${atLim.join(', ')}` : `Dragging link ${dragging.linkIdx} · TCP held`, atLim.length ? 'warn' : 'muted');
  if (path.length % 5 === 0) updatePathUI();
}

el('alts').onclick = () => {
  if (!locked) return;
  const sols = K.alternatives(sandbox, cfg.limits);
  const box = el('alt-list');
  if (sols.length <= 1) {
    box.innerHTML = `<div class="muted">No other configuration reaches this exact TCP pose within the joint limits` +
      `${K.isWristSingular(sandbox) ? ' (wrist is near A5 = 0: A4 and A6 can trade off freely)' : ''}.</div>`;
    return;
  }
  box.innerHTML = sols.map((q, k) => {
    const res = collOn() ? checkPose(q) : null;
    return `<button class="small ${res && res.any ? 'hitalt' : ''}" data-k="${k}" title="${q.map(v => v.toFixed(1)).join(', ')}${res && res.any ? ' · collides: ' + describe(res) : ''}">` +
      `${k === 0 ? 'current' : 'config ' + (k + 1)}${k > 0 && maxAbsDiff(q, K.wristFlip(sols[0])) < 1 ? ' (wrist flip)' : ''}${res && res.any ? ' ⚠' : ''}</button>`;
  }).join('') + '<div class="muted">Switching configuration is a joint move: the TCP is NOT held during that transit.</div>';
  box.querySelectorAll('button').forEach(b => b.onclick = () => {
    const q = sols[b.dataset.k];
    undoStack.push(path.length);
    q.forEach((v, i) => sandbox[i] = v); syncSliders(); setPose(preview, sandbox);
    recordWaypoint(sandbox, true); updatePathUI();
  });
};

el('path-clear').onclick = () => { path = [sandbox.slice()]; undoStack = []; updatePathUI(); };
el('path-undo').onclick = () => {
  if (!undoStack.length) return;
  path.length = Math.max(1, undoStack.pop());
  path[path.length - 1].forEach((v, i) => sandbox[i] = v); syncSliders(); setPose(preview, sandbox);
  updatePathUI();
};
el('path-preview').onclick = async () => {
  const rows = path.slice(1).map((q, k) => makeRow(k + 1, q.slice(), true, 2));
  loadSequence(rows, 'TCP-lock path', path[0], 'locked start');
  playing = true; updatePlayBtn();
};
el('path-export').onclick = () => {
  // same structure as the Dash Recording Mode download, so Path Plan accepts it as-is
  // time_stay 0: waypoints flow one after another (sequence_run does not dwell on it today)
  const blob = new Blob([K.recordedTableCSV(path, { timeStay: 0 })], { type: 'text/csv' });
  const a = document.createElement('a');
  const d = new Date(), z = n => String(n).padStart(2, '0');
  a.download = `recorded_joints_table_tcp_lock_${z(d.getDate())}${z(d.getMonth() + 1)}${String(d.getFullYear()).slice(2)}_${z(d.getHours())}${z(d.getMinutes())}${z(d.getSeconds())}.csv`;
  a.href = URL.createObjectURL(blob); a.click(); setTimeout(() => URL.revokeObjectURL(a.href), 1000);
};
el('lock-mode').onchange = () => { el('draghint').hidden = true; };

// ================================================================ loop
let lastFrame = performance.now(), panelTick = 0;
function frame(now) {
  const dt = Math.min(0.1, (now - lastFrame) / 1000); lastFrame = now;
  if (live) {
    const target = state && state.measured;
    if (target) {
      if (!liveShown) liveShown = target.slice();
      const k = 1 - Math.exp(-dt * 12);   // smooth between 5-10 Hz samples
      for (let i = 0; i < N; i++) liveShown[i] += (target[i] - liveShown[i]) * k;
      setPose(live, liveShown);
      pushTrail(tcpOf(live));
    }
    const cmd = state && state.commanded;
    ghost.visible = !!cmd && el('show-ghost').checked;
    if (cmd) setPose(ghost, cmd);

    dragTick();
    if (seq && seqT >= 0) {
      if (playing) {
        seqT += dt * parseFloat(el('speed').value);
        if (seqT >= seq.T) { seqT = seq.T; playing = false; updatePlayBtn(); }
      }
      const p = poseAt(seqT);
      if (p) {
        setPose(preview, p.deg);
        el('seq-rows').querySelectorAll('.cur').forEach(d => d.classList.remove('cur'));
        if (p.row) { const d = el('seq-rows').querySelector(`[data-n="${p.row.n}"]`); if (d) { d.classList.add('cur'); if (playing) d.scrollIntoView({ block: 'nearest' }); } }
      }
      el('scrub').value = seq.T ? seqT / seq.T * 1000 : 0;
      el('seq-time').textContent = `${mmss(seqT)} / ${mmss(seq.T)}`;
    }
    if (++panelTick % 6 === 0) {
      updatePanel();
      updateCollisions();
      accuracyTick();
      el('tcp-live').textContent = fmtP(state && state.measured ? tcpOf(live) : null);
      el('tcp-ghost').textContent = fmtP(cmd ? tcpOf(ghost) : null);
      el('tcp-preview').textContent = fmtP(tcpOf(preview));
    }
  }
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(frame);
}

// debug handle (browser console): hayleyTwin.robots.preview, hayleyTwin.kin.fk([...]) etc.
window.hayleyTwin = { get robots() { return { live, ghost, preview, checker }; }, get collision() { return coll; }, kin: K, camera, controls, world,
  screenOf(robot, linkName) { const v = robot.links[linkName].getWorldPosition(new THREE.Vector3()).project(camera); const r = renderer.domElement.getBoundingClientRect();
    return [r.left + (v.x + 1) / 2 * r.width, r.top + (1 - v.y) / 2 * r.height]; },
  get sandbox() { return sandbox.slice(); }, get path() { return path.map(q => q.slice()); } };

await loadConfig();
poll();
requestAnimationFrame(frame);
