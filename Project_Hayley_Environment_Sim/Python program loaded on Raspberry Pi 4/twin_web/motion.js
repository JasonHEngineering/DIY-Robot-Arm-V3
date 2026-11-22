// Motion timing model of the ESP32-S3 firmware, so previews / run-time estimates match the robot.
// Mirrors runMove() + the Core-1 queue logic. Parameters come from /twin/api/config ("motion"),
// which hayley_twin.py fills from its MOTION_PROFILES (keep those in sync with the .ino defines).
//
//   V22: trapezoid on the master axis (start/stop speed v0, accel, per-axis max rates) and
//        blending of consecutive queued rows when no axis reverses (1-row look-ahead).
//   V21: 200 master steps at 100 steps/s at both ends of every move, 1000 steps/s in between.

export function createMotion(p) {
  const m = Object.assign({
    firmware: 'V22', v0: 200, accel: 4000, axisMax: [2000, 2000, 2000, 2000, 2000, 1000],
    stepsPerDeg: [133.333, 133.333, 133.333, 133.333, 133.333, 4.4444], ratio: 1,
    launchSteps: 200, launchSpeed: 100, fullSpeed: 1000,
  }, p || {});

  const steps = (from, to) => to.map((v, i) => Math.round((v - from[i]) * m.stepsPerDeg[i]));   // signed
  const master = s => Math.max(...s.map(Math.abs));
  function vmaxFor(s) {
    const M = master(s);
    if (!M) return m.v0;
    let v = Infinity;
    s.forEach((a, i) => { if (a) v = Math.min(v, m.axisMax[i] * M / Math.abs(a)); });
    return Math.max(v * m.ratio, m.v0);
  }

  // continuous trapezoid: master progress (steps) as a function of time
  function trapezoid(M, vs, ve, vm) {
    const A = m.accel;
    vs = Math.min(Math.max(vs, m.v0), vm); ve = Math.min(Math.max(ve, m.v0), vm);
    const vp = Math.min(vm, Math.sqrt((vs * vs + ve * ve) / 2 + A * M));
    const d1 = Math.max(0, (vp * vp - vs * vs) / (2 * A)), d3 = Math.max(0, (vp * vp - ve * ve) / (2 * A));
    const d2 = Math.max(0, M - d1 - d3);
    const t1 = (vp - vs) / A, t2 = d2 / vp, t3 = (vp - ve) / A;
    const T = t1 + t2 + t3;
    const at = t => {
      if (t <= 0) return 0;
      if (t < t1) return Math.min(M, vs * t + A * t * t / 2);
      if (t < t1 + t2) return Math.min(M, d1 + vp * (t - t1));
      const u = Math.min(t - t1 - t2, t3);
      return Math.min(M, d1 + d2 + vp * u - A * u * u / 2);
    };
    return { T, at, vs, ve };
  }
  // V21 two-speed profile
  function launchProfile(M) {
    const L = Math.min(m.launchSteps, M / 2), lt = 1 / m.launchSpeed, ft = 1 / (m.fullSpeed * m.ratio);
    const T = 2 * L * lt + (M - 2 * L) * ft;
    const at = t => {
      if (t <= 0) return 0;
      if (t < L * lt) return t / lt;
      if (t < L * lt + (M - 2 * L) * ft) return L + (t - L * lt) / ft;
      return Math.min(M, M - L + (t - L * lt - (M - 2 * L) * ft) / lt);
    };
    return { T, at };
  }

  /**
   * Plan a queued sequence like Core 1 does.
   * rows: [{deg:[6], servo}] (only rows that will be sent), start: [6] deg
   * returns segments [{from, to, t0, T, progress(t)->0..1}] and total time
   */
  function plan(rows, start) {
    const segs = [];
    let t = 0, prev = start.slice(), pendingRate = 0;
    rows.forEach((row, k) => {
      const s = steps(prev, row.deg), M = master(s);
      let prof;
      if (!M) prof = { T: 0, at: () => 0 };
      else if (m.firmware === 'V21') prof = launchProfile(M);
      else {
        const vm = vmaxFor(s);
        const vs = pendingRate > 0 ? pendingRate * M : m.v0;
        pendingRate = 0;
        let ve = m.v0;
        const claw = row.servo === 0 || row.servo === 1;
        const next = rows[k + 1];
        if (next && !claw) {
          const ns = steps(row.deg, next.deg), Mn = master(ns);
          if (Mn) {
            const jump = Math.max(...s.map((a, i) => Math.abs(a - ns[i])));
            let r = jump ? m.v0 / jump : Infinity;
            r = Math.min(r, vm / M, vmaxFor(ns) / Mn,
              Math.sqrt(vs * vs + 2 * m.accel * M) / M,
              Math.sqrt(m.v0 * m.v0 + 2 * m.accel * Mn) / Mn);
            if (r * M > m.v0 * 1.05) { ve = r * M; pendingRate = r; }
          }
        }
        prof = trapezoid(M, vs, ve, vm);
      }
      const extra = (row.servo === 0 || row.servo === 1) ? 0.5 : 0;   // openclaw()/closeclaw() 500 ms
      segs.push({ from: prev.slice(), to: row.deg.slice(), t0: t, T: prof.T, M,
        progress: tt => (M ? prof.at(tt) / M : 1) });
      t += prof.T + extra;
      prev = row.deg.slice();
    });
    return { segs, T: t };
  }

  const moveTime = (from, to) => plan([{ deg: to, servo: 2 }], from).T;
  return { params: m, plan, moveTime, steps, vmaxFor };
}
