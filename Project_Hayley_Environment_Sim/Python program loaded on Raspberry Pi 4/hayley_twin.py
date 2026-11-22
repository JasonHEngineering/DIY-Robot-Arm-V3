"""
hayley_twin.py - Digital-twin add-on for the Project Hayley Dash UI (Stage 1)

What it does
------------
1. TwinBus      Wraps the I2C bus used by the Dash app. Every angle command
                (MSG_STEPPER_ARRAY) and every encoder read passes through it,
                so the twin always knows the *commanded* and *measured* joint
                angles - with no change to the app's existing logic.
2. SimESP32     A software stand-in for the ESP32-S3 (same I2C byte protocol).
                Lets the whole UI + twin run on a PC with no robot attached:
                    HAYLEY_SIM=1 python Project_Hayley_V5_IK_user_interface_V09.py
                Moves are queued and executed like Project_Hayley_ESP32_V20.ino:
                all axes interpolate together and finish at the same time,
                the slowest axis sets the duration (1000 steps/s master clock,
                8 microsteps, 30:1 on axes 1-5, 1:1 on axis 6).
3. register()   Adds Flask routes to the Dash server:
                    /twin/            3D twin page (three.js + URDF)
                    /twin/api/state   live JSON state polled by the page

Nothing here changes how the robot is driven: the twin only observes.
"""
import os
import csv
import json
import math
import struct
import threading
import time
from collections import deque

from flask import Blueprint, jsonify, send_from_directory, abort, request

MSG_STEPPER_ARRAY = 0x01
MSG_MODE_STRING = 0x02
MSG_PS_CONTROLLER = 0x03
MSG_SPEED = 0x04

N_AXES = 6
SEMAPHORE_VALUE = 888.8  # legacy "end of batch" marker rows - ignored by the twin
ESP32_QUEUE_LEN = 200    # jointQueue = xQueueCreate(200, ...) in V20; xQueueSend(..., 0) drops when full

# --------------------------------------------------------------------------
# Firmware motion timing (mirrors runMove() / Core 1 queue logic of the ESP32 sketch).
# Keep these numbers in sync with the #defines in Project_Hayley_ESP32_V2x.ino.
# Select with HAYLEY_FIRMWARE=V21 or V22 (default V22). Used by the simulator AND sent to the
# twin page (/twin/api/config -> "motion") so previews and run-time estimates match the robot.
# --------------------------------------------------------------------------
GEAR = [30, 30, 30, 30, 30, 1]
STEPS_PER_DEG = [200 * 8 * g / 360.0 for g in GEAR]          # 133.33 (30:1), 4.444 (1:1)
MOTION_PROFILES = {
    "V21": {"firmware": "V21", "launchSteps": 200, "launchSpeed": 100.0, "fullSpeed": 1000.0,
            "v0": 100.0, "accel": 1e9, "axisMax": [1000.0] * 6},
    "V22": {"firmware": "V22", "v0": 200.0, "accel": 4000.0,          # RAMP_START_SPEED, RAMP_ACCEL
            "axisMax": [2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 1000.0]},   # AXIS_MAX_STEP_RATE
}
FIRMWARE = os.environ.get("HAYLEY_FIRMWARE", "V22").upper()
MOTION = dict(MOTION_PROFILES.get(FIRMWARE, MOTION_PROFILES["V22"]), stepsPerDeg=STEPS_PER_DEG)
# legacy field kept for the page: top joint speed per axis (deg/s)
JOINT_SPEED_DEG_S = [MOTION.get("fullSpeed", MOTION["axisMax"][i]) / STEPS_PER_DEG[i] for i in range(6)]


class Motion:
    """Python twin of twin_web/motion.js (same maths)."""

    def __init__(self, params, ratio=1.0):
        self.p = dict(params)
        self.ratio = ratio

    def steps(self, a, b):
        return [int(round((b[i] - a[i]) * self.p["stepsPerDeg"][i])) for i in range(N_AXES)]

    def vmax_for(self, s):
        m = max(abs(v) for v in s)
        if m == 0:
            return self.p["v0"]
        v = min(self.p["axisMax"][i] * m / abs(s[i]) for i in range(N_AXES) if s[i])
        return max(v * self.ratio, self.p["v0"])

    def trapezoid(self, M, vs, ve, vm):
        A, v0 = self.p["accel"], self.p["v0"]
        vs = min(max(vs, v0), vm)
        ve = min(max(ve, v0), vm)
        vp = min(vm, math.sqrt((vs * vs + ve * ve) / 2 + A * M))
        d1 = max(0.0, (vp * vp - vs * vs) / (2 * A))
        d3 = max(0.0, (vp * vp - ve * ve) / (2 * A))
        d2 = max(0.0, M - d1 - d3)
        t1, t2, t3 = (vp - vs) / A, d2 / vp, (vp - ve) / A

        def at(t):
            if t <= 0:
                return 0.0
            if t < t1:
                return min(M, vs * t + A * t * t / 2)
            if t < t1 + t2:
                return min(M, d1 + vp * (t - t1))
            u = min(t - t1 - t2, t3)
            return min(M, d1 + d2 + vp * u - A * u * u / 2)
        return t1 + t2 + t3, at

    def launch(self, M):
        L = min(self.p["launchSteps"], M / 2.0)
        lt, ft = 1.0 / self.p["launchSpeed"], 1.0 / (self.p["fullSpeed"] * self.ratio)
        T = 2 * L * lt + (M - 2 * L) * ft

        def at(t):
            if t <= 0:
                return 0.0
            if t < L * lt:
                return t / lt
            if t < L * lt + (M - 2 * L) * ft:
                return L + (t - L * lt) / ft
            return min(M, M - L + (t - L * lt - (M - 2 * L) * ft) / lt)
        return T, at

    def plan_move(self, start, target, servo, nxt, pending_rate):
        """one queued row -> (duration, progress(t)->0..1, rate carried into the next row)"""
        s = self.steps(start, target)
        M = max(abs(v) for v in s)
        if M == 0:
            return 0.0, (lambda t: 1.0), 0.0
        if self.p["firmware"] == "V21":
            T, at = self.launch(M)
            return T, (lambda t: at(t) / M), 0.0
        v0, A = self.p["v0"], self.p["accel"]
        vm = self.vmax_for(s)
        vs = pending_rate * M if pending_rate > 0 else v0
        ve, rate = v0, 0.0
        if nxt is not None and servo not in (0.0, 1.0):
            ns = self.steps(target, nxt)
            Mn = max(abs(v) for v in ns)
            if Mn:
                jump = max(abs(s[i] - ns[i]) for i in range(N_AXES))
                r = v0 / jump if jump else 1e9
                r = min(r, vm / M, self.vmax_for(ns) / Mn,
                        math.sqrt(vs * vs + 2 * A * M) / M, math.sqrt(v0 * v0 + 2 * A * Mn) / Mn)
                if r * M > v0 * 1.05:
                    ve, rate = r * M, r
        T, at = self.trapezoid(M, vs, ve, vm)
        return T, (lambda t: at(t) / M), rate

HERE = os.path.dirname(os.path.abspath(__file__))
WEB_DIR = os.path.join(HERE, "twin_web")


# --------------------------------------------------------------------------
# Shared state
# --------------------------------------------------------------------------
class TwinState:
    def __init__(self, mode):
        self.lock = threading.RLock()
        self.mode = mode                     # "sim" or "hardware"
        self.measured = None                 # last encoder angles (deg)
        self.measured_t = 0.0
        self.commanded = None                # last commanded angles (deg)
        self.commanded_t = 0.0
        self.servo = None                    # 0.0 open, 1.0 closed
        self.stepper_enabled = None
        self.ps_enabled = None
        self.speed_mode = "high"
        self.commands = deque(maxlen=500)    # (seq, t, angles, servo)
        self.command_seq = 0                 # increments per angle command (for /twin/api/commands)
        self.io_errors = 0
        self.last_client_poll = 0.0

    def snapshot(self):
        with self.lock:
            return {
                "mode": self.mode,
                "t": time.time(),
                "measured": self.measured,
                "measured_age_s": (time.time() - self.measured_t) if self.measured_t else None,
                "commanded": self.commanded,
                "commanded_t": self.commanded_t or None,
                "servo": self.servo,
                "stepper_enabled": self.stepper_enabled,
                "ps_enabled": self.ps_enabled,
                "speed_mode": self.speed_mode,
                "commands_sent": len(self.commands),
                "command_seq": self.command_seq,
                "io_errors": self.io_errors,
            }


# --------------------------------------------------------------------------
# Simulated ESP32-S3
# --------------------------------------------------------------------------
class SimESP32:
    """Mimics the ESP32 I2C slave: accepts angle targets, reports encoder angles."""

    def __init__(self, start=None):
        self.angles = list(start or [0.0] * N_AXES)
        self.queue = deque()                 # (angles, servo)
        self.dropped = 0
        self.move = None                     # dict(t0, T, start, target, progress, dwell)
        self.pending_rate = 0.0
        self.motion = Motion(MOTION)
        self.stepper_enabled = True
        self.ps_enabled = False
        self.servo = 0.0
        self._lock = threading.Lock()

    def _start_next(self, t0):
        target, servo = self.queue.popleft()
        nxt = self.queue[0][0] if self.queue else None           # xQueuePeek
        start = list(self.angles)
        T, prog, rate = self.motion.plan_move(start, target, servo, nxt, self.pending_rate)
        self.pending_rate = rate
        self.move = {"t0": t0, "T": T, "start": start, "target": target, "progress": prog,
                     "dwell": 0.5 if servo in (0.0, 1.0) else 0.0, "servo": servo}

    def _advance(self):
        """Same timing as the firmware: trapezoid / blending (V22) or launch zones (V21)."""
        now = time.time()
        while True:
            if self.move is None:
                if not self.queue or not self.stepper_enabled:
                    return
                self._start_next(now)
            mv = self.move
            end = mv["t0"] + mv["T"]
            if now >= end:
                self.angles = list(mv["target"])
                if mv["servo"] in (0.0, 1.0):
                    self.servo = mv["servo"]
                if now < end + mv["dwell"]:
                    return                                       # claw open/close 500 ms
                self.move = None
                if self.queue and self.stepper_enabled:
                    self._start_next(end + mv["dwell"])          # next row starts where this one ended
                    continue
                self.pending_rate = 0.0
                return
            f = mv["progress"](now - mv["t0"])
            self.angles = [mv["start"][k] + (mv["target"][k] - mv["start"][k]) * f for k in range(N_AXES)]
            return

    # I2C side --------------------------------------------------------------
    def on_write(self, cmd, data):
        with self._lock:
            self._advance()
            if cmd == MSG_STEPPER_ARRAY:
                payload = bytes(data[1:])    # data[0] is the length byte the app inserts
                n = len(payload) // 4
                vals = list(struct.unpack("<%df" % n, payload[:n * 4]))
                angles, servo = vals[:N_AXES], (vals[N_AXES] if n > N_AXES else 2.0)
                if len(angles) == N_AXES and not any(abs(a - SEMAPHORE_VALUE) < 0.05 for a in angles):
                    if len(self.queue) >= ESP32_QUEUE_LEN:
                        self.dropped += 1   # same as the real firmware: queue full -> command lost
                        print("[twin-sim] ESP32 queue full (%d) - command dropped" % ESP32_QUEUE_LEN)
                    else:
                        self.queue.append((angles, servo))
            else:
                text = bytes(data[1:1 + data[0]]).decode("ascii", "ignore") if data else ""
                if text == "enable_stepper":
                    self.stepper_enabled = True
                elif text == "disable_stepper":
                    self.stepper_enabled = False
                    self.queue.clear(); self.move = None
                elif text == "enable_PS_control":
                    self.ps_enabled = True
                elif text == "disable_PS_control":
                    self.ps_enabled = False
                elif text in ("set_low_speed", "set_high_speed"):     # V22: also scales queued moves
                    self.motion.ratio = 0.2 if text == "set_low_speed" else 1.0

    def on_read(self, n):
        with self._lock:
            self._advance()
            status = (1 if self.stepper_enabled else 0) | (2 if self.ps_enabled else 0)
            raw = struct.pack("<6fB", *self.angles, status)
            return list(raw[:n]) + [0] * max(0, n - len(raw))


class SimBus:
    """smbus2.SMBus look-alike backed by SimESP32."""

    def __init__(self, esp=None):
        self.esp = esp or SimESP32()

    def write_i2c_block_data(self, addr, cmd, data, force=None):
        self.esp.on_write(cmd, list(data))

    def read_i2c_block_data(self, addr, reg, length, force=None):
        return self.esp.on_read(length)

    def close(self):
        pass


# --------------------------------------------------------------------------
# Observing bus wrapper
# --------------------------------------------------------------------------
class TwinBus:
    """Transparent proxy around the real/simulated bus that records traffic."""

    def __init__(self, bus, state, esp32_addr=0x05):
        self._bus = bus
        self.state = state
        self.addr = esp32_addr
        self.io_lock = threading.Lock()   # serialise app + twin access to I2C

    def write_i2c_block_data(self, addr, cmd, data, force=None):
        with self.io_lock:
            try:
                self._bus.write_i2c_block_data(addr, cmd, data)
            except IOError:
                with self.state.lock:
                    self.state.io_errors += 1
                raise
        if addr == self.addr:
            self._record_write(cmd, list(data))

    def read_i2c_block_data(self, addr, reg, length, force=None):
        with self.io_lock:
            try:
                data = self._bus.read_i2c_block_data(addr, reg, length)
            except IOError:
                with self.state.lock:
                    self.state.io_errors += 1
                raise
        if addr == self.addr and length >= 25:
            self._record_read(data)
        return data

    def __getattr__(self, name):           # anything else -> real bus
        return getattr(self._bus, name)

    # decoding --------------------------------------------------------------
    def _record_write(self, cmd, data):
        st = self.state
        try:
            if cmd == MSG_STEPPER_ARRAY:
                payload = bytes(data[1:])
                n = len(payload) // 4
                vals = list(struct.unpack("<%df" % n, payload[:n * 4]))
                angles = [round(v, 3) for v in vals[:N_AXES]]
                servo = vals[N_AXES] if n > N_AXES else None
                if len(angles) == N_AXES and not any(abs(a - SEMAPHORE_VALUE) < 0.05 for a in angles):
                    with st.lock:
                        st.commanded = angles
                        st.commanded_t = time.time()
                        if servo in (0.0, 1.0):
                            st.servo = servo
                        st.command_seq += 1
                        st.commands.append((st.command_seq, st.commanded_t, angles, servo))
            elif data:
                text = bytes(data[1:1 + data[0]]).decode("ascii", "ignore")
                with st.lock:
                    if text in ("set_low_speed", "set_high_speed"):
                        st.speed_mode = "low" if text == "set_low_speed" else "high"
        except Exception:
            pass

    def _record_read(self, data):
        try:
            vals = struct.unpack("<6fB", bytes(data[:25]))
        except struct.error:
            return
        with self.state.lock:
            self.state.measured = [round(v, 3) for v in vals[:6]]
            self.state.measured_t = time.time()
            self.state.stepper_enabled = bool(vals[6] & 1)
            self.state.ps_enabled = bool(vals[6] & 2)

    def poll_encoders(self):
        """Fresh encoder read on behalf of the twin page."""
        try:
            self.read_i2c_block_data(self.addr, 0x00, 25)
        except IOError:
            pass


# --------------------------------------------------------------------------
# Factory + Flask routes
# --------------------------------------------------------------------------
_twin_bus = None


def make_bus(smbus_module=None, bus_number=1, esp32_addr=0x05):
    """Return the bus the Dash app should use.

    HAYLEY_SIM=1 (or smbus2 not importable) -> simulated ESP32.
    Otherwise the real smbus2.SMBus(bus_number), wrapped for observation.
    """
    global _twin_bus
    sim = os.environ.get("HAYLEY_SIM", "0") == "1"
    if not sim and smbus_module is None:
        # never fall back to simulation silently when the robot is expected
        raise RuntimeError("smbus2 is not installed (pip install smbus2). "
                           "To run without hardware set HAYLEY_SIM=1 (run_twin_sim.bat on a PC).")
    if sim:
        raw, state = SimBus(), TwinState("sim")
        print("[twin] HAYLEY_SIM active - using simulated ESP32 (no hardware I/O)")
    else:
        raw, state = smbus_module.SMBus(bus_number), TwinState("hardware")
    _twin_bus = TwinBus(raw, state, esp32_addr)
    return _twin_bus


def _read_limits(config_csv):
    limits = []
    try:
        with open(config_csv, newline="") as f:
            for row in csv.DictReader(f):
                limits.append([float(row["Absolute low angle limit"]), float(row["Absolute high angle limit"])])
    except Exception:
        pass
    return limits


def register(server, config_csv=None, poll_hz=None):
    """Attach /twin routes to the Flask server behind the Dash app."""
    if _twin_bus is None:
        raise RuntimeError("call hayley_twin.make_bus() before register()")
    bus = _twin_bus
    poll_period = 1.0 / float(poll_hz or os.environ.get("HAYLEY_TWIN_POLL_HZ", 5))
    limits = _read_limits(config_csv) if config_csv else []

    bp = Blueprint("hayley_twin", __name__)

    @bp.route("/twin/")
    def twin_index():
        return send_from_directory(WEB_DIR, "index.html")

    @bp.route("/twin/<path:path>")
    def twin_static(path):
        full = os.path.normpath(os.path.join(WEB_DIR, path))
        if not full.startswith(os.path.normpath(WEB_DIR)) or not os.path.isfile(full):
            abort(404)
        resp = send_from_directory(WEB_DIR, path)
        if path.endswith(".stl"):
            resp.headers["Cache-Control"] = "public, max-age=86400"
        return resp

    @bp.route("/twin/api/state")
    def twin_state():
        st = bus.state
        with st.lock:
            stale = time.time() - st.measured_t > poll_period
            st.last_client_poll = time.time()
        if stale:
            bus.poll_encoders()   # only reads I2C while someone is watching
        return jsonify(bus.state.snapshot())

    @bp.route("/twin/api/commands")
    def twin_commands():
        """angle commands with seq > since (the twin rebuilds the queued path from these)"""
        try:
            since = int(request.args.get("since", 0))
        except ValueError:
            since = 0
        st = bus.state
        with st.lock:
            rows = [{"seq": q, "t": t, "deg": a, "servo": sv} for (q, t, a, sv) in st.commands if q > since]
            latest = st.command_seq
        return jsonify({"latest": latest, "commands": rows})

    @bp.route("/twin/api/config")
    def twin_config():
        return jsonify({
            "mode": bus.state.mode,
            "limits_deg": limits,
            "joint_speed_deg_s": JOINT_SPEED_DEG_S,
            "motion": MOTION,
            "poll_hz": 1.0 / poll_period,
        })

    server.register_blueprint(bp)
    print("[twin] digital twin at http://<pi-address>:8050/twin/  (mode: %s, firmware timing %s)"
          % (bus.state.mode, MOTION["firmware"]))
    return bus
