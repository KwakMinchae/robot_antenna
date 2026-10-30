"""
Continuous tracking: stage 1 (smooth sweep) + stage 2 (real-time tracking).

STEP 0 - timing test
  Takes a few SNR readings standing still to measure how long one reading
  takes. This sets how fast the gimbal can glide in stage 1.

STAGE 1 - find the best angle without stopping (you pick the algorithm)
  The gimbal never stops. It glides while the radio reads SNR non-stop.
  Every reading is time-stamped and matched to where the gimbal was
  at that moment. Rough search, one of:
    1) Coarse to fine: glide tilt up/down in a few pan columns.
    2) Bayesian: glide from target to target. Every reading on the way
       updates the SNR map, which picks the next most promising target.
    3) Sequential lobing: the circling tracker (same as stage 2), started
       with a wide circle, runs until it settles.
  Then for all three - refine: glide forward then backward through the
  rough best. The servo lags, so the forward pass sees the peak a bit late
  and the backward pass a bit early. The true peak is halfway between.
  The gap also gives the lag, which stage 2 uses.
  Each run adds one row to stage1_comparison.csv (readings, time, final
  SNR) so you can compare the three algorithms.

  Pick the algorithm when asked, or skip the question:
      python fyp_ee_continuous_tracking.py 3     -> sequential lobing

STAGE 2 - real-time tracking (move the Tx by hand, the Rx follows)
  The aim draws a small circle non-stop ("conical scan"). From the last
  few readings it works out which side of the circle is louder and
  shifts the centre that way. If SNR crashes (Tx moved a lot), the
  circle widens to re-find it, then shrinks back.
  STAGE2_MODE = "frozen" keeps the antenna still instead: run it with the
  same hand movement to get a baseline to compare against.

Same folder as fyp_ee_angle_snr_sweep.py (needs the `labview` package).
ESP32 must be running gimbal_serial_control.ino (no change needed).
Stop any time with Ctrl+C - results are still saved.
"""

import os
import sys
import math
import time
import threading
from collections import deque

import numpy as np
import serial
import win32com.client
import matplotlib.pyplot as plt
from ctypes import *

from motion.config import MotionConfig
from motion.tasks import UserOpenDevice
try:
    from motion.tasks import UserCloseDevice
except ImportError:
    from motion.control import UserCloseDevice

from labview.config import LinkConfig
from labview.agent import TxAgentRun, RxAgentRun, TxAgentStop, RxAgentStop
from labview.measure import SignalPowerMeasure, NoisePowerMeasure


# ------------------------------------------------------------
# SETTINGS - change these
# ------------------------------------------------------------

COM_PORT = "COM3"
PAN_LIM, TILT_LIM = 90, 30        # stay inside +-these

USE_STAGE = True                  # True: connect the XYZ stage and send it back to
                                  # (0, 0, 0) at the end. False: gimbal only.
FMC4030_DLL = ("C:/Users/Zhu Lipeng/Desktop/ma-prototype/scripts/"
               "FMC4030-dll/FMC4030-Dll-x64.dll")

RUN_STAGE1 = True                 # False: skip stage 1, start tracking from START_*
START_PAN, START_TILT = 0, 0

TIMING_READINGS = 10              # readings for the timing test

# Stage 1
DEG_PER_READING = 2.0             # how far the gimbal moves during ONE reading.
                                  # Smaller = sharper peak but slower sweep.
REFINE_SPAN_DEG = 20              # refine passes cover +-this around the rough best
REFINE_PAN = False                # omni antenna: pan barely changes SNR, so skip
FINAL_READINGS = 5                # readings averaged at the final best angle

# 1) Coarse to fine
RASTER_PANS = [-90, -45, 0, 45, 90]   # pan columns; tilt glides along each

# 2) Bayesian
BO_N_INIT = 3                     # random targets first
BO_TARGETS = 8                    # then this many targets picked by the model
BO_LS_PAN, BO_LS_TILT = 40, 15    # how far (deg) SNR stays similar on each axis
BO_TRAVEL_DEG = 60                # larger = care less about travel distance

# 3) Sequential lobing
LOBING_SETTLED_UPDATES = 4        # stop after this many "on target" updates in a row
LOBING_MAX_S = 90                 # give up after this long

# Stage 2
STAGE2_MODE = "track"             # "track" or "frozen" (baseline, antenna stays still)
STAGE2_SECONDS = None             # None = run until Ctrl+C
TRACK_PAN = False                 # omni antenna: track tilt only. True for both axes.
WOBBLE_DEG = 5                    # circle radius when locked on
WOBBLE_WIDE_DEG = 15              # circle radius when the source is lost
READINGS_PER_CIRCLE = 8           # readings per lap of the circle
GAIN = 10                         # deg moved per (dB/deg) of slope
MAX_STEP_DEG = 5                  # never shift the centre more than this per update
DEADBAND_DB = 0.5                 # SNR difference across the circle below this = on target
LOST_DROP_DB = 3.0                # SNR drop this big = source moved -> widen circle

MOVE_HZ = 20                      # move commands per second while gliding
SETTLE_S = 1.0                    # wait at the start of each pass

task_codename = "test-narrowband"
task_bandtype = "narrow"


# ------------------------------------------------------------
# GIMBAL (same as the other scripts)
# ------------------------------------------------------------

class Gimbal:
    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=15)
        time.sleep(2.5)                 # opening the port restarts the ESP32
        self.ser.reset_input_buffer()
        self.ser.write(b"?\n")
        self._wait_ok()
        print("Gimbal connected on", port)

    def _wait_ok(self):
        while True:
            line = self.ser.readline().decode(errors="ignore").strip()
            if line == "":
                raise RuntimeError("No reply from ESP32. Check COM port, USB cable and servo power.")
            if line.startswith("ERR"):
                raise RuntimeError("ESP32 said: " + line)
            if line.startswith("OK"):
                parts = line.split()        # e.g. ['OK', 'P30.0', 'T-10.0']
                return float(parts[1][1:]), float(parts[2][1:])

    def move(self, pan_deg, tilt_deg):
        self.ser.write(f"P{pan_deg:.1f} T{tilt_deg:.1f}\n".encode())
        return self._wait_ok()

    def close(self):
        try:
            self.move(0, 0)
        finally:
            self.ser.close()


# ------------------------------------------------------------
# MOTION THREAD - keeps the gimbal gliding while SNR is read
# ------------------------------------------------------------

class Motion:
    """Runs in the background. Many times a second it asks `target(t)` where
    the gimbal should be now, sends that small move, and logs the time and
    angle. The main code reads SNR at the same time and looks up the angle."""

    def __init__(self, gimbal):
        self.g = gimbal
        self.lock = threading.Lock()
        self.target = None
        self.hist = deque(maxlen=5000)       # (time, pan, tilt) of recent moves
        self.error = None
        self.running = True
        self.th = threading.Thread(target=self._loop, daemon=True)
        self.th.start()

    def set_target(self, fn):
        with self.lock:
            self.target = fn

    def hold(self, pan, tilt):
        self.set_target(lambda t: (pan, tilt))

    def _loop(self):
        period = 1.0 / MOVE_HZ
        while self.running:
            t0 = time.time()
            with self.lock:
                fn = self.target
            if fn is not None:
                try:
                    p, tl = fn(t0)
                    p = float(np.clip(p, -PAN_LIM, PAN_LIM))
                    tl = float(np.clip(tl, -TILT_LIM, TILT_LIM))
                    self.g.move(p, tl)
                    with self.lock:
                        self.hist.append((time.time(), p, tl))
                except Exception as e:
                    self.error = e
                    self.running = False
                    return
            wait = period - (time.time() - t0)
            if wait > 0:
                time.sleep(wait)

    def angle_at(self, t):
        with self.lock:
            h = np.array(self.hist)
        if self.error:
            raise RuntimeError("Gimbal stopped: " + str(self.error))
        if len(h) == 0:
            return np.nan, np.nan
        return float(np.interp(t, h[:, 0], h[:, 1])), float(np.interp(t, h[:, 0], h[:, 2]))

    def rate_hz(self):
        with self.lock:
            h = np.array(self.hist)[-50:]
        return (len(h) - 1) / (h[-1, 0] - h[0, 0]) if len(h) > 1 else 0.0

    def stop(self):
        self.running = False
        self.th.join(timeout=3)


# ------------------------------------------------------------
# SNR READING (one reading, time-stamped at its middle)
# ------------------------------------------------------------

def read_snr():
    t0 = time.time()
    p = SignalPowerMeasure(labview, link_setup, link_opts)
    t1 = time.time()
    return (t0 + t1) / 2, p - noise_power_dbm, t1 - t0


log = []          # [time_s, pan, tilt, snr, stage, centre_pan, centre_tilt, wobble]
t_zero = time.time()


def add_log(tm, pan, tilt, snr, stage, cp=np.nan, ct=np.nan, r=np.nan):
    log.append([tm - t_zero, pan, tilt, snr, stage, cp, ct, r])


# ------------------------------------------------------------
# STAGE 1 - continuous sweep
# ------------------------------------------------------------

def clip_angle(p, t):
    return float(np.clip(p, -PAN_LIM, PAN_LIM)), float(np.clip(t, -TILT_LIM, TILT_LIM))


def glide_pass(waypoints, speed, settle=True):
    """Glide through waypoints at constant speed (deg/s) while reading SNR.
    Returns arrays of (pan, tilt, snr), angles matched to each reading."""
    pts = np.array([clip_angle(*w) for w in waypoints], float)
    motion.hold(*pts[0])
    if settle:
        time.sleep(SETTLE_S)

    cum = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
    t_start = time.time()
    t_end = t_start + cum[-1] / speed

    def fn(t):
        d = min((t - t_start) * speed, cum[-1])
        return np.interp(d, cum, pts[:, 0]), np.interp(d, cum, pts[:, 1])

    motion.set_target(fn)
    out = []
    while time.time() < t_end:
        tm, snr, _ = read_snr()
        p, tl = motion.angle_at(tm)
        out.append((p, tl, snr))
        add_log(tm, p, tl, snr, 1)
    motion.hold(*pts[-1])
    return np.array(out)


def pass_peak(x, snr):
    """Angle where SNR peaks along one pass (smoothed, with sub-step fit)."""
    if len(snr) < 3:
        return float(x[np.argmax(snr)])
    s = np.convolve(snr, np.ones(3) / 3, mode="same")
    s[0], s[-1] = snr[0], snr[-1]
    i = int(np.argmax(s))
    if 0 < i < len(s) - 1:
        x0, x1, x2 = x[i - 1:i + 2]
        y0, y1, y2 = s[i - 1:i + 2]
        denom = (y0 - 2 * y1 + y2)
        if denom < 0 and x0 != x2:
            shift = 0.5 * (y0 - y2) / denom      # in steps
            return float(x1 + shift * (x2 - x0) / 2)
    return float(x[i])


def refine_axis(axis, cp, ct, speed):
    """Forward + backward pass through the rough best along one axis.
    Returns (best angle on that axis, lag in seconds)."""
    if axis == "tilt":
        a, b, k = (cp, ct - REFINE_SPAN_DEG), (cp, ct + REFINE_SPAN_DEG), 1
    else:
        a, b, k = (cp - REFINE_SPAN_DEG, ct), (cp + REFINE_SPAN_DEG, ct), 0
    fwd = glide_pass([a, b], speed)
    bwd = glide_pass([b, a], speed)
    xf = pass_peak(fwd[:, k], fwd[:, 2])
    xb = pass_peak(bwd[:, k], bwd[:, 2])
    best = (xf + xb) / 2
    lag = (xf - xb) / 2 / speed           # forward sees the peak late by speed*lag
    print(f"  {axis:4s} refine | forward peak {xf:6.1f}  backward peak {xb:6.1f}"
          f"  -> best {best:6.1f} deg, lag {lag:5.2f} s")
    return best, lag


def rough_coarse_to_fine(speed):
    pts = []
    for i, p in enumerate(RASTER_PANS):            # snake: up, down, up...
        lo, hi = (-TILT_LIM, TILT_LIM) if i % 2 == 0 else (TILT_LIM, -TILT_LIM)
        pts += [(p, lo), (p, hi)]
    data = glide_pass(pts, speed)
    i = int(np.argmax(data[:, 2]))
    return float(data[i, 0]), float(data[i, 1])


PAN_GRID, TILT_GRID = 5, 2        # Bayesian map resolution (deg)


def gp_predict(X, y, C):
    """Gaussian process: guessed SNR and uncertainty at candidates C."""
    def k(A, B):
        return np.exp(-0.5 * ((A[:, None, :] - B[None, :, :]) ** 2).sum(-1))
    m, s = y.mean(), y.std() + 1e-9
    yn = (y - m) / s
    L = np.linalg.cholesky(k(X, X) + 0.1 * np.eye(len(X)))
    alpha = np.linalg.solve(L.T, np.linalg.solve(L, yn))
    Ks = k(C, X)
    v = np.linalg.solve(L, Ks.T)
    mu = Ks @ alpha
    sd = np.sqrt(np.clip(1 - (v ** 2).sum(0), 1e-9, None))
    return m + s * mu, s * sd


def rough_bayesian(speed):
    pans = np.arange(-PAN_LIM, PAN_LIM + 1, PAN_GRID)
    tilts = np.arange(-TILT_LIM, TILT_LIM + 1, TILT_GRID)
    cand = np.array([(p, t) for p in pans for t in tilts], float)
    scale = np.array([BO_LS_PAN, BO_LS_TILT], float)
    erf = np.vectorize(math.erf)

    def binned(readings):
        """Average readings that fall in the same map cell."""
        R = np.array(readings)
        pi = np.clip(np.round((R[:, 0] + PAN_LIM) / PAN_GRID), 0, len(pans) - 1)
        ti = np.clip(np.round((R[:, 1] + TILT_LIM) / TILT_GRID), 0, len(tilts) - 1)
        cells, inv = np.unique((pi * len(tilts) + ti).astype(int), return_inverse=True)
        y = np.bincount(inv, R[:, 2]) / np.bincount(inv)
        return cells, y

    rng = np.random.default_rng()
    here = np.array(clip_angle(START_PAN, START_TILT))
    readings = []
    for k in range(BO_N_INIT + BO_TARGETS):
        if k < BO_N_INIT:
            nxt, why = cand[rng.integers(len(cand))], "random"
        else:
            cells, y = binned(readings)
            mu, sd = gp_predict(cand[cells] / scale, y, cand / scale)
            z = (mu - y.max()) / sd
            ei = (mu - y.max()) * 0.5 * (1 + erf(z / np.sqrt(2))) \
                + sd * np.exp(-0.5 * z ** 2) / np.sqrt(2 * np.pi)
            ei /= 1 + np.linalg.norm(cand - here, axis=1) / BO_TRAVEL_DEG
            nxt, why = cand[int(np.argmax(ei))], f"model guess {mu[int(np.argmax(ei))]:.1f} dB"
        if np.allclose(nxt, here):
            continue
        data = glide_pass([here, nxt], speed, settle=(k == 0))
        readings += data.tolist()
        here = nxt
        print(f"  target {k + 1:2d}: pan {nxt[0]:6.1f} tilt {nxt[1]:5.1f} ({why})"
              f" | {len(readings)} readings so far")
    cells, y = binned(readings)
    best = cand[cells[int(np.argmax(y))]]
    return float(best[0]), float(best[1])


def stage1(speed, algo, t_read):
    print(f"\n--- STAGE 1: {algo.replace('_', ' ')} (continuous) ---")
    t_s, n_s = time.time(), len(log)
    if algo == "coarse_to_fine":
        cp, ct = rough_coarse_to_fine(speed)
    elif algo == "bayesian":
        cp, ct = rough_bayesian(speed)
    else:
        # Lobing needs the lag BEFORE it starts circling: otherwise the servo delay
        # makes it misjudge which side is louder and it walks the wrong way.
        ct0, lag0 = refine_axis("tilt", START_PAN, START_TILT, speed)
        lag0 = lag0 if 0 <= lag0 <= 1.5 else 0.0
        cp, ct = conical(START_PAN, ct0, lag0, t_read, stage=1,
                         r_start=WOBBLE_WIDE_DEG, max_s=LOBING_MAX_S, search=True)
    print(f"  rough search: {len(log) - n_s} readings in {time.time() - t_s:.0f} s"
          f" -> pan {cp:.1f}, tilt {ct:.1f}")

    ct, lag = refine_axis("tilt", cp, ct, speed)
    if REFINE_PAN:
        cp, lag_p = refine_axis("pan", cp, ct, speed)
        lag = (lag + lag_p) / 2
    if not (0 <= lag <= 1.5):
        print(f"  lag {lag:.2f} s looks wrong (noisy pass?) -> using 0")
        lag = 0.0
    cp, ct = clip_angle(cp, ct)
    n_used, t_used = len(log) - n_s, time.time() - t_s

    motion.hold(cp, ct)                       # final check at the best angle
    time.sleep(SETTLE_S)
    final = []
    for _ in range(FINAL_READINGS):
        tm, snr, _ = read_snr()
        final.append(snr)
        add_log(tm, cp, ct, snr, 1)
    final_snr = float(np.mean(final))

    print(f"  STAGE 1 RESULT ({algo}): pan {cp:.1f}, tilt {ct:.1f} deg | SNR {final_snr:.2f} dB"
          f" | {n_used} readings, {t_used:.0f} s | lag {lag:.2f} s")
    new_file = not os.path.exists("stage1_comparison.csv")
    with open("stage1_comparison.csv", "a") as f:
        if new_file:
            f.write("date_time,algorithm,readings,time_s,best_pan,best_tilt,final_snr_db,lag_s\n")
        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')},{algo},{n_used},{t_used:.1f},"
                f"{cp:.1f},{ct:.1f},{final_snr:.2f},{lag:.3f}\n")
    print("  Added a row to stage1_comparison.csv")
    return cp, ct, lag


# ------------------------------------------------------------
# CIRCLING TRACKER - sequential lobing (stage 1 option 3) and stage 2
# ------------------------------------------------------------

def conical(cp, ct, lag, t_read, stage, r_start, max_s=None, search=False, update=True):
    """The aim draws a circle non-stop and drifts toward the louder side.
    search=True (stage 1): start wide, shrink once close, stop once settled.
    update=False: circle off, antenna stays still (stage 2 baseline).
    Returns the final aim (pan, tilt)."""
    st = {"cp": cp, "ct": ct, "r": r_start}
    w = 2 * np.pi / (READINGS_PER_CIRCLE * t_read)
    t0 = time.time()

    def fn(t):
        ph = w * (t - t0)
        dp = st["r"] * np.cos(ph) if TRACK_PAN else 0.0
        return st["cp"] + dp, st["ct"] + st["r"] * np.sin(ph)

    motion.set_target(fn)
    recent = deque(maxlen=READINGS_PER_CIRCLE)
    levels = deque(maxlen=10)
    lost, settled, n = False, 0, 0
    while max_s is None or time.time() - t0 < max_s:
        tm, snr, _ = read_snr()
        p, tl = motion.angle_at(tm - lag)
        recent.append((p, tl, snr))
        add_log(tm, p, tl, snr, stage, st["cp"], st["ct"], st["r"])
        n += 1
        if not update or len(recent) < READINGS_PER_CIRCLE or n % (READINGS_PER_CIRCLE // 2):
            continue

        # Which side is louder? Fit a flat slope to the last lap: snr = a + b*pan + c*tilt
        d = np.array(recent)
        cols = [np.ones(len(d))]
        use_p = TRACK_PAN and np.std(d[:, 0]) > 0.5
        use_t = np.std(d[:, 1]) > 0.5
        if use_p: cols.append(d[:, 0])
        if use_t: cols.append(d[:, 1])
        coef = np.linalg.lstsq(np.column_stack(cols), d[:, 2], rcond=None)[0]
        gp = coef[1] if use_p else 0.0
        gt = coef[-1] if use_t else 0.0

        level = float(np.mean(d[:, 2]))
        levels.append(level)
        note = "on target"
        if 2 * st["r"] * np.hypot(gp, gt) > DEADBAND_DB:     # one side clearly louder
            st["cp"] += float(np.clip(GAIN * gp, -MAX_STEP_DEG, MAX_STEP_DEG))
            st["ct"] += float(np.clip(GAIN * gt, -MAX_STEP_DEG, MAX_STEP_DEG))
            st["cp"], st["ct"] = clip_angle(st["cp"], st["ct"])
            note = "shift"
        settled = settled + 1 if note == "on target" else 0

        if search:
            if note == "on target" and st["r"] > WOBBLE_DEG:
                st["r"], note, settled = WOBBLE_DEG, "close -> small circle", 0
            elif settled >= LOBING_SETTLED_UPDATES:
                print(f"  t {tm - t0:6.1f} s | settled at pan {st['cp']:.1f} tilt {st['ct']:.1f}")
                break
        elif level < max(levels) - LOST_DROP_DB:
            st["r"], note, lost = WOBBLE_WIDE_DEG, "SNR dropped -> wide circle", True
        elif lost and level > max(levels) - 1.0:
            st["r"], note, lost = WOBBLE_DEG, "re-found -> small circle", False
        print(f"  t {tm - t0:6.1f} s | aim pan {st['cp']:6.1f} tilt {st['ct']:5.1f}"
              f" | SNR {level:6.2f} dB | slope {gp:+.2f}/{gt:+.2f} dB/deg"
              f" | circle {st['r']:4.1f} | {note}")
    return st["cp"], st["ct"]


def stage2(cp, ct, lag, t_read):
    print(f"\n--- STAGE 2: {STAGE2_MODE} (Ctrl+C to stop) ---")
    track = STAGE2_MODE == "track"
    conical(cp, ct, lag, t_read, stage=2, r_start=WOBBLE_DEG if track else 0.0,
            max_s=STAGE2_SECONDS, update=track)


# ------------------------------------------------------------
# SETUP
# ------------------------------------------------------------

ALGOS = {"1": "coarse_to_fine", "2": "bayesian", "3": "lobing"}


def choose_algo():
    if len(sys.argv) > 1 and sys.argv[1] in ALGOS:
        return ALGOS[sys.argv[1]]
    print("Which algorithm for stage 1?\n  1) Coarse to fine\n  2) Bayesian\n  3) Sequential lobing")
    while True:
        a = input("Type 1, 2 or 3: ").strip()
        if a in ALGOS:
            return ALGOS[a]
        print("  Please type 1, 2 or 3.")


algo = choose_algo() if RUN_STAGE1 else "no_stage1"
out_name = f"continuous_tracking_{algo}"

if USE_STAGE:
    fmc4030 = windll.LoadLibrary(FMC4030_DLL)
    motion_setup, motion_status, motion_opts = MotionConfig()
    UserOpenDevice(fmc4030, motion_setup, motion_opts)
    fmc4030.FMC4030_Get_Machine_Status(motion_setup.id, pointer(motion_status))
    print("Stage connected at", list(motion_status.realPos[:]))

gimbal = Gimbal(COM_PORT)
motion = Motion(gimbal)

labview = win32com.client.Dispatch("LabVIEW.Application")
link_setup, link_opts = LinkConfig(codename=task_codename, bandtype=task_bandtype)

RxAgentRun()
noise_power_dbm = NoisePowerMeasure(labview, link_setup, link_opts)   # TX off
link_setup.noise_power_dbm = noise_power_dbm
print("Noise power =", noise_power_dbm, "dBm")
TxAgentRun()

try:
    # STEP 0 - timing test
    motion.hold(START_PAN, START_TILT)
    time.sleep(SETTLE_S)
    t_read = float(np.median([read_snr()[2] for _ in range(TIMING_READINGS)]))
    speed = DEG_PER_READING / t_read
    print(f"\nOne SNR reading takes {t_read * 1000:.0f} ms -> glide speed {speed:.1f} deg/s"
          f" | gimbal updates at {motion.rate_hz():.0f} Hz")

    if RUN_STAGE1:
        cp, ct, lag = stage1(speed, algo, t_read)
    else:
        cp, ct, lag = START_PAN, START_TILT, 0.0
    stage2(cp, ct, lag, t_read)
except KeyboardInterrupt:
    print("\nStopped by user.")
finally:
    motion.stop()
    np.savetxt(out_name + ".csv",
               np.array(log) if log else np.empty((0, 8)), delimiter=",",
               header="time_s,pan_deg,tilt_deg,snr_db,stage,aim_pan,aim_tilt,circle_deg",
               comments="")
    print("Saved", out_name + ".csv")
    TxAgentStop()
    RxAgentStop()
    try:
        gimbal.close()                      # gimbal back to pan 0, tilt 0
    finally:
        if USE_STAGE:                       # stage back to (0, 0, 0), then disconnect
            UserCloseDevice(fmc4030, motion_setup, motion_status, motion_opts)
            print("Stage returned to (0, 0, 0) and disconnected")


# ------------------------------------------------------------
# PLOTS
# ------------------------------------------------------------

if log:
    L = np.array(log)
    s1, s2 = L[L[:, 4] == 1], L[L[:, 4] == 2]
    fig, ax = plt.subplots(1, 3, figsize=(17, 4.5))

    if len(s1):
        sc = ax[0].scatter(s1[:, 1], s1[:, 2], c=s1[:, 3], cmap="viridis", s=12)
        plt.colorbar(sc, ax=ax[0], label="SNR (dB)")
        if len(s2):
            ax[0].scatter(s2[0, 5], s2[0, 6], marker="x", s=150, color="red", label="stage 1 best")
            ax[0].legend()
    ax[0].set_xlabel("Pan (deg)"); ax[0].set_ylabel("Tilt (deg)")
    ax[0].set_title(f"Stage 1 ({algo}): readings"); ax[0].grid(True)

    if len(s2):
        tt = s2[:, 0] - s2[0, 0]
        ax[1].plot(tt, s2[:, 3], ".", ms=3, color="grey", label="each reading")
        k = READINGS_PER_CIRCLE
        if len(s2) >= k:
            ax[1].plot(tt[k - 1:], np.convolve(s2[:, 3], np.ones(k) / k, "valid"),
                       "b-", label="average over one lap")
        ax[1].set_xlabel("Time (s)"); ax[1].set_ylabel("SNR (dB)")
        ax[1].set_title(f"Stage 2 ({STAGE2_MODE}): SNR vs time"); ax[1].legend(); ax[1].grid(True)

        ax[2].plot(tt, s2[:, 5], label="aim pan")
        ax[2].plot(tt, s2[:, 6], label="aim tilt")
        ax[2].set_xlabel("Time (s)"); ax[2].set_ylabel("Angle (deg)")
        ax[2].set_title("Stage 2: where the antenna aims"); ax[2].legend(); ax[2].grid(True)

    plt.tight_layout()
    plt.savefig(out_name + ".png", dpi=300, bbox_inches="tight")
    plt.show()
