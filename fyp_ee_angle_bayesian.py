"""
Bayesian optimisation for the best pan/tilt angle (highest SNR).

How it works:
  1. Measure a few spread-out starting angles.
  2. Build a "best guess" SNR map over every angle from all measurements so far
     (Gaussian process: a guess + how unsure it is, at every angle).
  3. Measure next where the chance of beating the current best is highest
     (Expected Improvement), slightly favouring angles that are quick to reach.
  4. Stop when no angle looks worth measuring, or at MAX_MEASUREMENTS.
     Then point at the best measured angle, re-measure, and stay there.

Same folder as fyp_ee_angle_snr_sweep.py (needs the `labview` package).
Only numpy / matplotlib / pyserial - no extra installs.
Stop any time with Ctrl+C - results are still saved.
"""

import time
import math
import numpy as np
import serial
import win32com.client
import matplotlib.pyplot as plt

from labview.config import LinkConfig
from labview.agent import TxAgentRun, RxAgentRun, TxAgentStop, RxAgentStop
from labview.measure import SignalPowerMeasure, NoisePowerMeasure


# ------------------------------------------------------------
# SETTINGS - change these
# ------------------------------------------------------------

COM_PORT = "COM3"

PAN_MIN, PAN_MAX = -90, 90
TILT_MIN, TILT_MAX = -30, 30
GRID_STEP_DEG = 3          # candidate angles every 3 deg (servo accuracy ~1-2 deg)

N_INITIAL = 6              # spread-out starting measurements
MAX_MEASUREMENTS = 25      # total budget (coarse-to-fine used ~45-50)
MIN_MEASUREMENTS = 12      # never stop before this many
EI_STOP_DB = 0.1           # stop when the best expected gain anywhere is below this

NOISE = 0.02               # how much the model trusts each reading (bigger = noisier)
LENGTHSCALES_DEG = [8, 12, 18, 25, 35]   # beam "width" guesses; best one picked automatically
TRAVEL_PENALTY = 0.3       # 0 = ignore travel; higher = prefer nearby angles
SERVO_SPEED_DEG_S = 60     # matches the ESP32 code

STAY_AT_BEST = True
SETTLE_S = 1.0
N_AVG = 3
RANDOM_SEED = 0

task_codename = "test-narrowband"
task_bandtype = "narrow"


# ------------------------------------------------------------
# GIMBAL (with auto-reconnect)
# ------------------------------------------------------------

class NoReply(Exception):
    pass


class Gimbal:
    def __init__(self, port):
        self.port = port
        self.resets = 0
        self._connect()
        print("Gimbal connected on", port)

    def _connect(self):
        self.ser = serial.Serial(self.port, 115200, timeout=6)
        time.sleep(2.5)
        for attempt in range(4):
            self.ser.reset_input_buffer()
            self.ser.write(b"?\n")
            try:
                return self._wait_ok()
            except NoReply:
                print(f"  waiting for ESP32 to answer ({attempt + 1}/4)...")
        raise RuntimeError("ESP32 is not answering. Unplug its USB and the 6 V adapter, "
                           "wait 5 s, plug USB back first, then the adapter.")

    def _wait_ok(self):
        while True:
            line = self.ser.readline().decode(errors="ignore").strip()
            if line == "":
                raise NoReply()
            if line.startswith("ERR"):
                raise RuntimeError("ESP32 said: " + line)
            if line.startswith("OK"):
                parts = line.split()
                return float(parts[1][1:]), float(parts[2][1:])

    def move(self, pan_deg, tilt_deg):
        for attempt in range(3):
            try:
                self.ser.write(f"P{pan_deg:.1f} T{tilt_deg:.1f}\n".encode())
                return self._wait_ok()
            except (NoReply, serial.SerialException):
                self.resets += 1
                print(f"  ESP32 not replying - reconnecting ({attempt + 1}/3)...")
                try:
                    self.ser.close()
                except Exception:
                    pass
                time.sleep(1)
                try:
                    self._connect()
                except (NoReply, RuntimeError, serial.SerialException):
                    pass
        raise RuntimeError("ESP32 stopped replying 3 times. Check USB and servo power.")

    def close(self, go_home=True):
        try:
            if go_home:
                self.move(0, 0)
        finally:
            self.ser.close()


# ------------------------------------------------------------
# SNR MEASUREMENT (same as the sweep script)
# ------------------------------------------------------------

def measureSNR():
    readings_dbm = [SignalPowerMeasure(labview, link_setup, link_opts) for _ in range(N_AVG)]
    mean_mw = np.mean([10 ** (p / 10) for p in readings_dbm])
    return 10 * np.log10(mean_mw) - noise_power_dbm


# ------------------------------------------------------------
# GAUSSIAN PROCESS + EXPECTED IMPROVEMENT (numpy only)
# ------------------------------------------------------------

def rbf(A, B, ls):
    """Similarity between angles: 1 when identical, -> 0 when far apart."""
    d2 = ((A[:, None, :] - B[None, :, :]) ** 2).sum(-1)
    return np.exp(-0.5 * d2 / ls ** 2)


def gp_fit_predict(X, y, C):
    """Pick the best lengthscale, then return guessed SNR (dB) and uncertainty (dB) at C."""
    mu0, sd0 = y.mean(), y.std() + 1e-9
    yn = (y - mu0) / sd0
    best = None
    for ls in LENGTHSCALES_DEG:
        K = rbf(X, X, ls) + NOISE * np.eye(len(X))
        try:
            L = np.linalg.cholesky(K)
        except np.linalg.LinAlgError:
            continue
        a = np.linalg.solve(L.T, np.linalg.solve(L, yn))
        fit = -0.5 * yn @ a - np.log(np.diag(L)).sum()      # how well this lengthscale explains the data
        if best is None or fit > best[0]:
            best = (fit, ls, L, a)
    _, ls, L, a = best
    Ks = rbf(C, X, ls)
    mu = Ks @ a
    v = np.linalg.solve(L, Ks.T)
    var = np.clip(1.0 - (v ** 2).sum(0), 1e-12, None)
    return mu * sd0 + mu0, np.sqrt(var) * sd0, ls


def norm_cdf(z):
    # standard normal CDF via erf (Abramowitz-Stegun approximation, no scipy needed)
    x = np.abs(z) / math.sqrt(2)
    t = 1 / (1 + 0.3275911 * x)
    poly = t * (0.254829592 + t * (-0.284496736 + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429))))
    erf = 1 - poly * np.exp(-x * x)
    return 0.5 * (1 + np.sign(z) * erf)


def expected_improvement(mu, sd, best):
    """Expected gain (dB) over the current best, if we measured there."""
    z = (mu - best) / sd
    pdf = np.exp(-0.5 * z ** 2) / math.sqrt(2 * math.pi)
    return sd * (z * norm_cdf(z) + pdf)


# ------------------------------------------------------------
# SEARCH
# ------------------------------------------------------------

pans = np.arange(PAN_MIN, PAN_MAX + 0.1, GRID_STEP_DEG)
tilts = np.arange(TILT_MIN, TILT_MAX + 0.1, GRID_STEP_DEG)
CAND = np.array([(p, t) for t in tilts for p in pans], dtype=float)

results = []        # n, pan, tilt, snr, note
X, Y, measured_idx = [], [], []
pos = np.array([0.0, 0.0])


def measure_at(i, note):
    global pos
    p, t = CAND[i]
    gimbal.move(p, t)
    time.sleep(SETTLE_S)
    snr = measureSNR()
    pos = CAND[i].copy()
    X.append(CAND[i]); Y.append(snr); measured_idx.append(i)
    results.append([len(Y), p, t, snr, note])
    tag = "  <- NEW BEST" if snr >= max(Y) else ""
    print(f"#{len(Y):2d} | pan {p:6.1f}  tilt {t:5.1f} | SNR {snr:6.2f} dB | {note}{tag}")


def initial_points(n, rng):
    """Spread-out starting angles (Latin hypercube: one per row and column band)."""
    idx = []
    ps = (rng.permutation(n) + rng.random(n)) / n
    ts = (rng.permutation(n) + rng.random(n)) / n
    for a, b in zip(ps, ts):
        target = np.array([PAN_MIN + a * (PAN_MAX - PAN_MIN), TILT_MIN + b * (TILT_MAX - TILT_MIN)])
        idx.append(int(np.argmin(((CAND - target) ** 2).sum(1))))
    # visit them in pan order to reduce travel
    return sorted(set(idx), key=lambda i: CAND[i][0])


def bayesian_search():
    rng = np.random.default_rng(RANDOM_SEED)
    print(f"Starting points ({N_INITIAL}):")
    for i in initial_points(N_INITIAL, rng):
        measure_at(i, "start")

    while len(Y) < MAX_MEASUREMENTS:
        mu, sd, ls = gp_fit_predict(np.array(X), np.array(Y), CAND)
        ei = expected_improvement(mu, sd, max(Y))
        ei[measured_idx] = 0                       # never re-measure the same angle

        if len(Y) >= MIN_MEASUREMENTS and ei.max() < EI_STOP_DB:
            print(f"Stopping: nothing left worth > {EI_STOP_DB} dB (beam width guess {ls} deg)")
            break

        if TRAVEL_PENALTY > 0:                     # both servos move together -> time = bigger move
            travel_s = np.abs(CAND - pos).max(1) / SERVO_SPEED_DEG_S
            score = ei / (1 + TRAVEL_PENALTY * travel_s)
        else:
            score = ei
        i = int(np.argmax(score))
        measure_at(i, f"EI {ei[i]:.2f} dB, ls {ls}")

    best_n = int(np.argmax(Y))
    return CAND[measured_idx[best_n]], Y[best_n]


# ------------------------------------------------------------
# SETUP
# ------------------------------------------------------------

gimbal = Gimbal(COM_PORT)
gimbal.move(0, 0)

labview = win32com.client.Dispatch("LabVIEW.Application")
link_setup, link_opts = LinkConfig(codename=task_codename, bandtype=task_bandtype)

RxAgentRun()
noise_power_dbm = NoisePowerMeasure(labview, link_setup, link_opts)   # TX off
link_setup.noise_power_dbm = noise_power_dbm
print("Noise power =", noise_power_dbm, "dBm")
TxAgentRun()

start = time.time()
best = None

try:
    best, best_snr = bayesian_search()
    gimbal.move(*best)
    time.sleep(SETTLE_S)
    final_snr = measureSNR()

    print("\n========================================")
    print(f"Best angle: pan {best[0]:.1f}, tilt {best[1]:.1f} deg")
    print(f"Best SNR during search = {best_snr:.2f} dB")
    print(f"SNR re-measured at best = {final_snr:.2f} dB")
    print(f"Total measurements = {len(Y)}")
    print(f"Total time = {(time.time() - start) / 60:.1f} minutes | ESP32 reconnects: {gimbal.resets}")
except KeyboardInterrupt:
    print("\nStopped by user.")
finally:
    np.savetxt("angle_bayes_results.csv",
               np.array([r[:4] for r in results]) if results else np.empty((0, 4)),
               delimiter=",", header="n,pan_deg,tilt_deg,snr_db", comments="")
    print("Saved angle_bayes_results.csv")
    TxAgentStop()
    RxAgentStop()
    gimbal.close(go_home=not (STAY_AT_BEST and best is not None))


# ------------------------------------------------------------
# PLOTS: model's final SNR map + convergence
# ------------------------------------------------------------

if len(Y) >= 3:
    Xa, Ya = np.array(X), np.array(Y)
    mu, sd, ls = gp_fit_predict(Xa, Ya, CAND)

    fig, ax = plt.subplots(1, 2, figsize=(13, 4.5))
    im = ax[0].imshow(mu.reshape(len(tilts), len(pans)), origin="lower", aspect="auto",
                      extent=[PAN_MIN, PAN_MAX, TILT_MIN, TILT_MAX], cmap="viridis")
    plt.colorbar(im, ax=ax[0], label="Predicted SNR (dB)")
    ax[0].plot(Xa[:, 0], Xa[:, 1], "w.-", lw=0.6, ms=8, label="measured (in order)")
    if best is not None:
        ax[0].plot(best[0], best[1], "rx", ms=12, mew=3, label="best")
    ax[0].set_xlabel("Pan (deg)")
    ax[0].set_ylabel("Tilt (deg)")
    ax[0].set_title(f"Bayesian model of SNR (beam width guess {ls} deg)")
    ax[0].legend(loc="upper right", fontsize=8)

    ax[1].plot(np.arange(1, len(Ya) + 1), np.maximum.accumulate(Ya), "o-", label="best so far")
    ax[1].plot(np.arange(1, len(Ya) + 1), Ya, ".", alpha=0.5, label="each measurement")
    ax[1].set_xlabel("Measurement number")
    ax[1].set_ylabel("SNR (dB)")
    ax[1].set_title("Convergence")
    ax[1].grid(True)
    ax[1].legend()

    plt.tight_layout()
    plt.savefig("angle_bayes_search.png", dpi=300, bbox_inches="tight")
    plt.show()
