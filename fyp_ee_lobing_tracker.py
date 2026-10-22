"""
Bio-inspired "wiggle and listen" tracker (sequential lobing).

Like a bat or moth turning its head left/right and up/down to hear
which side is louder, the antenna:
  1. peeks a little LEFT and RIGHT of where it points, and compares SNR,
  2. peeks a little UP and DOWN, and compares SNR,
  3. steps toward the louder side.
If no side is clearly louder, it is pointing at the source, so it
makes the peeks smaller to fine-tune. If the SNR suddenly drops
(source moved), it makes the peeks bigger again to re-find it.

Same folder as fyp_ee_angle_snr_sweep.py (needs the `labview` package).
ESP32 must be running gimbal_serial_control.ino.
Stop any time with Ctrl+C - results are still saved.
"""

import time
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

START_PAN, START_TILT = 60, 20   # start AWAY from the source to test that it finds it

PAN_LIM, TILT_LIM = 90, 30       # stay inside +-these (same as the sweep)

PEEK_START = 15     # how far it peeks each side at the start (deg)
PEEK_MIN = 3        # smallest peek (deg) - fine-tuning size
DEADBAND_DB = 0.5   # left/right difference smaller than this = "equal" (noise)
LOST_DROP_DB = 3.0  # SNR drop this big = source moved -> peek wide again

MAX_ROUNDS = 25     # set to None to keep tracking forever (Ctrl+C to stop)

SETTLE_S = 1.0      # wait after each move so the antenna stops wobbling
N_AVG = 3           # SNR readings averaged per point

task_codename = "test-narrowband"
task_bandtype = "narrow"


# ------------------------------------------------------------
# GIMBAL (same as the sweep script)
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
                raise RuntimeError("No reply from ESP32. Check COM port and that the code is uploaded.")
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
# SNR MEASUREMENT (same as the sweep script)
# ------------------------------------------------------------

def measureSNR():
    readings_dbm = [SignalPowerMeasure(labview, link_setup, link_opts) for _ in range(N_AVG)]
    mean_mw = np.mean([10 ** (p / 10) for p in readings_dbm])
    return 10 * np.log10(mean_mw) - noise_power_dbm


n_measurements = 0
log = []    # every measurement: round, pan, tilt, snr, kind


def snr_at(pan, tilt, rnd, kind):
    global n_measurements
    pan = float(np.clip(pan, -PAN_LIM, PAN_LIM))
    tilt = float(np.clip(tilt, -TILT_LIM, TILT_LIM))
    gimbal.move(pan, tilt)
    time.sleep(SETTLE_S)
    snr = measureSNR()
    n_measurements += 1
    log.append([rnd, pan, tilt, snr, kind])
    return snr


# ------------------------------------------------------------
# WIGGLE AND LISTEN
# ------------------------------------------------------------

def compare(snr_plus, snr_minus, peek):
    """Return how far to step: +peek, -peek, or 0 if both sides are about equal."""
    diff = snr_plus - snr_minus
    if diff > DEADBAND_DB:
        return peek
    if diff < -DEADBAND_DB:
        return -peek
    return 0


def track():
    pan, tilt = START_PAN, START_TILT
    peek = PEEK_START
    prev_centre = snr_at(pan, tilt, 0, "centre")
    print(f"Start  | pan {pan:6.1f} tilt {tilt:5.1f} | SNR {prev_centre:6.2f} dB")

    last_d_pan, last_d_tilt = 0, 0
    rnd = 0
    while MAX_ROUNDS is None or rnd < MAX_ROUNDS:
        rnd += 1

        # 1. wiggle left/right
        right = snr_at(pan + peek, tilt, rnd, "peek")
        left = snr_at(pan - peek, tilt, rnd, "peek")
        # 2. wiggle up/down
        up = snr_at(pan, tilt + peek, rnd, "peek")
        down = snr_at(pan, tilt - peek, rnd, "peek")

        # 3. step toward the louder side
        d_pan = compare(right, left, peek)
        d_tilt = compare(up, down, peek)
        pan = float(np.clip(pan + d_pan, -PAN_LIM, PAN_LIM))
        tilt = float(np.clip(tilt + d_tilt, -TILT_LIM, TILT_LIM))

        centre = snr_at(pan, tilt, rnd, "centre")

        overshot = (d_pan * last_d_pan < 0) or (d_tilt * last_d_tilt < 0)
        if d_pan == 0 and d_tilt == 0:
            peek = max(PEEK_MIN, peek / 2)          # on target: fine-tune
            note = "centred -> smaller peek"
        elif overshot:
            peek = max(PEEK_MIN, peek / 2)          # went back the way it came: peak is in between
            note = "overshot -> smaller peek"
        else:
            note = "stepped"
        if peek < PEEK_START and centre < prev_centre - LOST_DROP_DB:
            peek = PEEK_START                       # source moved: look wide
            note = "SNR dropped -> wide peek"
        prev_centre = centre
        last_d_pan, last_d_tilt = d_pan, d_tilt

        print(f"Round {rnd:2d} | pan {pan:6.1f} tilt {tilt:5.1f} | "
              f"SNR {centre:6.2f} dB | L/R {left:5.1f}/{right:5.1f} "
              f"D/U {down:5.1f}/{up:5.1f} | peek {peek:4.1f} | {note}")

    return pan, tilt


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

try:
    final_pan, final_tilt = track()
    print("\n========================================")
    print(f"Final pointing: pan {final_pan:.1f}, tilt {final_tilt:.1f} deg")
    print(f"Measurements used: {n_measurements}")
    print(f"Time: {(time.time() - start) / 60:.1f} min")
except KeyboardInterrupt:
    print("\nStopped by user.")
finally:
    np.savetxt("lobing_results.csv",
               np.array([r[:4] + [1 if r[4] == "centre" else 0] for r in log]) if log else np.empty((0, 5)),
               delimiter=",", header="round,pan_deg,tilt_deg,snr_db,is_centre", comments="")
    print("Saved lobing_results.csv")
    TxAgentStop()
    RxAgentStop()
    gimbal.close()


# ------------------------------------------------------------
# PLOTS
# ------------------------------------------------------------

if log:
    data = np.array([r[:4] for r in log])
    is_c = np.array([r[4] == "centre" for r in log])
    c = data[is_c]

    fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))

    ax[0].scatter(data[~is_c, 1], data[~is_c, 2], c="lightgrey", s=15, label="peeks")
    sc = ax[0].scatter(c[:, 1], c[:, 2], c=c[:, 3], cmap="viridis", s=50, zorder=3)
    ax[0].plot(c[:, 1], c[:, 2], "k-", lw=0.8, zorder=2)
    ax[0].scatter(c[0, 1], c[0, 2], marker="s", s=120, facecolors="none",
                  edgecolors="red", label="start")
    ax[0].scatter(c[-1, 1], c[-1, 2], marker="x", s=150, color="red", label="end")
    plt.colorbar(sc, ax=ax[0], label="SNR (dB)")
    ax[0].set_xlabel("Pan (deg)")
    ax[0].set_ylabel("Tilt (deg)")
    ax[0].set_title("Pointing path")
    ax[0].legend()
    ax[0].grid(True)

    ax[1].plot(c[:, 0], c[:, 3], "o-")
    ax[1].set_xlabel("Round")
    ax[1].set_ylabel("SNR at centre (dB)")
    ax[1].set_title("SNR vs round")
    ax[1].grid(True)

    plt.tight_layout()
    plt.savefig("lobing_tracker.png", dpi=300, bbox_inches="tight")
    plt.show()
