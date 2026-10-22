"""
Coarse-to-fine search for the best pan/tilt angle (highest SNR).

Same idea as fyp_ee_coarse_to_fine.py (XYZ), but for the gimbal:
  Level 1 : coarse grid over the whole angle range (5 x 5 points).
  Level 2+: small 3 x 3 grid around the best angle so far,
            with the spacing halved each level.
  Stops when the spacing on both axes is <= MIN_STEP_DEG,
  then points at the best angle and re-measures it.

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

PAN_MIN, PAN_MAX = -90, 90      # search range (deg); keep inside ESP32 limits
TILT_MIN, TILT_MAX = -30, 30

COARSE_POINTS = 5     # points per axis at level 1 (pan spacing 45 deg, tilt 15 deg)
FINE_POINTS = 3       # points per axis at later levels
MIN_STEP_DEG = 3      # stop when spacing <= this (servo accuracy is ~1-2 deg)

STAY_AT_BEST = True   # True: leave the antenna pointing at the best angle at the end
                      # False: return to centre (0, 0)

SETTLE_S = 1.0        # wait after each move so the antenna stops wobbling
N_AVG = 3             # SNR readings averaged per angle

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
# HELPERS
# ------------------------------------------------------------

measured = {}   # (pan, tilt) -> snr, so the same angle is never measured twice
results = []    # level, pan, tilt, snr


def axis_points(centre, half, num, low, high):
    """Evenly spaced angles around centre, kept inside the limits."""
    if half <= 0:
        return np.array([centre])
    pts = np.linspace(centre - half, centre + half, num)
    return np.unique(np.round(np.clip(pts, low, high), 1))


def snake_order(pans, tilts):
    """Visit one tilt row at a time, reversing pan each row (less travel)."""
    points = []
    for i, t in enumerate(tilts):
        row = pans if i % 2 == 0 else pans[::-1]
        points += [(p, t) for p in row]
    return points


def measure_at(pan, tilt, level):
    key = (round(float(pan), 1), round(float(tilt), 1))
    if key in measured:
        return measured[key]
    gimbal.move(*key)
    time.sleep(SETTLE_S)
    snr = measureSNR()
    measured[key] = snr
    results.append([level, key[0], key[1], snr])
    print(f"  L{level} | pan {key[0]:6.1f}  tilt {key[1]:5.1f} | SNR {snr:6.2f} dB")
    return snr


# ------------------------------------------------------------
# COARSE-TO-FINE SEARCH
# ------------------------------------------------------------

def coarse_to_fine():
    centre = np.array([(PAN_MIN + PAN_MAX) / 2, (TILT_MIN + TILT_MAX) / 2])
    half = np.array([(PAN_MAX - PAN_MIN) / 2, (TILT_MAX - TILT_MIN) / 2])
    num = COARSE_POINTS
    level = 0

    while True:
        level += 1
        pans = axis_points(centre[0], half[0], num, PAN_MIN, PAN_MAX)
        tilts = axis_points(centre[1], half[1], num, TILT_MIN, TILT_MAX)
        spacing = np.array([
            2 * half[0] / (num - 1) if half[0] > 0 else 0,
            2 * half[1] / (num - 1) if half[1] > 0 else 0,
        ])

        print("\n========================================")
        print(f"LEVEL {level} | spacing pan {spacing[0]:.1f} deg, tilt {spacing[1]:.1f} deg "
              f"| {len(pans) * len(tilts)} points")
        print("========================================")

        for p, t in snake_order(pans, tilts):
            measure_at(p, t, level)

        best_key = max(measured, key=measured.get)
        centre = np.array(best_key)
        print(f"Best so far: pan {best_key[0]:.1f}, tilt {best_key[1]:.1f} "
              f"-> {measured[best_key]:.2f} dB")

        # zoom in: next grid spans half a spacing either side of the best angle
        # (so the spacing halves). An axis that is already fine enough is frozen.
        done = np.all((spacing <= MIN_STEP_DEG) | (half <= 0))
        if done:
            break
        half = np.where(spacing > MIN_STEP_DEG, spacing / 2, 0)
        num = FINE_POINTS

    return best_key, measured[best_key]


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
    best, best_snr = coarse_to_fine()

    # point at the best angle and confirm
    gimbal.move(*best)
    time.sleep(SETTLE_S)
    final_snr = measureSNR()

    print("\n========================================")
    print(f"Best angle: pan {best[0]:.1f}, tilt {best[1]:.1f} deg")
    print(f"Best SNR during search = {best_snr:.2f} dB")
    print(f"SNR re-measured at best = {final_snr:.2f} dB")
    print(f"Total measurements = {len(results)}")
    print(f"Total time = {(time.time() - start) / 60:.1f} minutes")
except KeyboardInterrupt:
    print("\nStopped by user.")
finally:
    np.savetxt("angle_c2f_results.csv",
               np.array(results) if results else np.empty((0, 4)),
               delimiter=",", header="level,pan_deg,tilt_deg,snr_db", comments="")
    print("Saved angle_c2f_results.csv")
    TxAgentStop()
    RxAgentStop()
    gimbal.close(go_home=not (STAY_AT_BEST and best is not None))


# ------------------------------------------------------------
# PLOT: every measured angle, coloured by SNR
# ------------------------------------------------------------

if results:
    data = np.array(results)
    plt.figure(figsize=(8, 4.5))
    sc = plt.scatter(data[:, 1], data[:, 2], c=data[:, 3], cmap="viridis",
                     s=120 / data[:, 0], edgecolors="k", linewidths=0.3)   # later levels = smaller dots
    plt.colorbar(sc, label="SNR (dB)")
    if best is not None:
        plt.scatter(best[0], best[1], marker="x", s=150, color="red", label="Best angle")
        plt.legend()
    plt.xlabel("Pan (deg)")
    plt.ylabel("Tilt (deg)")
    plt.title("Coarse-to-Fine Angle Search")
    plt.xlim(PAN_MIN - 5, PAN_MAX + 5)
    plt.ylim(TILT_MIN - 5, TILT_MAX + 5)
    plt.grid(True)
    plt.savefig("angle_c2f_search.png", dpi=300, bbox_inches="tight")
    plt.show()
