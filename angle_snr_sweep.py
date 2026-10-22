"""
Pan/tilt SNR sweep for the rotatable antenna gimbal.

Same LabVIEW / USRP measurement as fyp_ee_xyz_sweep.py, but instead of moving
the XYZ stage it rotates the gimbal (ESP32 over USB) to a grid of pan/tilt
angles and records the SNR at each one.

Put this file in the SAME folder as fyp_ee_xyz_sweep.py so the `labview`
package imports work. Needs: pyserial (import serial).

The ESP32 must be running gimbal_serial_control.ino.
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

COM_PORT = "COM7"      # ESP32's port on THIS PC: Device Manager -> Ports (COM & LPT)

# Angles in degrees from centre. Keep inside the limits set in the ESP32 code
# (pan -120..120, tilt -45..45).
PAN_MIN, PAN_MAX, PAN_STEP = -90, 90, 10
TILT_MIN, TILT_MAX, TILT_STEP = -30, 30, 10

SETTLE_S = 1.0         # wait after each move so the antenna stops wobbling
N_AVG = 3              # SNR readings averaged per angle

task_codename = "test-narrowband"
task_bandtype = "narrow"


# ------------------------------------------------------------
# GIMBAL (ESP32 over USB serial)
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
            # anything else (e.g. 'READY') is ignored

    def move(self, pan_deg, tilt_deg):
        self.ser.write(f"P{pan_deg:.1f} T{tilt_deg:.1f}\n".encode())
        return self._wait_ok()             # returns the angles actually reached

    def close(self):
        try:
            self.move(0, 0)
        finally:
            self.ser.close()


# ------------------------------------------------------------
# SNR MEASUREMENT (same as the XYZ sweep)
# ------------------------------------------------------------

def measureSNR():
    # Average the signal power in mW (linear), then convert back to dB.
    readings_dbm = [SignalPowerMeasure(labview, link_setup, link_opts) for _ in range(N_AVG)]
    mean_mw = np.mean([10 ** (p / 10) for p in readings_dbm])
    signal_power_dbm = 10 * np.log10(mean_mw)
    snr_db = signal_power_dbm - noise_power_dbm
    print("SNR approximate =", round(snr_db, 3), "dB")
    return snr_db


def generate_angles(a_min, a_max, step):
    angles = np.arange(a_min, a_max + step / 2, step)
    return angles[angles <= a_max]


def save_results(results, filename):
    np.savetxt(filename, np.array(results), delimiter=",",
               header="pan_deg,tilt_deg,snr_db", comments="")


# ------------------------------------------------------------
# SETUP
# ------------------------------------------------------------

pan_angles = generate_angles(PAN_MIN, PAN_MAX, PAN_STEP)
tilt_angles = generate_angles(TILT_MIN, TILT_MAX, TILT_STEP)
num_pan, num_tilt = len(pan_angles), len(tilt_angles)
total_measurements = num_pan * num_tilt

print("\n========================================")
print("PAN/TILT SWEEP SETTINGS")
print("========================================")
print("Pan  points =", num_pan, "| step =", PAN_STEP, "deg |", PAN_MIN, "to", PAN_MAX)
print("Tilt points =", num_tilt, "| step =", TILT_STEP, "deg |", TILT_MIN, "to", TILT_MAX)
print("Total measurements =", total_measurements)

gimbal = Gimbal(COM_PORT)
gimbal.move(0, 0)

labview = win32com.client.Dispatch("LabVIEW.Application")
link_setup, link_opts = LinkConfig(codename=task_codename, bandtype=task_bandtype)

RxAgentRun()
noise_power_dbm = NoisePowerMeasure(labview, link_setup, link_opts)   # TX off
link_setup.noise_power_dbm = noise_power_dbm
print("Noise power =", noise_power_dbm, "dBm")
TxAgentRun()

# snr_grid[tilt_index, pan_index]
snr_grid = np.full((num_tilt, num_pan), np.nan)
results = []
best_snr, best_pan, best_tilt = -np.inf, None, None
measurement_number = 0
start = time.time()

# ------------------------------------------------------------
# SERPENTINE SWEEP: one tilt row at a time, pan direction alternates
# ------------------------------------------------------------

try:
    for tilt_index, tilt in enumerate(tilt_angles):
        print("\n========================================")
        print("Tilt row", tilt_index + 1, "/", num_tilt, "| tilt =", tilt, "deg")
        print("========================================")

        pan_order = range(num_pan) if tilt_index % 2 == 0 else range(num_pan - 1, -1, -1)

        for pan_index in pan_order:
            pan = pan_angles[pan_index]

            actual_pan, actual_tilt = gimbal.move(pan, tilt)
            time.sleep(SETTLE_S)

            current_snr = measureSNR()
            snr_grid[tilt_index, pan_index] = current_snr
            results.append([actual_pan, actual_tilt, current_snr])
            measurement_number += 1

            print("Measurement", measurement_number, "/", total_measurements,
                  "| pan =", actual_pan, "| tilt =", actual_tilt,
                  "| SNR =", round(current_snr, 3), "dB")

            if current_snr > best_snr:
                best_snr, best_pan, best_tilt = current_snr, actual_pan, actual_tilt
                print("*** NEW BEST ANGLE ***")

        # Save after every row so nothing is lost if the run stops early
        save_results(results, "angle_snr_results_partial.csv")

finally:
    gimbal.close()          # returns to centre
    TxAgentStop()
    RxAgentStop()

# ------------------------------------------------------------
# FINISHED
# ------------------------------------------------------------

elapsed_time = time.time() - start
print("\n========================================")
print("PAN/TILT SWEEP COMPLETE")
print("========================================")
print("Best pan  =", best_pan, "deg")
print("Best tilt =", best_tilt, "deg")
print("Best SNR  =", best_snr, "dB")
print("Total time =", round(elapsed_time / 60, 2), "minutes")

save_results(results, "angle_snr_results.csv")
np.save("angle_snr_grid.npy", snr_grid)
print("Saved angle_snr_results.csv and angle_snr_grid.npy")

# ------------------------------------------------------------
# PLOT 1: SNR HEATMAP (pan x tilt)
# ------------------------------------------------------------

P, T = np.meshgrid(pan_angles, tilt_angles)
plt.figure()
mesh = plt.pcolormesh(P, T, snr_grid, shading="auto")
plt.colorbar(mesh, label="SNR (dB)")
plt.scatter(best_pan, best_tilt, marker="x", s=100, color="red", label="Best measured angle")
plt.xlabel("Pan angle (deg)")
plt.ylabel("Tilt angle (deg)")
plt.title("Measured SNR vs Antenna Pointing Angle")
plt.legend()
plt.savefig("angle_snr_heatmap.png", dpi=300, bbox_inches="tight")

# ------------------------------------------------------------
# PLOT 2: SNR vs PAN at the tilt row closest to 0 deg (beam cut)
# ------------------------------------------------------------

row = int(np.argmin(np.abs(tilt_angles)))
plt.figure()
plt.plot(pan_angles, snr_grid[row, :], marker="o")
plt.xlabel("Pan angle (deg)")
plt.ylabel("SNR (dB)")
plt.title("SNR vs Pan at tilt = " + str(tilt_angles[row]) + " deg")
plt.grid(True)
plt.savefig("pan_cut_snr.png", dpi=300, bbox_inches="tight")

plt.show()
