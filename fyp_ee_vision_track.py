"""
Camera-guided pointing (visual servoing) with an ArUco marker.

The camera rides on the gimbal. A printed ArUco marker (ID 0) is taped
to the TRANSMITTING antenna. The script:
  1. sweeps the gimbal to search for the marker (and searches again if it's lost),
  2. self-calibrates: nudges pan and tilt by a few degrees and watches how
     many pixels the marker moves (so no camera specs or sign guessing needed),
  3. keeps moving the gimbal until the marker sits at the target pixel
     (image centre by default),
  4. then either stops (pointing at the Tx) or keeps tracking it.

No radio / LabVIEW needed - this tests the vision part on its own.
Needs: opencv (4.7+), numpy, pyserial. ESP32 running gimbal_serial_control.ino.
Press q in the camera window to quit.
"""

import time
import csv
import numpy as np
import serial
import cv2


# ------------------------------------------------------------
# SETTINGS - change these
# ------------------------------------------------------------

COM_PORT = "COM3"
CAMERA_INDEX = 0          # try 1 or 2 if the wrong camera opens

MARKER_ID = 0             # the printed marker's ID

START_PAN, START_TILT = 0, 0
PAN_LIM, TILT_LIM = 90, 30

CAL_STEP_DEG = 5          # nudge size for self-calibration
GAIN = 0.7                # move 70% of the error each step (avoids overshoot)
MAX_STEP_DEG = 15         # never jump more than this in one step
TOL_DEG = 1.5             # "centred" if within this many degrees (servos are only ~1-2 deg accurate)
MIN_MOVE_DEG = 0.7        # skip moves smaller than this - the servo can't do them anyway
MAX_STEPS = 30            # give up after this many steps

SCAN_PAN_STEP = 20        # search sweep spacing (deg) - smaller than the camera's view so views overlap
SCAN_TILT_STEP = 15
LOST_LIMIT = 5            # marker missing this many looks in a row -> search again
TRACK_FOREVER = False     # True: keep following the marker until you press q

TARGET_PX = None          # None = image centre. Later: set to the pixel where the
                          # marker sits at the best-SNR angle, e.g. (330, 250)

SETTLE_S = 0.4            # wait after a move before looking again
FLUSH_FRAMES = 5          # old frames to throw away after a move (camera buffer lag)
AVG_FRAMES = 3            # frames averaged per detection


# ------------------------------------------------------------
# GIMBAL (same as the other scripts)
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
        time.sleep(2.5)                 # opening the port restarts the ESP32
        self.ser.reset_input_buffer()
        self.ser.write(b"?\n")
        self._wait_ok()

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
        pan_deg = float(np.clip(pan_deg, -PAN_LIM, PAN_LIM))
        tilt_deg = float(np.clip(tilt_deg, -TILT_LIM, TILT_LIM))
        for attempt in range(3):
            try:
                self.ser.write(f"P{pan_deg:.1f} T{tilt_deg:.1f}\n".encode())
                return self._wait_ok()
            except (NoReply, serial.SerialException):
                self.resets += 1
                print(f"  ESP32 not replying - reconnecting (attempt {attempt + 1}/3)...")
                try:
                    self.ser.close()
                except Exception:
                    pass
                time.sleep(1)
                try:
                    self._connect()
                except (NoReply, serial.SerialException):
                    pass
        raise RuntimeError("ESP32 stopped replying 3 times. Check USB cable, "
                           "servo power (6 V adapter) and the screw-terminal wires.")

    def close(self):
        self.ser.close()


# ------------------------------------------------------------
# MARKER DETECTION
# ------------------------------------------------------------

DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
try:                                    # OpenCV 4.7+
    _detector = cv2.aruco.ArucoDetector(DICT, cv2.aruco.DetectorParameters())
    def detect(gray):
        return _detector.detectMarkers(gray)
except AttributeError:                  # older OpenCV
    _params = cv2.aruco.DetectorParameters_create()
    def detect(gray):
        return cv2.aruco.detectMarkers(gray, DICT, parameters=_params)


def marker_centre(frame):
    """Pixel (u, v) of the marker's centre, or None if not seen."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detect(gray)
    if ids is None:
        return None
    for c, i in zip(corners, ids.flatten()):
        if i == MARKER_ID:
            return c[0].mean(axis=0)            # average of the 4 corners
    return None


def look(cap, status=""):
    """Grab fresh frames, return the marker position (median) and show the view."""
    for _ in range(FLUSH_FRAMES):
        cap.grab()
    found, frame = [], None
    for _ in range(AVG_FRAMES):
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError("Camera read failed. Check CAMERA_INDEX.")
        c = marker_centre(frame)
        if c is not None:
            found.append(c)
    uv = np.median(found, axis=0) if found else None
    show(frame, uv, status)
    return uv, frame


def show(frame, uv, status):
    h, w = frame.shape[:2]
    tx, ty = target_px(w, h)
    cv2.drawMarker(frame, (int(tx), int(ty)), (0, 255, 255), cv2.MARKER_CROSS, 30, 2)
    cv2.circle(frame, (int(tx), int(ty)), 15, (0, 255, 255), 1)
    if uv is not None:
        cv2.circle(frame, (int(uv[0]), int(uv[1])), 8, (0, 255, 0), 2)
        cv2.line(frame, (int(tx), int(ty)), (int(uv[0]), int(uv[1])), (0, 255, 0), 1)
    else:
        status = "MARKER NOT SEEN  " + status
    cv2.putText(frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.imshow("vision track (q = quit)", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        raise KeyboardInterrupt


def target_px(w, h):
    return TARGET_PX if TARGET_PX is not None else (w / 2, h / 2)


# ------------------------------------------------------------
# SEARCH: sweep the gimbal until the marker comes into view
# ------------------------------------------------------------

def search(cap, pan0, tilt0):
    """Look here first, then sweep row by row (nearest tilt rows first, snake in pan)."""
    print("Searching for the marker...")
    tilts = np.arange(-TILT_LIM, TILT_LIM + 0.1, SCAN_TILT_STEP)
    tilts = sorted(tilts, key=lambda t: abs(t - tilt0))
    pans = np.arange(-PAN_LIM, PAN_LIM + 0.1, SCAN_PAN_STEP)
    points = [(pan0, tilt0)]
    for i, t in enumerate(tilts):
        row = pans if i % 2 == 0 else pans[::-1]
        points += [(p, t) for p in row]

    for n, (p, t) in enumerate(points):
        p, t = gimbal.move(p, t)
        time.sleep(SETTLE_S)
        if look(cap, f"searching {n + 1}/{len(points)}")[0] is not None:
            print(f"Found the marker at pan {p:.1f}, tilt {t:.1f} (after {n + 1} looks)")
            return p, t
    raise RuntimeError("Marker not found anywhere in the range. "
                       "Check lighting, marker size, and that the camera is working.")


# ------------------------------------------------------------
# SELF-CALIBRATION: pixels moved per degree of pan / tilt
# ------------------------------------------------------------

def calibrate(cap, pan, tilt):
    uv0, _ = look(cap, "calibrating")
    k = []
    for axis in (0, 1):                         # 0 = pan, 1 = tilt
        for step in (CAL_STEP_DEG, -CAL_STEP_DEG):
            p = pan + (step if axis == 0 else 0)
            t = tilt + (step if axis == 1 else 0)
            gimbal.move(p, t)
            time.sleep(SETTLE_S)
            uv1, _ = look(cap, "calibrating")
            if uv1 is not None:
                break                           # marker still visible -> use this step
        if uv1 is None:
            raise RuntimeError("Lost the marker during calibration. Move it nearer the centre.")
        k.append((uv1[axis] - uv0[axis]) / step)  # pixels per degree (sign included)
        gimbal.move(pan, tilt)
        time.sleep(SETTLE_S)
    print(f"Calibration: {k[0]:+.1f} px per deg pan, {k[1]:+.1f} px per deg tilt")
    if abs(k[0]) < 2 or abs(k[1]) < 2:
        print("WARNING: very small movement - is the marker too far away?")
    return k


# ------------------------------------------------------------
# MAIN
# ------------------------------------------------------------

gimbal = Gimbal(COM_PORT)
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)   # DSHOW opens faster on Windows
if not cap.isOpened():
    cap = cv2.VideoCapture(CAMERA_INDEX)
if not cap.isOpened():
    raise RuntimeError("Could not open the camera. Try CAMERA_INDEX = 1.")

log = []
pan, tilt = START_PAN, START_TILT
start = time.time()

try:
    gimbal.move(pan, tilt)

    pan, tilt = search(cap, pan, tilt)
    k_pan, k_tilt = calibrate(cap, pan, tilt)

    step_no, centred, lost = 0, 0, 0
    while True:
        uv, frame = look(cap, f"step {step_no}")
        if uv is None:
            lost += 1
            if lost >= LOST_LIMIT:
                print("Marker lost - searching again")
                pan, tilt = search(cap, pan, tilt)
                lost = 0
            continue
        lost = 0

        h, w = frame.shape[:2]
        tx, ty = target_px(w, h)
        err_u, err_v = tx - uv[0], ty - uv[1]
        log.append([round(time.time() - start, 2), pan, tilt,
                    round(uv[0], 1), round(uv[1], 1), round(err_u, 1), round(err_v, 1)])

        err_pan_deg, err_tilt_deg = err_u / k_pan, err_v / k_tilt   # error in degrees
        if abs(err_pan_deg) <= TOL_DEG and abs(err_tilt_deg) <= TOL_DEG:
            centred += 1
            if centred == 2 and not TRACK_FOREVER:
                break
            continue
        centred = 0

        d_pan = float(np.clip(GAIN * err_pan_deg, -MAX_STEP_DEG, MAX_STEP_DEG))
        d_tilt = float(np.clip(GAIN * err_tilt_deg, -MAX_STEP_DEG, MAX_STEP_DEG))
        if abs(d_pan) < MIN_MOVE_DEG:
            d_pan = 0.0
        if abs(d_tilt) < MIN_MOVE_DEG:
            d_tilt = 0.0
        if d_pan == 0 and d_tilt == 0:          # too small for the servo: call it centred
            centred += 1
            if centred >= 2 and not TRACK_FOREVER:
                break
            continue

        if step_no >= MAX_STEPS and not TRACK_FOREVER:
            print(f"Stopped after {MAX_STEPS} steps without settling. "
                  "Check lighting / marker size, or lower GAIN.")
            break
        pan, tilt = gimbal.move(pan + d_pan, tilt + d_tilt)
        step_no += 1
        print(f"Step {step_no:2d} | error {err_u:+6.1f}, {err_v:+6.1f} px | "
              f"-> pan {pan:6.1f}, tilt {tilt:5.1f}")
        time.sleep(SETTLE_S)

    print("\n========================================")
    print(f"Centred on the marker: pan {pan:.1f}, tilt {tilt:.1f} deg")
    print(f"Steps: {step_no} | Time: {time.time() - start:.1f} s | ESP32 reconnects: {gimbal.resets}")

except KeyboardInterrupt:
    print("\nStopped by user.")
finally:
    with open("vision_track_log.csv", "w", newline="") as f:
        csv.writer(f).writerows([["t_s", "pan_deg", "tilt_deg", "u_px", "v_px",
                                  "err_u_px", "err_v_px"]] + log)
    print("Saved vision_track_log.csv")
    cap.release()
    cv2.destroyAllWindows()
    gimbal.close()          # stays pointing at the marker
