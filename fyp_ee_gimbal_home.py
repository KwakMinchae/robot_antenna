"""
Move the gimbal back to its neutral position (pan 0, tilt 0).

Usage:
  python fyp_ee_gimbal_home.py            -> goes to pan 0, tilt 0
  python fyp_ee_gimbal_home.py 30 -10     -> goes to pan 30, tilt -10

No radio / LabVIEW needed. ESP32 must be running gimbal_serial_control.ino.
"""

import sys
import time
import serial

COM_PORT = "COM3"


def wait_ok(ser):
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if line == "":
            return None
        if line.startswith("ERR"):
            raise RuntimeError("ESP32 said: " + line)
        if line.startswith("OK"):
            return line


pan = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
tilt = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0

ser = serial.Serial(COM_PORT, 115200, timeout=6)
time.sleep(2.5)                         # opening the port restarts the ESP32

try:
    for attempt in range(4):            # first message can be lost while it boots
        ser.reset_input_buffer()
        ser.write(b"?\n")
        if wait_ok(ser):
            break
        print(f"  waiting for ESP32 to answer ({attempt + 1}/4)...")
    else:
        raise RuntimeError("ESP32 is not answering. Unplug its USB and the 6 V adapter, "
                           "wait 5 s, plug USB back first, then the adapter.")

    ser.write(f"P{pan:.1f} T{tilt:.1f}\n".encode())
    reply = wait_ok(ser)
    if reply is None:
        raise RuntimeError("No reply after the move command.")
    print("Gimbal now at:", reply)
finally:
    ser.close()
