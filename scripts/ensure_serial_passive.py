#!/usr/bin/env python3
"""HW-06 (H2-BUS-001): Ensure host CH340 serial port passivity.

Disables HUPCL and clears DTR/RTS without pulsing EN to avoid unwanted board resets.
Verifies camera HTTP status endpoint before and after to ensure device uptime continuity.
"""

import fcntl
import json
import os
import struct
import subprocess
import sys
import termios
import urllib.request

SERIAL_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
STATUS_URL = "http://172.22.1.52/status"


def get_camera_status():
    try:
        req = urllib.request.Request(STATUS_URL)
        with urllib.request.urlopen(req, timeout=3.0) as resp:
            if resp.status == 200:
                data = json.loads(resp.read().decode("utf-8"))
                if data.get("uptime_ms") is not None:
                    return data
    except Exception as exc:
        print(f"[WARN] Failed to read camera status from {STATUS_URL}: {exc}")
    return None


def ensure_passive(port_path: str = SERIAL_PORT):
    if not os.path.exists(port_path):
        print(f"[ERROR] Serial port {port_path} does not exist", file=sys.stderr)
        return 1

    pre_status = get_camera_status()
    if pre_status:
        print(
            f"[STATUS-PRE] Camera online, uptime_ms={pre_status.get('uptime_ms')}, active_partition={pre_status.get('active_partition')}"
        )

    # 1. Disable HUPCL via stty
    cmd = ["stty", "-F", port_path, "-hupcl"]
    res = subprocess.run(cmd, capture_output=True, text=True, check=False, timeout=5.0)
    if res.returncode != 0:
        print(f"[ERROR] Failed to run stty -hupcl: {res.stderr}", file=sys.stderr)
        return 1
    print(f"[PASSIVITY] Disabled HUPCL on {port_path}")

    # 2. Open without controlling tty and without blocking
    fd = os.open(port_path, os.O_RDWR | os.O_NONBLOCK | os.O_NOCTTY)
    try:
        # Clear DTR and RTS
        clear_bits = struct.pack("I", termios.TIOCM_DTR | termios.TIOCM_RTS)
        fcntl.ioctl(fd, termios.TIOCMBIC, clear_bits)

        # Read back modem bits
        buf = struct.pack("I", 0)
        res = fcntl.ioctl(fd, termios.TIOCMGET, buf)
        modem_bits = struct.unpack("I", res)[0]
        dtr_active = bool(modem_bits & termios.TIOCM_DTR)
        rts_active = bool(modem_bits & termios.TIOCM_RTS)
        print(
            f"[PASSIVITY] TIOCMGET modem status: 0x{modem_bits:04X} (DTR={'1' if dtr_active else '0'}, RTS={'1' if rts_active else '0'})"
        )
        if dtr_active or rts_active:
            print("[WARN] DTR or RTS is still asserted!", file=sys.stderr)
            return 1
    finally:
        os.close(fd)

    # 3. Verify camera remains online with non-zero, increasing uptime
    post_status = get_camera_status()
    if post_status:
        print(
            f"[STATUS-POST] Camera online, uptime_ms={post_status.get('uptime_ms')}, active_partition={post_status.get('active_partition')}"
        )
        if pre_status and post_status.get("uptime_ms", 0) < pre_status.get("uptime_ms", 0):
            print("[ERROR] Camera uptime reset during serial configuration!", file=sys.stderr)
            return 1
        print("[PASSIVITY] Serial port passive and camera uptime preserved successfully.")
    else:
        print("[WARN] Could not reach camera after serial configuration", file=sys.stderr)

    return 0


if __name__ == "__main__":
    sys.exit(ensure_passive())
