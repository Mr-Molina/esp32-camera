#!/usr/bin/env python3
"""HW-06 (H2-BUS-001): Ensure host CH340 serial port passivity.

Disables HUPCL and clears DTR/RTS without pulsing EN to avoid unwanted board resets.
Verifies camera HTTP status endpoint before and after to ensure device uptime continuity.
"""

import argparse
import json
import os
import struct
import sys
import urllib.request

if sys.platform == "linux":
    import fcntl
    import termios
else:
    fcntl = None  # type: ignore
    termios = None  # type: ignore

SERIAL_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
STATUS_URL = "http://172.22.1.52/status"


def get_camera_status(status_url: str = STATUS_URL, timeout: float = 3.0):
    try:
        req = urllib.request.Request(status_url, headers={"Connection": "close"})
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            if resp.status == 200:
                data = json.loads(resp.read().decode("utf-8"))
                if data.get("uptime_ms") is not None:
                    return data
    except Exception as exc:
        print(f"[WARN] Failed to read camera status from {status_url}: {exc}")
    return None


def ensure_passive(
    port_path: str = SERIAL_PORT,
    status_url: str = STATUS_URL,
    timeout: float = 3.0,
):
    if sys.platform != "linux" or fcntl is None or termios is None:
        print(
            f"[ERROR] Serial passivity control is only supported on Linux (current: {sys.platform})",
            file=sys.stderr,
        )
        return 1

    if not os.path.exists(port_path):
        print(f"[ERROR] Serial port {port_path} does not exist", file=sys.stderr)
        return 1

    pre_status = get_camera_status(status_url, timeout=timeout)
    if pre_status:
        print(
            f"[STATUS-PRE] Camera online, uptime_ms={pre_status.get('uptime_ms')}, active_partition={pre_status.get('active_partition')}"
        )

    # 1. Open without controlling tty and without blocking directly (avoiding stty process invocation)
    fd = os.open(port_path, os.O_RDWR | os.O_NONBLOCK | os.O_NOCTTY)
    try:
        # Clear HUPCL via termios in attrs[2] (c_cflag)
        attrs = termios.tcgetattr(fd)
        attrs[2] &= ~termios.HUPCL
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        print(f"[PASSIVITY] Disabled HUPCL via termios on {port_path}")

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

    # 2. Verify camera remains online with non-zero, increasing uptime
    post_status = get_camera_status(status_url, timeout=timeout)
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
        return 1

    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Ensure host CH340 serial port passivity.")
    parser.add_argument("--port", default=SERIAL_PORT, help=f"Serial port path (default: {SERIAL_PORT})")
    parser.add_argument(
        "--status-url", default=STATUS_URL, help=f"Camera HTTP status URL (default: {STATUS_URL})"
    )
    args = parser.parse_args()
    return ensure_passive(port_path=args.port, status_url=args.status_url)


if __name__ == "__main__":
    sys.exit(main())
