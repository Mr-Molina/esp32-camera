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
import urllib.error
import urllib.request

if sys.platform == "linux":
    import fcntl
    import termios
else:
    fcntl = None  # type: ignore
    termios = None  # type: ignore

DEFAULT_SERIAL_PORT = os.environ.get("CAM_SERIAL_PORT", "/dev/ttyUSB-esp32")
DEFAULT_HOST = os.environ.get("CAM_HOST", "")
DEFAULT_STATUS_URL = f"http://{DEFAULT_HOST}/status" if DEFAULT_HOST else ""


def get_camera_status(status_url: str, timeout: float = 3.0):
    if not status_url:
        return None
    try:
        req = urllib.request.Request(status_url, headers={"Connection": "close"})
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            if resp.status == 200:
                data = json.loads(resp.read().decode("utf-8"))
                if data.get("uptime_ms") is not None:
                    return data
    except (OSError, urllib.error.URLError, TimeoutError, ValueError) as exc:
        print(f"[WARN] Failed to read camera status from {status_url}: {exc}")
    return None


def ensure_passive(
    port_path: str = DEFAULT_SERIAL_PORT,
    status_url: str = DEFAULT_STATUS_URL,
    timeout: float = 3.0,
    check_http: bool = True,
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

    pre_status = None
    if check_http and status_url:
        pre_status = get_camera_status(status_url, timeout=timeout)
        if pre_status:
            print(
                f"[STATUS-PRE] Camera online, uptime_ms={pre_status.get('uptime_ms')}, active_partition={pre_status.get('active_partition')}"
            )
    elif check_http and not status_url:
        print("[WARN] No camera host or status URL provided; skipping pre-check HTTP status.", file=sys.stderr)

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
    if check_http and status_url:
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
    else:
        print("[PASSIVITY] Serial port passive (HTTP check skipped).")

    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Ensure host CH340 serial port passivity.")
    parser.add_argument(
        "--port",
        default=DEFAULT_SERIAL_PORT,
        help=f"Serial port path (default: {DEFAULT_SERIAL_PORT} or $CAM_SERIAL_PORT)",
    )
    parser.add_argument(
        "--host",
        default=DEFAULT_HOST,
        help="Camera IP address (default: $CAM_HOST or <camera-ip>)",
    )
    parser.add_argument(
        "--status-url",
        default=DEFAULT_STATUS_URL,
        help="Camera HTTP status URL (default: http://<camera-ip>/status or $CAM_HOST)",
    )
    parser.add_argument(
        "--no-http-check",
        action="store_true",
        help="Skip camera HTTP status checks (verify serial passivity only)",
    )
    args = parser.parse_args()

    status_url = args.status_url
    if not status_url and args.host:
        status_url = f"http://{args.host}/status"

    check_http = not args.no_http_check
    if check_http and not status_url:
        print("[WARN] No CAM_HOST or --status-url specified (<camera-ip>); skipping HTTP checks.")
        check_http = False

    return ensure_passive(
        port_path=args.port,
        status_url=status_url,
        check_http=check_http,
    )


if __name__ == "__main__":
    sys.exit(main())
