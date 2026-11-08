"""Capture a serial port for a fixed time, prefixing each line with PC time.

Usage: python tools/serial_log.py COM7 10 [--reset] [--log FILE]

DTR/RTS are kept de-asserted so that opening the port does not reset the board.
--reset pulses RTS once (same as esptool's hard reset) to capture the boot log.
If the USB-CDC port disappears (board reset/re-enumeration), it is reopened.
"""
import argparse
import sys
import time

import serial


def open_port(name: str) -> serial.Serial:
    ser = serial.Serial()
    ser.port = name
    ser.baudrate = 115200
    ser.timeout = 0.2
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("seconds", type=float)
    ap.add_argument("--reset", action="store_true")
    ap.add_argument("--log")
    a = ap.parse_args()

    out = open(a.log, "a", encoding="utf-8") if a.log else None
    t0 = time.monotonic()

    def emit(text: str) -> None:
        line = f"[{time.monotonic() - t0:8.3f}] {text}"
        print(line, flush=True)
        if out:
            out.write(line + "\n")

    ser = open_port(a.port)
    if a.reset:
        # Same sequence as esptool's hard reset for USB-Serial/JTAG.
        ser.dtr = False
        ser.rts = True
        ser.dtr = ser.dtr  # Windows: force the control-line update
        time.sleep(0.2)
        ser.rts = False
        ser.dtr = ser.dtr
        emit("--- RTS reset pulse sent ---")
    buf = b""
    try:
        while time.monotonic() - t0 < a.seconds:
            try:
                buf += ser.read(512)
            except serial.SerialException:
                ser.close()
                emit("--- port lost, reopening ---")
                while time.monotonic() - t0 < a.seconds:
                    try:
                        time.sleep(0.2)
                        ser = open_port(a.port)
                        emit("--- port reopened ---")
                        break
                    except serial.SerialException:
                        continue
                continue
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                emit(line.decode("utf-8", "replace").rstrip())
    finally:
        ser.close()
        if out:
            out.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
