#!/usr/bin/env python3
"""Capture one bounded STM32 motor-test command to a timestamped serial log."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import serial


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("command", help="single already-validated motor-test command")
    parser.add_argument("output", type=Path, help="log file to create")
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--timeout", type=float, default=90.0)
    args = parser.parse_args()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    records = 0
    complete = False
    started = time.monotonic()
    with serial.Serial(args.port, args.baud, timeout=0.1) as device, args.output.open(
        "w", encoding="utf-8", buffering=1
    ) as log:
        time.sleep(0.25)
        device.reset_input_buffer()
        log.write(f"# host_command={args.command}\n")
        device.write((args.command + "\n").encode())
        device.flush()

        while time.monotonic() - started < args.timeout:
            payload = device.readline()
            if not payload:
                continue
            line = payload.decode(errors="replace").rstrip("\r\n")
            log.write(line + "\n")
            if line.startswith("MAGDATA,"):
                records += 1
                if records % 100 == 0:
                    print(f"captured {records} calibration samples", flush=True)
            elif line.startswith(("error:", "ABORT:")):
                print(line, flush=True)
            elif line.startswith(("FAR STOP", "SYNCCAPTURE result", "status steps:")):
                print(line, flush=True)
            elif line == "SAFE: STBY is LOW":
                complete = True
                print(line, flush=True)
                break
        else:
            device.write(b"x")
            device.flush()
            print("capture timeout; abort character sent", flush=True)

    print(f"saved {records} records to {args.output}")
    return 0 if complete and records else 1


if __name__ == "__main__":
    raise SystemExit(main())
