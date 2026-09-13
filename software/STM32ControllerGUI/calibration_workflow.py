"""Monitor the calibration routine built into the STM32 production firmware."""

from __future__ import annotations

import queue
import re
import subprocess
import sys
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import serial
from serial.tools import list_ports


BAUD_RATE = 921600
DEVICE_HINT = "F401RE_25MHZ_USB_CDC"
MIN_EXPECTED_SPAN_DEG = 75.0
MAX_RETURN_ERROR_DEG = 1.0
REPOSITORY = Path(__file__).resolve().parents[2]
CALIBRATION_DIR = REPOSITORY / "documentation" / "calibration_scan"
CALIBRATION_PLOTTER = REPOSITORY / "software" / "EncoderMonitor" / "plot_magscan.py"


class CalibrationAborted(RuntimeError):
    pass


@dataclass
class AxisCalibrationResult:
    axis: int
    measured_span_deg: float
    safe_limit_deg: float
    far_stop_command_deg: float
    return_error_deg: float
    samples: int
    encoder_fault_samples: int
    log_path: str


class CalibrationWorker(threading.Thread):
    """Send M56, capture progress/results, and leave firmware unchanged."""

    daemon = True

    def __init__(self, events: queue.Queue, preferred_port: str = "") -> None:
        super().__init__()
        self.events = events
        self.preferred_port = preferred_port
        self.abort_event = threading.Event()
        self.serial_lock = threading.Lock()
        self.device: serial.Serial | None = None
        self.process: subprocess.Popen | None = None

    def abort(self) -> None:
        self.abort_event.set()
        with self.serial_lock:
            if self.device and self.device.is_open:
                try:
                    self.device.write(b"x")
                    self.device.flush()
                    self.events.put(("log", "!!! CALIBRATION ABORT SENT !!!"))
                except (serial.SerialException, OSError):
                    pass

    def _emit_status(self, text: str, progress: int) -> None:
        self.events.put(("calibration_status", text, progress))
        self.events.put(("log", text))

    def _run_process(self, arguments: list[str], label: str) -> None:
        self.events.put(("log", f"--- {label} ---"))
        self.process = subprocess.Popen(
            arguments,
            cwd=REPOSITORY,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        assert self.process.stdout is not None
        for line in self.process.stdout:
            clean = line.rstrip()
            if clean:
                self.events.put(("log", clean))
        return_code = self.process.wait()
        self.process = None
        if return_code:
            raise RuntimeError(f"{label} failed with exit code {return_code}")

    @staticmethod
    def _port_candidates(preferred: str) -> list[str]:
        candidates: list[str] = []
        if preferred:
            candidates.append(preferred)
        by_id = Path("/dev/serial/by-id")
        if by_id.is_dir():
            candidates.extend(
                str(item)
                for item in sorted(by_id.iterdir())
                if DEVICE_HINT in item.name
            )
        candidates.extend(port.device for port in list_ports.comports())
        return list(dict.fromkeys(candidates))

    def _open_controller_port(self, timeout: float = 18.0) -> tuple[serial.Serial, str]:
        deadline = time.monotonic() + timeout
        last_error = "USB serial device did not appear"
        while time.monotonic() < deadline:
            for port in self._port_candidates(self.preferred_port):
                try:
                    device = serial.Serial(port, BAUD_RATE, timeout=0.08)
                    time.sleep(0.35)
                    device.reset_input_buffer()
                    self.preferred_port = port
                    return device, port
                except (serial.SerialException, OSError) as exc:
                    last_error = str(exc)
            time.sleep(0.25)
        raise RuntimeError(last_error)

    def _write_line(self, command: str) -> None:
        with self.serial_lock:
            if not self.device or not self.device.is_open:
                raise RuntimeError("controller serial port is not open")
            self.device.write((command + "\n").encode("ascii"))
            self.device.flush()

    def _calibration_command(self, timeout: float = 240.0) -> list[str]:
        if self.abort_event.is_set():
            raise CalibrationAborted("calibration cancelled")
        self.events.put(("log", "> M56"))
        self._write_line("M56")
        lines: list[str] = []
        records = 0
        safe_seen = False
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert self.device is not None
            payload = self.device.readline()
            if not payload:
                continue
            line = payload.decode(errors="replace").rstrip("\r\n")
            if not line:
                continue
            lines.append(line)
            if line.startswith("MAGDATA,"):
                records += 1
                if records % 100 == 0:
                    self.events.put(("log", f"Onboard calibration: {records} samples"))
            else:
                self.events.put(("log", line))
            progress = re.search(r"CALIBRATION progress axis (\d)/3", line)
            if progress:
                axis_number = int(progress.group(1))
                self._emit_status(
                    f"Onboard calibration: axis {axis_number} of 3…",
                    10 + (axis_number - 1) * 25,
                )
            if line.startswith("error:"):
                raise RuntimeError(line)
            if line == "SAFE: STBY is LOW":
                safe_seen = True
                if self.abort_event.is_set():
                    raise CalibrationAborted(
                        "calibration cancelled; motor drivers disabled"
                    )
            if line == "ok" and safe_seen:
                return lines
        self.abort()
        raise RuntimeError("onboard M56 calibration timed out; abort sent")

    @staticmethod
    def _parse_axis_result(
        axis: int, lines: list[str], path: Path
    ) -> AxisCalibrationResult:
        text = "\n".join(lines)
        result = re.search(
            rf"ORIGINALCAL result axis {axis}: forward measured "
            r"(-?[0-9.]+) deg; return error ([0-9.]+) deg; "
            r"safe limit ([0-9.]+) deg",
            text,
        )
        if not result:
            raise RuntimeError(f"axis {axis} did not complete its onboard calibration")
        statuses = [
            int(line.rsplit(",", 1)[1])
            for line in lines
            if line.startswith(f"MAGDATA,{axis},")
        ]
        return AxisCalibrationResult(
            axis=axis,
            measured_span_deg=float(result.group(1)),
            safe_limit_deg=float(result.group(3)),
            far_stop_command_deg=83.0,
            return_error_deg=float(result.group(2)),
            samples=len(statuses),
            encoder_fault_samples=sum(status != 0 for status in statuses),
            log_path=str(path),
        )

    def _close_device(self) -> None:
        with self.serial_lock:
            if self.device:
                try:
                    self.device.close()
                except (serial.SerialException, OSError):
                    pass
                self.device = None

    def run(self) -> None:
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        run_dir = CALIBRATION_DIR / f"gui_run_{timestamp}"
        run_dir.mkdir(parents=True, exist_ok=True)
        port = self.preferred_port
        try:
            self._emit_status("Connecting to onboard calibration…", 3)
            device, port = self._open_controller_port()
            with self.serial_lock:
                self.device = device
            self.events.put(("log", f"Calibration serial connected: {port}"))

            self._emit_status("Starting built-in M56 calibration…", 8)
            lines = self._calibration_command()
            path = run_dir / "all_axes.log"
            path.write_text(
                "\n".join(
                    [
                        f"# GUI onboard calibration run {timestamp}",
                        "# command=M56 method=original-adapted range_deg=83 "
                        "samples_per_pass=1024 calibration_amplitude=0.60",
                        *lines,
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            self._emit_status("Validating onboard calibration results…", 86)
            results = [self._parse_axis_result(axis, lines, path) for axis in range(3)]
            for result in results:
                if result.measured_span_deg < MIN_EXPECTED_SPAN_DEG:
                    raise RuntimeError(
                        f"axis {result.axis} span {result.measured_span_deg:.3f} deg "
                        f"is below {MIN_EXPECTED_SPAN_DEG:.0f} deg"
                    )
                if result.return_error_deg > MAX_RETURN_ERROR_DEG:
                    raise RuntimeError(
                        f"axis {result.axis} return error "
                        f"{result.return_error_deg:.3f} deg exceeds "
                        f"{MAX_RETURN_ERROR_DEG:.1f} deg"
                    )
                if result.encoder_fault_samples:
                    raise RuntimeError(
                        f"axis {result.axis} recorded "
                        f"{result.encoder_fault_samples} encoder faults"
                    )

            self._close_device()
            plot_path: Path | None = run_dir / "calibration_plot.png"
            try:
                self._run_process(
                    [
                        sys.executable,
                        str(CALIBRATION_PLOTTER),
                        str(path),
                        "--output",
                        str(plot_path),
                    ],
                    "Generate calibration plot",
                )
            except Exception as exc:
                self.events.put(("log", f"Plot warning: {exc}"))
                plot_path = None

            payload = {
                "results": [asdict(item) for item in results],
                "run_directory": str(run_dir),
                "plot": str(plot_path) if plot_path is not None else "",
                "port": port,
            }
            self._emit_status("Calibration saved in STM32 internal flash", 100)
            self.events.put(("calibration_finished", True, payload))
        except Exception as exc:
            self._close_device()
            self.events.put(
                (
                    "calibration_finished",
                    False,
                    {"error": str(exc), "port": port},
                )
            )
