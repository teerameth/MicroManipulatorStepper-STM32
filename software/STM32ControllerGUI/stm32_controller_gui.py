#!/usr/bin/env python3
"""Small Tk GUI for the calibrated STM32F401 micromanipulator controller."""

from __future__ import annotations

import queue
import math
import re
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import serial
from serial.tools import list_ports
import tkinter as tk
from tkinter import messagebox, ttk

try:
    from .calibration_workflow import CalibrationWorker
except ImportError:
    from calibration_workflow import CalibrationWorker


BAUD_RATE = 921600
DEFAULT_DEVICE_HINT = "F401RE_25MHZ_USB_CDC"
DEFAULT_DEVICE_SERIAL = "20883074534E"


@dataclass
class CommandRequest:
    command: str
    timeout: float
    quiet: bool
    callback: Callable[[str, list[str]], None] | None = None


class SerialWorker:
    """Own the serial port and execute commands sequentially off the Tk thread."""

    def __init__(self, events: queue.Queue):
        self.events = events
        self.requests: queue.Queue[CommandRequest | None] = queue.Queue()
        self.device: serial.Serial | None = None
        self.thread: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.emergency_event = threading.Event()
        self.state_lock = threading.Lock()
        self.active_command = ""
        self.pending = 0

    @property
    def connected(self) -> bool:
        return self.device is not None and self.device.is_open

    @property
    def idle(self) -> bool:
        with self.state_lock:
            return self.connected and self.pending == 0 and not self.active_command

    def connect(self, port: str) -> None:
        self.disconnect()
        # Do not carry a disconnect sentinel or abandoned requests into a new
        # connection. Only the Tk thread calls connect/disconnect.
        self.requests = queue.Queue()
        self.device = serial.Serial(port, BAUD_RATE, timeout=0.08)
        time.sleep(0.2)
        self.device.reset_input_buffer()
        self.stop_event.clear()
        self.emergency_event.clear()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        self.events.put(("connection", True, port))

    def disconnect(self) -> None:
        self.stop_event.set()
        if self.thread and self.thread.is_alive():
            self.requests.put(None)
            self.thread.join(timeout=1.0)
        if self.device:
            try:
                self.device.close()
            except serial.SerialException:
                pass
        self.device = None
        self.thread = None
        with self.state_lock:
            self.active_command = ""
            self.pending = 0

    def request(
        self,
        command: str,
        timeout: float = 5.0,
        quiet: bool = False,
        callback: Callable[[str, list[str]], None] | None = None,
    ) -> bool:
        if not self.connected:
            return False
        with self.state_lock:
            self.pending += 1
        self.requests.put(CommandRequest(command.strip(), timeout, quiet, callback))
        return True

    def emergency_disable(self) -> None:
        # The worker checks this every serial timeout. During G28, one byte invokes
        # the firmware's immediate homing abort. At idle it executes M18 normally.
        self.emergency_event.set()

    def cancel_queued(self) -> None:
        while True:
            try:
                request = self.requests.get_nowait()
            except queue.Empty:
                break
            if request is not None:
                with self.state_lock:
                    self.pending = max(0, self.pending - 1)
                self.events.put(("result", request, "cancelled", []))

    def _set_active(self, command: str) -> None:
        with self.state_lock:
            self.active_command = command

    def _finish_request(self) -> None:
        with self.state_lock:
            self.active_command = ""
            self.pending = max(0, self.pending - 1)

    def _write(self, payload: bytes) -> None:
        assert self.device is not None
        self.device.write(payload)
        self.device.flush()

    def _service_emergency(self, active_command: str = "") -> None:
        if not self.emergency_event.is_set() or not self.connected:
            return
        if active_command.upper().startswith("G28"):
            self.emergency_event.clear()
            # G28 treats any received character as an immediate driver-disable request.
            self._write(b"x")
            self.events.put(("log", "!!! HOMING ABORT SENT; DRIVERS WILL DISABLE !!!"))
        elif active_command:
            # Let a short request finish so its response cannot be confused with
            # M18's response. The loop services this event before the next request.
            return
        else:
            self.emergency_event.clear()
            # Balance _execute's request accounting: this priority M18 did not
            # arrive through request(), but must run before anything already queued.
            with self.state_lock:
                self.pending += 1
            self.events.put(("log", "!!! EMERGENCY M18 SENT !!!"))
            self._execute(CommandRequest("M18", 3.0, False, None))

    def _execute(self, request: CommandRequest) -> None:
        if not self.connected:
            self.events.put(("result", request, "error", ["serial disconnected"]))
            return
        self._set_active(request.command)
        if not request.quiet:
            self.events.put(("log", f"> {request.command}"))
        try:
            self._write((request.command + "\n").encode("ascii"))
            deadline = time.monotonic() + request.timeout
            response: list[str] = []
            status = "timeout"
            while time.monotonic() < deadline and not self.stop_event.is_set():
                self._service_emergency(request.command)
                assert self.device is not None
                payload = self.device.readline()
                if not payload:
                    continue
                line = payload.decode("utf-8", errors="replace").rstrip("\r\n")
                if not line:
                    continue
                if not request.quiet or line.startswith(("E)", "error:")):
                    self.events.put(("log", line))
                if line == "ok":
                    status = "ok"
                    break
                if line == "busy":
                    status = "busy"
                    break
                if line.startswith("error:"):
                    response.append(line)
                    status = "error"
                    break
                response.append(line)
            self.events.put(("result", request, status, response))
        except (serial.SerialException, OSError) as exc:
            self.events.put(("connection", False, str(exc)))
            self.events.put(("result", request, "error", [str(exc)]))
        finally:
            self._finish_request()

    def _run(self) -> None:
        while not self.stop_event.is_set():
            try:
                self._service_emergency()
            except (serial.SerialException, OSError) as exc:
                self.events.put(("connection", False, str(exc)))
                return
            try:
                item = self.requests.get(timeout=0.08)
            except queue.Empty:
                try:
                    self._service_emergency()
                    if self.connected and self.device and self.device.in_waiting:
                        line = self.device.readline().decode(errors="replace").strip()
                        if line:
                            self.events.put(("log", line))
                except (serial.SerialException, OSError) as exc:
                    self.events.put(("connection", False, str(exc)))
                    return
                continue
            if item is None:
                continue
            self._execute(item)


def available_ports() -> list[str]:
    ports: list[str] = []
    by_id = Path("/dev/serial/by-id")
    if by_id.is_dir():
        ports.extend(str(path) for path in sorted(by_id.iterdir()))
    ports.extend(port.device for port in list_ports.comports())
    return list(dict.fromkeys(ports))


def parse_pose(lines: list[str]) -> tuple[float, float, float] | None:
    match = re.search(
        r"X([-+0-9.eE]+)\s+Y([-+0-9.eE]+)\s+Z([-+0-9.eE]+)", "\n".join(lines)
    )
    return tuple(map(float, match.groups())) if match else None


def parse_controller_state(lines: list[str]) -> dict:
    state: dict = {"joints": {}}
    for line in lines:
        match = re.search(r"drivers=(\d+), fault=(\d+)", line)
        if match:
            state["drivers"] = bool(int(match.group(1)))
            state["fault"] = bool(int(match.group(2)))
        match = re.match(
            r"Joint (\d+): is_homed=(\d+), is_calibrated=(\d+), "
            r"angle=([-+0-9.eE]+) deg, target=([-+0-9.eE]+) deg, "
            r"(?:error=([-+0-9.eE]+) deg, phase=([-+0-9.eE]+) rad, )?"
            r"(?:drive=([-+0-9.eE]+), )?"
            r"limit=([-+0-9.eE]+) deg, raw_delta=(-?\d+), "
            r"direction=([-+0-9.eE]+), enc_status=(\d+)",
            line,
        )
        if match:
            axis = int(match.group(1))
            state["joints"][axis] = {
                "homed": bool(int(match.group(2))),
                "calibrated": bool(int(match.group(3))),
                "angle": float(match.group(4)),
                "target": float(match.group(5)),
                "error": (
                    float(match.group(6))
                    if match.group(6) is not None
                    else float(match.group(5)) - float(match.group(4))
                ),
                "phase": float(match.group(7)) if match.group(7) is not None else None,
                "drive": float(match.group(8)) if match.group(8) is not None else None,
                "limit": float(match.group(9)),
                "raw_delta": int(match.group(10)),
                "direction": float(match.group(11)),
                "encoder_status": int(match.group(12)),
            }
        match = re.match(r"Tool\[(\d+)\] output: ([-+0-9.eE]+)", line)
        if match:
            state.setdefault("tools", {})[int(match.group(1))] = float(match.group(2))
    return state


class ControllerGui(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("STM32 Micromanipulator Controller")
        self.minsize(920, 680)
        self.events: queue.Queue = queue.Queue()
        self.worker = SerialWorker(self.events)
        self.pose: tuple[float, float, float] | None = None
        self.target: tuple[float, float, float] | None = None
        self.target_revision = 0
        self.target_timer = None
        self.target_inflight = False
        self.motion_active = False
        self.bookmark = None
        self.driver_active = False
        self.monitoring = True
        self.poll_in_progress = False
        self.calibration_worker: CalibrationWorker | None = None
        self.close_after_calibration = False
        self._build_ui()
        self.refresh_ports()
        self.after(50, self.process_events)
        self.after(700, self.poll_state)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(5, weight=1)

        connection = ttk.LabelFrame(self, text="Connection", padding=8)
        connection.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 5))
        connection.columnconfigure(1, weight=1)
        ttk.Label(connection, text="Port").grid(row=0, column=0, padx=(0, 6))
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(connection, textvariable=self.port_var)
        self.port_box.grid(row=0, column=1, sticky="ew")
        ttk.Button(connection, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=5)
        self.connect_button = ttk.Button(connection, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=3, padx=5)
        self.connection_var = tk.StringVar(value="Disconnected")
        ttk.Label(connection, textvariable=self.connection_var, width=20).grid(row=0, column=4)

        safety = ttk.LabelFrame(self, text="Safety and homing", padding=8)
        safety.grid(row=1, column=0, sticky="ew", padx=10, pady=5)
        safety_controls = ttk.Frame(safety)
        safety_controls.pack(fill="x")
        ttk.Button(safety_controls, text="Home all (G28)", command=self.home_all).pack(side="left", padx=4)
        ttk.Button(safety_controls, text="Enable hold (M17)", command=lambda: self.send("M17")).pack(side="left", padx=4)
        ttk.Button(safety_controls, text="Disable motors (M18)", command=self.emergency_disable).pack(side="left", padx=4)
        self.calibration_button = ttk.Button(
            safety_controls,
            text="Onboard calibrate (M56)",
            command=self.start_calibration,
        )
        self.calibration_button.pack(side="left", padx=(16, 4))
        tk.Button(
            safety_controls,
            text="EMERGENCY DISABLE",
            bg="#b00020",
            fg="white",
            activebackground="#d32f2f",
            command=self.emergency_disable,
            padx=14,
        ).pack(side="right", padx=4)
        calibration_status = ttk.Frame(safety)
        calibration_status.pack(fill="x", pady=(7, 0))
        self.calibration_var = tk.StringVar(value="Calibration: ready")
        ttk.Label(calibration_status, textvariable=self.calibration_var, width=52).pack(side="left", padx=4)
        self.calibration_progress = ttk.Progressbar(
            calibration_status, mode="determinate", maximum=100
        )
        self.calibration_progress.pack(side="left", fill="x", expand=True, padx=4)

        state_frame = ttk.LabelFrame(self, text="Live state", padding=8)
        state_frame.grid(row=2, column=0, sticky="ew", padx=10, pady=5)
        self.summary_var = tk.StringVar(value="Drivers: —    Fault: —    Motion: —")
        ttk.Label(state_frame, textvariable=self.summary_var).pack(anchor="w")
        self.pose_var = tk.StringVar(value="Pose: X —   Y —   Z — mm")
        ttk.Label(state_frame, textvariable=self.pose_var, font=("TkDefaultFont", 11, "bold")).pack(anchor="w", pady=(3, 5))
        columns = ("home", "cal", "angle", "target", "error", "limit", "status")
        self.joint_table = ttk.Treeview(state_frame, columns=columns, show="headings", height=3)
        widths = {"home": 65, "cal": 70, "angle": 100, "target": 100, "error": 100, "limit": 100, "status": 90}
        labels = {"home": "Homed", "cal": "Calibrated", "angle": "Angle °", "target": "Target °", "error": "Error °", "limit": "Limit °", "status": "Encoder"}
        for column in columns:
            self.joint_table.heading(column, text=labels[column])
            self.joint_table.column(column, width=widths[column], anchor="center")
        for axis in range(3):
            self.joint_table.insert("", "end", iid=str(axis), values=("—", "—", "—", "—", "—", "—", "—"))
        self.joint_table.pack(fill="x")

        motion = ttk.LabelFrame(self, text="Cartesian motion", padding=8)
        motion.grid(row=3, column=0, sticky="ew", padx=10, pady=5)
        for column in range(9):
            motion.columnconfigure(column, weight=1 if column in (1, 3, 5) else 0)
        self.xyz_vars = [tk.StringVar(value="0.000") for _ in range(3)]
        for index, name in enumerate("XYZ"):
            ttk.Label(motion, text=name).grid(row=0, column=index * 2, padx=(4, 2))
            entry = ttk.Entry(motion, textvariable=self.xyz_vars[index], width=12)
            entry.grid(row=0, column=index * 2 + 1, sticky="ew", padx=(0, 8))
            entry.bind("<Return>", lambda event: self.move_absolute())
        ttk.Label(motion, text="Feed mm/s").grid(row=0, column=6, padx=(4, 2))
        self.feed_var = tk.StringVar(value="1.0")
        speed_box = ttk.Combobox(motion, textvariable=self.feed_var, values=("0.05", "0.2", "0.5", "1.0", "2.0"), width=9)
        speed_box.grid(row=0, column=7, padx=(0, 8))
        speed_box.bind("<<ComboboxSelected>>", self.change_speed)
        speed_box.bind("<Return>", self.change_speed)
        ttk.Button(motion, text="Set destination", command=self.move_absolute).grid(row=0, column=8, padx=4)

        ttk.Label(motion, text="Jog step mm").grid(row=1, column=0, pady=(8, 0))
        self.jog_var = tk.StringVar(value="0.1")
        ttk.Combobox(motion, textvariable=self.jog_var, values=("0.001", "0.01", "0.1", "0.5", "1.0"), width=9).grid(row=1, column=1, pady=(8, 0), sticky="w")
        jog_buttons = ttk.Frame(motion)
        jog_buttons.grid(row=1, column=2, columnspan=7, sticky="w", pady=(8, 0))
        for axis, name in enumerate("XYZ"):
            ttk.Button(jog_buttons, text=f"{name}−", command=lambda a=axis: self.jog(a, -1)).pack(side="left", padx=2)
            ttk.Button(jog_buttons, text=f"{name}+", command=lambda a=axis: self.jog(a, 1)).pack(side="left", padx=2)
        ttk.Button(jog_buttons, text="Use current pose", command=self.copy_current_pose).pack(side="left", padx=(14, 2))
        ttk.Button(motion, text="Stop & hold (Esc)", command=self.stop_hold).grid(row=2, column=8, pady=6)
        self.destination_var = tk.StringVar(value="Destination: waiting for controller")
        ttk.Label(motion, textvariable=self.destination_var).grid(row=2, column=0, columnspan=8, sticky="w")
        ttk.Label(motion, text="Jogs update the destination during travel. Peak speed: up to 2 mm/s; Y limited to 1 mm/s.").grid(row=3, column=0, columnspan=9, sticky="w")
        self.bind("<Escape>", lambda event: self.stop_hold())
        shortcuts = ttk.Frame(motion)
        shortcuts.grid(row=4, column=0, columnspan=9, sticky="ew", pady=(6, 0))
        ttk.Button(shortcuts, text="Mark this position", command=self.mark_position).pack(side="left", padx=3)
        ttk.Button(shortcuts, text="Return to mark", command=self.return_to_mark).pack(side="left", padx=3)
        self.keyboard_jog = tk.BooleanVar(value=False)
        ttk.Checkbutton(shortcuts, text="Keyboard jog: arrows = XY, PgUp/PgDn = Z", variable=self.keyboard_jog).pack(side="left", padx=12)
        for key, axis, direction in (("Left",0,-1),("Right",0,1),("Down",1,-1),("Up",1,1),("Prior",2,1),("Next",2,-1)):
            self.bind(f"<{key}>", lambda event, a=axis, d=direction: self.key_jog(event, a, d))

        tools = ttk.LabelFrame(self, text="Tools and raw command", padding=8)
        tools.grid(row=4, column=0, sticky="ew", padx=10, pady=5)
        tools.columnconfigure(7, weight=1)
        self.tool_vars = [tk.DoubleVar(value=0.0), tk.DoubleVar(value=0.0)]
        for index in range(2):
            ttk.Label(tools, text=f"Tool {index}").grid(row=0, column=index * 3, padx=(2, 4))
            ttk.Scale(tools, variable=self.tool_vars[index], from_=0.0, to=1.0, length=120).grid(row=0, column=index * 3 + 1)
            ttk.Button(tools, text="Apply", command=lambda i=index: self.set_tool(i)).grid(row=0, column=index * 3 + 2, padx=(4, 12))
        self.raw_var = tk.StringVar(value="M57")
        ttk.Entry(tools, textvariable=self.raw_var).grid(row=0, column=7, sticky="ew", padx=(8, 4))
        ttk.Button(tools, text="Send", command=self.send_raw).grid(row=0, column=8)

        log_frame = ttk.LabelFrame(self, text="Controller log", padding=6)
        log_frame.grid(row=5, column=0, sticky="nsew", padx=10, pady=(5, 10))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self.log = tk.Text(log_frame, height=14, wrap="word", state="disabled", font=("TkFixedFont", 9))
        scroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.log.yview)
        self.log.configure(yscrollcommand=scroll.set)
        self.log.grid(row=0, column=0, sticky="nsew")
        scroll.grid(row=0, column=1, sticky="ns")
        ttk.Button(log_frame, text="Clear log", command=self.clear_log).grid(row=1, column=0, sticky="e", pady=(5, 0))

    def append_log(self, text: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def clear_log(self) -> None:
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")

    def refresh_ports(self) -> None:
        ports = available_ports()
        self.port_box["values"] = ports
        if not self.port_var.get() or self.port_var.get() not in ports:
            preferred = next((p.device for p in list_ports.comports()
                              if p.serial_number == DEFAULT_DEVICE_SERIAL), None)
            preferred = preferred or next((port for port in ports if DEFAULT_DEVICE_HINT in port), None)
            if preferred or ports:
                self.port_var.set(preferred or ports[0])

    def toggle_connection(self) -> None:
        if self.worker.connected:
            self.cancel_target()
            self.worker.disconnect()
            self.bookmark = None
            self.target_inflight = False
            self.pose = self.target = None
            self.poll_in_progress = False
            self.connection_var.set("Disconnected")
            self.connect_button.configure(text="Connect")
            self.append_log("Disconnected; motor state on the controller was not changed.")
            return
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("No port", "Select a serial port first.")
            return
        try:
            self.worker.connect(port)
        except (serial.SerialException, OSError) as exc:
            messagebox.showerror("Connection failed", str(exc))

    def send(
        self,
        command: str,
        timeout: float = 5.0,
        quiet: bool = False,
        callback: Callable[[str, list[str]], None] | None = None,
    ) -> None:
        if not self.worker.request(command, timeout, quiet, callback):
            messagebox.showerror("Not connected", "Connect to the STM32 controller first.")

    def home_all(self) -> None:
        if not messagebox.askokcancel(
            "Home all axes",
            "All three motors will move toward their mechanical home together, then retract to 42°.\n\nKeep the mechanism in view and use EMERGENCY DISABLE if motion is abnormal.",
        ):
            return
        self.cancel_target()
        self.worker.cancel_queued()
        self.poll_in_progress = False
        self.driver_active = False
        self.bookmark = None
        self.send("G28", timeout=120.0)

    @property
    def calibration_active(self) -> bool:
        return self.calibration_worker is not None and self.calibration_worker.is_alive()

    def _set_normal_controls_enabled(self, enabled: bool) -> None:
        state = "normal" if enabled else "disabled"

        def visit(widget: tk.Misc) -> None:
            for child in widget.winfo_children():
                if isinstance(child, ttk.Button):
                    child.configure(state=state)
                visit(child)

        visit(self)

    def start_calibration(self) -> None:
        if self.calibration_active:
            return
        message = (
            "NO FIRMWARE FLASHING WILL OCCUR.\n\n"
            "The GUI will only send M56 to the firmware already running on the STM32. "
            "The STM32 then "
            "calibrates one axis at a time using the original firmware's method: negative "
            "home, 1.8° backoff, then a fixed 83° forward/reverse sweep with 1024 samples "
            "per pass. Homing uses 0.22 PWM amplitude and the measurement sweep uses the "
            "original 0.60 amplitude. Valid lookup tables are saved directly in STM32 "
            "internal flash and survive reset or power loss.\n\n"
            "The sequence can take several minutes. Keep the mechanism in view and use "
            "EMERGENCY DISABLE immediately if any axis behaves abnormally.\n\n"
            "Send M56 and start onboard calibration now?"
        )
        if not messagebox.askokcancel("Run onboard M56 calibration", message):
            return

        preferred_port = self.port_var.get().strip()
        self.cancel_target()
        self.bookmark = None
        self.target_inflight = False
        self.worker.disconnect()
        self.poll_in_progress = False
        self.connection_var.set("Onboard M56 running")
        self.connect_button.configure(text="Connect")
        self.calibration_var.set("Calibration: sending M56 (no flashing)…")
        self.calibration_progress["value"] = 0
        self._set_normal_controls_enabled(False)
        self.calibration_worker = CalibrationWorker(self.events, preferred_port)
        self.calibration_worker.start()

    def show_calibration_results(self, payload: dict) -> None:
        window = tk.Toplevel(self)
        window.title("Calibration results")
        window.transient(self)
        window.grab_set()
        frame = ttk.Frame(window, padding=12)
        frame.pack(fill="both", expand=True)
        ttk.Label(
            frame,
            text="Calibration completed and was saved in STM32 internal flash.",
            font=("TkDefaultFont", 11, "bold"),
        ).pack(anchor="w", pady=(0, 8))
        columns = ("axis", "span", "limit", "stop", "return", "samples", "faults")
        table = ttk.Treeview(frame, columns=columns, show="headings", height=3)
        labels = {
            "axis": "Axis",
            "span": "Measured span °",
            "limit": "Safe limit °",
            "stop": "Sweep command °",
            "return": "Return error °",
            "samples": "Samples",
            "faults": "Encoder faults",
        }
        for column in columns:
            table.heading(column, text=labels[column])
            table.column(column, width=115, anchor="center")
        for result in payload["results"]:
            table.insert(
                "",
                "end",
                values=(
                    result["axis"],
                    f'{result["measured_span_deg"]:.3f}',
                    f'{result["safe_limit_deg"]:.3f}',
                    f'{result["far_stop_command_deg"]:.2f}',
                    f'{result["return_error_deg"]:.3f}',
                    result["samples"],
                    result["encoder_fault_samples"],
                ),
            )
        table.pack(fill="x")
        ttk.Label(frame, text=f'Raw logs: {payload["run_directory"]}').pack(
            anchor="w", pady=(9, 0)
        )
        if payload.get("plot"):
            ttk.Label(frame, text=f'Plot: {payload["plot"]}').pack(anchor="w")
        ttk.Button(frame, text="Close", command=window.destroy).pack(anchor="e", pady=(10, 0))

    def emergency_disable(self) -> None:
        self.cancel_target()
        if self.calibration_active:
            self.append_log("Emergency calibration abort requested…")
            assert self.calibration_worker is not None
            self.calibration_worker.abort()
            return
        if not self.worker.connected:
            return
        self.append_log("Emergency disable requested…")
        self.worker.cancel_queued()
        self.worker.emergency_disable()

    def move_absolute(self) -> None:
        try:
            xyz = [float(item.get()) for item in self.xyz_vars]
            feed = float(self.feed_var.get())
        except ValueError:
            messagebox.showerror("Invalid motion", "X, Y, Z, and feed must be numbers.")
            return
        if not all(math.isfinite(v) for v in (*xyz, feed)) or not 0 < feed <= 2:
            messagebox.showerror("Invalid motion", "Use finite coordinates and speed greater than 0 and at most 2 mm/s.")
            return
        self.schedule_target(tuple(xyz))

    def jog(self, axis: int, direction: int) -> None:
        if self.pose is None:
            messagebox.showerror("No pose", "Wait for the live pose display before jogging.")
            return
        try:
            step = abs(float(self.jog_var.get()))
            feed = float(self.feed_var.get())
        except ValueError:
            messagebox.showerror("Invalid jog", "Jog step and feed must be numbers.")
            return
        if not math.isfinite(step) or step <= 0 or not math.isfinite(feed) or not 0 < feed <= 2:
            messagebox.showerror("Invalid jog", "Use a positive finite step and speed up to 2 mm/s.")
            return
        target = list(self.target or self.pose)
        target[axis] += direction * step
        self.schedule_target(tuple(target))

    def cancel_target(self) -> None:
        self.target_revision += 1
        if self.target_timer is not None:
            self.after_cancel(self.target_timer)
            self.target_timer = None
        self.target = None

    def key_jog(self, event, axis, direction):
        if self.keyboard_jog.get() and not isinstance(event.widget, (tk.Entry, ttk.Entry, ttk.Combobox, tk.Text)):
            self.jog(axis, direction)
            return "break"

    def mark_position(self) -> None:
        if self.pose is not None and self.driver_active:
            self.bookmark = self.pose
            self.append_log("Marked sample position: " + str(self.bookmark))

    def change_speed(self, event=None) -> None:
        if self.motion_active and self.target is not None:
            self.schedule_target(self.target)

    def return_to_mark(self) -> None:
        if self.bookmark is not None:
            self.schedule_target(self.bookmark)

    def stop_hold(self) -> None:
        self.cancel_target()
        if self.calibration_active or self.worker.active_command.startswith("G28"):
            self.emergency_disable()
        elif self.worker.connected:
            self.worker.cancel_queued()
            self.poll_in_progress = False
            self.send("M0")
            self.destination_var.set("Travel cancelled; holding position")

    def schedule_target(self, target: tuple[float, float, float]) -> None:
        if self.calibration_active or not self.worker.connected or not self.driver_active:
            messagebox.showerror("Stage not enabled", "Connect and enable the stage before moving.")
            return
        self.target = target
        self.target_revision += 1
        self.destination_var.set("Destination: " + "   ".join(f"{a} {v:+.4f}" for a, v in zip("XYZ", target)) + " mm")
        if self.target_timer is not None:
            self.after_cancel(self.target_timer)
        self.target_timer = self.after(60, self.flush_target)

    def flush_target(self) -> None:
        self.target_timer = None
        if self.target is None or not self.worker.connected:
            return
        if self.target_inflight:
            self.target_timer = self.after(40, self.flush_target)
            return
        revision = self.target_revision
        x, y, z = self.target
        try:
            feed = float(self.feed_var.get())
            if not math.isfinite(feed) or not 0 < feed <= 2:
                raise ValueError()
        except ValueError:
            self.cancel_target()
            self.destination_var.set("Invalid speed; target not sent")
            return
        self.target_inflight = True
        def finished(status, lines):
            self.target_inflight = False
            if status != "ok" and revision == self.target_revision:
                self.cancel_target()
                self.destination_var.set("Target rejected: " + ("; ".join(lines) or status))
        self.send(f"G0 X{x:.6f} Y{y:.6f} Z{z:.6f} F{feed:.4f}", callback=finished)

    def copy_current_pose(self) -> None:
        if self.pose is None:
            return
        for variable, value in zip(self.xyz_vars, self.pose):
            variable.set(f"{value:.6f}")

    def set_tool(self, index: int) -> None:
        self.send(f"M3 T{index} S{self.tool_vars[index].get():.4f}")

    def send_raw(self) -> None:
        command = self.raw_var.get().strip()
        if command:
            self.send(command, timeout=120.0 if command.upper().startswith("G28") else 5.0)

    def process_events(self) -> None:
        try:
            while True:
                event = self.events.get_nowait()
                if event[0] == "log":
                    self.append_log(event[1])
                elif event[0] == "connection":
                    connected, detail = event[1], event[2]
                    self.connection_var.set("Connected" if connected else "Disconnected")
                    self.connect_button.configure(text="Disconnect" if connected else "Connect")
                    self.append_log(f"{'Connected to' if connected else 'Connection lost:'} {detail}")
                elif event[0] == "result":
                    request, status, lines = event[1], event[2], event[3]
                    if request.callback:
                        request.callback(status, lines)
                    if not request.quiet and status == "timeout":
                        self.append_log(f"error: {request.command} timed out")
                elif event[0] == "calibration_status":
                    self.calibration_var.set(f"Calibration: {event[1]}")
                    self.calibration_progress["value"] = event[2]
                elif event[0] == "calibration_finished":
                    successful, detail = event[1], event[2]
                    self.calibration_worker = None
                    self._set_normal_controls_enabled(True)
                    reconnect_port = detail.get("port", "")
                    self.refresh_ports()
                    if reconnect_port and not self.close_after_calibration:
                        try:
                            self.worker.connect(reconnect_port)
                        except (serial.SerialException, OSError) as exc:
                            self.append_log(f"Controller reconnect failed: {exc}")
                    if successful:
                        limits = ", ".join(
                            f'{item["safe_limit_deg"]:.2f}°' for item in detail["results"]
                        )
                        self.calibration_var.set(f"Calibration complete — limits: {limits}")
                        self.calibration_progress["value"] = 100
                        if not self.close_after_calibration:
                            self.show_calibration_results(detail)
                    else:
                        error = detail["error"]
                        self.calibration_var.set(f"Calibration failed: {error}")
                        self.calibration_progress["value"] = 0
                        if not self.close_after_calibration:
                            messagebox.showerror("Calibration failed", error)
                    if self.close_after_calibration:
                        self.worker.disconnect()
                        self.destroy()
                        return
        except queue.Empty:
            pass
        self.after(50, self.process_events)

    def poll_state(self) -> None:
        if not self.calibration_active and self.worker.idle and not self.poll_in_progress:
            self.poll_in_progress = True
            self.send("M57", quiet=True, callback=self._state_received)
        self.after(200, self.poll_state)

    def _state_received(self, status: str, lines: list[str]) -> None:
        if status == "ok":
            state = parse_controller_state(lines)
            self.driver_active = state.get("drivers", False)
            fault = state.get("fault", False)
            if fault or not self.driver_active:
                self.cancel_target()
            for axis, item in state.get("joints", {}).items():
                self.joint_table.item(
                    str(axis),
                    values=(
                        "yes" if item["homed"] else "no",
                        "yes" if item["calibrated"] else "no",
                        f"{item['angle']:.4f}",
                        f"{item['target']:.4f}",
                        f"{item['error']:+.4f}",
                        f"{item['limit']:.3f}",
                        f"0x{item['encoder_status']:02X}",
                    ),
                )
            self.summary_var.set(
                f"Drivers: {'ON' if self.driver_active else 'OFF'}    "
                f"Fault: {'YES' if fault else 'no'}    Motion: checking…"
            )
        if self.worker.connected:
            self.send("M50", quiet=True, callback=self._pose_received)
        else:
            self.poll_in_progress = False

    def _pose_received(self, status: str, lines: list[str]) -> None:
        if status == "ok":
            pose = parse_pose(lines)
            if pose:
                self.pose = pose
                if self.target_timer is None and not self.target_inflight:
                    destination = next((line for line in lines if line.startswith("Destination:")), None)
                    if destination:
                        self.target = parse_pose([destination])
                        self.destination_var.set(destination + " mm")
                self.pose_var.set(
                    f"Pose: X {pose[0]:+.6f}   Y {pose[1]:+.6f}   Z {pose[2]:+.6f} mm"
                )
                if all(variable.get() == "0.000" for variable in self.xyz_vars):
                    self.copy_current_pose()
        if self.worker.connected:
            self.send("M53", quiet=True, callback=self._motion_received)
        else:
            self.poll_in_progress = False

    def _motion_received(self, status: str, lines: list[str]) -> None:
        moving = status == "ok" and bool(lines) and lines[0].strip() == "0"
        self.motion_active = moving
        text = self.summary_var.get().replace("Motion: checking…", f"Motion: {'MOVING' if moving else 'idle'}")
        if "Motion:" in text and "checking…" not in text:
            text = re.sub(r"Motion: (?:MOVING|idle)", f"Motion: {'MOVING' if moving else 'idle'}", text)
        self.summary_var.set(text)
        self.poll_in_progress = False

    def on_close(self) -> None:
        if self.calibration_active:
            if not messagebox.askokcancel(
                "Abort calibration",
                "Abort motor movement and close after the controller disables its drivers?",
            ):
                return
            self.close_after_calibration = True
            assert self.calibration_worker is not None
            self.calibration_worker.abort()
            self.calibration_var.set("Calibration: aborting motor movement…")
            return
        self.worker.disconnect()
        self.destroy()


def main() -> None:
    app = ControllerGui()
    app.mainloop()


if __name__ == "__main__":
    main()
