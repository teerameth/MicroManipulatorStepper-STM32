#!/usr/bin/env python3
"""
Real-time encoder monitor for the Open Micro-Manipulator.

Pairs with the 'pico_encmon' firmware, which never energizes a motor - it holds the
shared TB6612 enable line low so every rotor turns freely by hand. Turn each axis and
watch whether its trace follows smoothly and its health stays green.

    python3 encoder_monitor_gui.py                  # auto-detect the port
    python3 encoder_monitor_gui.py --port /dev/ttyACM0
    python3 encoder_monitor_gui.py --headless 10    # 10 s text report, no window
    python3 encoder_monitor_gui.py --snapshot o.png # run briefly, save the plot

License: MIT (see LICENSE file for full description)
"""

import argparse
import collections
import sys
import threading
import time

import serial
import serial.tools.list_ports

# --- how each health counter is judged ------------------------------------------------
INVALID_FRAC_FAIL = 0.001   # all three status bits set at once = floating SPI bus
WEAK_FRAC_WARN = 0.01
CRC_FRAC_WARN = 0.001
FROZEN_WARN = 200           # consecutive identical reads at ~37 kHz
NOISE_WARN_COUNTS = 25.0    # standstill stddev

PLOT_SECONDS = 20.0
AXIS_COLORS = ["#0072B2", "#D55E00", "#009E73"]


# --------------------------------------------------------------------------------------
# serial reader
# --------------------------------------------------------------------------------------

class Health:
    """Latest health window for one axis, as reported by the firmware."""

    def __init__(self):
        self.samples = 0
        self.weak = self.undervolt = self.overspeed = self.crc = 0
        self.max_jump = 0
        self.slips = 0
        self.frozen = 0
        self.span = 0
        self.sd = 0.0
        self.link = None      # None = not probed yet
        self.stamp = 0.0

    @property
    def invalid_frac(self):
        # A disconnected MT6835 leaves MISO floating, which reads back as all ones: every
        # status bit sets at once. Real faults set one bit, not three.
        if not self.samples:
            return 0.0
        return min(self.weak, self.undervolt, self.overspeed) / self.samples

    @property
    def weak_frac(self):
        return self.weak / self.samples if self.samples else 0.0

    @property
    def crc_frac(self):
        return self.crc / self.samples if self.samples else 0.0

    def verdict(self):
        """Returns (state, message) where state is one of OK / WARN / FAIL."""
        if self.samples == 0:
            return "WARN", "no data yet" if self.link is not False else "no SPI response"
        bad = self.invalid_frac
        if self.link is False and bad > INVALID_FRAC_FAIL:
            return "FAIL", f"SPI link dead or intermittent - {bad * 100:.1f}% of reads invalid"
        if self.link is False:
            # the one-shot register probe failed but the streamed reads are clean, so the
            # link is dropping in and out rather than being gone
            return "WARN", "register probe failed although streamed reads are clean - intermittent link"
        if bad > INVALID_FRAC_FAIL:
            return "FAIL", f"invalid reads {bad * 100:.1f}% - intermittent SPI link"
        if self.frozen > FROZEN_WARN:
            return "FAIL", f"output froze for {self.frozen} reads"
        if self.slips:
            return "WARN", f"{self.slips} period slips - turned too fast, or dropped reads"
        if self.weak_frac > WEAK_FRAC_WARN:
            return "WARN", f"weak magnetic field on {self.weak_frac * 100:.1f}% of reads"
        if self.crc_frac > CRC_FRAC_WARN:
            return "WARN", f"CRC errors on {self.crc_frac * 100:.2f}% of reads"
        if self.undervolt:
            return "WARN", f"undervolt on {self.undervolt} reads"
        if self.sd > NOISE_WARN_COUNTS:
            return "WARN", f"noisy: stddev {self.sd:.1f} counts"
        return "OK", f"stddev {self.sd:.1f} counts, span {self.span}"


def contention_note(health):
    """A chip that stops releasing MISO corrupts its neighbours on the shared bus, which
    shows up as two axes logging the exact same number of invalid reads."""
    bad = {i: h.weak for i, h in enumerate(health)
           if h.samples and h.invalid_frac > INVALID_FRAC_FAIL}
    for i in bad:
        for j in bad:
            if i < j and bad[i] == bad[j] != 0:
                return (f"axes {i} and {j} logged identical invalid-read counts "
                        f"({bad[i]}) - one faulty encoder is corrupting the shared SPI "
                        f"bus for the other")
    return None


class Reader(threading.Thread):
    """Consumes the firmware's line protocol on a background thread."""

    daemon = True

    def __init__(self, port, naxes=3):
        super().__init__()
        self.ser = serial.Serial(port, 921600, timeout=0.2)
        self.naxes = naxes
        self.lock = threading.Lock()
        self.t = collections.deque(maxlen=20000)
        self.raw = [collections.deque(maxlen=20000) for _ in range(naxes)]
        self.health = [Health() for _ in range(naxes)]
        self.raw_to_rotor_urad = 0.095367   # replaced by the #CFG line
        self.cfg = {}
        self.running = True
        self.t0 = None
        self.record_count = 0
        self.banner = []
        # transport integrity, kept separate from encoder health: a bad checksum means a
        # value arrived corrupted, a sequence gap only means a record went missing
        self.link_errors = 0
        self.seq_gaps = 0
        self.last_seq = None
        self.fw_dropped = 0
        self.fw_truncated = 0

    def send(self, cmd):
        try:
            self.ser.write((cmd + "\n").encode())
        except serial.SerialException:
            pass

    def stop(self):
        self.running = False

    def run(self):
        while self.running:
            try:
                line = self.ser.readline().decode(errors="replace").strip()
            except serial.SerialException:
                break
            if not line:
                continue
            try:
                self._parse(line)
            except (ValueError, IndexError):
                pass  # a partial line on connect is normal
        try:
            self.ser.close()
        except Exception:
            pass

    @staticmethod
    def _checksum_ok(line):
        body, star, cs = line.rpartition("*")
        if not star:
            return None, False
        x = 0
        for c in body.encode():
            x ^= c
        try:
            return body, x == int(cs, 16)
        except ValueError:
            return body, False

    def _parse(self, line):
        if line.startswith("#CFG,"):
            for kv in line[5:].split(","):
                if "=" in kv:
                    k, v = kv.split("=", 1)
                    self.cfg[k] = v
            if "raw_to_rotor_urad" in self.cfg:
                self.raw_to_rotor_urad = float(self.cfg["raw_to_rotor_urad"])
            return
        if line.startswith("#"):
            self.banner.append(line)
            return

        # Every data record carries an XOR checksum. Reject rather than plot a record
        # that was corrupted in transit - otherwise a single dropped byte draws a
        # position jump that looks exactly like an encoder fault.
        body, ok = self._checksum_ok(line)
        if body is None:
            return
        if not ok:
            with self.lock:
                self.link_errors += 1
            return

        f = body.split(",")
        if f[0] == "D" and len(f) >= 3 + self.naxes:
            seq, ms = int(f[1]), int(f[2])
            if self.t0 is None:
                self.t0 = ms
            with self.lock:
                if self.last_seq is not None and seq > self.last_seq + 1:
                    self.seq_gaps += seq - self.last_seq - 1
                self.last_seq = seq
                self.t.append((ms - self.t0) / 1000.0)
                for i in range(self.naxes):
                    self.raw[i].append(int(f[3 + i]))
                self.record_count += 1
        elif f[0] == "H" and len(f) >= 13:
            i = int(f[2])
            if 0 <= i < self.naxes:
                h = self.health[i]
                with self.lock:
                    (h.samples, h.weak, h.undervolt, h.overspeed, h.crc) = map(int, f[3:8])
                    h.max_jump = int(f[8])
                    h.slips = int(f[9])
                    h.frozen = int(f[10])
                    h.span = int(f[11])
                    h.sd = float(f[12]) / 100.0
                    if len(f) >= 15:
                        self.fw_dropped = int(f[13])
                        self.fw_truncated = int(f[14])
                    h.stamp = time.time()
        elif f[0] == "L" and len(f) >= 2 + self.naxes:
            with self.lock:
                for i in range(self.naxes):
                    self.health[i].link = f[2 + i] == "1"

    def snapshot(self):
        """Thread-safe copy of the plot data, converted to rotor degrees."""
        with self.lock:
            t = list(self.t)
            series = []
            for i in range(self.naxes):
                r = self.raw[i]
                base = r[0] if r else 0
                scale = self.raw_to_rotor_urad * 1e-6 * 57.29577951308232
                series.append([(v - base) * scale for v in r])
            health = []
            for h in self.health:
                c = Health()
                c.__dict__.update(h.__dict__)
                health.append(c)
        return t, series, health


# --------------------------------------------------------------------------------------
# port discovery
# --------------------------------------------------------------------------------------

def find_port(explicit=None):
    if explicit:
        return explicit
    candidates = []
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x2E8A:            # Raspberry Pi
            candidates.append(p.device)
        elif p.device.startswith("/dev/ttyACM"):
            candidates.append(p.device)
    if not candidates:
        sys.exit("No serial port found. Pass --port /dev/ttyACMx")
    return sorted(candidates)[0]


# --------------------------------------------------------------------------------------
# headless report
# --------------------------------------------------------------------------------------

def run_headless(reader, seconds):
    print(f"# collecting for {seconds:.0f} s - turn each rotor by hand now\n")
    deadline = time.time() + seconds
    while time.time() < deadline:
        time.sleep(0.5)
    t, series, health = reader.snapshot()
    rate = reader.record_count / seconds if seconds else 0.0

    print(f"position records : {reader.record_count} ({rate:.0f} Hz)")
    print(f"link integrity   : {reader.link_errors} corrupted records, "
          f"{reader.seq_gaps} missing (firmware dropped {reader.fw_dropped}, "
          f"truncated {reader.fw_truncated})")
    if reader.link_errors:
        print("                   corrupted records were REJECTED, not plotted")
    if reader.cfg:
        print(f"firmware config  : {reader.cfg}\n")
    worst = "OK"
    for i, h in enumerate(health):
        state, msg = h.verdict()
        if state == "FAIL" or (state == "WARN" and worst == "OK"):
            worst = state
        travel = (max(series[i]) - min(series[i])) if series[i] else 0.0
        print(f"axis {i}: [{state:4}] {msg}")
        print(f"        samples/window {h.samples}, travel seen {travel:.3f} deg, "
              f"max step {h.max_jump} counts, slips {h.slips}, frozen {h.frozen}")
        print(f"        weak {h.weak}  undervolt {h.undervolt}  overspeed {h.overspeed}  "
              f"crc {h.crc}  link {h.link}")
    note = contention_note(health)
    if note:
        print(f"\nNOTE: {note}")
    print(f"\noverall: {worst}")
    return 0 if worst != "FAIL" else 1


# --------------------------------------------------------------------------------------
# GUI
# --------------------------------------------------------------------------------------

def run_gui(reader, snapshot=None, snapshot_after=6.0):
    import matplotlib
    if snapshot:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec
    from matplotlib.widgets import Button
    from matplotlib.animation import FuncAnimation

    plt.rcParams.update({"figure.facecolor": "#f7f7f7", "axes.facecolor": "white",
                         "font.size": 9})
    fig = plt.figure(figsize=(12, 8))
    fig.canvas.manager.set_window_title("Micro-Manipulator encoder monitor")
    gs = GridSpec(3, 3, figure=fig, height_ratios=[3, 2, 1.5],
                  hspace=0.45, wspace=0.28, left=0.07, right=0.98, top=0.94, bottom=0.09)

    ax_pos = fig.add_subplot(gs[0, :])
    ax_pos.set_title("Rotor angle (relative to start) — turn each axis by hand and watch "
                     "for a smooth, continuous trace")
    ax_pos.set_ylabel("degrees")
    ax_pos.grid(alpha=0.3)
    lines = [ax_pos.plot([], [], color=AXIS_COLORS[i], lw=1.4, label=f"axis {i}")[0]
             for i in range(reader.naxes)]
    ax_pos.legend(loc="upper left", ncol=3)

    ax_step = fig.add_subplot(gs[1, :2])
    ax_step.set_title("Sample-to-sample step (a valid encoder never jumps)")
    ax_step.set_ylabel("counts")
    ax_step.set_xlabel("seconds")
    ax_step.grid(alpha=0.3)
    step_lines = [ax_step.plot([], [], color=AXIS_COLORS[i], lw=1.0)[0]
                  for i in range(reader.naxes)]

    ax_noise = fig.add_subplot(gs[1, 2])
    ax_noise.set_title("Noise (stddev)")
    ax_noise.set_ylabel("counts")
    ax_noise.grid(alpha=0.3, axis="y")
    bars = ax_noise.bar(range(reader.naxes), [0] * reader.naxes, color=AXIS_COLORS)
    ax_noise.set_xticks(range(reader.naxes))
    ax_noise.set_xticklabels([f"ax{i}" for i in range(reader.naxes)])

    ax_txt = fig.add_subplot(gs[2, :])
    ax_txt.axis("off")
    status_text = ax_txt.text(0.0, 0.95, "", family="monospace", va="top", fontsize=9.5)

    # --- buttons ---
    def mkbutton(left, label, cb):
        b = Button(fig.add_axes([left, 0.015, 0.11, 0.04]), label)
        b.on_clicked(cb)
        return b

    state = {"paused": False}
    buttons = [
        mkbutton(0.07, "Zero", lambda e: reader.send("zero")),
        mkbutton(0.19, "Probe links", lambda e: reader.send("link")),
        mkbutton(0.31, "Pause", lambda e: state.update(paused=not state["paused"])),
        mkbutton(0.43, "Save CSV", lambda e: save_csv(reader)),
    ]

    def update(_frame):
        if state["paused"]:
            return
        t, series, health = reader.snapshot()
        if not t:
            return
        tmax = t[-1]
        tmin = max(0.0, tmax - PLOT_SECONDS)
        keep = [k for k, v in enumerate(t) if v >= tmin]
        if not keep:
            return
        k0 = keep[0]
        tt = t[k0:]

        for i in range(reader.naxes):
            ys = series[i][k0:]
            lines[i].set_data(tt, ys)
            steps = [0] + [ys[j] - ys[j - 1] for j in range(1, len(ys))]
            # back to counts so the y axis matches the firmware's numbers
            scale = 1.0 / (reader.raw_to_rotor_urad * 1e-6 * 57.29577951308232)
            step_lines[i].set_data(tt, [s * scale for s in steps])
            bars[i].set_height(health[i].sd)
            bars[i].set_color("#D55E00" if health[i].sd > NOISE_WARN_COUNTS
                              else AXIS_COLORS[i])

        for a in (ax_pos, ax_step):
            a.set_xlim(tmin, max(tmin + 1.0, tmax))
            a.relim()
            a.autoscale_view(scalex=False)
        ax_noise.relim()
        ax_noise.autoscale_view()

        rows = [f"{'':5} {'state':5} {'samples':>8} {'weak':>7} {'uvolt':>6} {'ospd':>6} "
                f"{'crc':>6} {'slips':>6} {'frozen':>7} {'sd':>7}  detail"]
        for i, h in enumerate(health):
            st, msg = h.verdict()
            rows.append(f"ax{i}   {st:5} {h.samples:8d} {h.weak:7d} {h.undervolt:6d} "
                        f"{h.overspeed:6d} {h.crc:6d} {h.slips:6d} {h.frozen:7d} "
                        f"{h.sd:7.1f}  {msg}")
        rows.append("")
        rows.append(f"link: {reader.link_errors} corrupted (rejected), "
                    f"{reader.seq_gaps} missing, fw dropped {reader.fw_dropped}, "
                    f"truncated {reader.fw_truncated}")
        note = contention_note(health)
        rows.append(("NOTE: " + note) if note else
                    "Motors are disabled by this firmware - every rotor should turn "
                    "freely by hand.")
        status_text.set_text("\n".join(rows))

    anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)

    if snapshot:
        end = time.time() + snapshot_after
        while time.time() < end:
            time.sleep(0.2)
        update(0)
        fig.savefig(snapshot, dpi=110)
        print(f"saved {snapshot}")
        return 0

    plt.show()
    return 0


def save_csv(reader):
    t, series, _ = reader.snapshot()
    name = time.strftime("encoder_log_%Y%m%d_%H%M%S.csv")
    with open(name, "w") as f:
        f.write("t_s," + ",".join(f"axis{i}_deg" for i in range(reader.naxes)) + "\n")
        for k, tv in enumerate(t):
            f.write(f"{tv:.4f}," + ",".join(f"{series[i][k]:.6f}"
                                            for i in range(reader.naxes)) + "\n")
    print(f"saved {name} ({len(t)} rows)")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (default: auto-detect)")
    ap.add_argument("--headless", type=float, metavar="SECONDS",
                    help="collect for N seconds and print a text report instead of a window")
    ap.add_argument("--snapshot", metavar="PNG",
                    help="render the window to a PNG after a few seconds and exit")
    ap.add_argument("--rate", type=int, help="position record rate in Hz (1..2000)")
    args = ap.parse_args()

    port = find_port(args.port)
    print(f"# connecting to {port}")
    reader = Reader(port)
    reader.start()
    time.sleep(1.0)
    reader.send("info")
    if args.rate:
        reader.send(f"rate {args.rate}")
    reader.send("link")

    try:
        if args.headless:
            return run_headless(reader, args.headless)
        return run_gui(reader, snapshot=args.snapshot)
    finally:
        reader.stop()


if __name__ == "__main__":
    sys.exit(main())
