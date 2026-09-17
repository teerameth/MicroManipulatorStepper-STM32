# STM32F401 calibrated controller

This is the single firmware target for the STM32F401RET6-to-Pico-socket adapter. It
contains encoder diagnostics, guarded homing, onboard calibration, persistent LUT
storage, closed-loop motion, kinematics, tools, and USB G-code control. The table in
`calibration_data.h` is only the factory fallback used when internal flash has no valid
calibration record.

## Upload

From `firmware/MotionControllerRP/`:

```bash
~/.platformio/penv/bin/pio run -e f401re_controller -t upload
```

ST-Link is the configured uploader. After reset, USB CDC appears at 921600 baud. Prefer
the stable `/dev/serial/by-id/usb-STMicroelectronics_F401RE_25MHZ_USB_CDC_in_FS_Mode_20883074534E-if00`
path when it exists.

## Start and use

The controller always boots with the shared TB6612 `STBY` line low. Run:

```text
G28
```

All three joints must home together because they form a coupled parallel mechanism.
The firmware learns each commutation phase from centered negative/positive probes at
low amplitude, homes all motors
simultaneously with encoder-locked phase limits, retracts to 42 degrees, and replies
`ok` only if every axis settles safely.
The 42-degree retract starts from each axis's measured calibrated angle after the
drivers are re-enabled. This tolerates harmless linkage relaxation while the shared
driver standby line is low without weakening the jump or tracking-lag safety checks.

Guarded homing uses a `0.40` PWM/voltage amplitude. Normal closed-loop output uses
`0.60` on all three axes, matching the original firmware and giving every joint equal
electrical correction authority. These values are PWM/voltage fractions rather than
regulated motor current. The bounded calibration sweep also uses `0.60`.
Normal closed-loop commutation uses the calibrated raw-count-to-position lookup table
for its base field angle as well as for position feedback. Do not substitute the linear
raw-count geometric estimate: its local error is multiplied by the motor's 50 pole pairs
and can produce a position-dependent phase reversal.
Homing searches toward the negative home stop at 3 degrees/s; the 110-degree
allowance lets it start near the opposite end of the nominal 90-degree mechanism.
Encoder faults, unexpected jumps, a search overrun, and serial abort disable all drivers.
Every MT6835 angle frame is CRC-8 validated before it can update position feedback or
motor commutation. A bad frame is retried up to three times; exhausting all retries
sets encoder status bit `0x08` and disables the drivers. `M57` reports cumulative
`crc_retries` and `crc_failures` for each axis.
Normal motion uses the Pico-derived position/velocity PI cascade in
`cascaded_servo.h`: position P/I = 60/30000, velocity P/I = 0.2/90, with a 4 ms
velocity filter. The requested recovery speed is bounded to 0.35 mechanical
rad/s (about 20 deg/s); electrical phase is bounded to +/-0.45 pi, as on the Pico.
Full integral holding torque remains available at zero speed. The former direct
position PI loop with velocity damping is no longer used for normal motion.
`M57` also reports
per-move peak encoder jump, following error, filtered velocity, phase lead, and phase
saturation ticks. These counters reset when a new `G0` or `G1` move is accepted.

After retracting, homing includes small +/-2-degree field sweeps near the centre
on each joint (about 12 additional seconds). Calibrated encoder positions from
both directions determine a circular-mean electrical reference. This avoids
using the friction-loaded endpoint of a one-way retract as the neutral field.
The actual applied field is carried into the velocity integral for the handoff.
The stored calibration LUT is not overwritten by this reference measurement.

Normal commands include `G0`/`G1`, `G4`, `G24`, `M3`, `M17`, `M18`, `M50`–`M53`,
`M56`, `M57`, and `M58`. `M18` disables all motor drivers. Encoder faults, an encoder jump,
or calibrated travel violation also disables the drivers and latches a fault; run a
successful `G28` to clear it.

As of v1.2.0, `G0`/`G1` replace the current destination, including during travel.
Unspecified coordinates preserve the previous destination. A replacement is
validated before changing the active move; an invalid request leaves it intact.
Each segment starts from the current interpolated position with a fresh cubic
ramp; position remains continuous but retargeting restarts velocity at zero.
Feed is peak mm/s, accepted in `(0, 2]`, with the Y component capped at 1 mm/s;
acceleration within each ramp is limited
to 5 mm/s². `M0` cancels travel and holds the interpolated target. `M50` returns
measured Cartesian position while enabled, followed by a `Destination:` line.
Queries and tool commands remain available during normal movement. Homing and
calibration remain exclusive operations with their existing abort behavior.

For interactive operation and live state monitoring, use the desktop GUI in
[`software/STM32ControllerGUI`](../../../../software/STM32ControllerGUI/README.md).

Example:

```text
G28
M57
G0 X0.1 Y0 Z0 F0.2
M53
M18
```

The existing Python API is compatible with the implemented command subset. Its homing
timeout is 120 seconds to allow the safety-oriented 2 degree/s procedure to finish.

## Onboard calibration

Send `M56`, or click **Calibrate all axes** in the GUI. No firmware change or ST-Link
connection is required. For each axis the controller finds the negative stop, backs off
1.8 mechanical degrees, records 1024 samples over a fixed 83-degree forward pass and
1024 on the return, builds a 256-point LUT, and applies quality gates. Only after all
three axes pass does it write one versioned, CRC-protected record to internal flash.

The final 128 KiB flash sector is reserved for EEPROM emulation; the application size
is therefore capped at 384 KiB in the board definition. A failed or aborted measurement
does not replace the active calibration. After success, run `G28` before normal motion.
