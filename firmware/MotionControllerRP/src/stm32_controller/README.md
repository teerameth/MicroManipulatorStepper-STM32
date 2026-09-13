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

Guarded homing uses a `0.40` PWM/voltage amplitude. Normal closed-loop output uses
`0.60` on all three axes, matching the original firmware and giving every joint equal
electrical correction authority. These values are PWM/voltage fractions rather than
regulated motor current. The bounded calibration sweep also uses `0.60`.
Homing searches toward the negative home stop at 3 degrees/s; the 110-degree
allowance lets it start near the opposite end of the nominal 90-degree mechanism.
Encoder faults, unexpected jumps, a search overrun, and serial abort disable all drivers.

Normal commands include `G0`/`G1`, `G4`, `G24`, `M3`, `M17`, `M18`, `M50`–`M53`,
`M56`, `M57`, and `M58`. `M18` disables all motor drivers. Encoder faults, an encoder jump,
or calibrated travel violation also disables the drivers and latches a fault; run a
successful `G28` to clear it.

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
