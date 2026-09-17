# STM32 controller GUI

A small desktop panel for the calibrated STM32F401 micromanipulator firmware. It uses
only Python's built-in Tkinter and PySerial.

## Run

From the repository root:

```bash
python3 -m pip install -r software/STM32ControllerGUI/requirements.txt
python3 software/STM32ControllerGUI/stm32_controller_gui.py
```

On Debian/Ubuntu, install Tkinter once if Python reports that it is missing:

```bash
sudo apt install python3-tk
```

Select the `F401RE_25MHZ_USB_CDC` serial device and click **Connect**. After a reset or
power cycle, click **Home all (G28)** before moving. All axes home together because the
mechanism is mechanically coupled.

Firmware v1.3.2 adds a bidirectional phase-reference measurement near the centre
after the retract: each joint sweeps about +/-2 degrees before holding. This adds
about 12 seconds to homing and does not overwrite the saved calibration. Normal
holding and travel use a position/velocity PI cascade with bounded recovery speed.

With firmware v1.2.0, **Set destination** replaces an active move immediately.
Jog clicks accumulate against the requested destination, so repeated clicks work
while the stage is travelling. Rapid clicks are combined into the latest target.
The live pose is measured from the encoders while enabled; the destination is
displayed separately. The board with USB serial `20883074534E` is preferred when
choosing the port on Windows as well as Linux.

The default speed is 1 mm/s; presets range from 0.05 to 2 mm/s. Feed specifies
peak Cartesian speed; the Y velocity component is limited to 1 mm/s because
the loaded joint approached its safety threshold in the 2 mm/s reversal test.
X and Z can use 2 mm/s. Each move uses
cubic ramps limited to 5 mm/s² within each segment. Retargeting
starts a new ramp at the current commanded position (position is continuous,
but velocity is restarted). Small jogs are acceleration-limited. Step presets
range from 0.001 to 1 mm; a step setting does not certify mechanical accuracy.

**Stop & hold**, or Escape, cancels pending GUI moves and sends `M0`, retaining
motor holding torque. Emergency disable releases the motors with `M18`. During
homing or calibration Escape invokes the abort instead. Enable **Keyboard jog**
for XY arrow keys and Z Page Up/Page Down; keys do not jog while editing text.
**Mark this position** and **Return to mark** provide a session bookmark, cleared
on disconnect, homing, or calibration.

The red **EMERGENCY DISABLE** button aborts an active homing operation or sends `M18`
when the controller is idle. Closing or disconnecting the GUI does not disable a motor
that is already holding; click **Disable motors (M18)** first if that is desired.

## Full calibration

Connect USB and click **Calibrate all axes**. ST-Link is not needed for calibration;
the GUI sends `M56` to the calibration routine built into the production firmware:

1. Calibrate one axis at a time, matching the original RP2040 firmware: find only the
   negative stop at `0.22` PWM amplitude and back off by 90 electrical degrees
   (`1.8` mechanical degrees).
2. Restore the original `0.60` calibration amplitude and record 1024 samples over a
   fixed `83` degree forward sweep, followed by 1024 samples on the return sweep.
3. Build the three 256-point lookup tables onboard and store the validated record in
   reserved STM32 internal flash with a format marker and CRC.
4. Save a timestamped diagnostic log and result plot under
   `documentation/calibration_scan/gui_run_*`.

The result window reports the measured span, enforced safe limit, sweep command,
return error, sample count, and encoder-fault count for each axis. The red emergency
button sends an immediate abort during motor movement. If calibration is aborted or
fails, the previous calibration remains active. The STM32 adaptation also stops if an
axis falls more than four degrees behind
the commanded field, preventing the original unguarded fixed sweep from driving against
an unexpected obstruction indefinitely. A capture is rejected before LUT generation if
the measured span is below 75 degrees, return error exceeds 1 degree, or any encoder
status fault occurs. Do not disconnect power while calibration is being saved.

The live table shows homing/calibration state, measured and target joint angles,
position error, calibrated limit, and encoder status. The controller itself enforces
the measured joint limits and rejects an unsafe Cartesian path before movement.
