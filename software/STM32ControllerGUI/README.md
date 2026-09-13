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
