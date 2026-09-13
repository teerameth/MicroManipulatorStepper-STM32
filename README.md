# Open Micro-Manipulator — STM32F401 Edition

## Why this exists

I ran out of Raspberry Pi Pico 2 boards. The parts drawer, meanwhile, contained
an embarrassing number of STM32F401 BlackPills. Rather than wait for a delivery,
I chose the only sensible engineering response: make an adapter, rewire a
BlackPill into the Pico socket, and teach it to impersonate the original motion
controller.

The result is an STM32F401RET6 controller adaptation of the Open
Micro-Manipulator, an open-source motorized XYZ micromanipulator based on a
three-axis parallel linkage. It provides equivalent motor-driver, magnetic
encoder, USB G-code, guarded homing, and onboard-calibration functions for the
existing control board—just with an STM32 doing the work.

This is not a drop-in firmware binary for a Pico; it is a hardware-and-firmware
conversion. The required wiring is documented in [the adapter wiring guide]
(documentation/stm32_blackpill_adapter_wiring.md).

The original Raspberry Pi Pico implementation remains at
[0x23/MicroManipulatorStepper](https://github.com/0x23/MicroManipulatorStepper).
This repository intentionally contains only the STM32 production firmware.

## What is included

- STM32F401RET6 firmware for the Pico-socket adapter
- ST-Link upload configuration and 25 MHz HSE / USB CDC board definition
- guarded encoder-locked homing and closed-loop motor control
- onboard `M56` calibration with CRC-protected internal-flash storage
- Tk/PySerial desktop control and calibration GUI
- encoder-monitoring tools and adapter wiring documentation

## Hardware target

The supported controller is an **STM32F401RET6** board using a 25 MHz external
crystal and USB CDC. It connects to the existing motion-control PCB through the
adapter described in
[the adapter wiring guide](documentation/stm32_blackpill_adapter_wiring.md).

Use the original project's mechanics, encoder boards, motors, and control PCB.
Never connect or disconnect a motor while the motor rail is powered.

## Build and upload

Install PlatformIO, connect an ST-Link, and run:

```bash
cd firmware/MotionControllerRP
~/.platformio/penv/bin/pio run -e f401re_controller -t upload
```

The firmware starts with motor-driver standby low. After USB CDC appears, use
the stable `/dev/serial/by-id/...F401RE_25MHZ_USB_CDC...` path when available.

## First use

1. Connect the GUI:

   ```bash
   python3 -m pip install -r software/STM32ControllerGUI/requirements.txt
   python3 software/STM32ControllerGUI/stm32_controller_gui.py
   ```

2. Click **Home all (G28)**. The mechanically coupled axes must home together.
3. If this is a new controller or encoder/magnet alignment changed, run the
   built-in calibration with **Onboard calibrate (M56)**. This does not flash
   firmware; it saves validated calibration data to STM32 internal flash.
4. Use the GUI or USB G-code commands for guarded XYZ motion. Use `M18` to
   release the motors.

See the [firmware README](firmware/MotionControllerRP/src/stm32_controller/README.md)
and [GUI README](software/STM32ControllerGUI/README.md) for operating details.

## G-code subset

The STM32 controller supports `G0`, `G1`, `G4`, `G24`, `G28`, `M3`, `M17`,
`M18`, `M50`–`M53`, `M56`, `M57`, and `M58`. It accepts one guarded Cartesian
move at a time; it does not include the Pico firmware's look-ahead planner.

## License and upstream project

This is an adaptation of the original Open Micro-Manipulator project. Preserve
the upstream license and attribution when redistributing this work.
