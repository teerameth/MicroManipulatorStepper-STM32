# STM32F401RET6-to-Pico Socket Adapter — As-Built Wiring

This is the minimal wired adapter that replaces the Raspberry Pi Pico module with a
40-pin STM32 board fitted with an STM32F401RET6. It records the wiring after omitting the
optional 150 ohm series resistors and external CS/STBY pull resistors.

ST-Link read `DBGMCU_IDCODE = 0x10016433` (`DEV_ID 0x433`), confirming that the
installed die is an STM32F401xD/E. Firmware must therefore use the F401RE target
(84 MHz maximum, 96 KiB SRAM), even if a seller listing or earlier visual inspection
identified it as an F411.

The port names (`PAx` and `PBx`) are authoritative. `BPxx` is the physical Blackpill
header position, counted in the standard WeAct 40-pin layout with the USB-C connector
at the top. Pico pad numbers are the original PCB's Pico socket positions, also viewed
with its USB end at the top.

## Mandatory local link

Solder this short link on the adapter:

| From | To | Purpose |
|---|---|---|
| Pico pad 25 (`GP19`, shared `PWMA/PWMB`) | Pico pad 36 (`3V3`) | Holds all TB6612 PWM-enable inputs high. Use a direct wire; no resistor is required. |

Do **not** leave pad 25 open. The TB6612 internal pull-down would keep `PWMA/PWMB`
low, so the motors would not drive.

## Encoder SPI bus

| Blackpill | Pico socket pad | Signal |
|---|---:|---|
| BP3 `PB14` | 1 `GP0` | Encoder MISO |
| BP2 `PB13` | 4 `GP2` | Encoder SCK |
| BP4 `PB15` | 5 `GP3` | Encoder MOSI |
| BP27 `PA1` | 26 `GP20` | Encoder 1 CS |
| BP28 `PA2` | 27 `GP21` | Encoder 2 CS |
| BP32 `PA6` | 29 `GP22` | Encoder 3 CS |

No external CS pull-up is installed. During reset, a CS line may briefly float; the
STM32 firmware must configure **all three CS pins as outputs HIGH before starting SPI**.

## Motor-driver phase inputs

| Axis / TB6612 input | Blackpill | Pico socket pad |
|---|---|---:|
| Motor 1 `AIN1` | BP10 `PA15` | 17 `GP13` |
| Motor 1 `AIN2` | BP11 `PB3` | 16 `GP12` |
| Motor 1 `BIN1` | BP37 `PB10` | 19 `GP14` |
| Motor 1 `BIN2` | BP29 `PA3` | 20 `GP15` |
| Motor 2 `AIN1` | BP12 `PB4` | 12 `GP9` |
| Motor 2 `AIN2` | BP13 `PB5` | 11 `GP8` |
| Motor 2 `BIN1` | BP34 `PB0` | 14 `GP10` |
| Motor 2 `BIN2` | BP35 `PB1` | 15 `GP11` |
| Motor 3 `AIN1` | BP14 `PB6` | 7 `GP5` |
| Motor 3 `AIN2` | BP15 `PB7` | 6 `GP4` |
| Motor 3 `BIN1` | BP16 `PB8` | 9 `GP6` |
| Motor 3 `BIN2` | BP17 `PB9` | 10 `GP7` |

## Shared control, tools, and power

| Function | Blackpill | Pico socket pad | Notes |
|---|---|---:|---|
| Driver standby (`STBY`) | BP6 `PA9` | 24 `GP18` | No external pull-down; the TB6612 has an internal pull-down. |
| Tool 1 PWM | BP5 `PA8` | 21 `GP16` | |
| Tool 2 PWM | BP7 `PA10` | 22 `GP17` | |
| 3.3 V logic | BP38 `3V3` | 36 `3V3` | Never apply 5 V to an STM32 GPIO. |
| Ground | BP39 `GND` | 38 `GND` | |
| Ground | BP19 `GND` | 3 `GND` | Second ground wire for a low-impedance return. |
| Board power | BP40 `5V` | 39 `VSYS` | Install as a removable jumper only. |

Keep the BP40-to-pad-39 jumper **open whenever the Blackpill is powered from USB**.
Closing it while USB is connected can back-feed the board's 5 V rail.

## Parts deliberately not installed

- No 150 ohm series resistors. They are unnecessary for these short adapter wires.
- No 10 kOhm encoder-CS pull-ups. They improve reset-state determinism but are not
  required after firmware configures every CS pin HIGH.
- No 10 kOhm STBY pull-down. The TB6612 provides an internal approximately 200 kOhm
  pull-down, which holds the drivers in standby during reset.
- No resistor in the pad-25-to-pad-36 PWM-enable link: it is a direct wire.

## First-power checklist

1. Keep the motor supply and all motors disconnected. Confirm ground continuity across
   BP19/BP39 and Pico pads 3/38, and confirm the direct pad-25-to-pad-36 link.
2. With the BP40-to-pad-39 jumper **open**, power the Blackpill by USB. Verify 3.3 V at
   the Blackpill 3V3 pin and no unexpected voltage on Pico pad 39.
3. Disconnect USB. Apply the board's normal 5 V supply and close the BP40-to-pad-39
   jumper only if that supply is intended to power the Blackpill.
4. Run firmware that implements this STM32 pin map, initializes all encoder CS pins
   HIGH, and keeps `STBY` LOW until startup is complete. The existing Pico firmware
   cannot be flashed directly to the STM32.
5. Verify encoder communication with motors disabled before reconnecting a motor supply.
   Do not power the motor rail while the previously reported U2/U3 low-resistance
   faults remain unresolved.
