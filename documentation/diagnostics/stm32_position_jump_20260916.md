# STM32 position-dependent motion fault reproduction

Date: 2026-09-16  
Controller: STM32F401 on COM4, USB serial `20883074534E`  
Firmware report: `STM32F401 calibrated closed loop`, internal-flash calibration  

## Initial latched fault

After releasing COM4 from the controller GUI and reconnecting without removing
12 V power, `M57` reported that all axes were still homed. The previously
latched fault was:

```text
Controller: STM32F401 calibrated closed loop, drivers=0, fault=1
Fault: axis 0 encoder jump 0.762 deg
Joint 0: is_homed=1, angle=45.7903 deg, target=46.1469 deg,
         error=0.3565 deg, phase=0.523 rad, enc_status=0
```

This confirms that a USB serial disconnect/reconnect does not by itself erase
the STM32's in-RAM homed state. The requirement to run `G28` was caused by the
latched safety fault.

## Recovery

`G28` completed normally. The final retract errors were:

```text
Axis 0: +0.057 deg
Axis 1: -0.129 deg
Axis 2: -0.008 deg
```

The post-home Cartesian pose was approximately
`X-0.054903 Y-0.006260 Z-0.037929`.

## Slow reproduction

The previous failed pose was approached at one quarter of the GUI's default
feed rate:

```text
G0 X1.044683 Y-0.007013 Z-0.037803 F0.0500
```

Selected telemetry from `M57` polling:

```text
time_s  axis0_angle  axis0_error  axis0_phase  maximum_sample_delta
 0.02      42.000       -0.057        +0.052          0.000 deg
 4.00      42.237       +0.070        +0.272          0.013 deg
 8.97      43.412       +0.052        +0.495          0.026 deg
11.62      44.235       -0.016        +0.433          0.029 deg
11.80      44.297       -0.028        +0.413          0.032 deg
11.97      44.599       -0.278        -0.414          0.308 deg
```

The controller then disabled the drivers and latched:

```text
Fault: axis 0 encoder jump 1.370 deg
Joint 0: angle=44.5991 deg, target=44.3220 deg, error=-0.2771 deg,
         phase=-0.414 rad, enc_status=0
```

Axes 1 and 2 remained stable, and all encoder status values remained zero.
The fault therefore reproduces at low Cartesian speed, away from the calibrated
travel limits, and is not accompanied by a weak-field, undervoltage, or
overspeed encoder status.

## Firmware evidence

Normal position feedback uses the calibrated raw-to-position lookup table, but
normal commutation derives its base field from `geometric_position`, the linear
raw-count conversion. The open-loop calibration data shows that the difference
between these two position estimates is position dependent and is multiplied by
50 motor pole pairs. The working Pico 2W controller instead evaluates its
calibrated motor-position-to-field-angle lookup table before adding servo torque.

The observed position-dependent phase loss and encoder jump are consistent with
the STM32 commutation base using the uncalibrated position estimate.

## Post-fix hardware validation

Firmware `v1.1.8-stm32-f401` was confirmed on the controller with `M58`. Its
internal-flash calibration remained valid after flashing.

The first guarded `G28` attempt stopped safely when axis 2 relaxed by about 7°
during the open-loop retract transition. A second guarded attempt completed and
settled with final errors of `+0.054`, `+0.045`, and `-0.016` degrees. No safety
condition was bypassed.

The controller then executed the following continuous outward move, crossing
both previously failing regions near X=1.04 mm and X=3.65 mm:

```text
G0 X3.800000 Y-0.006318 Z-0.037893 F0.2000
```

It reached X=3.80 mm and returned to the post-home pose at the same feed rate.
The monitored results were:

```text
                 outward       return
duration          19.27 s       19.27 s
peak joint error   0.255 deg     0.213 deg
peak phase         0.388 rad     0.395 rad
peak sampled step  0.224 deg     0.106 deg
encoder status     all zero      all zero
safety fault       none          none
```

After the test, `M18` disabled the drivers. `M57` reported `fault=0` and
`drivers=0`.

## Loaded-axis reversal and final controller tuning

Subsequent full-span Y reversals isolated a second problem on loaded axis 1. The
failure occurred with valid encoder CRC and status at both `F0.5` and `F0.2`.
Immediately before the faster failure the controller had reached its phase-lead
limit, and the slower reproduction still produced a single-sample jump just over
the 0.75-degree safety limit. This ruled out USB baud rate and SPI corruption as
the cause.

Firmware `v1.1.11-stm32-f401` changes the normal-motion position gain from `80`
to `60` and velocity damping from `0.04` to `0.20`. The position gain now matches
the working Pico controller, while the fivefold stronger velocity feedback damps
the axis 1 reversal mode. Integral holding authority, drive amplitude, phase
limit, and the 0.75-degree safety threshold are unchanged. No safety threshold
was widened.

The firmware also records per-move peak jump, following error, filtered velocity,
phase lead, and phase saturation ticks in `M57`. A fault now preserves the actual
faulting sample in diagnostics. MT6835 frames are CRC-8 checked with up to three
immediate attempts before any sample can update feedback or commutation.

Hardware validation on COM4 (`20883074534E`) passed all of the following:

- exact `Y+3.8 -> Y-3.8 -> 0` reproduction at `F0.2`;
- full-span X and Y reversals at `F0.5`;
- four `(+/-3, +/-3)` XY diagonal traversals at `F0.5`;
- `Z+1 -> Z-1 -> 0` at `F0.2`;
- eight repeated `Y3.55 <-> Y3.75` short reversal cycles;
- USB reconnect, `M17` re-enable without rehoming, and return to the origin.
- explicit `G0` and `G1` moves, `M53` completion polling, rejection of a second
  move with `busy`, and rejection of an out-of-workspace move without a fault.

There were no safety trips, encoder-status faults, CRC retries, CRC failures, or
phase-saturation ticks. The worst normal-motion encoder sample jump was
`0.2730 deg` during the final return after re-enable, versus the unchanged
`0.75 deg` stop threshold. During the faster full-span suite the worst jump was
`0.2419 deg`. The stage was returned to `X0 Y0 Z0` and the drivers were disabled
with `M18` after testing.
