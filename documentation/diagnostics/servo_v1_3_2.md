# Vibration and bidirectional holding recovery

Board: STM32F401, USB serial `20883074534E`, COM4. Final firmware: v1.3.2.

## Changes

- Replaced the direct position PI/damping loop with the Pico's two-loop
  position/velocity PI structure and time-based 4 ms velocity filtering.
- Normal electrical phase limit follows the Pico's +/-0.45 pi (81 degrees).
  The encoder jump stop remains 0.75 mechanical degrees per sample.
- Limited requested joint recovery velocity and the position integrator to
  0.35 rad/s. This retains integral holding torque while preventing a displaced
  position from requesting the original Pico limit of 2 pi rad/s.
- Replaced one-way endpoint commutation rebasing with small symmetric field
  sweeps near the centre. A circular average of the calibrated encoder/field
  residuals estimates the reference from both directions. The actual applied
  field is transferred to the velocity PI integral at handoff.
- Kept the existing Cartesian speed limits, GUI retargeting, and flash LUTs.

Reference code inspected: `MicroManipulatorStepper/firmware/MotionControllerRP/`
in both the FPM-tele submodule and the adjacent local MicroManipulatorStepper
checkout. The PI implementation is attributed in `cascaded_servo.h`.

## Trials and evidence

The first trial (v1.3.0) removed endpoint rebasing but retained the old reference.
It homed accurately, then axis 1 stalled near 42 degrees with its positive phase
saturated while its target continued to 56.84 degrees. The test failed its
settling requirement, disabled the drivers, and the board was reset through
ST-Link before the next trial.

The bidirectional estimator in v1.3.1 measured approximately 1.18 radians of
reference correction on axis 1. Long X/Y/Z and diagonal travel then tracked
with peak errors around 0.03 degrees. An abrupt +0.2 mm Y target step still
caused a -0.763-degree sample jump and a safety stop. The board was reset by
flashing v1.3.2, which adds the recovery velocity limit.

The first v1.3.2 qualification run passed two full rounds: 24 travel moves and
24 abrupt step/return checks, then a five-second hold. No safety stop or phase
saturation occurred. Measured maxima from `servo_v1_3_2_trial1.log`:

| Test | Peak sample change | Peak following error | Error at accepted endpoint |
|---|---:|---:|---:|
| Normal travel | 0.0324 deg | 0.0557 deg | 0.0006 deg |
| +/-0.2 mm steps and returns | 0.0886 deg | 0.7650 deg* | 0.0004 deg |
| Stationary hold | 0.0002 deg | 0.0003 deg | Below displayed precision |

*The initial following error of a deliberately discontinuous target step is
expected; the relevant result is recovery and settling in both directions.

The second run is captured separately in `servo_v1_3_2_trial2.log`, using a new
G28, requested feed 2 mm/s (Y remains limited to 1), and +/-0.5 mm target steps.
It also passed all 24 travel moves and 24 step/return checks, with no safety
stops or phase saturation:

| Test | Peak sample change | Peak following error | Error at accepted endpoint |
|---|---:|---:|---:|
| Normal travel | 0.0374 deg | 0.0695 deg | 0.0008 deg |
| +/-0.5 mm steps and returns | 0.1617 deg | 1.9104 deg* | 0.0008 deg |
| Stationary hold | 0.0006 deg | 0.0008 deg | Below displayed precision |

Together, the final firmware passed 96 movement/return checks across two
homing cycles and two stationary hold tests. The stage returned to commanded
XYZ zero and the test disabled motor drivers with M18. The final PlatformIO
build succeeded and all 13 Python tests passed; the native C++ controller
test also passed. Failed earlier versions were not left on the board.

The hardware test requires every joint to stay within 0.05 degrees for 300 ms
before accepting arrival; it does not mistake an expired trajectory timer for
successful settling. `tests/hardware_servo_qualification.py --run-hardware`
reproduces the procedure. Native C++ tests execute the same controller header
and check polarity symmetry, finite bounded output, reversal of saturated
holding bias, and reset. Python tests cover GUI motion intent and firmware
integration checks.

These are encoder-based motor-control results, not an independent measurement
of absolute sample position. Abrupt target steps exercise bidirectional recovery,
but do not physically reproduce a person's hand pushing a horn. External-force
rejection cannot be fully verified without that physical disturbance.
