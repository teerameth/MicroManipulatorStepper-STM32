# Sample-stage interaction changes, v1.2.0

Controller tested: STM32 USB serial `20883074534E`, COM4.

G0/G1 now replace an active destination instead of returning busy. Unspecified
axes retain their destination. Replacement trajectories start at the current
interpolated position, avoiding the old jump back to the previous move's origin.
Each replacement restarts a cubic ramp at zero velocity: this is position-
continuous retargeting, not continuous-velocity trajectory blending.

The GUI accumulates jogs against the destination and combines rapid edits before
sending. M0 cancels travel while holding; Stop clears pending GUI commands first.
Measured position and destination have separate displays. Added speed/step
presets, opt-in keyboard jog, session mark/return, and board-serial preference.

Default GUI feed is 1 mm/s. Feed now denotes peak speed; X/Z can request up to
2 mm/s, the Y component is capped at 1 mm/s, and each ramp's acceleration is
limited to 5 mm/s². A long 3.8 mm move takes approximately 5.7 seconds at the new
default versus 19 seconds with the former GUI default F0.2. Short moves also
depend on acceleration. These limits apply to G0/G1, not legacy direct G24.

## Hardware results

The final image was flashed and verified through ST-Link. The reproducible test
is `tests/hardware_retarget.py --run-hardware`; its captured output is
`retarget_v1_2_0_hardware.log` in this directory.

Passed homing; X/Y +/-3.8 mm reversals; Z +/-1 mm; requested feeds 1 and 2;
G1 replacement during G0 travel; M50/M53 queries during travel; M0 hold; return
to zero; and M18 disable. No safety trip or CRC error occurred.

Y is still the limiting axis. An earlier uncapped 2 mm/s Y reversal reached
0.6236 degrees peak sample jump with phase saturation, motivating the Y cap.
The final run also showed variability at the 1 mm/s limit: maximum jump
0.6486 degrees (threshold remains 0.75), with 22 saturated ticks on one full Y
reversal and 39 on the final diagonal return. Passing this test does not establish
an absence of saturation or qualify higher speeds, other loads, or full travel.

The actual GUI and serial worker were then exercised on hardware: five rapid
0.1 mm X jog clicks produced one X0.5 command, a subsequent negative jog replaced
it with X0.4, and Stop cancelled a pending X3 command during an X2 move. M0 was
accepted; the stage returned toward X0 Y0 Z0 and was disabled with M18.

Software verification: 13 tests passed, including accumulation, coalescing,
cancelled queues, stale-reply handling, and measured/destination parsing. Tk GUI
construction and the PlatformIO release build also passed.
