from pathlib import Path
import re


SOURCE = (
    Path(__file__).parents[1]
    / "firmware"
    / "MotionControllerRP"
    / "src"
    / "stm32_controller"
    / "main.cpp"
)


def test_closed_loop_field_uses_calibrated_position() -> None:
    source = SOURCE.read_text(encoding="utf-8")

    assert "float closed_loop_field(" in source
    assert "calibrated_position * POLE_PAIRS + phase_lead" in source
    assert source.count("closed_loop_field(state, position") == 6


def test_raw_geometric_position_is_not_used_for_closed_loop_commutation() -> None:
    source = SOURCE.read_text(encoding="utf-8")

    assert not re.search(r"geometric_position\s*\*\s*POLE_PAIRS", source)
    retract = source.split("bool retract_all_from_home_open_loop", 1)[1].split("bool home_all_parallel", 1)[0]
    assert "state.home_field =" not in retract


def test_firmware_version_identifies_fixed_image() -> None:
    source = SOURCE.read_text(encoding="utf-8")

    assert 'FIRMWARE_VERSION[] = "v1.3.2-stm32-f401"' in source
    assert "Serial.println(FIRMWARE_VERSION)" in source


def test_home_retract_starts_from_measured_calibrated_positions() -> None:
    source = SOURCE.read_text(encoding="utf-8")

    assert "float start_position_deg[AXIS_COUNT]" in source
    assert "start_position_deg[axis] = position * RAD_TO_DEG_F" in source
    assert "(target_deg - start_position_deg[axis]) * smooth" in source
    assert "fabsf(command_deg[axis] - position * RAD_TO_DEG_F)" in source


def test_encoder_crc_is_checked_before_updating_absolute_position() -> None:
    source = SOURCE.read_text(encoding="utf-8")

    crc_check = source.index(
        "data[5] != calculate_crc(uint32_t(raw), last_sensor_status)"
    )
    position_update = source.index("absolute_raw_ += delta")
    assert crc_check < position_update
    assert "constexpr int MAX_ATTEMPTS = 3" in source
    assert "status_ = last_sensor_status | STATUS_CRC_ERROR" in source
    assert "crc_retries=%lu, crc_failures=%lu" in source


def test_normal_motion_uses_cascaded_servo_without_rebasing() -> None:
    source = SOURCE.read_text(encoding="utf-8")

    assert "state.servo.update(error, state.velocity, dt)" in source
    assert "state.servo.reset(initial_phase)" in source


def test_motion_diagnostics_capture_faulting_sample() -> None:
    source = SOURCE.read_text(encoding="utf-8")

    fault_position_update = source.index("state.position = position;", source.index("if (jump_deg > 0.75f)"))
    fault_latch = source.index("latch_fault(message);", fault_position_update)
    assert fault_position_update < fault_latch
    assert "peak_sample_jump" in source
    assert "peak_abs_velocity" in source
    assert "saturated_ticks" in source
    assert "reset_motion_diagnostics();" in source
