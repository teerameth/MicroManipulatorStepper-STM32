// STM32F401RET6 production controller for the hand-wired Pico-socket adapter.
//
// The RP2040 hardware layer is intentionally not reused here.  This target provides
// guarded encoder-locked homing, flash-resident measured calibration, closed-loop
// commutation, delta kinematics, bounded Cartesian moves, and the host-facing G-code
// subset used by software/PythonAPI/open_micro_stage_api.py.

#include <Arduino.h>
#include <HardwareTimer.h>
#include <SPI.h>
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include "calibration_data.h"
#include "cascaded_servo.h"
#include "kinematic_models/kinematic_model_delta3d.h"
#include "persistent_calibration.h"

namespace {

constexpr int AXIS_COUNT = 3;
constexpr int32_t ENCODER_CPR = 1 << 21;
constexpr float PI_F = 3.14159265358979323846f;
constexpr float TWO_PI_F = 2.0f * PI_F;
constexpr float DEG_TO_RAD_F = PI_F / 180.0f;
constexpr float RAD_TO_DEG_F = 180.0f / PI_F;
constexpr float RAW_TO_ROTOR_RAD = 1.0f / 10485760.0f;
constexpr float POLE_PAIRS = 50.0f;
constexpr uint32_t SPI_HZ = 4000000;
constexpr uint32_t PWM_HZ = 20000;
constexpr uint32_t PWM_MAX = 4095;
constexpr char FIRMWARE_VERSION[] = "v1.3.2-stm32-f401";
// Axes 0/1 plateaued early during the original 0.18-amplitude calibration.
// Use the already-qualified normal drive amplitude during the bounded homing
// sequence so ordinary linkage friction is less likely to look like an end stop.
constexpr float HOME_AMPLITUDE = 0.40f;
// Use the original firmware's drive level on every joint. Axis 1 needed this
// headroom against the installed linkage load, and equal amplitudes give equal
// electrical authority for equal-magnitude phase corrections on all axes.
constexpr float DRIVE_AMPLITUDE[AXIS_COUNT] = {0.60f, 0.60f, 0.60f};
constexpr float CALIBRATION_AMPLITUDE = 0.60f;
constexpr float CALIBRATION_RANGE_DEG = 83.0f;
constexpr float CALIBRATION_FIELD_VELOCITY_RAD_S = 20.0f;
constexpr int CALIBRATION_SAMPLE_COUNT = 1024;
constexpr float HOME_SPEED_DPS = 3.0f;
// This is only a one-direction home search. Allow enough field travel to find
// the negative stop when starting near the opposite end of the ~90-degree span.
constexpr float HOME_SEARCH_DEG = 110.0f;
// Match the original HomingController::finalize(): define the usable zero after
// retracting 90 electrical degrees (1.8 mechanical degrees for 50 pole pairs).
constexpr float HOME_REFERENCE_BACKOFF_FIELD_RAD = PI_F * 0.5f;
constexpr float HOME_RETRACT_DEG = 42.0f;
constexpr float MIN_COMMAND_DEG = 0.75f;
constexpr float MAX_PHASE_LEAD = PI_F / 3.0f;
// Guarded home-search gains; normal motion uses cascaded_servo.h.
constexpr float POSITION_KP = 60.0f;
constexpr float VELOCITY_DAMPING = 0.20f;
constexpr uint32_t SERVO_PERIOD_US = 1000;
constexpr uint8_t STATUS_OVERSPEED = 0x01;
constexpr uint8_t STATUS_WEAK_FIELD = 0x02;
constexpr uint8_t STATUS_UNDERVOLT = 0x04;
constexpr uint8_t STATUS_CRC_ERROR = 0x08;

constexpr uint32_t PIN_STBY = PA9;
constexpr uint32_t PIN_SCK = PB13;
constexpr uint32_t PIN_MISO = PB14;
constexpr uint32_t PIN_MOSI = PB15;
constexpr uint32_t PIN_CS[AXIS_COUNT] = {PA1, PA2, PA6};
constexpr uint32_t PIN_TOOL[2] = {PA8, PA10};

constexpr uint32_t MOTOR_GPIO[AXIS_COUNT][4] = {
    {PA15, PB3, PB10, PA3},
    {PB4, PB5, PB0, PB1},
    {PB6, PB7, PB8, PB9},
};

constexpr PinName MOTOR_PWM_PIN[AXIS_COUNT][4] = {
    {PA_15, PB_3, PB_10, PA_3},
    {PB_4, PB_5, PB_0_ALT1, PB_1_ALT1},
    {PB_6, PB_7, PB_8, PB_9},
};

SPIClass encoder_spi(PIN_MOSI, PIN_MISO, PIN_SCK);
HardwareTimer *motor_timer[AXIS_COUNT] = {};
HardwareTimer *tool_timer = nullptr;

void serialf(const char *format, ...) {
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  Serial.print(buffer);
}

class Encoder {
 public:
  explicit Encoder(uint32_t chip_select) : cs_(chip_select) {}

  void begin_gpio() {
    pinMode(cs_, OUTPUT);
    digitalWrite(cs_, HIGH);
  }

  int32_t read_raw() {
    constexpr int MAX_ATTEMPTS = 3;
    uint8_t last_sensor_status = 0;
    for (int attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
      uint8_t data[6] = {0xA0, 0x03, 0, 0, 0, 0};
      encoder_spi.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE3));
      digitalWrite(cs_, LOW);
      encoder_spi.transfer(data, sizeof(data));
      digitalWrite(cs_, HIGH);
      encoder_spi.endTransaction();

      last_sensor_status = data[4] & 0x07;
      const int32_t raw = (int32_t(data[2]) << 13) |
                          (int32_t(data[3]) << 5) | (data[4] >> 3);
      if (data[5] != calculate_crc(uint32_t(raw), last_sensor_status)) {
        ++crc_retries_;
        continue;
      }

      status_ = last_sensor_status;
      if (!initialized_) {
        last_raw_ = raw;
        initialized_ = true;
        return absolute_raw_;
      }
      int32_t delta = raw - last_raw_;
      if (delta > ENCODER_CPR / 2) delta -= ENCODER_CPR;
      if (delta < -ENCODER_CPR / 2) delta += ENCODER_CPR;
      absolute_raw_ += delta;
      last_raw_ = raw;
      return absolute_raw_;
    }

    // Never pass a corrupt angle into position feedback or commutation. The
    // caller disables the drivers if all immediate retries fail.
    ++crc_failures_;
    status_ = last_sensor_status | STATUS_CRC_ERROR;
    return absolute_raw_;
  }

  bool connected() {
    int valid = 0;
    for (int i = 0; i < 12; ++i) {
      read_raw();
      if (status_ != 0x07) ++valid;
    }
    return valid >= 10;
  }

  uint8_t status() const { return status_; }
  uint32_t crc_retries() const { return crc_retries_; }
  uint32_t crc_failures() const { return crc_failures_; }

 private:
  static uint8_t calculate_crc(uint32_t angle, uint8_t status) {
    uint8_t crc = 0;
    const uint8_t bytes[3] = {
        uint8_t(angle >> 13), uint8_t(angle >> 5),
        uint8_t((angle << 3) | (status & 0x07))};
    for (uint8_t value : bytes) {
      crc ^= value;
      for (int bit = 0; bit < 8; ++bit)
        crc = (crc & 0x80) ? uint8_t((crc << 1) ^ 0x07)
                           : uint8_t(crc << 1);
    }
    return crc;
  }

  uint32_t cs_;
  int32_t last_raw_ = 0;
  int32_t absolute_raw_ = 0;
  uint8_t status_ = 0x07;
  uint32_t crc_retries_ = 0;
  uint32_t crc_failures_ = 0;
  bool initialized_ = false;
};

Encoder encoder[AXIS_COUNT] = {Encoder(PIN_CS[0]), Encoder(PIN_CS[1]),
                               Encoder(PIN_CS[2])};

struct AxisState {
  int32_t home_raw = 0;
  int32_t last_raw = 0;
  float home_field = 0.0f;
  float direction = 1.0f;
  float geometric_position = 0.0f;
  float position = 0.0f;
  float previous_position = 0.0f;
  float velocity = 0.0f;
  float target = 0.0f;
  float integral_error = 0.0f;
  float phase_lead = 0.0f;
  CascadedServo::Controller servo;
  float last_sample_jump = 0.0f;
  float peak_sample_jump = 0.0f;
  float peak_abs_error = 0.0f;
  float peak_abs_velocity = 0.0f;
  float peak_abs_phase = 0.0f;
  uint32_t saturated_ticks = 0;
  uint32_t diagnostic_ticks = 0;
  bool homed = false;
};

// The calibration sweep records encoder raw count as a function of commanded
// mechanical motor position.  Normal feedback must therefore use the calibrated
// position for both the position error and the base commutation angle.  Using the
// linear raw-count estimate here creates a position-dependent electrical phase
// error that is multiplied by POLE_PAIRS and can reverse motor torque.
float closed_loop_field(const AxisState &state, float calibrated_position,
                        float phase_lead = 0.0f) {
  return state.home_field + state.direction *
      (calibrated_position * POLE_PAIRS + phase_lead);
}

AxisState axis_state[AXIS_COUNT];
PersistentCalibration::Data calibration{};
bool calibration_loaded_from_flash = false;
bool drivers_enabled = false;
bool fault_latched = false;
char fault_text[128] = {};
float tool_value[2] = {};
KinematicModel_Delta3D kinematics;
Pose6DF current_pose;

struct CartesianMove {
  bool active = false;
  Pose6DF start;
  Pose6DF end;
  uint32_t start_ms = 0;
  uint32_t duration_ms = 0;
} move;

char command_line[192];
size_t command_length = 0;
float default_feed_mm_s = 1.0f;
uint32_t last_servo_us = 0;

void set_compare(int axis, int channel, uint32_t value) {
  motor_timer[axis]->setCaptureCompare(channel + 1, value,
                                       RESOLUTION_12B_COMPARE_FORMAT);
}

void all_inputs_low() {
  for (int axis = 0; axis < AXIS_COUNT; ++axis)
    for (int channel = 0; channel < 4; ++channel) set_compare(axis, channel, 0);
}

void disable_drivers() {
  digitalWrite(PIN_STBY, LOW);
  drivers_enabled = false;
  all_inputs_low();
}

void latch_fault(const char *message) {
  disable_drivers();
  move.active = false;
  fault_latched = true;
  strncpy(fault_text, message, sizeof(fault_text) - 1);
  fault_text[sizeof(fault_text) - 1] = '\0';
  serialf("E)STM32 safety stop: %s\n", fault_text);
}

void set_signed_phase(int axis, int positive_channel, int negative_channel,
                      int32_t value) {
  value = constrain(value, -int32_t(PWM_MAX), int32_t(PWM_MAX));
  if (value >= 0) {
    set_compare(axis, positive_channel, PWM_MAX - value);
    set_compare(axis, negative_channel, PWM_MAX);
  } else {
    set_compare(axis, positive_channel, PWM_MAX);
    set_compare(axis, negative_channel, PWM_MAX + value);
  }
}

void set_field(int axis, float field_rad, float amplitude) {
  // Clamp every caller to the electrically qualified calibration amplitude.
  amplitude = constrain(amplitude, 0.0f, CALIBRATION_AMPLITUDE);
  const int32_t peak = int32_t(amplitude * float(PWM_MAX) + 0.5f);
  set_signed_phase(axis, 0, 1, int32_t(sinf(field_rad) * peak));
  set_signed_phase(axis, 2, 3, int32_t(cosf(field_rad) * peak));
}

void reset_motion_diagnostics() {
  for (auto &state : axis_state) {
    state.last_sample_jump = 0.0f;
    state.peak_sample_jump = 0.0f;
    state.peak_abs_error = 0.0f;
    state.peak_abs_velocity = 0.0f;
    state.peak_abs_phase = 0.0f;
    state.saturated_ticks = 0;
    state.diagnostic_ticks = 0;
  }
}

void start_one_axis(int axis, float field_rad, float amplitude) {
  disable_drivers();
  set_field(axis, field_rad, 0.0f);
  digitalWrite(PIN_STBY, HIGH);
  drivers_enabled = true;
  for (int step = 1; step <= 20; ++step) {
    set_field(axis, field_rad, amplitude * float(step) / 20.0f);
    delay(5);
  }
}

float read_geometric_position(int axis) {
  return float(encoder[axis].read_raw()) * RAW_TO_ROTOR_RAD;
}

float average_geometric_position(int axis, int samples = 24) {
  double sum = 0.0;
  for (int i = 0; i < samples; ++i) {
    sum += read_geometric_position(axis);
    delayMicroseconds(150);
  }
  return float(sum / samples);
}

float evaluate_calibration_deg(int axis, int32_t raw_from_home) {
  const auto &cal = calibration.axis[axis];
  if (raw_from_home <= 0) return 0.0f;
  if (raw_from_home >= cal.raw_max)
    return cal.position_deg[PersistentCalibration::LUT_SIZE - 1];
  const float index = float(raw_from_home) *
      float(PersistentCalibration::LUT_SIZE - 1) / float(cal.raw_max);
  const int lower = int(index);
  const float fraction = index - float(lower);
  return cal.position_deg[lower] +
         fraction * (cal.position_deg[lower + 1] - cal.position_deg[lower]);
}

void load_calibration() {
  calibration_loaded_from_flash = PersistentCalibration::load(calibration);
  if (calibration_loaded_from_flash) return;

  static_assert(Stm32Calibration::LUT_SIZE == PersistentCalibration::LUT_SIZE,
                "embedded and persistent calibration LUT sizes differ");
  for (int axis = 0; axis < AXIS_COUNT; ++axis) {
    calibration.axis[axis].raw_max = Stm32Calibration::AXIS[axis].raw_max;
    calibration.axis[axis].max_deg = Stm32Calibration::AXIS[axis].max_deg;
    memcpy(calibration.axis[axis].position_deg,
           Stm32Calibration::AXIS[axis].position_deg,
           sizeof(calibration.axis[axis].position_deg));
  }
  calibration.home_reference_after_backoff =
      Stm32Calibration::HOME_REFERENCE_AFTER_BACKOFF;
}

bool all_homed() {
  for (const auto &state : axis_state)
    if (!state.homed) return false;
  return true;
}

bool abort_requested() {
  if (!Serial.available()) return false;
  while (Serial.available()) Serial.read();
  disable_drivers();
  move.active = false;
  Serial.println("E)Operation aborted; motor drivers disabled");
  return true;
}

bool track_coupled_axes(int driven_axis) {
  for (int axis = 0; axis < AXIS_COUNT; ++axis) {
    if (axis == driven_axis) continue;
    encoder[axis].read_raw();
    if (encoder[axis].status() != 0) {
      disable_drivers();
      serialf("E)Axis %d encoder fault while axis %d was moving\n", axis,
              driven_axis);
      return false;
    }
  }
  return true;
}

bool learn_commutation(int axis, float &position, float &field, float &direction) {
  // Axis 1 needs a two-mechanical-degree probe to overcome static friction. Axes
  // 0/2 retain the proven smaller probe. Measure both directions so linkage load
  // and phase lag do not get baked into a one-sided commutation reference.
  const float probe_field_deg = axis == 1 ? 100.0f : 20.0f;
  const float probe_field = probe_field_deg * DEG_TO_RAD_F;
  const float required_motion_deg = axis == 1 ? 0.50f : (axis == 0 ? 0.10f : 0.15f);
  const uint32_t probe_duration_ms = axis == 1 ? 1000 : 600;
  start_one_axis(axis, 0.0f, HOME_AMPLITUDE);
  delay(150);
  const float p0 = average_geometric_position(axis);
  float previous = p0;

  auto ramp_probe = [&](float from_field, float to_field, uint32_t duration_ms,
                        float &result_position) -> bool {
    const uint32_t started = millis();
    while (millis() - started < duration_ms) {
      if (abort_requested()) return false;
      const float fraction = float(millis() - started) / float(duration_ms);
      const float probe = from_field + (to_field - from_field) *
          constrain(fraction, 0.0f, 1.0f);
      set_field(axis, probe, HOME_AMPLITUDE);
      const float sample = read_geometric_position(axis);
      if (!track_coupled_axes(axis)) return false;
      const float jump_deg = fabsf((sample - previous) * RAD_TO_DEG_F);
      previous = sample;
      if (encoder[axis].status() != 0 || jump_deg > 1.0f) {
        disable_drivers();
        serialf("E)Axis %d encoder fault during commutation probe\n", axis);
        return false;
      }
      delayMicroseconds(500);
    }
    set_field(axis, to_field, HOME_AMPLITUDE);
    delay(100);
    result_position = average_geometric_position(axis);
    previous = result_position;
    return true;
  };

  float positive_position = p0;
  if (!ramp_probe(0.0f, probe_field, probe_duration_ms, positive_position))
    return false;
  const float first_positive_move =
      (positive_position - p0) * RAD_TO_DEG_F;
  const bool first_positive_valid =
      fabsf(first_positive_move) >= required_motion_deg;

  float negative_position = positive_position;
  if (!ramp_probe(probe_field, -probe_field, 2 * probe_duration_ms,
                  negative_position))
    return false;
  // If the first positive probe started against a hard stop, move away first
  // and repeat it so both phase endpoints represent a freely followed field.
  if (!first_positive_valid &&
      !ramp_probe(-probe_field, probe_field, 2 * probe_duration_ms,
                  positive_position))
      return false;

  const float endpoint_span_deg =
      (positive_position - negative_position) * RAD_TO_DEG_F;
  serialf("I)Axis %d centered phase probe: first +%.3f deg, endpoint span %.3f deg\n",
          axis, first_positive_move, endpoint_span_deg);
  if (fabsf(endpoint_span_deg) < 2.0f * required_motion_deg) {
    disable_drivers();
    serialf("E)Axis %d did not follow the centered commutation probe\n", axis);
    return false;
  }

  direction = endpoint_span_deg > 0.0f ? 1.0f : -1.0f;
  const float positive_offset =
      probe_field - direction * positive_position * POLE_PAIRS;
  const float negative_offset =
      -probe_field - direction * negative_position * POLE_PAIRS;
  // Electrical phase is periodic. Unwrap the negative endpoint onto the same
  // turn as the positive endpoint before averaging the two offsets.
  const float centered_offset = positive_offset +
      0.5f * remainderf(negative_offset - positive_offset, TWO_PI_F);
  position = first_positive_valid ? negative_position : positive_position;
  // Return a virtual field at the centered offset. The homing caller derives
  // commutation_offset as field - direction*position*pole_pairs.
  field = direction * position * POLE_PAIRS + centered_offset;
  return true;
}

bool home_axis(int axis) {
  constexpr float MIN_WINDOW_MOVE_DEG = 0.04f;
  constexpr float MIN_TRAVEL_DEG = 0.25f;
  constexpr uint32_t WINDOW_MS = 200;
  constexpr int REQUIRED_STALL_WINDOWS = 3;

  if (!encoder[axis].connected()) {
    serialf("E)Axis %d encoder link failed; homing not started\n", axis);
    return false;
  }

  float start_position = 0.0f;
  float field = 0.0f;
  float direction = 0.0f;
  if (!learn_commutation(axis, start_position, field, direction)) return false;
  const float commutation_offset = field - direction * start_position * POLE_PAIRS;
  float previous_position = start_position;
  float window_position = start_position;
  float filtered_velocity = 0.0f;
  uint32_t previous_us = micros();
  const uint32_t start_ms = millis();
  uint32_t next_window_ms = WINDOW_MS;
  int stalled_windows = 0;
  int last_report = -1;

  serialf("I)Homing axis %d: encoder-locked %.1f deg/s\n", axis, HOME_SPEED_DPS);
  while (true) {
    if (abort_requested()) return false;
    const uint32_t elapsed_ms = millis() - start_ms;
    const float commanded_deg = HOME_SPEED_DPS * float(elapsed_ms) * 0.001f;
    if (commanded_deg >= HOME_SEARCH_DEG) {
      disable_drivers();
      serialf("E)Axis %d: no end stop within %.0f deg\n", axis, HOME_SEARCH_DEG);
      return false;
    }

    const uint32_t now_us = micros();
    float dt = float(now_us - previous_us) * 1e-6f;
    previous_us = now_us;
    dt = constrain(dt, 0.00005f, 0.02f);
    const int32_t raw = encoder[axis].read_raw();
    if (!track_coupled_axes(axis)) return false;
    const float position = float(raw) * RAW_TO_ROTOR_RAD;
    const float jump_deg = fabsf((position - previous_position) * RAD_TO_DEG_F);
    const float velocity = (position - previous_position) / dt;
    previous_position = position;
    filtered_velocity += 0.08f * (velocity - filtered_velocity);
    if (encoder[axis].status() != 0 || jump_deg > 1.0f) {
      disable_drivers();
      serialf("E)Axis %d encoder lost synchronization while homing\n", axis);
      return false;
    }

    const float target = start_position - commanded_deg * DEG_TO_RAD_F;
    const float error = target - position;
    const float phase_lead = constrain(
        POSITION_KP * error - VELOCITY_DAMPING * filtered_velocity,
        -MAX_PHASE_LEAD, MAX_PHASE_LEAD);
    field = direction * position * POLE_PAIRS + commutation_offset +
            direction * phase_lead;
    set_field(axis, field, HOME_AMPLITUDE);

    if (elapsed_ms >= next_window_ms) {
      next_window_ms += WINDOW_MS;
      const float average = average_geometric_position(axis, 12);
      const float window_move = fabsf((average - window_position) * RAD_TO_DEG_F);
      const float total_travel = fabsf((average - start_position) * RAD_TO_DEG_F);
      window_position = average;
      const bool saturated = fabsf(phase_lead) >= 0.90f * MAX_PHASE_LEAD;
      stalled_windows = saturated && total_travel >= MIN_TRAVEL_DEG &&
                                window_move < MIN_WINDOW_MOVE_DEG
                            ? stalled_windows + 1
                            : 0;
      const int report = int(commanded_deg / 5.0f);
      if (report != last_report) {
        last_report = report;
        serialf("I)Axis %d home: command %.1f deg, encoder %.3f deg\n", axis,
                commanded_deg, (average - start_position) * RAD_TO_DEG_F);
      }
      if (stalled_windows >= REQUIRED_STALL_WINDOWS) {
        const float home_position = average_geometric_position(axis);
        AxisState &state = axis_state[axis];
        state.home_raw = encoder[axis].read_raw();
        state.last_raw = state.home_raw;
        state.home_field = direction * home_position * POLE_PAIRS + commutation_offset;
        state.direction = direction;
        state.geometric_position = 0.0f;
        state.position = 0.0f;
        state.previous_position = 0.0f;
        state.target = 0.0f;
        state.velocity = 0.0f;
        state.integral_error = 0.0f;
        state.homed = true;
        disable_drivers();
        serialf("I)Axis %d home confirmed; encoder travel %.3f deg\n", axis,
                (home_position - start_position) * RAD_TO_DEG_F);
        return true;
      }
    }
    delayMicroseconds(500);
  }
}

// Scratch capture buffers are reused for each axis. Keeping only raw counts is
// sufficient because the original method's commanded field positions are uniformly
// spaced and therefore implicit in the sample index.
int32_t calibration_forward_raw[CALIBRATION_SAMPLE_COUNT];
int32_t calibration_reverse_raw[CALIBRATION_SAMPLE_COUNT];

float interpolate_raw_at_command(const int32_t *raw, bool reverse,
                                 float command_deg) {
  const float sample_position =
      constrain(command_deg / CALIBRATION_RANGE_DEG, 0.0f, 1.0f) *
      float(CALIBRATION_SAMPLE_COUNT - 1);
  const int lower = min(int(sample_position), CALIBRATION_SAMPLE_COUNT - 2);
  const float fraction = sample_position - float(lower);
  if (!reverse)
    return float(raw[lower]) + fraction * float(raw[lower + 1] - raw[lower]);
  const int high_index = CALIBRATION_SAMPLE_COUNT - 1 - lower;
  return float(raw[high_index]) +
         fraction * float(raw[high_index - 1] - raw[high_index]);
}

bool build_calibration_lut(PersistentCalibration::AxisLut &result) {
  constexpr float SAFE_END_MARGIN_DEG = 1.0f;
  const float command_step =
      CALIBRATION_RANGE_DEG / float(CALIBRATION_SAMPLE_COUNT - 1);

  // Each pass must remain monotonic. Small equal runs are allowed for quantization,
  // but a reversal larger than encoder noise indicates slip or invalid mechanics.
  for (int i = 1; i < CALIBRATION_SAMPLE_COUNT; ++i) {
    if (calibration_forward_raw[i] + 2000 < calibration_forward_raw[i - 1] ||
        calibration_reverse_raw[i] > calibration_reverse_raw[i - 1] + 2000)
      return false;
  }

  const float safe_command = CALIBRATION_RANGE_DEG - SAFE_END_MARGIN_DEG;
  result.raw_max = int32_t(floorf(min(
      interpolate_raw_at_command(calibration_forward_raw, false, safe_command),
      interpolate_raw_at_command(calibration_reverse_raw, true, safe_command))));
  if (result.raw_max <= 0) return false;

  int forward_index = 0;
  int reverse_index = 0;
  float first_estimate = 0.0f;
  for (int lut_index = 0; lut_index < PersistentCalibration::LUT_SIZE;
       ++lut_index) {
    const float query = float(result.raw_max) * float(lut_index) /
                        float(PersistentCalibration::LUT_SIZE - 1);
    while (forward_index + 1 < CALIBRATION_SAMPLE_COUNT - 1 &&
           float(calibration_forward_raw[forward_index + 1]) < query)
      ++forward_index;
    const float forward_span = float(calibration_forward_raw[forward_index + 1] -
                                     calibration_forward_raw[forward_index]);
    const float forward_fraction = forward_span > 0.0f
        ? constrain((query - float(calibration_forward_raw[forward_index])) /
                        forward_span,
                    0.0f, 1.0f)
        : 0.0f;
    const float forward_command =
        (float(forward_index) + forward_fraction) * command_step;

    auto reverse_raw_ascending = [](int index) {
      return calibration_reverse_raw[CALIBRATION_SAMPLE_COUNT - 1 - index];
    };
    while (reverse_index + 1 < CALIBRATION_SAMPLE_COUNT - 1 &&
           float(reverse_raw_ascending(reverse_index + 1)) < query)
      ++reverse_index;
    const float reverse_span =
        float(reverse_raw_ascending(reverse_index + 1) -
              reverse_raw_ascending(reverse_index));
    const float reverse_fraction = reverse_span > 0.0f
        ? constrain((query - float(reverse_raw_ascending(reverse_index))) /
                        reverse_span,
                    0.0f, 1.0f)
        : 0.0f;
    const float reverse_command =
        (float(reverse_index) + reverse_fraction) * command_step;

    float estimate = 0.5f * (forward_command + reverse_command);
    if (lut_index == 0) first_estimate = estimate;
    estimate -= first_estimate;
    if (lut_index > 0)
      estimate = max(estimate, result.position_deg[lut_index - 1]);
    result.position_deg[lut_index] = estimate;
  }
  result.position_deg[0] = 0.0f;
  result.max_deg = result.position_deg[PersistentCalibration::LUT_SIZE - 1];
  return result.max_deg >= 75.0f;
}

bool capture_calibration_axis(int axis,
                              PersistentCalibration::AxisLut &candidate) {
  constexpr uint32_t BACKOFF_MS = 100;
  constexpr float MAX_SAMPLE_JUMP_DEG = 1.0f;
  constexpr float MAX_TRACKING_LAG_DEG = 4.0f;
  constexpr int TRACKING_FAULT_SAMPLES = 5;
  constexpr float MAX_RETURN_ERROR_DEG = 1.0f;

  AxisState &state = axis_state[axis];
  const float stop_field = state.home_field;
  const float direction = state.direction;
  start_one_axis(axis, stop_field, HOME_AMPLITUDE);
  delay(50);
  const float stop_position = average_geometric_position(axis, 32);

  const uint32_t backoff_started = millis();
  while (millis() - backoff_started < BACKOFF_MS) {
    if (abort_requested()) return false;
    const float fraction = constrain(
        float(millis() - backoff_started) / float(BACKOFF_MS), 0.0f, 1.0f);
    set_field(axis,
              stop_field + direction * HOME_REFERENCE_BACKOFF_FIELD_RAD * fraction,
              HOME_AMPLITUDE);
    encoder[axis].read_raw();
    if (encoder[axis].status() != 0) {
      disable_drivers();
      serialf("error: axis %d encoder fault during home backoff\n", axis);
      return false;
    }
    delayMicroseconds(500);
  }
  const float scan_field =
      stop_field + direction * HOME_REFERENCE_BACKOFF_FIELD_RAD;
  set_field(axis, scan_field, HOME_AMPLITUDE);
  delay(80);
  const float scan_home_position = average_geometric_position(axis, 32);
  const float backoff_deg =
      (scan_home_position - stop_position) * RAD_TO_DEG_F;
  if (backoff_deg < 0.5f || backoff_deg > 3.0f) {
    disable_drivers();
    serialf("error: axis %d home backoff was %.3f deg; expected about +1.8 deg\n",
            axis, backoff_deg);
    return false;
  }
  const int32_t scan_home_raw = encoder[axis].read_raw();

  for (int step = 1; step <= 40; ++step) {
    if (abort_requested()) return false;
    const float amplitude = HOME_AMPLITUDE +
        (CALIBRATION_AMPLITUDE - HOME_AMPLITUDE) * float(step) / 40.0f;
    set_field(axis, scan_field, amplitude);
    delay(5);
  }

  serialf("CALIBRATION axis %d: fixed %.0f-deg forward/reverse, "
          "%d samples/pass, amplitude %.2f\n",
          axis, CALIBRATION_RANGE_DEG, CALIBRATION_SAMPLE_COUNT,
          CALIBRATION_AMPLITUDE);
  Serial.println("MAGDATA_HEADER,axis,pass,command_deg,raw_from_home,rotor_deg,status");

  const float command_step =
      CALIBRATION_RANGE_DEG / float(CALIBRATION_SAMPLE_COUNT - 1);
  const uint32_t step_us = uint32_t(
      command_step * DEG_TO_RAD_F * POLE_PAIRS /
      CALIBRATION_FIELD_VELOCITY_RAD_S * 1000000.0f);
  float previous_position = scan_home_position;
  float far_position = scan_home_position;
  int bad_tracking_samples = 0;
  uint32_t weak = 0, undervolt = 0, overspeed = 0;

  auto sample_pass = [&](bool forward, int32_t *capture) -> bool {
    Serial.println(forward ? "FORWARD fixed-range pass..." :
                             "REVERSE fixed-range pass...");
    for (int sample = 0; sample < CALIBRATION_SAMPLE_COUNT; ++sample) {
      if (abort_requested()) return false;
      const float command_deg = forward
          ? float(sample) * command_step
          : CALIBRATION_RANGE_DEG - float(sample) * command_step;
      set_field(axis,
                scan_field + direction * command_deg * DEG_TO_RAD_F * POLE_PAIRS,
                CALIBRATION_AMPLITUDE);
      const uint32_t step_started = micros();
      uint8_t status_or = 0;
      int32_t raw = encoder[axis].read_raw();
      while (uint32_t(micros() - step_started) < step_us) {
        if (abort_requested()) return false;
        raw = encoder[axis].read_raw();
        status_or |= encoder[axis].status();
        delayMicroseconds(250);
      }
      const int32_t raw_from_home = raw - scan_home_raw;
      capture[sample] = raw_from_home;
      const float position = float(raw) * RAW_TO_ROTOR_RAD;
      const float measured_deg =
          float(raw_from_home) * RAW_TO_ROTOR_RAD * RAD_TO_DEG_F;
      const float jump_deg =
          fabsf((position - previous_position) * RAD_TO_DEG_F);
      previous_position = position;
      if (status_or & STATUS_WEAK_FIELD) ++weak;
      if (status_or & STATUS_UNDERVOLT) ++undervolt;
      if (status_or & STATUS_OVERSPEED) ++overspeed;

      serialf("MAGDATA,%d,%c,%.5f,%ld,%.5f,%u\n", axis,
              forward ? 'F' : 'R', command_deg, long(raw_from_home),
              measured_deg, status_or);
      if (status_or != 0 || jump_deg > MAX_SAMPLE_JUMP_DEG) {
        disable_drivers();
        serialf("error: axis %d encoder fault during calibration sweep\n", axis);
        return false;
      }
      const float lag_deg = fabsf(command_deg - measured_deg);
      bad_tracking_samples = lag_deg > MAX_TRACKING_LAG_DEG
                                 ? bad_tracking_samples + 1
                                 : 0;
      if (bad_tracking_samples >= TRACKING_FAULT_SAMPLES) {
        disable_drivers();
        serialf("error: axis %d stopped tracking at command %.2f deg "
                "(encoder %.2f deg)\n",
                axis, command_deg, measured_deg);
        return false;
      }
      if (forward && sample == CALIBRATION_SAMPLE_COUNT - 1)
        far_position = position;
    }
    return true;
  };

  if (!sample_pass(true, calibration_forward_raw)) return false;
  delay(150);
  bad_tracking_samples = 0;
  if (!sample_pass(false, calibration_reverse_raw)) return false;
  delay(150);
  const float return_position = average_geometric_position(axis, 32);
  const float measured_span =
      (far_position - scan_home_position) * RAD_TO_DEG_F;
  const float return_error =
      fabsf((return_position - scan_home_position) * RAD_TO_DEG_F);
  disable_drivers();

  if (measured_span < 75.0f || return_error > MAX_RETURN_ERROR_DEG ||
      !build_calibration_lut(candidate)) {
    serialf("error: axis %d calibration quality check failed "
            "(span %.3f, return %.3f)\n",
            axis, measured_span, return_error);
    return false;
  }
  serialf("ORIGINALCAL result axis %d: forward measured %.3f deg; "
          "return error %.3f deg; safe limit %.3f deg\n",
          axis, measured_span, return_error, candidate.max_deg);
  serialf("status steps axis %d: weak=%lu, undervolt=%lu, overspeed=%lu\n",
          axis, static_cast<unsigned long>(weak),
          static_cast<unsigned long>(undervolt),
          static_cast<unsigned long>(overspeed));
  return true;
}

bool run_onboard_calibration() {
  disable_drivers();
  move.active = false;
  fault_latched = false;
  fault_text[0] = '\0';
  for (auto &state : axis_state) state.homed = false;

  PersistentCalibration::Data candidate = calibration;
  candidate.home_reference_after_backoff = true;
  Serial.println("CALIBRATION started: onboard original-method STM32 adaptation");
  Serial.println("CALIBRATION each axis: negative home, 1.8-deg backoff, 83-deg round trip");
  for (int axis = 0; axis < AXIS_COUNT; ++axis) {
    serialf("CALIBRATION progress axis %d/3: homing\n", axis + 1);
    if (!home_axis(axis) || !capture_calibration_axis(axis, candidate.axis[axis])) {
      disable_drivers();
      for (auto &state : axis_state) state.homed = false;
      Serial.println("error: onboard calibration aborted; previous calibration retained");
      Serial.println("SAFE: STBY is LOW");
      return false;
    }
    delay(500);
  }

  disable_drivers();
  Serial.println("CALIBRATION saving validated LUTs to internal flash...");
  if (!PersistentCalibration::save(candidate)) {
    for (auto &state : axis_state) state.homed = false;
    Serial.println("error: calibration flash verification failed; runtime data unchanged");
    Serial.println("SAFE: STBY is LOW");
    return false;
  }
  calibration = candidate;
  calibration_loaded_from_flash = true;
  for (auto &state : axis_state) state.homed = false;
  Serial.println("CALIBRATION complete: saved and activated; run G28 before motion");
  Serial.println("SAFE: STBY is LOW");
  Serial.println("ok");
  return true;
}

void initialize_motor_pwm() {
  for (int axis = 0; axis < AXIS_COUNT; ++axis) {
    for (int channel = 0; channel < 4; ++channel) {
      pinMode(MOTOR_GPIO[axis][channel], OUTPUT);
      digitalWrite(MOTOR_GPIO[axis][channel], LOW);
    }
  }
  motor_timer[0] = new HardwareTimer(TIM2);
  motor_timer[1] = new HardwareTimer(TIM3);
  motor_timer[2] = new HardwareTimer(TIM4);
  for (int axis = 0; axis < AXIS_COUNT; ++axis) {
    motor_timer[axis]->pause();
    motor_timer[axis]->setOverflow(PWM_HZ, HERTZ_FORMAT);
    for (int channel = 0; channel < 4; ++channel) {
      motor_timer[axis]->setMode(channel + 1, TIMER_OUTPUT_COMPARE_PWM1,
                                 MOTOR_PWM_PIN[axis][channel]);
      set_compare(axis, channel, 0);
    }
    motor_timer[axis]->resume();
  }
}

void initialize_tool_pwm() {
  tool_timer = new HardwareTimer(TIM1);
  tool_timer->pause();
  tool_timer->setOverflow(8000, HERTZ_FORMAT);
  tool_timer->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PA_8);
  tool_timer->setMode(3, TIMER_OUTPUT_COMPARE_PWM1, PA_10);
  tool_timer->setCaptureCompare(1, 0, RESOLUTION_8B_COMPARE_FORMAT);
  tool_timer->setCaptureCompare(3, 0, RESOLUTION_8B_COMPARE_FORMAT);
  tool_timer->resume();
}

void set_tool(int index, float value) {
  value = constrain(value, 0.0f, 1.0f);
  tool_value[index] = value;
  const uint32_t compare = uint32_t(value * 255.0f + 0.5f);
  tool_timer->setCaptureCompare(index == 0 ? 1 : 3, compare,
                                RESOLUTION_8B_COMPARE_FORMAT);
}

bool sample_axis(int axis, float &position) {
  AxisState &state = axis_state[axis];
  const int32_t raw = encoder[axis].read_raw();
  if (encoder[axis].status() != 0) {
    char message[96];
    snprintf(message, sizeof(message), "axis %d encoder status 0x%02X", axis,
             encoder[axis].status());
    latch_fault(message);
    return false;
  }
  const int32_t raw_from_home = raw - state.home_raw;
  const int32_t raw_margin = 250000;
  if (raw_from_home < -raw_margin ||
      raw_from_home > calibration.axis[axis].raw_max + raw_margin) {
    char message[96];
    snprintf(message, sizeof(message), "axis %d exceeded calibrated travel", axis);
    latch_fault(message);
    return false;
  }
  position = evaluate_calibration_deg(axis, raw_from_home) * DEG_TO_RAD_F;
  state.geometric_position = float(raw_from_home) * RAW_TO_ROTOR_RAD;
  state.last_raw = raw;
  return true;
}

bool targets_within_limits(const float targets[AXIS_COUNT], bool print_error) {
  for (int axis = 0; axis < AXIS_COUNT; ++axis) {
    const float degrees = targets[axis] * RAD_TO_DEG_F;
    const float maximum = calibration.axis[axis].max_deg;
    if (!isfinite(degrees) || degrees < MIN_COMMAND_DEG || degrees > maximum) {
      if (print_error)
        serialf("error: axis %d target %.3f deg outside calibrated [%.2f, %.2f]\n",
                axis, degrees, MIN_COMMAND_DEG, maximum);
      return false;
    }
  }
  return true;
}

bool enable_closed_loop() {
  if (!all_homed()) return false;
  for (int axis = 0; axis < AXIS_COUNT; ++axis) {
    float position;
    if (!sample_axis(axis, position)) return false;
    AxisState &state = axis_state[axis];
    state.position = position;
    state.previous_position = position;
    state.target = position;
    state.velocity = 0.0f;
    state.integral_error = 0.0f;
    set_field(axis, closed_loop_field(state, position), 0.0f);
  }
  digitalWrite(PIN_STBY, HIGH);
  drivers_enabled = true;
  for (int step = 1; step <= 20; ++step) {
    const float ramp_fraction = float(step) / 20.0f;
    for (int axis = 0; axis < AXIS_COUNT; ++axis) {
      AxisState &state = axis_state[axis];
      float position;
      if (!sample_axis(axis, position)) return false;
      state.position = position;
      state.previous_position = position;
      set_field(axis, closed_loop_field(state, position),
                DRIVE_AMPLITUDE[axis] * ramp_fraction);
    }
    delay(5);
  }
  // The last ramp step is followed by a 5 ms delay. Resample and realign at full
  // amplitude so motion during that final interval is not mistaken for an
  // encoder jump on the first servo tick. A few iterations also let coupled
  // linkage motion settle before capturing the closed-loop targets.
  for (int settle = 0; settle < 5; ++settle) {
    for (int axis = 0; axis < AXIS_COUNT; ++axis) {
      AxisState &state = axis_state[axis];
      float position;
      if (!sample_axis(axis, position)) return false;
      state.position = position;
      state.previous_position = position;
      set_field(axis, closed_loop_field(state, position),
                DRIVE_AMPLITUDE[axis]);
    }
    delay(5);
  }
  for (int axis = 0; axis < AXIS_COUNT; ++axis) {
    AxisState &state = axis_state[axis];
    float position;
    if (!sample_axis(axis, position)) return false;
    state.position = position;
    state.previous_position = position;
    state.target = position;
    state.velocity = 0.0f;
    state.integral_error = 0.0f;
    state.phase_lead = 0.0f;
    state.servo.reset();
  }
  last_servo_us = micros();
  return true;
}

void update_cartesian_target() {
  if (!move.active) return;
  const uint32_t elapsed = millis() - move.start_ms;
  float fraction = move.duration_ms ? float(elapsed) / float(move.duration_ms) : 1.0f;
  fraction = constrain(fraction, 0.0f, 1.0f);
  const float smooth = fraction * fraction * (3.0f - 2.0f * fraction);
  Pose6DF pose = Pose6DF::lerp(move.start, move.end, smooth);
  float targets[AXIS_COUNT];
  if (!kinematics.inverse(pose, targets) || !targets_within_limits(targets, false)) {
    latch_fault("trajectory left calibrated workspace");
    return;
  }
  for (int axis = 0; axis < AXIS_COUNT; ++axis) axis_state[axis].target = targets[axis];
  current_pose = pose;
  if (fraction >= 1.0f) {
    current_pose = move.end;
    move.active = false;
  }
}

void servo_tick(float dt) {
  if (!drivers_enabled) {
    for (auto &item : encoder) item.read_raw();
    return;
  }
  update_cartesian_target();
  if (!drivers_enabled) return;
  for (int axis = 0; axis < AXIS_COUNT; ++axis) {
    AxisState &state = axis_state[axis];
    float position;
    if (!sample_axis(axis, position)) return;
    const float signed_jump =
        (position - state.previous_position) * RAD_TO_DEG_F;
    const float jump_deg = fabsf(signed_jump);
    state.last_sample_jump = signed_jump;
    state.peak_sample_jump = max(state.peak_sample_jump, jump_deg);
    if (jump_deg > 0.75f) {
      // Preserve the faulting sample for M57 instead of leaving telemetry at
      // the preceding sample, which obscured the direction of a real slip.
      state.position = position;
      char message[128];
      snprintf(message, sizeof(message),
               "axis %d encoder jump %+.3f deg (target %.3f, position %.3f)",
               axis, signed_jump, state.target * RAD_TO_DEG_F,
               position * RAD_TO_DEG_F);
      latch_fault(message);
      return;
    }
    const float raw_velocity = (position - state.previous_position) / dt;
    state.previous_position = position;
    state.position = position;
    state.velocity += dt / (0.004f + dt) * (raw_velocity - state.velocity);

    const float error = state.target - position;
    state.peak_abs_error = max(state.peak_abs_error, fabsf(error));
    state.peak_abs_velocity = max(state.peak_abs_velocity, fabsf(state.velocity));
    state.phase_lead = state.servo.update(error, state.velocity, dt);
    state.peak_abs_phase = max(state.peak_abs_phase, fabsf(state.phase_lead));
    if (fabsf(state.phase_lead) >= 0.99f * CascadedServo::PHASE_LIMIT)
      ++state.saturated_ticks;
    ++state.diagnostic_ticks;
    const float field = closed_loop_field(state, position, state.phase_lead);
    set_field(axis, field, DRIVE_AMPLITUDE[axis]);
  }
}

bool move_all_joints_to(float target_deg, float speed_deg_s) {
  float start[AXIS_COUNT];
  for (int axis = 0; axis < AXIS_COUNT; ++axis) start[axis] = axis_state[axis].position;
  const uint32_t duration_ms = uint32_t(target_deg / speed_deg_s * 1000.0f);
  const uint32_t started = millis();
  uint32_t previous_us = micros();
  while (millis() - started < duration_ms) {
    if (abort_requested()) return false;
    const float fraction = constrain(float(millis() - started) / float(duration_ms),
                                     0.0f, 1.0f);
    const float smooth = fraction * fraction * (3.0f - 2.0f * fraction);
    for (int axis = 0; axis < AXIS_COUNT; ++axis)
      axis_state[axis].target = start[axis] +
          (target_deg * DEG_TO_RAD_F - start[axis]) * smooth;
    const uint32_t now_us = micros();
    if (now_us - previous_us >= SERVO_PERIOD_US) {
      const float dt = float(now_us - previous_us) * 1e-6f;
      previous_us = now_us;
      servo_tick(constrain(dt, 0.0005f, 0.01f));
      if (!drivers_enabled) return false;
    }
  }
  for (auto &state : axis_state) state.target = target_deg * DEG_TO_RAD_F;
  const uint32_t settle_start = millis();
  while (millis() - settle_start < 500) {
    const uint32_t now_us = micros();
    if (now_us - previous_us >= SERVO_PERIOD_US) {
      const float dt = float(now_us - previous_us) * 1e-6f;
      previous_us = now_us;
      servo_tick(constrain(dt, 0.0005f, 0.01f));
      if (!drivers_enabled) return false;
    }
  }
  bool accurate = true;
  for (int axis = 0; axis < AXIS_COUNT; ++axis) {
    const float error_deg = (axis_state[axis].target - axis_state[axis].position) * RAD_TO_DEG_F;
    serialf("I)Axis %d retract error %.3f deg\n", axis, error_deg);
    accurate &= fabsf(error_deg) < 0.75f;
  }
  if (!accurate) latch_fault("home retract did not settle within 0.75 deg");
  return accurate;
}

bool retract_all_from_home_open_loop(float target_deg, float speed_deg_s) {
  constexpr float MAX_TRACKING_LAG_DEG = 5.0f;
  constexpr int MAX_LAG_SAMPLES = 10;
  constexpr float MAX_RETRACT_JUMP_DEG = 1.0f;

  disable_drivers();
  float previous_position[AXIS_COUNT] = {};
  float start_position_deg[AXIS_COUNT] = {};
  float start_field[AXIS_COUNT] = {};
  int lag_samples[AXIS_COUNT] = {};
  for (int axis = 0; axis < AXIS_COUNT; ++axis) {
    AxisState &state = axis_state[axis];
    float position;
    if (!sample_axis(axis, position)) return false;
    state.position = position;
    state.previous_position = position;
    previous_position[axis] = position;
    start_position_deg[axis] = position * RAD_TO_DEG_F;
    start_field[axis] = closed_loop_field(state, position);
    set_field(axis, start_field[axis], 0.0f);
  }

  digitalWrite(PIN_STBY, HIGH);
  drivers_enabled = true;
  for (int step = 1; step <= 30; ++step) {
    for (int axis = 0; axis < AXIS_COUNT; ++axis) {
      float position;
      if (!sample_axis(axis, position)) return false;
      AxisState &state = axis_state[axis];
      state.position = position;
      state.previous_position = position;
      previous_position[axis] = position;
      set_field(axis, start_field[axis],
                DRIVE_AMPLITUDE[axis] * float(step) / 30.0f);
    }
    delay(5);
  }

  // The parallel mechanism can relax by several mechanical degrees while all
  // drivers are disabled between finding the stops and starting this retract.
  // Begin from each measured calibrated position instead of assuming every axis
  // is still exactly at zero. This avoids both a multi-step snap and a false
  // tracking fault at the start of an otherwise valid retract.
  float maximum_travel_deg = 0.0f;
  for (int axis = 0; axis < AXIS_COUNT; ++axis) {
    AxisState &state = axis_state[axis];
    float position;
    if (!sample_axis(axis, position)) return false;
    state.position = position;
    state.previous_position = position;
    previous_position[axis] = position;
    start_position_deg[axis] = position * RAD_TO_DEG_F;
    start_field[axis] = closed_loop_field(state, position);
    set_field(axis, start_field[axis], DRIVE_AMPLITUDE[axis]);
    maximum_travel_deg = max(maximum_travel_deg,
                             fabsf(target_deg - start_position_deg[axis]));
  }

  const uint32_t duration_ms = max(
      uint32_t(1), uint32_t(maximum_travel_deg / speed_deg_s * 1000.0f));
  const uint32_t started = millis();
  int last_report = -1;
  while (true) {
    if (abort_requested()) return false;
    const uint32_t elapsed_ms = millis() - started;
    const float fraction = constrain(float(elapsed_ms) / float(duration_ms),
                                     0.0f, 1.0f);
    const float smooth = fraction * fraction * (3.0f - 2.0f * fraction);
    float command_deg[AXIS_COUNT];

    for (int axis = 0; axis < AXIS_COUNT; ++axis) {
      AxisState &state = axis_state[axis];
      command_deg[axis] = start_position_deg[axis] +
          (target_deg - start_position_deg[axis]) * smooth;
      const float commanded_field = state.home_field + state.direction *
          command_deg[axis] * DEG_TO_RAD_F * POLE_PAIRS;
      set_field(axis, commanded_field, DRIVE_AMPLITUDE[axis]);
      float position;
      if (!sample_axis(axis, position)) return false;
      const float jump_deg =
          fabsf((position - previous_position[axis]) * RAD_TO_DEG_F);
      previous_position[axis] = position;
      state.position = position;
      state.previous_position = position;
      if (jump_deg > MAX_RETRACT_JUMP_DEG) {
        char message[96];
        snprintf(message, sizeof(message),
                 "axis %d encoder jump %.3f deg during home retract", axis,
                 jump_deg);
        latch_fault(message);
        return false;
      }
      const float lag_deg =
          fabsf(command_deg[axis] - position * RAD_TO_DEG_F);
      lag_samples[axis] = lag_deg > MAX_TRACKING_LAG_DEG
                              ? lag_samples[axis] + 1
                              : 0;
      if (lag_samples[axis] >= MAX_LAG_SAMPLES) {
        char message[96];
        snprintf(message, sizeof(message),
                 "axis %d lost tracking during home retract (lag %.2f deg)",
                 axis, lag_deg);
        latch_fault(message);
        return false;
      }
    }

    const int report = int(fraction * 10.0f);
    if (report != last_report) {
      last_report = report;
      serialf("I)Home retract %.0f%%; command %.2f %.2f %.2f deg; "
              "encoder %.2f %.2f %.2f deg\n",
              fraction * 100.0f, command_deg[0], command_deg[1], command_deg[2],
              axis_state[0].position * RAD_TO_DEG_F,
              axis_state[1].position * RAD_TO_DEG_F,
              axis_state[2].position * RAD_TO_DEG_F);
    }
    if (fraction >= 1.0f) break;
    delayMicroseconds(500);
  }

  // Estimate the local commutation reference from BOTH directions. The loaded
  // end of a one-way retract contains static friction and is not a neutral
  // rotor/field alignment. Circular averaging removes electrical wrap effects.
  float applied_field[AXIS_COUNT];
  for (int axis = 0; axis < AXIS_COUNT; ++axis)
    applied_field[axis] = axis_state[axis].home_field +
        axis_state[axis].direction * target_deg * DEG_TO_RAD_F * POLE_PAIRS;
  for (int axis = 0; axis < AXIS_COUNT; ++axis) {
    AxisState &state = axis_state[axis];
    float sum_sin = 0, sum_cos = 0;
    float low_position = 0, high_position = 0;
    float from = target_deg;
    const float endpoints[] = {target_deg - 2.0f, target_deg + 2.0f,
                               target_deg - 2.0f, target_deg};
    for (int pass = 0; pass < 4; ++pass) {
      const float to = endpoints[pass];
      const uint32_t duration = uint32_t(fabsf(to - from) / 3.0f * 1000);
      const uint32_t started = millis();
      while (true) {
        if (abort_requested()) return false;
        const float fraction = constrain(float(millis() - started) / duration, 0.0f, 1.0f);
        const float command_deg = from + (to - from) * fraction;
        const float field = state.home_field + state.direction * command_deg * DEG_TO_RAD_F * POLE_PAIRS;
        set_field(axis, field, DRIVE_AMPLITUDE[axis]);
        for (int j = 0; j < AXIS_COUNT; ++j) {
          float position;
          if (!sample_axis(j, position)) return false;
          if (fabsf(position - axis_state[j].position) > 0.75f * DEG_TO_RAD_F) {
            latch_fault("encoder jump during bidirectional phase reference");
            return false;
          }
          axis_state[j].position = position;
          axis_state[j].previous_position = position;
        }
        if ((pass == 1 || pass == 2) && fraction > .15f && fraction < .85f) {
          const float residual = (command_deg * DEG_TO_RAD_F - state.position) * POLE_PAIRS;
          sum_sin += sinf(residual);
          sum_cos += cosf(residual);
        }
        if (fraction >= 1) break;
        delay(1);
      }
      if (pass == 1) high_position = state.position;
      if (pass == 2) low_position = state.position;
      from = to;
    }
    if (high_position - low_position < 0.5f * DEG_TO_RAD_F) {
      latch_fault("phase reference sweep did not move joint");
      return false;
    }
    const float offset = atan2f(sum_sin, sum_cos);
    state.home_field += state.direction * offset;
    serialf("I)Axis %d bidirectional phase correction %.3f rad, span %.3f deg\n",
            axis, offset, (high_position - low_position) * RAD_TO_DEG_F);
  }
  // Hand off the actual applied field as torque state, without baking the
  // one-way endpoint's friction into the newly measured neutral reference.
  for (int axis = 0; axis < AXIS_COUNT; ++axis) {
    AxisState &state = axis_state[axis];
    const float initial_phase = state.direction * (applied_field[axis] - state.home_field) -
                                state.position * POLE_PAIRS;
    state.target = target_deg * DEG_TO_RAD_F;
    state.previous_position = state.position;
    state.velocity = 0.0f;
    state.integral_error = 0.0f;
    state.phase_lead = 0.0f;
    state.servo.reset(initial_phase);
  }
  last_servo_us = micros();

  const uint32_t settle_started = millis();
  uint32_t previous_us = micros();
  while (millis() - settle_started < 500) {
    const uint32_t now_us = micros();
    if (now_us - previous_us >= SERVO_PERIOD_US) {
      const float dt = float(now_us - previous_us) * 1e-6f;
      previous_us = now_us;
      servo_tick(constrain(dt, 0.0005f, 0.01f));
      if (!drivers_enabled) return false;
    }
  }

  bool accurate = true;
  for (int axis = 0; axis < AXIS_COUNT; ++axis) {
    const float error_deg =
        (axis_state[axis].target - axis_state[axis].position) * RAD_TO_DEG_F;
    serialf("I)Axis %d retract error %.3f deg\n", axis, error_deg);
    accurate &= fabsf(error_deg) < 0.75f;
  }
  if (!accurate) latch_fault("home retract did not settle within 0.75 deg");
  return accurate;
}

bool home_all_parallel() {
  constexpr float MIN_WINDOW_MOVE_DEG = 0.04f;
  constexpr float MIN_TRAVEL_DEG = 0.25f;
  constexpr uint32_t WINDOW_MS = 200;
  constexpr int REQUIRED_STALL_WINDOWS = 3;
  constexpr float HOLD_PHASE = PI_F / 4.0f;

  float commutation_offset[AXIS_COUNT];
  float direction[AXIS_COUNT];

  // Learn one motor at a time at low amplitude.  The non-driven encoders are still
  // sampled, so the phase offsets remain expressed in a common unwrapped frame even
  // when the parallel linkage back-drives them.
  for (int axis = 0; axis < AXIS_COUNT; ++axis) {
    float position = 0.0f;
    float field = 0.0f;
    if (!encoder[axis].connected() ||
        !learn_commutation(axis, position, field, direction[axis])) {
      disable_drivers();
      return false;
    }
    commutation_offset[axis] = field - direction[axis] * position * POLE_PAIRS;
  }

  disable_drivers();
  float position[AXIS_COUNT];
  float previous_position[AXIS_COUNT];
  float start_position[AXIS_COUNT];
  float window_position[AXIS_COUNT];
  float filtered_velocity[AXIS_COUNT] = {};
  int stalled_windows[AXIS_COUNT] = {};
  bool homed[AXIS_COUNT] = {};

  for (int axis = 0; axis < AXIS_COUNT; ++axis) {
    position[axis] = read_geometric_position(axis);
    previous_position[axis] = position[axis];
    start_position[axis] = position[axis];
    window_position[axis] = position[axis];
    const float neutral = direction[axis] * position[axis] * POLE_PAIRS +
                          commutation_offset[axis];
    set_field(axis, neutral, 0.0f);
  }
  digitalWrite(PIN_STBY, HIGH);
  drivers_enabled = true;
  for (int step = 1; step <= 20; ++step) {
    const float ramp_amplitude = HOME_AMPLITUDE * float(step) / 20.0f;
    for (int axis = 0; axis < AXIS_COUNT; ++axis) {
      position[axis] = read_geometric_position(axis);
      previous_position[axis] = position[axis];
      const float neutral = direction[axis] * position[axis] * POLE_PAIRS +
                            commutation_offset[axis];
      set_field(axis, neutral, ramp_amplitude);
    }
    delay(5);
  }
  for (int axis = 0; axis < AXIS_COUNT; ++axis) {
    start_position[axis] = position[axis];
    window_position[axis] = position[axis];
  }

  Serial.println("I)Parallel encoder-locked homing started");
  const uint32_t start_ms = millis();
  uint32_t previous_us = micros();
  uint32_t next_window_ms = WINDOW_MS;
  int last_report = -1;

  while (true) {
    if (abort_requested()) return false;
    const uint32_t elapsed_ms = millis() - start_ms;
    const float commanded_deg = HOME_SPEED_DPS * float(elapsed_ms) * 0.001f;
    if (commanded_deg >= HOME_SEARCH_DEG) {
      disable_drivers();
      serialf("E)Parallel home did not confirm all axes within %.0f deg\n",
              HOME_SEARCH_DEG);
      return false;
    }
    const uint32_t now_us = micros();
    float dt = float(now_us - previous_us) * 1e-6f;
    previous_us = now_us;
    dt = constrain(dt, 0.00005f, 0.02f);

    for (int axis = 0; axis < AXIS_COUNT; ++axis) {
      const int32_t raw = encoder[axis].read_raw();
      const uint8_t status = encoder[axis].status();
      position[axis] = float(raw) * RAW_TO_ROTOR_RAD;
      const float jump_deg =
          fabsf((position[axis] - previous_position[axis]) * RAD_TO_DEG_F);
      const float velocity = (position[axis] - previous_position[axis]) / dt;
      previous_position[axis] = position[axis];
      filtered_velocity[axis] += 0.08f * (velocity - filtered_velocity[axis]);
      if (status != 0 || jump_deg > 1.0f) {
        disable_drivers();
        serialf("E)Axis %d encoder fault during parallel home; status=0x%02X, "
                "jump=%.3f deg\n", axis, status, jump_deg);
        return false;
      }

      float phase_lead;
      if (homed[axis]) {
        phase_lead = -HOLD_PHASE;
      } else {
        const float target =
            start_position[axis] - commanded_deg * DEG_TO_RAD_F;
        const float error = target - position[axis];
        phase_lead = constrain(
            POSITION_KP * error - VELOCITY_DAMPING * filtered_velocity[axis],
            -MAX_PHASE_LEAD, MAX_PHASE_LEAD);
      }
      const float field = direction[axis] * position[axis] * POLE_PAIRS +
                          commutation_offset[axis] + direction[axis] * phase_lead;
      set_field(axis, field, HOME_AMPLITUDE);
    }

    if (elapsed_ms >= next_window_ms) {
      next_window_ms += WINDOW_MS;
      for (int axis = 0; axis < AXIS_COUNT; ++axis) {
        if (homed[axis]) continue;
        const float window_move =
            fabsf((position[axis] - window_position[axis]) * RAD_TO_DEG_F);
        const float total_travel =
            fabsf((position[axis] - start_position[axis]) * RAD_TO_DEG_F);
        window_position[axis] = position[axis];
        const float target =
            start_position[axis] - commanded_deg * DEG_TO_RAD_F;
        const float error = target - position[axis];
        const bool saturated =
            fabsf(POSITION_KP * error) >= 0.90f * MAX_PHASE_LEAD;
        stalled_windows[axis] =
            saturated && total_travel >= MIN_TRAVEL_DEG &&
                    window_move < MIN_WINDOW_MOVE_DEG
                ? stalled_windows[axis] + 1
                : 0;
        if (stalled_windows[axis] >= REQUIRED_STALL_WINDOWS) {
          AxisState &state = axis_state[axis];
          state.home_raw = encoder[axis].read_raw();
          state.last_raw = state.home_raw;
          state.home_field = direction[axis] * position[axis] * POLE_PAIRS +
                             commutation_offset[axis];
          state.direction = direction[axis];
          state.geometric_position = 0.0f;
          state.position = 0.0f;
          state.previous_position = 0.0f;
          state.target = 0.0f;
          state.velocity = 0.0f;
          state.integral_error = 0.0f;
          state.homed = true;
          homed[axis] = true;
          serialf("I)Axis %d parallel home confirmed at %.2f deg command, "
                  "encoder travel %.3f deg\n", axis, commanded_deg,
                  (position[axis] - start_position[axis]) * RAD_TO_DEG_F);
        }
      }

      bool finished = true;
      for (bool value : homed) finished &= value;
      if (finished) {
        if (!calibration.home_reference_after_backoff) {
          // Preserve the coordinate convention of an older embedded LUT if a new
          // calibration was aborted and the GUI restored the previous header.
          disable_drivers();
          return true;
        }
        // The calibration sweep starts after this same backoff. Move all axes
        // together so the parallel linkage is not constrained by two braked motors.
        constexpr uint32_t BACKOFF_MS = 100;
        int32_t stop_raw[AXIS_COUNT];
        for (int axis = 0; axis < AXIS_COUNT; ++axis)
          stop_raw[axis] = encoder[axis].read_raw();
        const uint32_t backoff_started = millis();
        while (millis() - backoff_started < BACKOFF_MS) {
          if (abort_requested()) return false;
          const float fraction = constrain(
              float(millis() - backoff_started) / float(BACKOFF_MS), 0.0f, 1.0f);
          for (int axis = 0; axis < AXIS_COUNT; ++axis) {
            set_field(axis,
                      axis_state[axis].home_field + direction[axis] *
                          HOME_REFERENCE_BACKOFF_FIELD_RAD * fraction,
                      HOME_AMPLITUDE);
            encoder[axis].read_raw();
            if (encoder[axis].status() != 0) {
              disable_drivers();
              serialf("E)Axis %d encoder fault during home backoff\n", axis);
              return false;
            }
          }
          delayMicroseconds(500);
        }
        delay(80);
        for (int axis = 0; axis < AXIS_COUNT; ++axis) {
          AxisState &state = axis_state[axis];
          const int32_t retracted_raw = encoder[axis].read_raw();
          const float retracted_deg =
              float(retracted_raw - stop_raw[axis]) * RAW_TO_ROTOR_RAD *
              RAD_TO_DEG_F;
          if (retracted_deg < 0.5f || retracted_deg > 3.0f) {
            disable_drivers();
            serialf("E)Axis %d home backoff was %.3f deg; expected about +1.8 deg\n",
                    axis, retracted_deg);
            return false;
          }
          state.home_field +=
              direction[axis] * HOME_REFERENCE_BACKOFF_FIELD_RAD;
          state.home_raw = retracted_raw;
          state.last_raw = state.home_raw;
          state.geometric_position = 0.0f;
          state.position = 0.0f;
          state.previous_position = 0.0f;
          state.target = 0.0f;
          state.velocity = 0.0f;
          state.integral_error = 0.0f;
        }
        Serial.println("I)Home reference backed off 1.8 deg from mechanical stops");
        disable_drivers();
        return true;
      }
      const int report = int(commanded_deg / 5.0f);
      if (report != last_report) {
        last_report = report;
        serialf("I)Parallel home command %.1f deg; travel %.2f %.2f %.2f deg\n",
                commanded_deg,
                (position[0] - start_position[0]) * RAD_TO_DEG_F,
                (position[1] - start_position[1]) * RAD_TO_DEG_F,
                (position[2] - start_position[2]) * RAD_TO_DEG_F);
      }
    }
    delayMicroseconds(500);
  }
}

bool home_selected(uint8_t mask) {
  disable_drivers();
  move.active = false;
  fault_latched = false;
  fault_text[0] = '\0';
  if (mask != 0x07) {
    Serial.println("E)STM32 parallel mechanism currently requires homing all axes together");
    return false;
  }
  for (auto &state : axis_state) state.homed = false;
  if (!home_all_parallel()) return false;
  if (!retract_all_from_home_open_loop(HOME_RETRACT_DEG, 5.0f)) return false;
  float joints[AXIS_COUNT];
  for (int axis = 0; axis < AXIS_COUNT; ++axis) joints[axis] = axis_state[axis].position;
  if (!kinematics.foreward(joints, current_pose)) {
    latch_fault("forward kinematics failed after homing");
    return false;
  }
  return true;
}

bool parse_word(const char *line, char word, float &value) {
  const char *cursor = line;
  while (*cursor) {
    while (*cursor == ' ' || *cursor == '\t') ++cursor;
    if (*cursor == word) {
      char *end = nullptr;
      const float parsed = strtof(cursor + 1, &end);
      if (end != cursor + 1) {
        value = parsed;
        return true;
      }
    }
    while (*cursor && *cursor != ' ' && *cursor != '\t') ++cursor;
  }
  return false;
}

bool has_word_token(const char *line, char word) {
  const char *cursor = line;
  while (*cursor) {
    while (*cursor == ' ' || *cursor == '\t') ++cursor;
    if (*cursor == word) return true;
    while (*cursor && *cursor != ' ' && *cursor != '\t') ++cursor;
  }
  return false;
}

bool command_is(const char *line, const char *command) {
  const size_t length = strlen(command);
  return !strncmp(line, command, length) &&
         (line[length] == '\0' || line[length] == ' ' || line[length] == '\t');
}

bool cartesian_path_is_safe(const Pose6DF &start, const Pose6DF &end) {
  for (int sample = 0; sample <= 20; ++sample) {
    const float fraction = float(sample) / 20.0f;
    const Pose6DF pose = Pose6DF::lerp(start, end, fraction);
    float targets[AXIS_COUNT];
    if (!kinematics.inverse(pose, targets) || !targets_within_limits(targets, false))
      return false;
  }
  return true;
}

void execute_command(char *line) {
  while (*line == ' ' || *line == '\t') ++line;
  for (char *p = line; *p; ++p)
    if (*p >= 'a' && *p <= 'z') *p = char(*p - 'a' + 'A');
  if (!*line) return;

  if (command_is(line, "M18")) {
    disable_drivers();
    move.active = false;
    Serial.println("ok");
    return;
  }
  if (command_is(line, "M0")) {
    // Cancel travel while retaining the current interpolated holding target.
    move.active = false;
    move.end = current_pose;
    Serial.println("ok");
    return;
  }
  if (command_is(line, "M58")) {
    Serial.println(FIRMWARE_VERSION);
    Serial.println("ok");
    return;
  }
  if (command_is(line, "G28")) {
    uint8_t mask = 0;
    if (has_word_token(line, 'A')) mask |= 1u << 0;
    if (has_word_token(line, 'B')) mask |= 1u << 1;
    if (has_word_token(line, 'C')) mask |= 1u << 2;
    if (!mask) mask = 0x07;
    Serial.println(home_selected(mask) ? "ok" : "error: guarded homing failed");
    return;
  }
  if (command_is(line, "M17")) {
    if (fault_latched) {
      Serial.println("error: safety fault latched; run G28");
    } else if (!all_homed()) {
      Serial.println("error: axes are not homed; run G28");
    } else if (drivers_enabled || enable_closed_loop()) {
      float joints[AXIS_COUNT];
      for (int axis = 0; axis < AXIS_COUNT; ++axis)
        joints[axis] = axis_state[axis].position;
      if (kinematics.foreward(joints, current_pose)) Serial.println("ok");
      else {
        latch_fault("forward kinematics failed while enabling motors");
        Serial.println("error: could not reconstruct pose");
      }
    } else {
      Serial.println("error: could not enable closed loop");
    }
    return;
  }
  if (command_is(line, "M50")) {
    Pose6DF measured = current_pose;
    if (all_homed() && drivers_enabled) {
      float joints[AXIS_COUNT];
      for (int axis = 0; axis < AXIS_COUNT; ++axis)
        joints[axis] = axis_state[axis].position;
      kinematics.foreward(joints, measured);
    }
    serialf("X%.6f Y%.6f Z%.6f\n", measured.translation.x,
            measured.translation.y, measured.translation.z);
    const Pose6DF &destination = move.active ? move.end : current_pose;
    serialf("Destination: X%.6f Y%.6f Z%.6f\n", destination.translation.x,
            destination.translation.y, destination.translation.z);
    Serial.println("ok");
    return;
  }
  if (command_is(line, "M51")) {
    for (int axis = 0; axis < AXIS_COUNT; ++axis) {
      const int32_t raw = encoder[axis].read_raw();
      const float position = axis_state[axis].homed
                                 ? evaluate_calibration_deg(axis, raw - axis_state[axis].home_raw)
                                 : float(raw) * RAW_TO_ROTOR_RAD * RAD_TO_DEG_F;
      serialf("Joint %d: %.6f deg (raw=%ld) status=0x%02X\n", axis, position,
              long(raw), encoder[axis].status());
    }
    Serial.println("ok");
    return;
  }
  if (command_is(line, "M52")) {
    Serial.println(move.active ? "1" : "0");
    Serial.println("ok");
    return;
  }
  if (command_is(line, "M53")) {
    Serial.println(move.active ? "0" : "1");
    Serial.println("ok");
    return;
  }
  if (command_is(line, "M57")) {
    serialf("Controller: STM32F401 calibrated closed loop, drivers=%d, fault=%d\n",
            drivers_enabled, fault_latched);
    if (fault_latched) serialf("Fault: %s\n", fault_text);
    for (int axis = 0; axis < AXIS_COUNT; ++axis) {
      serialf("Joint %d: is_homed=%d, is_calibrated=1, angle=%.4f deg, "
              "target=%.4f deg, error=%.4f deg, phase=%.3f rad, drive=%.2f, limit=%.3f deg, "
              "raw_delta=%ld, direction=%.0f, enc_status=%u, "
              "crc_retries=%lu, crc_failures=%lu\n",
              axis, axis_state[axis].homed,
              axis_state[axis].position * RAD_TO_DEG_F,
              axis_state[axis].target * RAD_TO_DEG_F,
              (axis_state[axis].target - axis_state[axis].position) * RAD_TO_DEG_F,
              axis_state[axis].phase_lead,
              DRIVE_AMPLITUDE[axis],
              calibration.axis[axis].max_deg,
              long(axis_state[axis].last_raw - axis_state[axis].home_raw),
              axis_state[axis].direction, encoder[axis].status(),
              static_cast<unsigned long>(encoder[axis].crc_retries()),
              static_cast<unsigned long>(encoder[axis].crc_failures()));
      serialf("  motion_diag: last_jump=%+.4f deg, peak_jump=%.4f deg, "
              "peak_error=%.4f deg, peak_velocity=%.3f deg/s, "
              "peak_phase=%.3f rad, saturated=%lu/%lu ticks\n",
              axis_state[axis].last_sample_jump,
              axis_state[axis].peak_sample_jump,
              axis_state[axis].peak_abs_error * RAD_TO_DEG_F,
              axis_state[axis].peak_abs_velocity * RAD_TO_DEG_F,
              axis_state[axis].peak_abs_phase,
              static_cast<unsigned long>(axis_state[axis].saturated_ticks),
              static_cast<unsigned long>(axis_state[axis].diagnostic_ticks));
    }
    serialf("Tool[0] output: %.3f\nTool[1] output: %.3f\n", tool_value[0],
            tool_value[1]);
    serialf("Calibration: %s (%s)\n",
            calibration.home_reference_after_backoff
                ? "original-method STM32 adaptation"
                : "legacy guarded sweep",
            calibration_loaded_from_flash ? "internal flash" : "embedded fallback");
    Serial.println("ok");
    return;
  }
  if (command_is(line, "M3")) {
    float tool = 0.0f, value = 0.0f;
    parse_word(line, 'T', tool);
    parse_word(line, 'S', value);
    const int index = int(tool);
    if (index < 0 || index >= 2 || value < 0.0f || value > 1.0f)
      Serial.println("error: use M3 T<0|1> S<0..1>");
    else {
      set_tool(index, value);
      Serial.println("ok");
    }
    return;
  }
  if (command_is(line, "M56")) {
    run_onboard_calibration();
    return;
  }
  if (command_is(line, "M204")) {
    Serial.println("ok");
    return;
  }
  if (command_is(line, "M55")) {
    Serial.println("error: live servo retuning is not supported by the STM32 controller");
    return;
  }
  if (command_is(line, "G4")) {
    if (move.active) {
      Serial.println("busy");
      return;
    }
    float seconds = 0.0f;
    float milliseconds = 0.0f;
    parse_word(line, 'S', seconds);
    parse_word(line, 'P', milliseconds);
    const float duration_ms = max(0.0f, seconds * 1000.0f + milliseconds);
    move.start = current_pose;
    move.end = current_pose;
    move.start_ms = millis();
    move.duration_ms = uint32_t(duration_ms);
    move.active = move.duration_ms > 0;
    Serial.println("ok");
    return;
  }
  if (command_is(line, "G24")) {
    if (!drivers_enabled) {
      Serial.println("error: motors are not enabled");
      return;
    }
    Pose6DF requested = current_pose;
    parse_word(line, 'X', requested.translation.x);
    parse_word(line, 'Y', requested.translation.y);
    parse_word(line, 'Z', requested.translation.z);
    float targets[AXIS_COUNT];
    if (!kinematics.inverse(requested, targets) || !targets_within_limits(targets, true))
      return;
    for (int axis = 0; axis < AXIS_COUNT; ++axis) axis_state[axis].target = targets[axis];
    move.active = false;
    current_pose = requested;
    Serial.println("ok");
    return;
  }
  if (command_is(line, "G0") || command_is(line, "G1")) {
    if (!drivers_enabled || fault_latched) {
      Serial.println("error: home and enable the motors first");
      return;
    }
    // Unspecified axes retain the destination; replan from the current
    // interpolated setpoint so replacing a target never jumps position.
    Pose6DF requested = move.active ? move.end : current_pose;
    parse_word(line, 'X', requested.translation.x);
    parse_word(line, 'Y', requested.translation.y);
    parse_word(line, 'Z', requested.translation.z);
    float feed = default_feed_mm_s;
    parse_word(line, 'F', feed);
    if (!isfinite(feed) || feed <= 0.0f || feed > 2.0f ||
        !isfinite(requested.translation.x) ||
        !isfinite(requested.translation.y) ||
        !isfinite(requested.translation.z)) {
      Serial.println("error: finite coordinates and feed 0 < F <= 2 mm/s required");
      return;
    }
    if (!cartesian_path_is_safe(current_pose, requested)) {
      Serial.println("error: requested path leaves calibrated joint workspace");
      return;
    }
    const Vec3F delta = requested.translation - current_pose.translation;
    const float distance = delta.length();
    default_feed_mm_s = feed;
    move.start = current_pose;
    move.end = requested;
    move.start_ms = millis();
    // Cubic smoothstep peaks at 1.5*distance/time and 6*distance/time^2.
    // Bound acceleration to 5 mm/s^2, including short jogs and retargets.
    const float speed_time = max(1.5f * distance / default_feed_mm_s,
                                 1.5f * fabsf(delta.y) / 1.0f);
    move.duration_ms = max(uint32_t(100), uint32_t(ceilf(1000.0f * max(
        speed_time, sqrtf(6.0f * distance / 5.0f)))));
    move.active = distance > 1e-6f;
    reset_motion_diagnostics();
    if (!move.active) current_pose = requested;
    Serial.println("ok");
    return;
  }
  Serial.println("error: unknown command");
}

}  // namespace

void setup() {
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, LOW);
  initialize_motor_pwm();
  disable_drivers();
  initialize_tool_pwm();
  set_tool(0, 0.0f);
  set_tool(1, 0.0f);

  for (auto &item : encoder) item.begin_gpio();
  encoder_spi.begin();
  for (auto &item : encoder) item.read_raw();
  load_calibration();

  Serial.begin(921600);
  const uint32_t wait_start = millis();
  while (!Serial && millis() - wait_start < 2000) {}
  Serial.println();
  serialf("I)Open Micro Stage STM32F401 controller %s\n", FIRMWARE_VERSION);
  Serial.println(calibration_loaded_from_flash
                     ? "I)Calibration loaded from internal flash"
                     : "I)Using embedded fallback calibration");
  Serial.println("I)Run G28 before motion. Send any character during homing to abort.");
  last_servo_us = micros();
}

void loop() {
  const uint32_t now_us = micros();
  if (now_us - last_servo_us >= SERVO_PERIOD_US) {
    const float dt = constrain(float(now_us - last_servo_us) * 1e-6f, 0.0005f, 0.01f);
    last_servo_us = now_us;
    servo_tick(dt);
  }

  while (Serial.available()) {
    const char c = char(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      command_line[command_length] = '\0';
      execute_command(command_line);
      command_length = 0;
    } else if (command_length < sizeof(command_line) - 1) {
      command_line[command_length++] = c;
    }
  }
}
