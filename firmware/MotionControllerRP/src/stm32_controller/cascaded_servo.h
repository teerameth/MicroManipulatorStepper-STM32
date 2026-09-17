#pragma once

// Position/velocity PI cascade adapted from the MIT-licensed Pico controller
// servo_control/servo_controller.cpp and pid.cpp (M. S., diffraction limited).
// All positions are mechanical radians; output is electrical phase radians.
namespace CascadedServo {
constexpr float PI_VALUE = 3.14159265358979323846f;
constexpr float PHASE_LIMIT = PI_VALUE * 0.45f;
// A displaced horn must regain position without requesting the Pico default's
// 360 deg/s recovery. Retain full integral torque, cap the requested joint speed.
constexpr float VELOCITY_LIMIT = 0.35f;  // mechanical rad/s (~20 deg/s)
inline float clamp(float value, float limit) {
  return value < -limit ? -limit : (value > limit ? limit : value);
}
struct PIController {
  float integral = 0;
  float previous_error = 0;
  float update(float error, float dt, float kp, float ki,
               float limit, float integral_limit) {
    integral = clamp(integral + 0.5f * ki * dt * (error + previous_error),
                     integral_limit);
    previous_error = error;
    return clamp(kp * error + integral, limit);
  }
};
struct Controller {
  PIController position_pi;
  PIController velocity_pi;
  void reset(float initial_phase = 0) {
    position_pi = PIController{};
    velocity_pi = PIController{};
    velocity_pi.integral = clamp(initial_phase, PHASE_LIMIT);
  }
  float update(float error, float velocity, float dt) {
    const float requested_velocity = position_pi.update(
        error, dt, 60.0f, 30000.0f, VELOCITY_LIMIT, VELOCITY_LIMIT);
    return velocity_pi.update(requested_velocity - velocity, dt,
                              0.2f, 90.0f, PHASE_LIMIT, PHASE_LIMIT);
  }
};
}  // namespace CascadedServo
