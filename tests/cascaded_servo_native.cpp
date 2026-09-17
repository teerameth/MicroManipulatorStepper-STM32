#include "../firmware/MotionControllerRP/src/stm32_controller/cascaded_servo.h"
#include <cassert>
#include <cmath>

int main() {
  using namespace CascadedServo;
  // Polarity symmetry through a long disturbance and reversal, including
  // saturation and differing sample intervals.
  Controller positive, negative;
  for (int i = 0; i < 3000; ++i) {
    float error = i < 1000 ? .03f : (i < 2000 ? -.03f : .0001f);
    float velocity = .2f * std::sin(i * .01f);
    float dt = i % 2 ? .001f : .0013f;
    float a = positive.update(error, velocity, dt);
    float b = negative.update(-error, -velocity, dt);
    assert(std::isfinite(a) && std::fabs(a) <= PHASE_LIMIT);
    assert(std::fabs(a + b) < 1e-6f);
  }
  // A stored holding bias must yield to opposite displacement, rather than
  // permanently consuming one direction's commutation authority.
  for (int sign = -1; sign <= 1; sign += 2) {
    Controller controller;
    controller.reset(sign * PHASE_LIMIT);
    float output = 0;
    for (int i=0; i<100; ++i)
      output = controller.update(-sign*.02f, 0, .001f);
    assert(sign * output < -0.9f * PHASE_LIMIT);
    controller.reset();
    assert(controller.update(0, 0, .001f) == 0);
  }
}
