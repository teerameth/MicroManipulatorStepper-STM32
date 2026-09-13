#pragma once

#include <stdint.h>

namespace PersistentCalibration {

constexpr uint16_t LUT_SIZE = 256;

struct AxisLut {
  int32_t raw_max;
  float max_deg;
  float position_deg[LUT_SIZE];
};

struct Data {
  AxisLut axis[3];
  bool home_reference_after_backoff;
};

// Returns false when flash is blank, corrupt, or from an incompatible format.
bool load(Data &data);

// Erases/programs the reserved final flash sector and verifies the new record.
bool save(const Data &data);

}  // namespace PersistentCalibration
