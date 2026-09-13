#include "persistent_calibration.h"

#include <EEPROM.h>
#include <string.h>

namespace PersistentCalibration {
namespace {

constexpr uint32_t MAGIC = 0x314C4143u;  // "CAL1" in little-endian flash.
constexpr uint16_t FORMAT_VERSION = 1;

struct __attribute__((packed, aligned(4))) Record {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint32_t sequence;
  uint32_t flags;
  AxisLut axis[3];
  uint32_t crc32;
};

static_assert(sizeof(Record) <= 8192,
              "calibration record exceeds STM32 EEPROM-emulation page");

uint32_t crc32(const uint8_t *bytes, size_t length) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < length; ++i) {
    crc ^= bytes[i];
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

bool valid(const Record &record) {
  if (record.magic != MAGIC || record.version != FORMAT_VERSION ||
      record.size != sizeof(Record))
    return false;
  const uint32_t expected =
      crc32(reinterpret_cast<const uint8_t *>(&record),
            sizeof(Record) - sizeof(record.crc32));
  return expected == record.crc32;
}

void read_record(Record &record) {
  eeprom_buffer_fill();
  uint8_t *destination = reinterpret_cast<uint8_t *>(&record);
  for (size_t i = 0; i < sizeof(Record); ++i)
    destination[i] = eeprom_buffered_read_byte(i);
}

}  // namespace

bool load(Data &data) {
  Record record{};
  read_record(record);
  if (!valid(record)) return false;
  memcpy(data.axis, record.axis, sizeof(data.axis));
  data.home_reference_after_backoff = (record.flags & 1u) != 0;
  return true;
}

bool save(const Data &data) {
  Record previous{};
  read_record(previous);

  Record record{};
  record.magic = MAGIC;
  record.version = FORMAT_VERSION;
  record.size = sizeof(Record);
  record.sequence = valid(previous) ? previous.sequence + 1u : 1u;
  record.flags = data.home_reference_after_backoff ? 1u : 0u;
  memcpy(record.axis, data.axis, sizeof(record.axis));
  record.crc32 = crc32(reinterpret_cast<const uint8_t *>(&record),
                       sizeof(Record) - sizeof(record.crc32));

  eeprom_buffer_fill();
  const uint8_t *source = reinterpret_cast<const uint8_t *>(&record);
  for (size_t i = 0; i < sizeof(Record); ++i)
    eeprom_buffered_write_byte(i, source[i]);
  eeprom_buffer_flush();

  Record verification{};
  read_record(verification);
  return valid(verification) &&
         memcmp(&verification, &record, sizeof(Record)) == 0;
}

}  // namespace PersistentCalibration
