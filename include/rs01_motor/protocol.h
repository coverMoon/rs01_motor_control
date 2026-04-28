#pragma once

#include <cstdint>
#include <cstring>

namespace rs01 {

namespace comm {
constexpr uint32_t kMotionControl = 0x01;
constexpr uint32_t kFeedback = 0x02;
constexpr uint32_t kEnable = 0x03;
constexpr uint32_t kDisable = 0x04;
constexpr uint32_t kSetZero = 0x06;
constexpr uint32_t kReadParameter = 0x11;
constexpr uint32_t kWriteParameter = 0x12;
constexpr uint32_t kFaultReport = 0x15;
} // namespace comm

namespace param {
constexpr uint16_t kRunMode = 0x7005;
constexpr uint16_t kIqRef = 0x7006;
constexpr uint16_t kSpeedRef = 0x700A;
constexpr uint16_t kLimitTorque = 0x700B;
constexpr uint16_t kLocRef = 0x7016;
constexpr uint16_t kLimitSpeed = 0x7017;
constexpr uint16_t kLimitCurrent = 0x7018;
constexpr uint16_t kMechPos = 0x7019;
constexpr uint16_t kIqFiltered = 0x701A;
constexpr uint16_t kMechVel = 0x701B;
constexpr uint16_t kVbus = 0x701C;
constexpr uint16_t kVelocityKp = 0x701F;
constexpr uint16_t kVelocityKi = 0x7020;
constexpr uint16_t kVelocityAcceleration = 0x7022;
constexpr uint16_t kPpVelocityMax = 0x7024;
constexpr uint16_t kPpAcceleration = 0x7025;
constexpr uint16_t kReportPeriod = 0x7026;
} // namespace param

namespace mode {
constexpr int8_t kMotion = 0;
constexpr int8_t kPositionPp = 1;
constexpr int8_t kVelocity = 2;
constexpr int8_t kCurrent = 3;
constexpr int8_t kPositionCsp = 5;
} // namespace mode

constexpr float kPositionMax = 4.0f * 3.14159265358979323846f;
constexpr float kVelocityMax = 44.0f;
constexpr float kTorqueMax = 17.0f;
constexpr float kKpMax = 500.0f;
constexpr float kKdMax = 5.0f;
constexpr float kCurrentMax = 23.0f;

inline uint32_t make_can_id(uint32_t communication_type, uint16_t extra_data,
                            uint8_t target_id) {
  return (communication_type << 24) | (static_cast<uint32_t>(extra_data) << 8) |
         target_id;
}

inline void pack_u16_le(uint8_t *data, uint16_t value) {
  data[0] = static_cast<uint8_t>(value & 0xFF);
  data[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

inline void pack_u16_be(uint8_t *data, uint16_t value) {
  data[0] = static_cast<uint8_t>((value >> 8) & 0xFF);
  data[1] = static_cast<uint8_t>(value & 0xFF);
}

inline void pack_float_le(uint8_t *data, float value) {
  std::memcpy(data, &value, sizeof(float));
}

} // namespace rs01
