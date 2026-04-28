#include "rs01_motor/rs01_motor.h"

#include "rs01_motor/protocol.h"

#include <algorithm>
#include <cstring>
#include <stdexcept>

#include <linux/can.h>

namespace rs01 {

Rs01Motor::Rs01Motor(std::string can_interface, uint8_t motor_id,
                     uint8_t host_id)
    : socket_(std::move(can_interface)), motor_id_(motor_id), host_id_(host_id) {}

void Rs01Motor::enable() {
  send_empty(comm::kEnable);
  expect_status();
}

void Rs01Motor::disable(bool clear_error) {
  uint8_t data[8] {};
  data[0] = clear_error ? 1 : 0;
  socket_.send_extended(make_can_id(comm::kDisable, host_id_, motor_id_), data,
                        8);
  expect_status();
}

void Rs01Motor::set_zero() {
  uint8_t data[8] {};
  data[0] = 1;
  socket_.send_extended(make_can_id(comm::kSetZero, host_id_, motor_id_), data,
                        8);
  expect_status();
}

std::optional<Feedback> Rs01Motor::read_feedback(int timeout_ms) {
  can_frame frame {};
  if (!socket_.receive(frame, timeout_ms)) {
    return std::nullopt;
  }
  if ((frame.can_id & CAN_EFF_FLAG) == 0) {
    return std::nullopt;
  }

  uint32_t can_id = frame.can_id & CAN_EFF_MASK;
  uint8_t communication_type = static_cast<uint8_t>((can_id >> 24) & 0x1F);
  uint16_t extra_data = static_cast<uint16_t>((can_id >> 8) & 0xFFFF);

  if (communication_type != comm::kFeedback &&
      communication_type != comm::kFaultReport) {
    return std::nullopt;
  }
  if (frame.can_dlc < 8) {
    throw std::runtime_error("feedback frame is shorter than 8 bytes");
  }
  if (communication_type == comm::kFaultReport) {
    throw std::runtime_error("received RS01 fault report frame");
  }

  uint16_t position_u16 =
      (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
  uint16_t velocity_u16 =
      (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3];
  uint16_t torque_u16 =
      (static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5];
  uint16_t temperature_u16 =
      (static_cast<uint16_t>(frame.data[6]) << 8) | frame.data[7];

  Feedback feedback;
  feedback.position = uint_to_float(position_u16, -kPositionMax, kPositionMax);
  feedback.velocity = uint_to_float(velocity_u16, -kVelocityMax, kVelocityMax);
  feedback.torque = uint_to_float(torque_u16, -kTorqueMax, kTorqueMax);
  feedback.temperature = static_cast<float>(temperature_u16) * 0.1f;
  feedback.mode = static_cast<uint8_t>((extra_data >> 14) & 0x03);
  feedback.fault = static_cast<uint8_t>((extra_data >> 8) & 0x3F);
  return feedback;
}

void Rs01Motor::write_param_u8(uint16_t index, uint8_t value) {
  uint8_t data[8] {};
  pack_u16_le(&data[0], index);
  data[4] = value;
  socket_.send_extended(make_can_id(comm::kWriteParameter, host_id_, motor_id_),
                        data, 8);
  expect_status();
}

void Rs01Motor::write_param_float(uint16_t index, float value) {
  uint8_t data[8] {};
  pack_u16_le(&data[0], index);
  pack_float_le(&data[4], value);
  socket_.send_extended(make_can_id(comm::kWriteParameter, host_id_, motor_id_),
                        data, 8);
  expect_status();
}

std::optional<float> Rs01Motor::read_param_float(uint16_t index,
                                                 int timeout_ms) {
  uint8_t data[8] {};
  pack_u16_le(&data[0], index);
  socket_.send_extended(make_can_id(comm::kReadParameter, host_id_, motor_id_),
                        data, 8);

  can_frame frame {};
  if (!socket_.receive(frame, timeout_ms) || frame.can_dlc < 8) {
    return std::nullopt;
  }

  uint32_t can_id = frame.can_id & CAN_EFF_MASK;
  if (((can_id >> 24) & 0x1F) != comm::kReadParameter) {
    return std::nullopt;
  }

  float value = 0.0f;
  std::memcpy(&value, &frame.data[4], sizeof(float));
  return value;
}

std::optional<uint8_t> Rs01Motor::read_param_u8(uint16_t index,
                                                int timeout_ms) {
  uint8_t data[8] {};
  pack_u16_le(&data[0], index);
  socket_.send_extended(make_can_id(comm::kReadParameter, host_id_, motor_id_),
                        data, 8);

  can_frame frame {};
  if (!socket_.receive(frame, timeout_ms) || frame.can_dlc < 8) {
    return std::nullopt;
  }

  uint32_t can_id = frame.can_id & CAN_EFF_MASK;
  if (((can_id >> 24) & 0x1F) != comm::kReadParameter) {
    return std::nullopt;
  }
  return frame.data[4];
}

void Rs01Motor::set_mode(uint8_t mode_value) {
  disable(false);
  write_param_u8(param::kRunMode, mode_value);
}

void Rs01Motor::motion_control(float torque, float position, float velocity,
                               float kp, float kd) {
  torque = clamp(torque, -kTorqueMax, kTorqueMax);
  position = clamp(position, -kPositionMax, kPositionMax);
  velocity = clamp(velocity, -kVelocityMax, kVelocityMax);
  kp = clamp(kp, 0.0f, kKpMax);
  kd = clamp(kd, 0.0f, kKdMax);

  uint16_t torque_u16 = float_to_uint(torque, -kTorqueMax, kTorqueMax);
  uint16_t position_u16 = float_to_uint(position, -kPositionMax, kPositionMax);
  uint16_t velocity_u16 = float_to_uint(velocity, -kVelocityMax, kVelocityMax);
  uint16_t kp_u16 = float_to_uint(kp, 0.0f, kKpMax);
  uint16_t kd_u16 = float_to_uint(kd, 0.0f, kKdMax);

  uint8_t data[8] {};
  pack_u16_be(&data[0], position_u16);
  pack_u16_be(&data[2], velocity_u16);
  pack_u16_be(&data[4], kp_u16);
  pack_u16_be(&data[6], kd_u16);

  socket_.send_extended(make_can_id(comm::kMotionControl, torque_u16, motor_id_),
                        data, 8);
}

void Rs01Motor::velocity_control(float velocity, float current_limit,
                                 float acceleration) {
  velocity = clamp(velocity, -kVelocityMax, kVelocityMax);
  current_limit = clamp(current_limit, 0.0f, kCurrentMax);
  acceleration = std::max(0.0f, acceleration);

  set_mode(mode::kVelocity);
  write_param_float(param::kLimitCurrent, current_limit);
  write_param_float(param::kVelocityAcceleration, acceleration);
  enable();
  write_param_float(param::kSpeedRef, velocity);
}

float Rs01Motor::clamp(float value, float low, float high) {
  return std::max(low, std::min(high, value));
}

uint16_t Rs01Motor::float_to_uint(float value, float min, float max) {
  value = clamp(value, min, max);
  return static_cast<uint16_t>(((value - min) * 65535.0f) / (max - min));
}

float Rs01Motor::uint_to_float(uint16_t value, float min, float max) {
  return static_cast<float>(value) * (max - min) / 65535.0f + min;
}

void Rs01Motor::send_empty(uint32_t communication_type) {
  uint8_t data[8] {};
  socket_.send_extended(make_can_id(communication_type, host_id_, motor_id_),
                        data, 8);
}

void Rs01Motor::expect_status() {
  if (!read_feedback(100)) {
    throw std::runtime_error("timeout waiting for RS01 status frame");
  }
}

} // namespace rs01
