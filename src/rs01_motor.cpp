#include "rs01_motor/rs01_motor.h"

#include "rs01_motor/protocol.h"

#include <algorithm>
#include <cstring>
#include <stdexcept>

#include <linux/can.h>

namespace rs01 {

// -----------------------------------------------------------------------------
// 生命周期
// -----------------------------------------------------------------------------

/**
 * @brief 创建一个绑定到指定 CAN 接口和电机 ID 的 RS01 控制对象。
 *
 * @param can_interface SocketCAN 接口名，例如 "can0"。
 * @param motor_id 目标 RS01 电机 ID。
 * @param host_id 主机 ID，普通命令会写入 CAN ID 的 extra_data 字段。
 */
Rs01Motor::Rs01Motor(std::string can_interface, uint8_t motor_id,
                     uint8_t host_id)
    : socket_(std::move(can_interface)), motor_id_(motor_id), host_id_(host_id) {}

// -----------------------------------------------------------------------------
// 基础状态命令
// -----------------------------------------------------------------------------

/**
 * @brief 发送使能命令，并等待电机状态反馈。
 */
void Rs01Motor::enable() {
  send_empty(comm::kEnable);
  expect_status();
}

/**
 * @brief 发送失能命令，并可选择同时清除错误。
 *
 * @param clear_error true 表示请求电机清除错误后失能；false 表示只失能。
 */
void Rs01Motor::disable(bool clear_error) {
  uint8_t data[8] {};
  // 失能命令的 data[0] 可请求清除错误：0 只失能，1 表示失能并清错。
  data[0] = clear_error ? 1 : 0;
  socket_.send_extended(make_can_id(comm::kDisable, host_id_, motor_id_), data,
                        8);
  expect_status();
}

/**
 * @brief 将当前机械位置设置为零点，并等待状态反馈。
 */
void Rs01Motor::set_zero() {
  uint8_t data[8] {};
  // 设置机械零点命令需要 data[0] = 1，其他字节保持 0。
  data[0] = 1;
  socket_.send_extended(make_can_id(comm::kSetZero, host_id_, motor_id_), data,
                        8);
  expect_status();
}

// -----------------------------------------------------------------------------
// 反馈读取
// -----------------------------------------------------------------------------

/**
 * @brief 接收并解析一帧 RS01 状态反馈。
 *
 * @param timeout_ms 接收超时时间，单位毫秒。
 * @return 成功解析时返回 Feedback；超时或收到非反馈帧时返回 std::nullopt。
 * @throws std::runtime_error 收到短帧或故障上报帧时抛出。
 */
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

  // 这里只解析 type-2 状态反馈；如果收到故障上报帧，直接抛异常提醒调用方处理。
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

  // 反馈 payload 依次是位置、速度、力矩、温度，均为大端 16 bit 数据。
  // 前三个字段需要按 RS01 固定范围反向映射回实际物理量。
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
  // 反馈帧 CAN ID 的 extra_data 高位携带当前模式和 fault 位，低位含义暂未在此解析。
  feedback.mode = static_cast<uint8_t>((extra_data >> 14) & 0x03);
  feedback.fault = static_cast<uint8_t>((extra_data >> 8) & 0x3F);
  return feedback;
}

// -----------------------------------------------------------------------------
// 参数访问
// -----------------------------------------------------------------------------

/**
 * @brief 写入 uint8 类型参数。
 *
 * @param index 参数索引，来自 rs01::param。
 * @param value 要写入的 8 bit 参数值。
 */
void Rs01Motor::write_param_u8(uint16_t index, uint8_t value) {
  uint8_t data[8] {};
  // 参数写入帧格式：data[0..1] 为参数索引，data[4..7] 为参数值区域。
  // uint8 参数只使用 data[4]，剩余字节保持 0。
  pack_u16_le(&data[0], index);
  data[4] = value;
  socket_.send_extended(make_can_id(comm::kWriteParameter, host_id_, motor_id_),
                        data, 8);
  expect_status();
}

/**
 * @brief 写入 float 类型参数。
 *
 * @param index 参数索引，来自 rs01::param。
 * @param value 要写入的 float 参数值。
 */
void Rs01Motor::write_param_float(uint16_t index, float value) {
  uint8_t data[8] {};
  // float 参数完整占用 data[4..7] 四个字节，例如速度目标、限流、加速度等。
  pack_u16_le(&data[0], index);
  pack_float_le(&data[4], value);
  socket_.send_extended(make_can_id(comm::kWriteParameter, host_id_, motor_id_),
                        data, 8);
  expect_status();
}

/**
 * @brief 读取 float 类型参数。
 *
 * @param index 参数索引，来自 rs01::param。
 * @param timeout_ms 接收响应的超时时间，单位毫秒。
 * @return 成功时返回参数值；超时、短帧或响应类型不匹配时返回 std::nullopt。
 */
std::optional<float> Rs01Motor::read_param_float(uint16_t index,
                                                 int timeout_ms) {
  uint8_t data[8] {};
  // 参数读取请求只需要填写参数索引，电机会在响应帧的 value 区域返回实际值。
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

  // 参数读取响应沿用相同的 payload 布局：data[4..7] 是 float 参数值。
  float value = 0.0f;
  std::memcpy(&value, &frame.data[4], sizeof(float));
  return value;
}

/**
 * @brief 读取 uint8 类型参数。
 *
 * @param index 参数索引，来自 rs01::param。
 * @param timeout_ms 接收响应的超时时间，单位毫秒。
 * @return 成功时返回参数值；超时、短帧或响应类型不匹配时返回 std::nullopt。
 */
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

// -----------------------------------------------------------------------------
// 控制封装
// -----------------------------------------------------------------------------

/**
 * @brief 切换 RS01 运行模式。
 *
 * @param mode_value 目标模式值，通常使用 rs01::mode 中的常量。
 */
void Rs01Motor::set_mode(uint8_t mode_value) {
  // 切换运行模式前先失能，避免电机在旧控制模式下继续执行未完成的目标。
  disable(false);
  write_param_u8(param::kRunMode, mode_value);
}

/**
 * @brief 发送运控模式控制帧。
 *
 * @param torque 目标力矩，单位 Nm。
 * @param position 目标位置，单位 rad。
 * @param velocity 目标速度，单位 rad/s。
 * @param kp 位置刚度系数。
 * @param kd 速度阻尼系数。
 */
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

  // 运控帧比较特殊：力矩不在 payload 中，而是放入 CAN ID 的 extra_data 字段；
  // payload 中依次放位置、速度、Kp、Kd 四个 16 bit 映射值。
  uint8_t data[8] {};
  pack_u16_be(&data[0], position_u16);
  pack_u16_be(&data[2], velocity_u16);
  pack_u16_be(&data[4], kp_u16);
  pack_u16_be(&data[6], kd_u16);

  socket_.send_extended(make_can_id(comm::kMotionControl, torque_u16, motor_id_),
                        data, 8);
}

/**
 * @brief 执行一次速度模式配置并写入速度目标。
 *
 * @param velocity 目标速度，单位 rad/s。
 * @param current_limit 电流限制，单位 A。
 * @param acceleration 速度模式加速度限制，单位 rad/s^2。
 */
void Rs01Motor::velocity_control(float velocity, float current_limit,
                                 float acceleration) {
  velocity = clamp(velocity, -kVelocityMax, kVelocityMax);
  current_limit = clamp(current_limit, 0.0f, kCurrentMax);
  acceleration = std::max(0.0f, acceleration);

  // 速度模式的最小安全流程：先切到速度模式，再设置限流和加速度，最后使能并写入速度目标。
  set_mode(mode::kVelocity);
  write_param_float(param::kLimitCurrent, current_limit);
  write_param_float(param::kVelocityAcceleration, acceleration);
  enable();
  write_param_float(param::kSpeedRef, velocity);
}

// -----------------------------------------------------------------------------
// 内部发送/等待工具
// -----------------------------------------------------------------------------

/**
 * @brief 发送 payload 全 0 的 RS01 命令帧。
 *
 * @param communication_type 通信类型，来自 rs01::comm。
 */
void Rs01Motor::send_empty(uint32_t communication_type) {
  uint8_t data[8] {};
  socket_.send_extended(make_can_id(communication_type, host_id_, motor_id_),
                        data, 8);
}

/**
 * @brief 等待一帧状态反馈作为命令响应。
 *
 * @throws std::runtime_error 超时或未读到反馈帧时抛出。
 */
void Rs01Motor::expect_status() {
  if (!read_feedback(100)) {
    throw std::runtime_error("timeout waiting for RS01 status frame");
  }
}

// -----------------------------------------------------------------------------
// 数值映射工具
// -----------------------------------------------------------------------------

/**
 * @brief 将数值限制在指定闭区间内。
 *
 * @param value 输入值。
 * @param low 下限。
 * @param high 上限。
 * @return 限幅后的结果。
 */
float Rs01Motor::clamp(float value, float low, float high) {
  return std::max(low, std::min(high, value));
}

/**
 * @brief 将物理量线性映射到 16 bit 无符号整数。
 *
 * @param value 物理量输入值。
 * @param min 映射范围下限。
 * @param max 映射范围上限。
 * @return 映射后的 0~65535 整数。
 */
uint16_t Rs01Motor::float_to_uint(float value, float min, float max) {
  value = clamp(value, min, max);
  // 将实际物理量从 [min, max] 线性映射到协议使用的 [0, 65535]。
  return static_cast<uint16_t>(((value - min) * 65535.0f) / (max - min));
}

/**
 * @brief 将 16 bit 无符号整数反向映射为物理量。
 *
 * @param value 协议中的 0~65535 整数。
 * @param min 映射范围下限。
 * @param max 映射范围上限。
 * @return 反向映射后的物理量。
 */
float Rs01Motor::uint_to_float(uint16_t value, float min, float max) {
  return static_cast<float>(value) * (max - min) / 65535.0f + min;
}

} // namespace rs01
