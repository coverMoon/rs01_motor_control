#include "rs01_motor/rs01_motor.h"

#include "rs01_motor/protocol.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <sstream>
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

/**
 * @brief 开启或关闭电机主动上报。
 *
 * @param enable true 开启主动上报；false 关闭主动上报。
 * @param timeout_ms 等待电机应答的超时时间，单位毫秒。
 * @return true 表示收到配置应答；false 表示命令已发送但未等到应答。
 */
bool Rs01Motor::set_active_report(bool enable, int timeout_ms) {
  uint8_t data[8] {};
  // 手册要求前 6 字节为固定序列，Byte6 为开关：0 关闭，1 开启。默认上报间隔为 10ms。
  data[0] = 0x01;
  data[1] = 0x02;
  data[2] = 0x03;
  data[3] = 0x04;
  data[4] = 0x05;
  data[5] = 0x06;
  data[6] = enable ? 0x01 : 0x00;
  data[7] = 0x00;

  socket_.send_extended(make_can_id(comm::kActiveReport, host_id_, motor_id_),
                        data, 8);

  // 新固件应答类型 24；0.1.3.2 及以前可能回普通 type-2 反馈。
  if (receive_matching_frame(comm::kActiveReport, std::nullopt, timeout_ms)) {
    return true;
  }
  if (read_feedback(timeout_ms)) {
    return true;
  }
  return false;
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
  auto frame = receive_matching_frame(comm::kFeedback, std::nullopt, timeout_ms);
  if (!frame) {
    return std::nullopt;
  }
  return parse_feedback(*frame);
}

/**
 * @brief 从原始 CAN 帧解析 RS01 反馈帧。
 *
 * @param frame 已确认或待确认的 CAN 帧。
 * @return 解析后的反馈物理量。
 * @throws std::runtime_error 非扩展帧、非反馈帧或帧长度不足时抛出。
 */
Feedback Rs01Motor::parse_feedback(const can_frame &frame) {
  if ((frame.can_id & CAN_EFF_FLAG) == 0) {
    throw std::runtime_error("feedback frame is not an extended CAN frame");
  }
  const uint8_t type = frame_communication_type(frame);
  if (type != comm::kFeedback && type != comm::kActiveReport) {
    throw std::runtime_error("CAN frame is not an RS01 feedback/report frame");
  }
  if (frame.can_dlc < 8) {
    throw std::runtime_error("feedback frame is shorter than 8 bytes");
  }

  const uint32_t can_id = frame.can_id & CAN_EFF_MASK;
  const uint16_t extra_data = static_cast<uint16_t>((can_id >> 8) & 0xFFFF);

  // 反馈 payload 依次是位置、速度、力矩、温度，均为大端 16 bit 数据。
  // 前三个字段需要按 RS01 固定范围反向映射回实际物理量。
  const uint16_t position_u16 = unpack_u16_be(&frame.data[0]);
  const uint16_t velocity_u16 = unpack_u16_be(&frame.data[2]);
  const uint16_t torque_u16 = unpack_u16_be(&frame.data[4]);
  const uint16_t temperature_u16 = unpack_u16_be(&frame.data[6]);

  Feedback feedback;
  feedback.position = uint_to_float(position_u16, -kPositionMax, kPositionMax);
  feedback.velocity = uint_to_float(velocity_u16, -kVelocityMax, kVelocityMax);
  feedback.torque = uint_to_float(torque_u16, -kTorqueMax, kTorqueMax);
  feedback.temperature = static_cast<float>(temperature_u16) * 0.1f;
  // 反馈帧 CAN ID 的 extra_data 高位携带当前模式和 fault 位。
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

  auto frame = receive_matching_frame(comm::kReadParameter, index, timeout_ms);
  if (!frame) {
    return std::nullopt;
  }
  if (frame->can_dlc < 8) {
    throw std::runtime_error("parameter float response is shorter than 8 bytes");
  }

  // 参数读取响应沿用相同的 payload 布局：data[4..7] 是 float 参数值。
  float value = 0.0f;
  std::memcpy(&value, &frame->data[4], sizeof(float));
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

  auto frame = receive_matching_frame(comm::kReadParameter, index, timeout_ms);
  if (!frame) {
    return std::nullopt;
  }
  if (frame->can_dlc < 5) {
    throw std::runtime_error("parameter u8 response is shorter than 5 bytes");
  }
  return frame->data[4];
}

/**
 * @brief 读取 uint32 类型参数。
 *
 * @param index 参数索引，来自 rs01::param。
 * @param timeout_ms 接收响应的超时时间，单位毫秒。
 * @return 成功时返回参数值；超时、短帧或响应类型不匹配时返回 std::nullopt。
 */
std::optional<uint32_t> Rs01Motor::read_param_u32(uint16_t index,
                                                  int timeout_ms) {
  uint8_t data[8] {};
  pack_u16_le(&data[0], index);
  socket_.send_extended(make_can_id(comm::kReadParameter, host_id_, motor_id_),
                        data, 8);

  auto frame = receive_matching_frame(comm::kReadParameter, index, timeout_ms);
  if (!frame) {
    return std::nullopt;
  }
  if (frame->can_dlc < 8) {
    throw std::runtime_error("parameter u32 response is shorter than 8 bytes");
  }
  return unpack_u32_le(&frame->data[4]);
}

// -----------------------------------------------------------------------------
// 故障解析
// -----------------------------------------------------------------------------

/**
 * @brief 解析反馈帧 CAN ID 中的 6 bit 简略故障字段。
 *
 * @param fault 反馈帧 extra_data 中 bit21..16 压缩后的故障字段。
 * @return 当前置位故障的中文说明；无故障时返回空数组。
 */
std::vector<std::string> Rs01Motor::describe_feedback_fault(uint8_t fault) {
  std::vector<std::string> descriptions;
  if (fault & (1U << 0)) {
    descriptions.emplace_back("欠压故障");
  }
  if (fault & (1U << 1)) {
    descriptions.emplace_back("三相电流故障");
  }
  if (fault & (1U << 2)) {
    descriptions.emplace_back("过温故障");
  }
  if (fault & (1U << 3)) {
    descriptions.emplace_back("磁编码故障");
  }
  if (fault & (1U << 4)) {
    descriptions.emplace_back("堵转过载故障");
  }
  if (fault & (1U << 5)) {
    descriptions.emplace_back("编码器未标定");
  }
  return descriptions;
}

/**
 * @brief 解析完整 fault bitmask。
 *
 * @param fault 功能码 0x3022 或通信类型 21 中的 fault 值。
 * @return 当前置位故障的中文说明；无故障时返回空数组。
 */
std::vector<std::string> Rs01Motor::describe_fault_bits(uint32_t fault) {
  std::vector<std::string> descriptions;
  if (fault & (1U << 0)) {
    descriptions.emplace_back("电机过温故障，默认 103 度");
  }
  if (fault & (1U << 1)) {
    descriptions.emplace_back("驱动芯片故障");
  }
  if (fault & (1U << 2)) {
    descriptions.emplace_back("欠压故障");
  }
  if (fault & (1U << 3)) {
    descriptions.emplace_back("过压故障");
  }
  if (fault & (1U << 4)) {
    descriptions.emplace_back("B 相电流采样过流");
  }
  if (fault & (1U << 5)) {
    descriptions.emplace_back("C 相电流采样过流");
  }
  if (fault & (1U << 7)) {
    descriptions.emplace_back("编码器未标定");
  }
  if (fault & (1U << 8)) {
    descriptions.emplace_back("硬件识别故障");
  }
  if (fault & (1U << 9)) {
    descriptions.emplace_back("位置初始化故障");
  }
  if (fault & (1U << 14)) {
    descriptions.emplace_back("堵转过载算法保护");
  }
  if (fault & (1U << 16)) {
    descriptions.emplace_back("A 相电流采样过流");
  }
  return descriptions;
}

/**
 * @brief 解析 warning bitmask。
 *
 * @param warning 通信类型 21 或功能码 0x3023 中的 warning 值。
 * @return 当前置位预警的中文说明；无预警时返回空数组。
 */
std::vector<std::string> Rs01Motor::describe_warning_bits(uint32_t warning) {
  std::vector<std::string> descriptions;
  if (warning & (1U << 0)) {
    descriptions.emplace_back("电机过温预警，默认 93 度");
  }
  return descriptions;
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

/**
 * @brief 执行一次电流模式配置并写入 q 轴电流目标。
 *
 * @param current q 轴电流目标，单位 A。
 */
void Rs01Motor::current_control(float current) {
  current = clamp(current, -kCurrentMax, kCurrentMax);

  // 电流模式会直接产生力矩，测试时应使用很小的 iq_ref，并在退出前写回 0。
  set_mode(mode::kCurrent);
  enable();
  write_param_float(param::kIqRef, current);
}

/**
 * @brief 执行一次 PP 位置模式配置并写入位置目标。
 *
 * @param position 目标位置，单位 rad。
 * @param velocity_limit PP 模式最大速度限制，单位 rad/s。
 * @param acceleration PP 模式加速度限制，单位 rad/s^2。
 */
void Rs01Motor::position_pp_control(float position, float velocity_limit,
                                    float acceleration) {
  position = clamp(position, -kPositionMax, kPositionMax);
  velocity_limit = clamp(velocity_limit, 0.0f, kVelocityMax);
  acceleration = std::max(0.0f, acceleration);

  // PP 模式的最小安全流程：先切到 PP 位置模式，再设置速度/加速度限制，最后使能并写位置目标。
  set_mode(mode::kPositionPp);
  write_param_float(param::kPpVelocityMax, velocity_limit);
  write_param_float(param::kPpAcceleration, acceleration);
  enable();
  write_param_float(param::kLocRef, position);
}

/**
 * @brief 执行一次 CSP 位置模式配置并写入初始位置目标。
 *
 * @param position 初始目标位置，单位 rad。
 * @param velocity_limit CSP 模式速度限制，单位 rad/s。
 */
void Rs01Motor::position_csp_control(float position, float velocity_limit) {
  position = clamp(position, -kPositionMax, kPositionMax);
  velocity_limit = clamp(velocity_limit, 0.0f, kVelocityMax);

  // CSP 模式由上位机周期性写 loc_ref，这里只完成模式切换、限速和初始目标写入。
  set_mode(mode::kPositionCsp);
  write_param_float(param::kLimitSpeed, velocity_limit);
  enable();
  write_param_float(param::kLocRef, position);
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

/**
 * @brief 在总超时时间内循环接收，直到找到目标类型和可选参数索引的帧。
 *
 * @param communication_type 期望的通信类型。
 * @param parameter_index 可选参数索引；用于参数读响应过滤。
 * @param timeout_ms 总超时时间，单位毫秒。
 * @return 找到匹配帧时返回 can_frame；超时返回 std::nullopt。
 * @throws std::runtime_error 收到故障帧或匹配帧长度明显不足时抛出。
 */
std::optional<can_frame> Rs01Motor::receive_matching_frame(
    uint32_t communication_type, std::optional<uint16_t> parameter_index,
    int timeout_ms) {
  using clock = std::chrono::steady_clock;
  const auto deadline = clock::now() + std::chrono::milliseconds(timeout_ms);

  while (true) {
    const auto now = clock::now();
    if (now >= deadline) {
      return std::nullopt;
    }

    const auto remaining_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now)
            .count();

    can_frame frame {};
    if (!socket_.receive(frame, static_cast<int>(std::max<int64_t>(
                                    1, remaining_ms)))) {
      return std::nullopt;
    }
    if ((frame.can_id & CAN_EFF_FLAG) == 0) {
      continue;
    }

    const uint8_t received_type = frame_communication_type(frame);
    if (received_type == comm::kFaultReport) {
      throw std::runtime_error(format_fault_report(parse_fault_report_frame(frame)));
    }
    if (received_type != communication_type) {
      continue;
    }

    if (parameter_index) {
      if (frame.can_dlc < 2) {
        throw std::runtime_error("parameter response is shorter than 2 bytes");
      }
      if (unpack_u16_le(&frame.data[0]) != *parameter_index) {
        continue;
      }
    }

    return frame;
  }
}

// -----------------------------------------------------------------------------
// 协议解析工具
// -----------------------------------------------------------------------------

/**
 * @brief 从扩展帧 CAN ID 中解析通信类型字段。
 *
 * @param frame 输入 CAN 帧。
 * @return 通信类型值。
 */
uint8_t Rs01Motor::frame_communication_type(const can_frame &frame) {
  const uint32_t can_id = frame.can_id & CAN_EFF_MASK;
  return static_cast<uint8_t>((can_id >> 24) & 0x1F);
}

/**
 * @brief 将 type-21 完整故障帧解析为 FaultReport。
 *
 * @param frame 已确认类型为 kFaultReport 的 CAN 帧。
 * @return 解析后的 fault/warning bitmask。
 * @throws std::runtime_error 帧长度不足时抛出。
 */
FaultReport Rs01Motor::parse_fault_report_frame(const can_frame &frame) {
  if (frame.can_dlc < 8) {
    throw std::runtime_error("fault report frame is shorter than 8 bytes");
  }

  FaultReport report;
  report.fault = unpack_u32_le(&frame.data[0]);
  report.warning = unpack_u32_le(&frame.data[4]);
  return report;
}

/**
 * @brief 将完整故障上报格式化为可读异常信息。
 *
 * @param report 已解析的故障上报。
 * @return 包含原始 bitmask 和中文说明的字符串。
 */
std::string Rs01Motor::format_fault_report(const FaultReport &report) {
  std::ostringstream oss;
  oss << "received RS01 fault report frame: fault=0x" << std::hex
      << report.fault << ", warning=0x" << report.warning << std::dec;

  const auto faults = describe_fault_bits(report.fault);
  if (!faults.empty()) {
    oss << ", fault_desc=[";
    for (std::size_t i = 0; i < faults.size(); ++i) {
      if (i > 0) {
        oss << "; ";
      }
      oss << faults[i];
    }
    oss << "]";
  }

  const auto warnings = describe_warning_bits(report.warning);
  if (!warnings.empty()) {
    oss << ", warning_desc=[";
    for (std::size_t i = 0; i < warnings.size(); ++i) {
      if (i > 0) {
        oss << "; ";
      }
      oss << warnings[i];
    }
    oss << "]";
  }

  return oss.str();
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
