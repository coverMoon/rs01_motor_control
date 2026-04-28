#pragma once

#include <cstdint>
#include <cstring>

namespace rs01 {

// -----------------------------------------------------------------------------
// 通信类型
// -----------------------------------------------------------------------------

// RS01 私有协议通信类型字段，位于扩展帧 CAN ID 的 bits 28..24。
// 这里仅保留当前库已使用的帧类型。
namespace comm {
constexpr uint32_t kMotionControl = 0x01;
constexpr uint32_t kFeedback = 0x02;
constexpr uint32_t kEnable = 0x03;
constexpr uint32_t kDisable = 0x04;
constexpr uint32_t kSetZero = 0x06;
constexpr uint32_t kReadParameter = 0x11;
constexpr uint32_t kWriteParameter = 0x12;
constexpr uint32_t kFaultReport = 0x15;
constexpr uint32_t kActiveReport = 0x18;
} // namespace comm

// -----------------------------------------------------------------------------
// 参数索引
// -----------------------------------------------------------------------------

// RS01 参数表索引。参数读写命令通过这些 16 bit 地址访问运行模式、
// 速度/电流/位置目标值、限制值、诊断量和故障 bitmask。
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
constexpr uint16_t kFaultStatus = 0x3022;
constexpr uint16_t kWarningStatus = 0x3023;
constexpr uint16_t kDriverFault = 0x3024;
} // namespace param

// -----------------------------------------------------------------------------
// 运行模式
// -----------------------------------------------------------------------------

// run_mode 参数支持的模式值。
// 使用这些常量可以避免业务代码里直接写 0/1/2/3/5 这类裸数字。
namespace mode {
constexpr int8_t kMotion = 0;
constexpr int8_t kPositionPp = 1;
constexpr int8_t kVelocity = 2;
constexpr int8_t kCurrent = 3;
constexpr int8_t kPositionCsp = 5;
} // namespace mode

// -----------------------------------------------------------------------------
// RS01 物理量范围
// -----------------------------------------------------------------------------

// RS01 手册给出的物理量范围。
// 运控帧和反馈帧使用这些范围在物理量和 16 bit 无符号整数之间做线性映射。
constexpr float kPositionMax = 4.0f * 3.14159265358979323846f;
constexpr float kVelocityMax = 44.0f;
constexpr float kTorqueMax = 17.0f;
constexpr float kKpMax = 500.0f;
constexpr float kKdMax = 5.0f;
constexpr float kCurrentMax = 23.0f;

// -----------------------------------------------------------------------------
// 协议打包工具
// -----------------------------------------------------------------------------

// RS01 扩展帧 CAN ID 布局：
// bits 28..24 为通信类型，bits 23..8 为附加数据，bits 7..0 为目标电机 ID。
// 附加数据在不同命令中含义不同，例如普通命令里常放 host_id，运控帧里放力矩映射值。
/**
 * @brief 生成 RS01 私有协议使用的 29 bit 扩展帧 CAN ID。
 *
 * @param communication_type 通信类型字段，来自 rs01::comm。
 * @param extra_data 16 bit 附加数据，含义由具体通信类型决定。
 * @param target_id 目标电机 ID。
 * @return 未附加 CAN_EFF_FLAG 的扩展帧 ID。
 */
inline uint32_t make_can_id(uint32_t communication_type, uint16_t extra_data,
                            uint8_t target_id) {
  return (communication_type << 24) | (static_cast<uint32_t>(extra_data) << 8) |
         target_id;
}

// 参数读写帧中的参数索引使用小端序，即低字节在前、高字节在后。
/**
 * @brief 按小端序写入 16 bit 无符号整数。
 *
 * @param data 至少有 2 字节空间的输出缓冲区。
 * @param value 要写入的数值。
 */
inline void pack_u16_le(uint8_t *data, uint16_t value) {
  data[0] = static_cast<uint8_t>(value & 0xFF);
  data[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

/**
 * @brief 按小端序读取 16 bit 无符号整数。
 *
 * @param data 至少有 2 字节空间的输入缓冲区。
 * @return 解析出的 16 bit 数值。
 */
inline uint16_t unpack_u16_le(const uint8_t *data) {
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

/**
 * @brief 按小端序读取 32 bit 无符号整数。
 *
 * @param data 至少有 4 字节空间的输入缓冲区。
 * @return 解析出的 32 bit 数值。
 */
inline uint32_t unpack_u32_le(const uint8_t *data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

// 运控帧和反馈帧中的 16 bit 物理量使用大端序，与参数索引的字节序不同。
/**
 * @brief 按大端序写入 16 bit 无符号整数。
 *
 * @param data 至少有 2 字节空间的输出缓冲区。
 * @param value 要写入的数值。
 */
inline void pack_u16_be(uint8_t *data, uint16_t value) {
  data[0] = static_cast<uint8_t>((value >> 8) & 0xFF);
  data[1] = static_cast<uint8_t>(value & 0xFF);
}

/**
 * @brief 按大端序读取 16 bit 无符号整数。
 *
 * @param data 至少有 2 字节空间的输入缓冲区。
 * @return 解析出的 16 bit 数值。
 */
inline uint16_t unpack_u16_be(const uint8_t *data) {
  return (static_cast<uint16_t>(data[0]) << 8) |
         static_cast<uint16_t>(data[1]);
}

// float 参数按 IEEE-754 原始字节写入 payload。当前运行环境是常见 x86/Linux 小端序，
// 因此直接 memcpy 即可；如果移植到大端架构，需要重新确认协议字节序。
/**
 * @brief 写入 float 参数的原始 IEEE-754 字节。
 *
 * @param data 至少有 4 字节空间的输出缓冲区。
 * @param value 要写入的 float 参数值。
 */
inline void pack_float_le(uint8_t *data, float value) {
  std::memcpy(data, &value, sizeof(float));
}

} // namespace rs01
