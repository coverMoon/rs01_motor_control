#pragma once

#include "rs01_motor/can_socket.h"

#include <cstdint>
#include <optional>
#include <string>

namespace rs01 {

// type-2 RS01 反馈帧解析后的物理量。
// position/velocity/torque 已从 16 bit 映射值转换为实际单位，temperature 单位为摄氏度。
struct Feedback {
  float position = 0.0f;
  float velocity = 0.0f;
  float torque = 0.0f;
  float temperature = 0.0f;
  uint8_t mode = 0;
  uint8_t fault = 0;
};

// 单个 RS01 电机的高层控制封装。
// 每个对象绑定一个 CAN 接口和一个电机 ID，内部通过 CanSocket 发送 RS01 私有扩展帧。
class Rs01Motor {
public:
  // ---------------------------------------------------------------------------
  // 生命周期
  // ---------------------------------------------------------------------------

  Rs01Motor(std::string can_interface, uint8_t motor_id,
            uint8_t host_id = 0xFF);

  // ---------------------------------------------------------------------------
  // 基础状态命令
  // ---------------------------------------------------------------------------

  // 基础状态命令。命令发出后会等待一帧状态反馈，用来确认电机侧有响应。
  void enable();
  void disable(bool clear_error = false);
  void set_zero();

  // ---------------------------------------------------------------------------
  // 反馈读取
  // ---------------------------------------------------------------------------

  // 尝试接收并解析一帧状态反馈；超时或收到非反馈帧时返回 std::nullopt。
  std::optional<Feedback> read_feedback(int timeout_ms = 100);

  // ---------------------------------------------------------------------------
  // 参数访问
  // ---------------------------------------------------------------------------

  // 通用参数读写接口。index 来自 protocol.h 中的 rs01::param 常量。
  // u8 用于 run_mode 等枚举/开关参数，float 用于速度、电流限制、加速度等参数。
  void write_param_u8(uint16_t index, uint8_t value);
  void write_param_float(uint16_t index, float value);
  std::optional<float> read_param_float(uint16_t index, int timeout_ms = 100);
  std::optional<uint8_t> read_param_u8(uint16_t index, int timeout_ms = 100);

  // ---------------------------------------------------------------------------
  // 控制封装
  // ---------------------------------------------------------------------------

  // 常用控制封装：set_mode 负责模式切换，motion_control 发送运控帧，
  // velocity_control 则按“切速度模式 -> 配限制 -> 使能 -> 写速度目标”的顺序执行。
  void set_mode(uint8_t mode);
  void motion_control(float torque, float position, float velocity, float kp,
                      float kd);
  void velocity_control(float velocity, float current_limit = 1.0f,
                        float acceleration = 2.0f);

private:
  // ---------------------------------------------------------------------------
  // 内部发送/等待工具
  // ---------------------------------------------------------------------------

  // 发送空 payload 命令，以及等待状态反馈的公共辅助函数。
  void send_empty(uint32_t communication_type);
  void expect_status();

  // ---------------------------------------------------------------------------
  // 数值映射工具
  // ---------------------------------------------------------------------------

  // 物理量和协议 16 bit 无符号整数之间的线性映射工具。
  static float clamp(float value, float low, float high);
  static uint16_t float_to_uint(float value, float min, float max);
  static float uint_to_float(uint16_t value, float min, float max);

  CanSocket socket_;
  uint8_t motor_id_;
  uint8_t host_id_;
};

} // namespace rs01
