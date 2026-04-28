#pragma once

#include "rs01_motor/can_socket.h"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace rs01 {

// RS01 状态反馈帧解析后的物理量。
// position/velocity/torque 已从 16 bit 映射值转换为实际单位，temperature 单位为摄氏度。
// mode/fault 来自反馈帧扩展 ID 中的状态字段。
struct Feedback {
  float position = 0.0f;
  float velocity = 0.0f;
  float torque = 0.0f;
  float temperature = 0.0f;
  uint8_t mode = 0;
  uint8_t fault = 0;
};

// 完整故障上报帧解析结果。fault/warning 均为原始 bitmask，具体含义见解析函数。
struct FaultReport {
  uint32_t fault = 0;
  uint32_t warning = 0;
};

// 单个 RS01 电机的控制封装。
// 每个对象绑定一个 CAN 接口和一个电机 ID，内部通过 Linux SocketCAN 发送 RS01 私有扩展帧。
//
// API 分为三层：
// 1. 基础命令和配置：使能、失能、清零、主动上报开关；
// 2. 参数/反馈访问：通用参数读写、反馈解析、故障解析；
// 3. 控制模式封装：速度、电流、PP、CSP 以及运控帧发送。
class Rs01Motor {
public:
  // ---------------------------------------------------------------------------
  // 生命周期
  // ---------------------------------------------------------------------------

  Rs01Motor(std::string can_interface, uint8_t motor_id,
            uint8_t host_id = 0xFF);

  // ---------------------------------------------------------------------------
  // 基础命令和配置
  // ---------------------------------------------------------------------------

  // 使能电机。命令发出后会等待一帧状态反馈，用来确认电机侧有响应。
  void enable();

  // 失能电机。clear_error=true 时，请求电机在失能时清除错误。
  void disable(bool clear_error = false);

  // 将当前机械位置设置为零点。
  void set_zero();

  // 开启或关闭主动上报。返回 true 表示收到应答；false 表示命令已发送但无应答。
  bool set_active_report(bool enable, int timeout_ms = 100);

  // ---------------------------------------------------------------------------
  // 反馈读取和解析
  // ---------------------------------------------------------------------------

  // 尝试接收并解析一帧状态反馈；超时或收到非反馈帧时返回 std::nullopt。
  std::optional<Feedback> read_feedback(int timeout_ms = 100);

  // 从原始 CAN 帧解析 RS01 状态反馈或主动上报帧，供只监听工具复用。
  static Feedback parse_feedback(const can_frame &frame);

  // ---------------------------------------------------------------------------
  // 参数访问
  // ---------------------------------------------------------------------------

  // 通用参数读写接口。index 来自 protocol.h 中的 rs01::param 常量。
  // u8 用于 run_mode 等枚举/开关参数。
  void write_param_u8(uint16_t index, uint8_t value);
  std::optional<uint8_t> read_param_u8(uint16_t index, int timeout_ms = 100);

  // float 用于速度、电流、位置、限制值、诊断量等参数。
  void write_param_float(uint16_t index, float value);
  std::optional<float> read_param_float(uint16_t index, int timeout_ms = 100);

  // u32 当前主要用于 fault_status/warning_status 这类 bitmask 参数。
  std::optional<uint32_t> read_param_u32(uint16_t index, int timeout_ms = 100);

  // ---------------------------------------------------------------------------
  // 故障解析
  // ---------------------------------------------------------------------------

  // 将反馈帧 6 bit 简略故障、完整 fault/warning bitmask 转换为中文说明。
  static std::vector<std::string> describe_feedback_fault(uint8_t fault);
  static std::vector<std::string> describe_fault_bits(uint32_t fault);
  static std::vector<std::string> describe_warning_bits(uint32_t warning);

  // ---------------------------------------------------------------------------
  // 控制模式封装
  // ---------------------------------------------------------------------------

  // 切换 run_mode。该函数会先失能电机，再写入目标模式。
  void set_mode(uint8_t mode);

  // 发送运控模式 type-1 控制帧。该函数只发控制帧，不切模式、不使能。
  // 调用方应先执行 set_mode(mode::kMotion) 和 enable()，再周期性调用本函数。
  void motion_control(float torque, float position, float velocity, float kp,
                      float kd);

  // 进入速度模式，设置限流/加速度，使能后写入速度目标。
  void velocity_control(float velocity, float current_limit = 1.0f,
                        float acceleration = 2.0f);

  // 进入电流模式，使能后写入 iq_ref。电流模式会直接产生力矩，调用方应使用小电流测试。
  void current_control(float current);

  // 进入 PP 位置模式，设置速度/加速度限制，使能后写入目标位置。
  void position_pp_control(float position, float velocity_limit = 1.0f,
                           float acceleration = 2.0f);

  // 进入 CSP 位置模式，设置速度限制，使能后写入初始位置；后续应周期性写 loc_ref。
  void position_csp_control(float position, float velocity_limit = 1.0f);

private:
  // ---------------------------------------------------------------------------
  // 内部发送/等待工具
  // ---------------------------------------------------------------------------

  // 发送空 payload 命令，以及等待状态反馈的公共辅助函数。
  void send_empty(uint32_t communication_type);
  void expect_status();

  // 在总超时时间内持续收帧，直到找到指定通信类型和可选参数索引的响应。
  std::optional<can_frame> receive_matching_frame(
      uint32_t communication_type, std::optional<uint16_t> parameter_index,
      int timeout_ms);

  // ---------------------------------------------------------------------------
  // 协议解析工具
  // ---------------------------------------------------------------------------

  // 从扩展帧 CAN ID 中取出通信类型字段。
  static uint8_t frame_communication_type(const can_frame &frame);

  // 解析 type-21 完整故障上报帧，并格式化为异常信息。
  static FaultReport parse_fault_report_frame(const can_frame &frame);
  static std::string format_fault_report(const FaultReport &report);

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
