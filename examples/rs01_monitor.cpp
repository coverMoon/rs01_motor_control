#include "rs01_motor/can_socket.h"
#include "rs01_motor/protocol.h"
#include "rs01_motor/rs01_motor.h"

#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <linux/can.h>

namespace {

volatile std::sig_atomic_t g_stop_requested = 0;

// -----------------------------------------------------------------------------
// 信号处理
// -----------------------------------------------------------------------------

/**
 * @brief 记录退出请求，让监听循环可以自然结束。
 *
 * @param signal 收到的信号编号。
 */
void handle_signal(int signal) {
  if (signal == SIGINT || signal == SIGTERM) {
    g_stop_requested = 1;
  }
}

// -----------------------------------------------------------------------------
// 打印工具
// -----------------------------------------------------------------------------

/**
 * @brief 把反馈帧模式字段转换为可读文本。
 *
 * @param mode 反馈帧中的模式字段。
 * @return 模式名称。
 */
std::string mode_name(uint8_t mode) {
  switch (mode) {
  case 0:
    return "Reset";
  case 1:
    return "Cali";
  case 2:
    return "Motor";
  default:
    return "Unknown";
  }
}

/**
 * @brief 把故障说明数组拼接成单行文本。
 *
 * @param descriptions 故障说明数组。
 * @return 拼接后的文本，无故障时返回 normal。
 */
std::string join_faults(const std::vector<std::string> &descriptions) {
  if (descriptions.empty()) {
    return "normal";
  }

  std::string result;
  for (std::size_t i = 0; i < descriptions.size(); ++i) {
    if (i > 0) {
      result += "; ";
    }
    result += descriptions[i];
  }
  return result;
}

/**
 * @brief 打印单行实时状态，下一帧到来时在原位置覆盖。
 *
 * @param motor_id 当前反馈帧来源电机 ID。
 * @param feedback 解析后的反馈数据。
 * @param fault_text 故障状态文本。
 */
void print_status_line(uint8_t motor_id, const rs01::Feedback &feedback,
                       const std::string &fault_text) {
  // "\r" 回到行首，"\033[2K" 清空当前行，避免新内容比旧内容短时留下残影。
  std::cout << "\r\033[2K" << std::left << std::setw(5)
            << static_cast<int>(motor_id) << std::setw(11) << std::fixed
            << std::setprecision(4) << feedback.position << std::setw(11)
            << feedback.velocity << std::setw(11) << feedback.torque
            << std::setw(9) << std::setprecision(1) << feedback.temperature
            << std::setw(8) << mode_name(feedback.mode) << fault_text
            << std::flush;
}

/**
 * @brief 从扩展帧 ID 中解析通信类型。
 *
 * @param frame 输入 CAN 帧。
 * @return 通信类型字段。
 */
uint8_t communication_type(const can_frame &frame) {
  return static_cast<uint8_t>(((frame.can_id & CAN_EFF_MASK) >> 24) & 0x1F);
}

/**
 * @brief 从反馈帧 ID 中解析当前电机 ID。
 *
 * @param frame 输入 CAN 帧。
 * @return 反馈帧中的电机 ID。
 */
uint8_t feedback_motor_id(const can_frame &frame) {
  const uint32_t can_id = frame.can_id & CAN_EFF_MASK;
  return static_cast<uint8_t>((can_id >> 8) & 0xFF);
}

} // namespace

/**
 * @brief 被动监听 RS01 反馈帧并持续打印状态。
 *
 * @param argc 命令行参数数量。
 * @param argv 命令行参数，支持 [CAN接口] [电机ID]。
 * @return 0 表示正常退出，非 0 表示监听失败。
 */
int main(int argc, char **argv) {
  // ---------------------------------------------------------------------------
  // 命令行参数
  // ---------------------------------------------------------------------------

  // 用法：rs01_monitor [CAN接口] [电机ID]
  // 该工具只监听，不发送任何 CAN 帧。若未指定电机 ID，则显示所有 RS01 反馈帧。
  const std::string iface = argc > 1 ? argv[1] : "can0";
  const bool filter_motor_id = argc > 2;
  const uint8_t motor_id =
      filter_motor_id
          ? static_cast<uint8_t>(std::strtoul(argv[2], nullptr, 0))
          : 0;

  // ---------------------------------------------------------------------------
  // 被动监听
  // ---------------------------------------------------------------------------

  try {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    rs01::CanSocket socket(iface);

    std::cout << "RS01 passive monitor: interface=" << iface;
    if (filter_motor_id) {
      std::cout << ", motor_id=" << static_cast<int>(motor_id);
    } else {
      std::cout << ", motor_id=all";
    }
    std::cout << "\n";
    std::cout << "等待反馈帧，按 Ctrl+C 退出...\n";
    std::cout << std::left << std::setw(5) << "id" << std::setw(11) << "pos"
              << std::setw(11) << "vel" << std::setw(11) << "torque"
              << std::setw(9) << "temp" << std::setw(8) << "mode"
              << "fault\n";

    while (!g_stop_requested) {
      can_frame frame {};
      if (!socket.receive(frame, 200)) {
        continue;
      }
      if ((frame.can_id & CAN_EFF_FLAG) == 0) {
        continue;
      }
      const uint8_t type = communication_type(frame);
      if (type != rs01::comm::kFeedback && type != rs01::comm::kActiveReport) {
        continue;
      }

      const uint8_t source_id = feedback_motor_id(frame);
      if (filter_motor_id && source_id != motor_id) {
        continue;
      }

      const rs01::Feedback feedback = rs01::Rs01Motor::parse_feedback(frame);
      const auto faults = rs01::Rs01Motor::describe_feedback_fault(feedback.fault);

      print_status_line(source_id, feedback, join_faults(faults));
    }

    std::cout << "\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << "\n";
    return 1;
  }

  return 0;
}
