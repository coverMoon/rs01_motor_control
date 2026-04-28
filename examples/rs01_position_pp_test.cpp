#include "rs01_motor/protocol.h"
#include "rs01_motor/rs01_motor.h"
#include "terminal_input.h"

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace {

volatile std::sig_atomic_t g_stop_requested = 0;

// -----------------------------------------------------------------------------
// 信号处理
// -----------------------------------------------------------------------------

/**
 * @brief 记录 Ctrl+C 请求，让主循环有机会先停止并失能电机。
 *
 * @param signal 收到的信号编号。
 */
void handle_signal(int signal) {
  if (signal == SIGINT || signal == SIGTERM) {
    g_stop_requested = 1;
  }
}

} // namespace

/**
 * @brief 对 RS01 执行交互式 PP 位置模式测试。
 *
 * @param argc 命令行参数数量。
 * @param argv 命令行参数，支持 [CAN接口] [电机ID] [目标位置rad]
 *             [最大速度rad/s] [加速度rad/s^2] [主机ID]。
 * @return 0 表示测试流程完成，非 0 表示通信或控制命令失败。
 */
int main(int argc, char **argv) {
  // ---------------------------------------------------------------------------
  // 命令行参数
  // ---------------------------------------------------------------------------

  // 用法：rs01_position_pp_test [CAN接口] [电机ID] [目标位置rad]
  //       [最大速度rad/s] [加速度rad/s^2] [主机ID]
  //
  // 如果没有指定目标位置，程序会使用启动时读取到的当前位置，避免突然运动。
  const std::string iface = argc > 1 ? argv[1] : "can0";
  const uint8_t motor_id =
      argc > 2 ? static_cast<uint8_t>(std::strtoul(argv[2], nullptr, 0)) : 1;
  const bool has_position_arg = argc > 3;
  const float requested_position =
      has_position_arg ? std::strtof(argv[3], nullptr) : 0.0f;
  const float velocity_limit = argc > 4 ? std::strtof(argv[4], nullptr) : 1.0f;
  const float acceleration = argc > 5 ? std::strtof(argv[5], nullptr) : 2.0f;
  const uint8_t host_id = argc > 6
                              ? static_cast<uint8_t>(
                                    std::strtoul(argv[6], nullptr, 0))
                              : 0xFF;

  // ---------------------------------------------------------------------------
  // 交互式 PP 位置测试流程
  // ---------------------------------------------------------------------------

  try {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    rs01::Rs01Motor motor(iface, motor_id, host_id);
    rs01_examples::TerminalInput input;

    auto current_position = motor.read_param_float(rs01::param::kMechPos);
    if (!current_position) {
      std::cerr << "No response while reading mech_pos\n";
      return 1;
    }

    const float hold_position = *current_position;
    const float target_position =
        has_position_arg ? requested_position : hold_position;

    // 先进入 PP 模式并保持当前位置，等待用户明确按键后再发目标位置。
    motor.position_pp_control(hold_position, velocity_limit, acceleration);

    std::cout << "Interactive PP position test\n"
              << "  current position: " << hold_position << " rad\n"
              << "  target position: " << target_position << " rad\n"
              << "  velocity_limit: " << velocity_limit << " rad/s\n"
              << "  acceleration: " << acceleration << " rad/s^2\n"
              << "  r: send target position\n"
              << "  h: read current position and hold there\n"
              << "  q: disable and quit\n";

    while (!g_stop_requested) {
      char key = 0;
      if (input.read_key(key)) {
        if (key == 'r' || key == 'R') {
          motor.write_param_float(rs01::param::kLocRef, target_position);
          std::cout << "target position sent: " << target_position << " rad\n";
        } else if (key == 'h' || key == 'H') {
          auto measured_position = motor.read_param_float(rs01::param::kMechPos);
          if (measured_position) {
            motor.write_param_float(rs01::param::kLocRef, *measured_position);
            std::cout << "holding current position: " << *measured_position
                      << " rad\n";
          } else {
            std::cout << "failed to read current position\n";
          }
        } else if (key == 'q' || key == 'Q') {
          break;
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    motor.disable(false);
  } catch (const std::exception &error) {
    std::cerr << error.what() << "\n";
    return 1;
  }

  return 0;
}
