#include "rs01_motor/protocol.h"
#include "rs01_motor/rs01_motor.h"
#include "terminal_input.h"

#include <algorithm>
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
 * @brief 记录 Ctrl+C 请求，让主循环有机会先失能电机。
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
 * @brief 对 RS01 执行交互式 CSP 位置模式测试。
 *
 * @param argc 命令行参数数量。
 * @param argv 命令行参数，支持 [CAN接口] [电机ID] [单步rad]
 *             [速度限制rad/s] [周期ms] [主机ID]。
 * @return 0 表示测试流程完成，非 0 表示通信或控制命令失败。
 */
int main(int argc, char **argv) {
  // ---------------------------------------------------------------------------
  // 命令行参数
  // ---------------------------------------------------------------------------

  // 用法：rs01_position_csp_test [CAN接口] [电机ID] [单步rad]
  //       [速度限制rad/s] [周期ms] [主机ID]
  //
  // CSP 由上位机周期性给位置目标。程序从当前位置开始，按键只做小步增减。
  const std::string iface = argc > 1 ? argv[1] : "can0";
  const uint8_t motor_id =
      argc > 2 ? static_cast<uint8_t>(std::strtoul(argv[2], nullptr, 0)) : 1;
  const float step = argc > 3 ? std::strtof(argv[3], nullptr) : 0.05f;
  const float velocity_limit = argc > 4 ? std::strtof(argv[4], nullptr) : 1.0f;
  const int period_ms =
      std::max(1, argc > 5 ? static_cast<int>(std::strtol(argv[5], nullptr, 0))
                           : 20);
  const uint8_t host_id = argc > 6
                              ? static_cast<uint8_t>(
                                    std::strtoul(argv[6], nullptr, 0))
                              : 0xFF;

  // ---------------------------------------------------------------------------
  // 交互式 CSP 位置测试流程
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

    float target_position = *current_position;

    // 先进入 CSP 模式并写入当前位置作为初始目标，避免启动瞬间位置跳变。
    motor.position_csp_control(target_position, velocity_limit);

    std::cout << "Interactive CSP position test\n"
              << "  start position: " << target_position << " rad\n"
              << "  step: " << step << " rad\n"
              << "  velocity_limit: " << velocity_limit << " rad/s\n"
              << "  period: " << period_ms << " ms\n"
              << "  [: target position -= step\n"
              << "  ]: target position += step\n"
              << "  h: read current position and hold there\n"
              << "  q: disable and quit\n";

    while (!g_stop_requested) {
      char key = 0;
      if (input.read_key(key)) {
        if (key == '[') {
          target_position -= step;
          std::cout << "target position: " << target_position << " rad\n";
        } else if (key == ']') {
          target_position += step;
          std::cout << "target position: " << target_position << " rad\n";
        } else if (key == 'h' || key == 'H') {
          auto measured_position = motor.read_param_float(rs01::param::kMechPos);
          if (measured_position) {
            target_position = *measured_position;
            std::cout << "holding current position: " << target_position
                      << " rad\n";
          } else {
            std::cout << "failed to read current position\n";
          }
        } else if (key == 'q' || key == 'Q') {
          break;
        }
      }

      // CSP 的关键是周期性刷新 loc_ref，即使目标不变也持续写入。
      motor.write_param_float(rs01::param::kLocRef, target_position);
      std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
    }

    motor.disable(false);
  } catch (const std::exception &error) {
    std::cerr << error.what() << "\n";
    return 1;
  }

  return 0;
}
