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
 * @brief 记录 Ctrl+C 请求，让主循环有机会先清零电流再退出。
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
 * @brief 对 RS01 执行交互式电流模式测试。
 *
 * @param argc 命令行参数数量。
 * @param argv 命令行参数，支持 [CAN接口] [电机ID] [目标电流A] [主机ID]。
 * @return 0 表示测试流程完成，非 0 表示通信或控制命令失败。
 */
int main(int argc, char **argv) {
  // ---------------------------------------------------------------------------
  // 命令行参数
  // ---------------------------------------------------------------------------

  // 用法：rs01_current_test [CAN接口] [电机ID] [目标电流A] [主机ID]
  // 电流模式会直接产生力矩，默认目标只给 0.1A，首次测试不要使用大电流。
  const std::string iface = argc > 1 ? argv[1] : "can0";
  const uint8_t motor_id =
      argc > 2 ? static_cast<uint8_t>(std::strtoul(argv[2], nullptr, 0)) : 1;
  const float current = argc > 3 ? std::strtof(argv[3], nullptr) : 0.1f;
  const uint8_t host_id = argc > 4
                              ? static_cast<uint8_t>(
                                    std::strtoul(argv[4], nullptr, 0))
                              : 0xFF;

  // ---------------------------------------------------------------------------
  // 交互式电流测试流程
  // ---------------------------------------------------------------------------

  try {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    rs01::Rs01Motor motor(iface, motor_id, host_id);
    rs01_examples::TerminalInput input;

    // 先切到电流模式并使能，但电流目标保持为 0，等待用户按键再输出。
    motor.current_control(0.0f);

    bool running = false;
    std::cout << "Interactive current test\n"
              << "  r: set iq_ref to " << current << " A\n"
              << "  s: set iq_ref to 0 A\n"
              << "  q: zero current, disable and quit\n";

    while (!g_stop_requested) {
      char key = 0;
      if (input.read_key(key)) {
        if (key == 'r' || key == 'R') {
          motor.write_param_float(rs01::param::kIqRef, current);
          running = true;
          std::cout << "iq_ref: " << current << " A\n";
        } else if (key == 's' || key == 'S') {
          motor.write_param_float(rs01::param::kIqRef, 0.0f);
          running = false;
          std::cout << "iq_ref: 0 A\n";
        } else if (key == 'q' || key == 'Q') {
          break;
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    if (running) {
      std::cout << "zero current before exit\n";
    }
    motor.write_param_float(rs01::param::kIqRef, 0.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    motor.disable(false);
  } catch (const std::exception &error) {
    std::cerr << error.what() << "\n";
    return 1;
  }

  return 0;
}
