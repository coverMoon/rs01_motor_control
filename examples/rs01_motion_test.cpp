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
 * @brief 记录 Ctrl+C 请求，让主循环有机会先发安全帧再退出。
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
 * @brief 对 RS01 执行交互式运控模式连通性测试。
 *
 * @param argc 命令行参数数量。
 * @param argv 命令行参数，支持 [CAN接口] [电机ID] [位置rad] [速度rad/s]
 *             [力矩Nm] [Kp] [Kd] [主机ID]。
 * @return 0 表示测试流程完成，非 0 表示通信或控制命令失败。
 */
int main(int argc, char **argv) {
  // ---------------------------------------------------------------------------
  // 命令行参数
  // ---------------------------------------------------------------------------

  // 用法：rs01_motion_test [CAN接口] [电机ID] [位置rad] [速度rad/s]
  //       [力矩Nm] [Kp] [Kd] [主机ID]
  //
  // 默认目标是阻尼测试：位置、速度、力矩、Kp 均为 0，Kd=1.0。
  // 这种目标不会主动追位置，只会在外力转动时产生阻尼。
  const std::string iface = argc > 1 ? argv[1] : "can0";
  const uint8_t motor_id =
      argc > 2 ? static_cast<uint8_t>(std::strtoul(argv[2], nullptr, 0)) : 1;
  const float position = argc > 3 ? std::strtof(argv[3], nullptr) : 0.0f;
  const float velocity = argc > 4 ? std::strtof(argv[4], nullptr) : 0.0f;
  const float torque = argc > 5 ? std::strtof(argv[5], nullptr) : 0.0f;
  const float kp = argc > 6 ? std::strtof(argv[6], nullptr) : 0.0f;
  const float kd = argc > 7 ? std::strtof(argv[7], nullptr) : 1.0f;
  const uint8_t host_id = argc > 8
                              ? static_cast<uint8_t>(
                                    std::strtoul(argv[8], nullptr, 0))
                              : 0xFF;

  // ---------------------------------------------------------------------------
  // 交互式运控测试流程
  // ---------------------------------------------------------------------------

  try {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    rs01::Rs01Motor motor(iface, motor_id, host_id);
    rs01_examples::TerminalInput input;

    // 运控模式需要连续发送控制帧。先切到运控模式并使能，但默认只发安全帧。
    motor.set_mode(rs01::mode::kMotion);
    motor.enable();

    bool running = false;
    std::cout << "Interactive motion-control test\n"
              << "  target: torque=" << torque << " Nm, position=" << position
              << " rad, velocity=" << velocity << " rad/s, kp=" << kp
              << ", kd=" << kd << "\n"
              << "  r: start sending target frame continuously\n"
              << "  s: send safe zero frame continuously\n"
              << "  q: stop and quit\n";

    while (!g_stop_requested) {
      char key = 0;
      if (input.read_key(key)) {
        if (key == 'r' || key == 'R') {
          running = true;
          std::cout << "running motion target\n";
        } else if (key == 's' || key == 'S') {
          running = false;
          std::cout << "safe zero frame\n";
        } else if (key == 'q' || key == 'Q') {
          break;
        }
      }

      if (running) {
        motor.motion_control(torque, position, velocity, kp, kd);
      } else {
        // 安全帧：零力矩、零位置/速度目标、零刚度和零阻尼，不主动输出控制量。
        motor.motion_control(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "stopping before exit\n";
    for (int i = 0; i < 20; ++i) {
      motor.motion_control(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    motor.disable(false);
  } catch (const std::exception &error) {
    std::cerr << error.what() << "\n";
    return 1;
  }

  return 0;
}
