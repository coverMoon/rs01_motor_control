#include "rs01_motor/protocol.h"
#include "rs01_motor/rs01_motor.h"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

/**
 * @brief 对 RS01 执行一次保守的速度模式连通性测试。
 *
 * @param argc 命令行参数数量。
 * @param argv 命令行参数，支持 [CAN接口] [电机ID] [目标速度rad/s]。
 * @return 0 表示测试流程完成，非 0 表示通信或控制命令失败。
 */
int main(int argc, char **argv) {
  // ---------------------------------------------------------------------------
  // 命令行参数
  // ---------------------------------------------------------------------------

  // 用法：rs01_velocity_test [CAN接口] [电机ID] [目标速度rad/s]。
  // 未指定时默认使用 can0、ID=1、0.5rad/s，适合作为第一次低速连通性测试。
  const std::string iface = argc > 1 ? argv[1] : "can0";
  const uint8_t motor_id =
      argc > 2 ? static_cast<uint8_t>(std::strtoul(argv[2], nullptr, 0)) : 1;
  const float velocity = argc > 3 ? std::strtof(argv[3], nullptr) : 0.5f;

  // ---------------------------------------------------------------------------
  // 低速测试流程
  // ---------------------------------------------------------------------------

  try {
    rs01::Rs01Motor motor(iface, motor_id);
    // 首次硬件测试使用较低电流限制和较小加速度，降低突然动作的风险。
    motor.velocity_control(velocity, 1.0f, 2.0f);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    // 先把速度目标写回 0，等待电机有时间降速，再执行失能。
    motor.write_param_float(rs01::param::kSpeedRef, 0.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    motor.disable(false);
  } catch (const std::exception &error) {
    std::cerr << error.what() << "\n";
    return 1;
  }

  return 0;
}
