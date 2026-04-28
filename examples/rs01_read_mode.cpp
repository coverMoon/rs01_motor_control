#include "rs01_motor/protocol.h"
#include "rs01_motor/rs01_motor.h"

#include <cstdlib>
#include <iostream>

/**
 * @brief 读取并打印 RS01 当前 run_mode 参数。
 *
 * @param argc 命令行参数数量。
 * @param argv 命令行参数，支持 [CAN接口] [电机ID]。
 * @return 0 表示读取成功，非 0 表示通信或参数读取失败。
 */
int main(int argc, char **argv) {
  // ---------------------------------------------------------------------------
  // 命令行参数
  // ---------------------------------------------------------------------------

  // 用法：rs01_read_mode [CAN接口] [电机ID]。未指定时默认读取 can0 上 ID=1 的电机。
  const std::string iface = argc > 1 ? argv[1] : "can0";
  const uint8_t motor_id =
      argc > 2 ? static_cast<uint8_t>(std::strtoul(argv[2], nullptr, 0)) : 1;

  // ---------------------------------------------------------------------------
  // 参数读取
  // ---------------------------------------------------------------------------

  try {
    rs01::Rs01Motor motor(iface, motor_id);
    // run_mode 是 uint8 参数，返回值可对照 rs01::mode 中的模式常量。
    auto mode = motor.read_param_u8(rs01::param::kRunMode);
    if (!mode) {
      std::cerr << "No response while reading run_mode\n";
      return 1;
    }
    std::cout << "run_mode = " << static_cast<int>(*mode) << "\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << "\n";
    return 1;
  }

  return 0;
}
