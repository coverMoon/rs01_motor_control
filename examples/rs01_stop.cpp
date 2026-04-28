#include "rs01_motor/protocol.h"
#include "rs01_motor/rs01_motor.h"

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

// -----------------------------------------------------------------------------
// 命令行工具
// -----------------------------------------------------------------------------

/**
 * @brief 判断命令行中是否包含指定选项。
 *
 * @param argc 命令行参数数量。
 * @param argv 命令行参数。
 * @param option 要查找的选项字符串。
 * @return true 表示存在该选项。
 */
bool has_option(int argc, char **argv, const std::string &option) {
  for (int i = 1; i < argc; ++i) {
    if (argv[i] == option) {
      return true;
    }
  }
  return false;
}

} // namespace

/**
 * @brief 尽量清零常见控制目标，并失能 RS01。
 *
 * @param argc 命令行参数数量。
 * @param argv 命令行参数，支持 [CAN接口] [电机ID] [主机ID] [--clear]。
 * @return 0 表示停机命令完成，非 0 表示通信失败。
 */
int main(int argc, char **argv) {
  // ---------------------------------------------------------------------------
  // 命令行参数
  // ---------------------------------------------------------------------------

  // 用法：rs01_stop [CAN接口] [电机ID] [主机ID] [--clear]
  // --clear 会请求电机在失能时清除故障。
  const std::string iface = argc > 1 ? argv[1] : "can0";
  const uint8_t motor_id =
      argc > 2 ? static_cast<uint8_t>(std::strtoul(argv[2], nullptr, 0)) : 1;
  const uint8_t host_id = argc > 3 && argv[3][0] != '-'
                              ? static_cast<uint8_t>(
                                    std::strtoul(argv[3], nullptr, 0))
                              : 0xFF;
  const bool clear_error = has_option(argc, argv, "--clear");

  // ---------------------------------------------------------------------------
  // 停机流程
  // ---------------------------------------------------------------------------

  try {
    rs01::Rs01Motor motor(iface, motor_id, host_id);

    // 这些写入可能因当前模式不同而不是全部必要，但尽量清掉常见目标值。
    motor.write_param_float(rs01::param::kSpeedRef, 0.0f);
    motor.write_param_float(rs01::param::kIqRef, 0.0f);
    motor.disable(clear_error);

    std::cout << "RS01 stopped: interface=" << iface
              << ", motor_id=" << static_cast<int>(motor_id)
              << ", host_id=0x" << std::hex << static_cast<int>(host_id)
              << std::dec << ", clear_error=" << (clear_error ? "true" : "false")
              << "\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << "\n";
    return 1;
  }

  return 0;
}
