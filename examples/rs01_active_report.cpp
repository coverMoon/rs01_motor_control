#include "rs01_motor/rs01_motor.h"

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

// -----------------------------------------------------------------------------
// 命令行工具
// -----------------------------------------------------------------------------

/**
 * @brief 把 on/off 参数转换为布尔值。
 *
 * @param value 命令行字符串，支持 on/off/1/0。
 * @param enable 输出的开关状态。
 * @return true 表示解析成功。
 */
bool parse_switch(const std::string &value, bool &enable) {
  if (value == "on" || value == "1") {
    enable = true;
    return true;
  }
  if (value == "off" || value == "0") {
    enable = false;
    return true;
  }
  return false;
}

} // namespace

/**
 * @brief 开启或关闭 RS01 主动上报。
 *
 * @param argc 命令行参数数量。
 * @param argv 命令行参数，支持 [CAN接口] [电机ID] [on|off] [主机ID]。
 * @return 0 表示设置成功，非 0 表示通信失败或参数错误。
 */
int main(int argc, char **argv) {
  // ---------------------------------------------------------------------------
  // 命令行参数
  // ---------------------------------------------------------------------------

  // 用法：rs01_active_report [CAN接口] [电机ID] [on|off] [主机ID]
  const std::string iface = argc > 1 ? argv[1] : "can0";
  const uint8_t motor_id =
      argc > 2 ? static_cast<uint8_t>(std::strtoul(argv[2], nullptr, 0)) : 1;
  const std::string switch_value = argc > 3 ? argv[3] : "on";
  const uint8_t host_id = argc > 4
                              ? static_cast<uint8_t>(
                                    std::strtoul(argv[4], nullptr, 0))
                              : 0xFF;

  bool enable = true;
  if (!parse_switch(switch_value, enable)) {
    std::cerr << "第三个参数必须是 on/off/1/0\n";
    return 1;
  }

  // ---------------------------------------------------------------------------
  // 主动上报配置
  // ---------------------------------------------------------------------------

  try {
    rs01::Rs01Motor motor(iface, motor_id, host_id);
    const bool acknowledged = motor.set_active_report(enable);

    std::cout << "RS01 active report " << (enable ? "enabled" : "disabled")
              << ": interface=" << iface
              << ", motor_id=" << static_cast<int>(motor_id)
              << ", host_id=0x" << std::hex << static_cast<int>(host_id)
              << std::dec << "\n";
    if (!acknowledged) {
      std::cerr << "warning: command sent, but no active-report response was received\n";
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << "\n";
    return 1;
  }

  return 0;
}
