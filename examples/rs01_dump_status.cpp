#include "rs01_motor/protocol.h"
#include "rs01_motor/rs01_motor.h"

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {

// -----------------------------------------------------------------------------
// 打印工具
// -----------------------------------------------------------------------------

/**
 * @brief 打印 uint8 参数读取结果。
 *
 * @param name 参数显示名称。
 * @param value 参数读取结果；无响应时为空。
 */
void print_u8(const std::string &name, const std::optional<uint8_t> &value) {
  std::cout << std::left << std::setw(22) << name << ": ";
  if (value) {
    std::cout << static_cast<int>(*value);
  } else {
    std::cout << "no response";
  }
  std::cout << "\n";
}

/**
 * @brief 打印 float 参数读取结果。
 *
 * @param name 参数显示名称。
 * @param value 参数读取结果；无响应时为空。
 * @param unit 参数单位，用于输出提示。
 */
void print_float(const std::string &name, const std::optional<float> &value,
                 const std::string &unit = "") {
  std::cout << std::left << std::setw(22) << name << ": ";
  if (value) {
    std::cout << std::fixed << std::setprecision(4) << *value;
    if (!unit.empty()) {
      std::cout << " " << unit;
    }
  } else {
    std::cout << "no response";
  }
  std::cout << "\n";
}

/**
 * @brief 打印 bitmask 参数及其中文解析结果。
 *
 * @param name 参数显示名称。
 * @param value 参数读取结果；无响应时为空。
 * @param descriptions 置位 bit 对应的中文说明。
 */
void print_bitmask(const std::string &name, const std::optional<uint32_t> &value,
                   const std::vector<std::string> &descriptions) {
  std::cout << std::left << std::setw(22) << name << ": ";
  if (!value) {
    std::cout << "no response\n";
    return;
  }

  std::cout << "0x" << std::hex << *value << std::dec;
  if (descriptions.empty()) {
    std::cout << " (normal)";
  } else {
    std::cout << " (";
    for (std::size_t i = 0; i < descriptions.size(); ++i) {
      if (i > 0) {
        std::cout << "; ";
      }
      std::cout << descriptions[i];
    }
    std::cout << ")";
  }
  std::cout << "\n";
}

} // namespace

/**
 * @brief 读取并打印 RS01 常用只读/配置参数。
 *
 * @param argc 命令行参数数量。
 * @param argv 命令行参数，支持 [CAN接口] [电机ID] [主机ID]。
 * @return 0 表示流程完成，非 0 表示通信或参数读取过程异常。
 */
int main(int argc, char **argv) {
  // ---------------------------------------------------------------------------
  // 命令行参数
  // ---------------------------------------------------------------------------

  // 用法：rs01_dump_status [CAN接口] [电机ID] [主机ID]。
  // 未指定时默认读取 can0 上 ID=1 的电机，主机 ID 默认 0xFF。
  const std::string iface = argc > 1 ? argv[1] : "can0";
  const uint8_t motor_id =
      argc > 2 ? static_cast<uint8_t>(std::strtoul(argv[2], nullptr, 0)) : 1;
  const uint8_t host_id = argc > 3
                              ? static_cast<uint8_t>(
                                    std::strtoul(argv[3], nullptr, 0))
                              : 0xFF;

  // ---------------------------------------------------------------------------
  // 只读诊断
  // ---------------------------------------------------------------------------

  try {
    rs01::Rs01Motor motor(iface, motor_id, host_id);

    std::cout << "RS01 status dump\n"
              << "interface=" << iface << ", motor_id="
              << static_cast<int>(motor_id)
              << ", host_id=0x" << std::hex << static_cast<int>(host_id)
              << std::dec << "\n\n";

    print_u8("run_mode", motor.read_param_u8(rs01::param::kRunMode));
    print_float("mech_pos", motor.read_param_float(rs01::param::kMechPos),
                "rad");
    print_float("mech_vel", motor.read_param_float(rs01::param::kMechVel),
                "rad/s");
    print_float("iq_filtered",
                motor.read_param_float(rs01::param::kIqFiltered), "A");
    print_float("vbus", motor.read_param_float(rs01::param::kVbus), "V");
    print_float("limit_current",
                motor.read_param_float(rs01::param::kLimitCurrent), "A");
    print_float("speed_ref", motor.read_param_float(rs01::param::kSpeedRef),
                "rad/s");
    print_float("velocity_acc",
                motor.read_param_float(rs01::param::kVelocityAcceleration),
                "rad/s^2");
    print_float("report_period",
                motor.read_param_float(rs01::param::kReportPeriod), "ms");

    const auto fault_status = motor.read_param_u32(rs01::param::kFaultStatus);
    const auto warning_status =
        motor.read_param_u32(rs01::param::kWarningStatus);
    const auto driver_fault = motor.read_param_u32(rs01::param::kDriverFault);

    print_bitmask("fault_status", fault_status,
                  fault_status ? rs01::Rs01Motor::describe_fault_bits(
                                     *fault_status)
                               : std::vector<std::string> {});
    print_bitmask("warning_status", warning_status,
                  warning_status ? rs01::Rs01Motor::describe_warning_bits(
                                       *warning_status)
                                 : std::vector<std::string> {});
    print_bitmask("driver_fault", driver_fault, {});
  } catch (const std::exception &error) {
    std::cerr << error.what() << "\n";
    return 1;
  }

  return 0;
}
