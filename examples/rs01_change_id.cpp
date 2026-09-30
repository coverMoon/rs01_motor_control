#include "rs01_motor/protocol.h"
#include "rs01_motor/rs01_motor.h"

#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

uint8_t parse_id(const char *text, uint8_t min_value, uint8_t max_value) {
  const std::string input(text);
  std::size_t parsed = 0;
  const unsigned long value = std::stoul(input, &parsed, 0);
  if (parsed != input.size() || value < min_value || value > max_value) {
    throw std::invalid_argument("ID out of range or invalid: " + input);
  }
  return static_cast<uint8_t>(value);
}

} // namespace

int main(int argc, char **argv) {
  if (argc != 4 && argc != 5) {
    std::cerr << "用法: rs01_change_id <CAN接口> <旧ID 1..127> "
                 "<新ID 1..127> [主机ID, 默认0xff]\n";
    return 2;
  }

  try {
    const std::string iface = argv[1];
    const uint8_t old_id = parse_id(argv[2], 1, 127);
    const uint8_t new_id = parse_id(argv[3], 1, 127);
    const uint8_t host_id = argc == 5 ? parse_id(argv[4], 0, 255) : 0xFF;
    if (old_id == new_id) {
      throw std::invalid_argument("新旧 ID 相同");
    }

    rs01::Rs01Motor source(iface, old_id, host_id);
    if (!source.read_param_u8(rs01::param::kRunMode)) {
      throw std::runtime_error("旧 ID 无响应，未发送改 ID 命令");
    }
    rs01::Rs01Motor target(iface, new_id, host_id);
    if (target.read_param_u8(rs01::param::kRunMode)) {
      throw std::runtime_error("新 ID 已有电机响应，未发送改 ID 命令");
    }

    std::cout << "RS01 " << static_cast<int>(old_id) << " -> "
              << static_cast<int>(new_id) << "，发送类型 7 命令...\n";
    const bool acknowledged = source.change_id(new_id);
    // 某些固件可能改号成功但广播应答丢失；最终以新 ID 能否读回为准。
    for (int attempt = 0; attempt < 3; ++attempt) {
      if (target.read_param_u8(rs01::param::kRunMode)) {
        std::cout << "新 ID " << static_cast<int>(new_id)
                  << " 读取成功，修改完成"
                  << (acknowledged ? "" : "（未收到广播应答）") << "\n";
        return 0;
      }
    }

    std::cerr << "未能通过新 ID 读回；"
              << (acknowledged ? "已收到改号广播应答" : "未收到改号广播应答")
              << "。请检查新旧 ID，避免盲目重复发送。\n";
    return 1;
  } catch (const std::exception &error) {
    std::cerr << error.what() << "\n";
    return 1;
  }
}
