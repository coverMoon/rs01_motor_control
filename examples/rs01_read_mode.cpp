#include "rs01_motor/protocol.h"
#include "rs01_motor/rs01_motor.h"

#include <cstdlib>
#include <iostream>

int main(int argc, char **argv) {
  const std::string iface = argc > 1 ? argv[1] : "can0";
  const uint8_t motor_id =
      argc > 2 ? static_cast<uint8_t>(std::strtoul(argv[2], nullptr, 0)) : 1;

  try {
    rs01::Rs01Motor motor(iface, motor_id);
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
