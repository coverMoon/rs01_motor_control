#include "rs01_motor/protocol.h"
#include "rs01_motor/rs01_motor.h"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

int main(int argc, char **argv) {
  const std::string iface = argc > 1 ? argv[1] : "can0";
  const uint8_t motor_id =
      argc > 2 ? static_cast<uint8_t>(std::strtoul(argv[2], nullptr, 0)) : 1;
  const float velocity = argc > 3 ? std::strtof(argv[3], nullptr) : 0.5f;

  try {
    rs01::Rs01Motor motor(iface, motor_id);
    motor.velocity_control(velocity, 1.0f, 2.0f);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    motor.write_param_float(rs01::param::kSpeedRef, 0.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    motor.disable(false);
  } catch (const std::exception &error) {
    std::cerr << error.what() << "\n";
    return 1;
  }

  return 0;
}
