#pragma once

#include "rs01_motor/can_socket.h"

#include <cstdint>
#include <optional>
#include <string>

namespace rs01 {

struct Feedback {
  float position = 0.0f;
  float velocity = 0.0f;
  float torque = 0.0f;
  float temperature = 0.0f;
  uint8_t mode = 0;
  uint8_t fault = 0;
};

class Rs01Motor {
public:
  Rs01Motor(std::string can_interface, uint8_t motor_id,
            uint8_t host_id = 0xFF);

  void enable();
  void disable(bool clear_error = false);
  void set_zero();

  std::optional<Feedback> read_feedback(int timeout_ms = 100);

  void write_param_u8(uint16_t index, uint8_t value);
  void write_param_float(uint16_t index, float value);
  std::optional<float> read_param_float(uint16_t index, int timeout_ms = 100);
  std::optional<uint8_t> read_param_u8(uint16_t index, int timeout_ms = 100);

  void set_mode(uint8_t mode);
  void motion_control(float torque, float position, float velocity, float kp,
                      float kd);
  void velocity_control(float velocity, float current_limit = 1.0f,
                        float acceleration = 2.0f);

private:
  static float clamp(float value, float low, float high);
  static uint16_t float_to_uint(float value, float min, float max);
  static float uint_to_float(uint16_t value, float min, float max);

  void send_empty(uint32_t communication_type);
  void expect_status();

  CanSocket socket_;
  uint8_t motor_id_;
  uint8_t host_id_;
};

} // namespace rs01
