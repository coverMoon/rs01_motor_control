#pragma once

#include <cstdint>
#include <linux/can.h>
#include <string>

namespace rs01 {

class CanSocket {
public:
  CanSocket() = default;
  explicit CanSocket(std::string interface);
  ~CanSocket();

  CanSocket(const CanSocket &) = delete;
  CanSocket &operator=(const CanSocket &) = delete;

  CanSocket(CanSocket &&other) noexcept;
  CanSocket &operator=(CanSocket &&other) noexcept;

  void open(const std::string &interface);
  void close();

  bool is_open() const;
  void send_extended(uint32_t can_id, const uint8_t *data, uint8_t dlc);
  bool receive(can_frame &frame, int timeout_ms);

private:
  int fd_ = -1;
  std::string interface_;
};

} // namespace rs01
