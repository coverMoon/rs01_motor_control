#include "rs01_motor/can_socket.h"

#include <cstring>
#include <stdexcept>
#include <string>

#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

namespace rs01 {

CanSocket::CanSocket(std::string interface) { open(interface); }

CanSocket::~CanSocket() { close(); }

CanSocket::CanSocket(CanSocket &&other) noexcept
    : fd_(other.fd_), interface_(std::move(other.interface_)) {
  other.fd_ = -1;
}

CanSocket &CanSocket::operator=(CanSocket &&other) noexcept {
  if (this != &other) {
    close();
    fd_ = other.fd_;
    interface_ = std::move(other.interface_);
    other.fd_ = -1;
  }
  return *this;
}

void CanSocket::open(const std::string &interface) {
  close();

  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    throw std::runtime_error("failed to create CAN socket");
  }

  ifreq ifr {};
  std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
  if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    close();
    throw std::runtime_error("failed to query CAN interface: " + interface);
  }

  sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    close();
    throw std::runtime_error("failed to bind CAN interface: " + interface);
  }

  interface_ = interface;
}

void CanSocket::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool CanSocket::is_open() const { return fd_ >= 0; }

void CanSocket::send_extended(uint32_t can_id, const uint8_t *data,
                              uint8_t dlc) {
  if (fd_ < 0) {
    throw std::runtime_error("CAN socket is not open");
  }
  if (dlc > 8) {
    throw std::runtime_error("CAN dlc exceeds 8 bytes");
  }

  can_frame frame {};
  frame.can_id = (can_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  frame.can_dlc = dlc;
  if (data != nullptr && dlc > 0) {
    std::memcpy(frame.data, data, dlc);
  }

  if (write(fd_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
    throw std::runtime_error("failed to send CAN frame");
  }
}

bool CanSocket::receive(can_frame &frame, int timeout_ms) {
  if (fd_ < 0) {
    throw std::runtime_error("CAN socket is not open");
  }

  fd_set read_set;
  FD_ZERO(&read_set);
  FD_SET(fd_, &read_set);

  timeval timeout {};
  timeout.tv_sec = timeout_ms / 1000;
  timeout.tv_usec = (timeout_ms % 1000) * 1000;

  int ready = select(fd_ + 1, &read_set, nullptr, nullptr, &timeout);
  if (ready < 0) {
    throw std::runtime_error("failed while waiting for CAN frame");
  }
  if (ready == 0) {
    return false;
  }

  return read(fd_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
}

} // namespace rs01
