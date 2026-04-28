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

// -----------------------------------------------------------------------------
// 生命周期
// -----------------------------------------------------------------------------

/**
 * @brief 构造并立即打开指定 SocketCAN 接口。
 *
 * @param interface CAN 网络接口名，例如 "can0"。
 */
CanSocket::CanSocket(std::string interface) { open(interface); }

/**
 * @brief 析构时关闭已打开的 CAN socket。
 */
CanSocket::~CanSocket() { close(); }

/**
 * @brief 移动构造，接管另一个 CanSocket 的文件描述符。
 *
 * @param other 被移动的对象，移动后不再持有 socket。
 */
CanSocket::CanSocket(CanSocket &&other) noexcept
    : fd_(other.fd_), interface_(std::move(other.interface_)) {
  // 移动构造时转移文件描述符所有权，避免两个对象析构时重复 close 同一个 fd。
  other.fd_ = -1;
}

/**
 * @brief 移动赋值，先释放当前 socket，再接管另一个对象的 socket。
 *
 * @param other 被移动的对象。
 * @return 当前对象引用。
 */
CanSocket &CanSocket::operator=(CanSocket &&other) noexcept {
  if (this != &other) {
    close();
    fd_ = other.fd_;
    interface_ = std::move(other.interface_);
    other.fd_ = -1;
  }
  return *this;
}

// -----------------------------------------------------------------------------
// 连接管理
// -----------------------------------------------------------------------------

/**
 * @brief 打开并绑定到指定 SocketCAN 接口。
 *
 * @param interface CAN 网络接口名，例如 "can0"。
 * @throws std::runtime_error 创建 socket、查询接口或绑定失败时抛出。
 */
void CanSocket::open(const std::string &interface) {
  close();

  // CAN_RAW 直接暴露 Linux SocketCAN 的 can_frame，适合这种自定义协议的底层收发。
  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    throw std::runtime_error("failed to create CAN socket");
  }

  // bind() 需要接口索引，因此先把 "can0" 这类接口名查询成内核 ifindex。
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

/**
 * @brief 关闭当前 socket；如果尚未打开则不做任何事。
 */
void CanSocket::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

/**
 * @brief 查询当前对象是否持有有效 socket。
 *
 * @return true 表示 socket 已打开。
 */
bool CanSocket::is_open() const { return fd_ >= 0; }

// -----------------------------------------------------------------------------
// 帧收发
// -----------------------------------------------------------------------------

/**
 * @brief 发送一帧 RS01 使用的扩展 CAN 数据帧。
 *
 * @param can_id 未带 CAN_EFF_FLAG 的 29 bit 扩展帧 ID。
 * @param data 数据区指针；dlc 为 0 时可为 nullptr。
 * @param dlc 数据长度，classic CAN 最大为 8。
 * @throws std::runtime_error socket 未打开、dlc 超限或发送失败时抛出。
 */
void CanSocket::send_extended(uint32_t can_id, const uint8_t *data,
                              uint8_t dlc) {
  if (fd_ < 0) {
    throw std::runtime_error("CAN socket is not open");
  }
  if (dlc > 8) {
    throw std::runtime_error("CAN dlc exceeds 8 bytes");
  }

  can_frame frame {};
  // RS01 私有协议使用 29 bit 扩展帧 ID；上层传入的是未带 CAN_EFF_FLAG 的原始 ID。
  frame.can_id = (can_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  frame.can_dlc = dlc;
  if (data != nullptr && dlc > 0) {
    std::memcpy(frame.data, data, dlc);
  }

  if (write(fd_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
    throw std::runtime_error("failed to send CAN frame");
  }
}

/**
 * @brief 带超时等待并读取一帧 CAN 数据。
 *
 * @param frame 输出的 CAN 帧。
 * @param timeout_ms 等待超时时间，单位毫秒。
 * @return true 表示成功读取一帧；false 表示超时或读取长度不完整。
 * @throws std::runtime_error socket 未打开或 select 失败时抛出。
 */
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

  // 使用 select() 做毫秒级超时等待，不需要把 socket 改成非阻塞模式。
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
