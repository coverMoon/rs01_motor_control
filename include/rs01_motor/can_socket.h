#pragma once

#include <cstdint>
#include <linux/can.h>
#include <string>

namespace rs01 {

// Linux SocketCAN RAW socket 的最小 RAII 封装。
// 该类只负责打开接口、发送扩展帧、带超时接收一帧，不理解 RS01 协议内容。
class CanSocket {
public:
  // ---------------------------------------------------------------------------
  // 生命周期
  // ---------------------------------------------------------------------------

  CanSocket() = default;
  explicit CanSocket(std::string interface);
  ~CanSocket();

  CanSocket(const CanSocket &) = delete;
  CanSocket &operator=(const CanSocket &) = delete;

  CanSocket(CanSocket &&other) noexcept;
  CanSocket &operator=(CanSocket &&other) noexcept;

  // ---------------------------------------------------------------------------
  // 连接管理
  // ---------------------------------------------------------------------------

  // 打开并绑定到指定 CAN 网络接口，例如 "can0"。
  void open(const std::string &interface);
  void close();

  bool is_open() const;

  // ---------------------------------------------------------------------------
  // 帧收发
  // ---------------------------------------------------------------------------

  // 发送 classic CAN 数据帧，并强制设置扩展帧标志。RS01 私有协议使用 29 bit CAN ID。
  void send_extended(uint32_t can_id, const uint8_t *data, uint8_t dlc);

  // 在 timeout_ms 内等待一帧。超时返回 false；收到任何 CAN 帧都直接返回给上层过滤。
  bool receive(can_frame &frame, int timeout_ms);

private:
  int fd_ = -1;
  std::string interface_;
};

} // namespace rs01
