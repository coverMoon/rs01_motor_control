#pragma once

#include <cstdint>
#include <linux/can.h>
#include <string>

namespace rs01 {

// Linux SocketCAN RAW socket 的最小 RAII 封装。
//
// 该类是纯传输层工具，只负责：
// - 打开/关闭 CAN 网络接口；
// - 发送 classic CAN 扩展帧；
// - 带超时接收一帧原始 can_frame。
//
// 它不理解 RS01 协议字段，也不做电机 ID 或通信类型过滤；这些逻辑放在 Rs01Motor。
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

  // 发送 classic CAN 数据帧，并强制设置扩展帧标志。
  // can_id 参数应为未带 CAN_EFF_FLAG 的 29 bit 扩展帧 ID。
  void send_extended(uint32_t can_id, const uint8_t *data, uint8_t dlc);

  // 在 timeout_ms 内等待一帧。超时返回 false；收到任何 CAN 帧都直接返回给上层过滤。
  bool receive(can_frame &frame, int timeout_ms);

private:
  int fd_ = -1;
  std::string interface_;
};

} // namespace rs01
