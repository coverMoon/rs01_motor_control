#!/usr/bin/env bash
set -euo pipefail

# -----------------------------------------------------------------------------
# SocketCAN 快速配置脚本
# -----------------------------------------------------------------------------
#
# 用法：
#   sudo ./scripts/setup_can.sh [CAN接口] [波特率]
#
# 示例：
#   sudo ./scripts/setup_can.sh
#   sudo ./scripts/setup_can.sh can0 1000000
#
# 说明：
# USB-CAN 重新插拔后，Linux 通常会重新创建 can0，此时 bitrate/up 状态不会保留。
# 该脚本把常用的 down -> set bitrate -> up -> show 流程封装成一条命令。

CAN_IF="${1:-can0}"
BITRATE="${2:-1000000}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "请使用 sudo 运行，例如：sudo $0 ${CAN_IF} ${BITRATE}" >&2
  exit 1
fi

if ! ip link show "${CAN_IF}" >/dev/null 2>&1; then
  echo "找不到 CAN 接口：${CAN_IF}" >&2
  echo "请先插入 USB-CAN，并用 ip link 确认接口名。" >&2
  exit 1
fi

# 接口可能已经处于 up 状态；先 down 可以避免重复设置 bitrate 时报错。
ip link set "${CAN_IF}" down 2>/dev/null || true
ip link set "${CAN_IF}" type can bitrate "${BITRATE}"
ip link set "${CAN_IF}" up

echo "已配置 ${CAN_IF}，bitrate=${BITRATE}"
ip -details link show "${CAN_IF}"
