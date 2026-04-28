#!/usr/bin/env bash
set -euo pipefail

# -----------------------------------------------------------------------------
# RS01 电机 ID 扫描脚本
# -----------------------------------------------------------------------------
#
# 用法：
#   ./scripts/scan_rs01_ids.sh [CAN接口] [起始ID] [结束ID] [主机ID]
#
# 示例：
#   ./scripts/scan_rs01_ids.sh
#   ./scripts/scan_rs01_ids.sh can0 1 127 0xff
#
# 说明：
# 脚本只调用 rs01_read_mode 读取 run_mode 参数，不会使能电机，也不会写入参数。

CAN_IF="${1:-can0}"
START_ID="${2:-1}"
END_ID="${3:-127}"
HOST_ID="${4:-0xff}"
READER="./bin/rs01_read_mode"

if [[ ! -x "${READER}" ]]; then
  echo "找不到 ${READER}，请先执行：cmake --build build -j" >&2
  exit 1
fi

echo "扫描 ${CAN_IF} 上的 RS01 ID：${START_ID}..${END_ID}，host_id=${HOST_ID}"

found=0
for id in $(seq "${START_ID}" "${END_ID}"); do
  output="$("${READER}" "${CAN_IF}" "${id}" "${HOST_ID}" 2>&1)" && status=0 || status=$?
  if [[ "${status}" -eq 0 ]]; then
    echo "ID=${id}: ${output}"
    found=1
  else
    echo "ID=${id}: no response"
  fi
done

if [[ "${found}" -eq 0 ]]; then
  echo "未发现响应的 RS01。请检查电机供电、CAN_H/CAN_L、GND、终端电阻、波特率和协议模式。" >&2
  exit 1
fi
