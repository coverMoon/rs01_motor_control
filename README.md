# RS01 Motor Control

RS01 专用 Linux SocketCAN 底层控制库。

## Scope

当前框架只面向 RobStride RS01，使用手册第 4 章私有 CAN 扩展帧协议，不做 RS02/CanOpen/MIT 标准帧抽象。

固定 RS01 范围：

- Position: `+-4*pi rad`
- Velocity: `+-44 rad/s`
- Torque: `+-17 Nm`
- Kp: `0~500`
- Kd: `0~5`
- Current: `+-23 A`

## Build

```bash
cd /home/windnotebook/PROJECT/RoboCon/Tools/rs01_motor_control
cmake -S . -B build
cmake --build build -j
```

Clean rebuild:

```bash
rm -rf build
cmake -S . -B build
cmake --build build -j
```

## CAN Setup

```bash
ip link
lsusb
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
```

If `can0` is already configured:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

Watch CAN frames:

```bash
candump can0
```

## USB-CAN

推荐使用支持 Linux SocketCAN 的 USB-CAN 模块。

已验证主机可识别：

```text
PEAK System PCAN-USB
```

插入后应能看到 `can0`：

```bash
ip link
lsusb
```

常见可选模块：

- PEAK PCAN-USB
- CANable / CandleLight 固件设备
- 其他明确支持 SocketCAN 的 USB-CAN

不建议优先使用只暴露 `/dev/ttyUSB*` 的串口协议 CAN 盒。那类设备通常需要厂家串口协议，本库不能直接使用。

接线注意：

- USB-CAN `CAN_H` 接 RS01 `CAN_H`
- USB-CAN `CAN_L` 接 RS01 `CAN_L`
- RS01 需要单独供电
- 按模块要求确认 GND 是否需要共地
- 总线两端需要合适的 120Ω 终端电阻
- 波特率使用 `1000000`

如果看得到 USB 设备但没有 `can0`，先检查驱动和内核模块：

```bash
lsusb
ip link
dmesg | tail -50
lsmod | grep -E 'can|peak|gs_usb'
```

## Examples

```bash
./build/rs01_read_mode can0 1
./build/rs01_velocity_test can0 1 0.5
```

`rs01_velocity_test` 默认使用保守参数：`current_limit=1.0A`，`acc=2.0rad/s^2`。

Run with another motor ID:

```bash
./build/rs01_read_mode can0 11
./build/rs01_velocity_test can0 11 0.5
```

Stop CAN:

```bash
sudo ip link set can0 down
```

## Status

当前不是完整驱动包，是 RS01 底层库框架和最小可测版本。

已完成：

- SocketCAN 打开、发送、超时接收
- RS01 私有扩展帧 ID 打包
- 使能、失能、设置零点
- 参数 `uint8` / `float` 读写
- 反馈帧位置、速度、力矩、温度解析
- 运控模式帧打包
- 速度模式基础控制
- `rs01_read_mode` 和 `rs01_velocity_test` 示例

待补充：

- 电流模式 `iq_ref`
- 位置模式 PP
- 位置模式 CSP
- 主动上报类型 24
- 保存参数类型 22
- CAN ID 修改和安全确认流程
- 版本读取、设备 ID 读取
- 完整 fault/warning 位解析
- 更多只读诊断示例
- ROS2 node 封装

## Safety Notes

- RS01 需要单独供电，USB-CAN 不给电机供电。
- 初次测试先用小速度、小电流限制。
- 模式切换前先失能，切换后再使能。
- 如果没有回包，先检查电源、CAN_H/CAN_L、波特率、ID、终端电阻。
