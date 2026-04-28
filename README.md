# RS01 电机控制库

RS01 专用 Linux SocketCAN 底层控制库和调试工具集。

当前项目只面向 RobStride RS01，使用手册第 4 章私有 CAN 扩展帧协议，不做 RS02、CanOpen、MIT 标准帧抽象。

固定 RS01 范围：

- 位置：`+-4*pi rad`
- 速度：`+-44 rad/s`
- 力矩：`+-17 Nm`
- 刚度系数 Kp：`0~500`
- 阻尼系数 Kd：`0~5`
- 电流：`+-23 A`

编译后的示例工具统一生成到项目根目录的 `bin/`。

## 快速开始

编译：

```bash
cd /home/windnotebook/PROJECT/RoboCon/Tools/rs01_motor_control
cmake -S . -B build
cmake --build build -j
```

配置 CAN：

```bash
sudo ./scripts/setup_can.sh
```

只读连通性测试：

```bash
./bin/rs01_read_mode can0 1
./bin/rs01_dump_status can0 1
```

调试后停机：

```bash
./bin/rs01_stop can0 1
```

如果不知道电机 ID，可以先扫描：

```bash
./scripts/scan_rs01_ids.sh can0 1 127
```

## 安全说明

- RS01 需要单独供电，USB-CAN 不给电机供电。
- 初次测试先用小速度、小电流、小位移和低刚度。
- 运行控制类工具前，确认电机固定方式、负载、供电、机械限位和急停手段。
- `rs01_velocity_test`、`rs01_current_test`、`rs01_motion_test`、`rs01_position_pp_test`、`rs01_position_csp_test` 都会使能电机。
- `rs01_read_mode`、`rs01_dump_status`、`rs01_monitor`、`scan_rs01_ids.sh` 不会使能电机。
- `rs01_active_report` 会写主动上报配置，但不会使能电机或写运动目标。
- 如果没有回包，先检查电源、CAN_H/CAN_L、GND、波特率、ID 和终端电阻。

## 编译

常规编译：

```bash
cmake -S . -B build
cmake --build build -j
```

清理后重新编译：

```bash
rm -rf build
cmake -S . -B build
cmake --build build -j
```

查看生成的工具：

```bash
ls bin
```

## CAN 配置

USB-CAN 每次断电或重新插拔后，`can0` 通常会被重新创建，之前设置的 bitrate 和 up 状态不会保留。

推荐使用脚本恢复配置：

```bash
sudo ./scripts/setup_can.sh
```

默认配置 `can0`，波特率 `1000000`。如果接口名或波特率不同：

```bash
sudo ./scripts/setup_can.sh can1 1000000
```

脚本内部等价于执行“先关闭接口、设置波特率、打开接口、显示详细状态”。

手动配置流程：

```bash
ip link
lsusb
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
```

如果 `can0` 已经配置过，需要先 down 再重新设置：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

监听原始 CAN 帧：

```bash
candump can0
```

## USB-CAN 和接线

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

## 工具总览

| 工具 | 类型 | 写参数 | 使能电机 | 用途 |
|---|---|---:|---:|---|
| `rs01_read_mode` | 只读 | 否 | 否 | 读取 `run_mode`，确认基础通信 |
| `rs01_dump_status` | 只读 | 否 | 否 | 读取常用诊断参数和故障位 |
| `rs01_monitor` | 监听 | 否 | 否 | 被动监听反馈帧/主动上报帧 |
| `scan_rs01_ids.sh` | 只读 | 否 | 否 | 扫描可响应的电机 ID |
| `rs01_active_report` | 配置 | 是 | 否 | 开启或关闭主动上报 |
| `rs01_velocity_test` | 控制 | 是 | 是 | 速度模式交互测试 |
| `rs01_current_test` | 控制 | 是 | 是 | 电流模式交互测试 |
| `rs01_motion_test` | 控制 | 是 | 是 | 运控模式交互测试 |
| `rs01_position_pp_test` | 控制 | 是 | 是 | PP 位置模式交互测试 |
| `rs01_position_csp_test` | 控制 | 是 | 是 | CSP 位置模式交互测试 |
| `rs01_stop` | 安全 | 是 | 否 | 清零常见目标并失能 |

## 只读和监听工具

### `rs01_read_mode`

读取电机当前运行模式 `run_mode`。这是最基础的只读连通性测试，不会使能电机，也不会写入任何参数。

```bash
./bin/rs01_read_mode [CAN接口] [电机ID] [主机ID]
```

常用示例：

```bash
./bin/rs01_read_mode can0 1
./bin/rs01_read_mode can0 1 0xfd
./bin/rs01_read_mode can0 11
```

第三个参数是可选主机 ID，默认值为 `0xff`。

### `rs01_dump_status`

读取并打印常用诊断参数，适合确认电机当前状态、电源电压、限流参数和故障状态。该工具只读参数，不会使能电机，也不会写入参数。

当前会读取：

- `run_mode`
- `mech_pos`
- `mech_vel`
- `iq_filtered`
- `vbus`
- `limit_current`
- `speed_ref`
- `velocity_acc`
- `report_period`
- `fault_status`
- `warning_status`
- `driver_fault`

```bash
./bin/rs01_dump_status [CAN接口] [电机ID] [主机ID]
```

常用示例：

```bash
./bin/rs01_dump_status can0 1
./bin/rs01_dump_status can0 1 0xfd
```

输出中的 `fault_status`、`warning_status` 会显示原始位掩码，并尽量解析成中文故障说明。

### `rs01_monitor`

被动实时监听工具，用于打印总线上的 RS01 反馈帧。它只打开 CAN 接口并接收帧，不会发送任何 CAN 命令，也不会使能电机或写参数。

适用场景：

- 运行速度、运控、PP、CSP 等控制工具时，在另一个终端观察反馈
- 电机已经开启主动上报时，持续查看位置、速度、力矩、温度和故障位

```bash
./bin/rs01_monitor [CAN接口] [电机ID]
```

常用示例：

```bash
./bin/rs01_monitor can0 1
./bin/rs01_monitor can0
```

不指定电机 ID 时，会显示总线上所有 RS01 反馈帧。如果总线上没有反馈帧，`rs01_monitor` 会保持等待，不会主动请求电机返回状态。

### `scan_rs01_ids.sh`

扫描电机 ID。脚本会逐个调用 `rs01_read_mode` 读取 `run_mode`，属于只读请求，不会使能电机，也不会写入参数。

```bash
./scripts/scan_rs01_ids.sh [CAN接口] [起始ID] [结束ID] [主机ID]
```

常用示例：

```bash
./scripts/scan_rs01_ids.sh can0 1 127
./scripts/scan_rs01_ids.sh can0 1 127 0xfd
```

## 配置工具

### `rs01_active_report`

开启或关闭 RS01 主动上报。开启后，电机会主动发送反馈帧，默认上报间隔为 `10ms`。该工具会发送通信类型 24 的配置帧，但不会使能电机，也不会写入运动目标。

```bash
./bin/rs01_active_report [CAN接口] [电机ID] [on|off] [主机ID]
```

常用示例：

```bash
./bin/rs01_active_report can0 1 on
./bin/rs01_active_report can0 1 off
./bin/rs01_active_report can0 1 on 0xfd
```

配合 `rs01_monitor` 使用：

```bash
./bin/rs01_active_report can0 1 on
./bin/rs01_monitor can0 1
```

当前实测设备在关闭主动上报时会执行关闭，但不一定返回配置应答；工具会打印 warning，不把它作为硬失败。

## 控制测试工具

### `rs01_velocity_test`

交互式速度模式测试程序。它会切换到速度模式、设置保守限流和加速度、使能电机，然后等待键盘控制速度目标。

默认安全参数：

- 电流限制：`current_limit=1.0A`
- 加速度：`acc=2.0rad/s^2`
- 初始速度目标：`0.0rad/s`

```bash
./bin/rs01_velocity_test [CAN接口] [电机ID] [目标速度rad/s] [主机ID]
```

常用示例：

```bash
./bin/rs01_velocity_test can0 1 0.5
./bin/rs01_velocity_test can0 1 0.5 0xfd
```

按键：

- `r`：按命令行指定的目标速度运行
- `s`：将速度目标写回 `0.0`
- `q`：停止并退出
- `Ctrl+C`：请求退出，程序会尽量先写 `0.0` 再失能

### `rs01_current_test`

交互式电流模式测试程序。它会切换到电流模式、使能电机，然后等待键盘写入 `iq_ref`。电流模式会直接产生力矩，首次测试应使用很小电流。

默认安全参数：

- 目标电流：`0.1A`
- 初始电流目标：`0.0A`

```bash
./bin/rs01_current_test [CAN接口] [电机ID] [目标电流A] [主机ID]
```

常用示例：

```bash
./bin/rs01_current_test can0 1 0.1
./bin/rs01_current_test can0 1 0.1 0xfd
```

按键：

- `r`：写入命令行指定的 `iq_ref`
- `s`：将 `iq_ref` 写回 `0.0`
- `q`：清零电流、失能并退出
- `Ctrl+C`：请求退出，程序会尽量先清零电流再失能

### `rs01_motion_test`

交互式运控模式测试程序。它会切换到运控模式、使能电机，并以固定周期连续发送运控帧。

运控帧包含 5 个控制量：

- `torque`：前馈力矩，单位 Nm
- `position`：目标位置，单位 rad
- `velocity`：目标速度，单位 rad/s
- `Kp`：位置刚度
- `Kd`：速度阻尼

默认目标是保守阻尼测试：`position=0.0`、`velocity=0.0`、`torque=0.0`、`Kp=0.0`、`Kd=1.0`。

```bash
./bin/rs01_motion_test [CAN接口] [电机ID] [位置rad] [速度rad/s] [力矩Nm] [Kp] [Kd] [主机ID]
```

常用示例：

```bash
./bin/rs01_motion_test can0 1
./bin/rs01_motion_test can0 1 0.0 0.0 0.0 1.0 0.5
./bin/rs01_motion_test can0 1 0.0 0.0 0.0 1.0 0.5 0xfd
```

按键：

- `r`：开始连续发送命令行指定的运控目标
- `s`：切回连续发送安全零帧
- `q`：发送安全零帧、失能并退出
- `Ctrl+C`：请求退出，程序会尽量先发送安全零帧再失能

### `rs01_position_pp_test`

交互式 PP 位置模式测试程序。它会切换到 PP 位置模式、设置最大速度和加速度、使能电机，然后等待键盘发送目标位置。

默认行为：

- 启动时先读取当前机械位置 `mech_pos`
- 如果没有在命令行指定目标位置，目标位置就使用当前机械位置
- 程序进入 PP 模式后先保持当前位置，不会立刻移动到新目标

```bash
./bin/rs01_position_pp_test [CAN接口] [电机ID] [目标位置rad] [最大速度rad/s] [加速度rad/s^2] [主机ID]
```

常用示例：

```bash
./bin/rs01_position_pp_test can0 1
./bin/rs01_position_pp_test can0 1 0.2 0.5 1.0
./bin/rs01_position_pp_test can0 1 0.2 0.5 1.0 0xfd
```

按键：

- `r`：发送命令行指定的目标位置
- `h`：读取当前位置，并把当前位置写为新的保持目标
- `q`：失能并退出
- `Ctrl+C`：请求退出，程序会尽量先失能

### `rs01_position_csp_test`

交互式 CSP 位置模式测试程序。CSP 模式由上位机周期性发送目标位置，电机持续跟随每一帧 `loc_ref`。

默认行为：

- 启动时读取当前机械位置 `mech_pos`
- 以当前位置作为初始目标，避免启动瞬间跳变
- 固定周期持续写入 `loc_ref`
- 按键只允许小步调整目标位置

```bash
./bin/rs01_position_csp_test [CAN接口] [电机ID] [单步rad] [速度限制rad/s] [周期ms] [主机ID]
```

常用示例：

```bash
./bin/rs01_position_csp_test can0 1
./bin/rs01_position_csp_test can0 1 0.02 0.5 20
./bin/rs01_position_csp_test can0 1 0.02 0.5 20 0xfd
```

按键：

- `[`：目标位置减少一个单步
- `]`：目标位置增加一个单步
- `h`：读取当前位置，并把当前位置作为新的保持目标
- `q`：失能并退出
- `Ctrl+C`：请求退出，程序会尽量先失能

## 安全工具

### `rs01_stop`

调试过程中用于尽量把常见控制目标清零并失能电机。它会尝试写入：

- `speed_ref = 0.0`
- `iq_ref = 0.0`
- `disable`

```bash
./bin/rs01_stop [CAN接口] [电机ID] [主机ID] [--clear]
```

常用示例：

```bash
./bin/rs01_stop can0 1
./bin/rs01_stop can0 1 0xff --clear
```

停止 CAN 接口：

```bash
sudo ip link set can0 down
```

## 当前状态

当前项目已经覆盖 RS01 私有 CAN 协议下的主要驱动能力，可以作为 RS01 的底层驱动包和现场调试工具集使用。

已具备的核心能力：

- SocketCAN 打开、发送、超时接收
- RS01 私有扩展帧 ID 打包
- 使能、失能、设置零点
- 参数 `uint8` / `float` / `uint32` 读写
- 反馈帧位置、速度、力矩、温度解析
- 完整故障/预警位解析
- 运控模式帧打包
- 速度模式基础控制
- 电流模式基础控制
- PP 位置模式基础控制
- CSP 位置模式基础控制
- 主动上报类型 24 开关
- 只读、监听、配置、控制和停机工具

后续可选增强：

- 保存参数类型 22
- CAN ID 修改和安全确认流程
- 版本读取、设备 ID 读取
- ROS2 node 封装

以上增强项不是使用当前驱动包的必要条件。常规调试和控制已经可以通过现有库接口与 `bin/` 下的工具完成。
