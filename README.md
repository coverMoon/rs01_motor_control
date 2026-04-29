# RS01 电机控制库

RS01 专用 Linux SocketCAN 底层驱动库，并附带一组现场调试工具。

本仓库的核心是 C++ 驱动库。`bin/` 下的可执行程序主要用于验证通信、调试电机和提供 API 使用参考；实际项目集成时，推荐直接使用 `rs01::Rs01Motor`。

当前项目只面向 RobStride RS01，使用手册第 4 章私有 CAN 扩展帧协议，不做 RS02、CanOpen、MIT 标准帧抽象。

RS01 物理量范围：

- 位置：`+-4*pi rad`
- 速度：`+-44 rad/s`
- 力矩：`+-17 Nm`
- 刚度系数 Kp：`0~500`
- 阻尼系数 Kd：`0~5`
- 电流：`+-23 A`

## 快速开始

编译：

```bash
cmake -S . -B build
cmake --build build -j
```

配置 CAN：

```bash
sudo ./scripts/setup_can.sh
```

用只读工具确认通信：

```bash
./bin/rs01_read_mode can0 1
./bin/rs01_dump_status can0 1
```

在自己的程序里使用驱动库：

```cpp
#include "rs01_motor/protocol.h"
#include "rs01_motor/rs01_motor.h"

int main() {
  rs01::Rs01Motor motor("can0", 1);

  auto mode = motor.read_param_u8(rs01::param::kRunMode);
  if (!mode) {
    return 1;
  }

  motor.velocity_control(0.5f, 1.0f, 2.0f);
  motor.write_param_float(rs01::param::kSpeedRef, 0.0f);
  motor.disable(false);
  return 0;
}
```

## 库结构

对外主要接口在 `include/rs01_motor/` 下：

| 文件 | 作用 | 使用建议 |
|---|---|---|
| `rs01_motor/rs01_motor.h` | 单个 RS01 电机的高级封装 | 业务代码优先使用 |
| `rs01_motor/protocol.h` | 通信类型、参数索引、运行模式和物理范围常量 | 配合 `Rs01Motor` 使用 |
| `rs01_motor/can_socket.h` | Linux SocketCAN RAW socket 的 RAII 封装 | 一般不需要直接使用 |

`Rs01Motor` 对象绑定一个 CAN 接口和一个电机 ID：

```cpp
rs01::Rs01Motor motor("can0", 1);
```

如果需要指定主机 ID，可以传第三个参数。默认主机 ID 是 `0xff`：

```cpp
rs01::Rs01Motor motor("can0", 1, 0xfd);
```

## API 使用流程

常规控制程序建议按下面顺序组织：

1. 配置并打开 CAN 接口。
2. 创建 `rs01::Rs01Motor`。
3. 读取 `run_mode` 或 `mech_pos`，确认通信和初始状态。
4. 切换到目标控制模式。
5. 设置限制参数，例如限流、速度限制、加速度。
6. 使能电机。
7. 写入目标值，或周期性发送控制帧。
8. 退出前把目标清零或保持当前位置。
9. 调用 `disable(false)` 失能电机。

库中部分封装函数已经包含了切模式、设置限制和使能流程。例如 `velocity_control()`、`current_control()`、`position_pp_control()` 和 `position_csp_control()` 都会自动切换模式并使能；`motion_control()` 只负责发送运控帧，调用前需要手动 `set_mode()` 和 `enable()`。

## 参数读写

RS01 的运行模式、目标值、限制值和诊断量都通过参数索引访问。索引常量在 `rs01::param` 命名空间中。

读取运行模式：

```cpp
auto mode = motor.read_param_u8(rs01::param::kRunMode);
if (mode) {
  // *mode 是当前 run_mode
}
```

读取当前位置、速度、电流和母线电压：

```cpp
auto position = motor.read_param_float(rs01::param::kMechPos);
auto velocity = motor.read_param_float(rs01::param::kMechVel);
auto current = motor.read_param_float(rs01::param::kIqFiltered);
auto voltage = motor.read_param_float(rs01::param::kVbus);
```

写入 float 参数：

```cpp
motor.write_param_float(rs01::param::kLimitCurrent, 1.0f);
motor.write_param_float(rs01::param::kSpeedRef, 0.5f);
```

读取故障位：

```cpp
auto fault = motor.read_param_u32(rs01::param::kFaultStatus);
if (fault) {
  auto descriptions = rs01::Rs01Motor::describe_fault_bits(*fault);
}
```

参数读取接口返回 `std::optional<T>`。返回空值表示在超时时间内没有收到匹配响应，业务代码应该把它当成通信失败处理。

## 速度模式

速度模式适合让电机按目标速度连续转动。推荐初次测试使用小速度、小限流和小加速度。

`velocity_control()` 会完成下面流程：

1. 切换到速度模式。
2. 写入限流 `limit_current`。
3. 写入加速度 `velocity_acc`。
4. 使能电机。
5. 写入速度目标 `speed_ref`。

示例：

```cpp
rs01::Rs01Motor motor("can0", 1);

motor.velocity_control(0.5f, 1.0f, 2.0f);

// 运行中可以继续修改速度目标。
motor.write_param_float(rs01::param::kSpeedRef, 0.2f);
motor.write_param_float(rs01::param::kSpeedRef, 0.0f);

motor.disable(false);
```

退出前建议先把 `speed_ref` 写回 `0.0f`，等待短时间后再失能。

## 电流模式

电流模式通过 `iq_ref` 控制电流，电流会直接影响输出力矩。初次测试应使用很小电流，并确保电机固定可靠。

`current_control()` 会切换到电流模式、使能电机并写入目标电流：

```cpp
rs01::Rs01Motor motor("can0", 1);

motor.current_control(0.1f);

// 运行中可以修改目标电流。
motor.write_param_float(rs01::param::kIqRef, 0.0f);

motor.disable(false);
```

退出前建议先把 `iq_ref` 写回 `0.0f`。

## 运控模式

运控模式使用 type-1 控制帧，一帧同时包含：

- 前馈力矩 `torque`
- 目标位置 `position`
- 目标速度 `velocity`
- 位置刚度 `Kp`
- 速度阻尼 `Kd`

这个模式适合需要上位机周期性发送控制量的场景。`motion_control()` 只发送控制帧，不会自动切模式或使能，所以调用顺序必须明确：

```cpp
rs01::Rs01Motor motor("can0", 1);

motor.set_mode(rs01::mode::kMotion);
motor.enable();

while (running) {
  motor.motion_control(
      0.0f,  // torque, Nm
      0.0f,  // position, rad
      0.0f,  // velocity, rad/s
      1.0f,  // Kp
      0.5f   // Kd
  );

  std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

for (int i = 0; i < 20; ++i) {
  motor.motion_control(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

motor.disable(false);
```

运控模式的关键是周期性发送。停止时建议先连续发送若干帧零力矩、零刚度、零阻尼的安全帧，再失能。

## PP 位置模式

PP 位置模式适合“给一个目标位置，让电机按内部规划运动过去”的场景。上位机不需要持续刷新轨迹点。

`position_pp_control()` 会完成：

1. 切换到 PP 位置模式。
2. 设置最大速度 `pp_velocity_max`。
3. 设置加速度 `pp_acceleration`。
4. 使能电机。
5. 写入目标位置 `loc_ref`。

建议先读取当前位置，把当前位置作为初始目标，避免启动瞬间跳变：

```cpp
rs01::Rs01Motor motor("can0", 1);

auto current_position = motor.read_param_float(rs01::param::kMechPos);
if (!current_position) {
  return 1;
}

motor.position_pp_control(*current_position, 0.5f, 1.0f);

// 确认安全后，再写入真正目标。
motor.write_param_float(rs01::param::kLocRef, *current_position + 0.2f);

motor.disable(false);
```

## CSP 位置模式

CSP 位置模式适合上位机自己生成位置轨迹，并周期性下发 `loc_ref` 的场景。和 PP 不同，CSP 需要持续刷新目标位置。

`position_csp_control()` 会切换到 CSP 位置模式、设置速度限制、使能电机并写入初始位置：

```cpp
rs01::Rs01Motor motor("can0", 1);

auto current_position = motor.read_param_float(rs01::param::kMechPos);
if (!current_position) {
  return 1;
}

float target = *current_position;
motor.position_csp_control(target, 0.5f);

while (running) {
  target += 0.001f;
  motor.write_param_float(rs01::param::kLocRef, target);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

motor.disable(false);
```

CSP 的安全重点是：启动时从当前位置开始，运行中按固定周期刷新，目标变化不要突变。

## PP 和 CSP 的区别

| 模式 | 目标写入方式 | 轨迹来源 | 适合场景 |
|---|---|---|---|
| PP | 写一次目标位置 | 电机内部规划 | 简单点到点位置控制 |
| CSP | 周期性写目标位置 | 上位机生成轨迹 | 多轴同步、外部轨迹跟踪 |

如果只是让单个电机转到某个位置，优先使用 PP。如果上位机需要严格控制每个周期的位置目标，使用 CSP。

## 主动上报和反馈解析

开启主动上报后，电机会按周期主动发送反馈帧。驱动库可以配置开关，也可以解析反馈帧。

开启主动上报：

```cpp
bool acknowledged = motor.set_active_report(true);
```

关闭主动上报：

```cpp
bool acknowledged = motor.set_active_report(false);
```

当前实测设备在关闭主动上报时会执行关闭，但不一定返回配置应答。因此 `set_active_report(false)` 返回 `false` 不一定表示关闭失败，也可能只是没有收到应答。

主动读取一帧反馈：

```cpp
auto feedback = motor.read_feedback(100);
if (feedback) {
  float position = feedback->position;
  float velocity = feedback->velocity;
  float torque = feedback->torque;
  float temperature = feedback->temperature;
}
```

如果业务代码自己接收原始 `can_frame`，可以使用静态函数解析：

```cpp
rs01::Feedback feedback = rs01::Rs01Motor::parse_feedback(frame);
```

## 故障处理

反馈帧中带有 6 bit 简略故障状态，可以直接解析：

```cpp
auto feedback = motor.read_feedback(100);
if (feedback) {
  auto items = rs01::Rs01Motor::describe_feedback_fault(feedback->fault);
}
```

完整故障和预警位建议通过参数读取：

```cpp
auto fault = motor.read_param_u32(rs01::param::kFaultStatus);
auto warning = motor.read_param_u32(rs01::param::kWarningStatus);

if (fault) {
  auto items = rs01::Rs01Motor::describe_fault_bits(*fault);
}

if (warning) {
  auto items = rs01::Rs01Motor::describe_warning_bits(*warning);
}
```

调试时也可以直接运行：

```bash
./bin/rs01_dump_status can0 1
```

## 停机建议

不同模式退出前应先让目标值回到安全状态：

| 模式 | 建议停机动作 |
|---|---|
| 速度模式 | `speed_ref = 0.0f`，短暂等待，`disable(false)` |
| 电流模式 | `iq_ref = 0.0f`，短暂等待，`disable(false)` |
| 运控模式 | 连续发送若干帧零力矩、零刚度、零阻尼控制帧，`disable(false)` |
| PP 位置模式 | 必要时写当前位置保持，`disable(false)` |
| CSP 位置模式 | 停止轨迹刷新前写当前位置或保持目标，`disable(false)` |

也可以使用停机工具做现场兜底：

```bash
./bin/rs01_stop can0 1
```

## 安全说明

- RS01 需要单独供电，USB-CAN 不给电机供电。
- 初次测试先用小速度、小电流、小位移和低刚度。
- 运行控制类程序前，确认电机固定方式、负载、供电、机械限位和急停手段。
- `velocity_control()`、`current_control()`、`position_pp_control()`、`position_csp_control()` 都会使能电机。
- `set_mode()` 会先失能再写入运行模式。
- `motion_control()` 不会自动切模式、不自动使能，调用方必须先执行 `set_mode(rs01::mode::kMotion)` 和 `enable()`。
- 如果没有回包，先检查电源、CAN_H/CAN_L、GND、波特率、ID 和终端电阻。

## 编译和集成

常规编译：

```bash
cmake -S . -B build
cmake --build build -j
```

编译后的示例工具统一生成到项目根目录的 `bin/`：

```bash
ls bin
```

如果在本仓库里新增自己的程序，可以在 `CMakeLists.txt` 中链接 `rs01_motor`：

```cmake
add_executable(my_rs01_app src/my_rs01_app.cpp)
target_link_libraries(my_rs01_app PRIVATE rs01_motor)
```

如果把本库作为子目录集成到其他 CMake 工程：

```cmake
add_subdirectory(path/to/rs01_motor_control)

add_executable(my_rs01_app src/my_rs01_app.cpp)
target_link_libraries(my_rs01_app PRIVATE rs01_motor)
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

## 调试工具总览

这些工具既可以用于现场调试，也可以作为库 API 的参考实现。

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
