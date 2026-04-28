cmake_minimum_required(VERSION 3.16)
project(rs01_motor_control VERSION 0.1.0 LANGUAGES CXX)

# 协议层接口使用了 std::optional 等 C++17 特性，因此这里固定 C++17。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 核心静态库：包含 Linux SocketCAN 封装，以及 RS01 私有 CAN 协议的打包/解析。
add_library(rs01_motor
  src/can_socket.cpp
  src/rs01_motor.cpp
)

# 对外暴露 include/ 目录，调用方可以使用 <rs01_motor/...> 的形式包含头文件。
target_include_directories(rs01_motor
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 库代码开启较严格的编译警告，方便尽早发现底层通信代码里的潜在问题。
target_compile_options(rs01_motor
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

# 面向真实硬件调试的小示例：一个用于读模式，一个用于低速转动测试。
add_executable(rs01_read_mode examples/rs01_read_mode.cpp)
target_link_libraries(rs01_read_mode PRIVATE rs01_motor)

add_executable(rs01_velocity_test examples/rs01_velocity_test.cpp)
target_link_libraries(rs01_velocity_test PRIVATE rs01_motor)

# 简单安装规则，便于后续把库、示例程序和头文件安装到指定 prefix。
install(TARGETS rs01_motor rs01_read_mode rs01_velocity_test
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
