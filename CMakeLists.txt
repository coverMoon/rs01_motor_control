cmake_minimum_required(VERSION 3.16)
project(rs01_motor_control VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(rs01_motor
  src/can_socket.cpp
  src/rs01_motor.cpp
)

target_include_directories(rs01_motor
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(rs01_motor
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

add_executable(rs01_read_mode examples/rs01_read_mode.cpp)
target_link_libraries(rs01_read_mode PRIVATE rs01_motor)

add_executable(rs01_velocity_test examples/rs01_velocity_test.cpp)
target_link_libraries(rs01_velocity_test PRIVATE rs01_motor)

install(TARGETS rs01_motor rs01_read_mode rs01_velocity_test
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
