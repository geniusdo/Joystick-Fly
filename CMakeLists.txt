cmake_minimum_required(VERSION 3.12)

project(JoystickFly)

set(CMAKE_CXX_STANDARD 17)

set(OIS_BUILD_DEMOS OFF)
add_subdirectory(third_party/OIS)

include_directories(third_party/OIS/includes)
include_directories(include)

add_executable(JoystickFly src/main.cc)
add_executable(receive src/receive.cc)

link_directories(${CMAKE_BINARY_DIR}/third_party/OIS)

target_link_libraries(JoystickFly OIS)