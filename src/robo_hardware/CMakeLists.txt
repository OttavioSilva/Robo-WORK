cmake_minimum_required(VERSION 3.16)
project(robo_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(hardware_bridge_node src/hardware_bridge_node.cpp)
target_compile_features(hardware_bridge_node PUBLIC cxx_std_17)
target_include_directories(hardware_bridge_node PRIVATE include)
ament_target_dependencies(hardware_bridge_node
  diagnostic_msgs
  geometry_msgs
  rcl_interfaces
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS hardware_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_sensor_clock test/test_sensor_clock.cpp)
  target_include_directories(test_sensor_clock PRIVATE include)
  target_compile_features(test_sensor_clock PRIVATE cxx_std_17)
  add_test(NAME sensor_clock COMMAND test_sensor_clock)
endif()

ament_package()
