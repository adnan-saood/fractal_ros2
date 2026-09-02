cmake_minimum_required(VERSION 3.8)
project(fractal_pad_bringup)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(joint_state_slider_command_bridge src/joint_state_slider_command_bridge.cpp)
ament_target_dependencies(joint_state_slider_command_bridge rclcpp sensor_msgs std_msgs)

install(
  TARGETS joint_state_slider_command_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
endif()

ament_package()
