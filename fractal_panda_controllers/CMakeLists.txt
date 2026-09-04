cmake_minimum_required(VERSION 3.8)
project(fractal_panda_controllers)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(Franka REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(fractal_joint_reference_impedance_controller SHARED
  src/joint_reference_impedance_controller.cpp)
target_compile_features(fractal_joint_reference_impedance_controller PUBLIC cxx_std_17)
target_include_directories(fractal_joint_reference_impedance_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(fractal_joint_reference_impedance_controller
  controller_interface Franka hardware_interface pluginlib rclcpp realtime_tools trajectory_msgs)
pluginlib_export_plugin_description_file(controller_interface fractal_panda_controllers.xml)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS fractal_joint_reference_impedance_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_dependencies(controller_interface Franka hardware_interface pluginlib rclcpp realtime_tools trajectory_msgs)
ament_export_libraries(fractal_joint_reference_impedance_controller)
ament_package()