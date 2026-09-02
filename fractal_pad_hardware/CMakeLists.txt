cmake_minimum_required(VERSION 3.8)
project(fractal_pad_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(
  fractal_pad_hardware
  SHARED
  src/fractal_pad_hardware.cpp
)
target_include_directories(
  fractal_pad_hardware
  PUBLIC
  include
)
ament_target_dependencies(
  fractal_pad_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
# prevent pluginlib from using boost
target_compile_definitions(fractal_pad_hardware PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface fractal_pad_hardware.xml)

install(
  TARGETS
  fractal_pad_hardware
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)

endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  fractal_pad_hardware
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
