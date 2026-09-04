cmake_minimum_required(VERSION 3.8)
project(fractal_pad_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add fractal_pad_controller library related compile commands
generate_parameter_library(fractal_pad_controller_parameters
  src/fractal_pad_controller.yaml
  include/fractal_pad_controller/validate_fractal_pad_controller_parameters.hpp
)
add_library(
  fractal_pad_controller
  SHARED
  src/fractal_pad_controller.cpp
)
target_include_directories(fractal_pad_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(fractal_pad_controller fractal_pad_controller_parameters)
ament_target_dependencies(fractal_pad_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(fractal_pad_controller PRIVATE "FRACTAL_PAD_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface fractal_pad_controller.xml)

install(
  TARGETS
  fractal_pad_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)

endif()

ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  fractal_pad_controller
)

# four-bar joint state publisher node
add_executable(four_bar_joint_state_publisher src/four_bar_joint_state_publisher.cpp)
target_include_directories(four_bar_joint_state_publisher PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(four_bar_joint_state_publisher
  rclcpp
  sensor_msgs)
install(TARGETS four_bar_joint_state_publisher
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
