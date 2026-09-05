cmake_minimum_required(VERSION 3.8)
project(fractal_tactile_exploration)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(paxini_hardware REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FractalMapSample.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs)

add_library(fractal_gp_map SHARED src/gp_map.cpp)
target_compile_features(fractal_gp_map PUBLIC cxx_std_17)
target_include_directories(fractal_gp_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch docs DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/omega_explorer_node.py DESTINATION lib/${PROJECT_NAME} RENAME omega_explorer_node)
install(FILES
  scripts/action_model.py
  scripts/belief_model.py
  scripts/exploration_policy.py
  scripts/gp_models.py
  scripts/mock_square_test.py
  scripts/moveit_executor.py
  scripts/omega_math.py
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS fractal_gp_map
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gp_map test/test_gp_map.cpp)
  target_link_libraries(test_gp_map fractal_gp_map)
endif()

ament_export_dependencies(geometry_msgs rosidl_default_runtime)
ament_export_libraries(fractal_gp_map)
ament_package()