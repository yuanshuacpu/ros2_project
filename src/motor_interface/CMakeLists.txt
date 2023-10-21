cmake_minimum_required(VERSION 3.8)
project(motor_interface LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

file(GLOB ALL_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(
  motor_interface
  SHARED
  ${ALL_SRC}
)

target_compile_features(motor_interface PUBLIC cxx_std_20)

target_include_directories(
  motor_interface
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/motor_interface
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(
  motor_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib
)

# prevent pluginlib from using boost
target_compile_definitions(motor_interface PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface motor_interface.xml)

# INSTALL
install(
  DIRECTORY include
  DESTINATION include/motor_interface
)

install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/motor_interface
)

install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/motor_interface
)

install(
  TARGETS
  motor_interface
  EXPORT export_motor_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_motor test/test_motor.cpp)
  target_include_directories(test_motor PRIVATE include)
  ament_target_dependencies(
    test_motor
    hardware_interface
    pluginlib
    ros2_control_test_assets
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  motor_interface
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
