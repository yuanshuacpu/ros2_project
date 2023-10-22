cmake_minimum_required(VERSION 3.8)
project(gimbal_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
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
  geometry_msgs
  pid
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add gimbal_controller library related compile commands
generate_parameter_library(gimbal_controller_parameters
  src/gimbal_controller.yaml
  include/gimbal_controller/validate_gimbal_controller_parameters.hpp
)

file(GLOB ALL_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(
  gimbal_controller
  SHARED
  ${ALL_SRC}
)

target_compile_features(gimbal_controller PUBLIC cxx_std_17)

target_include_directories(gimbal_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

# link_directories("/home/yuan/rm/code/Ros/my_ros2_control/install/tool_pid/lib")
# link_directories("${CMAKE_CURRENT_SOURCE_DIR}/lib")
target_link_libraries(gimbal_controller
  PUBLIC gimbal_controller_parameters
)

ament_target_dependencies(gimbal_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_compile_definitions(gimbal_controller PRIVATE "GIMBAL_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface gimbal_controller.xml)

install(
  TARGETS
  gimbal_controller gimbal_controller_parameters
  EXPORT export_gimbal_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_gimbal_controller HAS_LIBRARY_TARGET)

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
  gimbal_controller
)

ament_package()
