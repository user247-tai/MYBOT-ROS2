cmake_minimum_required(VERSION 3.16)
project(mybot_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  mybot_control
  SHARED
  hardware/diffbot_system.cpp
)

target_compile_features(mybot_control PUBLIC cxx_std_17)
target_include_directories(mybot_control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/mybot_control>
)

ament_target_dependencies(
  mybot_control PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(mybot_control PUBLIC serial)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "MYBOT_CONTROL_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface mybot_control.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/mybot_control
)
install(
  DIRECTORY description/ros2_control description/urdf description/rviz description/launch 
  DESTINATION share/mybot_control
)
install(
  DIRECTORY bringup/launch bringup/config config
  DESTINATION share/mybot_control
)
install(TARGETS mybot_control
  EXPORT export_mybot_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_mybot_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
