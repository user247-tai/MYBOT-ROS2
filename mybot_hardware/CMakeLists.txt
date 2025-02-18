cmake_minimum_required(VERSION 3.8)
project(mybot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(PkgConfig)

add_library(${PROJECT_NAME} SHARED 
  src/diffbot_system.cpp
)

target_compile_features(mybot_hardware PUBLIC cxx_std_17)

# target_include_directories(${PROJECT_NAME} PRIVATE 
#   include
#   "${SERIAL_INCLUDE_DIRS}"
# )

target_include_directories(mybot_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/mybot_hardware>
)

# target_link_libraries(${PROJECT_NAME}
#  "${SERIAL_LDFLAGS}")

ament_target_dependencies(
  ${PROJECT_NAME} PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_link_libraries(mybot_hardware PUBLIC serial)


target_compile_definitions(${PROJECT_NAME} PRIVATE "MYBOT_HARDWARE_BUILDING_DLL")


pluginlib_export_plugin_description_file(hardware_interface mybot_hardware.xml)


install(TARGETS mybot_hardware
  EXPORT export_mybot_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_mybot_hardware HAS_LIBRARY_TARGET)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_export_include_directories(
  include
)

ament_package()
