cmake_minimum_required(VERSION 3.8)
project(speed_meter_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(libqt5-widgets REQUIRED)
find_package(std_msgs REQUIRED)

add_library(speed_meter_plugin src/speed_meter_plugin.cpp)
add_library(speed_meter_plugin::speed_meter_plugin ALIAS speed_meter_plugin)
target_compile_features(speed_meter_plugin PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(speed_meter_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  speed_meter_plugin
  "rclcpp"
  "pluginlib"
  "rviz_common"
  "libqt5-widgets"
  "std_msgs"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(speed_meter_plugin PRIVATE "SPEED_METER_PLUGIN_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS speed_meter_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  speed_meter_plugin
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
