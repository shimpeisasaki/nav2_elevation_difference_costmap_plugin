cmake_minimum_required(VERSION 3.8)
project(nav2_elevation_difference_costmap_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
add_library(elevation_terrain_model SHARED src/terrain_model.cpp)
target_compile_features(elevation_terrain_model PUBLIC cxx_std_17)
target_include_directories(elevation_terrain_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(
  nav2_elevation_difference_costmap_plugin_core
  SHARED
  src/elevation_layer.cpp
)
target_compile_features(nav2_elevation_difference_costmap_plugin_core PUBLIC cxx_std_17)
target_link_libraries(nav2_elevation_difference_costmap_plugin_core elevation_terrain_model)

target_include_directories(
  nav2_elevation_difference_costmap_plugin_core
  PUBLIC
  include
)

ament_target_dependencies(
  nav2_elevation_difference_costmap_plugin_core
  nav2_costmap_2d
  pluginlib
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  std_msgs
  tf2_geometry_msgs
  rclcpp
)

pluginlib_export_plugin_description_file(
  nav2_costmap_2d
  elevation_layer.xml
)

install(
  TARGETS nav2_elevation_difference_costmap_plugin_core elevation_terrain_model
  DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_terrain_model test/test_terrain_model.cpp)
  target_link_libraries(test_terrain_model elevation_terrain_model)
  ament_target_dependencies(test_terrain_model sensor_msgs)
  ament_add_gtest(test_elevation_layer test/test_elevation_layer.cpp)
  target_link_libraries(test_elevation_layer nav2_elevation_difference_costmap_plugin_core)
  ament_target_dependencies(test_elevation_layer nav2_costmap_2d rclcpp sensor_msgs tf2_ros)
  set_tests_properties(test_elevation_layer PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=221;ROS_LOCALHOST_ONLY=1")
endif()
ament_export_include_directories(include)
ament_export_libraries(elevation_terrain_model)
ament_export_dependencies(nav2_costmap_2d rclcpp sensor_msgs tf2 tf2_ros)

install(
  FILES elevation_layer.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
