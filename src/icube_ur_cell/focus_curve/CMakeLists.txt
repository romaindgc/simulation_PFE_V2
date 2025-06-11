cmake_minimum_required(VERSION 3.8)
project(focus_curve)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)


#header include
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# === Executable focus_curve ===
add_executable(focus_curve src/focus_curve.cpp)
target_include_directories(focus_curve PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(focus_curve PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(focus_curve
  moveit_ros_planning_interface
  moveit_visual_tools
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
  trajectory_msgs
  ament_index_cpp
  rclcpp sensor_msgs
  cv_bridge
  OpenCV
)


install(TARGETS
  focus_curve
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch images csv
  DESTINATION share/${PROJECT_NAME}
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

ament_package()
