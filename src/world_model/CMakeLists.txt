cmake_minimum_required(VERSION 3.8)
project(world_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(var_dependencies
    ament_cmake
    rclcpp
    std_msgs
    nav_msgs
    ros2_utils
    geometry_msgs
    tf2
    tf2_ros 
    tf2_geometry_msgs
    tf2_sensor_msgs
    pcl_conversions
    pcl_ros
    sensor_msgs
    tf2_eigen
    OpenCV
    laser_geometry
    yaml-cpp
    PCL
    tf2_ros
    OpenMP)

foreach(var_dependency ${var_dependencies})
  find_package(${var_dependency} REQUIRED)
endforeach()

include_directories(include)

add_executable(pose_estimator src/pose_estimator.cpp)
ament_target_dependencies(pose_estimator ${var_dependencies})

add_executable(all_obstacle_filter src/all_obstacle_filter.cpp)
ament_target_dependencies(all_obstacle_filter ${var_dependencies})

if(OpenMP_CXX_FOUND)
  target_link_libraries(all_obstacle_filter OpenMP::OpenMP_CXX)
endif()

install(
  TARGETS 
  pose_estimator 
  all_obstacle_filter
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/waypoint_router.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/gps_logger.py DESTINATION lib/${PROJECT_NAME})

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
