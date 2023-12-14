cmake_minimum_required(VERSION 3.8)
project(navfuse)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(swri_transform_util REQUIRED)
find_package(swri_math_util REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/home/zyme/geographiclib/build/cmake/geographiclib.pc")
add_definitions (${GeographicLib_DEFINITIONS})
find_package (GeographicLib REQUIRED)



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


add_executable(gps_frame src/gps_frame.cpp)
ament_target_dependencies(gps_frame rclcpp sensor_msgs GeographicLib visualization_msgs tf2 tf2_ros)


add_executable(cube_test src/cube_test.cpp)
ament_target_dependencies(cube_test visualization_msgs rclcpp)

add_executable(line_strip_tuto src/line_trip_tuto.cpp)
ament_target_dependencies(line_strip_tuto visualization_msgs rclcpp)


# add_executable(gps_transform_publisher src/gps_transform_publisher.cpp)
# ament_target_dependencies(gps_transform_publisher rclcpp tf2 tf2_ros sensor_msgs swri_transform_util geometry_msgs swri_math_util)



install(TARGETS
  gps_frame
  cube_test
  line_strip_tuto
  #gps_transform_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  ${GeographicLib_INCLUDE_DIRS}
  DESTINATION share/${PROJECT_NAME}/
)


# install(FILES launch/test.launch.xml
#   DESTINATION share/${PROJECT_NAME}/launch)

install(PROGRAMS demo_data/fake_gps
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/fake_gps.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
