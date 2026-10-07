################################################################################
# Our MTRX3760 Project 1 code: the wall-following robot and the maze world.
# Third-party packages stay in A1 Maze Simulation/tb3_third_parties;
# this package depends on turtlebot3_gazebo and launches its robot and bridges.
################################################################################
cmake_minimum_required(VERSION 3.8)
project(tb3_maze)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

# The executable keeps the name of the stock node it replaces.
add_executable(turtlebot3_drive
  src/turtlebot3_drive.cpp
  src/c_robot.cpp
  src/c_right_wall_follower_robot.cpp
  src/c_sensor.cpp
)
target_link_libraries(turtlebot3_drive
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  rclcpp::rclcpp
)

install(TARGETS turtlebot3_drive
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch worlds rviz
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(right_wall_follower_test
    test/right_wall_follower_test.cpp
    src/c_robot.cpp
    src/c_right_wall_follower_robot.cpp
    src/c_sensor.cpp
  )
  target_link_libraries(right_wall_follower_test ${sensor_msgs_TARGETS})
endif()

ament_package()
