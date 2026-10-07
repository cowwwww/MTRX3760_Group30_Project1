cmake_minimum_required(VERSION 3.10)
project(mtrx3760_project1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wshadow -Wnon-virtual-dtor -pedantic)

# Plain CMake works without ROS or raylib; sourced ROS workspaces select their adapter.
set(PROJECT1_ROS_VERSION "$ENV{ROS_VERSION}" CACHE STRING "ROS adapter: 1, 2, or empty")
add_library(wall_controller src/WallFollower.cpp)
target_include_directories(wall_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

include(CTest)
if(BUILD_TESTING)
    add_executable(wall_controller_tests test/wall_controller_tests.cpp)
    target_link_libraries(wall_controller_tests PRIVATE wall_controller)
    set(controller_checks
        default_settings invalid_settings invalid_ordering
        no_scan straight_wall close_wall distant_wall wall_heading
        lost_wall blocked_front_priority corner_hysteresis
        stale_scan time_reversal invalid_scan_time
        corrupt_sector isolated_dropout front_minimum side_median
        no_returns malformed_metadata out_of_range missing_coverage
        zero_to_two_pi descending_scan rotated_lidar velocity_bounds)
    foreach(check IN LISTS controller_checks)
        add_test(NAME controller_${check} COMMAND wall_controller_tests ${check})
    endforeach()
endif()

if(PROJECT1_ROS_VERSION STREQUAL "2")
    set(CMAKE_CXX_STANDARD 17)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    add_executable(turtlebot3_drive src/turtlebot3_drive_ros2.cpp)
    target_link_libraries(turtlebot3_drive wall_controller)
    ament_target_dependencies(turtlebot3_drive rclcpp sensor_msgs geometry_msgs nav_msgs)
    install(TARGETS turtlebot3_drive DESTINATION lib/${PROJECT_NAME})
    install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
    if(BUILD_TESTING)
        find_package(Python3 REQUIRED COMPONENTS Interpreter)
        add_test(NAME ros2_adapter_integration
            COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/test/ros2_integration_test.py
                --executable $<TARGET_FILE:turtlebot3_drive>
                --artifact-dir ${CMAKE_CURRENT_BINARY_DIR}/ros2_test_artifacts)
        set_tests_properties(ros2_adapter_integration PROPERTIES TIMEOUT 90)
    endif()
    ament_package()
elseif(PROJECT1_ROS_VERSION STREQUAL "1")
    find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs nav_msgs)
    catkin_package()
    add_executable(turtlebot3_drive src/turtlebot3_drive_ros1.cpp)
    target_include_directories(turtlebot3_drive PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(turtlebot3_drive PRIVATE wall_controller ${catkin_LIBRARIES})
    install(TARGETS turtlebot3_drive RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
elseif(NOT PROJECT1_ROS_VERSION STREQUAL "")
    message(FATAL_ERROR "PROJECT1_ROS_VERSION must be empty, 1, or 2")
endif()
