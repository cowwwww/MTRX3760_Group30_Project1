cmake_minimum_required(VERSION 3.10)

project(MTRX3760_Lab2_A2)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wshadow -Wnon-virtual-dtor -pedantic")

find_package(raylib CONFIG REQUIRED)

add_executable(robot_simulator
    main.cpp
    CSimulation.cpp
    CWallFollower.cpp
    CLineFollower.cpp
    CRobot.cpp
    CRangeSensor.cpp
    CLineSensor.cpp
    CWallMap.cpp
    CLoopReader.cpp
    CRender.cpp
)

target_link_libraries(robot_simulator PRIVATE raylib)

configure_file(SimpleWalls.map SimpleWalls.map COPYONLY)
configure_file(SimpleLine.map SimpleLine.map COPYONLY)
