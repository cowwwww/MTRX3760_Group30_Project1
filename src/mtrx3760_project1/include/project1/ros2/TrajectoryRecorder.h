//----------------------------------------------------------------------------
// File:    TrajectoryRecorder.h
// Project: MTRX3760 Project 1 - Hello, Turtlebots! (Group 30)
// Purpose: Keeps a bounded nav_msgs::msg::Path of odometry poses so RViz can
//          draw the robot's trajectory.
//----------------------------------------------------------------------------
#ifndef PROJECT1_ROS2_TRAJECTORY_RECORDER_H
#define PROJECT1_ROS2_TRAJECTORY_RECORDER_H

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

namespace project1
{
namespace ros2
{
// Maintains bounded path history; ROS publication belongs to the node.
class TrajectoryRecorder
{
public:
    // Returns true when an accepted odometry sample changes the path.
    bool Record(const nav_msgs::msg::Odometry& odom);
    const nav_msgs::msg::Path& GetPath() const;

private:
    nav_msgs::msg::Path path_;
};
}
}
#endif
