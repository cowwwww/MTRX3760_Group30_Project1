//----------------------------------------------------------------------------
// File:    WallFollowerNode.h
// Project: MTRX3760 Project 1 - Hello, Turtlebots! (Group 30)
// Purpose: ROS 2 node: subscribes to scan and odometry, runs the WallFollower
//          each tick, and publishes velocity commands, the trajectory and
//          diagnostics.
//----------------------------------------------------------------------------
#ifndef PROJECT1_ROS2_WALL_FOLLOWER_NODE_H
#define PROJECT1_ROS2_WALL_FOLLOWER_NODE_H

#include "project1/WallFollower.h"
#include "project1/ros2/CommandPublisher.h"
#include "project1/ros2/TrajectoryRecorder.h"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace project1
{
namespace ros2
{
// Connects ROS subscriptions to the portable controller and path recorder.
class WallFollowerNode : public rclcpp::Node
{
public:
    WallFollowerNode();
    void Tick();
    void Stop();

private:
    Settings ReadSettings();
    void ReceiveScan(const sensor_msgs::msg::LaserScan& scan);
    void ReceiveOdometry(const nav_msgs::msg::Odometry& odom);

    WallFollower controller_;
    CommandPublisher commands_;
    TrajectoryRecorder trajectory_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnosticsPublisher_;
    std::string odomFrame_, odomChild_, lastReason_;
    WallFollower::State lastState_ = WallFollower::State::WaitingForScan;
    double diagnosticsTime_ = -1.0, scanSourceAge_ = -1.0, odomSourceAge_ = -1.0;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSubscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscriber_;
};
}
}
#endif
