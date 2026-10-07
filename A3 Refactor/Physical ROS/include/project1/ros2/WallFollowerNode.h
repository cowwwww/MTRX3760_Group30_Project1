#ifndef PROJECT1_ROS2_WALL_FOLLOWER_NODE_H
#define PROJECT1_ROS2_WALL_FOLLOWER_NODE_H

#include "project1/WallFollower.h"
#include "project1/ros2/CommandPublisher.h"
#include "project1/ros2/TrajectoryRecorder.h"
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
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSubscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscriber_;
};
}
}
#endif
