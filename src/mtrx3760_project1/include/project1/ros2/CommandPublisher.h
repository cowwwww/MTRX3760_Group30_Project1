//----------------------------------------------------------------------------
// File:    CommandPublisher.h
// Project: MTRX3760 Project 1 - Hello, Turtlebots! (Group 30)
// Purpose: Publishes velocity commands as Twist or TwistStamped, selected by
//          the stamped_cmd_vel parameter.
//----------------------------------------------------------------------------
#ifndef PROJECT1_ROS2_COMMAND_PUBLISHER_H
#define PROJECT1_ROS2_COMMAND_PUBLISHER_H

#include "project1/ControlTypes.h"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace project1
{
namespace ros2
{
// Owns the selected command message type, frame and ROS publisher.
class CommandPublisher
{
public:
    explicit CommandPublisher(rclcpp::Node& node);
    void Publish(const Velocity& velocity);

private:
    rclcpp::Node& node_;
    bool stamped_;
    std::string frame_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr plainPublisher_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr stampedPublisher_;
};
}
}
#endif
