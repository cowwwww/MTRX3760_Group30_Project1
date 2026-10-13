//----------------------------------------------------------------------------
// File:    CommandPublisher.cpp
// Project: MTRX3760 Project 1 - Hello, Turtlebots! (Group 30)
// Purpose: Creates the selected command publisher and converts a Velocity into
//          the ROS message.
//----------------------------------------------------------------------------
#include "project1/ros2/CommandPublisher.h"

namespace project1
{
namespace ros2
{
CommandPublisher::CommandPublisher(rclcpp::Node& node) : node_(node)
{
    stamped_ = node_.declare_parameter<bool>("stamped_cmd_vel", true);
    frame_ = node_.declare_parameter<std::string>("command_frame", "base_link");
    if (stamped_)
        stampedPublisher_ = node_.create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
    else
        plainPublisher_ = node_.create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

void CommandPublisher::Publish(const Velocity& velocity)
{
    geometry_msgs::msg::Twist command;
    command.linear.x = velocity.linear;
    command.angular.z = velocity.angular;
    if (stamped_)
    {
        geometry_msgs::msg::TwistStamped stamped;
        stamped.header.stamp = node_.now();
        stamped.header.frame_id = frame_;
        stamped.twist = command;
        stampedPublisher_->publish(stamped);
    }
    else plainPublisher_->publish(command);
}
}
}
