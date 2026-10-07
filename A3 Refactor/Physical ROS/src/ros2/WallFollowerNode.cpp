#include "project1/ros2/WallFollowerNode.h"
#include <chrono>

namespace project1
{
namespace ros2
{
namespace
{
double MonotonicSeconds()
{
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
}

WallFollowerNode::WallFollowerNode()
    : Node("project1_wall_follower"), controller_(ReadSettings()), commands_(*this)
{
    pathPublisher_ = create_publisher<nav_msgs::msg::Path>(
        "trajectory", rclcpp::QoS(1).transient_local());
    scanSubscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) { ReceiveScan(*scan); });
    odomSubscriber_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::SensorDataQoS().keep_last(1),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr odom) { ReceiveOdometry(*odom); });
    RCLCPP_INFO(get_logger(), "Right-wall follower ready; waiting for valid laser data");
}

Settings WallFollowerNode::ReadSettings()
{
    Settings settings;
    settings.wallDistance = declare_parameter<double>("wall_distance", settings.wallDistance);
    settings.forwardSpeed = declare_parameter<double>("forward_speed", settings.forwardSpeed);
    settings.turnSpeed = declare_parameter<double>("turn_speed", settings.turnSpeed);
    settings.searchSpeed = declare_parameter<double>("search_speed", settings.searchSpeed);
    settings.searchTurnSpeed = declare_parameter<double>("search_turn_speed", settings.searchTurnSpeed);
    settings.frontStop = declare_parameter<double>("front_stop", settings.frontStop);
    settings.frontRelease = declare_parameter<double>("front_release", settings.frontRelease);
    settings.lostWall = declare_parameter<double>("lost_wall", settings.lostWall);
    settings.distanceGain = declare_parameter<double>("distance_gain", settings.distanceGain);
    settings.headingGain = declare_parameter<double>("heading_gain", settings.headingGain);
    settings.scanTimeout = declare_parameter<double>("scan_timeout", settings.scanTimeout);
    settings.laserYaw = declare_parameter<double>("laser_yaw", settings.laserYaw);
    return settings;
}

void WallFollowerNode::ReceiveScan(const sensor_msgs::msg::LaserScan& scan)
{
    LaserScan input;
    input.angleMin = scan.angle_min;
    input.angleIncrement = scan.angle_increment;
    input.rangeMin = scan.range_min;
    input.rangeMax = scan.range_max;
    input.ranges = scan.ranges;
    controller_.UpdateScan(input, MonotonicSeconds());
}

void WallFollowerNode::ReceiveOdometry(const nav_msgs::msg::Odometry& odom)
{
    if (trajectory_.Record(odom)) pathPublisher_->publish(trajectory_.GetPath());
}

void WallFollowerNode::Tick() { commands_.Publish(controller_.Command(MonotonicSeconds())); }
void WallFollowerNode::Stop() { commands_.Publish(Velocity()); }
}
}
