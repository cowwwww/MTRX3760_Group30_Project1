// Project 1 ROS 2 adapter: LaserScan -> controller -> cmd_vel; odometry -> path.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include "project1/WallFollower.h"
#include <chrono>
#include <cmath>
#include <csignal>
#include <memory>
#include <thread>

namespace
{
volatile std::sig_atomic_t stopRequested = 0;
void RequestStop(int) { stopRequested = 1; }
double MonotonicSeconds()
{
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
}

class Turtlebot3Drive : public rclcpp::Node
{
public:
    Turtlebot3Drive() : Node("project1_wall_follower")
    {
        project1::Settings settings;
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
        controller_.reset(new project1::WallFollower(settings));
        stamped_ = declare_parameter<bool>("stamped_cmd_vel", false);
        commandFrame_ = declare_parameter<std::string>("command_frame", "base_link");
        if (stamped_)
            stampedPublisher_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
        else
            publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        pathPublisher_ = create_publisher<nav_msgs::msg::Path>(
            "trajectory", rclcpp::QoS(1).transient_local());
        scanSubscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS().keep_last(1),
            [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
            {
                project1::LaserScan input;
                input.angleMin = scan->angle_min;
                input.angleIncrement = scan->angle_increment;
                input.rangeMin = scan->range_min;
                input.rangeMax = scan->range_max;
                input.ranges = scan->ranges;
                controller_->UpdateScan(input, MonotonicSeconds());
            });
        odomSubscriber_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::SensorDataQoS().keep_last(1),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr odom) { RecordPath(*odom); });
        RCLCPP_INFO(get_logger(), "Right-wall follower ready; waiting for valid laser data");
    }

    void Tick() { Publish(controller_->Command(MonotonicSeconds())); }
    void Stop() { Publish(project1::Velocity()); }

private:
    void Publish(const project1::Velocity& velocity)
    {
        geometry_msgs::msg::Twist command;
        command.linear.x = velocity.linear;
        command.angular.z = velocity.angular;
        if (stamped_)
        {
            geometry_msgs::msg::TwistStamped stamped;
            stamped.header.stamp = now();
            stamped.header.frame_id = commandFrame_;
            stamped.twist = command;
            stampedPublisher_->publish(stamped);
        }
        else publisher_->publish(command);
    }

    void RecordPath(const nav_msgs::msg::Odometry& odom)
    {
        if (odom.header.frame_id.empty() || !std::isfinite(odom.pose.pose.position.x) ||
            !std::isfinite(odom.pose.pose.position.y)) return;
        if (path_.header.frame_id != odom.header.frame_id ||
            rclcpp::Time(odom.header.stamp) < rclcpp::Time(path_.header.stamp))
            path_.poses.clear();
        if (!path_.poses.empty())
        {
            const auto& last = path_.poses.back().pose.position;
            if (std::hypot(last.x - odom.pose.pose.position.x,
                           last.y - odom.pose.pose.position.y) < 0.03) return;
        }
        geometry_msgs::msg::PoseStamped pose;
        pose.header = odom.header;
        pose.pose = odom.pose.pose;
        path_.header = odom.header;
        if (path_.poses.size() >= 10000) path_.poses.erase(path_.poses.begin());
        path_.poses.push_back(pose);
        pathPublisher_->publish(path_);
    }

    std::unique_ptr<project1::WallFollower> controller_;
    bool stamped_;
    std::string commandFrame_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr stampedPublisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSubscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscriber_;
    nav_msgs::msg::Path path_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, RequestStop);
    std::signal(SIGTERM, RequestStop);
    try
    {
        auto node = std::make_shared<Turtlebot3Drive>();
        while (rclcpp::ok() && !stopRequested)
        {
            rclcpp::spin_some(node);
            node->Tick();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        // Publish while the ROS context is still alive, before shutdown.
        if (rclcpp::ok())
        {
            node->Stop();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    catch (const std::exception& error)
    {
        RCLCPP_FATAL(rclcpp::get_logger("project1_wall_follower"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
