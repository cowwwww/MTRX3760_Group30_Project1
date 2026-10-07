// Project 1 ROS 1 adapter: LaserScan -> controller -> cmd_vel; odometry -> path.
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
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

class Turtlebot3Drive
{
public:
    Turtlebot3Drive() : privateNode_("~")
    {
        project1::Settings settings;
        privateNode_.param("wall_distance", settings.wallDistance, settings.wallDistance);
        privateNode_.param("forward_speed", settings.forwardSpeed, settings.forwardSpeed);
        privateNode_.param("turn_speed", settings.turnSpeed, settings.turnSpeed);
        privateNode_.param("search_speed", settings.searchSpeed, settings.searchSpeed);
        privateNode_.param("search_turn_speed", settings.searchTurnSpeed, settings.searchTurnSpeed);
        privateNode_.param("front_stop", settings.frontStop, settings.frontStop);
        privateNode_.param("front_release", settings.frontRelease, settings.frontRelease);
        privateNode_.param("lost_wall", settings.lostWall, settings.lostWall);
        privateNode_.param("distance_gain", settings.distanceGain, settings.distanceGain);
        privateNode_.param("heading_gain", settings.headingGain, settings.headingGain);
        privateNode_.param("scan_timeout", settings.scanTimeout, settings.scanTimeout);
        privateNode_.param("laser_yaw", settings.laserYaw, settings.laserYaw);
        controller_.reset(new project1::WallFollower(settings));
        publisher_ = node_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        pathPublisher_ = node_.advertise<nav_msgs::Path>("trajectory", 1, true);
        scanSubscriber_ = node_.subscribe("scan", 1, &Turtlebot3Drive::Scan, this);
        odomSubscriber_ = node_.subscribe("odom", 1, &Turtlebot3Drive::RecordPath, this);
        ROS_INFO("Right-wall follower ready; waiting for valid laser data");
    }
    void Tick() { Publish(controller_->Command(MonotonicSeconds())); }
    void Stop() { Publish(project1::Velocity()); }

private:
    void Scan(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        project1::LaserScan input;
        input.angleMin = scan->angle_min;
        input.angleIncrement = scan->angle_increment;
        input.rangeMin = scan->range_min;
        input.rangeMax = scan->range_max;
        input.ranges = scan->ranges;
        controller_->UpdateScan(input, MonotonicSeconds());
    }
    void Publish(const project1::Velocity& velocity)
    {
        geometry_msgs::Twist command;
        command.linear.x = velocity.linear;
        command.angular.z = velocity.angular;
        publisher_.publish(command);
    }
    void RecordPath(const nav_msgs::Odometry::ConstPtr& odom)
    {
        if (odom->header.frame_id.empty() || !std::isfinite(odom->pose.pose.position.x) ||
            !std::isfinite(odom->pose.pose.position.y)) return;
        if (path_.header.frame_id != odom->header.frame_id ||
            odom->header.stamp < path_.header.stamp) path_.poses.clear();
        if (!path_.poses.empty())
        {
            const auto& last = path_.poses.back().pose.position;
            if (std::hypot(last.x - odom->pose.pose.position.x,
                           last.y - odom->pose.pose.position.y) < 0.03) return;
        }
        geometry_msgs::PoseStamped pose;
        pose.header = odom->header;
        pose.pose = odom->pose.pose;
        path_.header = odom->header;
        if (path_.poses.size() >= 10000) path_.poses.erase(path_.poses.begin());
        path_.poses.push_back(pose);
        pathPublisher_.publish(path_);
    }
    ros::NodeHandle node_;
    ros::NodeHandle privateNode_;
    std::unique_ptr<project1::WallFollower> controller_;
    ros::Publisher publisher_;
    ros::Publisher pathPublisher_;
    ros::Subscriber scanSubscriber_;
    ros::Subscriber odomSubscriber_;
    nav_msgs::Path path_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "project1_wall_follower", ros::init_options::NoSigintHandler);
    std::signal(SIGINT, RequestStop);
    std::signal(SIGTERM, RequestStop);
    try
    {
        Turtlebot3Drive node;
        while (ros::ok() && !stopRequested)
        {
            ros::spinOnce();
            node.Tick();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (ros::ok())
        {
            node.Stop();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    catch (const std::exception& error)
    {
        ROS_FATAL("%s", error.what());
        ros::shutdown();
        return 1;
    }
    ros::shutdown();
    return 0;
}
