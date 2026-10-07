#include "project1/ros2/TrajectoryRecorder.h"
#include <rclcpp/time.hpp>
#include <cmath>

namespace project1
{
namespace ros2
{
bool TrajectoryRecorder::Record(const nav_msgs::msg::Odometry& odom)
{
    if (odom.header.frame_id.empty() || !std::isfinite(odom.pose.pose.position.x) ||
        !std::isfinite(odom.pose.pose.position.y)) return false;
    if (path_.header.frame_id != odom.header.frame_id ||
        rclcpp::Time(odom.header.stamp) < rclcpp::Time(path_.header.stamp))
        path_.poses.clear();
    if (!path_.poses.empty())
    {
        const auto& last = path_.poses.back().pose.position;
        if (std::hypot(last.x - odom.pose.pose.position.x,
                       last.y - odom.pose.pose.position.y) < 0.03) return false;
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header = odom.header;
    pose.pose = odom.pose.pose;
    path_.header = odom.header;
    if (path_.poses.size() >= 10000) path_.poses.erase(path_.poses.begin());
    path_.poses.push_back(pose);
    return true;
}

const nav_msgs::msg::Path& TrajectoryRecorder::GetPath() const
{
    return path_;
}
}
}
