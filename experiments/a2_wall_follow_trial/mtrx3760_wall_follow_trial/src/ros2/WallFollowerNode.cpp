#include "project1/ros2/WallFollowerNode.h"
#include <chrono>
#include <cmath>
#include <sstream>

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
    : Node("project1_wall_follower_trial"), controller_(ReadSettings()), commands_(*this)
{
    pathPublisher_ = create_publisher<nav_msgs::msg::Path>(
        "trajectory", rclcpp::QoS(1).transient_local());
    scanSubscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) { ReceiveScan(*scan); });
    odomSubscriber_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::SensorDataQoS().keep_last(1),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr odom) { ReceiveOdometry(*odom); });
    diagnosticsPublisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("controller_diagnostics", 10);
    RCLCPP_INFO(get_logger(), "A2 wall follower ready; valid scan and odometry required");
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
    settings.laserX = declare_parameter<double>("laser_x", settings.laserX);
    settings.laserY = declare_parameter<double>("laser_y", settings.laserY);
    settings.bodyFront = declare_parameter<double>("body_front", settings.bodyFront);
    settings.bodyRear = declare_parameter<double>("body_rear", settings.bodyRear);
    settings.bodyHalfWidth = declare_parameter<double>("body_half_width", settings.bodyHalfWidth);
    settings.clearanceMargin = declare_parameter<double>("clearance_margin", settings.clearanceMargin);
    settings.brakingDeceleration = declare_parameter<double>("braking_deceleration", settings.brakingDeceleration);
    settings.commandLatency = declare_parameter<double>("command_latency", settings.commandLatency);
    settings.maxUnobservedSpan = declare_parameter<double>("max_unobserved_span", settings.maxUnobservedSpan);
    settings.cornerLookahead = declare_parameter<double>("corner_lookahead", settings.cornerLookahead);
    settings.cornerTimeout = declare_parameter<double>("corner_timeout", settings.cornerTimeout);
    settings.cornerMaxEntry = declare_parameter<double>("corner_max_entry", settings.cornerMaxEntry);
    declare_parameter<std::string>("scan_frame", "base_scan");
    return settings;
}

void WallFollowerNode::ReceiveScan(const sensor_msgs::msg::LaserScan& scan)
{
    const double sourceTime = scan.header.stamp.sec + scan.header.stamp.nanosec*1e-9;
    scanSourceAge_ = this->now().seconds()-sourceTime;
    if (scan.header.frame_id != get_parameter("scan_frame").as_string())
    { controller_.InvalidateScan("scan_frame_mismatch"); return; }
    if (!std::isfinite(scanSourceAge_) || scanSourceAge_ < -0.10 ||
        scanSourceAge_ > get_parameter("scan_timeout").as_double())
    { controller_.InvalidateScan("scan_header_age_or_clock_mismatch"); return; }
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
    odomSourceAge_ = this->now().seconds()-(odom.header.stamp.sec+odom.header.stamp.nanosec*1e-9);
    const auto& p = odom.pose.pose.position;
    const auto& q = odom.pose.pose.orientation;
    const double norm = std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
    if (odom.header.frame_id.empty() || odom.child_frame_id.empty() || !std::isfinite(norm) ||
        std::fabs(norm-1.0) > 0.10)
    { controller_.InvalidateOdometry("odom_frame_or_quaternion_invalid"); return; }
    if ((!odomFrame_.empty() && odom.header.frame_id != odomFrame_) ||
        (!odomChild_.empty() && odom.child_frame_id != odomChild_))
    { controller_.InvalidateOdometry("odom_frame_changed",true); return; }
    if (!std::isfinite(odomSourceAge_) || odomSourceAge_ < -0.10 ||
        odomSourceAge_ > get_parameter("scan_timeout").as_double())
    { controller_.InvalidateOdometry("odom_header_age_or_clock_mismatch"); return; }
    odomFrame_ = odom.header.frame_id; odomChild_ = odom.child_frame_id;
    const double yaw = std::atan2(2.0*(q.w*q.z+q.x*q.y)/(norm*norm),
                                 1.0-2.0*(q.y*q.y+q.z*q.z)/(norm*norm));
    controller_.UpdateOdometry(p.x,p.y,yaw,MonotonicSeconds());
}

void WallFollowerNode::Tick()
{
    const double time = MonotonicSeconds();
    const Velocity command = controller_.Command(time);
    commands_.Publish(command);
    const auto d = controller_.GetDiagnostics(time);
    const bool transition = d.state != lastState_;
    if (transition || time-diagnosticsTime_ >= 1.0 || d.reason != lastReason_)
    {
        diagnostic_msgs::msg::DiagnosticArray array;
        array.header.stamp = now();
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "project1_wall_follower_trial/control"; status.hardware_id = "TurtleBot3 Burger";
        status.level = d.state == WallFollower::State::Stopped ? status.ERROR :
                       (command.linear == 0.0 && command.angular == 0.0 ? status.WARN : status.OK);
        status.message = std::string(WallFollower::StateName(d.state))+": "+d.reason;
        const auto add = [&status](const std::string& key, double value)
        {
            diagnostic_msgs::msg::KeyValue item; item.key = key;
            std::ostringstream text; text << value; item.value = text.str(); status.values.push_back(item);
        };
        add("scan_age",d.scanAge); add("odom_age",d.odomAge);
        add("scan_source_age",scanSourceAge_); add("odom_source_age",odomSourceAge_);
        add("front_body_clearance",d.frontClearance); add("pivot_circle_clearance",d.pivotClearance);
        add("front_valid",d.frontValid); add("pivot_valid",d.pivotValid);
        add("right_valid",d.rightValid); add("heading_valid",d.headingValid);
        add("right_usable_rays",d.rightUsable); add("right_total_rays",d.rightTotal);
        add("front_unobserved_span",d.frontUnknownSpan); add("pivot_unobserved_span",d.pivotUnknownSpan);
        add("wall_distance",d.wallDistance); add("wall_heading",d.wallHeading);
        add("gap_rays",d.gapRays); add("gap_scans",d.gapScans);
        add("advance_remaining",d.advanceRemaining);
        add("turn_error",d.turnError); add("linear_command",command.linear); add("angular_command",command.angular);
        array.status.push_back(status); diagnosticsPublisher_->publish(array);
        if (transition)
            RCLCPP_INFO(get_logger(), "%s; front=%.3f pivot=%.3f wall=%.3f scan_age=%.3f odom_age=%.3f",
                status.message.c_str(),d.frontClearance,d.pivotClearance,d.wallDistance,d.scanAge,d.odomAge);
        else if (d.reason != lastReason_)
            RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,
                "%s; front=%.3f pivot=%.3f wall=%.3f scan_age=%.3f odom_age=%.3f",
                status.message.c_str(),d.frontClearance,d.pivotClearance,d.wallDistance,d.scanAge,d.odomAge);
        diagnosticsTime_ = time; lastReason_ = d.reason; lastState_ = d.state;
    }
}
void WallFollowerNode::Stop() { commands_.Publish(Velocity()); }
}
}
