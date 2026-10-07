// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Taehun Lim (Darby), Ryan Shim
//
// Modified for MTRX3760 Project 1: the original obstacle-avoiding state machine
// is replaced by a CRobot, which turns each lidar scan and the robot's pose into
// wheel commands by choosing waypoints and driving to them (see c_robot.h).

#include "tb3_maze/turtlebot3_drive.hpp"

#include <cmath>
#include <memory>

Turtlebot3Drive::Turtlebot3Drive()
: Node("turtlebot3_drive_node"),
  robot_(std::make_unique<CRightWallFollowerRobot>()),
  have_pose_(false)
{
  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers
  // The bridge and the real robot both expect TwistStamped on cmd_vel.
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", qos);

  // The waypoints the robot is driving to, for RViz.
  waypoints_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoints", qos);

  // Initialise subscribers
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", \
    rclcpp::SensorDataQoS(), \
    std::bind(
      &Turtlebot3Drive::scan_callback, \
      this, \
      std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", qos, std::bind(&Turtlebot3Drive::odom_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Turtlebot3 right wall follower has been initialised");
}

Turtlebot3Drive::~Turtlebot3Drive()
{
  RCLCPP_INFO(this->get_logger(), "Turtlebot3 right wall follower has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/
void Turtlebot3Drive::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto & q = msg->pose.pose.orientation;

  pose_.mPosition.mX = msg->pose.pose.position.x;
  pose_.mPosition.mY = msg->pose.pose.position.y;
  pose_.mHeading = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  have_pose_ = true;
}

void Turtlebot3Drive::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // The robot needs to know where it is to keep its waypoints in place.
  if (!have_pose_) {
    return;
  }

  // Lidar and pose in, wheel commands out: one command is sent for every scan.
  robot_->Update(*msg, pose_);
  update_cmd_vel();
  publish_waypoints();
}

void Turtlebot3Drive::update_cmd_vel()
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = this->now();
  cmd_vel.header.frame_id = "base_link";
  cmd_vel.twist.linear.x = robot_->GetLinearVelocity();
  cmd_vel.twist.angular.z = robot_->GetAngularVelocity();

  cmd_vel_pub_->publish(cmd_vel);
}

void Turtlebot3Drive::publish_waypoints()
{
  // The current waypoint is a big cyan ball, the two before it smaller and fainter.
  const double sizes[] = {0.10, 0.07, 0.05};
  const double alphas[] = {1.0, 0.6, 0.35};
  const auto & waypoints = robot_->GetWaypoints();

  visualization_msgs::msg::MarkerArray markers;

  for (size_t i = 0; i < 3; ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = this->now();
    marker.ns = "waypoints";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::SPHERE;

    if (i < waypoints.size()) {
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = waypoints[i].mX;
      marker.pose.position.y = waypoints[i].mY;
      marker.pose.position.z = 0.05;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = sizes[i];
      marker.scale.y = sizes[i];
      marker.scale.z = sizes[i];
      marker.color.r = 0.0;
      marker.color.g = 0.85;
      marker.color.b = 1.0;
      marker.color.a = alphas[i];
    } else {
      marker.action = visualization_msgs::msg::Marker::DELETE;
    }

    markers.markers.push_back(marker);
  }

  waypoints_pub_->publish(markers);
}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Turtlebot3Drive>());
  rclcpp::shutdown();

  return 0;
}
