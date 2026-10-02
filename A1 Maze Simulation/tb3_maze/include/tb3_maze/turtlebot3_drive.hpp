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

#ifndef TB3_MAZE__TURTLEBOT3_DRIVE_HPP_
#define TB3_MAZE__TURTLEBOT3_DRIVE_HPP_

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>

#include "tb3_maze/c_robot.h"

class Turtlebot3Drive : public rclcpp::Node
{
public:
  Turtlebot3Drive();
  ~Turtlebot3Drive();

private:
  // ROS topic publishers
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_pub_;

  // ROS topic subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // The control algorithm. Any CRobot can be used here.
  std::unique_ptr<CRobot> robot_;

  // Where the robot is in the odom frame, from the latest odometry
  CPose pose_;
  bool have_pose_;

  // Function prototypes
  void update_cmd_vel();
  void publish_waypoints();
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};
#endif  // TB3_MAZE__TURTLEBOT3_DRIVE_HPP_
