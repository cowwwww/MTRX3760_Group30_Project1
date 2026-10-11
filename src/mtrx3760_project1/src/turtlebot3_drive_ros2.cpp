// ROS 2 process lifecycle; the node and message helpers have separate files.
#include "project1/ros2/WallFollowerNode.h"
#include <chrono>
#include <csignal>
#include <memory>
#include <thread>

namespace
{
volatile std::sig_atomic_t stopRequested = 0;
void RequestStop(int) { stopRequested = 1; }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, RequestStop);
    std::signal(SIGTERM, RequestStop);
    try
    {
        auto node = std::make_shared<project1::ros2::WallFollowerNode>();
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
