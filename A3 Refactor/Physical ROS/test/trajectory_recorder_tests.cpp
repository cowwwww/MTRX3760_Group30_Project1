#include "project1/ros2/TrajectoryRecorder.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
using project1::ros2::TrajectoryRecorder;
void Require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}
nav_msgs::msg::Odometry Sample(double x, int seconds = 1, unsigned int nanoseconds = 0)
{
    nav_msgs::msg::Odometry sample;
    sample.header.frame_id = "odom";
    sample.header.stamp.sec = seconds;
    sample.header.stamp.nanosec = nanoseconds;
    sample.pose.pose.position.x = x;
    sample.pose.pose.orientation.w = 1.0;
    return sample;
}
void Spacing()
{
    TrajectoryRecorder recorder;
    Require(recorder.Record(Sample(0)), "First point not accepted");
    Require(!recorder.Record(Sample(0.029, 2)), "Sub-threshold point accepted");
    Require(recorder.Record(Sample(0.03, 3)), "Exact 3 cm boundary not accepted");
    Require(recorder.GetPath().poses.size() == 2, "Unexpected point count");
}
void Capacity()
{
    TrajectoryRecorder recorder;
    for (int index = 0; index < 10002; ++index)
        Require(recorder.Record(Sample(index * 0.05, index + 1)), "Spaced point rejected");
    const auto& path = recorder.GetPath();
    Require(path.poses.size() == 10000, "Path storage is not bounded");
    Require(std::fabs(path.poses.front().pose.position.x - 0.1) < 1e-9,
            "Oldest two points were not removed");
    Require(std::fabs(path.poses.back().pose.position.x - 500.05) < 1e-9,
            "Latest point was not retained");
}
void InvalidData()
{
    TrajectoryRecorder recorder;
    recorder.Record(Sample(0));
    auto bad = Sample(std::numeric_limits<double>::quiet_NaN(), 2);
    Require(!recorder.Record(bad), "NaN position accepted");
    bad = Sample(0.1, 2); bad.pose.pose.position.y = INFINITY;
    Require(!recorder.Record(bad), "Infinite position accepted");
    bad = Sample(0.1, 2); bad.header.frame_id.clear();
    Require(!recorder.Record(bad), "Empty frame accepted");
    Require(recorder.GetPath().poses.size() == 1 && recorder.GetPath().header.stamp.sec == 1,
            "Invalid input changed the stored path");
}
void FrameReset()
{
    TrajectoryRecorder recorder;
    recorder.Record(Sample(0));
    auto sample = Sample(0, 2); sample.header.frame_id = "new_odom";
    Require(recorder.Record(sample), "New frame with unchanged position was rejected");
    Require(recorder.GetPath().poses.size() == 1 && recorder.GetPath().header.frame_id == "new_odom",
            "Frame change failed to reset the path");
}
void TimeReset()
{
    TrajectoryRecorder recorder;
    recorder.Record(Sample(0, 1, 900));
    Require(recorder.Record(Sample(0, 1, 800)), "Backward nanoseconds did not reset the path");
    Require(recorder.GetPath().poses.size() == 1 && recorder.GetPath().header.stamp.nanosec == 800,
            "Timestamp precision was lost");
}
void PoseCopy()
{
    TrajectoryRecorder recorder;
    auto sample = Sample(0.1, 7, 123);
    sample.pose.pose.position.z = 0.2;
    sample.pose.pose.orientation.z = 0.3;
    sample.pose.pose.orientation.w = 0.8;
    recorder.Record(sample);
    Require(recorder.GetPath().poses.front().pose == sample.pose.pose &&
            recorder.GetPath().poses.front().header == sample.header, "Pose/header changed");
}
}
int main(int argc, char** argv)
{
    try
    {
        if (argc != 2) throw std::runtime_error("Expected a named check");
        const std::string check = argv[1];
        if (check == "spacing") Spacing();
        else if (check == "capacity") Capacity();
        else if (check == "invalid_data") InvalidData();
        else if (check == "frame_reset") FrameReset();
        else if (check == "time_reset") TimeReset();
        else if (check == "pose_copy") PoseCopy();
        else throw std::runtime_error("Unknown check");
        std::cout << "PASS " << check << '\n';
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
