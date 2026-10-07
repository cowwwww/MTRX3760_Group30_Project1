// Identical generated inputs for the pre-refactor and refactored controllers.
#ifdef TRACE_A1
#if __has_include("tb3_maze/c_right_wall_follower_robot.h")
#include "tb3_maze/c_right_wall_follower_robot.h"
#else
#include "tb3_maze/c_robot.h"
#endif
#else
#include "project1/WallFollower.h"
#endif
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

int main()
{
    constexpr double pi = 3.14159265358979323846;
    std::mt19937 generator(30);
#ifdef TRACE_A1
    CRightWallFollowerRobot controller;
#else
    project1::WallFollower controller;
#endif
    std::cout << std::hexfloat;
    for (int step = 0; step < 2000; ++step)
    {
        const double right = 0.15 + (generator() % 1100) / 1000.0;
        const double front = step % 7 == 0 ? INFINITY : 0.15 + (generator() % 1400) / 1000.0;
        const double start = step % 2 == 0 ? -pi : 0.0;
        const double increment = (step % 3 == 0 ? -1 : 1) * pi / 180.0;
        std::vector<float> ranges;
        for (int index = 0; index < 360; ++index)
        {
            const double angle = start + index * increment;
            const double sine = std::sin(angle), cosine = std::cos(angle);
            double value = sine < -1e-9 ? right / -sine : INFINITY;
            if (cosine > 1e-9) value = std::min(value, front / cosine);
            ranges.push_back(static_cast<float>(value <= 3.5 ? value : INFINITY));
        }
        if (step % 41 == 0) ranges[270] = std::numeric_limits<float>::quiet_NaN();
        if (step % 101 == 0) ranges.clear();
        std::cout << step << ' ';
#ifdef TRACE_A1
        sensor_msgs::msg::LaserScan scan;
        scan.angle_min = start; scan.angle_increment = increment;
        scan.angle_max = start + 359 * increment;
        scan.range_min = 0.12; scan.range_max = 3.5; scan.ranges = ranges;
        const CPose pose{{float(step * 0.001), float(std::sin(step * 0.03))},
                         float(std::sin(step * 0.01))};
        controller.Update(scan, pose);
        std::cout << controller.GetLeftWheelSpeed() << ' ' << controller.GetRightWheelSpeed()
                  << ' ' << controller.GetLinearVelocity() << ' ' << controller.GetAngularVelocity()
                  << ' ' << controller.GetWaypoints().size();
        for (const auto& point : controller.GetWaypoints())
            std::cout << ' ' << point.mX << ' ' << point.mY;
#else
        project1::LaserScan scan;
        scan.angleMin = start; scan.angleIncrement = increment;
        scan.rangeMin = 0.12; scan.rangeMax = 3.5; scan.ranges = ranges;
        const double arrival = 1.0 + step * 0.1;
        controller.UpdateScan(scan, arrival);
        const double elapsed = step % 13 == 0 ? 0.6 : step % 17 == 0 ? -0.1 : 0.05;
        const auto velocity = controller.Command(arrival + elapsed);
        std::cout << velocity.linear << ' ' << velocity.angular << ' ' << int(controller.GetState());
#endif
        std::cout << '\n';
    }
}
