// Behaviour tests use geometric laser scans, independently of ROS and raylib.
#include "project1/WallFollower.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
const double pi = 3.14159265358979323846;
int checks = 0;
void Check(bool condition, const char* description)
{
    ++checks;
    if (!condition) throw std::runtime_error(description);
}
project1::LaserScan Wall(double distance, double heading = 0.0,
                        double start = -pi, int count = 360)
{
    project1::LaserScan scan;
    scan.angleMin = start;
    scan.angleIncrement = 2.0 * pi / count;
    scan.rangeMin = 0.02;
    scan.rangeMax = 8.0;
    for (int i = 0; i < count; ++i)
    {
        const double dy = std::sin(start + i * scan.angleIncrement + heading);
        double range = dy < -1e-6 ? -distance / dy : scan.rangeMax + 1.0;
        scan.ranges.push_back(range <= scan.rangeMax ? static_cast<float>(range) :
                              std::numeric_limits<float>::infinity());
    }
    return scan;
}
void Front(project1::LaserScan& scan, double distance)
{
    for (std::size_t i = 0; i < scan.ranges.size(); ++i)
    {
        double angle = scan.angleMin + i * scan.angleIncrement;
        if (std::fabs(std::atan2(std::sin(angle), std::cos(angle))) < pi / 6.0)
            scan.ranges[i] = static_cast<float>(distance);
    }
}
bool Stopped(project1::Velocity command)
{
    return command.linear == 0.0 && command.angular == 0.0;
}
}
int main()
{
    try
    {
        project1::WallFollower follower;
        Check(Stopped(follower.Command(0.0)), "Must wait for first scan");
        follower.UpdateScan(Wall(0.30), 1.0);
        auto command = follower.Command(1.0);
        Check(command.linear > 0.09 && std::fabs(command.angular) < 0.02, "Parallel wall: straight");
        follower.UpdateScan(Wall(0.50), 2.0);
        Check(follower.Command(2.0).angular < 0.0, "Distant right wall: steer right");
        follower.UpdateScan(Wall(0.22), 3.0);
        Check(follower.Command(3.0).angular > 0.0, "Close right wall: steer left");
        follower.UpdateScan(Wall(0.30, 0.20), 4.0);
        Check(follower.Command(4.0).angular < 0.0, "Heading away from wall: steer right");
        follower.UpdateScan(Wall(0.30, -0.20), 5.0);
        Check(follower.Command(5.0).angular > 0.0, "Heading into wall: steer left");
        auto scan = Wall(0.30);
        Front(scan, 0.30);
        follower.UpdateScan(scan, 6.0);
        command = follower.Command(6.0);
        Check(command.linear == 0.0 && command.angular > 0.0, "Obstacle: pivot left");
        Front(scan, 0.45);
        follower.UpdateScan(scan, 6.1);
        Check(follower.Command(6.1).linear == 0.0, "Turn hysteresis prevents chatter");
        Front(scan, 0.60);
        follower.UpdateScan(scan, 6.2);
        Check(follower.Command(6.2).linear > 0.0, "Resume when front clears");
        scan = Wall(0.30);
        std::fill(scan.ranges.begin(), scan.ranges.end(), std::numeric_limits<float>::infinity());
        follower.UpdateScan(scan, 7.0);
        command = follower.Command(7.0);
        Check(command.linear > 0.0 && command.angular < 0.0, "Open right corner: search right");
        Check(Stopped(follower.Command(7.51)), "Stale scan: stop");
        Check(Stopped(follower.Command(6.0)), "Backward clock: stop");
        Check(Stopped(follower.Command(std::numeric_limits<double>::quiet_NaN())), "Bad clock: stop");
        for (float bad : {0.0f, -1.0f, 20.0f, -std::numeric_limits<float>::infinity(),
                          std::numeric_limits<float>::quiet_NaN()})
        {
            std::fill(scan.ranges.begin(), scan.ranges.end(), bad);
            follower.UpdateScan(scan, 8.0);
            Check(Stopped(follower.Command(8.0)), "Invalid scan must not masquerade as open space");
        }
        for (int resolution : {360, 720, 1080})
        {
            follower.UpdateScan(Wall(0.30, 0.0, 0.0, resolution), 9.0);
            command = follower.Command(9.0);
            Check(command.linear > 0.09 && std::fabs(command.angular) < 0.02,
                  "Angle indexing must support zero origin and different resolutions");
        }
        scan = Wall(0.30);
        std::reverse(scan.ranges.begin(), scan.ranges.end());
        scan.angleMin += (scan.ranges.size() - 1) * scan.angleIncrement;
        scan.angleIncrement = -scan.angleIncrement;
        follower.UpdateScan(scan, 10.0);
        Check(follower.Command(10.0).linear > 0.09, "Reversed scan order");
        scan.ranges.resize(10);
        follower.UpdateScan(scan, 11.0);
        Check(Stopped(follower.Command(11.0)), "Missing angular sectors: stop");
        scan = Wall(0.30);
        scan.angleIncrement = 0.0;
        follower.UpdateScan(scan, 12.0);
        Check(Stopped(follower.Command(12.0)), "Invalid scan metadata: stop");
        scan = Wall(0.30);
        scan.ranges[180] = 0.25f;
        follower.UpdateScan(scan, 13.0);
        Check(follower.Command(13.0).linear == 0.0, "Single thin frontal obstacle must be detected");
        project1::Settings settings;
        settings.laserYaw = pi;
        project1::WallFollower rotated(settings);
        scan = Wall(0.30, pi);
        rotated.UpdateScan(scan, 14.0);
        Check(rotated.Command(14.0).linear > 0.09, "Configured laser orientation");
        settings.frontRelease = settings.frontStop;
        bool rejected = false;
        try { project1::WallFollower invalid(settings); }
        catch (const std::invalid_argument&) { rejected = true; }
        Check(rejected, "Reject inconsistent tuning parameters");
        std::cout << checks << " controller checks passed\n";
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
