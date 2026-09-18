// Closed-loop regression: ray-cast scans drive a differential-drive model.
// This is a controller test, not evidence of Gazebo or physical functionality.
#include "project1/WallFollower.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace
{
const double pi = 3.14159265358979323846;
struct Point { double x; double y; };
struct Wall { Point a; Point b; };
double Cross(Point a, Point b) { return a.x * b.y - a.y * b.x; }
double Ray(Point origin, Point direction, const Wall& wall)
{
    Point segment{wall.b.x - wall.a.x, wall.b.y - wall.a.y};
    Point offset{wall.a.x - origin.x, wall.a.y - origin.y};
    double denominator = Cross(direction, segment);
    if (std::fabs(denominator) < 1e-9) return 8.0;
    double distance = Cross(offset, segment) / denominator;
    double along = Cross(offset, direction) / denominator;
    return distance >= 0.0 && along >= 0.0 && along <= 1.0 ? distance : 8.0;
}
double Clearance(Point point, const Wall& wall)
{
    double dx = wall.b.x - wall.a.x, dy = wall.b.y - wall.a.y;
    double t = ((point.x - wall.a.x) * dx + (point.y - wall.a.y) * dy) / (dx * dx + dy * dy);
    t = std::max(0.0, std::min(1.0, t));
    return std::hypot(point.x - wall.a.x - t * dx, point.y - wall.a.y - t * dy);
}
std::vector<Wall> LoadWalls()
{
    std::ifstream file(PROJECT1_MAZE_FILE);
    if (!file) throw std::runtime_error("Missing maze fixture");
    std::vector<Wall> surfaces;
    std::string line;
    while (std::getline(file, line))
    {
        if (line.empty() || line[0] == '#') continue;
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream row(line);
        Wall centre;
        if (!(row >> centre.a.x >> centre.a.y >> centre.b.x >> centre.b.y))
            throw std::runtime_error("Invalid maze fixture");
        double xmin = std::min(centre.a.x, centre.b.x) - 0.05;
        double xmax = std::max(centre.a.x, centre.b.x) + 0.05;
        double ymin = std::min(centre.a.y, centre.b.y) - 0.05;
        double ymax = std::max(centre.a.y, centre.b.y) + 0.05;
        surfaces.push_back({{xmin, ymin}, {xmax, ymin}});
        surfaces.push_back({{xmax, ymin}, {xmax, ymax}});
        surfaces.push_back({{xmax, ymax}, {xmin, ymax}});
        surfaces.push_back({{xmin, ymax}, {xmin, ymin}});
    }
    return surfaces;
}
void Navigate(const std::vector<Wall>& walls, double startHeading, int resolution, bool realistic = false)
{
    project1::WallFollower controller;
    Point position{0.6, 0.35};
    double heading = startHeading;
    const double dt = 0.05;
    double minimumClearance = 8.0;
    bool sawLeft = false, sawRight = false;
    std::mt19937 random(30);
    std::normal_distribution<double> noise(0.0, 0.01);
    for (int step = 0; step < 18000; ++step)
    {
        if (!realistic || step % 4 == 0)
        {
            project1::LaserScan scan;
            scan.angleMin = -pi;
            scan.angleIncrement = 2.0 * pi / resolution;
            scan.rangeMin = realistic ? 0.12 : 0.02;
            scan.rangeMax = realistic ? 3.5 : 8.0;
            Point laser = position;
            if (realistic)
            {
                laser.x -= 0.064 * std::cos(heading);
                laser.y -= 0.064 * std::sin(heading);
            }
            for (int i = 0; i < resolution; ++i)
            {
                double angle = heading + scan.angleMin + i * scan.angleIncrement;
                Point direction{std::cos(angle), std::sin(angle)};
                double distance = scan.rangeMax;
                for (const auto& wall : walls) distance = std::min(distance, Ray(laser, direction, wall));
                if (realistic) distance = std::min(scan.rangeMax, distance + noise(random));
                scan.ranges.push_back(static_cast<float>(distance));
            }
            controller.UpdateScan(scan, step * dt);
        }
        auto velocity = controller.Command(step * dt);
        sawLeft = sawLeft || velocity.angular > 0.2;
        sawRight = sawRight || velocity.angular < -0.2;
        position.x += velocity.linear * std::cos(heading + velocity.angular * dt / 2.0) * dt;
        position.y += velocity.linear * std::sin(heading + velocity.angular * dt / 2.0) * dt;
        heading += velocity.angular * dt;
        for (const auto& wall : walls)
        {
            double clearance = Clearance(position, wall);
            minimumClearance = std::min(minimumClearance, clearance);
            if (clearance < 0.20)
            {
                std::ostringstream error;
                error << "Collision at " << position.x << ',' << position.y << " clearance=" << clearance;
                throw std::runtime_error(error.str());
            }
        }
        // Exit is an assertion in the test only; the controller never sees it.
        if (position.x > 6.25 && position.y > 0.20 && position.y < 1.0)
        {
            if (!sawLeft || !sawRight) throw std::runtime_error("Corner coverage missing");
            std::cout << "Maze exited in " << step * dt << " s; minimum clearance "
                      << minimumClearance << " m; rays=" << resolution << "; noisy 5 Hz=" << realistic << '\n';
            return;
        }
    }
    std::ostringstream error;
    error << "Maze exit not reached; final position " << position.x << ',' << position.y;
    throw std::runtime_error(error.str());
}
}
int main()
{
    try
    {
        auto walls = LoadWalls();
        Navigate(walls, 0.0, 360);
        Navigate(walls, 0.12, 720);
        Navigate(walls, -0.12, 360);
        Navigate(walls, 0.0, 360, true);
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
