// Project 1: ROS-independent laser processing and right-wall control.
#ifndef PROJECT1_WALL_FOLLOWER_H
#define PROJECT1_WALL_FOLLOWER_H

#include <vector>

namespace project1
{
struct LaserScan
{
    double angleMin = 0.0;
    double angleIncrement = 0.0;
    double rangeMin = 0.0;
    double rangeMax = 0.0;
    std::vector<float> ranges;
};

struct Velocity
{
    double linear = 0.0;   // metres/second, forward positive
    double angular = 0.0;  // radians/second, left positive (ROS convention)
};

struct Settings
{
    double wallDistance = 0.30; // from laser origin, not robot skin
    double forwardSpeed = 0.10;
    double turnSpeed = 0.60;
    double searchSpeed = 0.08;
    double searchTurnSpeed = 0.28;
    double frontStop = 0.40;
    double frontRelease = 0.50;
    double lostWall = 0.65;
    double distanceGain = 1.5;
    double headingGain = 1.2;
    double scanTimeout = 0.50;
    double laserYaw = 0.0; // laser heading relative to robot, radians
    void Validate() const;
};

class ScanProcessor
{
public:
    struct Reading
    {
        double distance = 0.0;
        bool valid = false;
    };
    // Uses scan angles, so both [-pi, pi] and [0, 2*pi] scans work.
    static Reading Sector(const LaserScan& scan, double centre,
                          double halfWidth, double laserYaw, bool minimum);
};

class WallFollower
{
public:
    enum class State { WaitingForScan, FollowWall, TurnLeft, FindWall };
    explicit WallFollower(const Settings& settings = Settings());
    // now is monotonic elapsed time in seconds; a bad scan immediately stops motion.
    void UpdateScan(const LaserScan& scan, double now);
    Velocity Command(double now);
    State GetState() const;

private:
    Settings settings_;
    ScanProcessor::Reading front_;
    ScanProcessor::Reading right_;
    ScanProcessor::Reading diagonal_;
    double lastScan_;
    bool valid_;
    State state_;
};
}
#endif
