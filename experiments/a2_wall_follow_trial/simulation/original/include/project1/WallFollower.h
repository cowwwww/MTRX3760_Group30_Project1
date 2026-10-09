// Right-wall state and steering, independent of ROS.
#ifndef PROJECT1_WALLFOLLOWER_H
#define PROJECT1_WALLFOLLOWER_H

#include "project1/ScanProcessor.h"
#include "project1/Settings.h"

namespace project1
{
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
