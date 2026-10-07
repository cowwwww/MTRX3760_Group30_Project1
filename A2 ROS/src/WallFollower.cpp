// A2 right-wall steering; A1 Gazebo uses a separate waypoint strategy.
#include "project1/WallFollower.h"

#include <algorithm>
#include <cmath>

namespace project1
{
namespace
{
const double pi = 3.14159265358979323846;
double Clamp(double value, double limit)
{
    return std::max(-limit, std::min(limit, value));
}
}

WallFollower::WallFollower(const Settings& settings)
    : settings_(settings), lastScan_(0.0), valid_(false), state_(State::WaitingForScan)
{
    settings_.Validate();
}

void WallFollower::UpdateScan(const LaserScan& scan, double now)
{
    front_ = ScanProcessor::Sector(scan, 0.0, 35.0 * pi / 180.0, settings_.laserYaw, true);
    right_ = ScanProcessor::Sector(scan, -pi / 2.0, 5.0 * pi / 180.0, settings_.laserYaw, false);
    diagonal_ = ScanProcessor::Sector(scan, -pi / 4.0, 5.0 * pi / 180.0, settings_.laserYaw, false);
    valid_ = std::isfinite(now) && front_.valid && right_.valid && diagonal_.valid;
    lastScan_ = now;
    if (!valid_) state_ = State::WaitingForScan;
}

Velocity WallFollower::Command(double now)
{
    Velocity command;
    if (!valid_ || !std::isfinite(now) || now < lastScan_ ||
        now - lastScan_ > settings_.scanTimeout)
    {
        state_ = State::WaitingForScan;
        return command;
    }
    if (front_.distance < settings_.frontStop ||
        (state_ == State::TurnLeft && front_.distance < settings_.frontRelease))
    {
        state_ = State::TurnLeft;
        command.angular = settings_.turnSpeed;
        return command;
    }
    if (right_.distance > settings_.lostWall)
    {
        state_ = State::FindWall;
        command.linear = settings_.searchSpeed;
        command.angular = -settings_.searchTurnSpeed;
        return command;
    }
    state_ = State::FollowWall;
    // Two rays estimate wall orientation. Ignore the diagonal at an opening,
    // where it no longer measures the same wall as the right-hand ray.
    double wallAngle = 0.0;
    if (diagonal_.distance < settings_.lostWall * 1.5)
    {
        const double diagonal = diagonal_.distance;
        wallAngle = std::atan2(diagonal * std::cos(pi / 4.0) - right_.distance,
                              diagonal * std::sin(pi / 4.0));
    }
    const double distance = right_.distance * std::cos(wallAngle);
    command.angular = Clamp(settings_.distanceGain * (settings_.wallDistance - distance)
                          - settings_.headingGain * wallAngle, settings_.turnSpeed);
    command.linear = settings_.forwardSpeed *
                     (1.0 - 0.5 * std::fabs(command.angular) / settings_.turnSpeed);
    return command;
}

WallFollower::State WallFollower::GetState() const
{
    return state_;
}
}
