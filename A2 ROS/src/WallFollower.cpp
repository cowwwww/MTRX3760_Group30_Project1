// Project 1: right-wall steering shared by simulation and the physical robot.
#include "project1/WallFollower.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

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

void Settings::Validate() const
{
    const double positive[] = {wallDistance, forwardSpeed, turnSpeed, searchSpeed,
        searchTurnSpeed, frontStop, frontRelease, lostWall, distanceGain,
        headingGain, scanTimeout};
    for (double value : positive)
    {
        if (!std::isfinite(value) || value <= 0.0)
            throw std::invalid_argument("Controller settings must be finite and positive");
    }
    if (!std::isfinite(laserYaw) || frontRelease <= frontStop ||
        lostWall <= wallDistance || searchSpeed > forwardSpeed ||
        searchTurnSpeed > turnSpeed)
        throw std::invalid_argument("Invalid controller threshold/speed ordering");
}

ScanProcessor::Reading ScanProcessor::Sector(const LaserScan& scan, double centre,
    double halfWidth, double laserYaw, bool minimum)
{
    Reading result;
    if (scan.ranges.empty() || !std::isfinite(scan.angleMin) ||
        !std::isfinite(scan.angleIncrement) || scan.angleIncrement == 0.0 ||
        !std::isfinite(scan.rangeMin) || !std::isfinite(scan.rangeMax) ||
        scan.rangeMin < 0.0 || scan.rangeMax <= scan.rangeMin)
        return result;

    std::vector<double> values;
    unsigned int total = 0;
    double nearestAngle = pi;
    for (std::size_t i = 0; i < scan.ranges.size(); ++i)
    {
        const double angle = scan.angleMin + i * scan.angleIncrement + laserYaw;
        if (!std::isfinite(angle)) return result;
        const double error = std::fabs(std::atan2(std::sin(angle - centre),
                                                std::cos(angle - centre)));
        if (error > halfWidth + 1e-6) continue;
        ++total;
        nearestAngle = std::min(nearestAngle, error);
        const double range = scan.ranges[i];
        // Positive infinity is a normal no-return measurement in Gazebo/LDS.
        if (std::isinf(range) && range > 0.0)
            values.push_back(scan.rangeMax);
        else if (std::isfinite(range) && range > 0.0 &&
                 range >= scan.rangeMin && range <= scan.rangeMax)
            values.push_back(range);
    }
    // Missing angular coverage and mostly corrupt sectors are not free space.
    const double expectedRays = 2.0 * halfWidth / std::fabs(scan.angleIncrement);
    if (total == 0 || total < 0.7 * expectedRays || values.size() * 5 < total * 3 ||
        nearestAngle > halfWidth / 2.0)
        return result;
    std::sort(values.begin(), values.end());
    result.distance = minimum ? values.front() : values[values.size() / 2];
    result.valid = true;
    return result;
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
