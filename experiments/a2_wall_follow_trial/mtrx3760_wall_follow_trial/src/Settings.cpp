#include "project1/Settings.h"

#include <cmath>
#include <stdexcept>

namespace project1
{
void Settings::Validate() const
{
    const double positive[] = {wallDistance, forwardSpeed, turnSpeed, searchSpeed,
        searchTurnSpeed, frontStop, frontRelease, lostWall, distanceGain,
        headingGain, scanTimeout, bodyFront, bodyRear, bodyHalfWidth, clearanceMargin,
        brakingDeceleration, commandLatency, maxUnobservedSpan,
        maxFrontUnobservedSpan, distanceRateGain, cornerLookahead,
        cornerTimeout, cornerMaxEntry};
    for (double value : positive)
    {
        if (!std::isfinite(value) || value <= 0.0)
            throw std::invalid_argument("Controller settings must be finite and positive");
    }
    if (!std::isfinite(laserYaw) || !std::isfinite(laserX) || !std::isfinite(laserY) ||
        laserX <= -bodyRear || laserX >= bodyFront || std::fabs(laserY) >= bodyHalfWidth ||
        maxUnobservedSpan > bodyHalfWidth || maxFrontUnobservedSpan > bodyHalfWidth ||
        maxFrontUnobservedSpan < maxUnobservedSpan || commandLatency < scanTimeout ||
        frontRelease <= frontStop ||
        lostWall <= wallDistance || searchSpeed > forwardSpeed ||
        searchTurnSpeed > turnSpeed)
        throw std::invalid_argument("Invalid controller threshold/speed ordering");
}

}
