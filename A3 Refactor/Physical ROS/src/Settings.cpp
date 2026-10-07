#include "project1/Settings.h"

#include <cmath>
#include <stdexcept>

namespace project1
{
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

}
