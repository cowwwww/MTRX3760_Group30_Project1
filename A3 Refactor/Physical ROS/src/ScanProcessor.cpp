#include "project1/ScanProcessor.h"

#include <algorithm>
#include <cmath>

namespace project1
{
namespace
{
const double pi = 3.14159265358979323846;
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

}
