#include "project1/ScanProcessor.h"

#include <algorithm>
#include <cmath>
#include <limits>

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
        // No +inf free-space contract was verified for the physical LD19.
        if (std::isfinite(range) && range > 0.0 &&
                 range >= scan.rangeMin && range <= scan.rangeMax)
            values.push_back(range);
    }
    // Missing angular coverage and mostly corrupt sectors are not free space.
    const double expectedRays = 2.0 * halfWidth / std::fabs(scan.angleIncrement);
    result.usable = values.size(); result.total = total;
    if (total == 0 || total < 0.7 * expectedRays || values.size() * 5 < total * 3 ||
        nearestAngle > halfWidth / 2.0)
        return result;
    std::sort(values.begin(), values.end());
    result.distance = minimum ? values.front() : values[values.size() / 2];
    result.valid = true;
    return result;
}

ScanProcessor::Clearance ScanProcessor::MeasureClearance(const LaserScan& scan, const Settings& s)
{
    Clearance result;
    const auto wide = Sector(scan,0.0,35.0*pi/180.0,s.laserYaw,true);
    const auto direct = Sector(scan,0.0,10.0*pi/180.0,s.laserYaw,true);
    const auto whole = Sector(scan,0.0,pi,s.laserYaw,true);
    if (!whole.valid) return result;
    result.front = result.pivot = std::numeric_limits<double>::infinity();
    const double radius = std::hypot(std::max(s.bodyFront,s.bodyRear),s.bodyHalfWidth)+s.clearanceMargin;
    std::vector<double> bearings, frontBearings;
    for (std::size_t i=0;i<scan.ranges.size();++i)
    {
        const double r=scan.ranges[i], a=scan.angleMin+i*scan.angleIncrement+s.laserYaw;
        if (!std::isfinite(r) || r<=0.0 || r<scan.rangeMin || r>scan.rangeMax) continue;
        const double angle=std::atan2(std::sin(a),std::cos(a));
        const double x=s.laserX+r*std::cos(a), y=s.laserY+r*std::sin(a);
        result.pivot=std::min(result.pivot,std::hypot(x,y)-radius);
        if (x>=-s.bodyRear && std::fabs(y)<=s.bodyHalfWidth+s.clearanceMargin)
            result.front=std::min(result.front,x-s.bodyFront);
        bearings.push_back(angle);
        if (std::fabs(angle)<=35.0*pi/180.0) frontBearings.push_back(angle);
    }
    const auto maximumGap=[](std::vector<double> angles,bool circular)
    {
        if (angles.size()<3) return 2.0*pi;
        std::sort(angles.begin(),angles.end());
        if (circular) angles.push_back(angles.front()+2.0*pi);
        else { angles.insert(angles.begin(),-35.0*pi/180.0); angles.push_back(35.0*pi/180.0); }
        double gap=0.0;
        for (std::size_t i=1;i<angles.size();++i) gap=std::max(gap,angles[i]-angles[i-1]);
        return gap;
    };
    result.frontUnknownSpan=maximumGap(frontBearings,false)*(s.frontStop+s.bodyFront+std::fabs(s.laserX));
    result.pivotUnknownSpan=maximumGap(bearings,true)*(radius+std::hypot(s.laserX,s.laserY));
    result.frontValid=wide.valid && direct.valid && result.frontUnknownSpan<=s.maxUnobservedSpan;
    result.pivotValid=result.pivotUnknownSpan<=s.maxUnobservedSpan;
    return result;
}

ScanProcessor::Reading ScanProcessor::Opening(const LaserScan& scan,const Settings& s,
                                             double distance,double heading)
{
    Reading gap; gap.distance=std::numeric_limits<double>::infinity();
    if (!std::isfinite(distance) || distance<=0.0 || !std::isfinite(heading)) return gap;
    const double tx=std::cos(heading),ty=std::sin(heading);
    for (std::size_t i=0;i<scan.ranges.size();++i)
    {
        const double r=scan.ranges[i],a=scan.angleMin+i*scan.angleIncrement+s.laserYaw;
        if (!std::isfinite(r) || r<=0.0 || r<scan.rangeMin || r>scan.rangeMax) continue;
        const double normal=ty*std::cos(a)-tx*std::sin(a);
        if (normal<=1e-6 || r*normal<distance+0.10) continue;
        const double along=distance*(tx*std::cos(a)+ty*std::sin(a))/normal;
        if (along < -s.bodyRear || along > s.cornerLookahead) continue;
        ++gap.usable; gap.distance=std::min(gap.distance,along);
    }
    gap.valid=gap.usable>=3;
    return gap;
}
}
