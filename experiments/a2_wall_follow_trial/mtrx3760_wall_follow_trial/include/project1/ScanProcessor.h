// Angular sector selection and scan validity checks.
#ifndef PROJECT1_SCANPROCESSOR_H
#define PROJECT1_SCANPROCESSOR_H

#include "project1/ControlTypes.h"
#include "project1/Settings.h"

namespace project1
{
class ScanProcessor
{
public:
    struct Reading
    {
        double distance = 0.0;
        bool valid = false;
        unsigned int usable = 0, total = 0;
    };
    // Uses scan angles, so both [-pi, pi] and [0, 2*pi] scans work.
    static Reading Sector(const LaserScan& scan, double centre,
                          double halfWidth, double laserYaw, bool minimum);
    struct Clearance
    {
        bool frontValid = false, pivotValid = false;
        double front = 0.0, pivot = 0.0; // body-edge and swept-circle clearances
        double frontUnknownSpan = 0.0, pivotUnknownSpan = 0.0;
    };
    static Clearance MeasureClearance(const LaserScan& scan, const Settings& settings);
    // Estimate wall direction for corner-edge extrapolation only.
    static bool RightWallAngle(const LaserScan& scan, const Settings& settings,
                               double rightDistance, double& angle);
    // Finite returns beyond the predicted old wall support an opening.
    // distance is an upper bound on the edge's along-wall position from the laser.
    static Reading Opening(const LaserScan& scan, const Settings& settings,
                           double normalDistance, double tangentHeading);
};
}
#endif
