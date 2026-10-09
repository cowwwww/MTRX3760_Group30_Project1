// Angular sector selection and scan validity checks.
#ifndef PROJECT1_SCANPROCESSOR_H
#define PROJECT1_SCANPROCESSOR_H

#include "project1/ControlTypes.h"

namespace project1
{
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
}
#endif
