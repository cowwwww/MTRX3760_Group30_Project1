//-----------------------------------------------------------------------------
// CRangeSensor.h
//
// Range sensor for checking wall distance.
//-----------------------------------------------------------------------------

#ifndef CRANGESENSOR_H
#define CRANGESENSOR_H

#include "CLoopReader.h"

class CWallMap;

class CRangeSensor
{
    public:
        CRangeSensor( float aRelativeAngleDegrees, float aMaximumRange );

        float Read( const CWallMap& arWallMap,
                    const CPose& arRobotPose,
                    int aRobotRadius ) const;

    private:
        float mRelativeAngle;
        float mMaximumRange;
};

#endif
