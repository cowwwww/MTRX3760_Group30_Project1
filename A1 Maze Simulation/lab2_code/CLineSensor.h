//-----------------------------------------------------------------------------
// CLineSensor.h
//
// Floor sensor for checking the line.
//-----------------------------------------------------------------------------

#ifndef CLINESENSOR_H
#define CLINESENSOR_H

#include "CLoopReader.h"

class CWallMap;

class CLineSensor
{
    public:
        CLineSensor( float aForwardOffset, float aRightOffset );

        bool Read( const CWallMap& arWallMap, const CPose& arRobotPose ) const;

    private:
        float mForwardOffset;
        float mRightOffset;
};

#endif
