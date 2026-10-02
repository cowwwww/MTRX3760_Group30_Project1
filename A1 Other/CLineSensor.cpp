//-----------------------------------------------------------------------------
// CLineSensor.cpp
//-----------------------------------------------------------------------------

#include "CLineSensor.h"
#include "CWallMap.h"

#include <cmath>

CLineSensor::CLineSensor( float aForwardOffset, float aRightOffset )
    : mForwardOffset( aForwardOffset ),
      mRightOffset( aRightOffset )
{
}

bool CLineSensor::Read( const CWallMap& arWallMap, const CPose& arRobotPose ) const
{
    float rightHeading = arRobotPose.mHeading + 3.14159265f / 2.0f;

    Vec2D sensorPosition =
    {
        arRobotPose.mPosition.x
            + mForwardOffset * std::cos( arRobotPose.mHeading )
            + mRightOffset * std::cos( rightHeading ),
        arRobotPose.mPosition.y
            + mForwardOffset * std::sin( arRobotPose.mHeading )
            + mRightOffset * std::sin( rightHeading )
    };

    return arWallMap.IsOnLine( sensorPosition );
}
