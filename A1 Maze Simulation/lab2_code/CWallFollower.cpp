//-----------------------------------------------------------------------------
// CWallFollower.cpp
//-----------------------------------------------------------------------------

#include "CWallFollower.h"
#include "CWallMap.h"

CWallFollower::CWallFollower( CWallMap& arWallMap )
    : CRobot( arWallMap,
              "Wall follower",
              arWallMap.GetWallStartPose(),
              CRender::WALL_ROBOT,
              CRender::WALL_TRAIL ),
      mRightSensor( 90.0f, 300.0f ),
      mFrontRightSensor( 45.0f, 300.0f )
{
}

void CWallFollower::Control()
{
    // Two sensors are used here.
    // One checks the side, one checks the front-right corner.
    const float forwardSpeed = 45.0f;
    const float gentleTurn = 8.0f;
    const float strongTurn = 25.0f;
    const float desiredDistance = 30.0f;
    const float distanceTolerance = 5.0f;
    const float frontCornerDistance = 70.0f;
    const float lostWallDistance = 180.0f;

    float rightDistance = mRightSensor.Read( GetWallMap(), GetPose(), GetRadius() );
    float frontRightDistance = mFrontRightSensor.Read( GetWallMap(), GetPose(), GetRadius() );

    if( frontRightDistance < frontCornerDistance )
    {
        // Wall ahead-right, so turn left.
        SetWheelSpeeds( forwardSpeed - strongTurn, forwardSpeed + strongTurn );
    }
    else if( rightDistance > lostWallDistance )
    {
        // Wall is lost, so turn right.
        SetWheelSpeeds( forwardSpeed + strongTurn, forwardSpeed - strongTurn );
    }
    else if( rightDistance > desiredDistance + distanceTolerance )
    {
        SetWheelSpeeds( forwardSpeed + gentleTurn, forwardSpeed - gentleTurn );
    }
    else if( rightDistance < desiredDistance - distanceTolerance )
    {
        SetWheelSpeeds( forwardSpeed - gentleTurn, forwardSpeed + gentleTurn );
    }
    else
    {
        SetWheelSpeeds( forwardSpeed, forwardSpeed );
    }
}
