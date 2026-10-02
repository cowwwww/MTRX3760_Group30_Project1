//-----------------------------------------------------------------------------
// CRobot.cpp
//-----------------------------------------------------------------------------

#include "CRobot.h"
#include "CWallMap.h"

#include <cmath>
#include <iostream>

CRobot::CRobot( CWallMap& arWallMap,
                const std::string& arName,
                const CPose& arStartPose,
                CRender::EStyle aRobotStyle,
                CRender::EStyle aTrailStyle )
    : mrWallMap( arWallMap ),
      mName( arName ),
      mStartPose( arStartPose ),
      mPose( arStartPose ),
      mRadius( 15 ),
      mWheelSeparation( 30.0f ),
      mTimeStep( 0.04f ),
      mMaximumWheelSpeed( 80.0f ),
      mLeftWheelSpeed( 0.0f ),
      mRightWheelSpeed( 0.0f ),
      mRobotStyle( aRobotStyle ),
      mTrailStyle( aTrailStyle ),
      mUpdateCount( 0 ),
      mCollisionCount( 0 ),
      mHasLeftStart( false ),
      mCompletedLap( false )
{
    mTrail.push_back( mPose.mPosition );
}

CRobot::~CRobot()
{
}

void CRobot::Update()
{
    if( !mCompletedLap )
    {
        ++mUpdateCount;
        Control();
        Move();
        CheckLapCompletion();
        mTrail.push_back( mPose.mPosition );
    }
}

void CRobot::Draw( CRender& arRender ) const
{
    const float trailThickness = 2.0f;
    const float headingThickness = 3.0f;

    for( unsigned int i = 1; i < mTrail.size(); ++i )
    {
        arRender.DrawLine( mTrail[i - 1], mTrail[i], trailThickness, mTrailStyle );
    }

    arRender.DrawCircle( mPose.mPosition, mRadius, mRobotStyle );

    Vec2D headingEnd =
    {
        mPose.mPosition.x + float( mRadius ) * std::cos( mPose.mHeading ),
        mPose.mPosition.y + float( mRadius ) * std::sin( mPose.mHeading )
    };

    arRender.DrawLine( mPose.mPosition, headingEnd, headingThickness, CRender::SENSOR );
}

bool CRobot::HasCompletedLap() const
{
    return mCompletedLap;
}

int CRobot::GetUpdateCount() const
{
    return mUpdateCount;
}

int CRobot::GetCollisionCount() const
{
    return mCollisionCount;
}

const std::string& CRobot::GetName() const
{
    return mName;
}

void CRobot::ReportSummary() const
{
    std::cout << mName
              << ": updates = " << mUpdateCount
              << ", collisions = " << mCollisionCount
              << ", completed lap = " << ( mCompletedLap ? "yes" : "no" )
              << std::endl;
}

const CPose& CRobot::GetPose() const
{
    return mPose;
}

CWallMap& CRobot::GetWallMap()
{
    return mrWallMap;
}

int CRobot::GetRadius() const
{
    return mRadius;
}

void CRobot::SetWheelSpeeds( float aLeftSpeed, float aRightSpeed )
{
    if( aLeftSpeed > mMaximumWheelSpeed )
    {
        aLeftSpeed = mMaximumWheelSpeed;
    }
    else if( aLeftSpeed < -mMaximumWheelSpeed )
    {
        aLeftSpeed = -mMaximumWheelSpeed;
    }

    if( aRightSpeed > mMaximumWheelSpeed )
    {
        aRightSpeed = mMaximumWheelSpeed;
    }
    else if( aRightSpeed < -mMaximumWheelSpeed )
    {
        aRightSpeed = -mMaximumWheelSpeed;
    }

    mLeftWheelSpeed = aLeftSpeed;
    mRightWheelSpeed = aRightSpeed;
}

void CRobot::Move()
{
    float linearSpeed = ( mLeftWheelSpeed + mRightWheelSpeed ) / 2.0f;
    float angularSpeed = ( mLeftWheelSpeed - mRightWheelSpeed ) / mWheelSeparation;

    CPose nextPose = mPose;
    nextPose.mHeading += angularSpeed * mTimeStep;
    nextPose.mPosition.x += linearSpeed * std::cos( nextPose.mHeading ) * mTimeStep;
    nextPose.mPosition.y += linearSpeed * std::sin( nextPose.mHeading ) * mTimeStep;

    if( mrWallMap.CollidesWithWall( nextPose.mPosition, float( mRadius ) ) )
    {
        ++mCollisionCount;
        std::cout << "Collision: " << mName
                  << " at update " << mUpdateCount << std::endl;

        // Let it turn, but do not move through the wall.
        mPose.mHeading = nextPose.mHeading;
    }
    else
    {
        mPose = nextPose;
    }
}

void CRobot::CheckLapCompletion()
{
    const float leaveStartDistance = 80.0f;
    const float finishDistance = 25.0f;
    const int minimumUpdates = 300;

    float distance = DistanceFromStart();

    if( distance > leaveStartDistance )
    {
        mHasLeftStart = true;
    }

    if( mHasLeftStart && distance < finishDistance && mUpdateCount > minimumUpdates )
    {
        mCompletedLap = true;
    }
}

float CRobot::DistanceFromStart() const
{
    float differenceX = mPose.mPosition.x - mStartPose.mPosition.x;
    float differenceY = mPose.mPosition.y - mStartPose.mPosition.y;

    return std::sqrt( differenceX * differenceX + differenceY * differenceY );
}
