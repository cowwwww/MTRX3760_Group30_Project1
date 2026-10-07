//-----------------------------------------------------------------------------
// c_robot.cpp
//
// Implementation of CRobot and CRightWallFollowerRobot. See c_robot.h.
//-----------------------------------------------------------------------------

#include "tb3_maze/c_robot.h"

#include <cmath>

//---Degrees in the constants below are converted to radians for use here.-----
static const float kDegreesToRadians = float( M_PI ) / 180.0f;

// =============================================================================
// ---------------------------------- CRobot ----------------------------------
// =============================================================================

// The TurtleBot3 Burger: wheels 0.160 m apart, 0.22 m/s its top speed.
const float CRobot::mkWheelBase = 0.160f;
const float CRobot::mkMaximumWheelSpeed = 0.22f;

const int CRobot::mkWaypointCount = 3;
const float CRobot::mkReplaceDistance = 0.25f;
const float CRobot::mkReachedDistance = 0.12f;

const float CRobot::mkCruiseSpeed = 0.15f;
const float CRobot::mkTurnGain = 1.5f;
const float CRobot::mkMaximumTurnRate = 1.2f;
const float CRobot::mkTurnInPlaceAngle = 70.0f * kDegreesToRadians;
const float CRobot::mkSearchTurnRate = 0.5f;
const float CRobot::mkFrontHalfAngle = 20.0f;
const float CRobot::mkSlowdownDistance = 0.50f;
const float CRobot::mkStopDistance = 0.18f;

//-----------------------------------------------------------------------------
CRobot::CRobot()
    :
        mLidar( 0.0f ),   // the lidar's zero direction is the robot's heading
        mLeftWheelSpeed( 0.0f ),
        mRightWheelSpeed( 0.0f )
{
}

//-----------------------------------------------------------------------------
CRobot::~CRobot()
{
}

//-----------------------------------------------------------------------------
void CRobot::Update( const sensor_msgs::msg::LaserScan& arScan, const CPose& arPose )
{
    // Data in: take the new scan.
    mLidar.Sense( arScan );

    // Stop unless there is something to go on.
    SetWheelSpeeds( 0.0f, 0.0f );

    if( !mLidar.HasReading() )
    {
        return;
    }

    // Where to go: let the derived robot choose, and keep it as a waypoint.
    CPoint Target;

    if( ChooseWaypoint( Target ) )
    {
        UpdateWaypoints( ToOdomFrame( arPose, Target ), arPose );
    }

    // How to get there.
    DriveToWaypoint( arPose );
}

//-----------------------------------------------------------------------------
const std::deque<CPoint>& CRobot::GetWaypoints() const
{
    return mWaypoints;
}

//-----------------------------------------------------------------------------
float CRobot::GetLeftWheelSpeed() const
{
    return mLeftWheelSpeed;
}

//-----------------------------------------------------------------------------
float CRobot::GetRightWheelSpeed() const
{
    return mRightWheelSpeed;
}

//-----------------------------------------------------------------------------
float CRobot::GetLinearVelocity() const
{
    // Differential drive: the average of the two wheels moves the robot forward.
    return ( mLeftWheelSpeed + mRightWheelSpeed ) / 2.0f;
}

//-----------------------------------------------------------------------------
float CRobot::GetAngularVelocity() const
{
    // The difference between them turns it. The right wheel sits on the
    // robot's right, so a faster right wheel turns the robot to the left,
    // which is counter-clockwise, ROS's positive direction.
    return ( mRightWheelSpeed - mLeftWheelSpeed ) / mkWheelBase;
}

//-----------------------------------------------------------------------------
const CLidar& CRobot::GetLidar() const
{
    return mLidar;
}

//-----------------------------------------------------------------------------
void CRobot::UpdateWaypoints( const CPoint& arTarget, const CPose& arPose )
{
    bool Replace = mWaypoints.empty();

    if( !Replace )
    {
        const CPoint& Current = mWaypoints.front();

        float TargetMoved = std::hypot( arTarget.mX - Current.mX, arTarget.mY - Current.mY );
        float DistanceToCurrent = std::hypot( arPose.mPosition.mX - Current.mX, arPose.mPosition.mY - Current.mY );

        Replace = ( TargetMoved > mkReplaceDistance ) || ( DistanceToCurrent < mkReachedDistance );
    }

    if( Replace )
    {
        mWaypoints.push_front( arTarget );

        if( int( mWaypoints.size() ) > mkWaypointCount )
        {
            mWaypoints.pop_back();
        }
    }
}

//-----------------------------------------------------------------------------
void CRobot::DriveToWaypoint( const CPose& arPose )
{
    if( mWaypoints.empty() )
    {
        // Nowhere to go yet: turn on the spot, to the right, until the
        // algorithm finds somewhere.
        SetVelocity( 0.0f, -mkSearchTurnRate );
        return;
    }

    // Which way the waypoint is, as seen from the robot: 0 is dead ahead,
    // positive is to the left.
    CPoint Target = ToRobotFrame( arPose, mWaypoints.front() );
    float Bearing = std::atan2( Target.mY, Target.mX );

    // Turn toward it, in proportion to how far off it is.
    float TurnRate = mkTurnGain * Bearing;

    if( TurnRate > mkMaximumTurnRate )
    {
        TurnRate = mkMaximumTurnRate;
    }
    else if( TurnRate < -mkMaximumTurnRate )
    {
        TurnRate = -mkMaximumTurnRate;
    }

    // Drive forward as much as it is ahead: full speed when dead ahead, none
    // once it is off to one side by mkTurnInPlaceAngle, so a waypoint beside
    // or behind the robot is turned to face before the robot moves toward it.
    float Speed = 0.0f;

    if( std::fabs( Bearing ) < mkTurnInPlaceAngle )
    {
        Speed = mkCruiseSpeed * ( 1.0f - ( std::fabs( Bearing ) / mkTurnInPlaceAngle ) );
    }

    // Ease off for anything close ahead, down to a stop, leaving only the turn.
    float Ahead = mLidar.GetNearestDistance( 0.0f, mkFrontHalfAngle );
    float Clearance = ( Ahead - mkStopDistance ) / ( mkSlowdownDistance - mkStopDistance );

    if( Clearance < 0.0f )
    {
        Clearance = 0.0f;
    }
    else if( Clearance > 1.0f )
    {
        Clearance = 1.0f;
    }

    SetVelocity( Speed * Clearance, TurnRate );
}

//-----------------------------------------------------------------------------
CPoint CRobot::ToOdomFrame( const CPose& arPose, const CPoint& arPoint ) const
{
    float Cos = std::cos( arPose.mHeading );
    float Sin = std::sin( arPose.mHeading );

    CPoint Point;
    Point.mX = arPose.mPosition.mX + ( Cos * arPoint.mX ) - ( Sin * arPoint.mY );
    Point.mY = arPose.mPosition.mY + ( Sin * arPoint.mX ) + ( Cos * arPoint.mY );
    return Point;
}

//-----------------------------------------------------------------------------
CPoint CRobot::ToRobotFrame( const CPose& arPose, const CPoint& arPoint ) const
{
    float Cos = std::cos( arPose.mHeading );
    float Sin = std::sin( arPose.mHeading );
    float DeltaX = arPoint.mX - arPose.mPosition.mX;
    float DeltaY = arPoint.mY - arPose.mPosition.mY;

    CPoint Point;
    Point.mX = ( Cos * DeltaX ) + ( Sin * DeltaY );
    Point.mY = ( -Sin * DeltaX ) + ( Cos * DeltaY );
    return Point;
}

//-----------------------------------------------------------------------------
void CRobot::SetWheelSpeeds( float aLeftSpeed, float aRightSpeed )
{
    if( aLeftSpeed > mkMaximumWheelSpeed )
    {
        aLeftSpeed = mkMaximumWheelSpeed;
    }
    else if( aLeftSpeed < -mkMaximumWheelSpeed )
    {
        aLeftSpeed = -mkMaximumWheelSpeed;
    }

    if( aRightSpeed > mkMaximumWheelSpeed )
    {
        aRightSpeed = mkMaximumWheelSpeed;
    }
    else if( aRightSpeed < -mkMaximumWheelSpeed )
    {
        aRightSpeed = -mkMaximumWheelSpeed;
    }

    mLeftWheelSpeed = aLeftSpeed;
    mRightWheelSpeed = aRightSpeed;
}

//-----------------------------------------------------------------------------
void CRobot::SetVelocity( float aLinear, float aAngular )
{
    // Each wheel's rim moves at the forward speed, plus or minus the speed the
    // turn adds at half the wheel base from the centre.
    float HalfTurn = aAngular * mkWheelBase / 2.0f;

    SetWheelSpeeds( aLinear - HalfTurn, aLinear + HalfTurn );
}

// =============================================================================
// -------------------------  CRightWallFollowerRobot  --------------------------
// =============================================================================

const float CRightWallFollowerRobot::mkFieldOfView = 70.0f;
const float CRightWallFollowerRobot::mkClearDistance = 0.90f;
const float CRightWallFollowerRobot::mkMinGapWidth = 20.0f;

const float CRightWallFollowerRobot::mkEdgeClearance = 0.25f;
const float CRightWallFollowerRobot::mkOpenEdgeMargin = 15.0f;
const float CRightWallFollowerRobot::mkWaypointDistance = 0.50f;

//-----------------------------------------------------------------------------
CRightWallFollowerRobot::CRightWallFollowerRobot()
    :
        CRobot()
{
}

//-----------------------------------------------------------------------------
bool CRightWallFollowerRobot::ChooseWaypoint( CPoint& arTarget )
{
    const CLidar& Lidar = GetLidar();

    // The ways forward: the gaps within mkFieldOfView of straight ahead.
    std::vector<CGap> Gaps = Lidar.FindGaps( -mkFieldOfView, mkFieldOfView, mkClearDistance, mkMinGapWidth );

    // The gaps come back in order of increasing angle. Ahead, that runs from the
    // right to the left, so the first is the rightmost.
    bool Ahead = !Gaps.empty();

    if( !Ahead )
    {
        // Nothing open ahead, dead end. Look at the rest of the circle, behind it.
        // That sector runs from the left, round the back, to the right, so the last gap is
        // the rightmost.
        Gaps = Lidar.FindGaps( mkFieldOfView, 360.0f - mkFieldOfView, mkClearDistance, mkMinGapWidth );
    }

    if( Gaps.empty() )
    {
        return false;
    }

    // The right-hand rule: of the ways to go, take the rightmost.
    const CGap& Rightmost = Ahead ? Gaps.front() : Gaps.back();

    // Aim far enough inside its right edge to pass whatever makes that edge by
    // mkEdgeClearance. Increasing the angle moves left into the gap.
    float Margin = mkOpenEdgeMargin * kDegreesToRadians;

    if( Rightmost.mRightObstacle > 0.0f )
    {
        float Ratio = mkEdgeClearance / Rightmost.mRightObstacle;
        Margin = std::asin( ( Ratio < 1.0f ) ? Ratio : 1.0f );
    }

    // Never aim past the middle of the gap, so a narrow gap is aimed down, not out of.
    float Aim = Rightmost.mRightAngle + Margin;
    float Middle = ( Rightmost.mLeftAngle + Rightmost.mRightAngle ) / 2.0f;

    if( Aim > Middle )
    {
        Aim = Middle;
    }

    arTarget.mX = mkWaypointDistance * std::cos( Aim );
    arTarget.mY = mkWaypointDistance * std::sin( Aim );
    return true;
}
