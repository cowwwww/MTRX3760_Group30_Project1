#include "tb3_maze/c_right_wall_follower_robot.h"

#include <cmath>

static const float kDegreesToRadians = float( M_PI ) / 180.0f;

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
