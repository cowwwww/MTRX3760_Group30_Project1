//-----------------------------------------------------------------------------
// c_sensor.cpp
//
// Implementation of CSensor and CLidar. See c_sensor.h.
//-----------------------------------------------------------------------------

#include "tb3_maze/c_sensor.h"

#include <algorithm>
#include <cmath>

// =============================================================================
// ---------------------------------- CSensor ----------------------------------
// =============================================================================

//---Degrees in the mounting angle are converted to radians for use here.-----
static const float kDegreesToRadians = float( M_PI ) / 180.0f;

//-----------------------------------------------------------------------------
CSensor::CSensor( float aMountAngleDegrees )
    :
        mMountAngle( aMountAngleDegrees * kDegreesToRadians )
{
}

//-----------------------------------------------------------------------------
CSensor::~CSensor()
{
}

//-----------------------------------------------------------------------------
float CSensor::GetRobotAngle( float aSensorAngle ) const
{
    return aSensorAngle + mMountAngle;
}

//-----------------------------------------------------------------------------
float CSensor::GetSensorAngle( float aRobotAngle ) const
{
    return aRobotAngle - mMountAngle;
}

// =============================================================================
// ----------------------------------  CLidar  ---------------------------------
// =============================================================================

const int CLidar::mkBridgeRays = 2;
const float CLidar::mkMinimumCoverageFraction = 0.70f;
const float CLidar::mkMinimumUsableFraction = 0.60f;

//-----------------------------------------------------------------------------
CLidar::CLidar( float aMountAngleDegrees )
    :
        CSensor( aMountAngleDegrees ),
        mAngleMin( 0.0f ),
        mAngleIncrement( 0.0f ),
        mRangeMin( 0.0f ),
        mRangeMax( 0.0f )
{
}

//-----------------------------------------------------------------------------
bool CLidar::HasReading() const
{
    if( mRanges.empty() || !std::isfinite( mAngleMin ) ||
        !std::isfinite( mAngleIncrement ) || mAngleIncrement <= 0.0f ||
        !std::isfinite( mRangeMin ) || !std::isfinite( mRangeMax ) ||
        mRangeMin < 0.0f || mRangeMax <= mRangeMin || !std::isfinite( mMountAngle ) )
    {
        return false;
    }

    return std::any_of( mRanges.begin(), mRanges.end(),
        [this]( float aRange ) { return IsUsable( aRange ); } );
}

//-----------------------------------------------------------------------------
void CLidar::Sense( const sensor_msgs::msg::LaserScan& arScan )
{
    mAngleMin = arScan.angle_min;
    mAngleIncrement = arScan.angle_increment;
    mRangeMin = arScan.range_min;
    mRangeMax = arScan.range_max;
    mRanges = arScan.ranges;
}

//-----------------------------------------------------------------------------
bool CLidar::HasUsableSector( float aCentreDegrees, float aHalfWidthDegrees ) const
{
    if( !HasReading() || !std::isfinite( aCentreDegrees ) ||
        !std::isfinite( aHalfWidthDegrees ) || aHalfWidthDegrees <= 0.0f ||
        aHalfWidthDegrees > 180.0f )
    {
        return false;
    }

    const float Centre = aCentreDegrees * kDegreesToRadians;
    const float HalfWidth = aHalfWidthDegrees * kDegreesToRadians;
    std::size_t Total = 0;
    std::size_t Usable = 0;
    float NearestAngle = float( M_PI );

    for( std::size_t Index = 0; Index < mRanges.size(); ++Index )
    {
        const float Angle = GetRobotAngle( mAngleMin + float( Index ) * mAngleIncrement );
        if( !std::isfinite( Angle ) )
        {
            return false;
        }
        const float Difference = std::fabs( std::remainder( Angle - Centre, 2.0f * float( M_PI ) ) );
        if( Difference <= HalfWidth + 1e-6f )
        {
            ++Total;
            NearestAngle = std::min( NearestAngle, Difference );
            if( IsUsable( mRanges[Index] ) )
            {
                ++Usable;
            }
        }
    }

    // Coverage and return validity are separate: a full sector of NaNs is
    // unknown, while the same sector of +infinity is observed open space.
    const double ExpectedRays = 2.0 * HalfWidth / mAngleIncrement;
    return Total > 0 && Total >= mkMinimumCoverageFraction * ExpectedRays &&
        Usable >= mkMinimumUsableFraction * Total && NearestAngle <= HalfWidth / 2.0f;
}

//-----------------------------------------------------------------------------
float CLidar::GetDistance( float aAngleDegrees ) const
{
    float Distance = mRangeMax;

    if( HasReading() )
    {
        float Range = GetRange( aAngleDegrees * kDegreesToRadians );

        if( IsReturn( Range ) )
        {
            Distance = Range;
        }
    }

    return Distance;
}

//-----------------------------------------------------------------------------
float CLidar::GetNearestDistance( float aCentreDegrees, float aHalfWidthDegrees ) const
{
    if( !HasUsableSector( aCentreDegrees, aHalfWidthDegrees ) )
    {
        return 0.0f;
    }

    float Centre = aCentreDegrees * kDegreesToRadians;
    float HalfWidth = aHalfWidthDegrees * kDegreesToRadians;

    float Nearest = mRangeMax;

    for( int Index = 0; Index < int( mRanges.size() ); ++Index )
    {
        if( IsInSector( Index, Centre, HalfWidth ) && IsReturn( mRanges[Index] ) && ( mRanges[Index] < Nearest ) )
        {
            Nearest = mRanges[Index];
        }
    }

    return Nearest;
}

//-----------------------------------------------------------------------------
std::vector<CGap> CLidar::FindGaps( float aFromDegrees, float aToDegrees, float aClearDistance, float aMinWidthDegrees ) const
{
    std::vector<CGap> Gaps;

    if( !HasReading() )
    {
        return Gaps;
    }

    float From = aFromDegrees * kDegreesToRadians;
    float To = aToDegrees * kDegreesToRadians;
    float MinWidth = aMinWidthDegrees * kDegreesToRadians;
    int RayCount = int( ( To - From ) / mAngleIncrement ) + 1;

    // Walk across the sector one ray at a time, from right to left. A run of
    // open rays is a gap; it ends once more than mkBridgeRays blocked rays in a
    // row have followed it, or at the end of the sector.
    bool InGap = false;
    float GapRight = 0.0f;
    float GapLeft = 0.0f;
    int BlockedInARow = 0;

    for( int Ray = 0; Ray < RayCount; ++Ray )
    {
        float Angle = From + ( float( Ray ) * mAngleIncrement );

        if( IsClear( GetRange( Angle ), aClearDistance ) )
        {
            if( !InGap )
            {
                InGap = true;
                GapRight = Angle;
            }

            GapLeft = Angle;
            BlockedInARow = 0;
        }
        else if( InGap )
        {
            ++BlockedInARow;

            if( BlockedInARow > mkBridgeRays )
            {
                AddGap( Gaps, GapRight, GapLeft, From, To, MinWidth );
                InGap = false;
            }
        }
    }

    if( InGap )
    {
        AddGap( Gaps, GapRight, GapLeft, From, To, MinWidth );
    }

    return Gaps;
}

//-----------------------------------------------------------------------------
float CLidar::GetObstacleDistance( float aRobotAngle, float aFrom, float aTo ) const
{
    float Distance = 0.0f;

    if( ( aRobotAngle >= aFrom ) && ( aRobotAngle <= aTo ) )
    {
        float Range = GetRange( aRobotAngle );

        // An invalid reading is read as very close, the cautious way round.
        Distance = IsReturn( Range ) ? Range : mRangeMin;
    }

    return Distance;
}

//-----------------------------------------------------------------------------
void CLidar::AddGap( std::vector<CGap>& arGaps, float aRight, float aLeft, float aFrom, float aTo, float aMinWidth ) const
{
    // A gap's edges are the centres of its first and last rays, so it is one
    // ray's width wider than the angles between them.
    if( ( aLeft - aRight + mAngleIncrement ) >= aMinWidth )
    {
        CGap Gap;
        Gap.mRightAngle = aRight;
        Gap.mLeftAngle = aLeft;
        Gap.mRightObstacle = GetObstacleDistance( aRight - mAngleIncrement, aFrom, aTo );
        Gap.mLeftObstacle = GetObstacleDistance( aLeft + mAngleIncrement, aFrom, aTo );
        arGaps.push_back( Gap );
    }
}

//-----------------------------------------------------------------------------
float CLidar::GetRayAngle( int aIndex ) const
{
    return GetRobotAngle( mAngleMin + ( float( aIndex ) * mAngleIncrement ) );
}

//-----------------------------------------------------------------------------
float CLidar::GetRange( float aRobotAngle ) const
{
    // The ray nearest to the angle, wrapping round the scan so any angle,
    // including one past +-180 degrees, finds its ray.
    long Count = long( mRanges.size() );
    long Index = std::lround( ( GetSensorAngle( aRobotAngle ) - mAngleMin ) / mAngleIncrement );
    Index = ( ( Index % Count ) + Count ) % Count;

    return mRanges[Index];
}

//-----------------------------------------------------------------------------
bool CLidar::IsInSector( int aIndex, float aCentre, float aHalfWidth ) const
{
    // The difference is wrapped so a sector can straddle the scan's own
    // start/end angle (e.g. the one straight ahead when the scan starts at 0).
    float Difference = std::remainder( GetRayAngle( aIndex ) - aCentre, 2.0f * float( M_PI ) );
    return std::fabs( Difference ) <= aHalfWidth;
}

//-----------------------------------------------------------------------------
bool CLidar::IsReturn( float aRange ) const
{
    return std::isfinite( aRange ) && aRange > 0.0f &&
        aRange >= mRangeMin && aRange <= mRangeMax;
}

//-----------------------------------------------------------------------------
bool CLidar::IsUsable( float aRange ) const
{
    return ( std::isinf( aRange ) && aRange > 0.0f ) || IsReturn( aRange );
}

//-----------------------------------------------------------------------------
bool CLidar::IsClear( float aRange, float aDistance ) const
{
    // No return means nothing within range, which is open. A real reading is
    // open if it is far enough. Anything else, NaN included, is not.
    bool NoReturn = std::isinf( aRange ) && ( aRange > 0.0f );

    return NoReturn || ( IsReturn( aRange ) && ( aRange >= aDistance ) );
}
