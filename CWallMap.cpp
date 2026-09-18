//-----------------------------------------------------------------------------
// CWallMap.cpp
//-----------------------------------------------------------------------------

#include "CWallMap.h"

#include <cmath>
#include <vector>

CWallMap::CWallMap( const std::string& arWallFilename,
                const std::string& arLineFilename )
    : mReady( false )
{
    bool wallsReady = mWalls.ReadFile( arWallFilename );
    bool lineReady = mLine.ReadFile( arLineFilename );
    mReady = wallsReady && lineReady;
}

bool CWallMap::IsReady() const
{
    return mReady;
}

const CPose& CWallMap::GetWallStartPose() const
{
    return mWalls.GetStartPose();
}

const CPose& CWallMap::GetLineStartPose() const
{
    return mLine.GetStartPose();
}

void CWallMap::Draw( CRender& arRender ) const
{
    const float wallThickness = 3.0f;
    const float lineThickness = 5.0f;

    DrawLoop( arRender, mWalls, wallThickness, CRender::WALL );
    DrawLoop( arRender, mLine, lineThickness, CRender::FLOOR_LINE );
}

float CWallMap::MeasureWallRange( const Vec2D& arOrigin,
                                float aHeading,
                                float aMaximumRange ) const
{
    Vec2D direction = { std::cos( aHeading ), std::sin( aHeading ) };
    float closestDistance = aMaximumRange;
    const std::vector<Vec2D>& vertices = mWalls.GetVertices();

    if( !vertices.empty() )
    {
        Vec2D previous = vertices.back();
        for( unsigned int i = 0; i < vertices.size(); ++i )
        {
            float distance = 0.0f;
            bool hit = RaySegmentIntersection( arOrigin, direction,
                                               previous, vertices[i], distance );
            if( hit && distance < closestDistance )
            {
                closestDistance = distance;
            }
            previous = vertices[i];
        }
    }

    return closestDistance;
}

bool CWallMap::CollidesWithWall( const Vec2D& arPosition, float aRadius ) const
{
    bool collision = false;
    const std::vector<Vec2D>& vertices = mWalls.GetVertices();

    if( !vertices.empty() )
    {
        Vec2D previous = vertices.back();
        for( unsigned int i = 0; i < vertices.size() && !collision; ++i )
        {
            if( DistancePointToSegment( arPosition, previous, vertices[i] ) < aRadius )
            {
                collision = true;
            }
            previous = vertices[i];
        }
    }

    return collision;
}

bool CWallMap::IsOnLine( const Vec2D& arPosition ) const
{
    const float lineWidth = 5.0f;
    const float halfLineWidth = lineWidth / 2.0f;
    bool onLine = false;
    const std::vector<Vec2D>& vertices = mLine.GetVertices();

    if( !vertices.empty() )
    {
        Vec2D previous = vertices.back();
        for( unsigned int i = 0; i < vertices.size() && !onLine; ++i )
        {
            if( DistancePointToSegment( arPosition, previous, vertices[i] ) <= halfLineWidth )
            {
                onLine = true;
            }
            previous = vertices[i];
        }
    }

    return onLine;
}

void CWallMap::DrawLoop( CRender& arRender,
                       const CLoopReader& arLoop,
                       float aThickness,
                       CRender::EStyle aStyle ) const
{
    const std::vector<Vec2D>& vertices = arLoop.GetVertices();

    if( !vertices.empty() )
    {
        Vec2D previous = vertices.back();
        for( unsigned int i = 0; i < vertices.size(); ++i )
        {
            arRender.DrawLine( previous, vertices[i], aThickness, aStyle );
            previous = vertices[i];
        }
    }
}

bool CWallMap::RaySegmentIntersection( const Vec2D& arOrigin,
                                     const Vec2D& arDirection,
                                     const Vec2D& arA,
                                     const Vec2D& arB,
                                     float& arDistance ) const
{
    bool hit = false;
    Vec2D segment = { arB.x - arA.x, arB.y - arA.y };
    float cross = arDirection.x * segment.y - arDirection.y * segment.x;

    if( std::fabs( cross ) > 0.00001f )
    {
        Vec2D offset = { arA.x - arOrigin.x, arA.y - arOrigin.y };
        float rayDistance = ( offset.x * segment.y - offset.y * segment.x ) / cross;
        float segmentPosition = ( offset.x * arDirection.y - offset.y * arDirection.x ) / cross;

        if( rayDistance >= 0.0f && segmentPosition >= 0.0f && segmentPosition <= 1.0f )
        {
            arDistance = rayDistance;
            hit = true;
        }
    }

    return hit;
}

float CWallMap::DistancePointToSegment( const Vec2D& arPoint,
                                      const Vec2D& arA,
                                      const Vec2D& arB ) const
{
    float segmentX = arB.x - arA.x;
    float segmentY = arB.y - arA.y;
    float segmentLengthSquared = segmentX * segmentX + segmentY * segmentY;
    float position = 0.0f;

    if( segmentLengthSquared > 0.0f )
    {
        position = ( ( arPoint.x - arA.x ) * segmentX
                   + ( arPoint.y - arA.y ) * segmentY ) / segmentLengthSquared;
    }

    if( position < 0.0f )
    {
        position = 0.0f;
    }
    else if( position > 1.0f )
    {
        position = 1.0f;
    }

    float closestX = arA.x + position * segmentX;
    float closestY = arA.y + position * segmentY;
    float differenceX = arPoint.x - closestX;
    float differenceY = arPoint.y - closestY;

    return std::sqrt( differenceX * differenceX + differenceY * differenceY );
}
