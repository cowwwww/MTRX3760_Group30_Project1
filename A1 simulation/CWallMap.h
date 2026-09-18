//-----------------------------------------------------------------------------
// CWallMap.h
//
// Stores the walls and the floor line.
//-----------------------------------------------------------------------------

#ifndef CWALLMAP_H
#define CWALLMAP_H

#include "CLoopReader.h"
#include "CRender.h"

#include <string>

class CWallMap
{
    public:
        CWallMap( const std::string& arWallFilename,
                const std::string& arLineFilename );

        bool IsReady() const;
        const CPose& GetWallStartPose() const;
        const CPose& GetLineStartPose() const;

        void Draw( CRender& arRender ) const;

        float MeasureWallRange( const Vec2D& arOrigin,
                                float aHeading,
                                float aMaximumRange ) const;

        bool CollidesWithWall( const Vec2D& arPosition, float aRadius ) const;
        bool IsOnLine( const Vec2D& arPosition ) const;

    private:
        void DrawLoop( CRender& arRender,
                       const CLoopReader& arLoop,
                       float aThickness,
                       CRender::EStyle aStyle ) const;

        bool RaySegmentIntersection( const Vec2D& arOrigin,
                                     const Vec2D& arDirection,
                                     const Vec2D& arA,
                                     const Vec2D& arB,
                                     float& arDistance ) const;

        float DistancePointToSegment( const Vec2D& arPoint,
                                      const Vec2D& arA,
                                      const Vec2D& arB ) const;

        CLoopReader mWalls;
        CLoopReader mLine;
        bool mReady;
};

#endif
