//-----------------------------------------------------------------------------
// CRobot.h
//
// Common code for the robot movement and lap checking.
// Child classes choose the wheel speeds.
//-----------------------------------------------------------------------------

#ifndef CROBOT_H
#define CROBOT_H

#include "CLoopReader.h"
#include "CRender.h"

#include <string>
#include <vector>

class CWallMap;

class CRobot
{
    public:
        CRobot( CWallMap& arWallMap,
                const std::string& arName,
                const CPose& arStartPose,
                CRender::EStyle aRobotStyle,
                CRender::EStyle aTrailStyle );

        virtual ~CRobot();

        void Update();
        void Draw( CRender& arRender ) const;

        bool HasCompletedLap() const;
        int GetUpdateCount() const;
        int GetCollisionCount() const;
        const std::string& GetName() const;
        void ReportSummary() const;

    protected:
        virtual void Control() = 0;

        const CPose& GetPose() const;
        CWallMap& GetWallMap();
        int GetRadius() const;

        void SetWheelSpeeds( float aLeftSpeed, float aRightSpeed );

    private:
        void Move();
        void CheckLapCompletion();
        float DistanceFromStart() const;

        CWallMap& mrWallMap;
        std::string mName;

        CPose mStartPose;
        CPose mPose;

        const int mRadius;
        const float mWheelSeparation;
        const float mTimeStep;
        const float mMaximumWheelSpeed;

        float mLeftWheelSpeed;
        float mRightWheelSpeed;

        std::vector<Vec2D> mTrail;
        CRender::EStyle mRobotStyle;
        CRender::EStyle mTrailStyle;

        int mUpdateCount;
        int mCollisionCount;
        bool mHasLeftStart;
        bool mCompletedLap;
};

#endif
