//-----------------------------------------------------------------------------
// CWallFollower.h
//
// Robot for following the right wall.
//-----------------------------------------------------------------------------

#ifndef CWALLFOLLOWER_H
#define CWALLFOLLOWER_H

#include "CRangeSensor.h"
#include "CRobot.h"

class CWallMap;

class CWallFollower : public CRobot
{
    public:
        CWallFollower( CWallMap& arWallMap );

    protected:
        void Control() override;

    private:
        CRangeSensor mRightSensor;
        CRangeSensor mFrontRightSensor;
};

#endif
