//-----------------------------------------------------------------------------
// CLineFollower.h
//
// Robot for following the floor line.
//-----------------------------------------------------------------------------

#ifndef CLINEFOLLOWER_H
#define CLINEFOLLOWER_H

#include "CLineSensor.h"
#include "CRobot.h"

class CWallMap;

class CLineFollower : public CRobot
{
    public:
        CLineFollower( CWallMap& arWallMap );

    protected:
        void Control() override;

    private:
        CLineSensor mLineSensor;
        CLineSensor mSideSensor;
};

#endif
