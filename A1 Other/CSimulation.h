//-----------------------------------------------------------------------------
// CSimulation.h
//
// Main simulation class for A2.
//-----------------------------------------------------------------------------

#ifndef CSIMULATION_H
#define CSIMULATION_H

#include "CLineFollower.h"
#include "CRender.h"
#include "CWallFollower.h"
#include "CWallMap.h"

#include <vector>

class CSimulation
{
    public:
        CSimulation();
        CSimulation( const CSimulation& ) = delete;
        CSimulation& operator=( const CSimulation& ) = delete;

        void Run();

    private:
        void UpdateRobots();
        void Draw();
        bool AllRobotsStopped() const;
        void PrintSummary();

        CWallMap mWallMap;
        CRender mRender;
        CWallFollower mWallFollower;
        CLineFollower mLineFollower;
        std::vector<CRobot*> mRobots;
        bool mSummaryPrinted;
};

#endif
