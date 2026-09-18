//-----------------------------------------------------------------------------
// CSimulation.cpp
//-----------------------------------------------------------------------------

#include "CSimulation.h"

#include <iostream>

CSimulation::CSimulation()
    : mWallMap( "SimpleWalls.map", "SimpleLine.map" ),
      mRender(),
      mWallFollower( mWallMap ),
      mLineFollower( mWallMap ),
      mSummaryPrinted( false )
{
    // Store both robots in one list.
    mRobots.push_back( &mWallFollower );
    mRobots.push_back( &mLineFollower );
}

void CSimulation::Run()
{
    if( !mWallMap.IsReady() )
    {
        std::cout << "Simulation could not load the maps." << std::endl;
    }
    else
    {
        while( !mRender.WindowShouldClose() )
        {
            if( !AllRobotsStopped() )
            {
                UpdateRobots();
            }
            else if( !mSummaryPrinted )
            {
                PrintSummary();
            }

            Draw();
        }
    }

    mRender.CloseWindow();
}

void CSimulation::UpdateRobots()
{
    const int updatesPerFrame = 4;
    const int maximumUpdates = 2000;

    for( int step = 0; step < updatesPerFrame; ++step )
    {
        for( unsigned int i = 0; i < mRobots.size(); ++i )
        {
            if( !mRobots[i]->HasCompletedLap()
                && mRobots[i]->GetUpdateCount() < maximumUpdates )
            {
                mRobots[i]->Update();
            }
        }
    }
}

void CSimulation::Draw()
{
    mRender.BeginDrawing();
    mWallMap.Draw( mRender );

    for( unsigned int i = 0; i < mRobots.size(); ++i )
    {
        mRobots[i]->Draw( mRender );
    }

    mRender.EndDrawing();
}

bool CSimulation::AllRobotsStopped() const
{
    const int maximumUpdates = 2000;
    bool allStopped = true;

    for( unsigned int i = 0; i < mRobots.size() && allStopped; ++i )
    {
        bool stopped = mRobots[i]->HasCompletedLap()
                    || mRobots[i]->GetUpdateCount() >= maximumUpdates;
        if( !stopped )
        {
            allStopped = false;
        }
    }

    return allStopped;
}

void CSimulation::PrintSummary()
{
    std::cout << std::endl << "A2 simulation summary" << std::endl;
    for( unsigned int i = 0; i < mRobots.size(); ++i )
    {
        mRobots[i]->ReportSummary();
    }
    std::cout << "Close the window after taking the final screenshot." << std::endl;

    mSummaryPrinted = true;
}
