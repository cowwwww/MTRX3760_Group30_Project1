// Right-hand maze strategy: choose the rightmost gap and aim inside its edge.
#ifndef C_RIGHT_WALL_FOLLOWER_ROBOT_H
#define C_RIGHT_WALL_FOLLOWER_ROBOT_H

#include "tb3_maze/c_robot.h"

class CRightWallFollowerRobot : public CRobot
{
    public:
        //---Ctor---
        CRightWallFollowerRobot();

    protected:
        //---CRobot's customisation point---
        bool ChooseWaypoint( CPoint& arTarget ) override;

    private:
        //---What counts as a way to go---
        static const float mkFieldOfView;         // degrees either side of straight ahead that are looked at first; under 90 so the way it came is never chosen
        static const float mkClearDistance;       // metres a direction must be open for to count
        static const float mkMinGapWidth;         // degrees a gap must span to be wide enough for the robot

        //---Where in the rightmost gap to aim---
        static const float mkEdgeClearance;       // metres to keep from whatever makes the gap's right edge
        static const float mkOpenEdgeMargin;      // degrees inside the right edge if nothing makes it (it is the end of the sector)
        static const float mkWaypointDistance;    // metres ahead, along that direction, that the waypoint is put
};

#endif
