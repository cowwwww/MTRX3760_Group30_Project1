//-----------------------------------------------------------------------------
// c_robot.h
//
// Every maze-solving algorithm for the TurtleBot3, in one file: CRobot, the
// common base, plus the concrete robot derived from it. They are kept together
// here because they are really one system - CRightWallFollowerRobot only ever
// exists as "a CRobot that chooses its waypoints this particular way" - and
// the header comments below (one per class, each marked by its own banner
// further down) cover each of them in turn.
//
//
// CRobot is the robot as the control program sees it: a lidar and a pose that
// data comes in through, and two independently driven wheels that commands go
// out through. It splits the job in two:
//
//         lidar + pose in  ->  WHERE to go  ->  HOW to get there  ->  wheel commands out
//
// WHERE to go is a waypoint, a point on the floor. Choosing it is the only
// step that differs between maze-solving algorithms, so it is the only
// virtual: ChooseWaypoint(). HOW to get there - turning toward the waypoint,
// driving to it, slowing for anything in the way, and converting that to wheel
// speeds - is the same for every algorithm and lives here. Adding another
// algorithm (a different maze strategy, say) only means implementing
// ChooseWaypoint() again, never touching CRobot itself.
//
// Waypoints are kept in the odom frame, so they stay where they were put as
// the robot moves. CRobot remembers the current one and the two before it
// (GetWaypoints()), which the ROS node draws in RViz. The algorithm is asked
// for a waypoint on every scan, but the current one is only replaced when the
// new one is somewhere genuinely different, or the robot has arrived. Without
// that the robot would chase a target that creeps forward with it, and the
// history would fill with near-identical points.
//
// The robot has no kinematics to advance: Gazebo (or the real robot) moves it.
// The wheel speeds are therefore the program's output, not part of a
// simulation, and CRobot limits them to what the hardware can do.
//
// Wheel speeds are the speed in metres per second each wheel's rim travels
// along the ground. A positive turning rate is counter-clockwise (a left
// turn), as in ROS, so the right wheel runs faster than the left to turn left.
// (Lab 2's heading ran clockwise and its wheel speeds were distances per
// update; the geometry is otherwise the same.)
//
// The lidar is treated as sitting at the robot's origin; on the Burger it is
// really 3 cm behind it, which does not matter at this scale.
//
//
// CRightWallFollowerRobot solves the maze with the right-hand rule: at every
// choice of where to go, take the rightmost. It finds gaps in the lidar scan
// (see CLidar::FindGaps) and aims just inside the right-hand edge of the
// rightmost gap, maintaining mkEdgeClearance from the obstacle at that edge.
//
// Along a right wall it corrects towards or away from that wall. At a right
// opening it curves into the opening; when the corridor turns left it takes
// the available left passage. With no forward opening it searches the rear
// sector, preferring its rightmost gap. Left openings do not win while there
// is an available opening further right.
//-----------------------------------------------------------------------------

#ifndef C_ROBOT_H
#define C_ROBOT_H

#include "tb3_maze/c_sensor.h"   // for CLidar

#include <sensor_msgs/msg/laser_scan.hpp>

#include <deque>

//=============================================================================
//===  CPoint, CPose  ==========================================================
//=============================================================================
// A point on the floor, in metres.
struct CPoint
{
    float mX;
    float mY;
};

// Where the robot is in the odom frame.
struct CPose
{
    CPoint mPosition;
    float mHeading;   // radians, counter-clockwise from the odom frame's x axis
};

//=============================================================================
//===  CRobot  ================================================================
//=============================================================================
class CRobot
{
    public:
        //---Ctor/Dtor---
        CRobot();

        // Virtual: CRightWallFollowerRobot is destroyed through this base
        // whenever a CRobot holding one goes out of scope.
        virtual ~CRobot();

        //---Simulation---
        // One pass of the whole sequence: the lidar takes arScan, the algorithm
        // chooses where to go, and the wheel speeds are set to drive there.
        // arPose is where the robot was when the scan was taken. The robot
        // stops if the lidar has nothing usable to go on.
        void Update( const sensor_msgs::msg::LaserScan& arScan, const CPose& arPose );

        //---Waypoints---
        // The current waypoint and the ones before it, newest first. At most
        // mkWaypointCount of them, in the odom frame.
        const std::deque<CPoint>& GetWaypoints() const;

        //---Wheels---
        // The speed each wheel is to run at, in m/s, until the next Update().
        float GetLeftWheelSpeed() const;
        float GetRightWheelSpeed() const;

        // The same two speeds as ROS takes them: forward speed in m/s, and
        // turning rate in rad/s, counter-clockwise positive.
        float GetLinearVelocity() const;
        float GetAngularVelocity() const;

    protected:
        //---The step that differs between algorithms---
        // Where to go next, from the lidar reading Update() just took (see
        // GetLidar()). Put it in arTarget as a point in the ROBOT's frame -
        // x metres ahead, y metres to the left - and return true. Return
        // false if there is nowhere to go. Called on every scan; CRobot decides
        // whether the answer is different enough to replace the current waypoint.
        virtual bool ChooseWaypoint( CPoint& arTarget ) = 0;

        //---Access for derived classes---
        const CLidar& GetLidar() const;

    private:
        //---Waypoints---
        // Makes arTarget (odom frame) the current waypoint if there is none, if
        // the robot has arrived at the current one, or if arTarget is somewhere
        // different enough from it.
        void UpdateWaypoints( const CPoint& arTarget, const CPose& arPose );

        // Sets the wheel speeds to drive toward the current waypoint.
        void DriveToWaypoint( const CPose& arPose );

        //---Frames---
        // A point in the robot's frame as a point in the odom frame, and back.
        CPoint ToOdomFrame( const CPose& arPose, const CPoint& arPoint ) const;
        CPoint ToRobotFrame( const CPose& arPose, const CPoint& arPoint ) const;

        //---Wheels---
        // Sets the speed of each wheel directly, limited to what the robot can do.
        void SetWheelSpeeds( float aLeftSpeed, float aRightSpeed );

        // Sets the wheel speeds that give this forward speed (m/s) and turning
        // rate (rad/s, counter-clockwise positive).
        void SetVelocity( float aLinear, float aAngular );

        //---Consts: the robot---
        static const float mkWheelBase;           // distance between the two wheels, metres
        static const float mkMaximumWheelSpeed;   // fastest a wheel may be run, m/s

        //---Consts: waypoints---
        static const int mkWaypointCount;         // how many waypoints are remembered
        static const float mkReplaceDistance;     // metres a new waypoint must differ by to replace the current one
        static const float mkReachedDistance;     // metres from the current waypoint that counts as arriving

        //---Consts: driving to a waypoint---
        static const float mkCruiseSpeed;         // m/s, straight toward the waypoint with a clear way ahead
        static const float mkTurnGain;            // rad/s of turn per radian the waypoint is off to one side
        static const float mkMaximumTurnRate;     // rad/s
        static const float mkTurnInPlaceAngle;    // radians; farther off to one side than this, it does not drive forward
        static const float mkSearchTurnRate;      // rad/s, to the right, while there is no waypoint at all
        static const float mkFrontHalfAngle;      // degrees either side of straight ahead that is watched for obstacles
        static const float mkSlowdownDistance;    // metres; something nearer than this ahead eases the speed off
        static const float mkStopDistance;        // metres; something nearer than this ahead stops forward motion

        //---The lidar---
        CLidar mLidar;

        //---The waypoints: [0] is the current one---
        std::deque<CPoint> mWaypoints;

        //---The wheels---
        float mLeftWheelSpeed;    // m/s
        float mRightWheelSpeed;   // m/s
};

//=============================================================================
//===  CRightWallFollowerRobot  ================================================
//=============================================================================
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
