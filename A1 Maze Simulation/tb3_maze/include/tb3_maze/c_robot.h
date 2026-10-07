// Common robot motion, waypoint history and frame conversion.
// Maze strategies derive from CRobot and implement ChooseWaypoint().

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

#endif
