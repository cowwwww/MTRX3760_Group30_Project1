//-----------------------------------------------------------------------------
// c_sensor.h
//
// Every sensor a robot can carry, in one file: CSensor, the common base,
// plus the concrete sensor derived from it. They are kept together here for
// the same reason as c_robot.h: CLidar only ever exists as "a CSensor that
// senses this particular way", and the header comments below (one per class,
// each marked by its own banner further down) cover each in turn.
//
//
// CSensor is the common base for anything mounted on the robot at a fixed
// angle relative to its heading. Every sensor needs the same thing from that
// mounting - to turn an angle measured in the sensor's own frame into an
// angle measured from the robot's heading, and back - so that geometry lives
// here. Whether the sensor has produced anything usable yet is the one thing
// that differs between sensor types (a lidar needs a scan with rays in it, a
// camera would need an image), so HasReading() is the only virtual.
//
// Angles in this file are in degrees at every public function, counted
// COUNTER-CLOCKWISE from the robot's heading, as ROS counts them: 0 is
// straight ahead, +90 is to the robot's left, -90 is to its right. (Lab 2
// counted clockwise; the sign is the only difference.) Angles inside a CGap
// are in radians, in the same direction.
//
//
// CLidar is the TurtleBot3's 360 degree scanning laser. The ROS node hands it
// each LaserScan, it keeps the latest one, and it answers the questions a
// controller asks of it: how far is the nearest thing in this direction, and
// where are the gaps - the stretches of directions that are open for some
// distance. It asks nothing about what to do with the answers - that is the
// robot's business (see c_robot.h).
//
// A ray counts as open when it is a real distance at least as long as the
// clear distance asked for, or "no return" (+infinity in Gazebo and the LDS:
// nothing within range). An invalid reading (NaN, below the minimum range, or
// the 0.0 the real LDS reports for a dropout) is never open, so a bad ray can
// only ever make the robot more cautious. The scan is assumed to cover the
// full circle, as the TurtleBot3's does.
//-----------------------------------------------------------------------------

#ifndef C_SENSOR_H
#define C_SENSOR_H

#include <sensor_msgs/msg/laser_scan.hpp>

#include <vector>

//=============================================================================
//===  CSensor  ================================================================
//=============================================================================
class CSensor
{
    public:
        //---Ctor/Dtor---
        // aMountAngleDegrees is the angle of the sensor's own zero direction,
        // measured from the robot's heading, e.g. 0 is aligned with it.
        explicit CSensor( float aMountAngleDegrees );

        // Virtual: CLidar is destroyed through this base whenever a CSensor
        // holding one goes out of scope.
        virtual ~CSensor();

        //---Sensing---
        // True once the sensor has received something it can answer from.
        virtual bool HasReading() const = 0;

    protected:
        // An angle measured in the sensor's own frame, as an angle from the
        // robot's heading, in radians. And the reverse.
        float GetRobotAngle( float aSensorAngle ) const;
        float GetSensorAngle( float aRobotAngle ) const;

        //---Geometry---
        float mMountAngle;   // radians, offset from the robot's heading
};

//=============================================================================
//===  CGap  ===================================================================
//=============================================================================
// A stretch of open directions: the angles of its two edges, and how far away
// whatever ends it on each side is. That distance is what lets an algorithm
// keep a fixed number of metres from the edge instead of a fixed angle, which
// shrinks to nothing as the edge gets close.
struct CGap
{
    float mRightAngle;      // radians, the clockwise (right-hand) edge
    float mLeftAngle;       // radians, the counter-clockwise (left-hand) edge, the larger angle
    float mRightObstacle;   // metres to what ends the gap on the right; 0 if it runs to the end of the sector
    float mLeftObstacle;    // metres to what ends the gap on the left; 0 if it runs to the end of the sector
};

//=============================================================================
//===  CLidar  =================================================================
//=============================================================================
class CLidar : public CSensor
{
    public:
        //---Ctor---
        explicit CLidar( float aMountAngleDegrees );

        //---CSensor's virtual---
        bool HasReading() const override;

        //---Sensing---
        // Keeps a copy of arScan as the latest reading.
        void Sense( const sensor_msgs::msg::LaserScan& arScan );

        //---Access---
        // Distance in metres along the ray nearest to aAngleDegrees. The
        // maximum range if that ray has no return or an invalid reading.
        float GetDistance( float aAngleDegrees ) const;

        // Distance in metres to the nearest return within aHalfWidthDegrees
        // either side of aCentreDegrees. The maximum range if there is none.
        float GetNearestDistance( float aCentreDegrees, float aHalfWidthDegrees ) const;

        // The gaps between aFromDegrees and aToDegrees: each run of rays that
        // are open for at least aClearDistance metres and that together span at
        // least aMinWidthDegrees. The result is in order from the rightmost
        // gap to the leftmost, so the last one is the leftmost. The two limits
        // may be outside -180..180, so a sector can run through straight
        // behind (e.g. 100 to 260), and the gaps' angles then stay in that range.
        // One or two blocked rays in a row do not split a gap. Each gap says how
        // far away what ends it is (see CGap); an invalid reading there counts as very close.
        std::vector<CGap> FindGaps( float aFromDegrees, float aToDegrees, float aClearDistance, float aMinWidthDegrees ) const;

    private:
        //---Rays---
        // Angle of ray aIndex from the robot's heading, in radians.
        float GetRayAngle( int aIndex ) const;

        // The raw reading of the ray nearest to aRobotAngle (radians from the heading).
        float GetRange( float aRobotAngle ) const;

        // Whether ray aIndex lies within aHalfWidth either side of aCentre (radians).
        bool IsInSector( int aIndex, float aCentre, float aHalfWidth ) const;

        // Whether aRange is a real measurement, as opposed to no return or an invalid reading.
        bool IsReturn( float aRange ) const;

        // Whether aRange is open for at least aDistance metres.
        bool IsClear( float aRange, float aDistance ) const;

        // Distance in metres to what the ray at aRobotAngle (radians) hits, as the
        // end of a gap; 0 if the ray is outside aFrom..aTo (radians).
        float GetObstacleDistance( float aRobotAngle, float aFrom, float aTo ) const;

        // Adds the gap between aRight and aLeft (radians) to arGaps, if it spans
        // at least aMinWidth. aFrom and aTo are the ends of the sector being searched.
        void AddGap( std::vector<CGap>& arGaps, float aRight, float aLeft, float aFrom, float aTo, float aMinWidth ) const;

        //---Consts---
        static const int mkBridgeRays;   // blocked rays in a row that still do not split a gap

        //---Latest scan---
        float mAngleMin;         // radians, angle of ray 0 in the sensor's frame
        float mAngleIncrement;   // radians between rays
        float mRangeMin;         // metres
        float mRangeMax;         // metres
        std::vector<float> mRanges;
};

#endif
