//----------------------------------------------------------------------------
// File:    ControlTypes.h
// Project: MTRX3760 Project 1 - Hello, Turtlebots! (Group 30)
// Purpose: ROS-independent value types: the laser scan passed into the
//          controller and the velocity command it returns.
//----------------------------------------------------------------------------
#ifndef PROJECT1_CONTROLTYPES_H
#define PROJECT1_CONTROLTYPES_H

#include <vector>

namespace project1
{
struct LaserScan
{
    double angleMin = 0.0;
    double angleIncrement = 0.0;
    double rangeMin = 0.0;
    double rangeMax = 0.0;
    std::vector<float> ranges;
};

struct Velocity
{
    double linear = 0.0;   // metres/second, forward positive
    double angular = 0.0;  // radians/second, left positive (ROS convention)
};
}
#endif
