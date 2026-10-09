// Validated configuration for the right-wall controller.
#ifndef PROJECT1_SETTINGS_H
#define PROJECT1_SETTINGS_H


namespace project1
{
struct Settings
{
    double wallDistance = 0.30; // from laser origin, not robot skin
    double forwardSpeed = 0.10;
    double turnSpeed = 0.60;
    double searchSpeed = 0.08;
    double searchTurnSpeed = 0.28;
    double frontStop = 0.40;
    double frontRelease = 0.50;
    double lostWall = 0.65;
    double distanceGain = 1.5;
    double headingGain = 1.2;
    double scanTimeout = 0.50;
    double laserYaw = 0.0; // laser heading relative to robot, radians
    void Validate() const;
};
}
#endif
