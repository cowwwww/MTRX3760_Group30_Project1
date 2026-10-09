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
    double laserX = -0.032, laserY = 0.0; // recorded base_link -> base_scan
    double bodyFront = 0.038, bodyRear = 0.102, bodyHalfWidth = 0.089;
    double clearanceMargin = 0.025;
    double brakingDeceleration = 0.20, commandLatency = 0.50;
    double maxUnobservedSpan = 0.05; // required for stationary pivot clearance
    double maxFrontUnobservedSpan = 0.08; // small front-sector no-return gaps
    double cornerLookahead = 0.30, cornerTimeout = 30.0, cornerMaxEntry = 0.80;
    void Validate() const;
};
}
#endif
