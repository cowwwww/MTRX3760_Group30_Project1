// A2 sector-based right-wall steering, with bounded corner feedback.
#ifndef PROJECT1_WALLFOLLOWER_H
#define PROJECT1_WALLFOLLOWER_H
#include "project1/ScanProcessor.h"
#include "project1/Settings.h"
#include <string>
namespace project1
{
class WallFollower
{
public:
    enum class State { WaitingForScan, FollowWall, TurnLeft, FindWall, AdvanceRight, TurnRight, Stopped };
    struct Diagnostics
    {
        State state = State::WaitingForScan;
        std::string reason = "scan_missing";
        double scanAge=0,odomAge=0,frontClearance=0,pivotClearance=0;
        double wallDistance=0,wallHeading=0,advanceRemaining=0,turnError=0;
        unsigned int gapRays=0,gapScans=0,rightUsable=0,rightTotal=0;
        bool frontValid=false,pivotValid=false,rightValid=false,headingValid=false;
        double frontUnknownSpan=0,pivotUnknownSpan=0;
    };
    explicit WallFollower(const Settings& settings = Settings());
    void UpdateScan(const LaserScan& scan,double now);
    void UpdateOdometry(double x,double y,double yaw,double now);
    void InvalidateScan(const std::string& reason);
    void InvalidateOdometry(const std::string& reason,bool latch=false);
    Velocity Command(double now);
    State GetState() const;
    Diagnostics GetDiagnostics(double now) const;
    static const char* StateName(State state);
private:
    bool Manoeuvring() const;
    Velocity Hold(const std::string& reason);
    Velocity Fault(const std::string& reason);
    bool ForwardSafe(double speed) const;
    bool PivotSafe() const;
    void StartRight(double now);
    Settings settings_;
    ScanProcessor::Reading right_,diagonal_,rear_,opening_;
    ScanProcessor::Clearance clearance_;
    State state_=State::WaitingForScan;
    std::string reason_="scan_missing",scanFailure_="scan_missing",odomFailure_="odom_missing";
    bool scanValid_=false,poseValid_=false,poseSeen_=false,headingValid_=false,wallKnown_=false;
    double lastScan_=0,poseTime_=0,wallTime_=0,now_=0;
    double x_=0,y_=0,yaw_=0,wallAngle_=0,normalDistance_=0;
    double lastRightDistance_=0, lastRightTime_=0, filteredDistanceRate_=0;
    bool distanceRateReady_=false;
    double wallHeading_=0,wallAnchorX_=0,wallAnchorY_=0;
    unsigned int gapScans_=0,nearScans_=0;
    double startX_=0,startY_=0,startYaw_=0,approachHeading_=0,targetHeading_=0,advanceGoal_=0;
    double manoeuvreStart_=0,progressTime_=0,progressX_=0,progressY_=0,progressYaw_=0;
};
}
#endif
