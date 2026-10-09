// Retains A2's sector medians, distance/heading feedback and front hysteresis.
#include "project1/WallFollower.h"
#include <algorithm>
#include <cmath>
namespace project1
{
namespace
{
const double pi=3.14159265358979323846;
double Wrap(double a) { return std::atan2(std::sin(a),std::cos(a)); }
double Clamp(double value,double limit) { return std::max(-limit,std::min(limit,value)); }
}
WallFollower::WallFollower(const Settings& settings):settings_(settings) { settings_.Validate(); }
bool WallFollower::Manoeuvring() const
{ return state_==State::AdvanceRight || state_==State::TurnRight || state_==State::FindWall || state_==State::TurnLeft; }

void WallFollower::UpdateScan(const LaserScan& scan,double now)
{
    if (!std::isfinite(now) || now<lastScan_) { InvalidateScan("scan_time_invalid"); return; }
    lastScan_=now;
    // A wider side median tolerates several missing rays without selecting a far wall.
    // Prefer the same 5-degree right-sector measurement as the working A2.
    // Only widen to 15 degrees when that narrow sector has insufficient returns.
    right_=ScanProcessor::Sector(scan,-pi/2,5*pi/180,settings_.laserYaw,false);
    if (!right_.valid)
        right_=ScanProcessor::Sector(scan,-pi/2,15*pi/180,settings_.laserYaw,false);
    diagonal_=ScanProcessor::Sector(scan,-pi/4,5*pi/180,settings_.laserYaw,false);
    rear_=ScanProcessor::Sector(scan,-3*pi/4,5*pi/180,settings_.laserYaw,false);
    clearance_=ScanProcessor::MeasureClearance(scan,settings_);
    scanValid_=ScanProcessor::Sector(scan,0,pi,settings_.laserYaw,true).valid;
    scanFailure_="scan_invalid";
    // The original A2 steering angle and perpendicular distance are unchanged.
    // This estimate is used only by normal right-wall tracking.
    wallAngle_=0;
    if (diagonal_.valid && diagonal_.distance<settings_.lostWall*1.5)
        wallAngle_=std::atan2(diagonal_.distance*std::cos(pi/4)-right_.distance,
                              diagonal_.distance*std::sin(pi/4));
    normalDistance_=right_.distance*std::cos(wallAngle_);

    // This independent estimate is used ONLY to anchor/extrapolate a
    // right-wall endpoint at a corner, never to steer or limit cruise speed.
    cornerAngle_=0;
    headingValid_=right_.valid && right_.distance<=settings_.lostWall &&
        ScanProcessor::RightWallAngle(scan,settings_,right_.distance,cornerAngle_);
    if (!poseValid_ || !scanValid_) { gapScans_=nearScans_=0; return; }
    const double laserWorldX=x_+std::cos(yaw_)*settings_.laserX-std::sin(yaw_)*settings_.laserY;
    const double laserWorldY=y_+std::sin(yaw_)*settings_.laserX+std::cos(yaw_)*settings_.laserY;
    if (state_==State::WaitingForScan || state_==State::FollowWall)
    {
        if (right_.valid && right_.distance<=settings_.lostWall && headingValid_ && std::fabs(cornerAngle_)<pi/6)
        {
            const double heading=yaw_-cornerAngle_;
            const double cornerNormalDistance=right_.distance*std::cos(cornerAngle_);
            const double predicted=std::sin(wallHeading_)*(wallAnchorX_-laserWorldX)-
                                   std::cos(wallHeading_)*(wallAnchorY_-laserWorldY);
            if (!wallKnown_ || std::fabs(cornerNormalDistance-predicted)<0.08)
            {
                wallHeading_=heading; wallTime_=now; wallKnown_=true;
                wallAnchorX_=laserWorldX+cornerNormalDistance*std::sin(heading);
                wallAnchorY_=laserWorldY-cornerNormalDistance*std::cos(heading);
            }
        }
        const double distance=std::sin(wallHeading_)*(wallAnchorX_-laserWorldX)-
                              std::cos(wallHeading_)*(wallAnchorY_-laserWorldY);
        opening_=wallKnown_ && now-wallTime_<0.75 ?
            ScanProcessor::Opening(scan,settings_,distance,Wrap(wallHeading_-yaw_)) : ScanProcessor::Reading();
        // The original finite-return corner evidence remains unchanged.
        // For no-return-only evidence, require that the nearby right wall
        // really disappeared and that other current obstacle observations
        // still support safe movement. A failed/missing right sector alone
        // is NEVER sufficient evidence of a turn.
        if (opening_.valid && opening_.finiteEvidence<3)
        {
            const bool nearRight=right_.valid && right_.distance<=settings_.lostWall;
            if (nearRight || !clearance_.frontValid || !clearance_.pivotValid ||
                !std::isfinite(clearance_.pivot))
                opening_.valid=false;
        }
        gapScans_=opening_.valid ? gapScans_+1 : 0;
    }
    if (state_==State::FindWall)
        nearScans_=right_.valid && right_.distance<=settings_.lostWall ? nearScans_+1 : 0;
}

void WallFollower::UpdateOdometry(double x,double y,double yaw,double now)
{
    if (!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(yaw)||!std::isfinite(now)||now<poseTime_)
    { InvalidateOdometry("odom_invalid"); return; }
    const double change=poseSeen_ ? Wrap(yaw-yaw_) : 0;
    if (poseSeen_ && (std::hypot(x-x_,y-y_)>0.20 || std::fabs(change)>0.50))
    { InvalidateOdometry("odom_discontinuity",true); return; }
    yaw_=poseSeen_ ? yaw_+change : yaw; x_=x; y_=y; poseTime_=now; poseSeen_=poseValid_=true;
    if (Manoeuvring() && (std::hypot(x_-progressX_,y_-progressY_)>0.01 || std::fabs(yaw_-progressYaw_)>0.03))
    { progressTime_=now; progressX_=x_; progressY_=y_; progressYaw_=yaw_; }
}
void WallFollower::InvalidateScan(const std::string& reason)
{ scanValid_=false; scanFailure_=reason; gapScans_=nearScans_=0; }
void WallFollower::InvalidateOdometry(const std::string& reason,bool latch)
{ poseValid_=false; odomFailure_=reason; if(latch) Fault(reason); }
Velocity WallFollower::Hold(const std::string& reason)
{ reason_=reason; if(Manoeuvring()) progressTime_=now_; return Velocity(); }
Velocity WallFollower::Fault(const std::string& reason) { state_=State::Stopped; return Hold(reason); }
bool WallFollower::PivotSafe() const
{ return clearance_.pivotValid && std::isfinite(clearance_.pivot) && clearance_.pivot>0; }
bool WallFollower::ForwardSafe(double speed) const
{
    const double stop=settings_.clearanceMargin+speed*settings_.commandLatency+
                      speed*speed/(2*settings_.brakingDeceleration);
    return clearance_.frontValid && clearance_.front>stop;
}
void WallFollower::StartRight(double now)
{
    approachHeading_=wallHeading_; targetHeading_=approachHeading_-pi/2;
    const double heading=Wrap(approachHeading_-yaw_);
    advanceGoal_=opening_.distance+settings_.laserX*std::cos(heading)+settings_.laserY*std::sin(heading)+settings_.wallDistance;
    startX_=progressX_=x_; startY_=progressY_=y_; startYaw_=progressYaw_=yaw_;
    manoeuvreStart_=progressTime_=now; state_=State::AdvanceRight; gapScans_=nearScans_=0;
}

Velocity WallFollower::Command(double now)
{
    now_=now;
    if(state_==State::Stopped) return Velocity();
    if(!std::isfinite(now)) return Fault("control_time_invalid");
    if(Manoeuvring() && now-manoeuvreStart_>settings_.cornerTimeout) return Fault("manoeuvre_timeout");
    if(!poseValid_) return Hold(odomFailure_);
    if(now<poseTime_ || now-poseTime_>settings_.scanTimeout) return Hold("odom_stale");
    if(!scanValid_) return Hold(scanFailure_);
    if(now<lastScan_ || now-lastScan_>settings_.scanTimeout) return Hold("scan_stale");
    if(state_==State::WaitingForScan) state_=State::FollowWall;
    // A measured right opening precedes front avoidance; invalid side rays do not.
    if(state_==State::FollowWall && gapScans_>=3) StartRight(now);
    Velocity command;
    if(state_==State::AdvanceRight)
    {
        if(!clearance_.frontValid) return Hold("front_unobserved");
        if(!ForwardSafe(settings_.searchSpeed)) return Hold("corner_advance_obstructed");
        const double along=(x_-startX_)*std::cos(approachHeading_)+(y_-startY_)*std::sin(approachHeading_);
        if(along>=advanceGoal_) { state_=State::TurnRight; return Hold("right_pivot_ready"); }
        command.linear=settings_.searchSpeed;
        command.angular=Clamp(settings_.headingGain*Wrap(approachHeading_-yaw_),settings_.searchTurnSpeed);
        reason_="advancing_past_measured_edge";
    }
    else if(state_==State::TurnRight)
    {
        if(!PivotSafe()) return Hold(clearance_.pivotValid ? "pivot_clearance_insufficient" : "pivot_unobserved");
        if(yaw_-startYaw_>0.25 || startYaw_-yaw_>3*pi/4) return Fault("right_turn_angle_bound");
        const double error=Wrap(targetHeading_-yaw_);
        if(std::fabs(error)<3*pi/180)
        { state_=State::FindWall; startX_=x_; startY_=y_; nearScans_=0; return Hold("right_heading_reached"); }
        command.angular=Clamp(settings_.headingGain*error,settings_.turnSpeed); reason_="turning_right_with_feedback";
    }
    else if(state_==State::FindWall)
    {
        if(std::hypot(x_-startX_,y_-startY_)>settings_.cornerMaxEntry) return Fault("right_wall_acquisition_distance_bound");
        if(nearScans_>=3) { state_=State::FollowWall; wallKnown_=false; return Hold("right_wall_reacquired"); }
        if(!clearance_.frontValid) return Hold("front_unobserved");
        if(!ForwardSafe(settings_.searchSpeed)) return Hold("acquisition_obstructed");
        command.linear=settings_.searchSpeed;
        command.angular=Clamp(settings_.headingGain*Wrap(targetHeading_-yaw_),settings_.searchTurnSpeed);
        reason_="finding_near_right_wall";
    }
    else if(state_==State::TurnLeft)
    {
        if(!PivotSafe()) return Hold(clearance_.pivotValid ? "pivot_clearance_insufficient" : "pivot_unobserved");
        if(yaw_-startYaw_>pi+0.15 || yaw_-startYaw_<-0.25) return Fault("left_turn_angle_bound");
        if(clearance_.frontValid && clearance_.front>settings_.frontRelease && headingValid_ && std::fabs(wallAngle_)<0.25)
        { state_=State::FollowWall; wallKnown_=false; return Hold("front_obstacle_cleared"); }
        command.angular=settings_.turnSpeed; reason_="avoiding_front_obstacle";
    }
    else
    {
        if(!clearance_.frontValid) return Hold("front_unobserved");
        if(clearance_.front<settings_.frontStop)
        {
            if(!PivotSafe()) return Hold(clearance_.pivotValid ? "pivot_clearance_insufficient" : "pivot_unobserved");
            state_=State::TurnLeft; startYaw_=progressYaw_=yaw_; manoeuvreStart_=progressTime_=now;
            progressX_=x_; progressY_=y_; wallKnown_=false;
            command.angular=settings_.turnSpeed; reason_="avoiding_front_obstacle"; return command;
        }
        if(!right_.valid || right_.distance>settings_.lostWall) return Hold("right_wall_unobserved");
        // The original A2 feedback equation and speed reduction.
        command.angular=Clamp(settings_.distanceGain*(settings_.wallDistance-normalDistance_)-
                              settings_.headingGain*wallAngle_,settings_.turnSpeed);
        command.linear=settings_.forwardSpeed*(1-0.5*std::fabs(command.angular)/settings_.turnSpeed);
        // Corner-direction confidence must never gate straight-wall speed.
        if(gapScans_>0) command.linear=std::min(command.linear,settings_.searchSpeed);
        if(!ForwardSafe(command.linear)) return Hold("braking_clearance_insufficient");
        reason_=headingValid_ ? "following_right_wall" : "following_with_distance_only";
    }
    // PivotSafe checks a *stationary* rotation envelope, not a moving arc.
    // Preserve forward braking checks above. For forward right curves, also
    // bound the side displacement over the latency-plus-braking horizon.
    if (command.linear>0 && command.angular < -0.05 &&
        (state_==State::FollowWall ||
         (state_==State::FindWall && right_.valid &&
          right_.distance<=settings_.lostWall)))
    {
        const double horizon=settings_.commandLatency+
                             command.linear/settings_.brakingDeceleration;
        const double lateral=0.5*command.linear*std::fabs(command.angular)*horizon*horizon;
        const double sideClearance=right_.distance-settings_.bodyHalfWidth-
                                   std::fabs(settings_.laserY);
        if (!right_.valid || sideClearance<=settings_.clearanceMargin+lateral)
            return Hold("right_curve_clearance_insufficient");
    }
    if(Manoeuvring() && now-progressTime_>3.0) return Fault("odometry_no_progress");
    return command;
}
WallFollower::State WallFollower::GetState() const { return state_; }
const char* WallFollower::StateName(State s)
{
    switch(s) {
    case State::WaitingForScan:return "WaitingForScan"; case State::FollowWall:return "FollowWall";
    case State::TurnLeft:return "TurnLeft"; case State::FindWall:return "FindWall";
    case State::AdvanceRight:return "AdvanceRight"; case State::TurnRight:return "TurnRight";
    case State::Stopped:return "Stopped"; } return "Unknown";
}
WallFollower::Diagnostics WallFollower::GetDiagnostics(double now) const
{
    Diagnostics d; d.state=state_; d.reason=reason_; d.scanAge=now-lastScan_; d.odomAge=poseSeen_ ? now-poseTime_ : -1;
    d.frontClearance=clearance_.front; d.pivotClearance=clearance_.pivot;
    d.wallDistance=normalDistance_; d.wallHeading=-cornerAngle_; d.gapRays=opening_.usable;
    d.gapScans=gapScans_; d.rightUsable=right_.usable; d.rightTotal=right_.total;
    d.frontValid=clearance_.frontValid; d.pivotValid=clearance_.pivotValid;
    d.rightValid=right_.valid; d.headingValid=headingValid_;
    d.frontUnknownSpan=clearance_.frontUnknownSpan; d.pivotUnknownSpan=clearance_.pivotUnknownSpan;
    d.advanceRemaining=advanceGoal_-((x_-startX_)*std::cos(approachHeading_)+(y_-startY_)*std::sin(approachHeading_));
    d.turnError=Wrap(targetHeading_-yaw_); return d;
}
}
