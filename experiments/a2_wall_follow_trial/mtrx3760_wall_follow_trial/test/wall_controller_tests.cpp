// Independent finite-segment raycasting and closed-loop controller checks.
#include "project1/WallFollower.h"
#include "project1/ScanProcessor.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
const double pi = 3.14159265358979323846;
using project1::LaserScan;
using project1::Settings;
using project1::Velocity;
using project1::WallFollower;
struct P { double x, y; };
struct Segment { P a, b; };
using Scene = std::vector<Segment>;
double Cross(P a, P b) { return a.x*b.y-a.y*b.x; }
P Sub(P a, P b) { return {a.x-b.x,a.y-b.y}; }
void Require(bool value, const std::string& message) { if (!value) throw std::runtime_error(message); }
void Zero(Velocity v) { Require(v.linear == 0.0 && v.angular == 0.0,"Expected zero command"); }
Scene Room()
{
    return {{{-5,-5},{5,-5}},{{5,-5},{5,5}},{{5,5},{-5,5}},{{-5,5},{-5,-5}}};
}
Scene Corridor(double width=.6, double end=4.0)
{
    Scene scene=Room();
    scene.push_back({{-3,-width/2},{end,-width/2}});
    scene.push_back({{-3,width/2},{end,width/2}});
    scene.push_back({{end,-width/2},{end,width/2}});
    return scene;
}
Scene Bend(double width=.6, bool endWall=false, bool junction=false, bool second=false)
{
    Scene scene=Room(); const double edge=1.4;
    scene.push_back({{-3,-width/2},{edge,-width/2}});
    if (junction)
    {
        scene.push_back({{-3,width/2},{5,width/2}});
        scene.push_back({{edge+width,-width/2},{5,-width/2}});
    }
    else scene.push_back({{-3,width/2},{edge+width,width/2}});
    scene.push_back({{edge,-width/2},{edge,second ? -1.3 : -4.0}});
    scene.push_back({{edge+width,junction ? -width/2 : width/2},{edge+width,second ? -1.3-width : -4.0}});
    if (endWall) scene.push_back({{edge,-width/2-1.0},{edge+width,-width/2-1.0}});
    if (second)
    {
        scene.push_back({{edge,-1.3},{-3,-1.3}});
        scene.push_back({{edge+width,-1.3-width},{-3,-1.3-width}});
    }
    return scene;
}
LaserScan Rays(const Scene& scene, P body, double yaw, const Settings& s, double noise=0.0)
{
    LaserScan scan; scan.angleMin=0.0; scan.angleIncrement=2*pi/504;
    scan.rangeMin=.02; scan.rangeMax=12.0;
    const P origin={body.x+std::cos(yaw)*s.laserX-std::sin(yaw)*s.laserY,
                    body.y+std::sin(yaw)*s.laserX+std::cos(yaw)*s.laserY};
    for (int i=0;i<504;++i)
    {
        const double a=scan.angleMin+i*scan.angleIncrement+s.laserYaw+yaw;
        const P direction={std::cos(a),std::sin(a)};
        double best=scan.rangeMax+1.0;
        for (const auto& line:scene)
        {
            const P delta=Sub(line.b,line.a), offset=Sub(line.a,origin);
            const double denominator=Cross(direction,delta);
            if (std::fabs(denominator)<1e-10) continue;
            const double distance=Cross(offset,delta)/denominator, fraction=Cross(offset,direction)/denominator;
            if (distance>=0.0 && fraction>=0.0 && fraction<=1.0) best=std::min(best,distance);
        }
        best += noise*std::sin(i*1.731+body.x*9.0+body.y*13.0);
        scan.ranges.push_back(best<=scan.rangeMax && best>=scan.rangeMin ? float(best) : float(NAN));
    }
    return scan;
}
void Drop(LaserScan& scan, double centre, double halfWidth)
{
    for (std::size_t i=0;i<scan.ranges.size();++i)
    {
        const double a=scan.angleMin+i*scan.angleIncrement;
        if (std::fabs(std::atan2(std::sin(a-centre),std::cos(a-centre)))<=halfWidth) scan.ranges[i]=NAN;
    }
}
Velocity At(WallFollower& controller, const LaserScan& scan, double time=1.0)
{
    controller.UpdateOdometry(0,0,0,time); controller.UpdateScan(scan,time); return controller.Command(time);
}
bool Collision(const Scene& scene, P body, double yaw, const Settings& s)
{
    const auto local=[body,yaw](P p) { const P d=Sub(p,body); return P{std::cos(yaw)*d.x+std::sin(yaw)*d.y,-std::sin(yaw)*d.x+std::cos(yaw)*d.y}; };
    for (const auto& line:scene)
    {
        const P a=local(line.a), b=local(line.b), delta=Sub(b,a);
        double low=0, high=1;
        const double origin[]={a.x,a.y}, direction[]={delta.x,delta.y};
        const double minimum[]={-s.bodyRear,-s.bodyHalfWidth}, maximum[]={s.bodyFront,s.bodyHalfWidth};
        bool intersects=true;
        for (int axis=0;axis<2;++axis)
        {
            if (std::fabs(direction[axis])<1e-12)
            { if (origin[axis]<minimum[axis] || origin[axis]>maximum[axis]) intersects=false; }
            else
            {
                double first=(minimum[axis]-origin[axis])/direction[axis], last=(maximum[axis]-origin[axis])/direction[axis];
                if (first>last) std::swap(first,last);
                low=std::max(low,first); high=std::min(high,last);
                if (low>high) intersects=false;
            }
        }
        if (intersects) return true;
    }
    return false;
}
struct Result { bool first=false, second=false, advanced=false; std::string reason; double x=0,y=0,yaw=0; };
Result Run(Scene scene, double width, double startingYaw=0, double offset=0,
           int dropout=0, bool twoCorners=false, bool insertObstacle=false, const std::string& label="trial", bool deadEnd=false)
{
    Settings settings; settings.forwardSpeed=.08; settings.searchSpeed=.05;
    settings.wallDistance=.25; settings.frontStop=.28; settings.frontRelease=.38;
    WallFollower controller(settings);
    P pose={0,offset}; double yaw=startingYaw, actualV=0, actualW=0;
    Result result; bool obstacleAdded=false;
    std::ofstream record(label+".csv");
    record<<"time,x,y,yaw,v,w,state,reason,front,pivot,wall,gap_rays,advance_remaining,turn_error\n";
    for (int tick=0;tick<2400;++tick)
    {
        const double now=1.0+tick*.05;
        controller.UpdateOdometry(pose.x,pose.y,yaw,now);
        if (tick%2==0)
        {
            auto scan=Rays(scene,pose,yaw,settings,.004);
            if (dropout==1) { Drop(scan,-pi/2,.08); Drop(scan,-pi/4,.08); }
            if (dropout==2 && controller.GetState()==WallFollower::State::TurnRight && tick%16<4)
                for (auto& r:scan.ranges) r=NAN;
            controller.UpdateScan(scan,now);
        }
        if (insertObstacle && !obstacleAdded && controller.GetState()==WallFollower::State::AdvanceRight)
        {
            const double x=1.4+settings.wallDistance;
            scene.push_back({{x-.08,-.12},{x-.08,.12}}); obstacleAdded=true;
        }
        const auto command=controller.Command(now); const auto d=controller.GetDiagnostics(now);
        result.advanced=result.advanced || d.state==WallFollower::State::AdvanceRight;
        Require(command.linear>=0 && command.linear<=settings.forwardSpeed &&
                std::fabs(command.angular)<=settings.turnSpeed+1e-9,"Command outside configured bounds");
        record<<now<<','<<pose.x<<','<<pose.y<<','<<yaw<<','<<command.linear<<','<<command.angular<<','
              <<WallFollower::StateName(d.state)<<','<<d.reason<<','<<d.frontClearance<<','<<d.pivotClearance<<','
              <<d.wallDistance<<','<<d.gapRays<<','<<d.advanceRemaining<<','<<d.turnError<<'\n';
        Require(!Collision(scene,pose,yaw,settings),"Body intersects wall at "+label+" "+d.reason);
        // Independent plant: first-order velocity response, then midpoint integration.
        actualV += .05/.12*(command.linear-actualV); actualW += .05/.12*(command.angular-actualW);
        pose.x += actualV*.05*std::cos(yaw+actualW*.025);
        pose.y += actualV*.05*std::sin(yaw+actualW*.025); yaw += actualW*.05;
        if (pose.y < -width/2-.30 && pose.x>1.50 && pose.x<1.4+width-.10 &&
            std::fabs(yaw+pi/2)<.18 && d.state==WallFollower::State::FollowWall) result.first=true;
        if (deadEnd && pose.x < 0.2 && std::fabs(yaw-pi)<.20 && d.state==WallFollower::State::FollowWall)
            result.first = true;
        if (result.first && !twoCorners) break;
        if (pose.x<1.1 && pose.y<-1.35 && std::fabs(yaw+pi)<.20 && d.state==WallFollower::State::FollowWall)
        { result.second=true; break; }
        result.reason=d.reason;
        if (d.state==WallFollower::State::Stopped) break;
        if (insertObstacle && (d.reason=="corner_advance_obstructed" || d.reason=="body_clearance_occupied")) break;
    }
    result.x=pose.x; result.y=pose.y; result.yaw=yaw;
    if (!result.first) std::cerr<<label<<" failed x="<<pose.x<<" y="<<pose.y<<" yaw="<<yaw<<" reason="<<result.reason<<'\n';
    return result;
}
void SettingsCheck()
{
    Settings().Validate();
    Settings s; s.laserX=10; bool rejected=false;
    try { s.Validate(); } catch (const std::invalid_argument&) { rejected=true; }
    Require(rejected,"Sensor outside body accepted");
    s=Settings(); s.maxUnobservedSpan=s.bodyHalfWidth*2; rejected=false;
    try { s.Validate(); } catch (const std::invalid_argument&) { rejected=true; }
    Require(rejected,"Unlimited missing-ray uncertainty accepted");
}
void ScanGeometry()
{
    Settings s;
    const auto check=[&s](const LaserScan& scan) {
        const auto side=project1::ScanProcessor::Sector(scan,-pi/2,15*pi/180,s.laserYaw,false);
        const auto c=project1::ScanProcessor::MeasureClearance(scan,s);
        Require(side.valid && std::fabs(side.distance-.3)<.015,"Wall distance geometry wrong");
        Require(c.front>3.0 && c.pivot>0,"Side walls classified as front obstacles");
        Require(c.frontValid && c.pivotValid,"Finite scan not observable");
    };
    auto scan=Rays(Corridor(),{0,0},0,s); check(scan);
    s.laserYaw=pi/2; scan=Rays(Corridor(),{0,0},0,s); check(scan);
    std::reverse(scan.ranges.begin(),scan.ranges.end()); scan.angleMin=503*scan.angleIncrement;
    scan.angleIncrement=-scan.angleIncrement; check(scan);
}
void Heading()
{
    Settings s; WallFollower left(s),right(s);
    auto a=Rays(Corridor(),{0,0},.12,s); auto b=Rays(Corridor(),{0,0},-.12,s);
    Require(At(left,a).angular<0 && At(right,b).angular>0,"Wall-heading correction wrong sign");
}
void SideDropout()
{
    Settings s; WallFollower controller(s); auto scan=Rays(Corridor(),{0,0},0,s);
    Drop(scan,-pi/2,.09); Drop(scan,-pi/4,.09);
    const auto v=At(controller,scan); Require(v.linear>.07,"Narrow side/diagonal dropouts stop useful geometry");
}
void PhysicalRegression()
{
    Settings s; s.wallDistance=.25; s.frontStop=.28; s.frontRelease=.38;
    auto open=Rays(Corridor(),{0,0},0,s);
    // Physical-LDS-style +inf means a ray observed no finite return.
    // Deliberately create a 0.28-radian forward sector of +inf readings.
    for (std::size_t i=0;i<open.ranges.size();++i)
    {
        const double a=std::atan2(std::sin(open.angleMin+i*open.angleIncrement),
                                  std::cos(open.angleMin+i*open.angleIncrement));
        if(a>0.14 && a<0.42) open.ranges[i]=std::numeric_limits<float>::infinity();
    }
    auto c=project1::ScanProcessor::MeasureClearance(open,s);
    Require(c.frontValid,"Positive infinity from LDS incorrectly rejected as blind");
    Require(c.pivotValid,"Positive infinity incorrectly invalidated pivot observability");
    WallFollower follower(s);
    Require(At(follower,open).linear>0,"No-return readings stopped unobstructed cruise");

    // Identical angular gap of genuine invalid NaNs must remain unobservable.
    for(auto& r:open.ranges) if(std::isinf(r)) r=std::numeric_limits<float>::quiet_NaN();
    c=project1::ScanProcessor::MeasureClearance(open,s);
    Require(!c.frontValid,"NaNs incorrectly treated as a verified clear path");
    Zero(At(follower,open,1.1));

    // The body may be too close to pivot, but still have space to move
    // forwards while steering left away from the right wall.
    auto close=Rays(Corridor(),{0,-.16},0,s);
    WallFollower closeFollower(s);
    const auto clearance=project1::ScanProcessor::MeasureClearance(close,s);
    Require(clearance.pivot<0,"Test fixture should be too close for stationary pivot");
    const auto moving=At(closeFollower,close);
    Require(moving.linear>0 && moving.angular>0,
            "Stationary-pivot envelope wrongly prohibited forward steering");
}
void OriginalA2Cruise()
{
    Settings s; s.wallDistance=.25; s.frontStop=.28; s.frontRelease=.38;
    const auto scan=Rays(Corridor(),{0,0},.12,s);
    const auto right=project1::ScanProcessor::Sector(scan,-pi/2,5*pi/180,s.laserYaw,false);
    const auto diagonal=project1::ScanProcessor::Sector(scan,-pi/4,5*pi/180,s.laserYaw,false);
    Require(right.valid && diagonal.valid,"Original A2 sectors unavailable");
    double angle=0;
    if(diagonal.distance<s.lostWall*1.5)
        angle=std::atan2(diagonal.distance*std::cos(pi/4)-right.distance,
                         diagonal.distance*std::sin(pi/4));
    const double dist=right.distance*std::cos(angle);
    const double expected=std::max(-s.turnSpeed,std::min(s.turnSpeed,
        s.distanceGain*(s.wallDistance-dist)-s.headingGain*angle));
    WallFollower follower(s);
    const auto cmd=At(follower,scan);
    Require(cmd.linear>0 && std::fabs(cmd.angular-expected)<1e-6,
            "Normal steering diverges from last working A2 equation");
}
void Unknown()
{
    Settings s; WallFollower controller(s); auto scan=Rays(Room(),{0,0},0,s);
    for (float value:{float(NAN),float(INFINITY),-float(INFINITY),0.0f})
    { for (auto& r:scan.ranges)r=value; Zero(At(controller,scan)); Require(controller.GetState()!=WallFollower::State::AdvanceRight,"Unknown readings imply opening"); }
    scan.angleIncrement=0; Zero(At(controller,scan));
}
void Stale()
{
    Settings s; WallFollower controller(s); auto scan=Rays(Corridor(),{0,0},0,s);
    Require(At(controller,scan).linear>0,"Fresh inputs must drive"); Zero(controller.Command(1.6));
    controller.UpdateOdometry(0,0,0,1.6); Zero(controller.Command(1.6));
    controller.UpdateScan(scan,1.6); Require(controller.Command(1.6).linear>0,"Fresh data must resume");
}
void ObstaclePriority()
{
    Settings s; WallFollower controller(s); auto v=At(controller,Rays(Corridor(.6,.30),{0,0},0,s));
    Require(v.linear==0 && v.angular>0,"Real front wall should start bounded left avoidance");
}
void Recovery()
{
    Settings s; WallFollower controller(s); auto scan=Rays(Corridor(),{0,0},0,s); At(controller,scan);
    auto bad=scan; Drop(bad,0,.50); Zero(At(controller,bad,1.1));
    Require(controller.GetDiagnostics(1.1).reason=="front_unobserved","Missing front stop lacks reason");
    Require(At(controller,scan,1.2).linear>0,"Temporary missing data permanently latched stop");
}
void PoseJump()
{
    Settings s; WallFollower controller(s); At(controller,Rays(Corridor(),{0,0},0,s));
    controller.UpdateOdometry(1,0,0,1.05); Zero(controller.Command(1.05));
    Require(controller.GetDiagnostics(1.05).reason=="odom_discontinuity","Pose reset not diagnosed");
    controller.UpdateOdometry(0,0,0,1.1); Zero(controller.Command(1.1));
}
void NoProgress()
{
    Settings s; WallFollower controller(s); auto scan=Rays(Corridor(.6,.3),{0,0},0,s);
    for(int i=0;i<80;++i)At(controller,scan,1+i*.05);
    Require(controller.GetDiagnostics(5).reason=="odometry_no_progress","Unbounded stationary commanded spin");
}
void Widths()
{
    for(double w:{.6,.7,.8}) Require(Run(Bend(w),w,0,0,0,false,false,"width_"+std::to_string(w)).first,"Right bend failed across widths");
}
void Approaches()
{
    for(double w:{.6,.8})for(double a:{-.20,.20})for(double offset:{-.06,.06})
        Require(Run(Bend(w),w,a,offset,0,false,false,"approach_"+std::to_string(w)+"_"+std::to_string(a)+"_"+std::to_string(offset)).first,"Skewed approach failed");
}
void Junction() { Require(Run(Bend(.7,false,true),.7,0,0,0,false,false,"junction").first,"Right opening ignored when straight ahead available"); }
void EndWall() { for(double w:{.6,.8})Require(Run(Bend(w,true),w,0,0,0,false,false,"endwall_"+std::to_string(w)).first,"One-metre outgoing end wall prevents first turn"); }
void SecondCorner() { auto r=Run(Bend(.7,false,false,true),.7,0,0,0,true,false,"second_corner");Require(r.first&&r.second,"Repeated right bend failed"); }
void CornerDropouts()
{
    for(double w:{.6,.8})for(double a:{-.20,.20})
        Require(Run(Bend(w,true),w,a,0,1,false,false,"corner_dropout_"+std::to_string(w)+"_"+std::to_string(a)).first,
                "Persistent narrow sector holes prevent turn on skewed approach");
}
void ObstacleCorner() { auto r=Run(Bend(.7),.7,0,0,0,false,true,"corner_obstacle");Require(r.advanced&&!r.first&&r.x>1.0&&r.reason=="corner_advance_obstructed","Injected corner obstacle not stopped and diagnosed on first bend"); }
void SensorOutage() { Require(Run(Bend(.6),.6,0,0,2,false,false,"corner_outage").first,"Temporary complete scan loss during pivot prevents recovery"); }
void DeadEnd() { for(double w:{.6,.8})Require(Run(Corridor(w,1.4),w,0,0,0,false,false,"dead_end_"+std::to_string(w),true).first,"Closed corridor does not return along its wall"); }
void Bounds()
{
    auto scene=Bend(.7); scene.erase(scene.begin()+6);
    const auto r=Run(scene,.7,0,0,0,false,false,"acquisition_bound");
    Require(!r.first,"Missing outgoing right wall acquired from unrelated far wall");
    Require(r.reason=="right_wall_acquisition_distance_bound" || r.reason=="manoeuvre_timeout" || r.reason=="acquisition_obstructed", "Acquisition has no diagnosed bound");
}
}
int main(int argc,char** argv)
{
    const std::map<std::string,std::function<void()>> checks={
        {"settings",SettingsCheck},{"scan_geometry",ScanGeometry},{"wall_heading",Heading},{"side_dropout",SideDropout},
        {"physical_regression",PhysicalRegression},{"original_a2_cruise",OriginalA2Cruise},{"unknown_scan",Unknown},{"stale_inputs",Stale},{"obstacle_priority",ObstaclePriority},{"sensor_recovery",Recovery},
        {"pose_jump",PoseJump},{"no_progress",NoProgress},{"corridor_widths",Widths},{"corner_approaches",Approaches},
        {"right_junction",Junction},{"end_wall",EndWall},{"second_corner",SecondCorner},{"corner_dropouts",CornerDropouts},
        {"obstacle_in_corner",ObstacleCorner},{"sensor_outage_corner",SensorOutage},{"dead_end",DeadEnd},{"bounds",Bounds}};
    try { if(argc!=2||!checks.count(argv[1]))throw std::runtime_error("Unknown check");checks.at(argv[1])();std::cout<<"PASS "<<argv[1]<<'\n'; }
    catch(const std::exception& e){std::cerr<<"FAIL "<<(argc>1?argv[1]:"")<<": "<<e.what()<<'\n';return 1;}
}
