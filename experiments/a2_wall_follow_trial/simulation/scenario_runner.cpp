// Only this environment knows the maze. Controllers receive scans and odometry.
#include "simulation_geometry.h"

namespace
{
struct Config
{
    double width=.6,edge=1.4,depth=2,rotation=0,startY=0,startYaw=0,maxTime=180;
    double target=.25,lost=.65,lag=.12;
    bool deadEnd=false,junction=false,obstacle=false;
    std::string fault="none",output="trace.csv";
};
Point Rotate(Point p,double angle)
{ return {p.x*std::cos(angle)-p.y*std::sin(angle),p.x*std::sin(angle)+p.y*std::cos(angle)}; }
Scene MakeMaze(const Config& c)
{
    const double e=c.edge,w=c.width/2,end=-w-c.depth;
    const double lower=c.deadEnd ? end : -6;
    Scene scene={{{-8,-8},{8,-8}},{{8,-8},{8,8}},{{8,8},{-8,8}},{{-8,8},{-8,-8}},
                 {{-3,-w},{e,-w}},{{e,-w},{e,lower}},{{e+c.width,-w},{e+c.width,lower}}};
    if(c.junction)
    {
        scene.push_back({{-3,w},{6,w}});
        scene.push_back({{e+c.width,-w},{6,-w}});
    }
    else
    {
        scene.push_back({{-3,w},{e+c.width,w}});
        scene.push_back({{e+c.width,w},{e+c.width,-w}});
    }
    if(c.deadEnd)scene.push_back({{e,end},{e+c.width,end}});
    if(c.obstacle)
    {
        const double a=.55,b=.85,top=-w+.10;
        scene.push_back({{a,-w},{a,top}});
        scene.push_back({{a,top},{b,top}});
        scene.push_back({{b,top},{b,-w}});
    }
    for(auto& line:scene) { line.a=Rotate(line.a,c.rotation);line.b=Rotate(line.b,c.rotation); }
    return scene;
}
double CapGap(Point pose,double yaw,const Config& c,const Body& b)
{
    double minimum=100;
    for(const auto corner:std::vector<Point>{{b.front,b.halfWidth},{b.front,-b.halfWidth},
                                           {-b.rear,b.halfWidth},{-b.rear,-b.halfWidth}})
        minimum=std::min(minimum,pose.y+Rotate(corner,yaw).y+c.width/2+c.depth);
    return minimum;
}
#ifdef ORIGINAL_CONTROLLER
const char* StateName(project1::WallFollower::State state)
{
    using S=project1::WallFollower::State;
    if(state==S::WaitingForScan)return "WaitingForScan";
    if(state==S::FollowWall)return "FollowWall";
    if(state==S::TurnLeft)return "TurnLeft";
    return "FindWall";
}
#endif
}

int main(int argc,char** argv)
{
    try
    {
        Config c;
        for(int i=1;i<argc;i+=2)
        {
            if(i+1>=argc)throw std::runtime_error("Expected option/value pair");
            const std::string key=argv[i],v=argv[i+1];
            if(key=="--width")c.width=std::stod(v);else if(key=="--edge")c.edge=std::stod(v);
            else if(key=="--depth")c.depth=std::stod(v);else if(key=="--rotation")c.rotation=std::stod(v)*pi/180;
            else if(key=="--start-y")c.startY=std::stod(v);else if(key=="--start-yaw")c.startYaw=std::stod(v)*pi/180;
            else if(key=="--max-time")c.maxTime=std::stod(v);else if(key=="--lag")c.lag=std::stod(v);
            else if(key=="--target")c.target=std::stod(v);else if(key=="--lost")c.lost=std::stod(v);
            else if(key=="--dead-end")c.deadEnd=v=="true";else if(key=="--junction")c.junction=v=="true";
            else if(key=="--obstacle")c.obstacle=v=="true";else if(key=="--fault")c.fault=v;
            else if(key=="--output")c.output=v;else throw std::runtime_error("Unknown option: "+key);
        }
        if(c.width<.3 || c.width>2 || c.edge<.9 || c.depth<.4 || c.lag<.05 || c.maxTime<=0)
            throw std::runtime_error("Invalid environment dimensions/timing");
        if(c.fault!="none"&&c.fault!="partial"&&c.fault!="blind"&&c.fault!="transient_right"&&c.fault!="transient_front")
            throw std::runtime_error("Invalid fault name");
        project1::Settings settings;settings.wallDistance=c.target;settings.lostWall=c.lost;
        settings.forwardSpeed=.08;settings.searchSpeed=.05;settings.searchTurnSpeed=.28;
        settings.frontStop=.28;settings.frontRelease=.38;
        project1::WallFollower controller(settings);
        Body body;const Scene maze=MakeMaze(c);
        Point pose=Rotate({0,c.startY},c.rotation);
        double yaw=c.rotation+c.startYaw,actualV=0,actualW=0;
        bool faultStarted=false,faultEnded=false,scanFault=false,entered=false,approached=false,returned=false,completed=false;
        double faultTime=0,minCapGap=100;
        LaserScan scan;
        std::ofstream out(c.output);
        if(!out)throw std::runtime_error("Cannot write trace");
        out<<"time,x,y,yaw_deg,v,w,state,reason,fault,narrow_usable,wide_usable,front_sector,front_clearance,pivot_valid,heading_valid,right_valid,right_distance,cap_gap,min_cap_gap,entered,approached,returned,contact,completed\n";
        out<<std::setprecision(9);
        for(int tick=0;tick<=int(c.maxTime/.05);tick++)
        {
            const double time=tick*.05,clock=time+1;
            const Point local=Rotate(pose,-c.rotation);const double angle=yaw-c.rotation;
#ifndef ORIGINAL_CONTROLLER
            controller.UpdateOdometry(pose.x,pose.y,yaw,clock);
#endif
            if(tick%2==0)
            {
                scan=Raycast(maze,pose,yaw,body,time);
                if(!faultStarted && ((c.fault=="transient_front" && entered && local.y<-.8) ||
                   (c.fault!="transient_front" && local.x>c.edge-.2 && -angle>=22.5*pi/180 && -angle<85*pi/180)))
                { faultStarted=true;faultTime=time; }
                if(faultStarted && -angle>=85*pi/180)faultEnded=true;
                scanFault=false;
                if(c.fault=="partial" || c.fault=="blind")
                    scanFault=faultStarted&&!faultEnded;
                else if(c.fault=="transient_right")scanFault=faultStarted&&time-faultTime<.8;
                else if(c.fault=="transient_front")scanFault=faultStarted&&time-faultTime<.8;
                if(scanFault)
                {
                    if(c.fault=="transient_front")
                    {
                        for(std::size_t i=0;i<scan.ranges.size();i++)
                            if(std::fabs(Wrap(scan.angleMin+i*scan.angleIncrement))<35*pi/180)scan.ranges[i]=NAN;
                    }
                    else DropRight(scan,c.fault=="partial" ? "wide" : "blind");
                }
                controller.UpdateScan(scan,clock);
            }
            const auto command=controller.Command(clock);
            const auto right=project1::ScanProcessor::Sector(scan,-pi/2,15*pi/180,0,false);
            const auto front=project1::ScanProcessor::Sector(scan,0,35*pi/180,0,true);
            std::string state,reason;double frontClearance=-1;int pivot=-1,heading=-1,rightValid=right.valid;
#ifdef ORIGINAL_CONTROLLER
            state=StateName(controller.GetState());
            if(state=="WaitingForScan")reason="scan_sector_invalid";
            else if(state=="TurnLeft")reason="front_sector_priority";
            else if(state=="FindWall")reason="unbounded_right_search";else reason="following_right_wall";
#else
            const auto d=controller.GetDiagnostics(clock);
            state=project1::WallFollower::StateName(d.state);reason=d.reason;
            frontClearance=d.frontClearance;pivot=d.pivotValid;heading=d.headingValid;rightValid=d.rightValid;
#endif
            if(local.y<-c.width/2-.3 && std::fabs(Wrap(angle+pi/2))<.18 && state=="FollowWall")entered=true;
            const double capGap=c.deadEnd ? CapGap(local,angle,c,body) : -1;
            if(c.deadEnd) { minCapGap=std::min(minCapGap,capGap);if(entered && capGap<.4)approached=true; }
            if(approached && std::fabs(Wrap(angle-pi/2))<.18 && local.y>-c.width/2-.2 && state=="FollowWall")returned=true;
            completed=c.deadEnd ? returned : entered;
            const bool contact=Contact(maze,pose,yaw,body);
            out<<time<<','<<local.x<<','<<local.y<<','<<angle*180/pi<<','<<command.linear<<','<<command.angular<<','
               <<state<<','<<reason<<','<<scanFault<<','<<Usable(scan,SectorIndices(scan,5))<<','<<Usable(scan,SectorIndices(scan,15))<<','
               <<(front.valid?front.distance:-1)<<','<<frontClearance<<','<<pivot<<','<<heading<<','<<rightValid<<','
               <<(right.valid?right.distance:-1)<<','<<capGap<<','<<minCapGap<<','<<entered<<','<<approached<<','<<returned<<','<<contact<<','<<completed<<'\n';
            if(contact) { std::cerr<<"body_contact at "<<time<<" s\n";return 2; }
            if(completed)break;
            if(command.linear<0 || command.linear>settings.forwardSpeed+1e-8 || std::fabs(command.angular)>settings.turnSpeed+1e-8)
                throw std::runtime_error("Command outside configured bounds");
            actualV+=.05/c.lag*(command.linear-actualV);actualW+=.05/c.lag*(command.angular-actualW);
            pose.x+=actualV*.05*std::cos(yaw+actualW*.025);
            pose.y+=actualV*.05*std::sin(yaw+actualW*.025);yaw+=actualW*.05;
        }
        std::cout<<"completed="<<completed<<" entered="<<entered<<" approached="<<approached<<" returned="<<returned<<" min_cap_gap="<<minCapGap<<'\n';
        return 0;
    }
    catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1; }
}
