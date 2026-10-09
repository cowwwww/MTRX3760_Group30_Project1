// Test-only closed-loop raycast/plant. Links unchanged original or delivered code.
#include "project1/WallFollower.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
namespace
{
constexpr double pi=3.14159265358979323846;
struct Point { double x,y; };
struct Line { Point a,b; };
struct Body { double laserX=-.032, front=.038, rear=.102, halfWidth=.089; };
using Scene=std::vector<Line>;
using project1::LaserScan;
double Cross(Point a,Point b) { return a.x*b.y-a.y*b.x; }
Point Sub(Point a,Point b) { return {a.x-b.x,a.y-b.y}; }
double Wrap(double a) { return std::atan2(std::sin(a),std::cos(a)); }
LaserScan Raycast(const Scene& scene,Point pose,double yaw,const Body& body,double time)
{
    LaserScan s;s.angleMin=0;s.angleIncrement=2*pi/503;s.rangeMin=.02;s.rangeMax=12;
    Point origin={pose.x+body.laserX*std::cos(yaw),pose.y+body.laserX*std::sin(yaw)};
    for(int i=0;i<503;i++)
    {
        const double a=i*s.angleIncrement+yaw;Point u={std::cos(a),std::sin(a)};
        double range=13;
        for(auto line:scene)
        {
            const Point edge=Sub(line.b,line.a),delta=Sub(line.a,origin);double den=Cross(u,edge);
            if(std::fabs(den)<1e-10)continue;
            const double r=Cross(delta,edge)/den,f=Cross(delta,u)/den;
            if(r>=0&&f>=0&&f<=1)range=std::min(range,r);
        }
        range+=.004*std::sin(i*1.731+time*.17+pose.x*9+pose.y*13);
        s.ranges.push_back(range>=s.rangeMin&&range<=s.rangeMax ? float(range) : float(NAN));
    }
    return s;
}
std::vector<std::size_t> SectorIndices(const LaserScan& s,double degrees)
{
    std::vector<std::size_t> indices;
    for(std::size_t i=0;i<s.ranges.size();i++)
        if(std::fabs(Wrap(s.angleMin+i*s.angleIncrement+pi/2))<=degrees*pi/180+1e-6)indices.push_back(i);
    return indices;
}
int Usable(const LaserScan& scan,const std::vector<std::size_t>& indices)
{
    int n=0;for(auto i:indices)if(std::isfinite(scan.ranges[i])&&scan.ranges[i]>=scan.rangeMin&&scan.ranges[i]<=scan.rangeMax)n++;
    return n;
}
// Narrow mask matches Floor 01: positions 0-9 missing, positions 10-13 finite.
// Finite distances remain real raycast hits; no far readings are fabricated.
void DropRight(LaserScan& scan,const std::string& mode)
{
    if(mode=="none")return;
    const auto narrow=SectorIndices(scan,5),wide=SectorIndices(scan,15);
    if(narrow.size()!=14 || wide.size()!=42)throw std::runtime_error("Expected 14/42 sector rays");
    if(mode=="blind") { for(auto i:wide)scan.ranges[i]=NAN;return; }
    if(mode=="wide")
    {
        std::vector<std::size_t> outside;
        for(auto i:wide)if(std::find(narrow.begin(),narrow.end(),i)==narrow.end())outside.push_back(i);
        // Preserve 18/28 outer rays plus 4/14 inner rays: 22/42 (<60%).
        for(std::size_t j=0;j<outside.size();j++)
            if((j*18/outside.size())==((j+1)*18/outside.size()))scan.ranges[outside[j]]=NAN;
    }
    for(std::size_t j=0;j<narrow.size();j++)
        if(j<10)scan.ranges[narrow[j]]=NAN;
}
bool Contact(const Scene& scene,Point pose,double yaw,const Body& body)
{
    const auto local=[pose,yaw](Point p){Point d=Sub(p,pose);return Point{std::cos(yaw)*d.x+std::sin(yaw)*d.y,-std::sin(yaw)*d.x+std::cos(yaw)*d.y};};
    for(auto line:scene)
    {
        Point a=local(line.a),b=local(line.b),d=Sub(b,a);double low=0,high=1;bool hit=true;
        const double origin[]={a.x,a.y},direction[]={d.x,d.y},minimum[]={-body.rear,-body.halfWidth},maximum[]={body.front,body.halfWidth};
        for(int k=0;k<2;k++)
        {
            if(std::fabs(direction[k])<1e-12){if(origin[k]<minimum[k]||origin[k]>maximum[k])hit=false;}
            else
            {
                double first=(minimum[k]-origin[k])/direction[k],last=(maximum[k]-origin[k])/direction[k];
                if(first>last)std::swap(first,last);
                low=std::max(low,first);high=std::min(high,last);
                if(low>high)hit=false;
            }
        }
        if(hit)return true;
    }
    return false;
}
}

