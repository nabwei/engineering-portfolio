#include "robot/core.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace robot {
Twist pursuit(Pose pose, Point target, double speed) {
    const double separation=distance({pose.x,pose.y},target);
    if(separation<1e-9) return {};
    const double angle=wrap(std::atan2(target.y-pose.y,target.x-pose.x)-pose.heading);
    if(std::abs(angle)>pi/3) return {0,std::clamp(2*angle,-1.5,1.5)};
    return {speed,std::clamp(2*speed*std::sin(angle)/separation,-1.5,1.5)};
}
Point lookahead(const std::vector<Point>& path, Point position, double length) {
    if(path.empty() || length<0) throw std::invalid_argument("nonempty path and nonnegative lookahead required");
    auto nearest=std::min_element(path.begin(),path.end(),[&](Point a,Point b){return distance(a,position)<distance(b,position);});
    while(nearest+1!=path.end() && distance(*nearest,position)<length) ++nearest;
    return *nearest;
}
}
