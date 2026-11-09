#include "robot/core.hpp"
#include <cmath>
#include <stdexcept>
namespace robot {
double wrap(double angle) { return std::remainder(angle,2*pi); }
double distance(Point first, Point second) { return std::hypot(first.x-second.x,first.y-second.y); }
Twist wheel_twist(Drive drive, double left, double right) {
    if(!std::isfinite(drive.wheel_radius) || !std::isfinite(drive.axle_length) || drive.wheel_radius<=0 || drive.axle_length<=0 || !std::isfinite(left) || !std::isfinite(right)) throw std::invalid_argument("finite wheel rates and positive finite geometry required");
    return {drive.wheel_radius*(left+right)/2,drive.wheel_radius*(right-left)/drive.axle_length};
}
Pose integrate(Pose pose, Twist command, double seconds) {
    if(!std::isfinite(seconds) || seconds<0 || !std::isfinite(command.speed) || !std::isfinite(command.yaw_rate))
        throw std::invalid_argument("finite command and nonnegative timestep required");
    const double turn=command.yaw_rate*seconds;
    const double scale=std::abs(turn)<1e-8 ? 1-turn*turn/24 : std::sin(turn/2)/(turn/2);
    pose.x+=command.speed*seconds*scale*std::cos(pose.heading+turn/2);
    pose.y+=command.speed*seconds*scale*std::sin(pose.heading+turn/2);
    pose.heading=wrap(pose.heading+turn);
    return pose;
}
}
