#include "robot/core.hpp"
#include <cmath>
#include <stdexcept>
namespace robot {
std::vector<double> lidar(const World& world, Pose pose, Scan scan) {
    if(scan.beams<=0 || scan.max_range<=0 || !std::isfinite(scan.max_range)) throw std::invalid_argument("invalid scan");
    std::vector<double> ranges;
    const double increment=world.resolution()/50;
    for(int beam=0;beam<scan.beams;++beam) {
        const double angle=pose.heading+scan.start_angle+beam*scan.step;
        double range=0;
        while(range<scan.max_range && world.free({pose.x+range*std::cos(angle),pose.y+range*std::sin(angle)},0))
            range+=increment;
        ranges.push_back(std::min(range,scan.max_range));
    }
    return ranges;
}
}
