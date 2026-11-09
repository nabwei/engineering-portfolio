#include "robot/core.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot {
World::World(int width, int height, double resolution)
    : width_(width), height_(height), resolution_(resolution) {
    if (width <= 0 || height <= 0 || !std::isfinite(resolution) || resolution <= 0)
        throw std::invalid_argument("positive grid dimensions and resolution required");
    occupied_.resize(static_cast<std::size_t>(width)*height);
}
void World::occupy(int column, int row) {
    if(column<0 || row<0 || column>=width_ || row>=height_)
        throw std::out_of_range("occupied cell outside map");
    occupied_[row*width_+column]=true;
}
bool World::occupied(int column, int row) const {
    return column<0 || row<0 || column>=width_ || row>=height_ || occupied_[row*width_+column];
}
Point World::center(int column, int row) const {
    return {(column+0.5)*resolution_, (row+0.5)*resolution_};
}
bool World::free(Point point, double radius) const {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(radius) || radius<0)
        throw std::invalid_argument("finite position and nonnegative radius required");
    if(point.x-radius<=0 || point.y-radius<=0 || point.x+radius>=width_*resolution_ || point.y+radius>=height_*resolution_) return false;
    for(int row=static_cast<int>((point.y-radius)/resolution_); row<=static_cast<int>((point.y+radius)/resolution_); ++row)
        for(int column=static_cast<int>((point.x-radius)/resolution_); column<=static_cast<int>((point.x+radius)/resolution_); ++column) {
            if(!occupied(column,row)) continue;
            const double nearest_x=std::clamp(point.x,column*resolution_,(column+1)*resolution_);
            const double nearest_y=std::clamp(point.y,row*resolution_,(row+1)*resolution_);
            if(distance(point,{nearest_x,nearest_y})<=radius) return false;
        }
    return true;
}
World demonstration_world() {
    World world(30,20,0.3);
    for(int row=0;row<13;++row) world.occupy(13,row);
    for(int row=7;row<20;++row) world.occupy(22,row);
    return world;
}
bool World::segment_free(Point start,Point end,double radius) const {
    const double length=distance(start,end);
    const int intervals=std::max(1,static_cast<int>(std::ceil(length/(resolution_/20))));
    const double margin=length/intervals/2;
    for(int step=0;step<=intervals;++step) {
        const double fraction=static_cast<double>(step)/intervals;
        if(!free({start.x+fraction*(end.x-start.x),start.y+fraction*(end.y-start.y)},radius+margin)) return false;
    }
    return true;
}
}
