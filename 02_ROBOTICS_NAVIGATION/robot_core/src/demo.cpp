#include "robot/core.hpp"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <stdexcept>
int main(int argc,char** argv) {
    using namespace robot;
    const auto world=demonstration_world();
    const Point goal{8.25,4.95};
    Pose truth{0.75,0.75,0};
    Ekf filter(truth);
    const auto path=astar(world,{truth.x,truth.y},goal,0.25);
    if(path.empty()) throw std::runtime_error("planner found no path");
    std::ofstream trajectory(argc>1?argv[1]:"trajectory.csv");
    if(!trajectory) throw std::runtime_error("cannot open trajectory output");
    trajectory<<"time,x,y,heading,estimated_x,estimated_y,minimum_range\n"<<std::setprecision(12);
    std::mt19937 random(42);
    std::normal_distribution<double> noise(0,0.01);
    for(int step=0;step<12000;++step) {
        const auto command=pursuit(filter.pose(),lookahead(path,{filter.pose().x,filter.pose().y},0.28),0.28);
        const auto next=integrate(truth,command,0.02);
        if(!world.segment_free({truth.x,truth.y},{next.x,next.y},0.12)) throw std::runtime_error("collision detected; mission aborted");
        truth=next; filter.predict(command,0.02);
        if(step%5==0) filter.observe_position({truth.x+noise(random),truth.y+noise(random)},0.0001);
        if(step%10==0) {
            const auto ranges=lidar(world,truth,Scan{});
            trajectory<<step*0.02<<','<<truth.x<<','<<truth.y<<','<<truth.heading<<','<<filter.pose().x<<','<<filter.pose().y<<','<<*std::min_element(ranges.begin(),ranges.end())<<'\n';
        }
        if(distance({truth.x,truth.y},goal)<0.15) {std::cout<<"PASS mission: goal reached, no collisions, seconds="<<step*0.02<<'\n';return 0;}
    }
    throw std::runtime_error("mission timeout");
}
