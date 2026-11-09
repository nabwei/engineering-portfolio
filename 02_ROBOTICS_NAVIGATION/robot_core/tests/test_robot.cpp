#include "robot/core.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
using namespace robot;
void require(bool condition,const std::string& message) {if(!condition) throw std::runtime_error(message);}
void close(double actual,double expected,double tolerance=1e-9) {require(std::abs(actual-expected)<tolerance,"numeric mismatch: "+std::to_string(actual)+" vs "+std::to_string(expected));}
void test_kinematics() {
    const auto straight=integrate({0,0,0},{2,0},0.5);close(straight.x,1);close(straight.y,0);
    const auto circle=integrate({0,0,0},{1,1},pi/2);close(circle.x,1);close(circle.y,1);close(circle.heading,pi/2);
    const auto spin=wheel_twist(Drive{},-2,2);close(spin.speed,0);close(spin.yaw_rate,2.0/3);
    const auto still=integrate({1,2,3},{1,2},0);close(still.x,1);close(still.heading,3);
    try {integrate({},{1,0},-1);require(false,"negative time accepted");} catch(const std::invalid_argument&) {}
    for(double invalid: {std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity(),-1.0,0.0}) {
        try {wheel_twist({invalid,0.3},1,1);require(false,"invalid wheel geometry accepted");} catch(const std::invalid_argument&) {}
    }
    try {wheel_twist({},std::numeric_limits<double>::infinity(),1);require(false,"infinite wheel speed accepted");} catch(const std::invalid_argument&) {}
}
void test_sensor_and_collision() {
    World world(10,10,1);world.occupy(5,5);
    require(!world.free({4.9,5.5},0.11),"disc overlap missed");
    require(world.free({4.8,5.5},0.1),"clear disc rejected");
    require(!world.segment_free({4,5.5},{7,5.5},0.1),"swept obstacle crossing missed");
    const auto ranges=lidar(world,{2.5,5.5,0},{0,1,10,1});close(ranges[0],2.5,0.021);
    const auto bounded=lidar(world,{2.5,5.5,0},{0,1,1,1});close(bounded[0],1);
}
void test_planning() {
    World world(8,8,1);
    for(int row=0;row<7;++row) world.occupy(3,row);
    const auto path=astar(world,{1.5,1.5},{6.5,1.5},0.1);
    require(!path.empty(),"detour not found");
    for(const auto point:path) require(world.free(point,0.1),"planned collision");
    world.occupy(3,7);require(astar(world,{1.5,1.5},{6.5,1.5},0.1).empty(),"unreachable goal accepted");
    require(astar(world,{3.5,2.5},{6.5,1.5},0.1).empty(),"occupied start accepted");
}
void test_filter() {
    Ekf filter({0,0,0});
    for(int step=0;step<100;++step) {filter.predict({1,0},0.1);filter.observe_position({(step+1)*0.1,0},0.01);}
    close(filter.pose().x,10,1e-8);close(filter.pose().y,0);
    const auto covariance=filter.covariance();
    for(int i=0;i<3;++i) {require(covariance[i][i]>0,"nonpositive variance");for(int j=0;j<3;++j) close(covariance[i][j],covariance[j][i]);}
    require(covariance[0][0]<0.01,"position uncertainty not reduced");
    Ekf displaced({1,1,0});displaced.observe_position({0,0},0.01);require(std::hypot(displaced.pose().x,displaced.pose().y)<0.15,"measurement not correcting state");
}
void test_sensor_dropout_and_control_boundaries() {
    Ekf filter({0,0,0});filter.observe_position({0,0},0.01);
    const double observed_variance=filter.covariance()[0][0];
    for(int step=0;step<50;++step) filter.predict({1,0.1},0.1);
    require(filter.covariance()[0][0]>observed_variance,"dropout did not increase uncertainty");
    const auto before=filter.pose();filter.observe_position({before.x+0.2,before.y},0.01);
    require(std::abs(filter.pose().x-(before.x+0.2))<0.2,"returning sensor not correcting drift");
    const auto behind=pursuit({0,0,0},{-1,0},0.2);close(behind.speed,0);require(std::abs(behind.yaw_rate)<=1.5,"turn limit violated");
    const auto at_goal=pursuit({1,2,0},{1,2},0.2);close(at_goal.speed,0);close(at_goal.yaw_rate,0);
}
int main() {test_kinematics();test_sensor_and_collision();test_planning();test_filter();test_sensor_dropout_and_control_boundaries();std::cout<<"PASS analytical kinematics, LiDAR, swept collision, detour/unreachable, EKF covariance/dropout and control boundaries\n";}
