#include "robot/core.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
namespace robot {
std::vector<Point> astar(const World& world, Point start, Point goal, double radius) {
    if(!world.free(start,radius) || !world.free(goal,radius)) return {};
    const auto index=[&](Point point){return static_cast<int>(point.y/world.resolution())*world.width()+static_cast<int>(point.x/world.resolution());};
    const auto point=[&](int cell){return world.center(cell%world.width(),cell/world.width());};
    const int first=index(start), last=index(goal), count=world.width()*world.height();
    if(!world.segment_free(start,point(first),radius) || !world.segment_free(point(last),goal,radius)) return {};
    std::vector<double> cost(count,std::numeric_limits<double>::infinity());
    std::vector<int> parent(count,-1);
    using Candidate=std::pair<double,int>;
    std::priority_queue<Candidate,std::vector<Candidate>,std::greater<Candidate>> frontier;
    frontier.push({distance(point(first),point(last)),first}); cost[first]=0;
    while(!frontier.empty()) {
        const auto [priority,current]=frontier.top(); frontier.pop();
        if(priority>cost[current]+distance(point(current),point(last))+1e-10) continue;
        if(current==last) break;
        for(const auto& [dc,dr]:std::array<std::pair<int,int>,4>{{{1,0},{-1,0},{0,1},{0,-1}}}) {
            const int column=current%world.width()+dc,row=current/world.width()+dr;
            if(column<0 || row<0 || column>=world.width() || row>=world.height() || !world.free(world.center(column,row),radius)) continue;
            const int next=row*world.width()+column;
            if(!world.segment_free(point(current),point(next),radius)) continue;
            const double candidate=cost[current]+world.resolution();
            if(candidate>=cost[next]) continue;
            cost[next]=candidate; parent[next]=current;
            frontier.push({candidate+distance(point(next),point(last)),next});
        }
    }
    if(!std::isfinite(cost[last])) return {};
    std::vector<Point> path;
    for(int cell=last;cell!=-1;cell=parent[cell]) path.push_back(point(cell));
    std::reverse(path.begin(),path.end());
    path.insert(path.begin(),start); path.push_back(goal);
    return path;
}
}
