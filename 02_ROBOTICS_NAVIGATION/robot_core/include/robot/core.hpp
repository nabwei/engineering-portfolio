#pragma once
#include <array>
#include <vector>

namespace robot {
constexpr double pi = 3.14159265358979323846;
struct Point { double x{}, y{}; };
struct Pose { double x{}, y{}, heading{}; };
struct Twist { double speed{}, yaw_rate{}; };
struct Drive { double wheel_radius{0.05}, axle_length{0.3}; };
double wrap(double angle);
double distance(Point first, Point second);
Pose integrate(Pose pose, Twist command, double seconds);
Twist wheel_twist(Drive drive, double left, double right);

class World {
public:
    World(int width, int height, double resolution);
    void occupy(int column, int row);
    bool occupied(int column, int row) const;
    bool free(Point center, double radius) const;
    bool segment_free(Point start, Point end, double radius) const;
    Point center(int column, int row) const;
    int width() const { return width_; }
    int height() const { return height_; }
    double resolution() const { return resolution_; }
private:
    int width_, height_;
    double resolution_;
    std::vector<bool> occupied_;
};

struct Scan { double start_angle{-pi}, step{pi/90}, max_range{10}; int beams{181}; };
std::vector<double> lidar(const World& world, Pose pose, Scan scan);
std::vector<Point> astar(const World& world, Point start, Point goal, double radius);
Twist pursuit(Pose pose, Point target, double speed);
Point lookahead(const std::vector<Point>& path, Point position, double distance);

using Matrix3 = std::array<std::array<double,3>,3>;
class Ekf {
public:
    explicit Ekf(Pose initial);
    void predict(Twist command, double seconds);
    void observe_position(Point measurement, double variance);
    Pose pose() const { return pose_; }
    Matrix3 covariance() const { return covariance_; }
private:
    Pose pose_;
    Matrix3 covariance_{{{0.1,0,0},{0,0.1,0},{0,0,0.1}}};
};
World demonstration_world();
}
