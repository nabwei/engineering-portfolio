#include "robot/core.hpp"
#include <cmath>
#include <stdexcept>
namespace robot {
namespace {
Matrix3 multiply(Matrix3 left,Matrix3 right) {
    Matrix3 product{};
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) for(int k=0;k<3;++k) product[i][j]+=left[i][k]*right[k][j];
    return product;
}
Matrix3 transpose(Matrix3 matrix) {
    Matrix3 transposed{};
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) transposed[i][j]=matrix[j][i];
    return transposed;
}
}
Ekf::Ekf(Pose initial):pose_(initial) {}
void Ekf::predict(Twist command,double seconds) {
    const Pose predicted=integrate(pose_,command,seconds);
    Matrix3 jacobian{{{1,0,-(predicted.y-pose_.y)},{0,1,predicted.x-pose_.x},{0,0,1}}};
    covariance_=multiply(multiply(jacobian,covariance_),transpose(jacobian));
    for(int axis=0;axis<3;++axis) covariance_[axis][axis]+=seconds*0.002;
    pose_=predicted;
}
void Ekf::observe_position(Point measurement,double variance) {
    if(!std::isfinite(variance) || variance<=0 || !std::isfinite(measurement.x) || !std::isfinite(measurement.y)) throw std::invalid_argument("finite measurement and positive variance required");
    const double a=covariance_[0][0]+variance,b=covariance_[0][1],c=covariance_[1][1]+variance;
    const double determinant=a*c-b*b;
    std::array<std::array<double,2>,3> gain{};
    for(int i=0;i<3;++i) gain[i]={(covariance_[i][0]*c-covariance_[i][1]*b)/determinant,(-covariance_[i][0]*b+covariance_[i][1]*a)/determinant};
    const Point innovation{measurement.x-pose_.x,measurement.y-pose_.y};
    pose_.x+=gain[0][0]*innovation.x+gain[0][1]*innovation.y;
    pose_.y+=gain[1][0]*innovation.x+gain[1][1]*innovation.y;
    pose_.heading=wrap(pose_.heading+gain[2][0]*innovation.x+gain[2][1]*innovation.y);
    Matrix3 residual{{{1,0,0},{0,1,0},{0,0,1}}};
    for(int i=0;i<3;++i) for(int j=0;j<2;++j) residual[i][j]-=gain[i][j];
    covariance_=multiply(multiply(residual,covariance_),transpose(residual));
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) covariance_[i][j]+=variance*(gain[i][0]*gain[j][0]+gain[i][1]*gain[j][1]);
}
}
