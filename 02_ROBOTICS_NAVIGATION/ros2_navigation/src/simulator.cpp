#include "robot/core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <memory>

class Simulator final:public rclcpp::Node {
public:
    Simulator():Node("portfolio_sim"),world_(robot::demonstration_world()) {
        scan_=create_publisher<sensor_msgs::msg::LaserScan>("scan",rclcpp::SensorDataQoS());
        odometry_=create_publisher<nav_msgs::msg::Odometry>("odom",10);
        transform_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_transform_=std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
        geometry_msgs::msg::TransformStamped laser;
        laser.header.stamp=now();laser.header.frame_id="base_link";laser.child_frame_id="laser";laser.transform.rotation.w=1;
        static_transform_->sendTransform(laser);
        command_=create_subscription<geometry_msgs::msg::Twist>("cmd_vel",10,[this](const geometry_msgs::msg::Twist& message){
            if(!std::isfinite(message.linear.x)||!std::isfinite(message.angular.z)) return;
            twist_={std::clamp(message.linear.x,-0.4,0.4),std::clamp(message.angular.z,-1.5,1.5)};
            last_command_=now();
        });
        timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{tick();});
    }
private:
    void tick() {
        if((now()-last_command_).seconds()>0.5) twist_={};
        const auto candidate=robot::integrate(pose_,twist_,0.02);
        if(world_.segment_free({pose_.x,pose_.y},{candidate.x,candidate.y},0.12)) pose_=candidate;
        else {twist_={};RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),1000,"Collision guard stopped movement");}
        publish_odometry();
        if(++ticks_%5==0) publish_scan();
    }
    void publish_odometry() {
        nav_msgs::msg::Odometry message;
        message.header.stamp=now();message.header.frame_id="odom";message.child_frame_id="base_link";
        message.pose.pose.position.x=pose_.x;message.pose.pose.position.y=pose_.y;
        message.pose.pose.orientation.z=std::sin(pose_.heading/2);message.pose.pose.orientation.w=std::cos(pose_.heading/2);
        message.twist.twist.linear.x=twist_.speed;message.twist.twist.angular.z=twist_.yaw_rate;
        odometry_->publish(message);
        geometry_msgs::msg::TransformStamped transform;
        transform.header=message.header;transform.child_frame_id="base_link";
        transform.transform.translation.x=pose_.x;transform.transform.translation.y=pose_.y;
        transform.transform.rotation=message.pose.pose.orientation;
        transform_->sendTransform(transform);
    }
    void publish_scan() {
        const auto ranges=robot::lidar(world_,pose_,{-robot::pi,robot::pi/180,10,361});
        sensor_msgs::msg::LaserScan message;
        message.header.stamp=now();message.header.frame_id="laser";
        message.angle_min=-robot::pi;message.angle_max=robot::pi;message.angle_increment=robot::pi/180;
        message.range_min=0.02;message.range_max=10;message.scan_time=0.1;
        message.ranges.assign(ranges.begin(),ranges.end());scan_->publish(message);
    }
    robot::World world_;robot::Pose pose_{0.75,0.75,0};robot::Twist twist_{};
    rclcpp::Time last_command_{0,0,RCL_ROS_TIME};int ticks_{};
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> transform_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_;
};
int main(int argc,char** argv) {rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Simulator>());rclcpp::shutdown();}
