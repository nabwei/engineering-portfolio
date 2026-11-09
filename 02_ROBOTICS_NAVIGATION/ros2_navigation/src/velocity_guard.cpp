#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

class VelocityGuard final:public rclcpp::Node {
public:
    VelocityGuard():Node("velocity_guard") {
        output_=create_publisher<geometry_msgs::msg::Twist>("cmd_vel_safe",10);
        command_=create_subscription<geometry_msgs::msg::Twist>("cmd_vel",10,[this](const geometry_msgs::msg::Twist& command){
            requested_=geometry_msgs::msg::Twist{};
            if(std::isfinite(command.linear.x)&&std::isfinite(command.angular.z)) {
                requested_.linear.x=std::clamp(command.linear.x,-0.3,0.3);
                requested_.angular.z=std::clamp(command.angular.z,-1.2,1.2);
            }
            command_time_=now();seen_command_=true;
        });
        scan_=create_subscription<sensor_msgs::msg::LaserScan>("scan",rclcpp::SensorDataQoS(),[this](const sensor_msgs::msg::LaserScan& scan){
            if(!std::isfinite(scan.angle_min)||!std::isfinite(scan.angle_increment)||scan.angle_increment<=0||!std::isfinite(scan.range_min)||!std::isfinite(scan.range_max)||scan.range_min<0||scan.range_max<=scan.range_min) {seen_scan_=false;return;}
            front_clear_=true;bool valid_front=false;
            for(std::size_t index=0;index<scan.ranges.size();++index) {
                const double angle=scan.angle_min+index*scan.angle_increment;
                if(std::abs(angle)>0.65) continue;
                const double range=scan.ranges[index];
                if(std::isfinite(range)&&range>=scan.range_min&&range<=scan.range_max) {
                    valid_front=true;if(range<0.32) front_clear_=false;
                } else if(std::isinf(range)&&range>0) valid_front=true;
            }
            front_clear_=front_clear_&&valid_front;
            scan_time_=rclcpp::Time(scan.header.stamp);seen_scan_=true;
        });
        transforms_=create_subscription<tf2_msgs::msg::TFMessage>("tf",100,[this](const tf2_msgs::msg::TFMessage& message){
            for(const auto& transform:message.transforms) {
                if(transform.header.frame_id=="odom"&&transform.child_frame_id=="base_footprint") {
                    transform_time_=rclcpp::Time(transform.header.stamp);
                    seen_transform_=true;
                }
            }
        });
        timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{publish_safe_command();});
    }
private:
    void publish_safe_command() {
        auto safe=requested_;
        const double scan_age=(now()-scan_time_).seconds();
        const double transform_age=(now()-transform_time_).seconds();
        if(!seen_scan_||!seen_command_||!seen_transform_||scan_age<0||scan_age>0.3||transform_age<0||transform_age>0.3||(now()-command_time_).seconds()>0.5) safe=geometry_msgs::msg::Twist{};
        if(safe.linear.x>0&&!front_clear_) safe=geometry_msgs::msg::Twist{};
        output_->publish(safe);
    }
    geometry_msgs::msg::Twist requested_;
    rclcpp::Time scan_time_{0,0,RCL_ROS_TIME},command_time_{0,0,RCL_ROS_TIME},transform_time_{0,0,RCL_ROS_TIME};
    bool seen_scan_{false},seen_command_{false},front_clear_{false},seen_transform_{false};
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr transforms_;
    rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv) {rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<VelocityGuard>());rclcpp::shutdown();}
