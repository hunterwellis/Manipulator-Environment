#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <string>
#include <cmath>
#include <memory>
#include <chrono>

class JointPublisher : public rclcpp::Node
{
public:
    JointPublisher() : Node("joint_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller_joint_trajectory", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&JointPublisher::timer_callback, this));
    }
private:
    void timer_callback()
    {
        auto message = trajectory_msgs::msg::JointTrajectory();

        message.joint_names.push_back("J1");
        message.joint_names.push_back("J2");
        message.joint_names.push_back("J3");
        message.joint_names.push_back("J4");
        message.joint_names.push_back("J5");

        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        double position1 = 1.5*(1-cos(count_*0.1));
        double position2 = -position1;
        double position3 = -position2;
        double position4 = -position3;
        double position5 = -position4;
        point.positions.push_back(position1);
        point.positions.push_back(position2);
        point.positions.push_back(position3);
        point.positions.push_back(position4);
        point.positions.push_back(position5);
        point.time_from_start = rclcpp::Duration::from_seconds(1.0);
        message.points.push_back(point);
        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), 
            "Publishing: '%f', '%f', '%f', '%f', '%f'", 
            position1,
            position2, 
            position3, 
            position4, 
            position5);

        count_ += 1;
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointPublisher>());
    rclcpp::shutdown();
    return 0;
}