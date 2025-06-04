#ifndef ROS2_SWERVEBOT_SIM_KINEMATICS__KINEMATICS_WORKER_HPP_
#define ROS2_SWERVEBOT_SIM_KINEMATICS__KINEMATICS_WORKER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

using Float64MultiArray = std_msgs::msg::Float64MultiArray;
using Twist = geometry_msgs::msg::Twist;

class KinematicsWorker : public rclcpp::Node {
public:
    KinematicsWorker();

private:
    rclcpp::Publisher<Float64MultiArray>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<Twist>::SharedPtr command_listener_;
    rclcpp::TimerBase::SharedPtr kinematics_routine_;

    Float64MultiArray velocity_msg_;
    Twist command_msg_;

    void kinematicsRoutine_();
};

#endif