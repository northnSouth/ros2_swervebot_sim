#ifndef ROS2_SWERVEBOT_SIM_KINEMATICS__KINEMATICS_WORKER_HPP_
#define ROS2_SWERVEBOT_SIM_KINEMATICS__KINEMATICS_WORKER_HPP_

/**
* The comment is used for readability, clang deduce including rclcpp.hpp
* as useless and fire a warning, but it's not, because otherwise I had 
* to include every single headers I need that is included by rclcpp.hpp
*/
#include "rclcpp/rclcpp.hpp" // IWYU pragma: keep

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

    Twist command_msg_;

    void kinematicsRoutine_();
};

#endif