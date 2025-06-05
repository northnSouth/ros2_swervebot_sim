#ifndef ROS2_SWERVEBOT_SIM_KINEMATICS__KINEMATICS_WORKER_HPP_
#define ROS2_SWERVEBOT_SIM_KINEMATICS__KINEMATICS_WORKER_HPP_

#include "ros2_swervebot_sim_kinematics/common.hpp"
#include "geometry_msgs/msg/twist.hpp"

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