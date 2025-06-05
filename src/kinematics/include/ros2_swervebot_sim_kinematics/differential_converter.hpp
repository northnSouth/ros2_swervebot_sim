#ifndef ROS2_SWERVEBOT_SIM_KINEMATICS__DIFFERENTIAL_CONVERTER_HPP_
#define ROS2_SWERVEBOT_SIM_KINEMATICS__DIFFERENTIAL_CONVERTER_HPP_

#include "ros2_swervebot_sim_kinematics/common.hpp"

class DifferentialConverter : public rclcpp::Node {
public:
    DifferentialConverter();

private:
    rclcpp::Publisher<Float64MultiArray>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<DifferentialSwerve>::SharedPtr differential_listener_;

    void converterRoutine_(const DifferentialSwerve msg);
};

#endif