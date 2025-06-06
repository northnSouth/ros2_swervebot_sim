#ifndef ROS2_SWERVEBOT_SIM_KINEMATICS__COMMON_HPP_
#define ROS2_SWERVEBOT_SIM_KINEMATICS__COMMON_HPP_

/**
* The keep pragma is used for readability, clang deduce including rclcpp.hpp
* as useless and fire a warning, but it's not, because otherwise I had 
* to include every single headers I need that is included by rclcpp.hpp
*/
#include "rclcpp/rclcpp.hpp" // IWYU pragma: keep

#include "std_msgs/msg/float64_multi_array.hpp"
#include "ros2_swervebot_sim_msgs/msg/differential_swerve.hpp"

using Float64MultiArray = std_msgs::msg::Float64MultiArray;
using DifferentialSwerve = ros2_swervebot_sim_msgs::msg::DifferentialSwerve;

#endif