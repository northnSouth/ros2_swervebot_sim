#include "ros2_swervebot_sim_kinematics/differential_converter.hpp"

#include <memory>

namespace {
    constexpr const char* NODE_NAME = "swerve_differential_converter";
}

DifferentialConverter::DifferentialConverter()
: rclcpp::Node(NODE_NAME) {
    RCLCPP_INFO(this->get_logger(), 
        "\033[32mStarting Swerve Differential Converter\033[0m"
    );
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DifferentialConverter>());
    rclcpp::shutdown();
    return 0;
}