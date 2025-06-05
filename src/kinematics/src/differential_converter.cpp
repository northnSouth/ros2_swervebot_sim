#include "ros2_swervebot_sim_kinematics/differential_converter.hpp"

#include <memory>
#include <vector>

namespace {
    constexpr const char* NODE_NAME = "swerve_differential_converter";
    constexpr const char* VELOCITY_TOPIC = "/swerve_vel_control";
    constexpr const char* DIFFERENTIAL_TOPIC = "/swerve_diff_control";
    const auto QOS = rclcpp::QoS(1);
}

DifferentialConverter::DifferentialConverter()
: rclcpp::Node(NODE_NAME) {
    RCLCPP_INFO(this->get_logger(), 
        "\033[32mStarting Swerve Differential Converter\033[0m"
    );

    velocity_publisher_ = this->create_publisher<Float64MultiArray>(
        VELOCITY_TOPIC,
        QOS
    );

    differential_listener_ = this->create_subscription<DifferentialSwerve>(
        DIFFERENTIAL_TOPIC,
        QOS,
        [this](DifferentialSwerve::UniquePtr msg) {
            this->converterRoutine_(*msg);
        }
    );
};

void DifferentialConverter::converterRoutine_(const DifferentialSwerve msg) {
    RCLCPP_INFO(this->get_logger(), 
        "Got front left: %f", msg.front_left.gear_a
    );

    /**
    * Controller will fail when it receives incorrect array 
    * length, that is inequal to the number of joints under 
    * control, in this case 8. Refer to controller config.
    */
    Float64MultiArray veloc_msg;
    veloc_msg.data = std::vector<double>(8, 0.0);

    velocity_publisher_->publish(veloc_msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DifferentialConverter>());
    rclcpp::shutdown();
    return 0;
}