#include "ros2_swervebot_sim_kinematics/differential_converter.hpp"

#include <memory>

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
}

void DifferentialConverter::converterRoutine_(const DifferentialSwerve& msg) {
    auto diffToCoax =  [](double gear_a, double gear_b) 
        -> std::pair<double, double> 
    {
        double rotate;
        double spin;

        rotate = (gear_a + gear_b) / 2.0;
        spin = (gear_a - gear_b) / 2.0;

        return {rotate, spin};
    };

    auto [fl_rot, fl_spin] = diffToCoax(
        msg.front_left.gear_a, msg.front_left.gear_b);
    auto [fr_rot, fr_spin] = diffToCoax(
        msg.front_right.gear_a, msg.front_right.gear_b);
    auto [rl_rot, rl_spin] = diffToCoax(
        msg.rear_left.gear_a, msg.rear_left.gear_b);
    auto [rr_rot, rr_spin] = diffToCoax(
        msg.rear_right.gear_a, msg.rear_right.gear_b);

    Float64MultiArray veloc_msg;
    veloc_msg.data.resize(8);

    veloc_msg.data[0] = fl_rot;
    veloc_msg.data[1] = fl_spin;
    veloc_msg.data[2] = fr_rot;
    veloc_msg.data[3] = fr_spin;
    veloc_msg.data[4] = rl_rot;
    veloc_msg.data[5] = rl_spin;
    veloc_msg.data[6] = rr_rot;
    veloc_msg.data[7] = rr_spin;

    velocity_publisher_->publish(veloc_msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DifferentialConverter>());
    rclcpp::shutdown();
    return 0;
}