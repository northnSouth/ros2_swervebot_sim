#include "ros2_swervebot_sim_kinematics/kinematics_worker.hpp"

using time_ms = std::chrono::milliseconds;

namespace {
    constexpr const char* NODE_NAME = "swerve_kinematics_worker";
    constexpr const char* CMD_TOPIC = "/cmd_vel";
    constexpr const char* SWERVE_CONTROL_TOPIC = "/swerve_vel_control";
    constexpr time_ms LOOP_PERIOD_MS(10);
    const auto QOS = rclcpp::QoS(1);
}

KinematicsWorker::KinematicsWorker() 
: Node(NODE_NAME) {
    RCLCPP_INFO(this->get_logger(), 
        "\033[32mStarting Swerve Kinematics Worker\033[0m"
    );

    velocity_publisher_ = this->create_publisher<Float64MultiArray>(
        SWERVE_CONTROL_TOPIC,
        QOS
    );

    command_listener_ = this->create_subscription<Twist>(
        CMD_TOPIC, 
        QOS, 
        [this](Twist::UniquePtr msg){
            command_msg_ = *msg;
        }
    );

    kinematics_routine_ = this->create_wall_timer(
        LOOP_PERIOD_MS,
        std::bind(&KinematicsWorker::kinematicsRoutine_, this)
    );
}

void KinematicsWorker::kinematicsRoutine_() {
    
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinematicsWorker>());
    rclcpp::shutdown();
    return 0;
}