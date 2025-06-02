#include "std_msgs/msg/float64_multi_array.hpp"
#include <chrono>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <stdexcept>

class TestDiffswerveSim : public rclcpp::Node
{
  public:
    TestDiffswerveSim() : Node("test_diff_swerve_sim") {
      RCLCPP_INFO(this->get_logger(), "\033[32mStarting Test Diffswerve Sim\033[0m");
      
      velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/velo_c/commands", rclcpp::QoS(1));
      command_listener_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/diff_command", rclcpp::QoS(1), 
      [this](std_msgs::msg::Float64MultiArray::UniquePtr msg)  {command_array_ = *msg; });

      kinematics_worker_ = this->create_wall_timer(std::chrono::milliseconds(10), 
      std::bind(&TestDiffswerveSim::kinematicsRoutine_, this));
    }

  private:
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_listener_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr kinematics_worker_;
    std_msgs::msg::Float64MultiArray command_array_;

    void kinematicsRoutine_();
};

void TestDiffswerveSim::kinematicsRoutine_()
{
  float gear_vel_a(0);
  float gear_vel_b(0);
  
  try {
    gear_vel_a = command_array_.data.at(0);
    gear_vel_b = command_array_.data.at(1);
  } catch (const std::out_of_range& e) {}

  std_msgs::msg::Float64MultiArray veloc_msg;
  veloc_msg.data.assign(8, 0);

  // Rotate velocity
  veloc_msg.data.at(0) = (gear_vel_a + gear_vel_b) / 2;
  // Spin velocity
  veloc_msg.data.at(1) = (gear_vel_a - gear_vel_b) / 2;

  velocity_publisher_->publish(veloc_msg);
}

int main (int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestDiffswerveSim>());
  rclcpp::shutdown();
  return 0;
}