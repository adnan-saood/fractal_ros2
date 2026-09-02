#include <cmath>
#include <limits>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class JointStateSliderCommandBridge : public rclcpp::Node
{
public:
  JointStateSliderCommandBridge() : Node("joint_state_slider_command_bridge")
  {
    joint_name_ = declare_parameter<std::string>("joint_name", "motor_joint");
    minimum_position_ = declare_parameter<double>("minimum_position", -6.283185307);
    maximum_position_ = declare_parameter<double>("maximum_position", 6.283185307);
    publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_position_controller/commands", 1);
    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 1,
      std::bind(&JointStateSliderCommandBridge::slider_callback, this, std::placeholders::_1));
  }

private:
  void slider_callback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    for (size_t index = 0; index < message->name.size(); ++index)
    {
      if (message->name[index] != joint_name_ || index >= message->position.size()) { continue; }
      const double position = message->position[index];
      if (!std::isfinite(position) || position < minimum_position_ || position > maximum_position_)
      {
        RCLCPP_ERROR(
          get_logger(), "Rejected slider command %.6f rad; allowed range is [%.6f, %.6f] rad.",
          position, minimum_position_, maximum_position_);
        return;
      }
      std_msgs::msg::Float64MultiArray command;
      command.data = {position};
      publisher_->publish(command);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000, "Accepted motor angle command: %.6f rad", position);
      return;
    }
  }

  std::string joint_name_;
  double minimum_position_{0.0};
  double maximum_position_{0.0};
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateSliderCommandBridge>());
  rclcpp::shutdown();
  return 0;
}