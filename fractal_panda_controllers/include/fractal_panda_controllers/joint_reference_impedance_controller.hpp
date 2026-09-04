#ifndef FRACTAL_PANDA_CONTROLLERS__JOINT_REFERENCE_IMPEDANCE_CONTROLLER_HPP_
#define FRACTAL_PANDA_CONTROLLERS__JOINT_REFERENCE_IMPEDANCE_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "franka/robot_state.h"
#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace fractal_panda_controllers
{

class JointReferenceImpedanceController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr std::size_t kJointCount = 7U;
  using JointArray = std::array<double, kJointCount>;

  void reference_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr message);
  bool valid_reference(const trajectory_msgs::msg::JointTrajectory & message) const;

  std::string arm_id_;
  std::vector<std::string> joint_names_;
  JointArray stiffness_{};
  JointArray damping_{};
  JointArray lower_limits_{};
  JointArray upper_limits_{};
  JointArray desired_position_{};
  JointArray filtered_velocity_{};
  double hard_force_limit_{5.0};
  franka::RobotState * robot_state_{nullptr};
  realtime_tools::RealtimeBuffer<std::shared_ptr<JointArray>> reference_buffer_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr reference_subscription_;
};

}  // namespace fractal_panda_controllers

#endif  // FRACTAL_PANDA_CONTROLLERS__JOINT_REFERENCE_IMPEDANCE_CONTROLLER_HPP_