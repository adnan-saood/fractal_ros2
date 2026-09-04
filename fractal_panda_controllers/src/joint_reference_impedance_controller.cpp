#include "fractal_panda_controllers/joint_reference_impedance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace fractal_panda_controllers
{

controller_interface::CallbackReturn JointReferenceImpedanceController::on_init()
{
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<std::vector<double>>("stiffness", {24.0, 24.0, 24.0, 24.0, 10.0, 6.0, 2.0});
    auto_declare<std::vector<double>>("damping", {2.0, 2.0, 2.0, 1.0, 1.0, 1.0, 0.5});
    auto_declare<std::vector<double>>("lower_limits", {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973});
    auto_declare<std::vector<double>>("upper_limits", {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973});
    auto_declare<double>("hard_force_limit", 5.0);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Controller initialization failed: %s", error.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointReferenceImpedanceController::on_configure(
  const rclcpp_lifecycle::State &)
{
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  const auto stiffness = get_node()->get_parameter("stiffness").as_double_array();
  const auto damping = get_node()->get_parameter("damping").as_double_array();
  const auto lower_limits = get_node()->get_parameter("lower_limits").as_double_array();
  const auto upper_limits = get_node()->get_parameter("upper_limits").as_double_array();
    hard_force_limit_ = get_node()->get_parameter("hard_force_limit").as_double();
  if (stiffness.size() != kJointCount || damping.size() != kJointCount ||
      lower_limits.size() != kJointCount || upper_limits.size() != kJointCount || hard_force_limit_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "All impedance and joint-limit parameters must contain seven values.");
    return CallbackReturn::ERROR;
  }
  for (std::size_t index = 0; index < kJointCount; ++index) {
    if (!std::isfinite(stiffness[index]) || !std::isfinite(damping[index]) ||
        !std::isfinite(lower_limits[index]) || !std::isfinite(upper_limits[index]) ||
        stiffness[index] < 0.0 || damping[index] < 0.0 || lower_limits[index] >= upper_limits[index]) {
      RCLCPP_ERROR(get_node()->get_logger(), "Invalid impedance or joint limit at index %zu.", index);
      return CallbackReturn::ERROR;
    }
    stiffness_[index] = stiffness[index];
    damping_[index] = damping[index];
    lower_limits_[index] = lower_limits[index];
    upper_limits_[index] = upper_limits[index];
  }
  joint_names_.clear();
  for (std::size_t index = 1; index <= kJointCount; ++index) {
    joint_names_.push_back(arm_id_ + "_joint" + std::to_string(index));
  }
  reference_subscription_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "~/reference", rclcpp::SystemDefaultsQoS(),
    std::bind(&JointReferenceImpedanceController::reference_callback, this, std::placeholders::_1));
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointReferenceImpedanceController::on_activate(
  const rclcpp_lifecycle::State &)
{
  auto initial_reference = std::make_shared<JointArray>();
  for (std::size_t index = 0; index < kJointCount; ++index) {
    desired_position_[index] = state_interfaces_[2U * index].get_value();
    filtered_velocity_[index] = 0.0;
    (*initial_reference)[index] = desired_position_[index];
  }
  const auto robot_state_interface = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
    [this](const auto & interface) { return interface.get_name() == arm_id_ + "/robot_state"; });
  if (robot_state_interface == state_interfaces_.end()) return CallbackReturn::ERROR;
  const double encoded_pointer = robot_state_interface->get_value();
  static_assert(sizeof(robot_state_) == sizeof(encoded_pointer), "Franka state pointer must fit in state interface");
  std::memcpy(&robot_state_, &encoded_pointer, sizeof(robot_state_));
  if (robot_state_ == nullptr) return CallbackReturn::ERROR;
  reference_buffer_.writeFromNonRT(initial_reference);
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
JointReferenceImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint_name : joint_names_) {
    configuration.names.push_back(joint_name + "/effort");
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
JointReferenceImpedanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint_name : joint_names_) {
    configuration.names.push_back(joint_name + "/position");
    configuration.names.push_back(joint_name + "/velocity");
  }
  configuration.names.push_back(arm_id_ + "/robot_state");
  return configuration;
}

controller_interface::return_type JointReferenceImpedanceController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const auto reference = reference_buffer_.readFromRT();
  if (reference && *reference) {
    desired_position_ = **reference;
  }
  const auto & wrench = robot_state_->K_F_ext_hat_K;
  const double force_norm = std::sqrt(wrench[0] * wrench[0] + wrench[1] * wrench[1] + wrench[2] * wrench[2]);
  if (!std::isfinite(force_norm) || force_norm >= hard_force_limit_) {
    for (auto & command_interface : command_interfaces_) command_interface.set_value(0.0);
    return controller_interface::return_type::ERROR;
  }
  constexpr double velocity_filter = 0.99;
  for (std::size_t index = 0; index < kJointCount; ++index) {
    const double position = state_interfaces_[2U * index].get_value();
    const double velocity = state_interfaces_[2U * index + 1U].get_value();
    filtered_velocity_[index] = (1.0 - velocity_filter) * filtered_velocity_[index] + velocity_filter * velocity;
    const double commanded_effort = stiffness_[index] * (desired_position_[index] - position) -
      damping_[index] * filtered_velocity_[index];
    command_interfaces_[index].set_value(commanded_effort);
  }
  return controller_interface::return_type::OK;
}

void JointReferenceImpedanceController::reference_callback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr message)
{
  if (!valid_reference(*message)) {
    RCLCPP_WARN(get_node()->get_logger(), "Rejected unsafe or malformed joint reference.");
    return;
  }
  const auto & point = message->points.back();
  auto reference = std::make_shared<JointArray>();
  for (std::size_t index = 0; index < kJointCount; ++index) {
    (*reference)[index] = point.positions[index];
  }
  reference_buffer_.writeFromNonRT(std::move(reference));
}

bool JointReferenceImpedanceController::valid_reference(
  const trajectory_msgs::msg::JointTrajectory & message) const
{
  if (message.joint_names != joint_names_ || message.points.empty()) {
    return false;
  }
  const auto & point = message.points.back();
  if (point.positions.size() != kJointCount) {
    return false;
  }
  for (std::size_t index = 0; index < kJointCount; ++index) {
    if (!std::isfinite(point.positions[index]) || point.positions[index] < lower_limits_[index] ||
        point.positions[index] > upper_limits_[index]) {
      return false;
    }
  }
  return true;
}

}  // namespace fractal_panda_controllers

PLUGINLIB_EXPORT_CLASS(fractal_panda_controllers::JointReferenceImpedanceController,
  controller_interface::ControllerInterface)