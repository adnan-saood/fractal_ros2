// Copyright (c) 2026, adnan-saood
// Copyright (c) 2026, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <sstream>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "fractal_pad_hardware/fractal_pad_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fractal_pad_hardware
{
namespace
{
speed_t to_baud_rate(const int baud_rate)
{
  switch (baud_rate)
  {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    default: return 0;
  }
}
}  // namespace

hardware_interface::CallbackReturn FractalPadHW::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 1U)
  {
    RCLCPP_ERROR(rclcpp::get_logger("FractalPadHW"), "Exactly one motor joint is required.");
    return CallbackReturn::ERROR;
  }
  const auto device_it = info_.hardware_parameters.find("device");
  if (device_it == info_.hardware_parameters.end() || device_it->second.empty())
  {
    RCLCPP_ERROR(rclcpp::get_logger("FractalPadHW"), "Missing required hardware parameter 'device'.");
    return CallbackReturn::ERROR;
  }

  device_ = device_it->second;
  if (const auto it = info_.hardware_parameters.find("baud_rate"); it != info_.hardware_parameters.end())
  {
    try { baud_rate_ = std::stoi(it->second); } catch (const std::exception &) { return CallbackReturn::ERROR; }
  }
  if (to_baud_rate(baud_rate_) == 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("FractalPadHW"), "Unsupported baud rate: %d", baud_rate_);
    return CallbackReturn::ERROR;
  }
  if (const auto it = info_.hardware_parameters.find("command_prefix"); it != info_.hardware_parameters.end())
  {
    command_prefix_ = it->second;
  }
  if (command_prefix_.empty()) { command_prefix_ = "M"; }

  hw_states_.assign(1U, 0.0);
  hw_velocities_.assign(1U, 0.0);
  hw_commands_.assign(1U, 0.0);
  hw_targets_.assign(1U, 0.0);
  hw_currents_q_.assign(1U, 0.0);
  hw_voltages_q_.assign(1U, 0.0);

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FractalPadHW::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return open_serial_port() ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

std::vector<hardware_interface::StateInterface> FractalPadHW::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "target_position", &hw_targets_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "current_q", &hw_currents_q_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "voltage_q", &hw_voltages_q_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FractalPadHW::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn FractalPadHW::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (serial_fd_ < 0 && !open_serial_port())
  {
    return CallbackReturn::ERROR;
  }
  read_telemetry();
  hw_commands_[0] = hw_states_[0];
  active_ = true;
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FractalPadHW::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  active_ = false;
  close_serial_port();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type FractalPadHW::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  read_telemetry();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FractalPadHW::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!active_ || serial_fd_ < 0 || !std::isfinite(hw_commands_[0]))
  {
    return hardware_interface::return_type::ERROR;
  }
  std::ostringstream command;
  command.precision(6);
  command << command_prefix_ << std::fixed << hw_commands_[0] << '\n';
  const auto command_text = command.str();
  const ssize_t sent = ::write(serial_fd_, command_text.data(), command_text.size());
  if (sent != static_cast<ssize_t>(command_text.size()))
  {
    RCLCPP_ERROR(rclcpp::get_logger("FractalPadHW"), "Serial write failed: %s", std::strerror(errno));
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

bool FractalPadHW::open_serial_port()
{
  close_serial_port();
  serial_fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("FractalPadHW"), "Unable to open %s: %s", device_.c_str(), std::strerror(errno));
    return false;
  }
  termios tty{};
  if (tcgetattr(serial_fd_, &tty) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("FractalPadHW"), "Unable to configure %s: %s", device_.c_str(), std::strerror(errno));
    close_serial_port();
    return false;
  }
  const speed_t speed = to_baud_rate(baud_rate_);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cflag = static_cast<tcflag_t>((tty.c_cflag & ~CSIZE) | CS8 | CLOCAL | CREAD);
  tty.c_iflag = 0;
  tty.c_oflag = 0;
  tty.c_lflag = 0;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("FractalPadHW"), "Unable to apply serial settings: %s", std::strerror(errno));
    close_serial_port();
    return false;
  }
  tcflush(serial_fd_, TCIOFLUSH);
  RCLCPP_INFO(rclcpp::get_logger("FractalPadHW"), "Connected to SimpleFOC controller on %s at %d baud", device_.c_str(), baud_rate_);
  return true;
}

void FractalPadHW::close_serial_port()
{
  if (serial_fd_ >= 0) { ::close(serial_fd_); serial_fd_ = -1; }
}

void FractalPadHW::read_telemetry()
{
  if (serial_fd_ < 0) { return; }
  char data[256];
  ssize_t bytes_read = 0;
  while ((bytes_read = ::read(serial_fd_, data, sizeof(data))) > 0)
  {
    receive_buffer_.append(data, static_cast<size_t>(bytes_read));
  }
  size_t start = receive_buffer_.find('<');
  while (start != std::string::npos)
  {
    const size_t end = receive_buffer_.find('>', start + 1U);
    if (end == std::string::npos) { break; }
    parse_telemetry_frame(receive_buffer_.substr(start + 1U, end - start - 1U));
    receive_buffer_.erase(0U, end + 1U);
    start = receive_buffer_.find('<');
  }
  if (receive_buffer_.size() > 1024U) { receive_buffer_.clear(); }
}

bool FractalPadHW::parse_telemetry_frame(const std::string & frame)
{
  // SimpleFOC monitor() emits enabled fields in its fixed implementation order:
  // <target,voltage_q,current_q_mA,velocity,shaft_angle>.
  std::istringstream stream(frame);
  std::string value;
  double target = 0.0;
  double voltage_q = 0.0;
  double current_q_milliamps = 0.0;
  double velocity = 0.0;
  double shaft_angle = 0.0;
  double * const fields[]{
    &target, &voltage_q, &current_q_milliamps, &velocity, &shaft_angle};
  for (const auto field : fields)
  {
    if (!std::getline(stream, value, ',')) { return false; }
    try { *field = std::stod(value); } catch (const std::exception &) { return false; }
  }

  hw_targets_[0] = target;
  hw_velocities_[0] = velocity;
  hw_states_[0] = shaft_angle;
  hw_currents_q_[0] = current_q_milliamps / 1000.0;
  hw_voltages_q_[0] = voltage_q;
  return true;
}

}  // namespace fractal_pad_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  fractal_pad_hardware::FractalPadHW, hardware_interface::SystemInterface)
