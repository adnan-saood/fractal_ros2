// This line states something the code cannot show on its own: the solver assumes O2 at (0,0) and O4 at (r1,0)
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <cmath>
#include <string>
#include <vector>
#include <cstdint>

using std::string;

class FourBarJointStatePublisher : public rclcpp::Node
{
public:
  struct LinkageSolution
  {
    double coupler_angle;
    double rocker_angle;
  };

  FourBarJointStatePublisher()
  : Node("four_bar_joint_state_publisher")
  {
    this->declare_parameter<string>("input_joint_states_topic", "/four_bar/input_joint_states");
    this->declare_parameter<string>("output_joint_states_topic", "/joint_states");
    this->declare_parameter<string>("motor_joint_name", "L1_L2_joint");
    this->declare_parameter<string>("input_motor_joint_name", "L1_L2_joint");
    this->declare_parameter<string>("l3_joint_name", "L1_L3_joint");
    this->declare_parameter<string>("l4_joint_name", "L2_L4_joint");
    this->declare_parameter<double>("r1", 0.08);  // L1: motor pivot to L3 pivot
    this->declare_parameter<double>("r2", 0.035); // L2: motor pivot to L2-L4 pivot
    this->declare_parameter<double>("r3", 0.045); // L3: L1 pivot to L3-L4 pivot
    this->declare_parameter<double>("r4", 0.05);  // L4: L2 pivot to L3 pivot
    this->declare_parameter<double>("input_angle_offset", 0.0);
    this->declare_parameter<double>("motor_joint_direction", 1.0);
    this->declare_parameter<double>("l3_joint_direction", -1.0);
    this->declare_parameter<double>("l4_joint_direction", 1.0);
    this->declare_parameter<int64_t>("solution_sign", 1); // choose intersection branch: +1 or -1

    input_joint_states_topic_ = this->get_parameter("input_joint_states_topic").as_string();
    output_joint_states_topic_ = this->get_parameter("output_joint_states_topic").as_string();
    motor_joint_name_ = this->get_parameter("motor_joint_name").as_string();
    input_motor_joint_name_ = this->get_parameter("input_motor_joint_name").as_string();
    l3_joint_name_ = this->get_parameter("l3_joint_name").as_string();
    l4_joint_name_ = this->get_parameter("l4_joint_name").as_string();
    r1_ = this->get_parameter("r1").as_double();
    r2_ = this->get_parameter("r2").as_double();
    r3_ = this->get_parameter("r3").as_double();
    r4_ = this->get_parameter("r4").as_double();
    input_angle_offset_ = this->get_parameter("input_angle_offset").as_double();
    motor_joint_direction_ = this->get_parameter("motor_joint_direction").as_double();
    l3_joint_direction_ = this->get_parameter("l3_joint_direction").as_double();
    l4_joint_direction_ = this->get_parameter("l4_joint_direction").as_double();
    solution_sign_ = this->get_parameter("solution_sign").as_int();

    home_solution_valid_ = solve(input_angle_offset_, home_solution_);
    if (!home_solution_valid_)
    {
      RCLCPP_ERROR(this->get_logger(), "four-bar home configuration is unreachable");
    }

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      output_joint_states_topic_, 10);

    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      input_joint_states_topic_, 10,
      std::bind(&FourBarJointStatePublisher::joint_state_cb, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "four_bar_joint_state_publisher started (%s -> %s; input motor: %s, model motor: %s, l3: %s, l4: %s)",
      input_joint_states_topic_.c_str(), output_joint_states_topic_.c_str(),
      input_motor_joint_name_.c_str(), motor_joint_name_.c_str(), l3_joint_name_.c_str(),
      l4_joint_name_.c_str());
  }

private:
  bool solve(double theta2, LinkageSolution & solution) const
  {
    const double O4x = r1_;
    const double Px = r2_ * std::cos(theta2);
    const double Py = r2_ * std::sin(theta2);
    const double dx = O4x - Px;
    const double dy = -Py;
    const double distance = std::hypot(dx, dy);

    if (distance <= 1e-12 || distance > (r3_ + r4_) || distance < std::fabs(r3_ - r4_))
    {
      return false;
    }

    const double a = (r4_ * r4_ - r3_ * r3_ + distance * distance) / (2.0 * distance);
    const double h = std::sqrt(std::max(0.0, r4_ * r4_ - a * a));
    const double xm = Px + a * dx / distance;
    const double ym = Py + a * dy / distance;
    const double rx = -dy / distance;
    const double ry = dx / distance;
    const double Bx = xm + static_cast<double>(solution_sign_) * h * rx;
    const double By = ym + static_cast<double>(solution_sign_) * h * ry;

    solution.coupler_angle = std::atan2(By - Py, Bx - Px);
    solution.rocker_angle = std::atan2(By, Bx - O4x);
    return true;
  }

  void joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // find motor joint angle
    double motor_command = NAN;
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      if (msg->name[i] == input_motor_joint_name_ && i < msg->position.size())
      {
        motor_command = msg->position[i];
        break;
      }
    }
    if (!std::isfinite(motor_command)) { return; }

    const double theta2 = input_angle_offset_ + motor_joint_direction_ * motor_command;
    LinkageSolution solution;
    if (!home_solution_valid_ || !solve(theta2, solution))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "four-bar geometry: no solution for motor position %.6f", motor_command);
      return;
    }

    // Merge computed passive joints into a copy of the incoming joint state
    sensor_msgs::msg::JointState out = *msg;
    out.header.stamp = this->now();

    // Do not republish the raw hardware joint. This makes /joint_states safe as
    // both the input and output topic: the solver ignores its own messages.
    for (size_t i = out.name.size(); i-- > 0;) {
      if (out.name[i] == input_motor_joint_name_) {
        out.name.erase(out.name.begin() + static_cast<std::ptrdiff_t>(i));
        if (i < out.position.size()) out.position.erase(out.position.begin() + static_cast<std::ptrdiff_t>(i));
        if (i < out.velocity.size()) out.velocity.erase(out.velocity.begin() + static_cast<std::ptrdiff_t>(i));
        if (i < out.effort.size()) out.effort.erase(out.effort.begin() + static_cast<std::ptrdiff_t>(i));
      }
    }

    auto set_or_append = [&](const std::string & name, double value) {
      // try to find existing entry
      for (size_t i = 0; i < out.name.size(); ++i) {
        if (out.name[i] == name) {
          if (i >= out.position.size()) out.position.resize(i + 1, 0.0);
          out.position[i] = value;
          return;
        }
      }
      // append if not found
      out.name.push_back(name);
      if (out.position.size() < out.name.size()) out.position.push_back(value);
      else out.position[out.name.size() - 1] = value;
    };

    // The URDF origins define the assembled home pose, so all three joint states are zero at home.
    set_or_append(motor_joint_name_, motor_joint_direction_ * motor_command);
    set_or_append(
      l3_joint_name_,
      l3_joint_direction_ * (solution.rocker_angle - home_solution_.rocker_angle));
    set_or_append(
      l4_joint_name_,
      l4_joint_direction_ *
      ((solution.coupler_angle - theta2) -
      (home_solution_.coupler_angle - input_angle_offset_)));

    joint_state_pub_->publish(out);
  }

  string input_joint_states_topic_;
  string output_joint_states_topic_;
  string motor_joint_name_;
  string input_motor_joint_name_;
  string l3_joint_name_;
  string l4_joint_name_;
  double r1_, r2_, r3_, r4_;
  double input_angle_offset_;
  double motor_joint_direction_;
  double l3_joint_direction_;
  double l4_joint_direction_;
  int64_t solution_sign_;
  LinkageSolution home_solution_;
  bool home_solution_valid_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FourBarJointStatePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
