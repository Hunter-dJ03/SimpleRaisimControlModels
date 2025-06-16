#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class PendulumController : public rclcpp::Node
{
public:
  PendulumController() : Node("pendulum_controller")
  {
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        std::bind(&PendulumController::jointStateCallback, this, std::placeholders::_1));

    effort_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_effort_commands", 10);

    RCLCPP_INFO(this->get_logger(), "Pendulum Controller Node started");
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Example: apply negative proportional torque based on position
    std_msgs::msg::Float64MultiArray effort_msg;

    effort_msg = calculateGravityCompensator(msg);

    effort_pub_->publish(effort_msg);
  }

  std_msgs::msg::Float64MultiArray calculateGravityCompensator(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std_msgs::msg::Float64MultiArray gravity_effort;
    gravity_effort.data.resize(msg->position.size());

    gravity_effort.data[0] = link_mass[0] * gravity * link_centroid_length[0] * cos(msg->position[0]) + link_mass[1] * gravity * link_length[0] * cos(msg->position[0]) + link_mass[1] * gravity * link_centroid_length[1] * cos(msg->position[0] + msg->position[1]);

    gravity_effort.data[1] = link_mass[1] * gravity * link_centroid_length[1] * cos(msg->position[0] + msg->position[1]);

    return gravity_effort;
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_pub_;

  double link_mass[2] = {37.1262, 24.9644};
  double link_length[2] = {1.2, 0.8};
  double link_centroid_length[2] = {0.637977, 0.417115};
  double gravity = -9.81;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PendulumController>());
  rclcpp::shutdown();
  return 0;
}
