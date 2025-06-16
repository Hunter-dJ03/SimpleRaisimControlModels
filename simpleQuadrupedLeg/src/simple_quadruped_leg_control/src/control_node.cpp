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

    desired_control_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_desired_control", 10);

    RCLCPP_INFO(this->get_logger(), "Pendulum Controller Node started");
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {

    // Calculations for kinematics and dynamics
    // Output = desired position and velocity

    sensor_msgs::msg::JointState control_effort;
    int dof = msg->position.size();
    control_effort.header.stamp = now();
    control_effort.name.resize(dof);
    control_effort.position.resize(dof);
    control_effort.velocity.resize(dof);

    // Offset cosine waveform: pos = A * (1 - cos(omega * t)), vel = A * omega * sin(omega * t)
    // At t=0, pos=0, vel=0
    // Joint 0 parameters
    double A0 = M_PI_2;         // amplitude for joint 0
    double period0 = 2.0;       // period for joint 0 in seconds
    double omega0 = 2.0 * M_PI / period0;

    // Joint 1 parameters
    double A1 = M_PI_4;         // amplitude for joint 1
    double period1 = 3.0;       // period for joint 1 in seconds
    double omega1 = 2.0 * M_PI / period1;

    double time_s = static_cast<double>(t) * 0.001; // convert t from ms to seconds

    control_effort.position[0] = A0 * (1.0 - cos(omega0 * time_s));
    control_effort.position[1] = A1 * (1.0 - cos(omega1 * time_s));

    control_effort.velocity[0] = A0 * omega0 * sin(omega0 * time_s);
    control_effort.velocity[1] = A1 * omega1 * sin(omega1 * time_s);

    t += 1; // increment time by 1 ms per callback

    desired_control_pub_->publish(control_effort);
  }

  // std_msgs::msg::Float64MultiArray calculateGravityCompensator(const sensor_msgs::msg::JointState::SharedPtr msg)
  // {
  //   std_msgs::msg::Float64MultiArray gravity_effort;
  //   gravity_effort.data.resize(msg->position.size());

  //   gravity_effort.data[0] = link_mass[0] * gravity * link_centroid_length[0] * cos(msg->position[0]) + link_mass[1] * gravity * link_length[0] * cos(msg->position[0]) + link_mass[1] * gravity * link_centroid_length[1] * cos(msg->position[0] + msg->position[1]);

  //   gravity_effort.data[1] = link_mass[1] * gravity * link_centroid_length[1] * cos(msg->position[0] + msg->position[1]);

  //   return gravity_effort;
  // }

  std_msgs::msg::Float64MultiArray calculateSimplePDController(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std_msgs::msg::Float64MultiArray effot_msg;
    effot_msg.data.resize(msg->position.size());

    // Simple PD controller: P gain = 10, D gain = 0.1
    double p_gain = 8000.0;
    double d_gain = 2000.0;
    double desired_position[2] = {0.0, 0.0}; // Desired positions for the two joints
    double desired_velocity[2] = {0.0, 0.0}; // Desired velocities for the two joints
    for (size_t i = 0; i < msg->position.size(); ++i)
    {
      double position_error = desired_position[i] - msg->position[i];
      double velocity_error = desired_velocity[i] - msg->velocity[i];

      // PD control law
      effot_msg.data[i] = p_gain * position_error + d_gain * velocity_error;
    }

    return effot_msg;
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desired_control_pub_;

  double link_mass[2] = {37.1262, 24.9644};
  double link_length[2] = {1.2, 0.8};
  double link_centroid_length[2] = {0.637977, 0.417115};
  double gravity = -9.81;

  int t = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PendulumController>());
  rclcpp::shutdown();
  return 0;
}
