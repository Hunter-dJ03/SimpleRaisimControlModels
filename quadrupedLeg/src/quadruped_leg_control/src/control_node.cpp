#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class PendulumController : public rclcpp::Node
{
public:
  PendulumController() : Node("pendulum_controller")
  {
    // Setup ROS2 parameter time step for simulation, timers and models
    time_step_ms = this->declare_parameter<float>("time_step_ms", 1.0);
    init_pos = this->declare_parameter<std::vector<double>>("joint_initial_positions", std::vector<double>{});
    // this->get_parameter("joint_initial_positions", init_pos);

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

    // Offset cosine waveform
    // Joint 0 parameters
    double A0 = M_PI_4;         // amplitude for joint 0
    double period0 = 3.0;       // period for joint 0 in seconds
    double omega0 = 2.0 * M_PI / period0;

    // Joint 1 parameters
    double A1 = M_PI_4;         // amplitude for joint 1
    double period1 = 3.0;       // period for joint 1 in seconds
    double omega1 = 2.0 * M_PI / period1;

    // Joint 3 parameters
    double A2 = M_PI_4;         // amplitude for joint 1
    double period2 = 3.0;       // period for joint 1 in seconds
    double omega2 = 2.0 * M_PI / period2;

    // control_effort.position[0] = A0 * (1.0 - cos(omega0 * time_s));
    // control_effort.position[1] = A1 * (1.0 - cos(omega1 * time_s));
    // control_effort.position[2] = A2 * (1.0 - cos(omega2 * time_s));

    // control_effort.velocity[0] = A0 * omega0 * sin(omega0 * time_s);
    // control_effort.velocity[1] = A1 * omega1 * sin(omega1 * time_s);
    // control_effort.velocity[2] = A2 * omega2 * sin(omega2 * time_s);

    control_effort.position[0] = A0 * (sin(omega0 * time)) + init_pos[0];
    control_effort.position[1] = -A1 * (sin(omega1 * time)) + init_pos[1];
    control_effort.position[2] = -A2 * (sin(omega2 * time)) + init_pos[2];

    control_effort.velocity[0] = A0 * omega0 * cos(omega0 * time);
    control_effort.velocity[1] = -A1 * omega1 * cos(omega1 * time);
    control_effort.velocity[2] = -A2 * omega2 * cos(omega2 * time);

    // control_effort.position[0] = 0;
    // control_effort.position[1] = -0.85148;
    // control_effort.position[2] = -1.45637;

    // control_effort.velocity[0] = 0;
    // control_effort.velocity[1] = 0;
    // control_effort.velocity[2] = 0;

    time += time_step_ms / 1000; // increment time by 1 ms per callback

    desired_control_pub_->publish(control_effort);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desired_control_pub_;

  std::vector<double> init_pos;

  double gravity = -9.81;
  float time_step_ms;
  float time = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PendulumController>());
  rclcpp::shutdown();
  return 0;
}
