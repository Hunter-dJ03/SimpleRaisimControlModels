#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "quadruped_leg_interfaces/msg/endpoint.hpp"
#include <Eigen/Dense>
#include <Eigen/QR>

class QuadrupedLegController : public rclcpp::Node
{
public:
  QuadrupedLegController() : Node("quadruped_leg_controller")
  {
    // Setup ROS2 parameter time step for simulation, timers and models
    time_step_ms = this->declare_parameter<float>("time_step_ms", 1.0);
    init_pos = this->declare_parameter<std::vector<double>>("joint_initial_positions", std::vector<double>{});
    link_lengths = this->declare_parameter<std::vector<double>>("link_lengths", std::vector<double>{});

    legJointPosition = Eigen::Map<Eigen::VectorXd>(init_pos.data(), init_pos.size());
    legJointVelocity = Eigen::VectorXd::Zero(3);

    footPositionActual = forwardKinematics(init_pos[0], init_pos[1], init_pos[2]);
    footPosition = footPositionActual; // Initialize foot position to actual position

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        std::bind(&QuadrupedLegController::jointStateCallback, this, std::placeholders::_1));

    desired_control_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_desired_control", 10);

    endpoint_publisher_ = this->create_publisher<quadruped_leg_interfaces::msg::Endpoint>("endpoint", 10);

    

    RCLCPP_INFO(this->get_logger(), "Pendulum Controller Node started");
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    

    sensor_msgs::msg::JointState control_effort;
    control_effort.header.stamp = now();
    control_effort.name.resize(dof);
    control_effort.position.resize(dof);
    control_effort.velocity.resize(dof);

    quadruped_leg_interfaces::msg::Endpoint endpoint_msg;
    endpoint_msg.header.stamp = now();

    /******************   World Control   *****************/

    double vel_x = A0 * sin(omega0 * time); // Desired velocity in x direction
    double vel_y = A1 * cos(omega1 * time); // Desired velocity in y direction
    double vel_z = A2 * sin(omega2 * time); // Desired velocity in z direction
    // double vel_roll = 0.0; // Desired roll velocity
    // double vel_pitch = 0.0; // Desired pitch velocity
    // double vel_yaw = 0.0; // Desired yaw velocity

    // Eigen::VectorXd desired_velocity(6);
    // desired_velocity << vel_x, vel_y, vel_z, vel_roll, vel_pitch, vel_yaw;

    Eigen::VectorXd desired_velocity(3);
    desired_velocity << vel_x, vel_y, vel_z; // Only considering 3D Cartesian space for the leg

    footPositionActual = forwardKinematics(msg->position[0], msg->position[1], msg->position[2]);

    footPosition += desired_velocity * time_step_ms / 1000.0;

    // Cartesian position error
    Eigen::VectorXd position_error = footPosition - footPositionActual;

    // PD control in Cartesian space (only P for now)
    
    Eigen::VectorXd corrected_velocity = Kp_cartesian * position_error + desired_velocity;

    Eigen::MatrixXd jacobian = computeJacobian(msg->position[0], msg->position[1], msg->position[2]);
    auto jacobianPseudoInverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::VectorXd legJointVelocity = jacobianPseudoInverse * corrected_velocity;

    for (int i = 0; i < dof; ++i)
    {
      legJointPosition[i] += legJointVelocity[i] * time_step_ms / 1000.0;
      control_effort.position[i] = legJointPosition[i];
      control_effort.velocity[i] = legJointVelocity[i];
    }

    
    /******************   Joint Control   *****************/
    /*  // Offset cosine waveform
        control_effort.position[0] = A0 * (1.0 - cos(omega0 * time));
        control_effort.position[1] = A1 * (1.0 - cos(omega1 * time));
        control_effort.position[2] = A2 * (1.0 - cos(omega2 * time));

        control_effort.velocity[0] = A0 * omega0 * sin(omega0 * time);
        control_effort.velocity[1] = A1 * omega1 * sin(omega1 * time);
        control_effort.velocity[2] = A2 * omega2 * sin(omega2 * time);

        // Using sine wave for position and velocity control
        control_effort.position[0] = A0 * (sin(omega0 * time)) + init_pos[0];
        control_effort.position[1] = -A1 * (sin(omega1 * time)) + init_pos[1];
        control_effort.position[2] = -A2 * (sin(omega2 * time)) + init_pos[2];

        control_effort.velocity[0] = A0 * omega0 * cos(omega0 * time);
        control_effort.velocity[1] = -A1 * omega1 * cos(omega1 * time);
        control_effort.velocity[2] = -A2 * omega2 * cos(omega2 * time);

        // Holding Position
        control_effort.position[0] = init_pos[0];
        control_effort.position[1] = init_pos[1];
        control_effort.position[2] = init_pos[2];

        control_effort.velocity[0] = 0;
        control_effort.velocity[1] = 0;
        control_effort.velocity[2] = 0; */

    desired_control_pub_->publish(control_effort);

    time += time_step_ms / 1000; // increment time by 1 ms per callback


    // Fill in desired position
    endpoint_msg.desired.x = footPosition.x();
    endpoint_msg.desired.y = footPosition.y();
    endpoint_msg.desired.z = footPosition.z();

    // Fill in actual position
    endpoint_msg.actual.x = footPositionActual.x();
    endpoint_msg.actual.y = footPositionActual.y();
    endpoint_msg.actual.z = footPositionActual.z();

    // Fill in error (desired - actual)
    endpoint_msg.error.x = position_error.x();
    endpoint_msg.error.y = position_error.y();
    endpoint_msg.error.z = position_error.z();

    
    endpoint_publisher_->publish(endpoint_msg);
  }

  Eigen::MatrixXd computeJacobian(double theta1, double theta2, double theta3)
  {
    // double s1 = std::sin(theta1);
    // double c1 = std::cos(theta1);
    // double s2 = std::sin(theta2);
    // double c2 = std::cos(theta2);
    // double s3 = std::sin(theta3);
    // double c3 = std::cos(theta3);
    // double s23 = std::sin(theta2 + theta3);
    // double c23 = std::cos(theta2 + theta3);

    // // Link lengths
    double l1 = link_lengths[0];
    double l2 = link_lengths[1];
    double l3 = link_lengths[2];

    // // σ terms
    // double sigma1 = l3 * c23 + l2 * c2;
    // double sigma2 = l3 * s23;

    Eigen::MatrixXd J(3, 3);
    J << -l1 * std::sin(theta1) - l2 * std::cos(theta1) * std::sin(theta2) - l3 * std::cos(theta1) * std::cos(theta2) * std::sin(theta3) - l3 * std::cos(theta1) * std::cos(theta3) * std::sin(theta2),
        -std::sin(theta1) * (l3 * std::cos(theta2 + theta3) + l2 * std::cos(theta2)),
        -l3 * std::cos(theta2 + theta3) * std::sin(theta1),
        0,
        l3 * std::sin(theta2 + theta3) + l2 * std::sin(theta2),
        l3 * std::sin(theta2 + theta3),
        l2 * std::sin(theta1) * std::sin(theta2) - l1 * std::cos(theta1) + l3 * std::cos(theta2) * std::sin(theta1) * std::sin(theta3) + l3 * std::cos(theta3) * std::sin(theta1) * std::sin(theta2),
        -std::cos(theta1) * (l3 * std::cos(theta2 + theta3) + l2 * std::cos(theta2)),
        -l3 * std::cos(theta2 + theta3) * std::cos(theta1);
        // 0,
        // std::cos(theta1),
        // std::cos(theta1),
        // 1,
        // 0,
        // 0,
        // 0,
        // -std::sin(theta1),
        // -std::sin(theta1);

    return J;
  }

  Eigen::VectorXd forwardKinematics(double theta1, double theta2, double theta3)
{
    // Precompute useful terms
    // float s1 = std::sin(theta1), c1 = std::cos(theta1);
    // float s2 = std::sin(theta2), c2 = std::cos(theta2);
    // float s3 = std::sin(theta3), c3 = std::cos(theta3);
    // float s23 = std::sin(theta2 + theta3), c23 = std::cos(theta2 + theta3);

    // // Link lengths
    double l1 = link_lengths[0];
    double l2 = link_lengths[1];
    double l3 = link_lengths[2];

    // Build the transformation matrix
    Eigen::Matrix4d fk;
    fk << -std::sin(theta2 + theta3)*std::sin(theta1),
        -std::cos(theta2 + theta3)*std::sin(theta1),
        std::cos(theta1), 
        l1*std::cos(theta1) - l2*std::sin(theta1)*std::sin(theta2) - l3*std::cos(theta2)*std::sin(theta1)*std::sin(theta3) - l3*std::cos(theta3)*std::sin(theta1)*std::sin(theta2),
        -std::cos(theta2 + theta3),
        std::sin(theta2 + theta3),
        0,
        - l3*std::cos(theta2 + theta3) - l2*std::cos(theta2),
        -std::sin(theta2 + theta3)*std::cos(theta1), 
        -std::cos(theta2 + theta3)*std::cos(theta1), 
        -std::sin(theta1), 
        - l1*std::sin(theta1) - l2*std::cos(theta1)*std::sin(theta2) - l3*std::cos(theta1)*std::cos(theta2)*std::sin(theta3) - l3*std::cos(theta1)*std::cos(theta3)*std::sin(theta2),
        0, 
        0,
        0, 
        1;

    // Extract and return the position (4th column, top 3 rows)
    return fk.block<3, 1>(0, 3);  // XYZ position
}


  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desired_control_pub_;
  rclcpp::Publisher<quadruped_leg_interfaces::msg::Endpoint>::SharedPtr endpoint_publisher_; 

  std::vector<double> init_pos;
  std::vector<double> link_lengths;

  Eigen::VectorXd legJointPosition;
  Eigen::VectorXd legJointVelocity;

  Eigen::VectorXd footPosition;
  Eigen::VectorXd footPositionActual;

  double gravity = -9.81;
  float time_step_ms;
  float time = 0;
  int dof = 3;

  // Waveform A parameters
  double A0 = 0.2;   // amplitude
  double period0 = 3.0; // period in seconds
  double omega0 = 2.0 * M_PI / period0;

  // Waveform B parameters
  double A1 = -0.5;   // amplitude
  double period1 = 3.0; // period in seconds
  double omega1 = 2.0 * M_PI / period1;

  // Waveform C parameters
  double A2 = 0.0;        // amplitude
  double period2 = 3.0; // period in seconds
  double omega2 = 2.0 * M_PI / period2;

  double Kp_cartesian = 10.0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuadrupedLegController>());
  rclcpp::shutdown();
  return 0;
}
