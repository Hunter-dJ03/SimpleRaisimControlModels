#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <raisim/World.hpp>
#include <raisim/RaisimServer.hpp>

#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <random>
#include <cmath>

class RaisimBridge : public rclcpp::Node
{
public:
  RaisimBridge() : Node("raisim_bridge")
  {
    // Setup ROS2 parameter time step for simulation, timers and models
    float time_step_ms = this->declare_parameter<float>("time_step_ms", 1);

    // Set world timestep for simulation
    world.setTimeStep(time_step_ms / 1000.0f);
    // auto ground = world.addGround(-2);

    // Variable Gravity option
    // world.setGravity(Eigen::Vector3d(0, 0, -9.81));

    // Raisim Activation Key
    raisim::World::setActivationKey("$ENV{HOME}/.raisim");

    // Get robot URDF file path from description package
    std::string urdf_path_base = this->declare_parameter<std::string>("robot_description_path", "/default/path");
    std::string urdf_file = urdf_path_base + "/urdf/kohaku.urdf";

    // Load robot into RaiSim and give name
    robot = world.addArticulatedSystem(urdf_file);
    robot->setName("Kohaku");

    // Remove Collision Meshes
    for (int i = 0; i <= 8; ++i)
    {
      for (int j = i + 1; j <= 8; ++j)
      {
        robot->ignoreCollisionBetween(i, j);
      }
    }

    // Setup parameter sizes for generalised position, velocity, acceleration, force and damping and set to zero
    gc = Eigen::VectorXd::Zero(robot->getGeneralizedCoordinateDim());
    gv = Eigen::VectorXd::Zero(robot->getDOF());
    ga = Eigen::VectorXd::Zero(robot->getDOF());
    gf = Eigen::VectorXd::Zero(robot->getDOF());
    damping = Eigen::VectorXd::Zero(robot->getDOF());

    // Set siulation position and velocity
    robot->setGeneralizedCoordinate(gc);
    robot->setGeneralizedVelocity(gv);
    robot->setGeneralizedVelocity(gf);

    // Setup control mode for the simulation (May be wrong or unnecessary)
    robot->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);

    // Create Publisher for robot joint states
    joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Create subscription to control node topic for joint effort commands
    effort_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "joint_effort_commands", 10,
        std::bind(&RaisimBridge::effortCommandCallback, this, std::placeholders::_1));

    // Delay sim startup
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // Create timer to update the simulation
    timer_ = this->create_wall_timer(
        std::chrono::microseconds((int)(time_step_ms * 1000)),
        std::bind(&RaisimBridge::update, this));

    // Setup raisim server
    server.launchServer(8080);

    // Wait for server connection
    RCLCPP_INFO(this->get_logger(), "Awaiting Connection to raisim server");
    while (!server.isConnected())
      ;
    RCLCPP_INFO(this->get_logger(), "Server Connected");

    // Focus on the robot
    server.focusOn(robot);

    // Set start time checking dimulation time displacement
    startTime = std::chrono::high_resolution_clock::now();
  }

  ~RaisimBridge() override
  {
    // Call cleanup function
    cleanup();
  }

  // Function to safely kill the simulator
  void cleanup()
  {
    // If shutdown hasnt occured already
    if (!shutdown_called_)
    {
      // Calculate simulation time displacement
      auto endTime = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
      RCLCPP_INFO(this->get_logger(), "Ran for %ld ms", duration);

      RCLCPP_INFO(this->get_logger(), "Shutting down RaisimBridge");

      // Kill the raisim server
      server.killServer();

      // Ensure shutdown not triggered again
      shutdown_called_ = true;
    }
  }

private:
  // Main simulation loop
  void update()
  {
    // Step simulation
    server.integrateWorldThreadSafe();

    // Log system energy (use class gravity if you store it later)
    // RCLCPP_INFO(this->get_logger(), "Energy: %f", robot->getEnergy(Eigen::Vector3d(0, 0, -9.81)));

    // Update internal state vectors
    gc = robot->getGeneralizedCoordinate().e();
    gv = robot->getGeneralizedVelocity().e();
    ga = robot->getGeneralizedAcceleration().e();

    // Setup the joinstate message
    sensor_msgs::msg::JointState js;
    int dof = robot->getDOF();
    js.header.stamp = now();
    js.name.resize(dof);
    js.position.resize(dof);
    js.velocity.resize(dof);

    // Add each joint to the joinstate message
    for (int i = 0; i < dof; ++i)
    {
      js.name[i] = "joint_" + std::to_string(i); // Generic joint name
      js.position[i] = gc[i];
      js.velocity[i] = gv[i];
    }

    // Publish joint states
    joint_state_pub->publish(js);
  }

  // Reads commands from the control node and sends to the simulation
  void effortCommandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    // Make sure control commands match the robot dof
    if (msg->data.size() != robot->getDOF())
    {
      RCLCPP_WARN(this->get_logger(), "Received effort command of wrong size: %ld (expected %ld)",
                  msg->data.size(), robot->getDOF());
      return;
    }

    // Move forces from message into usable vector format
    Eigen::VectorXd tau(robot->getDOF());
    for (size_t i = 0; i < msg->data.size(); ++i)
    {
      tau[i] = msg->data[i];
    }

    // Send forces to the simulation
    robot->setGeneralizedForce(tau);
  }

  bool shutdown_called_ = false;
  int time_step;
  raisim::World world;
  raisim::RaisimServer server{&world};
  raisim::ArticulatedSystem *robot;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr joint_cmd_sub;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr effort_cmd_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  Eigen::VectorXd gc, gv, ga, gf, damping;

  std::chrono::_V2::system_clock::time_point startTime;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RaisimBridge>();

  // Trigger node cleanup on shutdown (e.g., Ctrl+C)
  rclcpp::on_shutdown([node]()
                      { node->cleanup(); });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
