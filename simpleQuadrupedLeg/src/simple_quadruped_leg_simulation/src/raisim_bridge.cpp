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
    std::string urdf_file = urdf_path_base + "/urdf/robot.urdf";

    // Load robot into RaiSim and give name
    robot = world.addArticulatedSystem(urdf_file);
    robot->setName("Double Pendulum");
    

    // Remove Collision Meshes
    for (int i = 0; i <= 3; ++i)
    {
      for (int j = i + 1; j <= 3; ++j)
      {
        robot->ignoreCollisionBetween(i, j);
      }
    }

    // Setup parameter sizes for generalised position, velocity, acceleration, force and damping and set to zero
    gc = Eigen::VectorXd::Zero(robot->getGeneralizedCoordinateDim());
    gv = Eigen::VectorXd::Zero(robot->getDOF());
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
    desired_cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_desired_control", 10,
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

    // Create graphs for joint positions and velocities
    joint0_pos_graph = server.addTimeSeriesGraph("Joint 0 Pos", {"Measured", "Desired"}, "Time (s)", "Position (rad)");
    joint1_pos_graph = server.addTimeSeriesGraph("Joint 1 Pos", {"Measured", "Desired"}, "Time (s)", "Position (rad)");
    joint0_vel_graph = server.addTimeSeriesGraph("Joint 0 Vel", {"Measured", "Desired"}, "Time (s)", "Velocity (rad/s)");
    joint1_vel_graph = server.addTimeSeriesGraph("Joint 1 Vel", {"Measured", "Desired"}, "Time (s)", "Velocity (rad/s)");

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

    // Setup the joinstate message
    sensor_msgs::msg::JointState js;
    int dof = robot->getDOF();
    js.header.stamp = now();
    js.name.resize(dof);
    js.position.resize(dof);
    js.velocity.resize(dof);

    // Add each joint to the jointstate message
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
  void effortCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Make sure control commands match the robot dof
    if (msg->position.size() != robot->getDOF())
    {
      RCLCPP_WARN(this->get_logger(), "Received effort command of wrong size: %ld (expected %ld)",
                  msg->position.size(), robot->getDOF());
      return;
    }

    // Move forces from message into usable vector format
    Eigen::VectorXd tau(robot->getDOF());

    for (size_t i = 0; i < msg->position.size(); ++i)
    {
      // PD control law
      tau[i] = p_gain[i] * (msg->position[i] - gc[i]) + d_gain[i] * (msg->velocity[i] - gv[i]);
    }

    // Joint 0: position and velocity
    raisim::VecDyn joint0_pos(2);
    joint0_pos[0] = gc[0];           // measured position
    joint0_pos[1] = msg->position[0]; // desired position

    raisim::VecDyn joint0_vel(2);
    joint0_vel[0] = gv[0];           // measured velocity
    joint0_vel[1] = msg->velocity[0]; // desired velocity

    // Joint 1: position and velocity
    raisim::VecDyn joint1_pos(2);
    joint1_pos[0] = gc[1];           // measured position
    joint1_pos[1] = msg->position[1]; // desired position

    raisim::VecDyn joint1_vel(2);
    joint1_vel[0] = gv[1];           // measured velocity
    joint1_vel[1] = msg->velocity[1]; // desired velocity

    // Update graphs with current joint states
    joint0_pos_graph->addDataPoints(world.getWorldTime(), joint0_pos);
    joint1_pos_graph->addDataPoints(world.getWorldTime(), joint1_pos);
    joint0_vel_graph->addDataPoints(world.getWorldTime(), joint0_vel);
    joint1_vel_graph->addDataPoints(world.getWorldTime(), joint1_vel);


    // Send forces to the simulation
    robot->setGeneralizedForce(tau);
  }

  bool shutdown_called_ = false;
  int time_step;
  raisim::World world;
  raisim::RaisimServer server{&world};
  raisim::ArticulatedSystem *robot;
  raisim::TimeSeriesGraph *joint0_pos_graph;
  raisim::TimeSeriesGraph *joint1_pos_graph;
  raisim::TimeSeriesGraph *joint0_vel_graph;
  raisim::TimeSeriesGraph *joint1_vel_graph;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr joint_cmd_sub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr desired_cmd_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  Eigen::VectorXd gc, gv, gf, damping;

  std::chrono::_V2::system_clock::time_point startTime;

  // const double p_gain = 15500.0;
  // const double d_gain = 2000.0;
  const double p_gain[3] = {1500.0, 1500.0, 1500.0};
  const double d_gain[3] = {0.1, 0.05, 0.001};
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
