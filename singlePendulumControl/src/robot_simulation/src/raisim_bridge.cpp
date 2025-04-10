#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include <raisim/World.hpp>
#include <raisim/RaisimServer.hpp>

#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <thread>

class RaisimBridge : public rclcpp::Node
{
public:
  RaisimBridge() : Node("raisim_bridge")
  {
    int time_step_ms = this->declare_parameter<int>("time_step_ms", 1);

    world.setTimeStep(time_step_ms / 1000.0f);
    auto ground = world.addGround(-2);

    world.setGravity(Eigen::Vector3d(0, 0, -9.81));

    // raisim::World::setLicenseDirectory("/home/hunter/.raisim");
    raisim::World::setActivationKey("/home/hunter/.raisim");

    std::string urdf_path_base = this->declare_parameter<std::string>("robot_description_path", "/default/path");

    std::string urdf_file = urdf_path_base + "/urdf/robot.urdf";

    // Load into RaiSim
    robot = world.addArticulatedSystem(urdf_file);

    robot->setName("Single Pendulum");

    // Remove Collision Meshes
    for (int i = 0; i <= 2; ++i)
    {
      for (int j = i + 1; j <= 2; ++j)
      {
        robot->ignoreCollisionBetween(i, j);
      }
    }

    Eigen::VectorXd gc(robot->getGeneralizedCoordinateDim()), gv(robot->getDOF()), damping(robot->getDOF());
    gc.setZero();
    gv.setZero();
    
    robot->setGeneralizedCoordinate({0.0});
    robot->setGeneralizedVelocity({0.0});
    robot->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    // robot->setPdGains(Eigen::VectorXd::Constant(1, 50.0),
                      // Eigen::VectorXd::Constant(1, 1.0));

    // joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    // joint_cmd_sub = this->create_subscription<std_msgs::msg::Float64>(
    //     "hinge_position_command", 10,
    //     [this](const std_msgs::msg::time_stepFloat64::SharedPtr msg) {
    //       robot->setPdTarget({msg->data}, {0.0});
    //     });

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(time_step_ms),
        std::bind(&RaisimBridge::update, this));

    server.launchServer(8080);

    // Wait for server connection
    RCLCPP_INFO(this->get_logger(), "Awaiting Connection to raisim server");
    while (!server.isConnected())
      ;
    RCLCPP_INFO(this->get_logger(), "Server Connected");
    server.focusOn(robot);

    startTime = std::chrono::high_resolution_clock::now();
  }

  ~RaisimBridge() override
  {
    cleanup(); // destructor still calls cleanup as backup
  }

  void cleanup()
  {
    if (!shutdown_called_)
    {
      auto endTime = std::chrono::high_resolution_clock::now();
      RCLCPP_INFO(this->get_logger(), "Shutting down RaisimBridge");

      server.killServer();

      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
      RCLCPP_INFO(this->get_logger(), "Ran for %ld ms", duration);

      shutdown_called_ = true;
    }
  }

private:
  void update()
  {
    // world.integrate();
    server.integrateWorldThreadSafe();

    // sensor_msgs::msg::JointState js;
    // js.header.stamp = now();
    // js.name.push_back("hinge");
    // js.position.push_back(robot->getGeneralizedCoordinate()[0]);
    // js.velocity.push_back(robot->getGeneralizedVelocity()[0]);
    // joint_state_pub->publish(js);
  }

  bool shutdown_called_ = false;
  int time_step;
  raisim::World world;
  raisim::RaisimServer server{&world};
  raisim::ArticulatedSystem *robot;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr joint_cmd_sub;
  rclcpp::TimerBase::SharedPtr timer_;

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
