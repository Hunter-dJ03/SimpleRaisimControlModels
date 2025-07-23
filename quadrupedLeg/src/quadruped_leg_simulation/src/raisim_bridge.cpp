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
        time_step_ms = this->declare_parameter<float>("time_step_ms", 1.0);
        fixed_robot_body = this->declare_parameter<bool>("fixed_robot_body", true);

        // Logging info
        RCLCPP_INFO(this->get_logger(), "Time step for simulation: %f ms", time_step_ms);
        RCLCPP_INFO(this->get_logger(), "Fixed robot body: %s", fixed_robot_body ? "true" : "false");

        // Setup initial leg positions, orientations and joint positions
        this->declare_parameter<std::vector<double>>("body_initial_position", std::vector<double>{});
        this->declare_parameter<std::vector<double>>("body_initial_orientation", std::vector<double>{});
        this->declare_parameter<std::vector<double>>("joint_initial_positions", std::vector<double>{});
        std::vector<double> body_initial_position;
        std::vector<double> body_initial_orientation;
        std::vector<double> joint_initial_positions;
        this->get_parameter("body_initial_position", body_initial_position);
        this->get_parameter("body_initial_orientation", body_initial_orientation);
        this->get_parameter("joint_initial_positions", joint_initial_positions);

        // Map each std::vector to an Eigen::VectorXd
        Eigen::VectorXd body_pos = Eigen::Map<Eigen::VectorXd>(body_initial_position.data(), body_initial_position.size());
        Eigen::VectorXd body_orient = Eigen::Map<Eigen::VectorXd>(body_initial_orientation.data(), body_initial_orientation.size());
        Eigen::VectorXd joint_pos = Eigen::Map<Eigen::VectorXd>(joint_initial_positions.data(), joint_initial_positions.size());

        // Allocate one big VectorXd
        int N_pos = body_pos.size();
        int N_orient = body_orient.size();
        int N_joints = joint_pos.size();

        if (fixed_robot_body)
        {
            // If the robot body is fixed, we only need to set the joint positions
            init_state.resize(N_joints);

            init_state.segment(0, N_joints) = joint_pos;
        }
        else
        {
            // If the robot body is not fixed, we need to set the body position, orientation and joint positions
            init_state.resize(N_pos + N_orient + N_joints);

            init_state.segment(0, N_pos) = body_pos;
            init_state.segment(N_pos, N_orient) = body_orient;
            init_state.segment(N_pos + N_orient, N_joints) = joint_pos;
        }

        // Set world timestep for simulation
        world.setTimeStep(time_step_ms / 1000.0f);
        auto ground = world.addGround(0);

        // Variable Gravity option
        // world.setGravity(Eigen::Vector3d(0, 0, 0));

        // Raisim Activation Key
        raisim::World::setActivationKey("$ENV{HOME}/.raisim");

        // Get robot URDF file path from description package
        std::string urdf_path_base = this->declare_parameter<std::string>("robot_description_path", "/default/path");
        std::string urdf_file;

        // Setup URDF file path based on whether the robot body is fixed or not
        if (fixed_robot_body)
        {
            urdf_file = urdf_path_base + "/urdf/robot_fixed.urdf";
        }
        else
        {
            urdf_file = urdf_path_base + "/urdf/robot.urdf";
        }

        // Load robot into RaiSim and give name
        robot = world.addArticulatedSystem(urdf_file);
        robot->setName("Quadruped Leg");

        // Remove Collision Meshes between neighbouring joints
        robot->ignoreCollisionBetween(0, 1);
        robot->ignoreCollisionBetween(1, 2);
        robot->ignoreCollisionBetween(2, 3);

        // Setup parameter sizes for generalised position, velocity, acceleration, force and damping and set to zero
        gc = Eigen::VectorXd::Zero(robot->getGeneralizedCoordinateDim());
        gv = Eigen::VectorXd::Zero(robot->getDOF());
        gf = Eigen::VectorXd::Zero(robot->getDOF());
        // damping = Eigen::VectorXd::Zero(robot->getDOF());

        // Set siulation position and velocity
        robot->setGeneralizedCoordinate(init_state);
        robot->setGeneralizedVelocity(gv);
        robot->setGeneralizedVelocity(gf);

        // Setup control mode for the simulation (May be wrong or unnecessary)
        // robot->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);

        // Setup raisim server
        server.launchServer(8080);

        // Wait for server connection
        RCLCPP_INFO(this->get_logger(), "Awaiting Connection to raisim server");
        while (!server.isConnected())
            ;
        RCLCPP_INFO(this->get_logger(), "Server Connected");

        std::this_thread::sleep_for(std::chrono::milliseconds(3000));

        RCLCPP_INFO(this->get_logger(), "RaisimBridge Node Initialised");

        // Focus on the robot
        server.focusOn(robot);

        // Create Publisher for robot joint states
        joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // Create subscription to control node topic for joint effort commands
        desired_cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_desired_control", 10,
            std::bind(&RaisimBridge::effortCommandCallback, this, std::placeholders::_1));

        // Create timer to update the simulation
        timer_ = this->create_wall_timer(
            std::chrono::microseconds((int)(time_step_ms * 1000)),
            std::bind(&RaisimBridge::update, this));

        // Set start time checking simulation time displacement
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

        // Update internal state vectors
        gc = robot->getGeneralizedCoordinate().e();
        gv = robot->getGeneralizedVelocity().e();
        gf = robot->getGeneralizedForce().e();

        // Setup the joinstate message
        sensor_msgs::msg::JointState js;
        int dof = robot->getDOF();
        js.header.stamp = now();
        js.name.resize(dof);
        js.position.resize(dof);
        js.velocity.resize(dof);
        js.effort.resize(dof);

        // Add each joint to the jointstate message
        if (fixed_robot_body)
        {
            // Add each joint to the jointstate message
            for (int i = 0; i < dof; ++i)
            {
                js.name[i] = "joint_" + std::to_string(i); // Generic joint name
                js.position[i] = gc[i];
                js.velocity[i] = gv[i];
                js.effort[i] = gf[i];
            }
        }
        else
        {
            // Add each joint to the jointstate message
            for (int i = 0; i < dof - 6; ++i)
            {
                js.name[i] = "joint_" + std::to_string(i); // Generic joint name
                js.position[i] = gc[i + 7];
                js.velocity[i] = gv[i + 6];
                js.effort[i] = gf[i + 6];
            }
        }

        // Publish joint states
        joint_state_pub->publish(js);
    }

    // Reads commands from the control node and sends to the simulation
    void effortCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {

        // Make sure control commands match the robot dof
        if (msg->position.size() != robot->getDOF() - 6 * !fixed_robot_body)
        {
            RCLCPP_WARN(this->get_logger(), "Received effort command of wrong size: %ld (expected %ld)", msg->position.size(), robot->getDOF() - 6 * !fixed_robot_body);
            return;
        }

        // Move forces from message into usable vector format
        Eigen::VectorXd tau = Eigen::VectorXd::Zero(robot->getDOF());

        if (fixed_robot_body)
        {
            for (size_t i = 0; i < msg->position.size(); ++i)
            {
                // PD control law
                tau[i] = p_gain[i] * (msg->position[i] - gc[i]) + d_gain[i] * (msg->velocity[i] - gv[i]) + msg->effort[i];
                // tau[i+6] = std::clamp(tau[i+6], -30.0, 30.0); // Clamp to max effort
            }
        }
        else
        {
            for (size_t i = 0; i < msg->position.size(); ++i)
            {
                // PD control law
                tau[i + 6] = p_gain[i] * (msg->position[i] - gc[i + 7]) + d_gain[i] * (msg->velocity[i] - gv[i + 6] + msg->effort[i]);
                // tau[i+6] = std::clamp(tau[i+6], -30.0, 30.0); // Clamp to max effort
            }
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
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr desired_cmd_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    Eigen::VectorXd gc, gv, gf, damping, init_state;

    std::chrono::_V2::system_clock::time_point startTime;

    float time_step_ms;
    bool fixed_robot_body;

    // const double p_gain[3] = {220.0, 220.0, 220.0};
    // const double d_gain[3] = {5.0, 5.0, 5.0};

    const double p_gain[3] = {0, 0, 0};
    const double d_gain[3] = {0, 0, 0};
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
