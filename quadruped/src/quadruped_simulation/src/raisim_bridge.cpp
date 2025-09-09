#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <raisim/World.hpp>
#include <raisim/RaisimServer.hpp>

#include <chrono>
#include <thread>
#include <Eigen/Dense>

class RaisimBridge : public rclcpp::Node
{
public:
	RaisimBridge() : Node("raisim_bridge")
	{
		// Setup ROS2 parameter time step for simulation, timers and models
		pd_time_step_ms = this->declare_parameter<float>("pd_time_step_ms", 1.0);
		fixed_robot_body = this->declare_parameter<bool>("fixed_robot_body", false);

		// Logging info
		RCLCPP_INFO(this->get_logger(), "Time step for simulation: %f ms", pd_time_step_ms);
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
		dt_ = pd_time_step_ms * 1e-3; // seconds
		world.setTimeStep(dt_);

		// Set default material properties (restitution, friction, adhesion)
		world.setDefaultMaterial(1.0, 0.2, 0.0);

		clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>(
			"/clock", rclcpp::QoS(10).best_effort());

		[[maybe_unused]] auto ground = world.addGround(0);

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

		// Remove Collision Meshes between adjacent links
		robot->ignoreCollisionBetween(0, 1);   // Body to Coxa 1
		robot->ignoreCollisionBetween(1, 2);   // Coxa 1 to Femur 1
		robot->ignoreCollisionBetween(2, 3);   // Femur 1 to Tibia 1
		robot->ignoreCollisionBetween(0, 4);   // Body to Coxa 2
		robot->ignoreCollisionBetween(4, 5);   // Coxa 2 to Femur 2
		robot->ignoreCollisionBetween(5, 6);   // Femur 2 to Tibia 2
		robot->ignoreCollisionBetween(0, 7);   // Body to Coxa 3
		robot->ignoreCollisionBetween(7, 8);   // Coxa 3 to Femur 3
		robot->ignoreCollisionBetween(8, 9);   // Femur 3 to Tibia 3
		robot->ignoreCollisionBetween(0, 10);  // Body to Coxa 4
		robot->ignoreCollisionBetween(10, 11); // Coxa 4 to Femur 4
		robot->ignoreCollisionBetween(11, 12); // Femur 4 to Tibia 4

		// Setup parameter sizes for generalised position, velocity, acceleration, force and damping and set to zero
		gc = Eigen::VectorXd::Zero(robot->getGeneralizedCoordinateDim());
		gv = Eigen::VectorXd::Zero(robot->getDOF());
		gf = Eigen::VectorXd::Zero(robot->getDOF());
		// damping = Eigen::VectorXd::Zero(robot->getDOF());

		q_ref = joint_pos;
		qd_ref = Eigen::VectorXd::Zero(N_joints);
		tau_comp = Eigen::VectorXd::Zero(N_joints);

		// Set siulation position and velocity
		robot->setGeneralizedCoordinate(init_state);
		robot->setGeneralizedVelocity(gv);
		robot->setGeneralizedForce(gf);

		// CoM Ball Display
		comSphere = server.addVisualSphere("viz_sphere", 0.01, 1, 0, 0, 1);

		// Setup raisim server
		server.launchServer(8080);

		// Wait for server connection
		RCLCPP_INFO(this->get_logger(), "Awaiting Connection to raisim server");
		while (!server.isConnected())
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		};

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
			std::chrono::duration<double>(dt_),
			std::bind(&RaisimBridge::update, this));

		// Set start time checking dimulation time displacement
		startTime = std::chrono::high_resolution_clock::now();
	}

	/*
	 * Destructor to clean up resources and safely shut down the Raisim server
	 */
	~RaisimBridge() override
	{
		// Call cleanup function
		cleanup();
	}

	/*
	 * Cleanup function to ensure the Raisim server is properly shut down
	 * This function is called in the destructor and can also be called manually
	 * It calculates the total simulation time and logs it before shutting down the server
	 */
	void cleanup()
	{
		// If shutdown hasnt occured already
		if (!shutdown_called_)
		{
			// Calculate simulation time displacement
			auto endTime = std::chrono::high_resolution_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
			RCLCPP_INFO(
				this->get_logger(),
				"\nRan for %.3f s\nSimulated time: %.3f s\nShutting down RaisimBridge",
				duration * 1e-3,
				sim_time_ns_ * 1e-9);

			// Kill the raisim server
			server.killServer();

			// Ensure shutdown not triggered again
			shutdown_called_ = true;
		}
	}

private:
	/*
	 * Function to update the simulation and publish joint states
	 * This function is called at a fixed time interval defined by the timer
	 */
	void update()
	{
		// RCLCPP_DEBUG(this->get_logger(), "Received joint effort command");

		// Update internal state vectors
		gc = robot->getGeneralizedCoordinate().e();
		gv = robot->getGeneralizedVelocity().e();
		gf = robot->getGeneralizedForce().e();

		// Get Centre of Mass (COM) position and update visual sphere
		auto com = robot->getCOM();
		comSphere->setPosition(com[0], com[1], com[2]);

		// Build a single time stamp for this step in sim time
		builtin_interfaces::msg::Time stamp;
		stamp.sec = static_cast<int32_t>(sim_time_ns_ / 1000000000LL);
		stamp.nanosec = static_cast<uint32_t>(sim_time_ns_ % 1000000000LL);

		// Send stamp to /clock topic for sim time
		rosgraph_msgs::msg::Clock clk;
		clk.clock = stamp;
		clock_pub_->publish(clk);

		// Setup the joinstate message
		sensor_msgs::msg::JointState js;
		int dof = robot->getDOF();
		js.header.stamp = stamp;
		js.position.resize(dof);
		js.velocity.resize(dof);
		js.effort.resize(dof);

		// Initialize torque vector
		Eigen::VectorXd tau = Eigen::VectorXd::Zero(dof);

		if (fixed_robot_body)
		{
			// For each joint
			for (int i = 0; i < dof; ++i)
			{
				// PD Control Law
				tau[i] = p_gain[i] * (q_ref[i] - gc[i]) + d_gain[i] * (qd_ref[i] - gv[i]) + tau_comp[i];
				tau[i] = std::clamp(tau[i], -60.0, 60.0);

				// Add each joint to the jointstate message
				js.position[i] = gc[i];
				js.velocity[i] = gv[i];
				js.effort[i] = gf[i];
			}
		}
		else
		{
			// For each joint (ignoring first 6 dof of floating base)
			for (int i = 0; i < dof - 6; ++i)
			{
				// PD Control Law
				// Note the offset in gc and gv for the floating base
				// gc has 7 offset (3 pos, 4 orient [quaternion]), gv has 6 offset (3 linear, 3 angular)
				tau[i + 6] = p_gain[i] * (q_ref[i] - gc[i + 7]) + d_gain[i] * (qd_ref[i] - gv[i + 6]) + tau_comp[i];
				tau[i + 6] = std::clamp(tau[i + 6], -60.0, 60.0); // Clamp torques to reasonable values

				// Add each joint to the jointstate message
				js.position[i] = gc[i + 7];
				js.velocity[i] = gv[i + 6];
				js.effort[i] = gf[i + 6];
			}
		}

		// Send forces to the simulation
		robot->setGeneralizedForce(tau);

		// Publish joint states
		joint_state_pub->publish(js);

		// Step simulation
		server.integrateWorldThreadSafe();

		// Update simulated time
		sim_time_ns_ += static_cast<int64_t>(dt_ * 1e9);
	}

	/*
	 * Callback function to handle incoming joint effort commands
	 * This function is triggered when a new message is received on the "joint_desired_control" topic
	 * It saves the control reference commands to memory for the next simulation step
	 *
	 * @param msg The incoming message containing joint positions, velocities, and efforts
	 */
	void effortCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
	{
		// Make sure control commands match the robot dof
		if (msg->position.size() != robot->getDOF() - 6 * !fixed_robot_body)
		{
			RCLCPP_WARN(this->get_logger(), "Received effort command of wrong size: %ld (expected %ld)", msg->position.size(), robot->getDOF() - 6 * !fixed_robot_body);
			return;
		}

		// Save the commands to memory for the next simulation step
		for (size_t i = 0; i < msg->position.size(); ++i)
		{
			q_ref[i] = msg->position[i];
			qd_ref[i] = msg->velocity[i];
			tau_comp[i] = msg->effort[i];
		}

		return;
	}

	// Raisim control variables
	bool shutdown_called_ = false;
	raisim::World world;
	raisim::RaisimServer server{&world};
	raisim::ArticulatedSystem *robot;
	raisim::Visuals *comSphere;
	Eigen::VectorXd gc, gv, gf, damping, init_state;
	Eigen::VectorXd q_ref, qd_ref, tau_comp;

	// Declare ROS2 publishers, sibscribers and timers
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr desired_cmd_sub_;
	rclcpp::TimerBase::SharedPtr timer_;

	// Declare internal timer variables
	std::chrono::_V2::system_clock::time_point startTime;

	// Declare parameters for simulation and control
	float pd_time_step_ms;
	rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
	int64_t sim_time_ns_ = 0; // simulated time in nanoseconds
	double dt_ = 0.0;		  // seconds, equals world timestep

	bool fixed_robot_body;

	// PD Control Gains
	const double p_gain[12] = {800.0, 800.0, 800.0, 800.0, 800.0, 800.0, 800.0, 800.0, 800.0, 800.0, 800.0, 800.0};
	const double d_gain[12] = {8.0, 8.0, 8.0, 8.0, 8.0, 8.0, 8.0, 8.0, 8.0, 8.0, 8.0, 8.0};
	// const double p_gain[12] = {120.0, 120.0, 120.0, 120.0, 120.0, 120.0, 120.0, 120.0, 120.0, 120.0, 120.0, 120.0};
	// const double d_gain[12] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
	// const double p_gain[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	// const double d_gain[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
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
