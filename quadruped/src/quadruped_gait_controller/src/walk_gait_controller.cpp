#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "quadruped_interfaces/msg/endpoint.hpp"
#include "quadruped_interfaces/msg/foot_states.hpp"
#include "quadruped_interfaces/msg/full_body_control_command.hpp"
#include <quadruped_interfaces/srv/set_generalized_coordinate.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>


#include <Eigen/Dense>
#include <Eigen/QR>
#include <array>
#include <cmath>

class QuadrupedGaitController : public rclcpp::Node
{
public:
	QuadrupedGaitController() : Node("quadruped_gait_controller")
	{
		// Setup ROS2 parameter time step for simulation, timers and models
		control_time_step_ms = this->declare_parameter<float>("control_time_step_ms", 1.0);
		init_pos = this->declare_parameter<std::vector<double>>("joint_initial_positions", std::vector<double>{});
		link_lengths = this->declare_parameter<std::vector<double>>("link_lengths", std::vector<double>{});

		// Initialise variables for leg joint positions, velocities, and foot positions
		legJointPosition.resize(4);
		legJointVelocity = std::vector<Eigen::Vector3d>(4, Eigen::Vector3d::Zero());
		footPosition = std::vector<Eigen::Vector3d>(4, Eigen::Vector3d::Zero());
		footPositionInit = std::vector<Eigen::Vector3d>(4, Eigen::Vector3d::Zero());
		footPositionWalk = std::vector<Eigen::Vector3d>(4, Eigen::Vector3d::Zero());
		footPositionActual = std::vector<Eigen::Vector3d>(4, Eigen::Vector3d::Zero());
		q = std::vector<Eigen::Vector3d>(4, Eigen::Vector3d::Zero());
		qd = std::vector<Eigen::Vector3d>(4, Eigen::Vector3d::Zero());

		dqb_ref = Eigen::VectorXd::Zero(6);
		dqp_ref = Eigen::VectorXd::Zero(24);

		// Fill variables based on intial configuration
		for (int leg = 0; leg < 4; ++leg)
		{

			legJointPosition[leg] = Eigen::Vector3d(
				init_pos[leg * 3 + 0],
				init_pos[leg * 3 + 1],
				init_pos[leg * 3 + 2]);

			// Compute foot position from FK
			footPositionActual[leg] = forwardKinematics(
				legJointPosition[leg],
				leg);

			footPosition[leg] = footPositionActual[leg];

			footPositionInit[leg] = footPositionActual[leg];

			legJointPosition[leg] = inverseKinematics(Eigen::Vector3d(footPositionActual[leg][0] + forwardWalkOffset[leg] *0.0, footPositionActual[leg][1] + sideWalkOffset[leg], footPositionActual[leg][2]), leg);

			footPositionActual[leg] = forwardKinematics(
				legJointPosition[leg],
				leg);

			// std::cout << "Vector Walk: " << footPositionActual[leg].transpose() << std::endl;

			footPositionWalk[leg] = footPositionActual[leg];

			// std::cout << "Vector Init: " << footPositionInit[leg].transpose() << "        Vector Walk: " << footPositionWalk[leg].transpose() << std::endl;

			q[leg] = legJointPosition[leg];
		}

		// Set up subscription to encoder feedback for joint states
		joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
			"joint_states", 10,
			std::bind(&QuadrupedGaitController::jointStateCallback, this, std::placeholders::_1));

		// Set up publisher for desired foot states messages
		foot_state_publisher_ = this->create_publisher<quadruped_interfaces::msg::FootStates>("foot_states", 10);

		full_body_command = this->create_publisher<quadruped_interfaces::msg::FullBodyControlCommand>("full_body_control_command", 10);

		// Create the service client
		set_gc_client_ = this->create_client<quadruped_interfaces::srv::SetGeneralizedCoordinate>(
			"/set_generalized_coordinate");

		// Block until the service is ready
		while (!set_gc_client_->wait_for_service(std::chrono::seconds(1)))
		{
			if (!rclcpp::ok())
			{
				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for /set_generalized_coordinate");
				throw std::runtime_error("Service wait interrupted");
			}
			RCLCPP_INFO(this->get_logger(), "Waiting for /set_generalized_coordinate...");
		}

		// Build the 12 joint targets from the IK results you already computed in q[0..3]
		// If you want to drive it from footPositionWalk explicitly, you can also do
		// q[leg] = inverseKinematics(footPositionWalk[leg], leg);
		auto req = std::make_shared<quadruped_interfaces::srv::SetGeneralizedCoordinate::Request>();
		for (int leg = 0; leg < 4; ++leg)
		{
			req->q[leg * 3 + 0] = q[leg][0];
			req->q[leg * 3 + 1] = q[leg][1];
			req->q[leg * 3 + 2] = q[leg][2];
		}

		// Send request and block until the response arrives
		auto future = set_gc_client_->async_send_request(req);

// Use a temporary executor; do NOT call shared_from_this() in a ctor.
rclcpp::executors::SingleThreadedExecutor exec;
exec.add_node(this->get_node_base_interface());
auto ret = exec.spin_until_future_complete(future, std::chrono::seconds(5));


		if (ret != rclcpp::FutureReturnCode::SUCCESS)
		{
			RCLCPP_ERROR(this->get_logger(), "Service call to /set_generalized_coordinate did not complete");
			throw std::runtime_error("Initial joint teleport failed");
		}

		const auto res = future.get();
		if (!res->ok)
		{
			RCLCPP_ERROR(this->get_logger(), "Service returned error: %s", res->message.c_str());
			throw std::runtime_error("Initial joint teleport rejected by server");
		}

		RCLCPP_INFO(this->get_logger(), "Initial joint teleport applied: %s", res->message.c_str());

		// Create timer to update the control commands
		timer_ = rclcpp::create_timer(
			this->get_node_base_interface(),
			this->get_node_timers_interface(),
			this->get_clock(),
			std::chrono::microseconds((int)(control_time_step_ms * 1000)),
			std::bind(&QuadrupedGaitController::controlCommands, this));

		// Feedback for controller start
		RCLCPP_INFO(this->get_logger(), "Quadruped Gait Controller Node started");
	}

private:
	/*
	 * Callback that clauclated the desired joint states based on the current joint states and desired trajectory.
	 *  Current implements each leg as a single 3-DOF joint with a desired trajectory.
	 *
	 * @param msg The message containing the joint states.
	 *
	 */
	void controlCommands()
	{

		// If sim time has not started yet, do nothing
		const auto now_ros = this->get_clock()->now();
		if (now_ros.seconds() == 0.0)
			return;

		// one stamp for all messages this tick
		const auto stamp = now_ros;

		// return;
		// Create control effort message
		// quadruped_interfaces::msg::FootStates foot_state_msg;
		// foot_state_msg.header.stamp = stamp;
		// foot_state_msg.desired_positions.resize(4);
		// foot_state_msg.desired_velocities.resize(4);

		// Create endpoint message
		// quadruped_interfaces::msg::Endpoint endpoint_msg;
		// endpoint_msg.header.stamp = stamp;

		quadruped_interfaces::msg::FullBodyControlCommand command_msg;
		command_msg.header.stamp = stamp;
		command_msg.dqb_ref.resize(6);
		command_msg.dqp_ref.resize(24);

		dqb_ref.setZero();
		dqp_ref.setZero();

		dqb_ref[0] = forwardStepLength / (stepDuration) * 1000.0;

		// dqb_ref[0] = A1 * cos(omega1 * (now_ros.seconds() - 2.0));

		// For each leg, calculate the desired joint states based on the current joint states and desired trajectory
		for (size_t leg = 0; leg < 4; ++leg)
		{
			if (stepTimer[leg] >= stepDuration)
			{
				stepTimer[leg] = 0;
				// RCLCPP_INFO(this->get_logger(), "Resetting step timer for leg %ld", leg);
			}

			if (stepTimer[leg] < stepDuration / 4) // Swing phase
			{

				dqp_ref[leg * 6 + 0] = forwardStepLength * (6*a[6]*pow(stepTimer[leg],5) + 5*a[5]*pow(stepTimer[leg],4) + 4*a[4]*pow(stepTimer[leg],3) + 3*a[3]*pow(stepTimer[leg],2) + 2*a[2]*stepTimer[leg] + a[1]);
				dqp_ref[leg * 6 + 1] = sideStepLength * (6*a[6]*pow(stepTimer[leg],5) + 5*a[5]*pow(stepTimer[leg],4) + 4*a[4]*pow(stepTimer[leg],3) + 3*a[3]*pow(stepTimer[leg],2) + 2*a[2]*stepTimer[leg] + a[1]);
				dqp_ref[leg * 6 + 2] = stepHeight * (6*b[6]*pow(stepTimer[leg],5) + 5*b[5]*pow(stepTimer[leg],4) + 4*b[4]*pow(stepTimer[leg],3) + 3*b[3]*pow(stepTimer[leg],2) + 2*b[2]*stepTimer[leg] + b[1]);
			}

			// Wait for 1 second before walking
			if (now_ros.seconds() >= 2.0)
			{
				stepTimer[leg] += control_time_step_ms;
			}
		}

		if (now_ros.seconds() < 2.0)
		{
			dqb_ref.setZero();
			dqp_ref.setZero();
		}

		for (size_t i = 0; i < 6; ++i)
		{
			command_msg.dqb_ref[i] = dqb_ref[i];
		}

		for (size_t i = 0; i < 24; ++i)
		{
			command_msg.dqp_ref[i] = dqp_ref[i] * 1000; // Convert to m/s from m/ms
		}

		full_body_command->publish(command_msg);
	}

	void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
	{
		// Check if the message has the correct size
		// if (msg->position.size() != 12 || msg->velocity.size() != 12)
		// {
		// 	RCLCPP_ERROR(this->get_logger(), "Received joint state message with incorrect size");
		// 	return;
		// }

		// Unpack joint states from the message
		for (size_t leg = 0; leg < 4; ++leg)
		{
			q[leg] = Eigen::Vector3d(
				msg->position[0 + leg * 3],
				msg->position[1 + leg * 3],
				msg->position[2 + leg * 3]);
			qd[leg] = Eigen::Vector3d(
				msg->velocity[0 + leg * 3],
				msg->velocity[1 + leg * 3],
				msg->velocity[2 + leg * 3]);
		};

		return; // This function is not used in this controller
	};

	/*
	 * Computes the position of foot position for a 3-DOF leg based on the joint angles.
	 * Contains the full 4x4 transformation matrix however only using position atm
	 *
	 * @param q The joint angles of the leg (3 DOF).
	 * @param leg The index of the leg (0-3).
	 *
	 * @return The position of the foot in world coordinates (3D vector).
	 */
	Eigen::VectorXd forwardKinematics(const Eigen::Vector3d &q,
									  const int leg)
	{
		double theta1 = q(0);
		double theta2 = q(1);
		double theta3 = q(2);

		// Precompute useful terms
		// // Link lengths
		double l1 = link_lengths[0];
		double l2 = link_lengths[1];
		double l3 = link_lengths[2];

		if (leg == 0 || leg == 1)
		{
			l1 *= -1;
		}

		double x0 = 0.2631; // Base position in x
		double y0 = 0.1560; // Base position in y
		// double z0 = -38.5 - 25.0; // Base position in z
		double z0 = 0.0; // Base position in z

		if (leg == 2 || leg == 3)
		{
			y0 *= -1; // Adjust y position for right legs
		}

		if (leg == 1 || leg == 2)
		{
			x0 *= -1; // Adjust x position for rear legs
		}

		// Build the transformation matrix
		Eigen::Matrix4d fk;
		fk << -std::cos(theta2 + theta3),
			std::sin(theta2 + theta3),
			0,
			x0 - l3 * std::sin(theta2 + theta3) - l2 * std::cos(theta2),
			-std::sin(theta2 + theta3) * std::sin(theta1),
			-std::cos(theta2 + theta3) * std::sin(theta1),
			std::cos(theta1),
			y0 - l1 * std::cos(theta1) - l2 * std::sin(theta1) * std::sin(theta2) + l3 * std::cos(theta2) * std::cos(theta3) * std::sin(theta1) - l3 * std::sin(theta1) * std::sin(theta2) * std::sin(theta3),
			std::sin(theta2 + theta3) * std::cos(theta1),
			std::cos(theta2 + theta3) * std::cos(theta1),
			std::sin(theta1),
			z0 - l1 * std::sin(theta1) + l2 * std::cos(theta1) * std::sin(theta2) - l3 * std::cos(theta1) * std::cos(theta2) * std::cos(theta3) + l3 * std::cos(theta1) * std::sin(theta2) * std::sin(theta3),
			0,
			0,
			0,
			1;

		// Extract and return the position (4th column, top 3 rows)
		return fk.block<3, 1>(0, 3); // XYZ position
	}

	/*
	 * Computes the inverse kinematics for a 3-DOF leg based on the desired foot position.
	 *
	 * @param q The joint angles of the leg (3 DOF).
	 * @param leg The index of the leg (0-3).
	 *
	 * @return The joint angles that achieve the desired foot position (3D vector).
	 */
	Eigen::Vector3d inverseKinematics(const Eigen::Vector3d &pos,
									  const int leg)
	{
		double xd = pos(0);
		double yd = pos(1);
		double zd = pos(2);

		// Precompute useful terms
		// // Link lengths
		double l1 = link_lengths[0];
		double l2 = link_lengths[1];
		double l3 = link_lengths[2];

		if (leg == 0 || leg == 1)
		{
			l1 *= -1;
		}

		double x0 = 0.2631; // Base position in x
		double y0 = 0.1560; // Base position in y
		// double z0 = -38.5 - 25.0; // Base position in z
		double z0 = 0.0; // Base position in z

		if (leg == 2 || leg == 3)
		{
			y0 *= -1; // Adjust y position for right legs
		}

		if (leg == 1 || leg == 2)
		{
			x0 *= -1; // Adjust x position for rear legs
		}

		// Calculate the joint angles using inverse kinematics
		Eigen::Vector3d qsol;

		// Relative position
		double x = xd - x0;
		double y = yd - y0;
		double z = zd - z0;

		// Compute q1
		double A = std::sqrt(z * z + y * y);
		double a1 = std::atan2(z, y);
		double a2 = std::asin(l1 / A);
		double a3 = M_PI / 2.0 - a2;

		double q1 = a1 - a3 + M_PI;

		// Rotate [x; y; z] by -q1 around X axis
		Eigen::Matrix3d R_x;
		R_x << 1, 0, 0,
			0, std::cos(-q1), -std::sin(-q1),
			0, std::sin(-q1), std::cos(-q1);

		Eigen::Vector3d P = R_x * Eigen::Vector3d(x, y, z);

		double x_p = P(0);
		double z_p = P(2);

		// Compute q2 and q3
		double B = std::sqrt(x_p * x_p + z_p * z_p);
		double b1 = std::atan2(z_p, x_p);
		double b2 = std::acos((l2 * l2 + B * B - l3 * l3) / (2.0 * l2 * B));
		double b3 = std::acos((l2 * l2 + l3 * l3 - B * B) / (2.0 * l2 * l3));

		double q2 = b2 - b1 - M_PI;
		double q3 = b3 - M_PI_2;

		qsol << q1, q2, q3;
		return qsol;
	}

	// Declaration for ROS2 subscriptions and publishers
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
	rclcpp::Publisher<quadruped_interfaces::msg::FootStates>::SharedPtr foot_state_publisher_;
	rclcpp::Publisher<quadruped_interfaces::msg::FullBodyControlCommand>::SharedPtr full_body_command;
	rclcpp::Client<quadruped_interfaces::srv::SetGeneralizedCoordinate>::SharedPtr set_gc_client_;

	rclcpp::TimerBase::SharedPtr timer_;

	// Declaration for model parameters and variables
	std::vector<double> init_pos;
	std::vector<double> link_lengths;

	std::vector<Eigen::Vector3d> legJointPosition;	 // size 4, each is 3-DOF joint position
	std::vector<Eigen::Vector3d> legJointVelocity;	 // size 4, each is 3-DOF joint velocity
	std::vector<Eigen::Vector3d> footPosition;		 // size 4, each is foot position (x, y, z)
	std::vector<Eigen::Vector3d> footPositionInit;	 // size 4, each is foot position (x, y, z)
	std::vector<Eigen::Vector3d> footPositionWalk;	 // size 4, each is foot position (x, y, z)
	std::vector<Eigen::Vector3d> footPositionActual; // size 4, actual foot positions (x, y, z)
	std::vector<Eigen::Vector3d> q;					 // size 4, actual foot positions (x, y, z)
	std::vector<Eigen::Vector3d> qd;				 // size 4, actual foot positions (x, y, z)

	Eigen::VectorXd dqb_ref; // Body velocity (XYZRPY)
	Eigen::VectorXd dqp_ref; // Desired foot velocity

	Eigen::Vector3d zero3 = Eigen::Vector3d::Zero(3);

	double gravity = -9.81;
	float control_time_step_ms;
	// float time = 0;
	int dof = 12;

	// // Waveform A parameters (x)
	double A0 = 0.1;	  // amplitude
	double period0 = 3.0; // period in seconds
	double omega0 = 2.0 * M_PI / period0;

	// Waveform B parameters (y)
	double A1 = 0.3;	  // amplitude
	double period1 = 3.0; // period in seconds
	double omega1 = 2.0 * M_PI / period1;

	// Waveform C parameters (z)
	double A2 = 0.0;	  // amplitude
	double period2 = 3.0; // period in seconds
	double omega2 = 2.0 * M_PI / period2;

	double forwardStepLength = 0.3; // 0.375
	double sideStepLength = 0.0;	// 0.2
	double stepHeight = 0.1;
	double stepDuration = 1000.0;
	double forwardWalkOffset[4] = {-forwardStepLength / 2.0, -forwardStepLength / 6.0, forwardStepLength / 2.0, forwardStepLength / 6.0};
	double sideWalkOffset[4] = {-sideStepLength / 2.0, -sideStepLength / 6.0, sideStepLength / 2.0, sideStepLength / 6.0};
	double stepTimer[4] = {stepDuration * (0.0 / 4.0), stepDuration * (3.0 / 4.0), stepDuration * (1.0 / 4.0), stepDuration * (2.0 / 4.0)};

	double T = stepDuration; // Convert ms to seconds for polynomial coeffs
	std::vector<double> a = {-1.0 / 2.0, -1.0 / T, 0, 800.0 / pow(T, 3), -4800.0 / pow(T, 4), 7680.0 / pow(T, 5), 0};
	std::vector<double> b = {0, 0, 0, 4096.0 / pow(T, 3), -49152.0 / pow(T, 4), 196608.0 / pow(T, 5), -262144.0 / pow(T, 6)};

	bool once = true;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<QuadrupedGaitController>());
	rclcpp::shutdown();
	return 0;
}
