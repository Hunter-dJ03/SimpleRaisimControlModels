#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "quadruped_interfaces/msg/endpoint.hpp"
#include "quadruped_interfaces/msg/foot_states.hpp"
#include "quadruped_interfaces/msg/full_body_control_command.hpp"
#include <quadruped_interfaces/srv/set_generalized_coordinate.hpp>


#include <Eigen/Dense>
#include <Eigen/QR>
#include <array>
#include <cmath>

class QuadrupedLegController : public rclcpp::Node
{
public:
	QuadrupedLegController() : Node("quadruped_controller")
	{
		// Setup ROS2 parameter time step for simulation, timers and models
		control_time_step_ms = this->declare_parameter<float>("control_time_step_ms", 1.0);
		init_pos = this->declare_parameter<std::vector<double>>("joint_initial_positions", std::vector<double>{});
		link_lengths = this->declare_parameter<std::vector<double>>("link_lengths", std::vector<double>{});

		// Initialise variables for leg joint positions, velocities, and foot positions
		legJointPosition.resize(4);
		legJointVelocity = std::vector<Eigen::Vector3d>(4, Eigen::Vector3d::Zero());
		footPosition = std::vector<Eigen::Vector3d>(4, Eigen::Vector3d::Zero());
		footPositionActual = std::vector<Eigen::Vector3d>(4, Eigen::Vector3d::Zero());
		q = std::vector<Eigen::Vector3d>(4, Eigen::Vector3d::Zero());
		qd = std::vector<Eigen::Vector3d>(4, Eigen::Vector3d::Zero());
		desired_footPosition = std::vector<Eigen::Vector3d>(4, Eigen::Vector3d::Zero());
		desired_footVelocity = std::vector<Eigen::Vector3d>(4, Eigen::Vector3d::Zero());

		footPositionWalk = std::vector<Eigen::Vector3d>(4, Eigen::Vector3d::Zero());

		qb = Eigen::VectorXd::Zero(6);
		vl = Eigen::VectorXd::Zero(24);
		qld = Eigen::VectorXd::Zero(12);
		ql = Eigen::VectorXd::Zero(12);

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

			// legJointPosition[leg] = inverseKinematics(Eigen::Vector3d(footPositionActual[leg][0] + walkOffset[leg], footPositionActual[leg][1], footPositionActual[leg][2]), leg);

			// footPositionActual[leg] = forwardKinematics(
			// 	legJointPosition[leg],
			// 	leg);

			// std::cout << "Vector: " << footPositionActual[leg].transpose() << std::endl;

			desired_footPosition[leg] = footPositionActual[leg];
			desired_footVelocity[leg] = zero3;

			q[leg] = legJointPosition[leg];

			ql[leg * 3 + 0] = q[leg](0);
			ql[leg * 3 + 1] = q[leg](1);
			ql[leg * 3 + 2] = q[leg](2);
		}

		I[0] << 190521.10058e-6, 0.0, 0.0,
			0.0, 588124.01325e-6, 0.0,
			0.0, 0.0, 769095.07872e-6;
		I[1] << 3880.429e-6, -0.33101e-6, -0.54295e-6,
			-0.33101e-6, 1737.42582e-6, -2.41837e-6,
			-0.54295e-6, -2.41837e-6, 3527.932e-6;
		I[2] << 1548.5157e-6, -2160.22133e-6, 174.745e-6,
			-2160.22133e-6, 43591.04282e-6, 18.95697e-6,
			174.745e-6, 18.95697e-6, 43277.27545e-6;
		I[3] << 6651.29948e-6, -86.04365e-6, 0.14142e-6,
			-86.04365e-6, 64.86598e-6, 0.99056e-6,
			0.14142e-6, 0.99056e-6, 6643.89297e-6;
		I[4] << 3880.429e-6, -0.33101e-6, 0.54295e-6,
			-0.33101e-6, 1737.42582e-6, 2.41837e-6,
			0.54295e-6, 2.41837e-6, 3527.932e-6;
		I[5] << 1548.5157e-6, -2160.22133e-6, 174.745e-6,
			-2160.22133e-6, 43591.04282e-6, 18.95697e-6,
			174.745e-6, 18.95697e-6, 43277.27545e-6;
		I[6] << 6651.29948e-6, -86.04365e-6, 0.14142e-6,
			-86.04365e-6, 64.86598e-6, 0.99056e-6,
			0.14142e-6, 0.99056e-6, 6643.89297e-6;
		I[7] << 3880.429e-6, 0.33101e-6, 0.54295e-6,
			0.33101e-6, 1737.42582e-6, -2.41837e-6,
			0.54295e-6, -2.41837e-6, 3527.932e-6;
		I[8] << 1548.5157e-6, -2160.22133e-6, -174.745e-6,
			-2160.22133e-6, 43591.04282e-6, -18.95697e-6,
			-174.745e-6, -18.95697e-6, 43277.27545e-6;
		I[9] << 6651.29948e-6, -86.04365e-6, 0.14142e-6,
			-86.04365e-6, 64.86598e-6, 0.99056e-6,
			0.14142e-6, 0.99056e-6, 6643.89297e-6;
		I[10] << 3880.429e-6, 0.33101e-6, -0.54295e-6,
			0.33101e-6, 1737.42582e-6, 2.41837e-6,
			-0.54295e-6, 2.41837e-6, 3527.932e-6;
		I[11] << 1548.5157e-6, -2160.22133e-6, -174.745e-6,
			-2160.22133e-6, 43591.04282e-6, -18.95697e-6,
			-174.745e-6, -18.95697e-6, 43277.27545e-6;
		I[12] << 6651.29948e-6, -86.04365e-6, 0.14142e-6,
			-86.04365e-6, 64.86598e-6, 0.99056e-6,
			0.14142e-6, 0.99056e-6, 6643.89297e-6;

		pcom[0] << 0.0e-3, 0.0e-3, 25.086e-3;
		pcom[1] << 0.00027e-3, 12.11336e-3, -0.78754e-3;
		pcom[2] << 29.30799e-3, 1.23559e-3, -3.87239e-3;
		pcom[3] << -6.46276e-3, -165.43468e-3, 0.24478e-3;
		pcom[4] << 0.00027e-3, 12.11336e-3, 0.78754e-3;
		pcom[5] << 29.30799e-3, 1.23559e-3, -3.87239e-3;
		pcom[6] << -6.46276e-3, -165.43468e-3, 0.24478e-3;
		pcom[7] << 0.00027e-3, -12.11336e-3, 0.78754e-3;
		pcom[8] << 29.30799e-3, 1.23559e-3, 3.87239e-3;
		pcom[9] << -6.46276e-3, -165.43468e-3, -0.24478e-3;
		pcom[10] << 0.00027e-3, -12.11336e-3, -0.78754e-3;
		pcom[11] << 29.30799e-3, 1.23559e-3, 3.87239e-3;
		pcom[12] << -6.46276e-3, -165.43468e-3, -0.24478e-3;

		// Set up subscription to encoder feedback for joint states
		joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
			"joint_states", 10,
			std::bind(&QuadrupedLegController::jointStateCallback, this, std::placeholders::_1));

		foot_state_sub_ = this->create_subscription<quadruped_interfaces::msg::FootStates>(
			"foot_states", 10,
			std::bind(&QuadrupedLegController::footStateCallback, this, std::placeholders::_1));

		full_body_command_sub_ = this->create_subscription<quadruped_interfaces::msg::FullBodyControlCommand>(
			"full_body_control_command", 10,
			std::bind(&QuadrupedLegController::fullBodyCommandCallback, this, std::placeholders::_1));

		// Set up publishers for desired control effort
		desired_control_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_desired_control", 10);

		// Set up publisher for endpoint messages
		endpoint_publisher_ = this->create_publisher<quadruped_interfaces::msg::Endpoint>("endpoint", 10);

		// Create timer to update the control commands
		timer_ = rclcpp::create_timer(
			this->get_node_base_interface(),
			this->get_node_timers_interface(),
			this->get_clock(),
			std::chrono::microseconds((int)(control_time_step_ms * 1000)),
			std::bind(&QuadrupedLegController::controlCommands, this));

		set_gc_srv_ = this->create_service<quadruped_interfaces::srv::SetGeneralizedCoordinate>(
			"set_generalized_coordinate",
			std::bind(&QuadrupedLegController::setGcCallback, this, std::placeholders::_1, std::placeholders::_2));

		// Feedback for controller start
		RCLCPP_INFO(this->get_logger(), "Quadruped Controller Node started");

		std::cout << "ql: " << ql.transpose() << std::endl;
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
		sensor_msgs::msg::JointState control_effort;
		control_effort.header.stamp = stamp;
		control_effort.position.resize(dof);
		control_effort.velocity.resize(dof);
		control_effort.effort.resize(dof);

		// Create endpoint message
		quadruped_interfaces::msg::Endpoint endpoint_msg;
		endpoint_msg.header.stamp = stamp;

		// For each leg, calculate the forward kinematics
		for (size_t leg = 0; leg < 4; ++leg)
		{
			footPositionActual[leg] = forwardKinematics(q[leg], leg);
		}

		// qb.setZero(); // Assume body is stationary for now
		// qb[3] = -0.2;
		// vl.setZero(); // Assume feet are stationary for now

		// qld = trajectoryGenerator(q, footPositionActual, qb, vl);
		qld = trajectoryGeneratorLinearOnly(q, footPositionActual, qb, vl);
		ql = ql + qld * (control_time_step_ms / 1000.0);

		for (size_t leg = 0; leg < 4; ++leg)
		{
			// Slice from global vectors
			const Eigen::Vector3d ql_leg = ql.segment<3>(3 * leg);
			const Eigen::Vector3d qld_leg = qld.segment<3>(3 * leg);

			// Keep your per-leg arrays updated (if you use them elsewhere)
			legJointPosition[leg] = ql_leg;
			legJointVelocity[leg] = qld_leg;

			// Optional dynamics (commented for now)
			// Eigen::Vector3d zero3 = Eigen::Vector3d::Zero();
			Eigen::VectorXd NE_Gravity_torques = NE_Dynamics(q[leg], zero3, zero3, -gravity, leg);
			Eigen::VectorXd NE_Ccorcent_torques = NE_Dynamics(q[leg], qd[leg], zero3, 0, leg);

			// Populate control effort message for this leg
			for (int joint = 0; joint < 3; ++joint)
			{
				const int idx = static_cast<int>(leg) * 3 + joint;
				control_effort.position[idx] = ql_leg(joint);
				control_effort.velocity[idx] = qld_leg(joint);
				control_effort.effort[idx] = NE_Gravity_torques[joint] + NE_Ccorcent_torques[joint];
				// control_effort.effort[idx] = 0.0; // simple for now
			}
		}

		// Publish the control effort for the desired joint states
		desired_control_pub_->publish(control_effort);

		// Fill in desired position
		endpoint_msg.desired.x = footPosition[0].x();
		endpoint_msg.desired.y = footPosition[0].y();
		endpoint_msg.desired.z = footPosition[0].z();

		// Fill in actual position
		endpoint_msg.actual.x = footPositionActual[0].x();
		endpoint_msg.actual.y = footPositionActual[0].y();
		endpoint_msg.actual.z = footPositionActual[0].z();

		endpoint_msg.error.x = footPosition[0].x() - footPositionActual[0].x();
		endpoint_msg.error.y = footPosition[0].y() - footPositionActual[0].y();
		endpoint_msg.error.z = footPosition[0].z() - footPositionActual[0].z();

		endpoint_publisher_->publish(endpoint_msg);
	}

	/*
	 * Callback that updates the current joint states based on encoder feedback.
	 * Receives current joint states and saves the most recent to internal variables.
	 *
	 * @param msg The message containing the joint states.
	 */
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
	 * Callback that updates the desired foot states based on incoming messages.
	 * Receives desired foot positions and velocities and saves the most recent to internal variables.
	 *
	 * @param msg The message containing the desired foot states.
	 */
	void footStateCallback(const quadruped_interfaces::msg::FootStates::SharedPtr msg)
	{
		// Unpack desired foot states from the message
		for (size_t leg = 0; leg < 4; ++leg)
		{
			desired_footPosition[leg] = Eigen::Vector3d(
				msg->desired_positions[leg].x,
				msg->desired_positions[leg].y,
				msg->desired_positions[leg].z);

			desired_footVelocity[leg] = Eigen::Vector3d(
				msg->desired_velocities[leg].x,
				msg->desired_velocities[leg].y,
				msg->desired_velocities[leg].z);
		};

		return;
	};

	/*
	 * Callback that updates the desired foot states based on incoming messages.
	 * Receives desired foot positions and velocities and saves the most recent to internal variables.
	 *
	 * @param msg The message containing the desired foot states.
	 */
	void fullBodyCommandCallback(const quadruped_interfaces::msg::FullBodyControlCommand::SharedPtr msg)
	{
		// Map directly into Eigen vectors
		qb = Eigen::Map<const Eigen::VectorXd>(msg->qb.data(), msg->qb.size());
		vl = Eigen::Map<const Eigen::VectorXd>(msg->vl.data(), msg->vl.size());

		return;
	}

	/*
	 * Computes the Jacobian matrix for a 3-DOF leg based on the joint angles.
	 * Contains the full 6x3 matric however only using linear velocity components
	 *
	 * @param 		Eigen::VectorXd ql;
		ql = trajectoryGenerator()q The joint angles of the leg (3 DOF).
	 * @param leg The index of the leg (0-3).
	 *
	 * @return The Jacobian matrix (3x3) for the leg.
	 */
	Eigen::MatrixXd computeJacobian(const Eigen::Vector3d &q,
									const int leg)
	{
		// Unpack joint angles from the input vector
		double theta1 = q(0);
		double theta2 = q(1);
		double theta3 = q(2);

		// Link lengths
		double l1 = link_lengths[0];
		double l2 = link_lengths[1];
		double l3 = link_lengths[2];

		// Adjust link length based on leg index (for left legs)
		if (leg == 0 || leg == 1)
		{
			l1 *= -1;
		}

		Eigen::VectorXd J1(6);
		J1 << 0,
			l1 * std::sin(theta1) - l2 * std::cos(theta1) * std::sin(theta2) + l3 * std::cos(theta1) * std::cos(theta2) * std::cos(theta3) - l3 * std::cos(theta1) * std::sin(theta2) * std::sin(theta3),
			l3 * std::cos(theta2) * std::cos(theta3) * std::sin(theta1) - l2 * std::sin(theta1) * std::sin(theta2) - l1 * std::cos(theta1) - l3 * std::sin(theta1) * std::sin(theta2) * std::sin(theta3),
			1,
			0,
			0; // Rotation axis for joint 1

		Eigen::VectorXd J2(6);
		J2 << l2 * std::sin(theta2) - l3 * std::cos(theta2 + theta3),
			-std::sin(theta1) * (l3 * std::sin(theta2 + theta3) + l2 * std::cos(theta2)),
			std::cos(theta1) * (l3 * std::sin(theta2 + theta3) + l2 * std::cos(theta2)),
			0,
			std::cos(theta1),
			std::sin(theta1); // Rotation axis for joint 2

		Eigen::VectorXd J3(6);
		J3 << -l3 * std::cos(theta2 + theta3),
			-l3 * std::sin(theta2 + theta3) * std::sin(theta1),
			l3 * std::sin(theta2 + theta3) * std::cos(theta1),
			0,
			std::cos(theta1),
			std::sin(theta1); // Rotation axis for joint 3

		// Compute the Jacobian matrix for the 3-DOF leg (Currently ignored roll, pitch, and yaw)
		Eigen::MatrixXd J(6, 3);
		// J << 0,
		// 	l2 * std::sin(theta2) - l3 * std::cos(theta2 + theta3),
		// 	-l3 * std::cos(theta2 + theta3),
		// 	l1 * std::sin(theta1) - l2 * std::cos(theta1) * std::sin(theta2) + l3 * std::cos(theta1) * std::cos(theta2) * std::cos(theta3) - l3 * std::cos(theta1) * std::sin(theta2) * std::sin(theta3),
		// 	-std::sin(theta1) * (l3 * std::sin(theta2 + theta3) + l2 * std::cos(theta2)),
		// 	-l3 * std::sin(theta2 + theta3) * std::sin(theta1),
		// 	l3 * std::cos(theta2) * std::cos(theta3) * std::sin(theta1) - l2 * std::sin(theta1) * std::sin(theta2) - l1 * std::cos(theta1) - l3 * std::sin(theta1) * std::sin(theta2) * std::sin(theta3),
		// 	std::cos(theta1) * (l3 * std::sin(theta2 + theta3) + l2 * std::cos(theta2)),
		// 	l3 * std::sin(theta2 + theta3) * std::cos(theta1);
		// 1,
		// 0,
		// 0,
		// 0,
		// std::cos(theta1),
		// std::cos(theta1),
		// 0,
		// std::sin(theta1),
		// std::sin(theta1),

		J.col(0) = J1;
		J.col(1) = J2;
		J.col(2) = J3;

		return J;
	}

	/*
	 * Computes the Jacobian matrix for full floating body kinematics.
	 * Contains the full matrix
	 *
	 * @param q The joint angles of the system
	 * @param pawPosition The positions of the feet in world coordinates
	 *
	 * @return The Jacobian matrix (3x3) for the leg.
	 */
	Eigen::MatrixXd computeFullJacobian(const std::vector<Eigen::Vector3d> &q,
										const std::vector<Eigen::Vector3d> &pawPosition)
	{

		std::vector<Eigen::MatrixXd> J_legs(4); // 4 sets of 6x3
		std::vector<Eigen::MatrixXd> J_body(4); // 4 sets of 6x6

		for (size_t leg = 0; leg < 4; ++leg)
		{
			J_legs[leg] = computeJacobian(q[leg], leg);

			J_body[leg] = Eigen::MatrixXd::Zero(6, 6);
			J_body[leg].block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
			J_body[leg].block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();

			Eigen::Matrix3d S;
			S << 0.0, -pawPosition[leg].z(), pawPosition[leg].y(),
				pawPosition[leg].z(), 0.0, -pawPosition[leg].x(),
				-pawPosition[leg].y(), pawPosition[leg].x(), 0.0;

			J_body[leg].block<3, 3>(0, 3) = -S; // Skew-symmetric matrix for cross product
		}

		Eigen::MatrixXd J_full(24, 18);
		J_full.setZero();

		J_full.block<6, 6>(0, 0) = J_body[0];
		J_full.block<6, 6>(6, 0) = J_body[1];
		J_full.block<6, 6>(12, 0) = J_body[2];
		J_full.block<6, 6>(18, 0) = J_body[3];

		J_full.block<6, 3>(0, 6) = J_legs[0];
		J_full.block<6, 3>(6, 9) = J_legs[1];
		J_full.block<6, 3>(12, 12) = J_legs[2];
		J_full.block<6, 3>(18, 15) = J_legs[3];

		return J_full;
	}

	Eigen::VectorXd trajectoryGenerator(const std::vector<Eigen::Vector3d> &q,
										const std::vector<Eigen::Vector3d> &pawPosition,
										const Eigen::VectorXd &qb,
										const Eigen::VectorXd &vl)
	{
		// Get Full Jacobian matric
		Eigen::MatrixXd J_full = computeFullJacobian(q, pawPosition);

		Eigen::MatrixXd Jb = J_full.leftCols(6);   // Body velocity part
		Eigen::MatrixXd Jl = J_full.rightCols(12); // Leg velocity part

		// Joint velocity
		Eigen::VectorXd ql(12);

		ql = Jl.completeOrthogonalDecomposition().pseudoInverse() * (vl - Jb * qb);

		return ql;
	}

	Eigen::VectorXd trajectoryGeneratorLinearOnly(
		const std::vector<Eigen::Vector3d> &q,			 // 4 joint vectors
		const std::vector<Eigen::Vector3d> &pawPosition, // 4 foot positions
		const Eigen::VectorXd &qb,						 // 6x1 body twist [v; w]
		const Eigen::VectorXd &vl)						 // 24x1 desired [v; w] per foot
	{
		// Full Jacobian: 24x18
		Eigen::MatrixXd J_full = computeFullJacobian(q, pawPosition);

		// Split into body and legs
		Eigen::MatrixXd Jb = J_full.leftCols(6);   // 24x6
		Eigen::MatrixXd Jl = J_full.rightCols(12); // 24x12

		// Compact linear-only Jacobians: 12 rows total
		Eigen::MatrixXd Jb_v(12, 6);
		Eigen::MatrixXd Jl_v(12, 12);
	    Jl_v.setZero();

		for (int leg = 0; leg < 4; ++leg)
		{
			int r = 6 * leg;  // start row in full Jacobian
			int rv = 3 * leg; // start row in compact Jacobian
			int c = 3 * leg;  // start col for leg’s joints

			// take only top 3 rows (linear part)
			Jb_v.block<3, 6>(rv, 0) = Jb.block<3, 6>(r, 0);
			Jl_v.block<3, 3>(rv, c) = Jl.block<3, 3>(r, c);
		}
		

		// Extract only linear desired velocities (first 3 per leg)
		Eigen::VectorXd vl_v(12);
		for (int leg = 0; leg < 4; ++leg)
		{
			vl_v.segment<3>(3 * leg) = vl.segment<3>(6 * leg); // take top 3 of each 6
		}

		// Solve least squares
		Eigen::VectorXd ql = Jl_v.completeOrthogonalDecomposition()
								 .solve(vl_v - Jb_v * qb);

		return ql; // 12x1 joint velocities
	}

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

		double x0 = 0.28375; // Base position in x
		double y0 = 0.1540;	 // Base position in y
		// double z0 = -38.5 - 25.0; // Base position in z
		double z0 = 0.025; // Base position in z

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

		double x0 = 0.28375; // Base position in x
		double y0 = 0.1540;	 // Base position in y
		// double z0 = -38.5 - 25.0; // Base position in z
		double z0 = 0.025; // Base position in z

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

	/*
	 * Implemetation of the Newton-Euler dynamics for a 3-DOF leg.
	 *
	 * @param q The joint angles of the leg (3 DOF).
	 * @param qd The joint velocities of the leg (3 DOF).
	 * @param qdd The joint accelerations of the leg (3 DOF).
	 * @param g The gravitational acceleration (negative value).
	 * @param leg The index of the leg (0-3).
	 *
	 * @return The Newton-Euler dynamics vector (3D vector) containing the torques for each joint.
	 */
	Eigen::VectorXd NE_Dynamics(const Eigen::Vector3d &q,
								const Eigen::Vector3d &qd,
								const Eigen::Vector3d &qdd,
								const double g,
								const int leg)
	{
		assert(q.size() == 3 && qd.size() == 3 && qdd.size() == 3);

		// Link lengths
		double l1 = link_lengths[0], l2 = link_lengths[1], l3 = link_lengths[2];

		// Adjust link length based on leg index (for left legs)
		if (leg == 0 || leg == 1)
		{
			l1 *= -1;
		}

		// Precompute useful terms
		const double c1 = std::cos(q(0)), s1 = std::sin(q(0));
		const double c2 = std::cos(q(1)), s2 = std::sin(q(1));
		const double c3 = std::cos(q(2)), s3 = std::sin(q(2));

		// Rotation matrices from joint i to joint i+1
		std::array<Eigen::Matrix3d, 4> R;
		R[0] << c1, -s1, 0,
			s1, c1, 0,
			0, 0, 1; // R01

		R[1] << s2, c2, 0,
			0, 0, -1,
			-c2, s2, 0; // R12 (alpha1 = -pi/2 folded in)

		R[2] << c3, -s3, 0,
			s3, c3, 0,
			0, 0, 1; // R23

		R[3].setIdentity(); // R3E (end effector)

		// Joint offset in each link frame
		std::array<Eigen::Vector3d, 4> oc;
		oc[0] = Eigen::Vector3d::Zero();	// from frame0 to frame1
		oc[1] = Eigen::Vector3d(0, l1, 0);	// from frame1 to frame2
		oc[2] = Eigen::Vector3d(l2, 0, 0);	// from frame2 to frame3
		oc[3] = Eigen::Vector3d(0, -l3, 0); // from frame3 to end

		// Declare variables for centorid of mass (COM) positions and inertia tensors
		// @todo: restructure pcom variable similar to I variable
		// Eigen::Vector3d pcom0;
		// Eigen::Vector3d pcom1;
		// Eigen::Vector3d pcom2;
		// std::array<Eigen::Matrix3d, 3> Il;
		// std::array<Eigen::Vector3d, 3> pcoml;

		// if (leg == 0) // Front Left Leg
		// {
		// 	pcom0 << pcom[1];
		// 	pcom1 << pcom[2];
		// 	pcom2 << pcom[3];

		// 	Il[0] = I[1];
		// 	Il[1] = I[2];
		// 	Il[2] = I[3];
		// pcom0 << 0.000391481, 0.0100851, -0.00369477;
		// pcom1 << 0.07500288, 0.00474222, -0.00461711;
		// pcom2 << -0.0069932, -0.20108691, -0.0000484705;

		// // I₁ expressed about frame 1
		// Il[0] << 0.00233403831, -1.250985e-05, -2.28685e-06,
		// 	-1.250985e-05, 0.00120555158, -5.558186e-05,
		// 	-2.28685e-06, -5.558186e-05, 0.00180943675;

		// // I₂ expressed about frame 2
		// Il[1] << 0.00147687636, 0.00150249799, 6.8578831e-04,
		// 	0.00150249799, 0.02752790728, -4.343353e-05,
		// 	6.8578831e-04, -4.343353e-05, 0.02715762933;

		// // I₃ expressed about frame 3
		// Il[2] << 0.00533739913, -1.0343004e-04, 2.839e-08,
		// 	-1.0343004e-04, 5.278181e-05, -1.283e-08,
		// 	2.839e-08, -1.283e-08, 0.00534034752;
		// }
		// else if (leg == 1) // Back Left Leg
		// {

		// pcom0 << 0.000391481, 0.0100851, 0.00369477;
		// pcom1 << 0.07500288, 0.00474222, -0.00461711;
		// pcom2 << -0.0069932, -0.20108691, -0.0000484705;

		// // I₁ expressed about frame 1
		// Il[0] << 0.00233403831, -1.250985e-05, 2.28685e-06,
		// 	-1.250985e-05, 0.00120555158, 5.558186e-05,
		// 	2.28685e-06, 5.558186e-05, 0.00180943675;

		// // I₂ expressed about frame 2
		// Il[1] << 0.00147687636, 0.00150249799, 6.8578831e-04,
		// 	0.00150249799, 0.02752790728, -4.343353e-05,
		// 	6.8578831e-04, -4.343353e-05, 0.02715762933;

		// // I₃ expressed about frame 3
		// Il[2] << 0.00533739913, -1.0343004e-04, -2.839e-08,
		// 	-1.0343004e-04, 5.278181e-05, -1.283e-08,
		// 	-2.839e-08, -1.283e-08, 0.00534034752;
		// }
		// else if (leg == 2) // Back Right Leg
		// {
		// pcom0 << 0.000391481, -0.0100851, 0.00369477;
		// pcom1 << 0.07500288, 0.00474222, 0.00461711;
		// pcom2 << -0.0069932, -0.20108691, 0.0000484705;

		// // I₁ expressed about frame 1
		// Il[0] << 0.00233403831, 1.250985e-05, 2.28685e-06,
		// 	1.250985e-05, 0.00120555158, -5.558186e-05,
		// 	2.28685e-06, -5.558186e-05, 0.00180943675;

		// // I₂ expressed about frame 2
		// Il[1] << 0.00147687636, 0.00150249799, -6.8578831e-04,
		// 	0.00150249799, 0.02752790728, 4.343353e-05,
		// 	-6.8578831e-04, 4.343353e-05, 0.02715762933;

		// // I₃ expressed about frame 3
		// Il[2] << 0.00533739913, -1.0343004e-04, 2.839e-08,
		// 	-1.0343004e-04, 5.278181e-05, -1.283e-08,
		// 	2.839e-08, -1.283e-08, 0.00534034752;
		// }
		// else if (leg == 3) // Front Right leg
		// {
		// Test in single leg
		// pcom0 << 0.000391481, -0.0100851, -0.00369477;
		// pcom1 << 0.07500288, 0.00474222, 0.00461711;
		// pcom2 << -0.0069932, -0.20108691, 0.0000484705;

		// // I₁ expressed about frame 1
		// Il[0] << 0.00233403831, 1.250985e-05, -2.28685e-06,
		// 	1.250985e-05, 0.00120555158, 5.558186e-05,
		// 	-2.28685e-06, 5.558186e-05, 0.00180943675;

		// // I₂ expressed about frame 2
		// Il[1] << 0.00147687636, 0.00150249799, -6.8578831e-04,
		// 	0.00150249799, 0.02752790728, 4.343353e-05,
		// 	-6.8578831e-04, 4.343353e-05, 0.02715762933;

		// // I₃ expressed about frame 3
		// Il[2] << 0.00533739913, -1.0343004e-04, 2.839e-08,
		// 	-1.0343004e-04, 5.278181e-05, -1.283e-08,
		// 	2.839e-08, -1.283e-08, 0.00534034752;
		// }

		const std::array<Eigen::Vector3d, 3> pcoml = {
			pcom[leg * 3 + 1],
			pcom[leg * 3 + 2],
			pcom[leg * 3 + 3]};

		const std::array<Eigen::Matrix3d, 3> Il = {
			I[leg * 3 + 1],
			I[leg * 3 + 2],
			I[leg * 3 + 3]};

		// Convert inertia tensors from g*cm^2 to kg*m^2
		// Il[0] *= 1e-06;
		// Il[1] *= 1e-06;
		// Il[2] *= 1e-06;

		// Declare Newton Euler variables
		Eigen::Vector3d z0(0, 0, 1);
		std::vector<Eigen::Vector3d> w(4, Eigen::Vector3d::Zero());
		std::vector<Eigen::Vector3d> wd(4, Eigen::Vector3d::Zero());
		std::vector<Eigen::Vector3d> vd(4, Eigen::Vector3d::Zero());
		std::array<Eigen::Vector3d, 3> vdcom;

		// Gravity in frame 0
		vd[0] << g, 0, 0;

		// Neuton Euler forward iteration
		for (int i = 0; i < 3; ++i)
		{
			const Eigen::Matrix3d Rt = R[i].transpose();
			const Eigen::Vector3d wi = w[i];
			const Eigen::Vector3d wdi = wd[i];
			const Eigen::Vector3d vdi = vd[i];

			w[i + 1] = Rt * wi + qd(i) * z0;
			wd[i + 1] = Rt * wdi + qdd(i) * z0 + qd(i) * ((Rt * wi).cross(z0));
			vd[i + 1] = Rt * (vdi + wdi.cross(oc[i]) + wi.cross(wi.cross(oc[i])));
			vdcom[i] = vd[i + 1] + wd[i + 1].cross(pcoml[i]) + w[i + 1].cross(w[i + 1].cross(pcoml[i]));
		}

		// Neuton Euler Backward iteration
		std::vector<Eigen::Vector3d> f(4, Eigen::Vector3d::Zero());
		std::vector<Eigen::Vector3d> n(4, Eigen::Vector3d::Zero());
		Eigen::VectorXd tau(3);

		for (int i = 2; i >= 0; --i)
		{
			const Eigen::Matrix3d &Rnext = R[i + 1];
			f[i] = Rnext * f[i + 1] + mass[i + 1] * vdcom[i];
			n[i] = Il[i] * wd[i + 1] + w[i + 1].cross(Il[i] * w[i + 1]) - f[i].cross(pcoml[i]) + Rnext * n[i + 1] + (Rnext * f[i + 1]).cross(pcoml[i] - oc[i + 1]);
			tau(i) = n[i].dot(z0);
		}

		return tau;
	}

	void setGcCallback(
		const std::shared_ptr<quadruped_interfaces::srv::SetGeneralizedCoordinate::Request> req,
		std::shared_ptr<quadruped_interfaces::srv::SetGeneralizedCoordinate::Response> res)
	{
		// const int expected_dim = robot->getGeneralizedCoordinateDim(); // 19 floating, 12 fixed
		// const size_t n_in = req->q.size();

		// if (static_cast<int>(n_in) != expected_dim)
		// {
		// 	res->ok = false;
		// 	res->message = "Wrong q length. Got " + std::to_string(n_in) +
		// 				   ", expected " + std::to_string(expected_dim) +
		// 				   (fixed_robot_body ? " for fixed base" : " for floating base");
		// 	RCLCPP_WARN(this->get_logger(), "%s", res->message.c_str());
		// 	return;
		// }

		Eigen::VectorXd target(12);
		for (int i = 0; i < 12; ++i)
			target[i] = req->q[i];

		ql = target;

		res->ok = true;
		res->message = "Queued generalized coordinate set.";
		// RCLCPP_INFO(this->get_logger(), "Queued GC of size %d in mode: %s",
		// 			expected_dim, fixed_robot_body ? "fixed" : "floating");
	}

	// Declaration for ROS2 subscriptions and publishers
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
	rclcpp::Subscription<quadruped_interfaces::msg::FootStates>::SharedPtr foot_state_sub_;

	rclcpp::Subscription<quadruped_interfaces::msg::FullBodyControlCommand>::SharedPtr full_body_command_sub_;

	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desired_control_pub_;
	rclcpp::Publisher<quadruped_interfaces::msg::Endpoint>::SharedPtr endpoint_publisher_;

	rclcpp::Service<quadruped_interfaces::srv::SetGeneralizedCoordinate>::SharedPtr set_gc_srv_;
	rclcpp::TimerBase::SharedPtr timer_;

	// Declaration for model parameters and variables
	std::vector<double> init_pos;
	std::vector<double> link_lengths;

	std::vector<Eigen::Vector3d> legJointPosition;	   // size 4, each is 3-DOF joint position
	std::vector<Eigen::Vector3d> legJointVelocity;	   // size 4, each is 3-DOF joint velocity
	std::vector<Eigen::Vector3d> footPosition;		   // size 4, each is foot position (x, y, z)
	std::vector<Eigen::Vector3d> footPositionActual;   // size 4, actual foot positions (x, y, z)
	std::vector<Eigen::Vector3d> q;					   // size 4, actual foot positions (x, y, z)
	std::vector<Eigen::Vector3d> qd;				   // size 4, actual foot positions (x, y, z)
	std::vector<Eigen::Vector3d> desired_footPosition; // size 4, each is desired foot position (x, y, z)
	std::vector<Eigen::Vector3d> desired_footVelocity; // size 4, each is desired foot velocity (x, y, z)

	std::vector<Eigen::Vector3d> footPositionWalk; // size 4, each is foot position (x, y, z)

	Eigen::VectorXd qb;	 // Body velocity (XYZRPY)
	Eigen::VectorXd vl;	 // Desired foot velocity
	Eigen::VectorXd qld; // desired joint velocities (12 DOF)
	Eigen::VectorXd ql;	 // joint torques (12 DOF)

	Eigen::Vector3d zero3 = Eigen::Vector3d::Zero();

	Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d O3 = Eigen::Matrix3d::Zero();

	double gravity = -9.81;
	float control_time_step_ms;
	// float time = 0;Eigen::Vector3d pcomb = {0.0, 0.0, 0.086};
	// Eigen::Matrix3d Ib = {
	// 	190521.10058, 0.0, 0.0,
	// 	0.0, 588124.01325, 0.0,
	// 	0.0, 0.0, 769095.07872
	// };
	int dof = 12;

	// Waveform A parameters (x)
	double A0 = 0.0;	  // amplitude
	double period0 = 6.0; // period in seconds
	double omega0 = 2.0 * M_PI / period0;

	// Waveform B parameters (y)
	double A1 = 0.0;	  // amplitude
	double period1 = 3.0; // period in seconds
	double omega1 = 2.0 * M_PI / period1;

	// Waveform C parameters (z)
	double A2 = 0.0;	  // amplitude
	double period2 = 3.0; // period in seconds
	double omega2 = 2.0 * M_PI / period2;

	double stepLength = 0.3;
	double stepHeight = 0.2;
	double stepFrequency = 2.0;
	double walkOffset[4] = {-stepLength / 2, -stepLength / 6, stepLength / 6, stepLength / 2};

	// Eigen::Vector3d pcomb = {0.0, 0.0, 0.086};

	// Eigen::Matrix3d Ib = {
	// 	190521.10058, 0.0, 0.0,
	// 	0.0, 588124.01325, 0.0,
	// 	0.0, 0.0, 769095.07872
	// };

	std::array<Eigen::Matrix3d, 13> I;
	std::array<Eigen::Vector3d, 13> pcom;
	// Link masses
	const std::array<double, 4> mass = {17.122, 1.952, 2.437, 0.247};
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<QuadrupedLegController>());
	rclcpp::shutdown();
	return 0;
}
