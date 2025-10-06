#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "quadruped_interfaces/msg/endpoint.hpp"
#include "quadruped_interfaces/msg/foot_states.hpp"
#include "quadruped_interfaces/msg/full_body_control_command.hpp"
#include <quadruped_interfaces/srv/set_generalized_coordinate.hpp>

#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/Geometry>
#include <array>
#include <cmath>

struct LegKinematics
{
	Eigen::Vector3d r1;
	Eigen::Vector3d r2;
	Eigen::Vector3d r3;
	Eigen::Vector3d rp;
	Eigen::Vector3d z1;
	Eigen::Vector3d z2;
	Eigen::Vector3d z3;
};

class QuadrupedLegController : public rclcpp::Node
{
public:
	QuadrupedLegController() : Node("quadruped_controller")
	{
		// Setup ROS2 parameter time step for simulation, timers and models
		control_time_step_ms = this->declare_parameter<float>("control_time_step_ms", 1.0);
		init_pos = this->declare_parameter<std::vector<double>>("joint_initial_positions", std::vector<double>{});
		link_lengths = this->declare_parameter<std::vector<double>>("link_lengths", std::vector<double>{});

		// auto kp_param = this->declare_parameter<std::vector<double>>("cartesian_kp", {5000.0, 5000.0, 7000.0});
		auto kp_param = this->declare_parameter<std::vector<double>>("cartesian_kp", {0.0, 0.0, 0.0});
		if (kp_param.size() == 3)
		{
			cartesian_kp_ << kp_param[0], kp_param[1], kp_param[2];
		}
		else
		{
			cartesian_kp_ << 5000.0, 5000.0, 7000.0;
			RCLCPP_WARN(this->get_logger(), "Parameter cartesian_kp must have 3 entries. Using defaults.");
		}

		// auto kd_param = this->declare_parameter<std::vector<double>>("cartesian_kd", {20.0, 20.0, 35.0});
		auto kd_param = this->declare_parameter<std::vector<double>>("cartesian_kd", {0.0, 0.0, 0.0});

		if (kd_param.size() == 3)
		{
			cartesian_kd_ << kd_param[0], kd_param[1], kd_param[2];
		}
		else
		{
			cartesian_kd_ << 20.0, 20.0, 35.0;
			RCLCPP_WARN(this->get_logger(), "Parameter cartesian_kd must have 3 entries. Using defaults.");
		}

		qb = Eigen::VectorXd::Zero(6); // Measured body positions (XYZRPY)
		dqb = Eigen::VectorXd::Zero(6); // Measured body velocitys (XYZRPY)
		qb_ref = Eigen::VectorXd::Zero(6); // Reference body positions (XYZRPY)
		dqb_ref = Eigen::VectorXd::Zero(6); // Reference body velocitys (XYZRPY)

		qp = Eigen::VectorXd::Zero(12);		 // Measured Paw positions (p1 XYZRPY, p2 XYZRPY, ...)
		dqp = Eigen::VectorXd::Zero(12);	 // Measured Paw velocitys (dp1 XYZRPY, dp2 XYZRPY, ...)
		qp_ref = Eigen::VectorXd::Zero(12);	 // Reference Paw positions (p1 XYZRPY, p2 XYZRPY, ...)
		dqp_ref = Eigen::VectorXd::Zero(24); // Reference Paw velocitys (dp1 XYZRPY, dp2 XYZRPY, ...)

		qJ = Eigen::VectorXd::Zero(12);		 // Measured Joint positions (j1, j2, j3, j4, ...)
		dqJ = Eigen::VectorXd::Zero(12);	 // Measured Joint velocitys (l1_hipAA, l1_hipFE, l1_knee, l2_hipAA, ...)
		qJ_ref = Eigen::VectorXd::Zero(12);	 // Reference Joint positions (j1, j2, j3, j4, ...)
		dqJ_ref = Eigen::VectorXd::Zero(12); // Reference Joint velocitys (l1_hipAA, l1_hipFE, l1_knee, l2_hipAA, ...)

		qJ_prev = Eigen::VectorXd::Zero(12);	  // Previous joint velocitys (l1_hipAA, l1_hipFE, l1_knee, l2_hipAA, ...)
		dqJ_prev = Eigen::VectorXd::Zero(12);	  // Previous joint velocitys (l1_hipAA, l1_hipFE, l1_knee, l2_hipAA, ...)
		qJ_ref_prev = Eigen::VectorXd::Zero(12);  // Previous reference joint velocitys (l1_hipAA, l1_hipFE, l1_knee, l2_hipAA, ...)
		dqJ_ref_prev = Eigen::VectorXd::Zero(12); // Previous reference joint velocitys (l1_hipAA, l1_hipFE, l1_knee, l2_hipAA, ...)

		qT_comp = Eigen::VectorXd::Zero(12); // Measured Joint positions (j1, j2, j3, j4, ...)

		leg_constraint.resize(4, true);

		latest_odom_.pose.pose.orientation.w = 1.0;
		latest_odom_.header.frame_id = "odom";
		latest_odom_.child_frame_id = "base_link";

		qJ = Eigen::Map<Eigen::VectorXd>(init_pos.data(), init_pos.size()); // Assign initial joint positions from parameter
		qJ_ref = qJ;
		qJ_prev = qJ;
		qJ_ref_prev = qJ; // Set reference and previous values to initial positions
		trajectory_q_des_ = qJ;

		qp = fullForwardKinematics();
		qp_ref = qp;

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

		odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
			"odom", 10,
			std::bind(&QuadrupedLegController::odometryCallback, this, std::placeholders::_1));

		full_body_command_sub_ = this->create_subscription<quadruped_interfaces::msg::FullBodyControlCommand>(
			"full_body_control_command", 10,
			std::bind(&QuadrupedLegController::fullBodyCommandCallback, this, std::placeholders::_1));

		// Set up publishers for desired control effort
		desired_control_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_desired_control", 10);

		// Set up publishers for endpoint messages
		endpoint_publisher_ = this->create_publisher<quadruped_interfaces::msg::Endpoint>("endpoint", 10);
		endpoint_velocity_publisher_ = this->create_publisher<quadruped_interfaces::msg::Endpoint>("endpoint_velocity", 10);

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

		qb[0] = latest_odom_.pose.pose.position.x;
		qb[1] = latest_odom_.pose.pose.position.y;
		qb[2] = latest_odom_.pose.pose.position.z;
		Eigen::Quaterniond q_body(
			latest_odom_.pose.pose.orientation.w,
			latest_odom_.pose.pose.orientation.x,
			latest_odom_.pose.pose.orientation.y,
			latest_odom_.pose.pose.orientation.z);
		q_body.normalize();
		const Eigen::Vector3d rpy = q_body.toRotationMatrix().eulerAngles(0, 1, 2);
		qb[3] = rpy[0];
		qb[4] = rpy[1];
		qb[5] = rpy[2];

		dqb[0] = latest_odom_.twist.twist.linear.x;
		dqb[1] = latest_odom_.twist.twist.linear.y;
		dqb[2] = latest_odom_.twist.twist.linear.z;
		dqb[3] = latest_odom_.twist.twist.angular.x;
		dqb[4] = latest_odom_.twist.twist.angular.y;
		dqb[5] = latest_odom_.twist.twist.angular.z;

		// Create endpoint message
		quadruped_interfaces::msg::Endpoint endpoint_msg;
		endpoint_msg.header.stamp = stamp;

		quadruped_interfaces::msg::Endpoint endpoint_velocity_msg;
		endpoint_velocity_msg.header.stamp = stamp;

		qp = fullForwardKinematics();

		std::array<Eigen::Matrix3d, 4> leg_jacobians_linear;

		for (int leg = 0; leg < 4; ++leg)
		{
			const int idx = 3 * leg;
			const Eigen::Matrix<double, 6, 3> J_leg = computeJacobian(qJ.segment<3>(idx), leg);
			leg_jacobians_linear[leg] = J_leg.topRows(3);
			dqp.segment<3>(idx).noalias() = leg_jacobians_linear[leg] * dqJ.segment<3>(idx);
			dqp.segment<3>(idx) += dqb_ref.segment<3>(0);
		}

		const double dt = control_time_step_ms / 1000.0;
		if (dqp_ref.size() == 24 && qp_ref.size() == 12)
		{
			for (int leg = 0; leg < 4; ++leg)
			{
				const int qp_idx = 3 * leg;
				const int dqp_idx = 6 * leg;

				qp_ref.segment<3>(qp_idx) -= dqb_ref.segment<3>(0) * dt;
				qp_ref.segment<3>(qp_idx) += dqp_ref.segment<3>(dqp_idx) * dt;

			}
		}

		Eigen::VectorXd qT_ref = Eigen::VectorXd::Zero(12);
		for (int leg = 0; leg < 4; ++leg)
		{
			const int qp_idx = 3 * leg;
			const int dqp_idx = 6 * leg;
			const Eigen::Vector3d pos_error = qp_ref.segment<3>(qp_idx) - qp.segment<3>(qp_idx);
			const Eigen::Vector3d vel_error = dqp_ref.segment<3>(dqp_idx) - dqp.segment<3>(qp_idx);
			const Eigen::Vector3d force = cartesian_kp_.cwiseProduct(pos_error) + cartesian_kd_.cwiseProduct(vel_error);
			qT_ref.segment<3>(qp_idx).noalias() = leg_jacobians_linear[leg].transpose() * force;
		}

		qT_comp = fullNEDynamics();

		for (int i = 0; i < 12; ++i)
		{
			control_effort.position[i] = 0.0;
			control_effort.velocity[i] = 0.0;
			control_effort.effort[i] = qT_ref(i) + qT_comp(i);
		}

		// Publish the control effort for the desired joint states
		desired_control_pub_->publish(control_effort);

		if (qp_ref.size() >= 3 && qp.size() >= 3)
		{
			// Fill in desired position
			endpoint_msg.desired.x = qp_ref[0];
			endpoint_msg.desired.y = qp_ref[1];
			endpoint_msg.desired.z = qp_ref[2];

			// Fill in actual position
			endpoint_msg.actual.x = qp[0];
			endpoint_msg.actual.y = qp[1];
			endpoint_msg.actual.z = qp[2];

			endpoint_msg.error.x = endpoint_msg.desired.x - endpoint_msg.actual.x;
			endpoint_msg.error.y = endpoint_msg.desired.y - endpoint_msg.actual.y;
			endpoint_msg.error.z = endpoint_msg.desired.z - endpoint_msg.actual.z;
		}

		if (dqp_ref.size() >= 3 && dqp.size() >= 3)
		{
			endpoint_velocity_msg.desired.x = dqp_ref[0];
			endpoint_velocity_msg.desired.y = dqp_ref[1];
			endpoint_velocity_msg.desired.z = dqp_ref[2];

			endpoint_velocity_msg.actual.x = dqp[0];
			endpoint_velocity_msg.actual.y = dqp[1];
			endpoint_velocity_msg.actual.z = dqp[2];

			endpoint_velocity_msg.error.x = endpoint_velocity_msg.desired.x - endpoint_velocity_msg.actual.x;
			endpoint_velocity_msg.error.y = endpoint_velocity_msg.desired.y - endpoint_velocity_msg.actual.y;
			endpoint_velocity_msg.error.z = endpoint_velocity_msg.desired.z - endpoint_velocity_msg.actual.z;
		}

		endpoint_publisher_->publish(endpoint_msg);
		endpoint_velocity_publisher_->publish(endpoint_velocity_msg);
	}

	/*
	 * Callback that updates the current joint states based on encoder feedback.
	 * Receives current joint states and saves the most recent to internal variables.
	 *
	 * @param msg The message containing the joint states.
	 */
	void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
	{

		qJ = Eigen::VectorXd::Map(msg->position.data(), 12);
		dqJ = Eigen::VectorXd::Map(msg->velocity.data(), 12);

		return; // This function is not used in this controller
	};

	void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
	{
		latest_odom_ = *msg;
	}

	/*
	 * Callback that updates the desired foot states based on incoming messages.
	 * Receives desired foot positions and velocities and saves the most recent to internal variables.
	 *
	 * @param msg The message containing the desired foot states.
	 */
	void fullBodyCommandCallback(const quadruped_interfaces::msg::FullBodyControlCommand::SharedPtr msg)
	{
			// Map directly into Eigen vectors
			dqb_ref = Eigen::Map<const Eigen::VectorXd>(msg->dqb_ref.data(), msg->dqb_ref.size());
			dqp_ref = Eigen::Map<const Eigen::VectorXd>(msg->dqp_ref.data(), msg->dqp_ref.size());
			if (!msg->leg_constraint.empty())
			{
				leg_constraint.assign(msg->leg_constraint.begin(), msg->leg_constraint.end());
			}

		return;
	}

	/*
	 * Computes the Jacobian matrix for a 3-DOF leg based on the joint angles.
	 * Contains the full 6x3 matric however only using linear velocity components
	 *
	 * @param q: The joint angles of the leg (3 DOF).
	 * @param leg The index of the leg (0-3).
	 *
	 * @return The Jacobian matrix (3x3) for the leg.
	 */
	Eigen::MatrixXd computeJacobian(const Eigen::Vector3d &q,
									const int leg)
	{
		const auto kin = computeLegForwardKinematics(q, leg);
		const Eigen::Matrix<double, 6, 3> J_leg = legJacobianFromKinematics(kin);
		return J_leg;
	}

	/*
	 * Computes the Jacobian matrix for full floating body kinematics.
	 * Contains the full matrix
	 *
	 * @param q The joint angles of the system
	 *
	 * @return The Jacobian matrix (3x3) for the leg.
	 */
	Eigen::MatrixXd computeFullJacobian()
	{
		Eigen::MatrixXd J_full = Eigen::MatrixXd::Zero(24, 18);

		for (size_t leg = 0; leg < 4; ++leg)
		{
			const int row = static_cast<int>(6 * leg);
			const int col = static_cast<int>(6 + 3 * leg);
			const auto kin = computeLegForwardKinematics(qJ.segment<3>(3 * leg), static_cast<int>(leg));

			leg_kinematics_[leg] = kin;

			J_full.block<3, 3>(row, 0) = Eigen::Matrix3d::Identity();
			J_full.block<3, 3>(row, 3) = -skew(kin.rp);
			J_full.block<3, 3>(row + 3, 3) = Eigen::Matrix3d::Identity();

			const Eigen::Matrix<double, 6, 3> J_leg = legJacobianFromKinematics(kin);
			J_full.block<6, 3>(row, col) = J_leg;
		}

		return J_full;
	}

	Eigen::VectorXd trajectoryGenerator(
		const Eigen::VectorXd &dqb_ref,
		const Eigen::VectorXd &dqp_ref)
	{
		(void)dqb_ref;
		(void)dqp_ref;

		const double time = this->get_clock()->now().seconds();
		const double dt = control_time_step_ms / 1000.0;

		if (!trajectory_initialized_ || trajectory_q_des_.size() != dof || time <= 1e-12)
		{
			trajectory_q_des_ = qJ;
			trajectory_initialized_ = true;
		}

		std::array<LegKinematics, 4> legs{};
		for (size_t leg = 0; leg < 4; ++leg)
		{
			legs[leg] = computeLegForwardKinematics(qJ.segment<3>(3 * leg), static_cast<int>(leg));
		}

		const Eigen::Matrix<double, 12, 18> Jp = feetPositionJacobian(legs);
		const Eigen::Matrix<double, 12, 6> Jb = Jp.block<12, 6>(0, 0);
		const Eigen::Matrix<double, 12, 12> Jq = Jp.block<12, 12>(0, 6);

		const double f = 0.25;
		const double amp = 0.1;
		Eigen::Vector3d vB_des(0.0, amp * std::sin(2.0 * M_PI * f * time), 0.0);
		Eigen::Vector3d wB_des = Eigen::Vector3d::Zero();
		Eigen::Matrix<double, 6, 1> dqb;
		dqb << vB_des, wB_des;

		const Eigen::Matrix<double, 12, 1> rhs = Jb * dqb;
		const double lambda = 1e-3;
		const Eigen::Matrix<double, 12, 12> I = Eigen::Matrix<double, 12, 12>::Identity();
		const Eigen::Matrix<double, 12, 1> dq_des = -(Jq.transpose() * Jq + lambda * lambda * I).ldlt().solve(Jq.transpose() * rhs);

		if (trajectory_q_des_.size() != dof)
		{
			trajectory_q_des_ = qJ;
		}

		trajectory_q_des_.noalias() += dq_des * dt;
		leg_kinematics_ = legs;

		return dq_des;
	}

	Eigen::VectorXd trajectoryGeneratorLinearOnly() // 24x1 desired [v; w] per foot
	{
		// Full Jacobian: 24x18
		Eigen::MatrixXd J_full = computeFullJacobian();

		// Split into body and legs
		Eigen::MatrixXd Jb = J_full.leftCols(6);   // 24x6
		Eigen::MatrixXd Jl = J_full.rightCols(12); // 24x12

		// Compact linear-only Jacobians: 12 rows total
		Eigen::MatrixXd Jb_v(12, 6);
		Eigen::MatrixXd Jl_v(12, 12);
		Jb_v.setZero();
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
			vl_v.segment<3>(3 * leg) = dqp_ref.segment<3>(6 * leg); // take top 3 of each 6
		}

		// Solve least squares
		// Eigen::VectorXd qld_calc = Jl_v.completeOrthogonalDecomposition().pseudoInverse() * (vl_v - Jb_v * dqb_ref);

		Eigen::VectorXd qld_calc = Jl_v.inverse() * (vl_v - Jb_v * dqb_ref);

		// double lambda = 1e-5;
		// Eigen::MatrixXd I = Eigen::MatrixXd::Identity(Jl.cols(), Jl.cols());
		// Eigen::VectorXd qld_calc = (Jl.transpose() * Jl + lambda * lambda * I)
		// 			   .ldlt()
		// 			   .solve(Jl.transpose() * (dqp_ref - Jb * dqb_ref));

		return qld_calc; // 12x1 joint velocities
	}

	static Eigen::Matrix3d skew(const Eigen::Vector3d &a)
	{
		Eigen::Matrix3d A;
		A << 0.0, -a.z(), a.y(),
			 a.z(), 0.0, -a.x(),
			-a.y(), a.x(), 0.0;
		return A;
	}

	Eigen::Vector3d legBasePosition(int leg) const
	{
		switch (leg)
		{
		case 0:
			return Eigen::Vector3d(0.28375, 0.1540, 0.025);
		case 1:
			return Eigen::Vector3d(-0.28375, 0.1540, 0.025);
		case 2:
			return Eigen::Vector3d(-0.28375, -0.1540, 0.025);
		case 3:
			return Eigen::Vector3d(0.28375, -0.1540, 0.025);
		default:
			return Eigen::Vector3d::Zero();
		}
	}

	double hipOffsetSign(int leg) const
	{
		return (leg < 2) ? 1.0 : -1.0;
	}

	LegKinematics computeLegForwardKinematics(const Eigen::Vector3d &q_leg, int leg) const
	{
		const double d1 = 0.1345;
		const double d2 = 0.37034477;
		const double d3 = 0.36328592;

		const Eigen::Vector3d r_bl = legBasePosition(leg);
		const double d1_sgn = hipOffsetSign(leg);

		Eigen::Matrix4d TB0;
		TB0 <<
			0, 0, 1, r_bl.x(),
			0, -1, 0, r_bl.y(),
			1, 0, 0, r_bl.z(),
			0, 0, 0, 1;

		Eigen::Matrix4d T01;
		const double th1 = q_leg(0);
		const double c1 = std::cos(th1);
		const double s1 = std::sin(th1);
		T01 <<
			c1, -s1, 0, 0,
			s1, c1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;

		Eigen::Matrix4d T12;
		const double th2 = q_leg(1) - M_PI / 2.0;
		const double c2 = std::cos(th2);
		const double s2 = std::sin(th2);
		T12 <<
			c2, -s2, 0, 0,
			0, 0, -1, -d1 * d1_sgn,
			s2, c2, 0, 0,
			0, 0, 0, 1;

		Eigen::Matrix4d T23;
		const double th3 = q_leg(2);
		const double c3 = std::cos(th3);
		const double s3 = std::sin(th3);
		T23 <<
			c3, -s3, 0, d2,
			s3, c3, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;

		Eigen::Matrix4d T3P;
		T3P <<
			1, 0, 0, 0,
			0, 1, 0, -d3,
			0, 0, 1, 0,
			0, 0, 0, 1;

		const Eigen::Matrix4d TB1 = TB0 * T01;
		const Eigen::Matrix4d TB2 = TB1 * T12;
		const Eigen::Matrix4d TB3 = TB2 * T23;
		const Eigen::Matrix4d TBP = TB3 * T3P;

		LegKinematics kin;
		kin.r1 = TB1.block<3, 1>(0, 3);
		kin.r2 = TB2.block<3, 1>(0, 3);
		kin.r3 = TB3.block<3, 1>(0, 3);
		kin.rp = TBP.block<3, 1>(0, 3);

		kin.z1 = TB1.block<3, 3>(0, 0).col(2);
		kin.z2 = TB2.block<3, 3>(0, 0).col(2);
		kin.z3 = TB3.block<3, 3>(0, 0).col(2);

		return kin;
	}

	Eigen::Matrix<double, 6, 3> legJacobianFromKinematics(const LegKinematics &kin) const
	{
		Eigen::Matrix<double, 6, 3> J;
		J.block<3, 1>(0, 0) = kin.z1.cross(kin.rp - kin.r1);
		J.block<3, 1>(0, 1) = kin.z2.cross(kin.rp - kin.r2);
		J.block<3, 1>(0, 2) = kin.z3.cross(kin.rp - kin.r3);
		J.block<3, 1>(3, 0) = kin.z1;
		J.block<3, 1>(3, 1) = kin.z2;
		J.block<3, 1>(3, 2) = kin.z3;
		return J;
	}

	Eigen::Matrix<double, 12, 18> feetPositionJacobian(const std::array<LegKinematics, 4> &legs) const
	{
		Eigen::Matrix<double, 12, 18> Jp;
		Jp.setZero();

		for (size_t leg = 0; leg < 4; ++leg)
		{
			const int row = static_cast<int>(3 * leg);
			const int col = static_cast<int>(6 + 3 * leg);

			Jp.block<3, 3>(row, 0) = Eigen::Matrix3d::Identity();
			Jp.block<3, 3>(row, 3) = -skew(legs[leg].rp);
			Jp.block<3, 1>(row, col + 0) = legs[leg].z1.cross(legs[leg].rp - legs[leg].r1);
			Jp.block<3, 1>(row, col + 1) = legs[leg].z2.cross(legs[leg].rp - legs[leg].r2);
			Jp.block<3, 1>(row, col + 2) = legs[leg].z3.cross(legs[leg].rp - legs[leg].r3);
		}

		return Jp;
	}

	Eigen::VectorXd forwardKinematics(const Eigen::Vector3d &q,
									  const int leg)
	{
		const auto kin = computeLegForwardKinematics(q, leg);
		Eigen::VectorXd position(3);
		position = kin.rp;
		return position;
	}

	Eigen::VectorXd fullForwardKinematics()
	{
		Eigen::VectorXd pawPosition(12);

		for (size_t leg = 0; leg < 4; ++leg)
		{
			leg_kinematics_[leg] = computeLegForwardKinematics(qJ.segment<3>(3 * leg), static_cast<int>(leg));
			pawPosition.segment<3>(3 * leg) = leg_kinematics_[leg].rp;
		}

		return pawPosition;
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

		// Link geometry
		double l1 = link_lengths[0];
		const double l2 = link_lengths[1];
		const double l3 = link_lengths[2];
		if (leg == 0 || leg == 1)
		{
			l1 *= -1.0;
		}

		// Precompute rotation terms
		const double c1 = std::cos(q(0)), s1 = std::sin(q(0));
		const double c2 = std::cos(q(1)), s2 = std::sin(q(1));
		const double c3 = std::cos(q(2)), s3 = std::sin(q(2));

		std::array<Eigen::Matrix3d, 4> R;
		R[0] << c1, -s1, 0,
			s1, c1, 0,
			0, 0, 1; // R01
		R[1] << s2, c2, 0,
			0, 0, -1,
			-c2, s2, 0; // R12
		R[2] << c3, -s3, 0,
			s3, c3, 0,
			0, 0, 1; // R23
		R[3].setIdentity(); // R3E

		std::array<Eigen::Vector3d, 4> oc;
		oc[0] = Eigen::Vector3d::Zero();
		oc[1] = Eigen::Vector3d(0.0, l1, 0.0);
		oc[2] = Eigen::Vector3d(l2, 0.0, 0.0);
		oc[3] = Eigen::Vector3d(0.0, -l3, 0.0);

		const std::array<Eigen::Vector3d, 3> pcoml = {
			pcom[leg * 3 + 1],
			pcom[leg * 3 + 2],
			pcom[leg * 3 + 3]};

		const std::array<Eigen::Matrix3d, 3> Il = {
			I[leg * 3 + 1],
			I[leg * 3 + 2],
			I[leg * 3 + 3]};

		Eigen::Vector3d z0(0.0, 0.0, 1.0);

		std::array<Eigen::Vector3d, 4> w{};
		std::array<Eigen::Vector3d, 4> wd{};
		std::array<Eigen::Vector3d, 4> v{};
		std::array<Eigen::Vector3d, 4> vcom{};
		v[0] << g, 0.0, 0.0;

		for (int idx = 1; idx <= 3; ++idx)
		{
			const Eigen::Matrix3d Rt = R[idx - 1].transpose();
			// const Eigen::Vector3d &wi_prev = w[idx - 1];
			// const Eigen::Vector3d &wdi_prev = wd[idx - 1];
			const Eigen::Vector3d &o_i = oc[idx - 1];     // ^i o_{i-1}
			const Eigen::Vector3d &p_com_i = pcoml[idx - 1]; // ^i p_{CoM_i}
			

			const double qd_i = qd(idx - 1);
			const double qdd_i = qdd(idx - 1);

			w[idx]   = Rt * (w[idx - 1]  + qd_i  * z0);
			wd[idx]  = Rt * (wd[idx - 1] + qdd_i * z0 + qd_i * w[idx - 1].cross(z0));
			v[idx]   = Rt * (v[idx - 1]  + wd[idx - 1].cross(o_i) + w[idx - 1].cross(w[idx - 1].cross(o_i)));
			vcom[idx]= v[idx] + wd[idx].cross(p_com_i) + w[idx].cross(w[idx].cross(p_com_i));
			
		}

		std::array<Eigen::Vector3d, 4> f{};
		std::array<Eigen::Vector3d, 4> n{};

		Eigen::VectorXd tau(3);

		for (int idx = 3; idx >= 1; --idx)
		{
			const int link = idx - 1;
			const Eigen::Matrix3d &Rnext = R[idx];
			const Eigen::Vector3d f_next_in_curr = Rnext * f[idx];
			const Eigen::Vector3d n_next_in_curr = Rnext * n[idx];

			f[link] = f_next_in_curr + mass[link + 1] * vcom[idx];
			n[link] = Il[link] * wd[idx] + w[idx].cross(Il[link] * w[idx]) - f[link].cross(pcoml[link]) + n_next_in_curr + f_next_in_curr.cross(pcoml[link] - oc[idx]);
			tau(link) = n[link].dot(z0);
		}

		return tau;
	}

	Eigen::VectorXd fullNEDynamics()
	{
		Eigen::VectorXd tau(12);

		for (size_t leg = 0; leg < 4; ++leg)
		{
			tau.segment<3>(3 * leg) = NE_Dynamics(qJ.segment<3>(3 * leg), dqJ.segment<3>(3 * leg), zero3, -gravity, leg);
		}

		return tau;
	}

	void setGcCallback(
		const std::shared_ptr<quadruped_interfaces::srv::SetGeneralizedCoordinate::Request> req,
		std::shared_ptr<quadruped_interfaces::srv::SetGeneralizedCoordinate::Response> res)
	{

		Eigen::VectorXd target(12);
		for (int i = 0; i < 12; ++i)
			target[i] = req->q[i];

		qJ_ref = target;

		res->ok = true;
		res->message = "Queued generalized coordinate set.";
	}

	// Declaration for ROS2 subscriptions and publishers
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

	rclcpp::Subscription<quadruped_interfaces::msg::FullBodyControlCommand>::SharedPtr full_body_command_sub_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desired_control_pub_;
	rclcpp::Publisher<quadruped_interfaces::msg::Endpoint>::SharedPtr endpoint_publisher_;
	rclcpp::Publisher<quadruped_interfaces::msg::Endpoint>::SharedPtr endpoint_velocity_publisher_;

	rclcpp::Service<quadruped_interfaces::srv::SetGeneralizedCoordinate>::SharedPtr set_gc_srv_;
	rclcpp::TimerBase::SharedPtr timer_;

	nav_msgs::msg::Odometry latest_odom_;

	// Declaration for model parameters and variables
	std::vector<double> init_pos;
	std::vector<double> link_lengths;

	Eigen::VectorXd qb; // Measured body positions (XYZRPY)
	Eigen::VectorXd dqb; // Measured body velocitys (XYZRPY)
	Eigen::VectorXd qb_ref; // Reference body positions (XYZRPY)
	Eigen::VectorXd dqb_ref; // Reference body velocitys (XYZRPY)

	Eigen::VectorXd qp;		 // Measured Paw positions (p1 XYZRPY, p2 XYZRPY, ...)
	Eigen::VectorXd dqp;	 // Measured Paw velocitys (p1 XYZRPY, p2 XYZRPY, ...)
	Eigen::VectorXd qp_ref;	 // Reference Paw positions (p1 XYZRPY, p2 XYZRPY, ...)
	Eigen::VectorXd dqp_ref; // Reference Paw velocitys (p1 XYZRPY, p2 XYZRPY, ...)

	Eigen::VectorXd qJ;		 // Measured Joint positions (l1_hipAA, l1_hipFE, l1_knee, l2_hipAA, ...)
	Eigen::VectorXd dqJ;	 // Measured Joint velocitys (l1_hipAA, l1_hipFE, l1_knee, l2_hipAA, ...)
	Eigen::VectorXd qJ_ref;	 // Reference Joint positions (l1_hipAA, l1_hipFE, l1_knee, l2_hipAA, ...)
	Eigen::VectorXd dqJ_ref; // Reference Joint velocitys (l1_hipAA, l1_hipFE, l1_knee, l2_hipAA, ...)

	Eigen::VectorXd qJ_prev;	  // Previous joint velocitys (l1_hipAA, l1_hipFE, l1_knee, l2_hipAA, ...)
	Eigen::VectorXd dqJ_prev;	  // Previous joint velocitys (l1_hipAA, l1_hipFE, l1_knee, l2_hipAA, ...)
	Eigen::VectorXd qJ_ref_prev;  // Previous reference joint velocitys (l1_hipAA, l1_hipFE, l1_knee, l2_hipAA, ...)
	Eigen::VectorXd dqJ_ref_prev; // Previous reference joint velocitys (l1_hipAA, l1_hipFE, l1_knee, l2_hipAA, ...)

	Eigen::VectorXd qT_comp; // Reference joint torques (l1_hipAA, l1_hipFE, l1_knee, l2_hipAA, ...)
	Eigen::VectorXd trajectory_q_des_;
	bool trajectory_initialized_ = false;

	std::vector<bool> leg_constraint;	 // Inclusion of leg in constraint matrix (1 = included, 0 = not)

	Eigen::Vector3d cartesian_kp_;
	Eigen::Vector3d cartesian_kd_;

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

	std::array<LegKinematics, 4> leg_kinematics_{};
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
