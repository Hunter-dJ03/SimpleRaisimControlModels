#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "quadruped_interfaces/msg/endpoint.hpp"
#include <Eigen/Dense>
#include <Eigen/QR>

class QuadrupedLegController : public rclcpp::Node
{
public:
	QuadrupedLegController() : Node("quadruped_controller")
	{
		// Setup ROS2 parameter time step for simulation, timers and models
		time_step_ms = this->declare_parameter<float>("time_step_ms", 1.0);
		init_pos = this->declare_parameter<std::vector<double>>("joint_initial_positions", std::vector<double>{});
		link_lengths = this->declare_parameter<std::vector<double>>("link_lengths", std::vector<double>{});

		legJointPosition.resize(4);
		legJointVelocity = std::vector<Eigen::Vector3d>(4, Eigen::Vector3d::Zero());
		footPosition = std::vector<Eigen::Vector3d>(4, Eigen::Vector3d::Zero());
		footPositionActual = std::vector<Eigen::Vector3d>(4, Eigen::Vector3d::Zero());

		// Fill legJointPosition from init_pos
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
		}

		joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
			"joint_states", 10,
			std::bind(&QuadrupedLegController::jointStateCallback, this, std::placeholders::_1));

		desired_control_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_desired_control", 10);

		endpoint_publisher_ = this->create_publisher<quadruped_interfaces::msg::Endpoint>("endpoint", 10);

		RCLCPP_INFO(this->get_logger(), "Quadruped Controller Node started");
	}

private:
	void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
	{

		sensor_msgs::msg::JointState control_effort;
		control_effort.header.stamp = now();
		control_effort.name.resize(dof);
		control_effort.position.resize(dof);
		control_effort.velocity.resize(dof);
		control_effort.effort.resize(dof);

		quadruped_interfaces::msg::Endpoint endpoint_msg;
		endpoint_msg.header.stamp = now();

		/******************   World Control   *****************/

		double vel_x = A0 * cos(omega0 * time); // Desired velocity in x direction
		double vel_y = A1 * cos(omega1 * time); // Desired velocity in y direction
		double vel_z = A2 * sin(omega2 * time); // Desired velocity in z direction

		Eigen::VectorXd desired_velocity(3);
		desired_velocity.setZero();

		for (size_t leg = 0; leg < 4; ++leg)
		{
			if (leg == 0)
			{
				desired_velocity << vel_x, vel_y, vel_z;
			}
			else if (leg == 1)
			{
				desired_velocity << vel_x, vel_y, vel_z;
			}
			else if (leg == 2)
			{
				desired_velocity << vel_x, vel_y, vel_z;
			}
			else if (leg == 3)
			{
				desired_velocity << vel_x, vel_y, vel_z;
			}
			else
			{
				desired_velocity.setZero();
			}

			// unpack your 3‑DOF vectors:
			Eigen::VectorXd q(3), qd(3), qdd(3);
			q << msg->position[0 + leg * 3], msg->position[1 + leg * 3], msg->position[2 + leg * 3];
			qd << msg->velocity[0 + leg * 3], msg->velocity[1 + leg * 3], msg->velocity[2 + leg * 3];

			Eigen::VectorXd NE_Gravity_torques = NE_Dynamics(q, zero3, zero3, -gravity, leg);
			Eigen::VectorXd NE_Ccorcent_torques = NE_Dynamics(q, qdd, zero3, 0, leg);

			Eigen::Vector3d fk = forwardKinematics(q, leg);
			footPositionActual[leg] = fk;
			footPosition[leg] += desired_velocity * time_step_ms / 1000.0;

			Eigen::Vector3d position_error = footPosition[leg] - footPositionActual[leg];
			Eigen::Vector3d corrected_velocity = Kp_cartesian * position_error + desired_velocity;
			Eigen::MatrixXd jacobian = computeJacobian(q, leg);

			auto jacobianPseudoInverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();
			legJointVelocity[leg] = jacobianPseudoInverse * corrected_velocity;
			legJointPosition[leg] += legJointVelocity[leg] * time_step_ms / 1000.0;

			for (int joint = 0; joint < 3; ++joint)
			{
				// legJointPosition(leg, joint) += legJointVelocity(leg, joint) * time_step_ms / 1000.0;
				// control_effort.name[leg * 3 + joint] = msg->name[leg * 3 + joint];
				control_effort.position[leg * 3 + joint] = legJointPosition[leg](joint);
				control_effort.velocity[leg * 3 + joint] = legJointVelocity[leg](joint);
				// control_effort.position[leg * 3 + joint] = init_pos[leg * 3 + joint]; // Set desired position to initial position
				// control_effort.velocity[leg * 3 + joint] = 0;
				control_effort.effort[leg * 3 + joint] = NE_Gravity_torques[joint] + NE_Ccorcent_torques[joint];
			}
		}

		// Holding Position
		// for (int i = 0; i < dof; ++i)
		// {
		// 	// legJointPosition[i] += legJointVelocity[i] * time_step_ms / 1000.0;
		// 	// control_effort.position[i] = legJointPosition[i];
		// 	// control_effort.velocity[i] = legJointVelocity[i];
		// 	control_effort.position[i] = init_pos[i]; // Set desired position to initial position
		// 	control_effort.velocity[i] = 0;			  // Set desired velocity to zero
		// 	control_effort.effort[i] = 0;			  // Set effort to zero
		// }

		desired_control_pub_->publish(control_effort);

		time += time_step_ms / 1000; // increment time by 1 ms per callback

		// Fill in desired position
		// endpoint_msg.desired.x = footPosition.x();
		// endpoint_msg.desired.y = footPosition.y();
		// endpoint_msg.desired.z = footPosition.z();

		// // Fill in actual position
		// endpoint_msg.actual.x = footPositionActual.x();
		// endpoint_msg.actual.y = footPositionActual.y();
		// endpoint_msg.actual.z = footPositionActual.z();

		// // Fill in error (desired - actual)
		// endpoint_msg.error.x = position_error.x();
		// endpoint_msg.error.y = position_error.y();
		// endpoint_msg.error.z = position_error.z();

		endpoint_publisher_->publish(endpoint_msg);
	}

	Eigen::MatrixXd computeJacobian(const Eigen::VectorXd &q,
									const int leg)
	{
		double theta1 = q(0);
		double theta2 = q(1);
		double theta3 = q(2);

		// // Link lengths
		double l1 = link_lengths[0];
		double l2 = link_lengths[1];
		double l3 = link_lengths[2];

		if (leg == 0 || leg == 1)
		{
			l1 *= -1;
		}

		// Compute the Jacobian matrix for the 3-DOF leg (Currently ignored roll, pitch, and yaw)
		Eigen::MatrixXd J(3, 3);
		J << 0,
			l2 * std::sin(theta2) - l3 * std::cos(theta2 + theta3),
			-l3 * std::cos(theta2 + theta3),
			l1 * std::sin(theta1) - l2 * std::cos(theta1) * std::sin(theta2) + l3 * std::cos(theta1) * std::cos(theta2) * std::cos(theta3) - l3 * std::cos(theta1) * std::sin(theta2) * std::sin(theta3),
			-std::sin(theta1) * (l3 * std::sin(theta2 + theta3) + l2 * std::cos(theta2)),
			-l3 * std::sin(theta2 + theta3) * std::sin(theta1),
			l3 * std::cos(theta2) * std::cos(theta3) * std::sin(theta1) - l2 * std::sin(theta1) * std::sin(theta2) - l1 * std::cos(theta1) - l3 * std::sin(theta1) * std::sin(theta2) * std::sin(theta3),
			std::cos(theta1) * (l3 * std::sin(theta2 + theta3) + l2 * std::cos(theta2)),
			l3 * std::sin(theta2 + theta3) * std::cos(theta1);
		// 1,
		// 0,
		// 0,
		// 0,
		// std::cos(theta1),
		// std::cos(theta1),
		// 0,
		// std::sin(theta1),
		// std::sin(theta1),

		return J;
	}

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

		double x0 = 0.0; // Base position in x
		double y0 = 0.0; // Base position in y
		double z0 = 0.0; // Base position in z

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

	Eigen::VectorXd NE_Dynamics(const Eigen::VectorXd &q,
								const Eigen::VectorXd &qd,
								const Eigen::VectorXd &qdd,
								const double g,
								const int leg)
	{
		assert(q.size() == 3 && qd.size() == 3 && qdd.size() == 3);

		// ─── Constants ─────────────────────────────────────────────────────────────
		//  link lengths (m)
		double l1 = link_lengths[0], l2 = link_lengths[1], l3 = link_lengths[2];

		if (leg == 0 || leg == 1)
		{
			l1 *= -1;
		}

		// updated masses (kg)
		const std::array<double, 3> mass = {1.5, 1.952, 0.219992};

		// ─── Trigonometric precomputations ─────────────────────────────────────────
		const double c1 = std::cos(q(0)), s1 = std::sin(q(0));
		const double c2 = std::cos(q(1)), s2 = std::sin(q(1));
		const double c3 = std::cos(q(2)), s3 = std::sin(q(2));

		// ─── Rotation matrices R(i→i−1) ────────────────────────────────────────────
		std::array<Eigen::Matrix3d, 4> R;
		R[0] << c1, -s1, 0,
			s1, c1, 0,
			0, 0, 1; // R01

		R[1] << s2, c2, 0,
			0, 0, -1,
			-c2, s2, 0; // R12 (alpha1 = -pi/2 folded in)

		R[2] << c3, -s3, 0,
			s3, c3, 0,
			0, 0, 1;		// R23

		R[3].setIdentity(); // R3E (end effector)

		// ─── Joint offset vectors oc(i) in each link frame ─────────────────────────
		std::array<Eigen::Vector3d, 4> oc;
		oc[0] = Eigen::Vector3d::Zero();   // from frame0 to frame1
		oc[1] = Eigen::Vector3d(0, l1, 0); // from frame1 to frame2

		// if (leg == 0 || leg == 1)
		// {
		// 	oc[1] *= -1; // Adjust for left legs
		// }

		oc[2] = Eigen::Vector3d(l2, 0, 0);	// from frame2 to frame3
		oc[3] = Eigen::Vector3d(0, -l3, 0); // from frame3 to end

		Eigen::Vector3d pcom0;
		Eigen::Vector3d pcom1;
		Eigen::Vector3d pcom2;

		std::array<Eigen::Matrix3d, 3> I;

		if (leg == 0) // Front Left Leg
		{
			pcom0 << 0.000391481, 0.0100851, -0.00369477;
			pcom1 << 0.07500288, 0.00474222, -0.00461711;
			pcom2 << -0.0069932, -0.20108691, -0.0000484705;

			// I₁ expressed about frame 1
			I[0] << 0.00233403831, -1.250985e-05, -2.28685e-06,
				-1.250985e-05, 0.00120555158, -5.558186e-05,
				-2.28685e-06, -5.558186e-05, 0.00180943675;

			// I₂ expressed about frame 2
			I[1] << 0.00147687636, 0.00150249799, 6.8578831e-04,
				0.00150249799, 0.02752790728, -4.343353e-05,
				6.8578831e-04, -4.343353e-05, 0.02715762933;

			// I₃ expressed about frame 3
			I[2] << 0.00533739913, -1.0343004e-04, 2.839e-08,
				-1.0343004e-04, 5.278181e-05, -1.283e-08,
				2.839e-08, -1.283e-08, 0.00534034752;
		}
		else if (leg == 1) // Back Left Leg
		{
			pcom0 << 0.000391481, 0.0100851, 0.00369477;
			pcom1 << 0.07500288, 0.00474222, -0.00461711;
			pcom2 << -0.0069932, -0.20108691, -0.0000484705;

			// I₁ expressed about frame 1
			I[0] << 0.00233403831, -1.250985e-05, 2.28685e-06,
				-1.250985e-05, 0.00120555158, 5.558186e-05,
				2.28685e-06, 5.558186e-05, 0.00180943675;

			// I₂ expressed about frame 2
			I[1] << 0.00147687636, 0.00150249799, 6.8578831e-04,
				0.00150249799, 0.02752790728, -4.343353e-05,
				6.8578831e-04, -4.343353e-05, 0.02715762933;

			// I₃ expressed about frame 3
			I[2] << 0.00533739913, -1.0343004e-04, -2.839e-08,
				-1.0343004e-04, 5.278181e-05, -1.283e-08,
				-2.839e-08, -1.283e-08, 0.00534034752;
		}
		else if (leg == 2) // Back Right Leg
		{
			pcom0 << 0.000391481, -0.0100851, 0.00369477;
			pcom1 << 0.07500288, 0.00474222, 0.00461711;
			pcom2 << -0.0069932, -0.20108691, 0.0000484705;

			// I₁ expressed about frame 1
			I[0] << 0.00233403831, 1.250985e-05, 2.28685e-06,
				1.250985e-05, 0.00120555158, -5.558186e-05,
				2.28685e-06, -5.558186e-05, 0.00180943675;

			// I₂ expressed about frame 2
			I[1] << 0.00147687636, 0.00150249799, -6.8578831e-04,
				0.00150249799, 0.02752790728, 4.343353e-05,
				-6.8578831e-04, 4.343353e-05, 0.02715762933;

			// I₃ expressed about frame 3
			I[2] << 0.00533739913, -1.0343004e-04, 2.839e-08,
				-1.0343004e-04, 5.278181e-05, -1.283e-08,
				2.839e-08, -1.283e-08, 0.00534034752;
		}
		else if (leg == 3) // Front Right leg
		{
			// Test in single leg
			pcom0 << 0.000391481, -0.0100851, -0.00369477;
			pcom1 << 0.07500288, 0.00474222, 0.00461711;
			pcom2 << -0.0069932, -0.20108691, 0.0000484705;

			// I₁ expressed about frame 1
			I[0] << 0.00233403831, 1.250985e-05, -2.28685e-06,
				1.250985e-05, 0.00120555158, 5.558186e-05,
				-2.28685e-06, 5.558186e-05, 0.00180943675;

			// I₂ expressed about frame 2
			I[1] << 0.00147687636, 0.00150249799, -6.8578831e-04,
				0.00150249799, 0.02752790728, 4.343353e-05,
				-6.8578831e-04, 4.343353e-05, 0.02715762933;

			// I₃ expressed about frame 3
			I[2] << 0.00533739913, -1.0343004e-04, 2.839e-08,
				-1.0343004e-04, 5.278181e-05, -1.283e-08,
				2.839e-08, -1.283e-08, 0.00534034752;
		}

		const std::array<Eigen::Vector3d, 3> pcom = {
			pcom0,
			pcom1,
			pcom2};

		I[0] *= 1e-06; // Convert to kg*m^2
		I[1] *= 1e-06; // Convert to kg*m^2
		I[2] *= 1e-06; // Convert to kg*m^2

		// ─── Newton‐Euler variables ─────────────────────────────────────────────────
		Eigen::Vector3d z0(0, 0, 1);
		std::vector<Eigen::Vector3d> w(4, Eigen::Vector3d::Zero());
		std::vector<Eigen::Vector3d> wd(4, Eigen::Vector3d::Zero());
		std::vector<Eigen::Vector3d> vd(4, Eigen::Vector3d::Zero());
		std::array<Eigen::Vector3d, 3> vdcom;

		// base linear acceleration in frame 0
		vd[0] << g, 0, 0;

		// ─── Forward iteration ─────────────────────────────────────────────────────
		for (int i = 0; i < 3; ++i)
		{
			const Eigen::Matrix3d Rt = R[i].transpose();
			const Eigen::Vector3d wi = w[i];
			const Eigen::Vector3d wdi = wd[i];
			const Eigen::Vector3d vdi = vd[i];

			w[i + 1] = Rt * wi + qd(i) * z0;
			wd[i + 1] = Rt * wdi + qdd(i) * z0 + qd(i) * ((Rt * wi).cross(z0));
			vd[i + 1] = Rt * (vdi + wdi.cross(oc[i]) + wi.cross(wi.cross(oc[i])));
			vdcom[i] = vd[i + 1] + wd[i + 1].cross(pcom[i]) + w[i + 1].cross(w[i + 1].cross(pcom[i]));
		}

		// ─── Backward iteration ────────────────────────────────────────────────────
		std::vector<Eigen::Vector3d> f(4, Eigen::Vector3d::Zero());
		std::vector<Eigen::Vector3d> n(4, Eigen::Vector3d::Zero());
		Eigen::VectorXd tau(3);

		for (int i = 2; i >= 0; --i)
		{
			const Eigen::Matrix3d &Rnext = R[i + 1];
			f[i] = Rnext * f[i + 1] + mass[i] * vdcom[i];
			n[i] = I[i] * wd[i + 1] + w[i + 1].cross(I[i] * w[i + 1]) - f[i].cross(pcom[i]) + Rnext * n[i + 1] + (Rnext * f[i + 1]).cross(pcom[i] - oc[i + 1]);
			tau(i) = n[i].dot(z0);
		}

		return tau;
	}

	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desired_control_pub_;
	rclcpp::Publisher<quadruped_interfaces::msg::Endpoint>::SharedPtr endpoint_publisher_;

	std::vector<double> init_pos;
	std::vector<double> link_lengths;

	std::vector<Eigen::Vector3d> legJointPosition;	 // size 4, each is 3-DOF joint position
	std::vector<Eigen::Vector3d> legJointVelocity;	 // size 4, each is 3-DOF joint velocity
	std::vector<Eigen::Vector3d> footPosition;		 // size 4, each is foot position (x, y, z)
	std::vector<Eigen::Vector3d> footPositionActual; // size 4, actual foot positions (x, y, z)

	Eigen::VectorXd zero3 = Eigen::VectorXd::Zero(3);

	double gravity = -9.81;
	float time_step_ms;
	float time = 0;
	int dof = 12;

	// Waveform A parameters (x)
	double A0 = 0.0;	  // amplitude
	double period0 = 3.0; // period in seconds
	double omega0 = 2.0 * M_PI / period0;

	// Waveform B parameters (y)
	double A1 = 0.3;	  // amplitude
	double period1 = 3.0; // period in seconds
	double omega1 = 2.0 * M_PI / period1;

	// Waveform C parameters (z)
	double A2 = 0.3;	  // amplitude
	double period2 = 3.0; // period in seconds
	double omega2 = 2.0 * M_PI / period2;

	double Kp_cartesian = 0.0;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<QuadrupedLegController>());
	rclcpp::shutdown();
	return 0;
}
