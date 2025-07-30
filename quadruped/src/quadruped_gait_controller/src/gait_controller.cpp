#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "quadruped_interfaces/msg/endpoint.hpp"
#include <Eigen/Dense>
#include <Eigen/QR>

class QuadrupedGaitController : public rclcpp::Node
{
public:
	QuadrupedGaitController() : Node("quadruped_gait_controller")
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

	}

private:

};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<QuadrupedGaitController>());
	rclcpp::shutdown();
	return 0;
}
