#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "quadruped_leg_interfaces/msg/endpoint.hpp"
#include <Eigen/Dense>
#include <Eigen/QR>

#include <array>
#include <vector>
#include <cassert>

class QuadrupedLegController : public rclcpp::Node
{
public:
    QuadrupedLegController() : Node("quadruped_leg_controller")
    {
        // Setup ROS2 parameter time step for simulation, timers and models
        time_step_ms = this->declare_parameter<float>("time_step_ms", 1.0);
        init_pos = this->declare_parameter<std::vector<double>>("joint_initial_positions", std::vector<double>{});
        link_lengths = this->declare_parameter<std::vector<double>>("link_lengths", std::vector<double>{});

        // Create initial joint positions and velocities
        legJointPosition = Eigen::Map<Eigen::VectorXd>(init_pos.data(), init_pos.size());
        legJointVelocity = Eigen::VectorXd::Zero(3);

        // Calculate starting foot position
        footPositionActual = forwardKinematics(init_pos[0], init_pos[1], init_pos[2]);
        footPosition = footPositionActual; // Initialize foot position to actual position

        // Subscriber for retreiveing joint states
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            std::bind(&QuadrupedLegController::jointStateCallback, this, std::placeholders::_1));

        // Publisher for sending desired control efforts
        desired_control_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_desired_control", 10);

        // Publisher for logging foot endpoint position
        endpoint_publisher_ = this->create_publisher<quadruped_leg_interfaces::msg::Endpoint>("endpoint", 10);

        RCLCPP_INFO(this->get_logger(), "Quadruped Leg Control Node started");
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Create a control effort message of size DOF
        sensor_msgs::msg::JointState control_effort;
        control_effort.header.stamp = now();
        control_effort.name.resize(dof);
        control_effort.position.resize(dof);
        control_effort.velocity.resize(dof);
        control_effort.effort.resize(dof);

        // Create foot endpoint message
        quadruped_leg_interfaces::msg::Endpoint endpoint_msg;
        endpoint_msg.header.stamp = now();

        /******************   World Control   *****************/

        double vel_x = A0 * cos(omega0 * time); // Desired velocity in x direction
        double vel_y = A1 * cos(omega1 * time); // Desired velocity in y direction
        double vel_z = A2 * sin(omega2 * time); // Desired velocity in z direction

        // Uncomment the following lines if you want to control roll, pitch, and yaw velocities (doesnt work with current jacobian implementation)
        // double vel_roll = 0.0; // Desired roll velocity
        // double vel_pitch = 0.0; // Desired pitch velocity
        // double vel_yaw = 0.0; // Desired yaw velocity

        // Eigen::VectorXd desired_velocity(6);
        // desired_velocity << vel_x, vel_y, vel_z, vel_roll, vel_pitch, vel_yaw;

        Eigen::VectorXd desired_velocity(3);
        desired_velocity << vel_x, vel_y, vel_z; // Only considering 3D Cartesian space for the leg

        // Calculate the current foot positin based on current joint positions
        footPositionActual = forwardKinematics(msg->position[0], msg->position[1], msg->position[2]);

        // Calculate the desired foot position based on the desired velocity
        footPosition += desired_velocity * time_step_ms / 1000.0;

        // Cartesian position error
        Eigen::VectorXd position_error = footPosition - footPositionActual;

        // PD control in Cartesian space (only P for now)
        Eigen::VectorXd corrected_velocity = Kp_cartesian * position_error + desired_velocity;

        // Calculate required joint and joint velocities using the Jacobian
        Eigen::MatrixXd jacobian = computeJacobian(msg->position[0], msg->position[1], msg->position[2]);
        auto jacobianPseudoInverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::VectorXd legJointVelocity = jacobianPseudoInverse * corrected_velocity;
        // Update desired leg joint positions
        legJointPosition += legJointVelocity * time_step_ms / 1000.0;

        // unpack your 3‑DOF vectors:
        Eigen::VectorXd q(3), qd(3), qdd(3);
        q << msg->position[0], msg->position[1], msg->position[2];
        qd << msg->velocity[0], msg->velocity[1], msg->velocity[2]; // or use your desired qd_ref
        qdd << 0.0, 0.0, 0.0;

        // 1) Coriolis + Centrifugal (set g=0, zero accel, keep qd):
        // Eigen::VectorXd Ccorcent = NE_Dynamics(q, qd, qdd, 0.0);

        // 2) Gravity        (set g=9.81, zero vel+accel):
        Eigen::VectorXd zero3 = Eigen::VectorXd::Zero(3);
        Eigen::VectorXd NE_G = NE_Dynamics(q, zero3, zero3, -gravity);
        Eigen::VectorXd LD_G = LD_gravity(q);

        RCLCPP_INFO_STREAM(this->get_logger(), "Joint Position:" << q.transpose());
        RCLCPP_INFO_STREAM(this->get_logger(), "NE: Gravity Compensation:" << NE_G.transpose());
        RCLCPP_INFO_STREAM(this->get_logger(), "LD: Gravity Compensation:" << LD_G.transpose());

        // NE_G(0) *= -1;

        // For each DOF
        for (int i = 0; i < dof; ++i)
        {
            // Fill in the control effort message
            control_effort.position[i] = legJointPosition[i];
            control_effort.velocity[i] = legJointVelocity[i];
            control_effort.effort[i] = NE_G[i]; // Use gravity compensation for effort
        }

        /******************   Joint Control   *****************/
        // Offset cosine waveform
        // control_effort.position[0] = A0 * (1.0 - cos(omega0 * time));
        // control_effort.position[1] = A1 * (1.0 - cos(omega1 * time));
        // control_effort.position[2] = A2 * (1.0 - cos(omega2 * time));
        // control_effort.velocity[0] = A0 * omega0 * sin(omega0 * time);
        // control_effort.velocity[1] = A1 * omega1 * sin(omega1 * time);
        // control_effort.velocity[2] = A2 * omega2 * sin(omega2 * time);

        // Using sine wave for position and velocity control
        // control_effort.position[0] = A0 * (sin(omega0 * time)) + init_pos[0];
        // control_effort.position[1] = -A1 * (sin(omega1 * time)) + init_pos[1];
        // control_effort.position[2] = -A2 * (sin(omega2 * time)) + init_pos[2];
        // control_effort.velocity[0] = A0 * omega0 * cos(omega0 * time);
        // control_effort.velocity[1] = -A1 * omega1 * cos(omega1 * time);
        // control_effort.velocity[2] = -A2 * omega2 * cos(omega2 * time);

        // Holding Position
        // control_effort.position[0] = init_pos[0];
        // control_effort.position[1] = init_pos[1];
        // control_effort.position[2] = init_pos[2];
        // control_effort.velocity[0] = 0;
        // control_effort.velocity[1] = 0;
        // control_effort.velocity[2] = 0;

        // Send control forces to the simulation
        desired_control_pub_->publish(control_effort);

        // Increment time
        time += time_step_ms / 1000; // increment time by 1 ms per callback

        // Fill in desired position for logging
        endpoint_msg.desired.x = footPosition.x();
        endpoint_msg.desired.y = footPosition.y();
        endpoint_msg.desired.z = footPosition.z();

        // Fill in actual position for logging
        endpoint_msg.actual.x = footPositionActual.x();
        endpoint_msg.actual.y = footPositionActual.y();
        endpoint_msg.actual.z = footPositionActual.z();

        // Fill in error (desired - actual) for logging
        endpoint_msg.error.x = position_error.x();
        endpoint_msg.error.y = position_error.y();
        endpoint_msg.error.z = position_error.z();

        // Publish the foot position endpoint message for logging information
        endpoint_publisher_->publish(endpoint_msg);
    }

    Eigen::MatrixXd computeJacobian(double theta1, double theta2, double theta3)
    {
        // // Link lengths
        double l1 = link_lengths[0];
        double l2 = link_lengths[1];
        double l3 = link_lengths[2];

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

    Eigen::VectorXd forwardKinematics(double theta1, double theta2, double theta3)
    {
        // Precompute useful terms
        // // Link lengths
        double l1 = link_lengths[0];
        double l2 = link_lengths[1];
        double l3 = link_lengths[2];

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

    Eigen::VectorXd LD_gravity(const Eigen::VectorXd &q)
    {
        // // Link lengths
        double l1 = link_lengths[0];
        double l2 = link_lengths[1];
        double l3 = link_lengths[2];

        double q1 = q(0);
        double q2 = q(1);
        double q3 = q(2);

        Eigen::Vector3d G;

        G << (61508064159549433491130382826314582769.0*std::cos(q1))/259614842926741381426524816461004800000.0 - (21253021110606633327.0*std::sin(q1))/3689348814741910323200.0 - (32717374743569109219.0*std::cos(q2)*std::sin(q1))/360287970189639680000.0 - (10106634528669052737.0*std::sin(q1)*std::sin(q2))/7036874417766400000.0 - (37484095691314515767.0*l1*std::cos(q1))/1759218604441600000.0 - (2200489262373673701012225888739.0*std::sin(q1)*std::sin(q2)*std::sin(q3))/5070602400912917605986812821504.0 - (1214914409161531.0*l2*std::sin(q1)*std::sin(q2))/562949953421312.0 + (2200489262373673701012225888739.0*std::cos(q2)*std::cos(q3)*std::sin(q1))/5070602400912917605986812821504.0 + (4897690936801508983255272868831.0*std::cos(q2)*std::sin(q1)*std::sin(q3))/324518553658426726783156020576256.0 + (4897690936801508983255272868831.0*std::cos(q3)*std::sin(q1)*std::sin(q2))/324518553658426726783156020576256.0, (std::cos(q1)*(440097852474734740202445177747800000.0*std::sin(q2 + q3) - 15305284177504715572672727715096875.0*std::cos(q2 + q3) + 1456519535913079500132169550779121664.0*std::cos(q2) - 92091222939799198375252526251376640.0*std::sin(q2) + 2188595232154766929314259035750400000.0*l2*std::cos(q2)))/1014120480182583521197362564300800000.0, -(1214914409161531.0*std::cos(q1)*(4031305333008301.0*std::cos(q2 + q3) - 115918711416970816.0*std::sin(q2 + q3)))/324518553658426726783156020576256.0;

        return G;
    }

    Eigen::VectorXd NE_Dynamics(const Eigen::VectorXd &q,
                                const Eigen::VectorXd &qd,
                                const Eigen::VectorXd &qdd,
                                const double g)
    {
        assert(q.size() == 3 && qd.size() == 3 && qdd.size() == 3);

        // ─── Constants ─────────────────────────────────────────────────────────────
        //  link lengths (m)
        const double l1 = link_lengths[0], l2 = link_lengths[1], l3 = link_lengths[2];

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
            0, 0, 1,
            -c2, s2, 0; // R12 (alpha1 = -pi/2 folded in)
        R[2] << c3, -s3, 0,
            s3, c3, 0,
            0, 0, 1;        // R23
        R[3].setIdentity(); // R3E (end effector)

        // ─── Joint offset vectors oc(i) in each link frame ─────────────────────────
        std::array<Eigen::Vector3d, 4> oc;
        oc[0] = Eigen::Vector3d::Zero(); // from frame0 to frame1
        // oc[1] = Eigen::Vector3d(0, -l1, 0);  // from frame1 to frame2
        // oc[2] = Eigen::Vector3d(l2, 0, 0);  // from frame2 to frame3
        // oc[3] = Eigen::Vector3d(0, -l3, 0); // from frame3 to end

        oc[1] = Eigen::Vector3d(0, l1, 0); // from frame1 to frame2
        oc[2] = Eigen::Vector3d(l2, 0, 0);  // from frame2 to frame3
        oc[3] = Eigen::Vector3d(0, -l3, 0); // from frame3 to end

        // ─── Center of mass in each link’s local frame ─────────────────────────────
        // const std::array<Eigen::Vector3d, 3> pcom = {
        //     Eigen::Vector3d(0.000391481, 0.0100851, -0.0598052),
        //     Eigen::Vector3d(0.0750029, -0.00474222, 0.0593829),
        //     Eigen::Vector3d(-0.0069932, -0.201087, 4.84705e-05)};

        const std::array<Eigen::Vector3d, 3> pcom = {
            Eigen::Vector3d(0.000391481, -0.0100851, -0.00369477),
            Eigen::Vector3d(0.07500288, 0.00474222, 0.00461711),
            Eigen::Vector3d(-0.0069932, -0.20108691, 0.0000484705)};

        // ─── Inertia tensors about each link’s COM, in link frames ────────────────
        std::array<Eigen::Matrix3d, 3> I;
        // I[0] << 0.00233404, -1.25099e-05, 2.28685e-06,
        //     -1.25099e-05, 0.00120555, 5.55819e-05,
        //     2.28685e-06, 5.55819e-05, 0.00180944;

        // I[1] << 0.00147688, 0.0015025, -0.000685788,
        //     0.0015025, 0.0275279, 4.34335e-05,
        //     -0.000685788, 4.34335e-05, 0.0271576;

        // I[2] << 0.0053374, -0.00010343, 2.83936e-08,
        //     -0.00010343, 5.27818e-05, -1.28273e-08,
        //     2.83936e-08, -1.28273e-08, 0.00534035;

        // e-06

        I[0] << 2507.07906, 18.43205, -0.1172,
            18.43205, 1226.25839, -0.31124,
            -0.1172, -0.31124, 1962.23045;

        I[1] << 1562.386, -2196.785, 9.818,
            -2196.785, 38550.362, 0.694,
            9.818, 0.694, 38182.37;

        I[2] << 14232.98552, -412.79182, 0.10296,
            -412.79182, 63.541, 2.13138,
            0.10296, 2.13138, 14246.69206;

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
    rclcpp::Publisher<quadruped_leg_interfaces::msg::Endpoint>::SharedPtr endpoint_publisher_;

    std::vector<double> init_pos;
    std::vector<double> link_lengths;

    Eigen::VectorXd legJointPosition;
    Eigen::VectorXd legJointVelocity;

    Eigen::VectorXd footPosition;
    Eigen::VectorXd footPositionActual;

    double gravity = -9.81;
    float time_step_ms;
    float time = 0;
    int dof = 3;

    // Waveform A parameters
    double A0 = 0.0;      // amplitude
    double period0 = 3.0; // period in seconds
    double omega0 = 2.0 * M_PI / period0;

    // Waveform B parameters
    double A1 = 0.0;      // amplitude
    double period1 = 3.0; // period in seconds
    double omega1 = 2.0 * M_PI / period1;

    // Waveform C parameters
    double A2 = 0.0;      // amplitude
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
