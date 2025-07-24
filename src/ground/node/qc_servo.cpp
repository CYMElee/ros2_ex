#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "Eigen/Dense"
#include "cmath"
#include "string"

// Define the motor curve index
double p7 = 0.03169;
double p6 = 0.2807;
double p5 = -0.07407;
double p4 = 0.01128;
double p3 = -0.0007524;
double p2 = 1.375e-05;
double p1 = 3.274e-07;

using namespace Eigen;
using namespace std;

Quaterniond q;

std_msgs::msg::Float64MultiArray fd; // Desire force for each UAV

class MAV {
private:
    std_msgs::msg::Float64MultiArray T; // T[0] is net thrust, T[1] is alpha, T[2] is beta
    Vector3d fd_e;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mav_cmd;

public:

    MAV(std::shared_ptr<rclcpp::Node> node, string topic);
    void Thrust(std_msgs::msg::Float64MultiArray fd, int i);
};

MAV::MAV(std::shared_ptr<rclcpp::Node> node, string topic)
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    T.data.resize(3);
    mav_cmd = node->create_publisher<std_msgs::msg::Float64MultiArray>(topic, qos);
}

void MAV::Thrust(std_msgs::msg::Float64MultiArray fd, int i)
{
    fd_e(0) = fd.data[i]; // x
    fd_e(1) = fd.data[i + 1]; // y
    fd_e(2) = fd.data[i + 2]; // z
    double f = (fd_e.norm() / 4); // Because we have 4 motors for each sd420

    T.data[0] = p1 * pow(f, 6) + p2 * pow(f, 5) + p3 * pow(f, 4) + p4 * pow(f, 3) + p5 * pow(f, 2) + p6 * pow(f, 1) + p7; // Net thrust (PWM 0~1)
    if (T.data[0] >= 0.3)
        T.data[0] = 0.3;
    if (T.data[0] <= 0.1)
        T.data[0] = 0.1;

    /* Using the desired thrust (vector) on platform body frame to get the alpha and beta */
    double alpha = atan2(-fd_e(1), fd_e(2));
    double beta = asin(fd_e(0) / fd_e.norm());

    T.data[1] = alpha;
    T.data[2] = beta;

    mav_cmd->publish(T);
}

void thrust_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    fd = *msg;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("qc_servo");

    fd.data.resize(12);
    RCLCPP_INFO(node->get_logger(), "SUCCESS LAUNCH QC_SERVO!!");
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    auto thrust = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/platform/desire_thrust_each", qos, thrust_cb);

    MAV mav[4] = {
        MAV(node, "/MAV1/cmd"),
        MAV(node, "/MAV2/cmd"),
        MAV(node, "/MAV3/cmd"),
        MAV(node, "/MAV4/cmd")
    };

    rclcpp::Rate rate(100);
    while (rclcpp::ok())
    {
        mav[0].Thrust(fd, 0);
        mav[1].Thrust(fd, 3);
        mav[2].Thrust(fd, 6);
        mav[3].Thrust(fd, 9);

        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}