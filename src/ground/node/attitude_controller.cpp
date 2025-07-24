#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "Eigen/Dense"

#define Lw_sq 0.80
double IBXY = 2 * 1.5 * 0.80;
double IBZ = 4 * 1.5 * 0.80;

using namespace Eigen;

double IBxy = IBXY;
double IBz = IBZ;

std_msgs::msg::Float64MultiArray u2;
std_msgs::msg::Float64MultiArray er;
Matrix<double, 3, 3> IB; // rotation inertia
Matrix<double, 3, 3> R;  // measure attitude, rotation matrix from body frame to inertial frame
Matrix<double, 3, 3> Rr; // desire attitude
Vector3d agvr;
Vector3d omega;
Vector3d m;
Vector3d h;

// attitude control gains
Matrix<double, 3, 3> KR;
Matrix<double, 3, 3> Kw;
Matrix<double, 3, 3> Ki;

Matrix<double, 3, 3> E;
Vector3d eR;
Vector3d eW;

Quaterniond quaternion1, quaternion2;

class AttitudeController : public rclcpp::Node
{
public:
  AttitudeController() : Node("attitude_controller")
  {
    // Resize message data
    u2.data.resize(3);
    er.data.resize(3);

    // Initialize matrices
    KR << 5, 0, 0,
          0, 5, 0,
          0, 0, 3;

    Kw << 3, 0, 0,
          0, 3, 0,
          0, 0, 1;

    IB << IBxy, 0, 0,
          0, IBxy, 0,
          0, 0, IBz;

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // Create subscribers
    desire_attitude_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/platform/desire_attitude", qos,
        std::bind(&AttitudeController::desire_attitude_cb, this, std::placeholders::_1));
    measure_attitude_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/platform/measure_attitude", qos,
        std::bind(&AttitudeController::measure_attitude_cb, this, std::placeholders::_1));
    desire_omega_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/platform/desire_omega", qos,
        std::bind(&AttitudeController::desire_omega_cb, this, std::placeholders::_1));
    measure_omega_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/platform/measure_omega", qos,
        std::bind(&AttitudeController::measure_omega_cb, this, std::placeholders::_1));

    // Create publishers
    total_moment_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/platform/desire_total_moment", qos);
    attitude_error_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/platform/attitude_error", qos);

    // Create timer for 100 Hz loop
    timer_ = create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&AttitudeController::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "SUCCESS LAUNCH ATTITUDE CONTROLLER");
  }

private:
  void desire_attitude_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    quaternion1 = AngleAxisd(msg->data[0], Vector3d(msg->data[1], msg->data[2], msg->data[3]));
    Rr = quaternion1.toRotationMatrix();
  }

  void measure_attitude_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    quaternion2 = AngleAxisd(msg->data[0], Vector3d(msg->data[1], msg->data[2], msg->data[3]));
    R = quaternion2.toRotationMatrix();
  }

  void desire_omega_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    agvr(0) = msg->data[0];
    agvr(1) = msg->data[1];
    agvr(2) = msg->data[2];
  }

  void measure_omega_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    omega(0) = msg->data[0];
    omega(1) = msg->data[1];
    omega(2) = msg->data[2];
  }

  void attitude_er()
  {
    E = 0.5 * (Rr.transpose() * R - R.transpose() * Rr);
    eR(0) = E(2, 1);
    eR(1) = E(0, 2);
    eR(2) = E(1, 0);
    er.data[0] = eR(0);
    er.data[1] = eR(1);
    er.data[2] = eR(2);

    eW = omega - (R.transpose() * (Rr * agvr));

    // Physical dynamic to cancel
    h = omega.cross(IB * omega);
  }

  void moment()
  {
    m = h - IB * (KR * eR + Kw * eW);

    u2.data[0] = m(0);
    u2.data[1] = m(1);
    u2.data[2] = m(2);
  }

  void control_loop()
  {
    attitude_er();
    moment();

    total_moment_pub_->publish(u2);
    attitude_error_pub_->publish(er);
  }

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr desire_attitude_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr measure_attitude_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr desire_omega_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr measure_omega_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr total_moment_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr attitude_error_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AttitudeController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}