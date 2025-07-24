#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "Eigen/Dense"

#define G 9.81 // gravity
#define M_main 1.5 // kg
#define M_qc 1.5 // kg

using namespace Eigen;

std_msgs::msg::Float64MultiArray t; // desire_thrust
std_msgs::msg::Float64MultiArray T;

Matrix<double, 3, 1> p;
Matrix<double, 3, 1> pd;
Matrix<double, 3, 1> p_dot;
Matrix<double, 3, 1> pd_dot;
Matrix<double, 3, 1> u1;
Matrix<double, 3, 1> fr;

Matrix<double, 3, 3> Kp;
Matrix<double, 3, 3> Kv;
Matrix<double, 3, 3> Ki;
Matrix<double, 3, 3> R;

Matrix<double, 3, 1> ep;
Matrix<double, 3, 1> ev;

Matrix<double, 3, 1> z; // 0,0,1

Quaterniond quaternion;

class PositionController : public rclcpp::Node
{
public:
  PositionController() : Node("position_controller")
  {
    // Initialize variables
    t.data.resize(3);
    T.data.resize(3);

    // Control gains
    Kp << 2, 0, 0,
          0, 2, 0,
          0, 0, 1;

    Kv << 1, 0, 0,
          0, 1, 0,
          0, 0, 0.5;

    Ki << 1, 0, 0,
          0, 1, 0,
          0, 0, 1;

    z << 0, 0, 1;

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // Create subscribers
    desire_position_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/platform/desire_position", qos,
        std::bind(&PositionController::desire_position_cb, this, std::placeholders::_1));

    desire_velocity_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/platform/desire_velocity", qos,
        std::bind(&PositionController::desire_velocity_cb, this, std::placeholders::_1));

    measure_position_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/platform/measure_position", qos,
        std::bind(&PositionController::measure_position_cb, this, std::placeholders::_1));

    measure_velocity_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/platform/measure_velocity", qos,
        std::bind(&PositionController::measure_velocity_cb, this, std::placeholders::_1));

    measure_attitude_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/platform/measure_attitude", qos,
        std::bind(&PositionController::measure_attitude_cb, this, std::placeholders::_1));

    // Create publisher
    desire_thrust_total_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/platform/desire_thrust_total", qos);

    // Create timer for 100 Hz loop
    timer_ = create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&PositionController::control_loop, this));

    RCLCPP_INFO_ONCE(this->get_logger(), "SUCCESS LAUNCH POSITION CONTROLLER");
  }

private:
  void desire_position_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {

    pd(0) = msg->data[0]; // x
    pd(1) = msg->data[1]; // y
    pd(2) = msg->data[2]; // z
  }

  void desire_velocity_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {

    pd_dot(0) = msg->data[0]; // vx
    pd_dot(1) = msg->data[1]; // vy
    pd_dot(2) = msg->data[2]; // vz
  }

  void measure_position_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {

    p(0) = msg->data[0]; // x
    p(1) = msg->data[1]; // y
    p(2) = msg->data[2]; // z
  }

  void measure_velocity_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {

    p_dot(0) = msg->data[0]; // vx
    p_dot(1) = msg->data[1]; // vy
    p_dot(2) = msg->data[2]; // vz
  }

  void measure_attitude_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {

    quaternion = Quaterniond(msg->data[0], msg->data[1], msg->data[2], msg->data[3]); // q_w, q_x, q_y, q_z
    quaternion.normalize(); // 確保四元數規範化
    R = quaternion.toRotationMatrix();
  }

  void total_thrust()
  {
    ep(0) = p(0) - pd(0);
    ep(1) = p(1) - pd(1);
    ep(2) = p(2) - pd(2);

    ev(0) = p_dot(0) - pd_dot(0);
    ev(1) = p_dot(1) - pd_dot(1);
    ev(2) = p_dot(2) - pd_dot(2);

    fr = (M_qc * 4 + M_main) * (G * z - Kp * ep - Kv * ev);
    u1 = R.transpose() * fr; // transform the desire thrust from inertial frame to body fixed frame

    T.data[0] = u1(0); // u_x
    T.data[1] = u1(1); // u_y
    T.data[2] = u1(2); // u_z
  }

  void control_loop()
  {
    total_thrust();
    desire_thrust_total_pub_->publish(T);
  }

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr desire_position_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr desire_velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr measure_position_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr measure_velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr measure_attitude_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr desire_thrust_total_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PositionController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}