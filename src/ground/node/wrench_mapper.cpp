#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <Eigen/Dense>
#include <Eigen/QR>    


 

using namespace Eigen;

class WrenchMapper : public rclcpp::Node {
public:
  WrenchMapper() : Node("wrench_mapper") {
    // Initialize matrices and vectors
    BP1 << lw, 0, 0;
    BP2 << 0, lw, 0;
    BP3 << -lw, 0, 0;
    BP4 << 0, -lw, 0;

    BP1x << 0, -BP1(2), BP1(1),
            BP1(2), 0, -BP1(0),
            -BP1(1), BP1(0), 0;

    BP2x << 0, -BP2(2), BP2(1),
            BP2(2), 0, -BP2(0),
            -BP2(1), BP2(0), 0;

    BP3x << 0, -BP3(2), BP3(1),
            BP3(2), 0, -BP3(0),
            -BP3(1), BP3(0), 0;

    BP4x << 0, -BP4(2), BP4(1),
            BP4(2), 0, -BP4(0),
            -BP4(1), BP4(0), 0;

    B1R = Matrix3d::Identity();
    B2R << 0, -1, 0,
           1, 0, 0,
           0, 0, 1;
    B3R << -1, 0, 0,
           0, -1, 0,
           0, 0, 1;
    B4R << 0, 1, 0,
           -1, 0, 0,
           0, 0, 1;

    A << Matrix3d::Identity(), Matrix3d::Identity(), Matrix3d::Identity(), Matrix3d::Identity(),
         BP1x, BP2x, BP3x, BP4x;

    Ar_temp1 << B1R, MatrixXd::Zero(3, 9),
                MatrixXd::Zero(3, 3), B2R, MatrixXd::Zero(3, 6),
                MatrixXd::Zero(3, 6), B3R, MatrixXd::Zero(3, 3),
                MatrixXd::Zero(3, 9), B4R;

    Ar_temp2 = A * Ar_temp1;

    W1_temp << 0, 0, 1;
    W2_temp << 0, 0, -1;
    W3_temp << 0, 0, 1;
    W4_temp << 0, 0, -1;

    W1 = W1_temp * (BP1x * B1R);
    W2 = W2_temp * (BP2x * B2R);
    W3 = W3_temp * (BP3x * B3R);
    W4 = W4_temp * (BP4x * B4R);

    Ar << Ar_temp2, W1, W2, W3, W4;
    

    ArT = Ar .completeOrthogonalDecomposition().pseudoInverse();

    u1.data.resize(3);
    u2.data.resize(3);
    Fd.data.resize(12);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // Initialize subscribers and publisher
    desire_thrust_total_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/platform/desire_thrust_total", qos,
        std::bind(&WrenchMapper::desire_thrust_total_cb, this, std::placeholders::_1));

    desire_moment_attitude_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/platform/desire_total_moment", qos,
        std::bind(&WrenchMapper::desire_moment_attitude_cb, this, std::placeholders::_1));

    desire_moment_angle_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/gripper/desire_moment", qos,
        std::bind(&WrenchMapper::desire_moment_angle_cb, this, std::placeholders::_1));

    desire_thrust_each_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/platform/desire_thrust_each",qos);

    // Initialize timer for periodic execution
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), // 100 Hz
        std::bind(&WrenchMapper::timer_callback, this));

    RCLCPP_INFO_ONCE(this->get_logger(), "SUCCESS LAUNCH WRENCH_MAPPER!!!");
  }

private:
  void desire_thrust_total_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    u1 = *msg;
  }

  void desire_moment_attitude_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    u2 = *msg;
  }

  void desire_moment_angle_cb(const std_msgs::msg::Float64::SharedPtr msg) {
    M = *msg;
  }

  void fd_gen() {
    for (int r = 0; r < 3; r++) {
      fd_temp1(r, 0) = u1.data[r];
    }
    for (int r = 0; r < 3; r++) {
      fd_temp1(r + 3, 0) = u2.data[r];
    }
    fd_temp1(6, 0) = M.data;
    fd = ArT * fd_temp1;
    for (int r = 0; r < 12; r++) {
      Fd.data[r] = fd(r, 0);
    }
  }

  void timer_callback() {
    fd_gen();
    desire_thrust_each_pub_->publish(Fd);
  }

  // Member variables
  std_msgs::msg::Float64MultiArray u1;
  std_msgs::msg::Float64MultiArray u2;
  std_msgs::msg::Float64 M;
  std_msgs::msg::Float64MultiArray Fd;

  Vector3d BP1, BP2, BP3, BP4;
  Matrix<double, 3, 3> BP1x, BP2x, BP3x, BP4x;
  Matrix<double, 3, 3> B1R, B2R, B3R, B4R;
  Matrix<double, 6, 12> A;
  Matrix<double, 12, 12> Ar_temp1;
  Matrix<double, 6, 12> Ar_temp2;
  Matrix<double, 7, 12> Ar;
  Matrix<double, 12, 7> ArT;
  Matrix<double, 7, 1> fd_temp1;
  Matrix<double, 12, 1> fd;
  RowVector3d W1, W2, W3, W4;
  RowVector3d W1_temp, W2_temp, W3_temp, W4_temp;

  float lw = 0.8;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr desire_thrust_total_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr desire_moment_attitude_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr desire_moment_angle_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr desire_thrust_each_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WrenchMapper>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}