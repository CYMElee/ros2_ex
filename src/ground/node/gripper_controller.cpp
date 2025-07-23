#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"

#define Iz 6e-5

#define B 0
#define K 0
#define a 1
#define K1 800

float phi_error = 0;
float phi_error_d = 0;
float r = 0.0001;

std_msgs::msg::Float64MultiArray phi;
std_msgs::msg::Float64MultiArray phid;
std_msgs::msg::Float64 M;

class GripperController : public rclcpp::Node
{
public:
  GripperController() : Node("gripper_controller")
  {
    // Initialize variables
    initialize();
    
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    // Create subscriber
    phi_desire_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/gripper/phi_desire", qos,
        std::bind(&GripperController::phid_cb, this, std::placeholders::_1));

    // Create publisher
    gripper_moment_pub_ = create_publisher<std_msgs::msg::Float64>("/gripper/desire_moment", qos);

    // Create timer for 100 Hz loop
    timer_ = create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&GripperController::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "SUCCESS LAUNCH GRIPPER CONTROLLER");
  }

private:
  void phid_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    phid = *msg;
  }

  void initialize()
  {
    phi.data.resize(3);
    phid.data.resize(3);
    phi.data[0] = 0;
    phi.data[1] = 0;
    phi.data[2] = 0;
    phid.data[0] = 0;
    phid.data[1] = 0;
    phid.data[2] = 0;
    M.data = 0;
  }

  void control_loop()
  {
    phi_error = phid.data[0] - phi.data[0];
    phi_error_d = phid.data[1] - phi.data[1];
    r = phi_error_d + a * phi_error;
    M.data = Iz * (B * phi.data[1] + K * phi.data[0]) + Iz * phid.data[2] +
             (Iz * K1 * r) - (Iz * (a * a) * phi_error);
    M.data = (M.data / 2);
    gripper_moment_pub_->publish(M);
  }

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr phi_desire_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_moment_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}