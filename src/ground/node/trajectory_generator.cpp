#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>

#define D2R 0.0174 // degrees to radians
#define DT 0.01    // time step

class TrajectoryGenerator : public rclcpp::Node {
public:
  TrajectoryGenerator() : Node("trajectory_generator") {
    // Initialize variables
    initialize();


    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    // Initialize subscribers
    mode_sub_ = this->create_subscription<std_msgs::msg::Int16>(
        "/ground_station/set_mode", qos,
        std::bind(&TrajectoryGenerator::mode_cb, this, std::placeholders::_1));

    pose_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/platform/measure_position", qos,
        std::bind(&TrajectoryGenerator::pose_cb, this, std::placeholders::_1));

    // Initialize publishers
    desire_position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/platform/desire_position", qos);
    desire_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/platform/desire_velocity", qos);
    desire_attitude_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/platform/desire_attitude", qos);
    desire_omega_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/platform/desire_omega", qos);
    desire_phi_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/gripper/phi_desire", qos);

    // Initialize timer for 100 Hz execution
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), // 100 Hz
        std::bind(&TrajectoryGenerator::timer_callback, this));

    RCLCPP_INFO_ONCE(this->get_logger(), "SUCCESS LAUNCH TRAJECTORY_GENERATOR!!!");
  }

private:
  enum class MAV_mod {
    DISARM, //1
    IDLE, // 2
    TAKEOFF, // 3
    LAND, // 4
    SET_HOME, // 5
  };

  // Member variables
  std_msgs::msg::Float64MultiArray p_home;
  std_msgs::msg::Float64MultiArray pose;
  std_msgs::msg::Float64MultiArray pd;
  std_msgs::msg::Float64MultiArray pd_d;
  std_msgs::msg::Float64MultiArray Rr;
  std_msgs::msg::Float64MultiArray agvr;
  std_msgs::msg::Float64MultiArray phid;
  std_msgs::msg::Int16 Mode;

  // Subscribers and publishers
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pose_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr desire_position_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr desire_velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr desire_attitude_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr desire_omega_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr desire_phi_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void mode_cb(const std_msgs::msg::Int16::SharedPtr msg) {
    Mode = *msg;
  }

  void pose_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    pose = *msg;
  }

  void timer_callback() {
    if (Mode.data == static_cast<int>(MAV_mod::IDLE)) {
      // Do nothing
    } else if (Mode.data == static_cast<int>(MAV_mod::TAKEOFF)) {
      hovering();
      RCLCPP_INFO_ONCE(this->get_logger(), "SET_THE_HOVERING_TRAJECTORY!!!");
    } else if (Mode.data == static_cast<int>(MAV_mod::LAND)) {
      land();
      RCLCPP_INFO_ONCE(this->get_logger(), "SET_THE_LAND_TRAJECTORY!!!");
    } else if (Mode.data == static_cast<int>(MAV_mod::SET_HOME)) {
      set_home();
    }

    // Publish messages
    desire_position_pub_->publish(pd);
    desire_velocity_pub_->publish(pd_d);
    desire_attitude_pub_->publish(Rr);
    desire_omega_pub_->publish(agvr);
    desire_phi_pub_->publish(phid);
  }

  void initialize() {
    Mode.data = static_cast<int>(MAV_mod::IDLE);
    RCLCPP_INFO_ONCE(this->get_logger(), "INIT(TRAJECTORY)!!!");
    pd.data.resize(3);     // desire position: x, y, z
    pd_d.data.resize(3);   // desire velocity: x, y, z
    Rr.data.resize(4);     // desire attitude (quaternion or similar)
    agvr.data.resize(3);   // desire angular rate
    phid.data.resize(3);   // desire gripper angle
    p_home.data.resize(3); // home position
    pose.data.resize(3);   // measured position
  }

  void hovering() {
    // In hovering mode, platform hovers at z = 0.3m
    pd.data[0] = p_home.data[0];
    pd.data[1] = p_home.data[1];
    pd.data[2] = 0.3;

    // Platform velocity
    pd_d.data[0] = 0;
    pd_d.data[1] = 0;
    pd_d.data[2] = 0;

    // Platform attitude
    Rr.data[0] = 1;
    Rr.data[1] = 0;
    Rr.data[2] = 0;
    Rr.data[3] = 0;

    // Platform angular rate
    agvr.data[0] = 0;
    agvr.data[1] = 0;
    agvr.data[2] = 0;

    // Platform gripper angle
    phid.data[0] = 1.57;
    phid.data[1] = 0;
    phid.data[2] = 0;
  }

  void land() {
    pd.data[0] = p_home.data[0];
    pd.data[1] = p_home.data[1];
    pd.data[2] = 0.1;

    // Platform velocity
    pd_d.data[0] = 0;
    pd_d.data[1] = 0;
    pd_d.data[2] = 0;

    // Platform attitude
    Rr.data[0] = 1;
    Rr.data[1] = 0;
    Rr.data[2] = 0;
    Rr.data[3] = 0;

    // Platform angular rate
    agvr.data[0] = 0;
    agvr.data[1] = 0;
    agvr.data[2] = 0;

    // Platform gripper angle
    phid.data[0] = 1.57;
    phid.data[1] = 0;
    phid.data[2] = 0;
  }

  void set_home() {
    p_home.data[0] = pose.data[0]; // x
    p_home.data[1] = pose.data[1]; // y
    p_home.data[2] = pose.data[2]; // z
    RCLCPP_INFO_ONCE(this->get_logger(), "SET_THE_PLATFORM_HOME_POSITION(TRAJECTORY)!!!");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryGenerator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}