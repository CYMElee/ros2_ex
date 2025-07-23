#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <string>

class MocapNode : public rclcpp::Node
{
public:
  MocapNode() : Node("topic")
  {
    // Declare and get parameter
    this->declare_parameter("UAV_ID", 0);
    int uav_id = this->get_parameter("UAV_ID").as_int();
    std::string sub_topic = "/vrpn_mocap/platform/pose";

    // Create publisher and subscriber
    mocap_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/fmu/in/vehicle_visual_odometry", 2);
    host_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      sub_topic, 10, std::bind(&MocapNode::host_pos, this, std::placeholders::_1));

    // Initialize rate
    rate_ = std::make_shared<rclcpp::Rate>(100.0);

    RCLCPP_INFO(this->get_logger(), "Mocap node initialized with UAV_ID: %d", uav_id);
  }

  void run()
  {
    while (rclcpp::ok())
    {
      RCLCPP_INFO(this->get_logger(), "odom: %.3f, %.3f, %.3f",
                  host_mocap_.pose.position.x,
                  host_mocap_.pose.position.y,
                  host_mocap_.pose.position.z);
      mocap_pos_pub_->publish(host_mocap_);

      rclcpp::spin_some(this->shared_from_this());
      rate_->sleep();
    }
  }

private:
  void host_pos(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    host_mocap_ = *msg;
    tf2::Quaternion q(
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  }

  geometry_msgs::msg::PoseStamped host_mocap_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_pos_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr host_sub_;
  std::shared_ptr<rclcpp::Rate> rate_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MocapNode>();
  node->run();
  rclcpp::shutdown();
  return 0;
}