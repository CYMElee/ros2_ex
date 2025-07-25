#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_angular_velocity.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "Eigen/Dense"
#include "cmath"

class PositionAttitudeVelocityConverterNode : public rclcpp::Node
{
public:
    PositionAttitudeVelocityConverterNode() : Node("position_attitude_velocity_converter_node")
    {
        // 初始化 QoS 設置
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // 創建訂閱者，訂閱 /fmu/out/vehicle_local_position
        position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/MAV5/fmu/out/vehicle_local_position", qos,
            std::bind(&PositionAttitudeVelocityConverterNode::position_callback, this, std::placeholders::_1));

        // 創建訂閱者，訂閱 /fmu/out/vehicle_attitude
        attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/MAV5/fmu/out/vehicle_attitude", qos,
            std::bind(&PositionAttitudeVelocityConverterNode::attitude_callback, this, std::placeholders::_1));

        // 創建訂閱者，訂閱 /fmu/out/vehicle_angular_velocity
        angular_velocity_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
            "/MAV5/fmu/out/vehicle_angular_velocity", qos,
            std::bind(&PositionAttitudeVelocityConverterNode::angular_velocity_callback, this, std::placeholders::_1));

        // 創建發布者，發布到 /platform/measure_position
        position_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/platform/measure_position", qos);

        // 創建發布者，發布到 /platform/measure_attitude
        attitude_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/platform/measure_attitude", qos);

        // 創建發布者，發布到 /platform/measure_velocity
        velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/platform/measure_velocity", qos);

        // 創建發布者，發布到 /platform/measure_omega
        omega_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/platform/measure_omega", qos);

        // 初始化發布消息
        position_msg_.data.resize(3); // x, y, z 三個分量
        attitude_msg_.data.resize(4); // q_w, q_x, q_y, q_z 四個分量
        velocity_msg_.data.resize(3); // vx, vy, vz 三個分量
        omega_msg_.data.resize(3);    // wx, wy, wz 三個分量

        // 初始化數據儲存
        latest_position_.x = latest_position_.y = latest_position_.z = 0.0;
        latest_position_.vx = latest_position_.vy = latest_position_.vz = 0.0;
        latest_attitude_.q[0] = 1.0; // w
        latest_attitude_.q[1] = latest_attitude_.q[2] = latest_attitude_.q[3] = 0.0; // x, y, z
        latest_angular_velocity_.xyz[0] = latest_angular_velocity_.xyz[1] = latest_angular_velocity_.xyz[2] = 0.0;

        // 創建 100 Hz 定時器（每 10 毫秒）
        timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&PositionAttitudeVelocityConverterNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Position, Attitude, Velocity, and Omega Converter Node with Timer has been started!");
    }

private:
    void position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        // 儲存最新的位置和速度數據
        latest_position_ = *msg;
      
    }

    void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
    {
        // 儲存最新的姿態數據
        latest_attitude_ = *msg;
       
    }

    void angular_velocity_callback(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg)
    {
        // 儲存最新的角速度數據
        latest_angular_velocity_ = *msg;
        
    }

    void timer_callback()
    {


        // 處理位置和速度 (NED 到 NWU)
        position_msg_.data[0] = latest_position_.x;      
        position_msg_.data[1] = -latest_position_.y;   
        position_msg_.data[2] = -latest_position_.z;     
        position_publisher_->publish(position_msg_);

        velocity_msg_.data[0] = latest_position_.vx;     
        velocity_msg_.data[1] = -latest_position_.vy;     
        velocity_msg_.data[2] = -latest_position_.vz;  
        velocity_publisher_->publish(velocity_msg_);

        // 處理姿態 (NED 到 NWU)
        Eigen::Quaterniond q_ned(latest_attitude_.q[0], latest_attitude_.q[1], latest_attitude_.q[2], latest_attitude_.q[3]);
        q_ned.normalize(); 
    
        attitude_msg_.data[0] = q_ned.w(); // q_w
        attitude_msg_.data[1] = q_ned.x(); // q_x
        attitude_msg_.data[2] = -q_ned.y(); // q_y
        attitude_msg_.data[3] = -q_ned.z(); // q_z
        attitude_publisher_->publish(attitude_msg_);

        // 處理角速度 (FRD 到 NWU)
        Eigen::Vector3d omega_frd(latest_angular_velocity_.xyz[0], latest_angular_velocity_.xyz[1], latest_angular_velocity_.xyz[2]);
        omega_msg_.data[0] = omega_frd[0]; 
        omega_msg_.data[1] = -omega_frd[1];
        omega_msg_.data[2] = -omega_frd[2]; 
        omega_publisher_->publish(omega_msg_);


    }

    // 成員變數
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr angular_velocity_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr attitude_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr omega_publisher_;
    std_msgs::msg::Float64MultiArray position_msg_;
    std_msgs::msg::Float64MultiArray attitude_msg_;
    std_msgs::msg::Float64MultiArray velocity_msg_;
    std_msgs::msg::Float64MultiArray omega_msg_;
    px4_msgs::msg::VehicleLocalPosition latest_position_;
    px4_msgs::msg::VehicleAttitude latest_attitude_;
    px4_msgs::msg::VehicleAngularVelocity latest_angular_velocity_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PositionAttitudeVelocityConverterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}