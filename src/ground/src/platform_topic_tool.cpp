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

        // 創建訂閱者，訂閱 /MAV5/fmu/out/vehicle_local_position
        position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos,
            std::bind(&PositionAttitudeVelocityConverterNode::position_callback, this, std::placeholders::_1));

        // 創建訂閱者，訂閱 /MAV5/fmu/out/vehicle_attitude
        attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos,
            std::bind(&PositionAttitudeVelocityConverterNode::attitude_callback, this, std::placeholders::_1));

        // 創建訂閱者，訂閱 /MAV5/fmu/out/vehicle_angular_velocity
        angular_velocity_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
            "/fmu/out/vehicle_angular_velocity", qos,
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

        RCLCPP_INFO(this->get_logger(), "Position, Attitude, Velocity, and Omega Converter Node has been started!");
    }

private:
    void position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        // 檢查數據有效性
        if (!msg->xy_valid || !msg->z_valid || !msg->v_xy_valid || !msg->v_z_valid)
        {
            RCLCPP_WARN(this->get_logger(), "Invalid position or velocity data received!");
            return;
        }

        position_msg_.data[0] = msg->y;      // ENU x = NED y (East)
        position_msg_.data[1] = msg->x;      // ENU y = NED x (North)
        position_msg_.data[2] = -msg->z;     // ENU z = -NED z (Up)

        // 發布轉換後的坐標
        position_publisher_->publish(position_msg_);

        // 將 NED 坐標系轉換為 ENU 坐標系（速度）
        // NED: vx (North), vy (East), vz (Down)
        // ENU: vx (East), vy (North), vz (Up)
        velocity_msg_.data[0] = msg->vy;     // ENU vx = NED vy (East)
        velocity_msg_.data[1] = msg->vx;     // ENU vy = NED vx (North)
        velocity_msg_.data[2] = -msg->vz;    // ENU vz = -NED vz (Up)

        // 發布轉換後的速度
        velocity_publisher_->publish(velocity_msg_);

        // 記錄發布的數據（用於調試）
        RCLCPP_DEBUG(this->get_logger(), "Published ENU position: x=%.2f, y=%.2f, z=%.2f",
                     position_msg_.data[0], position_msg_.data[1], position_msg_.data[2]);
        RCLCPP_DEBUG(this->get_logger(), "Published ENU velocity: vx=%.2f, vy=%.2f, vz=%.2f",
                     velocity_msg_.data[0], velocity_msg_.data[1], velocity_msg_.data[2]);
    }

    void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
    {
        // 提取 NED 四元數並規範化
        Eigen::Quaterniond q_ned(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
        q_ned.normalize(); // 確保四元數規範化

        // NED 到 ENU 的旋轉：繞 z 軸旋轉 180 度（pi 弧度）
        Eigen::Quaterniond q_rot(cos(M_PI / 2.0), 0.0, 0.0, sin(M_PI / 2.0)); // 旋轉四元數
        Eigen::Quaterniond q_enu = q_rot * q_ned * q_rot.conjugate();

        // 儲存到發布消息（姿態）
        attitude_msg_.data[0] = q_enu.w(); // q_w
        attitude_msg_.data[1] = q_enu.x(); // q_x
        attitude_msg_.data[2] = q_enu.y(); // q_y
        attitude_msg_.data[3] = q_enu.z(); // q_z

        // 發布轉換後的姿態
        attitude_publisher_->publish(attitude_msg_);

        // 記錄發布的數據（用於調試）
        RCLCPP_DEBUG(this->get_logger(), "Published ENU attitude: q_w=%.2f, q_x=%.2f, q_y=%.2f, q_z=%.2f",
                     attitude_msg_.data[0], attitude_msg_.data[1], attitude_msg_.data[2], attitude_msg_.data[3]);
    }

    void angular_velocity_callback(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg)
    {
        // 提取 NED 機體坐標系中的角速度
        Eigen::Vector3d omega_ned(msg->xyz[0], msg->xyz[1], msg->xyz[2]);

        // NED 到 ENU 的旋轉：繞 z 軸旋轉 180 度（pi 弧度）
        Eigen::Quaterniond q_rot(cos(M_PI / 2.0), 0.0, 0.0, sin(M_PI / 2.0)); // 旋轉四元數
        Eigen::Matrix3d R_ned_to_enu = q_rot.toRotationMatrix(); // 旋轉矩陣
        Eigen::Vector3d omega_enu = R_ned_to_enu * omega_ned;

        // 儲存到發布消息（角速度）
        omega_msg_.data[0] = omega_enu[0]; // wx (roll rate)
        omega_msg_.data[1] = omega_enu[1]; // wy (pitch rate)
        omega_msg_.data[2] = omega_enu[2]; // wz (yaw rate)
       

        // 發布轉換後的角速度
        omega_publisher_->publish(omega_msg_);

        // 記錄發布的數據（用於調試）
        RCLCPP_DEBUG(this->get_logger(), "Published ENU omega: wx=%.2f, wy=%.2f, wz=%.2f",
                     omega_msg_.data[0], omega_msg_.data[1], omega_msg_.data[2]);
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
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PositionAttitudeVelocityConverterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}