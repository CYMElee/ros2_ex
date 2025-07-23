#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <Eigen/Dense>
#include <string>
#include <cmath>

enum MAV_mod {
    IDLE,
    TAKEOFF,
    LAND,
    SET_HOME,
};

class MAVControl_Node : public rclcpp::Node {
public:
    MAVControl_Node() : Node("mav_control_node") {
        // Get UAV_ID parameter
        this->declare_parameter<int>("UAV_ID", 1);
        this->get_parameter("UAV_ID", UAV_ID_);

        // Initialize variables
        initialize();

        // Publishers
        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/fmu/in/vehicle_local_position_setpoint", 10);
        T_pub_ = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
            "/fmu/in/vehicle_attitude_setpoint", 10);
        T_pub_debug_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/fmu/in/vehicle_attitude_setpoint_euler", 10);

        // Subscribers
        state_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/MAV" + std::to_string(UAV_ID_) + "/fmu/out/vehicle_status", 10,
            std::bind(&MAVControl_Node::state_cb, this, std::placeholders::_1));
        platform_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/platform/mavros/local_position/pose", 12,
            std::bind(&MAVControl_Node::platform_pose_cb, this, std::placeholders::_1));
        mav_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/MAV" + std::to_string(UAV_ID_) + "/local_position/pose", 10,
            std::bind(&MAVControl_Node::mav_pose, this, std::placeholders::_1));
        T_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "cmd", 12,
            std::bind(&MAVControl_Node::T_sub, this, std::placeholders::_1));
        takeoff_signal_sub_ = this->create_subscription<std_msgs::msg::Int16>(
            "/ground_station/set_mode", 10,
            std::bind(&MAVControl_Node::mode_cb, this, std::placeholders::_1));

        // Service clients
        vehicle_command_client_ = this->create_client<px4_msgs::srv::VehicleCommand>(
            "/fmu/in/vehicle_command");

        // Wait for vehicle status connection
        RCLCPP_INFO(this->get_logger(), "Waiting for vehicle status connection...");
        while (rclcpp::ok() && !current_state_.nav_state) {
            rclcpp::spin_some(this->get_node_base_interface());
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
        RCLCPP_INFO(this->get_logger(), "Vehicle status connected.");

        // Initialize setpoint
        pose_.pose.position.x = 0;
        pose_.pose.position.y = 0;
        pose_.pose.position.z = 0;

        // Publish initial setpoints
        auto rate = rclcpp::Rate(100);
        for (int i = 100; rclcpp::ok() && i > 0; --i) {
            local_pos_pub_->publish(pose_);
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }

        // Set OFFBOARD mode and arm
        set_offboard_mode();
        arm_vehicle();

        // Main loop
        main_loop();
    }

private:
    int UAV_ID_;
    geometry_msgs::msg::PoseStamped pose_;
    geometry_msgs::msg::PoseStamped platform_pose_;
    geometry_msgs::msg::PoseStamped mav_pose_;
    px4_msgs::msg::VehicleStatus current_state_;
    std_msgs::msg::Float64MultiArray T_cmd_;
    std_msgs::msg::Float32MultiArray Eul_cmd_;
    std_msgs::msg::Int16 Change_Mode_Trigger_;
    px4_msgs::msg::VehicleAttitudeSetpoint T_;
    px4_msgs::msg::VehicleAttitudeSetpoint T_PREARM_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr T_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr T_pub_debug_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr platform_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mav_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr T_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr takeoff_signal_sub_;
    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;

    void initialize() {
        T_cmd_.data.resize(3);
        T_PREARM_.q_d[0] = 1.0; // w
        T_PREARM_.q_d[1] = 0.0; // x
        T_PREARM_.q_d[2] = 0.0; // y
        T_PREARM_.q_d[3] = 0.0; // z
        T_PREARM_.thrust_body[0] = 0.0;
        T_PREARM_.thrust_body[1] = 0.0;
        T_PREARM_.thrust_body[2] = -0.3; // Negative for upward thrust
        Eul_cmd_.data.resize(3);
        Change_Mode_Trigger_.data = MAV_mod::IDLE;
    }

    void state_cb(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        current_state_ = *msg;
    }

    void mode_cb(const std_msgs::msg::Int16::SharedPtr msg) {
        Change_Mode_Trigger_ = *msg;
    }

    void platform_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        platform_pose_ = *msg;
    }

    void mav_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        mav_pose_ = *msg;
    }

    void T_sub(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        T_cmd_ = *msg;
    }

    void T_cmd_calculate() {
        Eigen::Quaterniond quaternion_mav(
            mav_pose_.pose.orientation.w,
            mav_pose_.pose.orientation.x,
            mav_pose_.pose.orientation.y,
            mav_pose_.pose.orientation.z);
        Eigen::Matrix3d rotationMatrix_mav = quaternion_mav.toRotationMatrix();
        Eigen::Vector3d eulerAngles_mav = rotationMatrix_mav.eulerAngles(2, 1, 0);
        double alpha = T_cmd_.data[1];
        double beta = T_cmd_.data[2];
        double thrust = T_cmd_.data[0];
        double z = eulerAngles_mav(0);

        Eigen::Quaterniond MAV_pose_cmd;
        MAV_pose_cmd = Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(beta, Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX());
        Eigen::Matrix3d rotationMatrix_mav_des = MAV_pose_cmd.toRotationMatrix();

        Eigen::Quaterniond quaternion_platform(
            platform_pose_.pose.orientation.w,
            platform_pose_.pose.orientation.x,
            platform_pose_.pose.orientation.y,
            platform_pose_.pose.orientation.z);
        Eigen::Matrix3d rotationMatrix_platform = quaternion_platform.toRotationMatrix();

        Eigen::Matrix3d rotationMatrix_mav_des_b2i = rotationMatrix_platform.transpose() * rotationMatrix_mav_des;
        Eigen::Vector3d eulerAngles_mav_des = rotationMatrix_mav_des_b2i.eulerAngles(2, 1, 0);

        Eigen::Quaterniond mav_pose_desire;
        mav_pose_desire = Eigen::AngleAxisd(eulerAngles_mav_des(0), Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(eulerAngles_mav_des(1), Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(eulerAngles_mav_des(2), Eigen::Vector3d::UnitX());

        Eul_cmd_.data[0] = eulerAngles_mav_des(0);
        Eul_cmd_.data[1] = eulerAngles_mav_des(1);
        Eul_cmd_.data[2] = eulerAngles_mav_des(2);

        T_.q_d[0] = mav_pose_desire.w();
        T_.q_d[1] = mav_pose_desire.x();
        T_.q_d[2] = mav_pose_desire.y();
        T_.q_d[3] = mav_pose_desire.z();
        T_.thrust_body[0] = 0.0;
        T_.thrust_body[1] = 0.0;
        T_.thrust_body[2] = -thrust; // Negative for upward thrust
        T_.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    }

    void set_offboard_mode() {
        auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();
        request->request.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        request->request.param1 = 1.0; // Custom mode
        request->request.param2 = 6.0; // OFFBOARD mode
        request->request.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        while (!vehicle_command_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for vehicle command service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for vehicle command service...");
        }

        try {
            auto result = vehicle_command_client_->async_send_request(request);
            if (result.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
                RCLCPP_INFO(this->get_logger(), "OFFBOARD mode command sent");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to set OFFBOARD mode: service call timed out");
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set OFFBOARD mode: %s", e.what());
        }
    }

    void arm_vehicle() {
        auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();
        request->request.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        request->request.param1 = 1.0; // Arm
        request->request.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        while (!vehicle_command_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for vehicle command service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for vehicle command service...");
        }

        try {
            auto result = vehicle_command_client_->async_send_request(request);
            if (result.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
                RCLCPP_INFO(this->get_logger(), "Vehicle arm command sent");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to arm vehicle: service call timed out");
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to arm vehicle: %s", e.what());
        }
    }

    void main_loop() {
        auto rate = rclcpp::Rate(100);
        while (rclcpp::ok() && Change_Mode_Trigger_.data != MAV_mod::TAKEOFF && Change_Mode_Trigger_.data != MAV_mod::LAND) {
            RCLCPP_INFO(this->get_logger(), "READY_TAKEOFF!!");
            if (current_state_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
                set_offboard_mode();
            }
            if (!current_state_.arming_state) {
                arm_vehicle();
            }
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }

        while (rclcpp::ok()) {
            T_cmd_calculate();
            T_pub_->publish(T_);
            T_pub_debug_->publish(Eul_cmd_);
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MAVControl_Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}