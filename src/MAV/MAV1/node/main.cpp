#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include <px4_msgs/srv/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <Eigen/Dense>
#include <string>
#include <cmath>

enum MAV_mod {
    DISARM, //1
    IDLE, // 2
    TAKEOFF, // 3
    LAND, // 4
    SET_HOME, // 5
};

class MAVControl_Node : public rclcpp::Node {
public:
    MAVControl_Node() : Node("mav_control_node") {

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        // Publishers
        T_pub_ = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
            "/MAV1/fmu/in/vehicle_attitude_setpoint", qos);
        T_pub_debug_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/MAV1/fmu/in/vehicle_attitude_setpoint_euler", qos);
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/MAV1/fmu/in/offboard_control_mode", qos);

        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/MAV1/fmu/in/vehicle_command", qos);

        // Subscribers

        platform_pose_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/platfMAVControl_Nodefmu/out/vehicle_attitude", qos,
            std::bind(&MAVControl_Node::mav_pose, this, std::placeholders::_1));
        T_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/MAV1/cmd", qos,
            std::bind(&MAVControl_Node::T_sub, this, std::placeholders::_1));
        takeoff_signal_sub_ = this->create_subscription<std_msgs::msg::Int16>(
            "/ground_station/set_mode", qos,
            std::bind(&MAVControl_Node::mode_cb, this, std::placeholders::_1));
       
        offboard_setpoint_counter_ = 0;

        timer2_ = this->create_wall_timer(std::chrono::milliseconds(10),std::bind(&MAVControl_Node::timer2_callback, this));

		timer_ = this->create_wall_timer(std::chrono::milliseconds(10),std::bind(&MAVControl_Node::timer_callback, this));

	}
       



private:
    
    geometry_msgs::msg::PoseStamped pose_;
    geometry_msgs::msg::PoseStamped platform_pose_;
    geometry_msgs::msg::PoseStamped mav_pose_;
    std_msgs::msg::Float64MultiArray T_cmd_;
    std_msgs::msg::Float32MultiArray Eul_cmd_;
    std_msgs::msg::Int16 Change_Mode_Trigger_;
    px4_msgs::msg::VehicleAttitudeSetpoint T_;
    px4_msgs::msg::VehicleAttitudeSetpoint T_PREARM_;

    uint64_t offboard_setpoint_counter_;

    rclcpp::TimerBase::SharedPtr timer_,timer2_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr T_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr T_pub_debug_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr platform_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mav_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr T_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr takeoff_signal_sub_;

    void initialize() {
        T_cmd_.data.resize(3);
        T_PREARM_.q_d[0] = 1.0; // w
        T_PREARM_.q_d[1] = 0.0; // x
        T_PREARM_.q_d[2] = 0.0; // y
        T_PREARM_.q_d[3] = 0.0; // z
        T_PREARM_.thrust_body[0] = 0.0;
        T_PREARM_.thrust_body[1] = 0.0;
        T_PREARM_.thrust_body[2] = -0.2; // Negative for upward thrust
        Eul_cmd_.data.resize(3);
        Change_Mode_Trigger_.data = MAV_mod::IDLE;
    }


    void mode_cb(const std_msgs::msg::Int16::SharedPtr msg) {
        Change_Mode_Trigger_ = *msg;
    }

    void platform_pose_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
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
        T_pub_->publish(T_);
        T_pub_debug_->publish(Eul_cmd_);
        T_.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    }


    void publish_vehicle_command(uint16_t command, float param1, float param2)
    {
	    px4_msgs::msg::VehicleCommand msg{};
	    msg.param1 = param1;
	    msg.param2 = param2;
	    msg.command = command;
	    msg.target_system = 1;
	    msg.target_component = 1;
	    msg.source_system = 1;
	    msg.source_component = 1;
	    msg.from_external = true;
	    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	    vehicle_command_publisher_->publish(msg);
    }


    void publish_offboard_control_mode()
    {
	    px4_msgs::msg::OffboardControlMode msg{};
	    msg.position = false;
	    msg.velocity = false;
	    msg.acceleration = false;
	    msg.attitude = true;
	    msg.body_rate = false;
	    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	    offboard_control_mode_publisher_->publish(msg);
    }

    void arm()
    {
	    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	    RCLCPP_INFO(this->get_logger(), "Arm command send");
    }
	void timer_callback(){

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
                offboard_setpoint_counter_ = 12;
			}

			publish_offboard_control_mode();
			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		}

    void timer2_callback(){

			if (offboard_setpoint_counter_ == 12) {
                if(Change_Mode_Trigger_ == static_cast<int16_t>(MAV_mod::IDLE)){

                    initialize();
                    
                }
                if(Change_Mode_Trigger_ == static_cast<int16_t>(MAV_mod::TAKEOFF)){
                    T_cmd_calculate();
                }
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