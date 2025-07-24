#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <px4_msgs/msg/vehicle_odometry.hpp>

#ifdef PX4_ROS_TIMESYNC
	#include <px4_msgs/msg/timesync.hpp>
#endif
#include <geometry_msgs/msg/pose_stamped.hpp>

using std::placeholders::_1;

using namespace std::chrono_literals;


class MocapPX4Bridge : public rclcpp::Node
{
public:
	MocapPX4Bridge() : Node("mocap_px4_bridge") {



		const std::string MAV1_mocap_topic = "/vrpn_mocap/MAV1/pose";
		const std::string MAV2_mocap_topic = "/vrpn_mocap/MAV2/pose";
		const std::string MAV3_mocap_topic = "/vrpn_mocap/MAV3/pose";
		const std::string MAV4_mocap_topic = "/vrpn_mocap/MAV4/pose";
		const std::string MAV5_mocap_topic = "/vrpn_mocap/MAV5/pose";

		const std::string MAV1_px4_topic = "/MAV1/fmu/in/vehicle_visual_odometry";
		const std::string MAV2_px4_topic = "/MAV2/fmu/in/vehicle_visual_odometry";
		const std::string MAV3_px4_topic = "/MAV3/fmu/in/vehicle_visual_odometry";
		const std::string MAV4_px4_topic = "/MAV4/fmu/in/vehicle_visual_odometry";
		const std::string MAV5_px4_topic = "/MAV5/fmu/in/vehicle_visual_odometry";

 		//Define The QoS 
		auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
            .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)  // 匹配發布者的 BEST_EFFORT
            .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)      // 匹配發布者的 VOLATILE
            .lifespan(rclcpp::Duration::from_seconds(0))         // Infinite
            .deadline(rclcpp::Duration::from_seconds(0))         // Infinite
            .liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC)     // AUTOMATIC
            .liveliness_lease_duration(rclcpp::Duration::from_seconds(0)); // Infinite


		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos_1 = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);


        //Create the subscriber that from optitrack
		MAV1_poseSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(MAV1_mocap_topic, qos, std::bind(&MocapPX4Bridge::MAV1_poseCallback, this, _1));
		MAV2_poseSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(MAV2_mocap_topic, qos, std::bind(&MocapPX4Bridge::MAV2_poseCallback, this, _1));
		MAV3_poseSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(MAV3_mocap_topic, qos, std::bind(&MocapPX4Bridge::MAV3_poseCallback, this, _1));
		MAV4_poseSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(MAV4_mocap_topic, qos, std::bind(&MocapPX4Bridge::MAV4_poseCallback, this, _1));
		MAV5_poseSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(MAV5_mocap_topic, qos, std::bind(&MocapPX4Bridge::MAV5_poseCallback, this, _1));

		// Create the publisher that send to PX4 FCU
		MAV1_odomPub = this->create_publisher<px4_msgs::msg::VehicleOdometry>(MAV1_px4_topic, qos_1);
		MAV2_odomPub = this->create_publisher<px4_msgs::msg::VehicleOdometry>(MAV2_px4_topic, qos_1);
		MAV3_odomPub = this->create_publisher<px4_msgs::msg::VehicleOdometry>(MAV3_px4_topic, qos_1);
		MAV4_odomPub = this->create_publisher<px4_msgs::msg::VehicleOdometry>(MAV4_px4_topic, qos_1);
		MAV5_odomPub = this->create_publisher<px4_msgs::msg::VehicleOdometry>(MAV5_px4_topic, qos_1);

	}

private:
	void MAV1_poseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr);
	void MAV2_poseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr);
	void MAV3_poseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr);
	void MAV4_poseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr);
	void MAV5_poseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr);

	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr MAV1_poseSub;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr MAV2_poseSub;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr MAV3_poseSub;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr MAV4_poseSub;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr MAV5_poseSub;

	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr MAV1_odomPub;
	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr MAV2_odomPub;
	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr MAV3_odomPub;
	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr MAV4_odomPub;
	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr MAV5_odomPub;
};

void MocapPX4Bridge::MAV1_poseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr poseMsg){
	RCLCPP_INFO_ONCE(get_logger(), "Recived first msg from optitrack.");
	RCLCPP_INFO_ONCE(get_logger(), "P: %f, %f, %f", poseMsg->pose.position.x, 
					poseMsg->pose.position.y, poseMsg->pose.position.z);
	RCLCPP_INFO_ONCE(get_logger(), "q: %f, %f, %f, %f", poseMsg->pose.orientation.w,
					poseMsg->pose.orientation.x, poseMsg->pose.orientation.y, poseMsg->pose.orientation.z);
	px4_msgs::msg::VehicleOdometry MAV1_odomMsg;

	MAV1_odomMsg.pose_frame = MAV1_odomMsg.POSE_FRAME_FRD;
	MAV1_odomMsg.timestamp = uint64_t(poseMsg->header.stamp.sec)*1000000 + uint64_t(poseMsg->header.stamp.nanosec)/1000;
	MAV1_odomMsg.timestamp_sample = MAV1_odomMsg.timestamp;

	MAV1_odomMsg.position[0] = poseMsg->pose.position.x;
	MAV1_odomMsg.position[1] = -poseMsg->pose.position.y;
	MAV1_odomMsg.position[2] = -poseMsg->pose.position.z;

	MAV1_odomMsg.q[0] = poseMsg->pose.orientation.w;
	MAV1_odomMsg.q[1] = poseMsg->pose.orientation.x;
	MAV1_odomMsg.q[2] = - poseMsg->pose.orientation.y;
	MAV1_odomMsg.q[3] = - poseMsg->pose.orientation.z;
	
	MAV1_odomPub -> publish(MAV1_odomMsg);
	RCLCPP_INFO_ONCE(get_logger(), "Sent to PX4 as:");
	RCLCPP_INFO_ONCE(get_logger(), "P: %f, %f, %f", MAV1_odomMsg.position[0], 
					MAV1_odomMsg.position[1], MAV1_odomMsg.position[2]);
	RCLCPP_INFO_ONCE(get_logger(), "q: %f, %f, %f, %f", MAV1_odomMsg.q[0],
					MAV1_odomMsg.q[1], MAV1_odomMsg.q[2], MAV1_odomMsg.q[3]);
}



void MocapPX4Bridge::MAV2_poseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr poseMsg){
	RCLCPP_INFO_ONCE(get_logger(), "Recived first msg from optitrack.");
	RCLCPP_INFO_ONCE(get_logger(), "P: %f, %f, %f", poseMsg->pose.position.x, 
					poseMsg->pose.position.y, poseMsg->pose.position.z);
	RCLCPP_INFO_ONCE(get_logger(), "q: %f, %f, %f, %f", poseMsg->pose.orientation.w,
					poseMsg->pose.orientation.x, poseMsg->pose.orientation.y, poseMsg->pose.orientation.z);
	px4_msgs::msg::VehicleOdometry MAV2_odomMsg;

	MAV2_odomMsg.pose_frame = MAV2_odomMsg.POSE_FRAME_FRD;
	MAV2_odomMsg.timestamp = uint64_t(poseMsg->header.stamp.sec)*1000000 + uint64_t(poseMsg->header.stamp.nanosec)/1000;
	MAV2_odomMsg.timestamp_sample = MAV2_odomMsg.timestamp;

	MAV2_odomMsg.position[0] = poseMsg->pose.position.x;
	MAV2_odomMsg.position[1] = -poseMsg->pose.position.y;
	MAV2_odomMsg.position[2] = -poseMsg->pose.position.z;

	MAV2_odomMsg.q[0] = poseMsg->pose.orientation.w;
	MAV2_odomMsg.q[1] = poseMsg->pose.orientation.x;
	MAV2_odomMsg.q[2] = - poseMsg->pose.orientation.y;
	MAV2_odomMsg.q[3] = - poseMsg->pose.orientation.z;
	MAV2_odomMsg.quality = 1;
	MAV2_odomPub -> publish(MAV2_odomMsg);
	RCLCPP_INFO_ONCE(get_logger(), "Sent to PX4 as:");
	RCLCPP_INFO_ONCE(get_logger(), "P: %f, %f, %f", MAV2_odomMsg.position[0], 
					MAV2_odomMsg.position[1], MAV2_odomMsg.position[2]);
	RCLCPP_INFO_ONCE(get_logger(), "q: %f, %f, %f, %f", MAV2_odomMsg.q[0],
					MAV2_odomMsg.q[1], MAV2_odomMsg.q[2], MAV2_odomMsg.q[3]);
}



void MocapPX4Bridge::MAV3_poseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr poseMsg){
	RCLCPP_INFO_ONCE(get_logger(), "Recived first msg from optitrack.");
	RCLCPP_INFO_ONCE(get_logger(), "P: %f, %f, %f", poseMsg->pose.position.x, 
					poseMsg->pose.position.y, poseMsg->pose.position.z);
	RCLCPP_INFO_ONCE(get_logger(), "q: %f, %f, %f, %f", poseMsg->pose.orientation.w,
					poseMsg->pose.orientation.x, poseMsg->pose.orientation.y, poseMsg->pose.orientation.z);
	px4_msgs::msg::VehicleOdometry MAV3_odomMsg;

	MAV3_odomMsg.pose_frame = MAV3_odomMsg.POSE_FRAME_FRD;
	MAV3_odomMsg.timestamp = uint64_t(poseMsg->header.stamp.sec)*1000000 + uint64_t(poseMsg->header.stamp.nanosec)/1000;
	MAV3_odomMsg.timestamp_sample = MAV3_odomMsg.timestamp;

	MAV3_odomMsg.position[0] = poseMsg->pose.position.x;
	MAV3_odomMsg.position[1] = -poseMsg->pose.position.y;
	MAV3_odomMsg.position[2] = -poseMsg->pose.position.z;

	MAV3_odomMsg.q[0] = poseMsg->pose.orientation.w;
	MAV3_odomMsg.q[1] = poseMsg->pose.orientation.x;
	MAV3_odomMsg.q[2] = - poseMsg->pose.orientation.y;
	MAV3_odomMsg.q[3] = - poseMsg->pose.orientation.z;
	MAV3_odomMsg.quality = 1;
	MAV3_odomPub -> publish(MAV3_odomMsg);
	RCLCPP_INFO_ONCE(get_logger(), "Sent to PX4 as:");
	RCLCPP_INFO_ONCE(get_logger(), "P: %f, %f, %f", MAV3_odomMsg.position[0], 
					MAV3_odomMsg.position[1], MAV3_odomMsg.position[2]);
	RCLCPP_INFO_ONCE(get_logger(), "q: %f, %f, %f, %f", MAV3_odomMsg.q[0],
					MAV3_odomMsg.q[1], MAV3_odomMsg.q[2], MAV3_odomMsg.q[3]);
}



void MocapPX4Bridge::MAV4_poseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr poseMsg){
	RCLCPP_INFO_ONCE(get_logger(), "Recived first msg from optitrack.");
	RCLCPP_INFO_ONCE(get_logger(), "P: %f, %f, %f", poseMsg->pose.position.x, 
					poseMsg->pose.position.y, poseMsg->pose.position.z);
	RCLCPP_INFO_ONCE(get_logger(), "q: %f, %f, %f, %f", poseMsg->pose.orientation.w,
					poseMsg->pose.orientation.x, poseMsg->pose.orientation.y, poseMsg->pose.orientation.z);
	px4_msgs::msg::VehicleOdometry MAV4_odomMsg;

	MAV4_odomMsg.pose_frame = MAV4_odomMsg.POSE_FRAME_FRD;
	MAV4_odomMsg.timestamp = uint64_t(poseMsg->header.stamp.sec)*1000000 + uint64_t(poseMsg->header.stamp.nanosec)/1000;
	MAV4_odomMsg.timestamp_sample = MAV4_odomMsg.timestamp;

	MAV4_odomMsg.position[0] = poseMsg->pose.position.x;
	MAV4_odomMsg.position[1] = -poseMsg->pose.position.y;
	MAV4_odomMsg.position[2] = -poseMsg->pose.position.z;

	MAV4_odomMsg.q[0] = poseMsg->pose.orientation.w;
	MAV4_odomMsg.q[1] = poseMsg->pose.orientation.x;
	MAV4_odomMsg.q[2] = - poseMsg->pose.orientation.y;
	MAV4_odomMsg.q[3] = - poseMsg->pose.orientation.z;
	MAV4_odomMsg.quality = 1;
	MAV4_odomPub -> publish(MAV4_odomMsg);
	RCLCPP_INFO_ONCE(get_logger(), "Sent to PX4 as:");
	RCLCPP_INFO_ONCE(get_logger(), "P: %f, %f, %f", MAV4_odomMsg.position[0], 
					MAV4_odomMsg.position[1], MAV4_odomMsg.position[2]);
	RCLCPP_INFO_ONCE(get_logger(), "q: %f, %f, %f, %f", MAV4_odomMsg.q[0],
					MAV4_odomMsg.q[1], MAV4_odomMsg.q[2], MAV4_odomMsg.q[3]);
}



void MocapPX4Bridge::MAV5_poseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr poseMsg){
	RCLCPP_INFO_ONCE(get_logger(), "Recived first msg from optitrack.");
	RCLCPP_INFO_ONCE(get_logger(), "P: %f, %f, %f", poseMsg->pose.position.x, 
					poseMsg->pose.position.y, poseMsg->pose.position.z);
	RCLCPP_INFO_ONCE(get_logger(), "q: %f, %f, %f, %f", poseMsg->pose.orientation.w,
					poseMsg->pose.orientation.x, poseMsg->pose.orientation.y, poseMsg->pose.orientation.z);
	px4_msgs::msg::VehicleOdometry MAV5_odomMsg;

	MAV5_odomMsg.pose_frame = MAV5_odomMsg.POSE_FRAME_FRD;
	MAV5_odomMsg.timestamp = uint64_t(poseMsg->header.stamp.sec)*1000000 + uint64_t(poseMsg->header.stamp.nanosec)/1000;
	MAV5_odomMsg.timestamp_sample = MAV5_odomMsg.timestamp;

	MAV5_odomMsg.position[0] = poseMsg->pose.position.x;
	MAV5_odomMsg.position[1] = -poseMsg->pose.position.y;
	MAV5_odomMsg.position[2] = -poseMsg->pose.position.z;

	MAV5_odomMsg.q[0] = poseMsg->pose.orientation.w;
	MAV5_odomMsg.q[1] = poseMsg->pose.orientation.x;
	MAV5_odomMsg.q[2] = - poseMsg->pose.orientation.y;
	MAV5_odomMsg.q[3] = - poseMsg->pose.orientation.z;
	MAV5_odomMsg.quality = 100;
	MAV5_odomPub -> publish(MAV5_odomMsg);
	RCLCPP_INFO_ONCE(get_logger(), "Sent to PX4 as:");
	RCLCPP_INFO_ONCE(get_logger(), "P: %f, %f, %f", MAV5_odomMsg.position[0], 
					MAV5_odomMsg.position[1], MAV5_odomMsg.position[2]);
	RCLCPP_INFO_ONCE(get_logger(), "q: %f, %f, %f, %f", MAV5_odomMsg.q[0],
					MAV5_odomMsg.q[1], MAV5_odomMsg.q[2], MAV5_odomMsg.q[3]);
}




int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MocapPX4Bridge>());
	rclcpp::shutdown();
	return 0;
}
