#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "vicon_receiver/msg/position.hpp"
#include <px4_msgs/msg/vehicle_visual_odometry.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <tf2/utils.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
//#include "px4_ros_com/frame_transforms.h"

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class ViconPX4Bridge : public rclcpp::Node
{
public:
	ViconPX4Bridge() : Node("vicon_px4_bridge") {
		this->declare_parameter<std::string>("px4_name", "drone1");
		this->declare_parameter<std::string>("vicon_name", "drone1");
		std::string vicon_name;
		this->get_parameter("px4_name", px4_name);
		this->get_parameter("vicon_name", vicon_name);
		std::string vicon_sub_name = "vicon/" + vicon_name + "/" + vicon_name;
		std::string timesync_sub_name = px4_name + "/fmu/timesync/out";
		std::string px4_pub_name = px4_name + "/fmu/vehicle_visual_odometry/in";
		std::string px4_gps_pub_name = px4_name + "/fmu/sensor_gps/in";
		RCLCPP_INFO(this->get_logger(), "vicon_sub_name: %s \n", vicon_sub_name.c_str());
		RCLCPP_INFO(this->get_logger(), "timesync_sub_name: %s \n", timesync_sub_name.c_str());
		RCLCPP_INFO(this->get_logger(), "px4_pub_name: %s \n", px4_pub_name.c_str());
		RCLCPP_INFO(this->get_logger(), "px4_gps_pub_name: %s \n", px4_gps_pub_name.c_str());
		// if (px4_name.find("rover") != std::string::npos) {
		// 	RCLCPP_INFO(this->get_logger(), "Using rover callback!");
		// 	position_sub_ = this->create_subscription<vicon_receiver::msg::Position>(vicon_sub_name, 10, std::bind(&ViconPX4Bridge::rover_position_callback, this, _1));
		// }
		// else {
		// 	RCLCPP_INFO(this->get_logger(), "Using drone callback!");
		// 	position_sub_ = this->create_subscription<vicon_receiver::msg::Position>(vicon_sub_name, 10, std::bind(&ViconPX4Bridge::position_topic_callback, this, _1));
		// }
		RCLCPP_INFO(this->get_logger(), "Using Vicon -> Visual Odometry callback!");
		position_sub_ = this->create_subscription<vicon_receiver::msg::Position>(vicon_sub_name, 10, std::bind(&ViconPX4Bridge::position_topic_callback, this, _1));
		odometry_pub_ = this->create_publisher<px4_msgs::msg::VehicleVisualOdometry>(px4_pub_name, 10);
		gps_pub_ = this->create_publisher<px4_msgs::msg::SensorGps>(px4_gps_pub_name, 10);
		timesync_sub_ =
		this->create_subscription<px4_msgs::msg::Timesync>(timesync_sub_name, 10,
			[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
				this->px4_timestamp_.store(msg->timestamp);
    			this->px4_server_timestamp_.store(this->get_clock()->now().nanoseconds());
			});
    
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
	}

private:

  std::string px4_name;


	rclcpp::Publisher<px4_msgs::msg::SensorGps>::SharedPtr gps_pub_;
	rclcpp::Publisher<px4_msgs::msg::VehicleVisualOdometry>::SharedPtr odometry_pub_;
  
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  rclcpp::Subscription<vicon_receiver::msg::Position>::SharedPtr position_sub_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	std::atomic<uint64_t> px4_timestamp_;
	std::atomic<uint64_t> px4_server_timestamp_;
	float _ref_lat = 42.2944420 * M_PI / 180.0;
	float _ref_lon = -83.7104540 * M_PI / 180.0;
	float _ref_sin_lat = sin(_ref_lat);
	float _ref_cos_lat = cos(_ref_lat);
	static constexpr double CONSTANTS_RADIUS_OF_EARTH = 6371000;
	//Eigen::Matrix3d rotation_ned2lab;
	
	//  {
    //   {0, 1, 0},
    //   {1, 0, 0},
	//   {0, 0, -1}
	// };

	uint64_t get_current_timestamp() {
		auto delta = (this->get_clock()->now().nanoseconds() - this->px4_server_timestamp_) / 1e6;
		// RCLCPP_INFO(this->get_logger(), "px4: %lu", px4_timestamp_.load());
		// RCLCPP_INFO(this->get_logger(), "delta: %lu", delta);
		// RCLCPP_INFO(this->get_logger(), "time: %lu", px4_timestamp_.load() + delta);
		return px4_timestamp_.load() + delta;
	}

	void position_topic_callback(const vicon_receiver::msg::Position & position) {
		auto message = px4_msgs::msg::VehicleVisualOdometry();
		auto timestamp = get_current_timestamp();
		message.timestamp = timestamp;
		message.timestamp_sample = timestamp;
		message.local_frame = px4_msgs::msg::VehicleVisualOdometry::LOCAL_FRAME_NED;
		message.x = position.y_trans / 1000.0;
		message.y = position.x_trans / 1000.0;
		message.z = -position.z_trans / 1000.0;
		tf2::Quaternion tf2_vicon(position.x_rot, position.y_rot, position.z_rot, position.w);
		auto yaw = tf2::getYaw(tf2_vicon);
		auto yaw_ned = -yaw + 1.57;
		Eigen::Quaternion<double> out(cos(yaw_ned / 2.0), 0, 0, sin(yaw_ned / 2.0));
		// Eigen::Quaternion<double> vicon_quat(position.w, position.x_rot, position.y_rot, position.z_rot);
		// auto rotation_lab2quad = vicon_quat.toRotationMatrix();
		// auto rotation_ned2quad = rotation_ned2lab * rotation_lab2quad;
		// Eigen::Quaternion<double> out(rotation_ned2quad);

		// Eigen::Quaternion<double> identity_quat(1, 0, 0, 0);
		// auto Q_ENU = px4_ros_com::frame_transforms::enu_to_ned_orientation(identity_quat);
		// auto Q_ENU_CONJ = Eigen::Quaternion<double>(Q_ENU.w(), -Q_ENU.x(), -Q_ENU.y(), -Q_ENU.z());
		// auto out = Q_ENU * vicon_quat * Q_ENU_CONJ;

		// out.normalize();
		// auto vicon_euler = px4_ros_com::frame_transforms::utils::quaternion::quaternion_to_euler(vicon_quat);
		// auto out_euler = px4_ros_com::frame_transforms::utils::quaternion::quaternion_to_euler(out);
		// RCLCPP_INFO(this->get_logger(), "w: %f, x: %f, y: %f, z: %f \n", Q_ENU.w(), Q_ENU.x(), Q_ENU.y(), Q_ENU.z());
		// RCLCPP_INFO(this->get_logger(), "vicon: w: %f, x: %f, y: %f, z: %f \n", vicon_quat.w(), vicon_quat.x(), vicon_quat.y(), vicon_quat.z());
		// RCLCPP_INFO(this->get_logger(), "out:   w: %f, x: %f, y: %f, z: %f \n", out.w(), out.x(), out.y(), out.z());
		// RCLCPP_INFO(this->get_logger(), "yaw: %f \n", yaw * 180 / 3.14);
		// RCLCPP_INFO(this->get_logger(), "yaw: %f \n", yaw_ned * 180 / 3.14);
		
		// RCLCPP_INFO(this->get_logger(), "roll: %f, pitch: %f, yaw: %f\n", vicon_euler[0], vicon_euler[1], vicon_euler[2]);
		// RCLCPP_INFO(this->get_logger(), "roll: %f, pitch: %f, yaw: %f\n", out_euler[0], out_euler[1], out_euler[2]);

		// message.q[0] = position.w;
		// message.q[1] = position.y_rot;
		// message.q[2] = position.x_rot;
		// message.q[3] = -position.z_rot;
		message.q[0] = out.w();
		message.q[1] = out.x();
		message.q[2] = out.y();
		message.q[3] = out.z();
		message.q_offset[0] = 1.0;
		message.q_offset[1] = 0.0;
		message.q_offset[2] = 0.0;
		message.q_offset[3] = 0.0;
		// message.pose_covariance[0] = NAN;
		// message.pose_covariance[15] = NAN;
		message.pose_covariance[px4_msgs::msg::VehicleVisualOdometry::COVARIANCE_MATRIX_X_VARIANCE] = 0.01;
		message.pose_covariance[px4_msgs::msg::VehicleVisualOdometry::COVARIANCE_MATRIX_Y_VARIANCE] = 0.01;
		message.pose_covariance[px4_msgs::msg::VehicleVisualOdometry::COVARIANCE_MATRIX_Z_VARIANCE] = 0.01;
		message.pose_covariance[px4_msgs::msg::VehicleVisualOdometry::COVARIANCE_MATRIX_ROLL_VARIANCE] = 0.0523599;
		message.pose_covariance[px4_msgs::msg::VehicleVisualOdometry::COVARIANCE_MATRIX_PITCH_VARIANCE] = 0.0523599;
		message.pose_covariance[px4_msgs::msg::VehicleVisualOdometry::COVARIANCE_MATRIX_YAW_VARIANCE] = 0.0523599;

		message.velocity_frame = px4_msgs::msg::VehicleVisualOdometry::LOCAL_FRAME_NED;
		message.vx = NAN;
		message.vy = NAN;
		message.vz = NAN;
		message.rollspeed = NAN;
		message.pitchspeed = NAN;
		message.yawspeed = NAN;
		message.velocity_covariance[0] = NAN;
		message.velocity_covariance[15] = NAN;
		message.reset_counter = 0;

		// std::cout << "publishing pose" << std::endl;
    odometry_pub_->publish(message);

    // publish the tf transform
    geometry_msgs::msg::TransformStamped _tf;
    _tf.header.stamp = this->get_clock()->now();
    _tf.header.frame_id = "world";
    _tf.child_frame_id = ("vicon/" + px4_name).c_str();

    _tf.transform.translation.x = position.x_trans / 1000.0;
    _tf.transform.translation.y = position.y_trans / 1000.0;
    _tf.transform.translation.z = position.z_trans / 1000.0;

    _tf.transform.rotation.x = position.x_rot;
    _tf.transform.rotation.y = position.y_rot;
    _tf.transform.rotation.z = position.z_rot;
    _tf.transform.rotation.w = position.w;

    tf_broadcaster_->sendTransform(_tf);

	}

	void reproject(float x, float y, double &lat, double &lon) const
	{
		const double x_rad = (double)x / CONSTANTS_RADIUS_OF_EARTH;
		const double y_rad = (double)y / CONSTANTS_RADIUS_OF_EARTH;
		const double c = sqrt(x_rad * x_rad + y_rad * y_rad);

		if (fabs(c) > 0) {
			const double sin_c = sin(c);
			const double cos_c = cos(c);

			const double lat_rad = asin(cos_c * _ref_sin_lat + (x_rad * sin_c * _ref_cos_lat) / c);
			const double lon_rad = (_ref_lon + atan2(y_rad * sin_c, c * _ref_cos_lat * cos_c - x_rad * _ref_sin_lat * sin_c));

			lat = lat_rad * 180.0 / M_PI;
			lon = lon_rad * 180.0 / M_PI;

		} else {
			lat = _ref_lat * 180.0 / M_PI;
			lon = _ref_lon * 180.0 / M_PI;
		}
	}

	void rover_position_callback(const vicon_receiver::msg::Position & position) {
		double lat = NAN, lon = NAN;
		reproject(position.y_trans / 1000.0, position.x_trans / 1000.0, lat, lon);
		tf2::Quaternion tf2_vicon(position.x_rot, position.y_rot, position.z_rot, position.w);
		auto yaw = tf2::getYaw(tf2_vicon);
		auto yaw_ned = -yaw + 1.57;
		auto gps_msg = px4_msgs::msg::SensorGps();
		gps_msg.timestamp = get_current_timestamp();
		gps_msg.device_id = 0;
		gps_msg.lat = lat * 1e7;
		gps_msg.lon = lon * 1e7;
		gps_msg.alt = position.z_trans;
		gps_msg.alt_ellipsoid = position.z_trans;
		gps_msg.s_variance_m_s = 0.1;
		gps_msg.c_variance_rad = 0.05;
		gps_msg.fix_type = 5;
		gps_msg.eph = 0.02;//0.02;
		gps_msg.epv = 0.02;//0.02;
		gps_msg.hdop = 1.0;
		gps_msg.vdop = 1.0;
		gps_msg.noise_per_ms = 101;
		gps_msg.jamming_indicator = 35;
		gps_msg.jamming_state = 0;
		gps_msg.vel_m_s = 0.0;
		gps_msg.vel_n_m_s = 0.0;
		gps_msg.vel_e_m_s = 0.0;
		gps_msg.vel_d_m_s = 0.0;
		gps_msg.cog_rad = NAN;
		gps_msg.vel_ned_valid = true;
		gps_msg.timestamp_time_relative = 0;
		gps_msg.satellites_used = 12;
		gps_msg.heading = yaw_ned;
		gps_msg.heading_offset = 0.0;
		gps_msg.heading_accuracy = 0.2;
		gps_pub_->publish(gps_msg);
	}
};

int main(int argc, char *argv[])
{
	std::cout << "Starting vicon px4 bridge node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ViconPX4Bridge>());

	rclcpp::shutdown();
	return 0;
}
