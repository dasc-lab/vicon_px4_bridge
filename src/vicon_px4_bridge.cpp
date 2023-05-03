#include "vicon_receiver/msg/position.hpp"
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <chrono>
//#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class ViconPX4Bridge : public rclcpp::Node {
public:
  ViconPX4Bridge() : Node("vicon_px4_bridge") {
    this->declare_parameter<std::string>("px4_name", "drone1");
    this->declare_parameter<std::string>("vicon_name", "drone1");
    std::string vicon_name;
    this->get_parameter("px4_name", px4_name);
    this->get_parameter("vicon_name", vicon_name);
    std::string vicon_sub_name = "vicon/" + vicon_name + "/" + vicon_name;
    
    std::string timesync_sub_name = px4_name + "/fmu/out/timesync";
    std::string px4_pub_name = px4_name + "/fmu/in/vehicle_visual_odometry";
    
    RCLCPP_INFO(this->get_logger(), "vicon_sub_name: %s \n",
                vicon_sub_name.c_str());
    RCLCPP_INFO(this->get_logger(), "timesync_sub_name: %s \n",
                timesync_sub_name.c_str());
    RCLCPP_INFO(this->get_logger(), "px4_pub_name: %s \n",
                px4_pub_name.c_str());
    RCLCPP_INFO(this->get_logger(), "Using Vicon -> Visual Odometry callback!");
    
    
    position_sub_ = this->create_subscription<vicon_receiver::msg::Position>(
        vicon_sub_name, 10,
	[this](const vicon_receiver::msg::Position::UniquePtr msg) {
		this->position_topic_callback(*msg);
		}
		);

    odometry_pub_ =
        this->create_publisher<px4_msgs::msg::VehicleOdometry>(
            px4_pub_name, 10);
    
    timesync_sub_ = this->create_subscription<px4_msgs::msg::TimesyncStatus>(
        timesync_sub_name, 10,
        [this](const px4_msgs::msg::TimesyncStatus::UniquePtr msg) {
          this->px4_timestamp_.store(msg->timestamp);
          this->px4_server_timestamp_.store(
              this->get_clock()->now().nanoseconds());
        });
  }

private:
  std::string px4_name;

  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr
      odometry_pub_;

  rclcpp::Subscription<vicon_receiver::msg::Position>::SharedPtr position_sub_;
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
  std::atomic<uint64_t> px4_timestamp_;
  std::atomic<uint64_t> px4_server_timestamp_;
  float _ref_lat = 42.2944420 * M_PI / 180.0;
  float _ref_lon = -83.7104540 * M_PI / 180.0;
  float _ref_sin_lat = sin(_ref_lat);
  float _ref_cos_lat = cos(_ref_lat);
  static constexpr double CONSTANTS_RADIUS_OF_EARTH = 6371000;

  uint64_t get_current_timestamp() {
    auto delta =
        (this->get_clock()->now().nanoseconds() - this->px4_server_timestamp_) /
        1e6;
    return px4_timestamp_.load() + delta;
  }

  void position_topic_callback(const vicon_receiver::msg::Position &position) {
    auto message = px4_msgs::msg::VehicleOdometry();

    // timestamps
    auto timestamp = get_current_timestamp();
    message.timestamp = timestamp;
    message.timestamp_sample = timestamp;

    // pose frame
    message.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

    //position
    message.position[0] = position.y_trans / 1000.0;
    message.position[1] = position.x_trans / 1000.0;
    message.position[2] = -position.z_trans / 1000.0;

    // quaternion
    tf2::Quaternion tf2_vicon(position.x_rot, position.y_rot, position.z_rot,
                              position.w);
    auto yaw = tf2::getYaw(tf2_vicon);
    auto yaw_ned = -yaw + 1.57;
    Eigen::Quaternion<double> out(cos(yaw_ned / 2.0), 0, 0, sin(yaw_ned / 2.0));
    
    message.q[0] = out.w();
    message.q[1] = out.x();
    message.q[2] = out.y();
    message.q[3] = out.z();
    
    // velocity frame 
    message.velocity_frame =
        px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;


    for (size_t i=0; i<3; i++) {

    // velocity
    message.velocity[i] = NAN;

    // angular velocity
    message.angular_velocity[i] = NAN;
    }


    // position covariance
    for (size_t i=0; i<3; i++) {
	    message.position_variance[i] = 0.01;
	    message.orientation_variance[i] = 0.0523; // approx 3 degrees
	    message.velocity_variance[i] = NAN;
    }
    
    message.reset_counter = 0;

    odometry_pub_->publish(message);

  }

  // void reproject(float x, float y, double &lat, double &lon) const {
  //   const double x_rad = (double)x / CONSTANTS_RADIUS_OF_EARTH;
  //   const double y_rad = (double)y / CONSTANTS_RADIUS_OF_EARTH;
  //   const double c = sqrt(x_rad * x_rad + y_rad * y_rad);

  //   if (fabs(c) > 0) {
  //     const double sin_c = sin(c);
  //     const double cos_c = cos(c);

  //     const double lat_rad =
  //         asin(cos_c * _ref_sin_lat + (x_rad * sin_c * _ref_cos_lat) / c);
  //     const double lon_rad =
  //         (_ref_lon + atan2(y_rad * sin_c, c * _ref_cos_lat * cos_c -
  //                                              x_rad * _ref_sin_lat * sin_c));

  //     lat = lat_rad * 180.0 / M_PI;
  //     lon = lon_rad * 180.0 / M_PI;

  //   } else {
  //     lat = _ref_lat * 180.0 / M_PI;
  //     lon = _ref_lon * 180.0 / M_PI;
  //   }
  // }

  //void rover_position_callback(const vicon_receiver::msg::Position &position) {
  //  double lat = NAN, lon = NAN;
  //  reproject(position.y_trans / 1000.0, position.x_trans / 1000.0, lat, lon);
  //  tf2::Quaternion tf2_vicon(position.x_rot, position.y_rot, position.z_rot,
  //                            position.w);
  //  auto yaw = tf2::getYaw(tf2_vicon);
  //  auto yaw_ned = -yaw + 1.57;
  //  auto gps_msg = px4_msgs::msg::SensorGps();
  //  gps_msg.timestamp = get_current_timestamp();
  //  gps_msg.device_id = 0;
  //  gps_msg.lat = lat * 1e7;
  //  gps_msg.lon = lon * 1e7;
  //  gps_msg.alt = position.z_trans;
  //  gps_msg.alt_ellipsoid = position.z_trans;
  //  gps_msg.s_variance_m_s = 0.1;
  //  gps_msg.c_variance_rad = 0.05;
  //  gps_msg.fix_type = 5;
  //  gps_msg.eph = 0.02; // 0.02;
  //  gps_msg.epv = 0.02; // 0.02;
  //  gps_msg.hdop = 1.0;
  //  gps_msg.vdop = 1.0;
  //  gps_msg.noise_per_ms = 101;
  //  gps_msg.jamming_indicator = 35;
  //  gps_msg.jamming_state = 0;
  //  gps_msg.vel_m_s = 0.0;
  //  gps_msg.vel_n_m_s = 0.0;
  //  gps_msg.vel_e_m_s = 0.0;
  //  gps_msg.vel_d_m_s = 0.0;
  //  gps_msg.cog_rad = NAN;
  //  gps_msg.vel_ned_valid = true;
  //  gps_msg.timestamp_time_relative = 0;
  //  gps_msg.satellites_used = 12;
  //  gps_msg.heading = yaw_ned;
  //  gps_msg.heading_offset = 0.0;
  //  gps_msg.heading_accuracy = 0.2;
  //  gps_pub_->publish(gps_msg);
  //}
};

int main(int argc, char *argv[]) {
  std::cout << "Starting vicon px4 bridge node..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ViconPX4Bridge>());

  rclcpp::shutdown();
  return 0;
}
