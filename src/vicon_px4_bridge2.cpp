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
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

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
    vicon_sub_name_ = "vicon/" + vicon_name + "/" + vicon_name;
    
    std::string timesync_sub_name = px4_name + "/fmu/out/timesync";
    std::string px4_pub_name = px4_name + "/fmu/in/vehicle_visual_odometry";
    
    RCLCPP_INFO(this->get_logger(), "vicon_sub_name: %s \n",
                vicon_sub_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "timesync_sub_name: %s \n",
                timesync_sub_name.c_str());
    RCLCPP_INFO(this->get_logger(), "px4_pub_name: %s \n",
                px4_pub_name.c_str());
    RCLCPP_INFO(this->get_logger(), "Using Vicon -> Visual Odometry callback!");
    
    
//     position_sub_ = this->create_subscription<vicon_receiver::msg::Position>(
//         vicon_sub_name, 10,
// 	[this](const vicon_receiver::msg::Position::UniquePtr msg) {
// 		this->position_topic_callback(*msg);
// 		}
// 		);
//
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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

    timer_ = this->create_wall_timer(
      100ms, std::bind(&ViconPX4Bridge::timer_callback, this));
  }

private:
  std::string px4_name;
  std::string vicon_sub_name_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr timer_{nullptr};

  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr
      odometry_pub_;

  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
  std::atomic<uint64_t> px4_timestamp_;
  std::atomic<uint64_t> px4_server_timestamp_;

  uint64_t get_current_timestamp() {
    auto delta =
        (this->get_clock()->now().nanoseconds() - this->px4_server_timestamp_) /
        1e6;
    return px4_timestamp_.load() + delta;
  }

  void timer_callback() {
	  
     std::string fromFrameRel = "vicon/world/NED";
    std::string toFrameRel = vicon_sub_name_; 

	  // see if you can get a position from vicon
     geometry_msgs::msg::TransformStamped t;

     try {
          t = tf_buffer_->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return;
        }


    auto message = px4_msgs::msg::VehicleOdometry();

    // timestamps
    auto timestamp = get_current_timestamp();
    message.timestamp = timestamp;
    message.timestamp_sample = timestamp;

    // pose frame
    message.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

    //position
    message.position[0] = t.transform.translation.x; 
    message.position[1] = t.transform.translation.y;
    message.position[2] = t.transform.translation.z;

    tf2::Quaternion tf2_vicon(t.transform.rotation.x,
		    t.transform.rotation.y,
		    t.transform.rotation.z,
		    t.transform.rotation.w);

    auto yaw_ned = tf2::getYaw(tf2_vicon);
    Eigen::Quaternion<double> out( std::cos(yaw_ned / 2.0), 0, 0, sin(yaw_ned/2.0));

    message.q[0] = out.w();
    message.q[1] = out.x();
    message.q[2] = out.y();
    message.q[3] = out.z();
    
    // message.q[0] = t.transform.rotation.w;
    // message.q[1] = t.transform.rotation.x;
    // message.q[2] = t.transform.rotation.y;
    // message.q[3] = t.transform.rotation.z;
    
    // velocity frame 
    message.velocity_frame =
        px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;


    for (size_t i=0; i<3; i++) {
      message.velocity[i] = NAN;
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

};

int main(int argc, char *argv[]) {
  std::cout << "Starting vicon px4 bridge node..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ViconPX4Bridge>());

  rclcpp::shutdown();
  return 0;
}
