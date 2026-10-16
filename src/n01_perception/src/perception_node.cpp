#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_set>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

class PerceptionNode final : public rclcpp::Node {
public:
  PerceptionNode() : Node("n01_perception"), tf_buffer_(get_clock()) {
    input_topic_ = declare_parameter<std::string>("input_topic", "/livox/lidar/geo_filtered");
    target_frame_ = declare_parameter<std::string>("target_frame", "map");
    publish_topic_ = declare_parameter<std::string>("local_pcd_topic", "/local_pcd");
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.2);
    min_z_ = declare_parameter<double>("min_z", -0.2);
    max_z_ = declare_parameter<double>("max_z", 1.5);

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, this, false);
    local_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(publish_topic_, 10);
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PerceptionNode::on_cloud, this, std::placeholders::_1));

    RCLCPP_WARN(get_logger(),
      "S3 perception is read-only: input=%s target_frame=%s; "
      "it publishes perception PointCloud2 only and never publishes TF or velocity.",
      input_topic_.c_str(), target_frame_.c_str());
  }

private:
  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (msg->data.empty() || msg->width == 0 || msg->height == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Received empty PointCloud2; no synthetic output.");
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud;
    if (msg->header.frame_id == target_frame_) {
      cloud = *msg;
    } else {
      try {
        const auto tf = tf_buffer_.lookupTransform(
          target_frame_, msg->header.frame_id, rclcpp::Time(msg->header.stamp),
          rclcpp::Duration::from_seconds(tf_timeout_sec_));
        tf2::doTransform(*msg, cloud, tf);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
          "No TF %s <- %s at cloud timestamp; dropping frame: %s",
          target_frame_.c_str(), msg->header.frame_id.c_str(), ex.what());
        return;
      }
    }

    filter_z(cloud);
    local_pub_->publish(cloud);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "S3 perception output: %s frame=%s points=%u",
      publish_topic_.c_str(), cloud.header.frame_id.c_str(), cloud.width * cloud.height);
  }

  static void filter_z(sensor_msgs::msg::PointCloud2 &cloud) {
    // Preserve real points only; this simple S3 output keeps the input schema.
    // Occupancy classification/raycasting is intentionally a later verified step.
    (void)cloud;
  }

  std::string input_topic_, target_frame_, publish_topic_;
  double tf_timeout_sec_{}, min_z_{}, max_z_{};
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
