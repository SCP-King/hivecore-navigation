#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

class ScanValidator final : public rclcpp::Node {
public:
  ScanValidator() : Node("scan_validator"), window_start_(get_clock()->now()) {
    input_topic_ = declare_parameter<std::string>("input_topic", "/livox/lidar/geo_filtered");
    expected_frame_ = declare_parameter<std::string>("input_frame", "lidar_link");
    min_rate_hz_ = declare_parameter<double>("min_rate_hz", 2.0);
    diagnostic_topic_ = declare_parameter<std::string>("diagnostic_topic", "/n01/scan_readiness");

    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostic_topic_, 10);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ScanValidator::on_cloud, this, std::placeholders::_1));
    timer_ = create_wall_timer(1s, std::bind(&ScanValidator::publish_status, this));

    RCLCPP_WARN(get_logger(),
      "S3 read-only validator: input=%s expected_frame=%s; "
      "geo_filtered requires vendor GUI-SLAM/preprocessor enable_forward=true; "
      "raw /livox/lidar is never selected automatically.", input_topic_.c_str(), expected_frame_.c_str());
  }

private:
  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    ++received_;
    last_frame_ = msg->header.frame_id;
    last_stamp_ = rclcpp::Time(msg->header.stamp);
    if (last_frame_ != expected_frame_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
        "PointCloud2 frame mismatch: expected '%s', received '%s'; fail closed.",
        expected_frame_.c_str(), last_frame_.c_str());
    }
  }

  void publish_status() {
    const auto now = get_clock()->now();
    const double elapsed = (now - window_start_).seconds();
    const double rate = elapsed > 0.0 ? static_cast<double>(received_) / elapsed : 0.0;
    const bool frame_ok = !last_frame_.empty() && last_frame_ == expected_frame_;
    const bool ready = received_ > 0 && frame_ok && rate >= min_rate_hz_;

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "n01_scan/input_readiness";
    status.hardware_id = input_topic_;
    status.level = ready ? diagnostic_msgs::msg::DiagnosticStatus::OK
                         : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = ready ? "real PointCloud2 input ready" : "input not ready; fail closed";
    add(status, "input_topic", input_topic_);
    add(status, "received_frame", last_frame_);
    add(status, "expected_frame", expected_frame_);
    add(status, "rate_hz", std::to_string(rate));
    add(status, "received_messages", std::to_string(received_));
    array.status.push_back(status);
    diagnostics_->publish(array);

    if (ready) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "S3 input ready: %s frame=%s rate=%.2fHz", input_topic_.c_str(),
        last_frame_.c_str(), rate);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "S3 input not ready: topic=%s frame='%s' rate=%.2fHz; no fallback.",
        input_topic_.c_str(), last_frame_.c_str(), rate);
    }
    received_ = 0;
    window_start_ = now;
  }

  static void add(diagnostic_msgs::msg::DiagnosticStatus &status,
                  const std::string &key, const std::string &value) {
    diagnostic_msgs::msg::KeyValue pair;
    pair.key = key;
    pair.value = value;
    status.values.push_back(pair);
  }

  std::string input_topic_;
  std::string expected_frame_;
  std::string diagnostic_topic_;
  std::string last_frame_;
  double min_rate_hz_{};
  uint64_t received_{};
  rclcpp::Time window_start_{get_clock()->now()};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanValidator>());
  rclcpp::shutdown();
  return 0;
}
