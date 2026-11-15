#include <algorithm>
#include <iostream>
#include <memory>
#include <cstdlib>
#include <sstream>
#include <iomanip>
#include <sys/prctl.h>
#include <signal.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Humble rclpy discards publisher GID. rclcpp preserves it in MessageInfo.
// This helper only subscribes; it never publishes a robot command.
int main(int argc, char ** argv) {
  const auto parent = getppid();
  prctl(PR_SET_PDEATHSIG, SIGTERM);
  if (getppid() != parent || parent == 1) return 1;
  const char * configured_gid = std::getenv("N01_EXECUTOR_PUBLISHER_GID");
  if (!configured_gid) return 2;
  const std::string own_gid(configured_gid);
  if (own_gid.size() != RMW_GID_STORAGE_SIZE * 2) return 2;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("n01_command_source_observer");
  auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::SensorDataQoS(),
    [node, own_gid](geometry_msgs::msg::Twist::ConstSharedPtr,
           const rclcpp::MessageInfo & info) {
      const auto & gid = info.get_rmw_message_info().publisher_gid;
      std::ostringstream identity;
      for (const auto byte : gid.data)
        identity << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(byte);
      // Match the actual publisher, even before remote graph discovery knows
      // its node name. Never exempt another publisher merely by node name.
      if (identity.str() == own_gid) return;
      for (const auto & endpoint : node->get_publishers_info_by_topic("/cmd_vel")) {
        const auto & candidate = endpoint.endpoint_gid();
        if (!std::equal(candidate.begin(), candidate.end(), gid.data)) continue;
        std::cout << "SOURCE " << endpoint.node_namespace() << "/"
                  << endpoint.node_name() << std::endl;
        return;
      }
      std::cout << "SOURCE unknown_external_publisher" << std::endl;
    });
  std::cout << "READY" << std::endl;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
