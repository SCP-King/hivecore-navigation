// N-01 trajectory tracker.
//
// Port of SCAN-Planner's upstream `closed_loop_controller.cpp` (ROS 1) to ROS 2,
// hardened for the Hive2 4WIS chassis:
//   * disabled by default (enable topic / parameter must be true to output /cmd_vel)
//   * double clamping (planner max_vel and chassis limits)
//   * watchdog: stale / missing B-spline -> zero twist
//   * emits a JSON status string for the operator UI
//
// Hive2 is a four-wheel independent steering (4WIS / Swerve) chassis: the
// velocity space is (vx, vy, omega), so the upstream all-directional command is
// forwarded as-is instead of being projected onto a differential-drive model.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <bspline_opt/uniform_bspline.h>

namespace
{

using scan_planner::UniformBspline;

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double clampValue(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

Eigen::Vector3d toVector3(const Eigen::VectorXd & v)
{
  return Eigen::Vector3d(v(0), v(1), v(2));
}

}  // namespace

class TrajTracker : public rclcpp::Node
{
public:
  TrajTracker()
  : Node("n01_traj_tracker")
  {
    bspline_topic_ = declare_parameter<std::string>("bspline_topic", "/planning/bspline");
    body_pose_topic_ = declare_parameter<std::string>("body_pose_topic", "/body_pose");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    status_topic_ = declare_parameter<std::string>("status_topic", "/n01/traj_tracker/status");
    enable_topic_ = declare_parameter<std::string>("enable_topic", "/n01/traj_tracker/enable");

    enable_ = declare_parameter<bool>("enable", false);
    time_forward_ = declare_parameter<double>("time_forward", 0.5);
    heading_error_threshold_ = declare_parameter<double>("heading_error_threshold", 0.6);
    kp_pos_ = declare_parameter<double>("kp_pos", 1.0);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.2);
    max_vx_ = declare_parameter<double>("max_vx", 0.30);
    max_vy_ = declare_parameter<double>("max_vy", 0.30);
    max_vyaw_ = declare_parameter<double>("max_vyaw", 0.40);
    finish_dist_ = declare_parameter<double>("finish_dist", 0.15);
    cmd_rate_hz_ = declare_parameter<double>("cmd_rate_hz", 20.0);
    plan_timeout_ = declare_parameter<double>("plan_timeout", 0.35);
    pose_timeout_ = declare_parameter<double>("pose_timeout", 0.35);

    if (cmd_rate_hz_ < 1.0) {
      cmd_rate_hz_ = 1.0;
    }

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
    frozen_pub_ = create_publisher<std_msgs::msg::Bool>("~/rotating_in_place", 10);

    // /planning/bspline is published RELIABLE by scan_planner_node.
    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
      bspline_topic_, rclcpp::QoS(10),
      std::bind(&TrajTracker::onBspline, this, std::placeholders::_1));

    // /body_pose is published with sensor-data QoS (BEST_EFFORT).
    pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      body_pose_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TrajTracker::onBodyPose, this, std::placeholders::_1));

    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      enable_topic_, rclcpp::QoS(10),
      std::bind(&TrajTracker::onEnable, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / cmd_rate_hz_);
    timer_ = create_wall_timer(period, std::bind(&TrajTracker::tick, this));

    RCLCPP_WARN(
      get_logger(),
      "n01_traj_tracker ready. enable=%s rate=%.1fHz limits vx=%.2f vy=%.2f w=%.2f "
      "-> cmd_vel=%s (topic gate: %s)",
      enable_ ? "true" : "false", cmd_rate_hz_, max_vx_, max_vy_, max_vyaw_,
      cmd_vel_topic_.c_str(), enable_topic_.c_str());
  }

private:
  void onBspline(const scan_planner_msgs::msg::Bspline::SharedPtr msg)
  {
    if (msg->pos_pts.size() < 4) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "rejecting B-spline with %zu control points (need >= 4)",
        msg->pos_pts.size());
      return;
    }
    if (msg->knots.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "rejecting B-spline without knots");
      return;
    }

    Eigen::MatrixXd pos_pts(3, static_cast<int>(msg->pos_pts.size()));
    for (size_t i = 0; i < msg->pos_pts.size(); ++i) {
      pos_pts(0, static_cast<int>(i)) = msg->pos_pts[i].x;
      pos_pts(1, static_cast<int>(i)) = msg->pos_pts[i].y;
      pos_pts(2, static_cast<int>(i)) = msg->pos_pts[i].z;
    }

    Eigen::VectorXd knots(static_cast<int>(msg->knots.size()));
    for (size_t i = 0; i < msg->knots.size(); ++i) {
      knots(static_cast<int>(i)) = msg->knots[i];
    }

    try {
      UniformBspline pos_traj(pos_pts, msg->order, 0.1);
      pos_traj.setKnot(knots);
      traj_.clear();
      traj_.push_back(pos_traj);
      traj_.push_back(traj_[0].getDerivative());
      traj_.push_back(traj_[1].getDerivative());
      traj_duration_ = traj_[0].getTimeSum();
    } catch (const std::exception & exc) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "failed to build B-spline: %s", exc.what());
      traj_.clear();
      recv_traj_ = false;
      return;
    }

    if (!(traj_duration_ > 0.05) || !std::isfinite(traj_duration_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "B-spline duration invalid: %.3f", traj_duration_);
      traj_.clear();
      recv_traj_ = false;
      return;
    }

    traj_id_ = msg->traj_id;
    exec_time_ = 0.0;
    last_bspline_time_ = now();
    recv_traj_ = true;
    finished_ = false;
    last_dt_time_ = std::nullopt;
  }

  void onBodyPose(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_pos_ = Eigen::Vector3d(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z);
    odom_yaw_ = yawFromQuaternion(msg->pose.pose.orientation);
    last_pose_time_ = now();
    have_odom_ = true;
  }

  void onEnable(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool requested = msg->data;
    if (requested == enable_) {
      return;
    }
    enable_ = requested;
    if (!enable_) {
      publishZero();
    } else {
      last_dt_time_ = std::nullopt;
    }
    RCLCPP_WARN(
      get_logger(), "trajectory tracking %s by operator", enable_ ? "ENABLED" : "DISABLED");
  }

  double horizonYaw(double t_cur, const Eigen::Vector3d & pos_des) const
  {
    const double t_look = std::min(traj_duration_, t_cur + time_forward_);
    Eigen::Vector3d dir = toVector3(traj_[0].evaluateDeBoorT(t_look)) - pos_des;
    if (dir.head<2>().squaredNorm() < 1e-4) {
      dir = toVector3(traj_[1].evaluateDeBoorT(t_cur));
    }
    if (dir.head<2>().squaredNorm() < 1e-4) {
      return odom_yaw_;
    }
    return std::atan2(dir(1), dir(0));
  }

  void publishZero()
  {
    if (!enable_) {
      return;
    }
    cmd_pub_->publish(geometry_msgs::msg::Twist());
  }

  void publishStatus(const std::string & state, double vx, double vy, double wz)
  {
    std::ostringstream os;
    os.setf(std::ios::fixed);
    os.precision(3);
    os << "{"
       << "\"enabled\":" << (enable_ ? "true" : "false")
       << ",\"state\":\"" << state << "\""
       << ",\"cmd\":{\"vx\":" << vx << ",\"vy\":" << vy << ",\"wz\":" << wz << "}"
       << ",\"traj_id\":" << traj_id_
       << ",\"traj_duration\":" << traj_duration_
       << ",\"exec_time\":" << exec_time_
       << ",\"pose\":{\"x\":" << odom_pos_.x() << ",\"y\":" << odom_pos_.y()
       << ",\"yaw\":" << odom_yaw_ << "}"
       << ",\"have_pose\":" << (have_odom_ ? "true" : "false")
       << ",\"have_traj\":" << (recv_traj_ ? "true" : "false")
       << ",\"limits\":{\"vx\":" << max_vx_ << ",\"vy\":" << max_vy_
       << ",\"wz\":" << max_vyaw_ << "}"
       << "}";
    std_msgs::msg::String msg;
    msg.data = os.str();
    status_pub_->publish(msg);
  }

  void tick()
  {
    const rclcpp::Time now_time = now();

    if (!enable_) {
      publishStatus("DISABLED", 0.0, 0.0, 0.0);
      return;
    }
    if (!have_odom_) {
      publishZero();
      publishStatus("WAITING_FOR_POSE", 0.0, 0.0, 0.0);
      return;
    }
    if (!have_odom_ || (now_time - last_pose_time_).seconds() > pose_timeout_) {
      publishZero();
      publishStatus("POSE_STALE", 0.0, 0.0, 0.0);
      return;
    }
    if (!recv_traj_) {
      publishZero();
      publishStatus("WAITING_FOR_TRAJ", 0.0, 0.0, 0.0);
      return;
    }
    // SCAN re-plans on the order of seconds (bspline smoothing alone takes
    // ~3.8 s) while a single B-spline covers ~10 s of motion. So a short
    // "no new trajectory" watchdog would zero the command almost always.
    // Keep executing the trajectory we hold; only treat it as stale once it
    // has run out AND no replacement has arrived within a bounded grace.
    const double since_bspline = (now_time - last_bspline_time_).seconds();
    const double stale_limit = std::max(3.0, traj_duration_);
    if (since_bspline > stale_limit) {
      publishZero();
      publishStatus("TRAJ_STALE", 0.0, 0.0, 0.0);
      return;
    }

    double dt = 0.0;
    if (last_dt_time_.has_value()) {
      dt = (now_time - last_dt_time_.value()).seconds();
      if (dt < 0.0 || dt > 0.5) {
        dt = 0.0;
      }
    }
    last_dt_time_ = now_time;

    const double t_eval = std::min(exec_time_, traj_duration_);
    Eigen::Vector3d pos_des = toVector3(traj_[0].evaluateDeBoorT(t_eval));
    const double yaw_des = horizonYaw(t_eval, pos_des);
    const double yaw_err = normalizeAngle(yaw_des - odom_yaw_);
    const double wz = clampValue(kp_yaw_ * yaw_err, -max_vyaw_, max_vyaw_);

    std_msgs::msg::Bool frozen;
    if (std::fabs(yaw_err) > heading_error_threshold_) {
      frozen.data = true;
      frozen_pub_->publish(frozen);
      geometry_msgs::msg::Twist cmd;
      cmd.angular.z = wz;
      cmd_pub_->publish(cmd);
      publishStatus("ROTATING_IN_PLACE", 0.0, 0.0, wz);
      return;
    }
    frozen.data = false;
    frozen_pub_->publish(frozen);

    exec_time_ = std::min(traj_duration_, exec_time_ + dt);
    pos_des = toVector3(traj_[0].evaluateDeBoorT(exec_time_));
    Eigen::Vector3d vel_des = toVector3(traj_[1].evaluateDeBoorT(exec_time_));

    const Eigen::Vector2d pos_err(pos_des.x() - odom_pos_.x(), pos_des.y() - odom_pos_.y());
    const Eigen::Vector2d vel_ff(vel_des.x(), vel_des.y());
    Eigen::Vector2d vel_world = vel_ff + kp_pos_ * pos_err;

    const double v_cap = std::max(max_vx_, max_vy_);
    const double norm = vel_world.norm();
    if (norm > v_cap && norm > 1e-9) {
      vel_world = vel_world / norm * v_cap;
    }

    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = clampValue(c * vel_world.x() + s * vel_world.y(), -max_vx_, max_vx_);
    cmd.linear.y = clampValue(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);
    cmd.angular.z = wz;

    std::string state = "TRACKING";
    if (exec_time_ >= traj_duration_ && pos_err.norm() < finish_dist_) {
      cmd = geometry_msgs::msg::Twist();
      state = "FINISHED";
      finished_ = true;
    }
    cmd_pub_->publish(cmd);
    publishStatus(state, cmd.linear.x, cmd.linear.y, cmd.angular.z);
  }

  std::string bspline_topic_;
  std::string body_pose_topic_;
  std::string cmd_vel_topic_;
  std::string status_topic_;
  std::string enable_topic_;

  bool enable_{false};
  bool have_odom_{false};
  bool recv_traj_{false};
  bool finished_{false};

  double time_forward_{0.5};
  double heading_error_threshold_{0.6};
  double kp_pos_{1.0};
  double kp_yaw_{1.2};
  double max_vx_{0.30};
  double max_vy_{0.30};
  double max_vyaw_{0.40};
  double finish_dist_{0.15};
  double cmd_rate_hz_{20.0};
  double plan_timeout_{0.35};
  double pose_timeout_{0.35};

  double traj_duration_{0.0};
  double exec_time_{0.0};
  int64_t traj_id_{0};
  double odom_yaw_{0.0};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};

  rclcpp::Time last_bspline_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};
  std::optional<rclcpp::Time> last_dt_time_;

  std::vector<UniformBspline> traj_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr frozen_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajTracker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
