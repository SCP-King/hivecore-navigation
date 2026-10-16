#include <algorithm>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "voxel_map.hpp"
#include "ground_filter.hpp"
#include "map_policy.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace n01_scan {

// S4 world-coordinate SCAN occupancy map.
//
// Two modes:
//  - sensor_only  (S3 semantics preserved): each /local_pcd frame is a
//    single-frame snapshot. Raycasts are cast from the cloud's own frame
//    origin (0,0,0) of the input frame; the map is reset every frame; the
//    publish frame is the input frame. No TF, no accumulation -- byte-identical
//    semantics to the S3 node.
//  - map_localized (S4): world-coordinate sliding map. The perception node has
//    already transformed /livox/lidar/geo_filtered -> map, so the points are in
//    map coordinates. The lidar position in map is queried from TF map<-lidar_link
//    at the SAME stamp as the cloud (never a stale/latest TF), and is used as the
//    real raycast origin. Cells accumulate across frames with no per-frame reset;
//    the sliding window follows the real sensor position; on TF loss or a
//    relocalization jump the map resets (policy). Ground points (deterministic
//    RANSAC / z-percentile) are inserted as free rays. Outputs are published in
//    the map frame.
//
// Read-only: this node never publishes TF or /cmd_vel.
class ScanGridMap final : public rclcpp::Node {
public:
  ScanGridMap() : Node("n01_scan_grid_map"), tf_buffer_(get_clock()) {
    mode_ = declare_parameter<std::string>("mode", "map_localized");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "lidar_link");
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.2);
    resolution_ = declare_parameter<double>("resolution", 0.05);
    half_x_ = declare_parameter<double>("half_x", 5.0);
    half_y_ = declare_parameter<double>("half_y", 5.0);
    min_z_ = declare_parameter<double>("min_z", -1.0);
    max_z_ = declare_parameter<double>("max_z", 2.0);
    p_hit_ = declare_parameter<double>("p_hit", 0.85);
    p_miss_ = declare_parameter<double>("p_miss", 0.30);
    p_min_ = declare_parameter<double>("p_min", 0.12);
    p_max_ = declare_parameter<double>("p_max", 0.98);
    p_occ_ = declare_parameter<double>("p_occ", 0.80);
    inflation_radius_ = declare_parameter<int>("inflation_radius_voxels", 2);
    max_cells_ = declare_parameter<int>("max_cells", 400000);
    max_range_ = declare_parameter<double>("max_range", 30.0);
    tf_loss_frames_before_reset_ = declare_parameter<int>("tf_loss_frames_before_reset", 1);
    relocalization_jump_m_ = declare_parameter<double>("relocalization_jump_m", 2.0);
    relocalization_max_speed_mps_ = declare_parameter<double>("relocalization_max_speed_mps", 5.0);
    // Ground params (Section 4): configurable, units/frame annotated in
    // config/scan_map.yaml. In sensor_only these are inert (stays a pure
    // snapshot; ground=every point a hit ray).
    ground_mode_ = declare_parameter<std::string>("ground_mode", "plane_ransac");
    ground_subsample_stride_ = declare_parameter<int>("ground_subsample_stride", 5);
    ground_max_points_ = declare_parameter<int>("ground_max_points", 20000);
    ground_iterations_ = declare_parameter<int>("ground_iterations", 24);
    ground_inlier_dist_m_ = declare_parameter<double>("ground_inlier_dist_m", 0.04);
    ground_normal_max_cos_ = declare_parameter<double>("ground_normal_max_cos", 0.8660254);
    ground_min_inlier_frac_ = declare_parameter<double>("ground_min_inlier_frac", 0.25);
    ground_fallback_ = declare_parameter<std::string>("ground_fallback", "robust_z_percentile");
    ground_fallback_percentile_q_ = declare_parameter<double>("ground_fallback_percentile_q", 2.0);
    ground_fallback_radius_m_ = declare_parameter<double>("ground_fallback_radius_m", 3.0);

    validate_params();

    const bool accum = (mode_ == "map_localized");
    map_ = std::make_unique<BoundedVoxelMap>(resolution_, half_x_, half_y_, min_z_, max_z_, p_hit_,
                                             p_miss_, p_min_, p_max_, p_occ_, inflation_radius_,
                                             static_cast<std::size_t>(max_cells_), max_range_,
                                             /*inflate_2d=*/accum);
    if (accum) map_->set_reject_out_of_window(true);

    ground_ = std::make_unique<GroundFilter>(GroundParams{
      ground_mode_,
      std::max(ground_subsample_stride_, 1),
      static_cast<std::size_t>(std::max(ground_max_points_, 0)),
      std::max(ground_iterations_, 1),
      1337u,
      ground_inlier_dist_m_,
      ground_normal_max_cos_,
      ground_min_inlier_frac_,
      ground_fallback_,
      ground_fallback_percentile_q_,
      ground_fallback_radius_m_,
    });
    policy_ = std::make_unique<MapLocalizedPolicy>();
    policy_->loss_threshold = std::max(tf_loss_frames_before_reset_, 1);
    policy_->jump_m = relocalization_jump_m_;
    policy_->max_speed_mps = relocalization_max_speed_mps_;

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, this, false);

    occupied_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/grid_map/occupancy", 10);
    inflated_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/grid_map/occupancy_inflate", 10);

    // One subscription for both modes. The TF lookup inside on_cloud is
    // stamp-matched and no-timeout: perception has already delayed this cloud
    // until 甲方's map<-lidar_link TF at its stamp existed, so the lookup
    // succeeds in steady state; a rare miss only skips one frame (the S4
    // policy handles sustained TF loss).
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/local_pcd", rclcpp::SensorDataQoS(),
      std::bind(&ScanGridMap::on_cloud, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
        "S4 map node: mode=%s input=/local_pcd lidar_frame=%s out_frame=%s res=%.3f z=[%.1f,%.1f] "
        "ground=%s tf_loss_reset=%d jump=%.2fm | read-only, never publishes TF or /cmd_vel",
        mode_.c_str(), lidar_frame_.c_str(), map_frame_.c_str(), resolution_, min_z_, max_z_,
        ground_mode_.c_str(), policy_->loss_threshold, policy_->jump_m);
  }

private:
  void validate_params() const {
    if (mode_ != "sensor_only" && mode_ != "map_localized")
      throw std::invalid_argument("mode must be 'sensor_only' or 'map_localized'");
    if (resolution_ <= 0.0) throw std::invalid_argument("resolution must be > 0");
    if (half_x_ <= 0.0 || half_y_ <= 0.0) throw std::invalid_argument("half_x/half_y must be > 0");
    if (min_z_ >= max_z_) throw std::invalid_argument("min_z must be < max_z");
    if (inflation_radius_ < 0) throw std::invalid_argument("inflation_radius_voxels must be >= 0");
    if (max_cells_ <= 0) throw std::invalid_argument("max_cells must be > 0");
    if (max_range_ <= 0.0) throw std::invalid_argument("max_range must be > 0");
    if (!(p_hit_ > p_miss_)) throw std::invalid_argument("p_hit must be > p_miss");
    if (!(p_occ_ > 0.5 && p_occ_ < 1.0)) throw std::invalid_argument("p_occ must be in (0.5, 1)");
    if (!(p_hit_ > p_occ_)) throw std::invalid_argument("p_hit must be > p_occ");
    if (!(p_min_ > 0.0 && p_min_ < p_hit_)) throw std::invalid_argument("p_min must be in (0, p_hit)");
    if (!(p_max_ < 1.0 && p_max_ > p_hit_)) throw std::invalid_argument("p_max must be in (p_hit, 1)");
    if (tf_loss_frames_before_reset_ < 0)
      throw std::invalid_argument("tf_loss_frames_before_reset must be >= 0");
    if (relocalization_jump_m_ <= 0.0) throw std::invalid_argument("relocalization_jump_m must be > 0");
    if (relocalization_max_speed_mps_ <= 0.0)
      throw std::invalid_argument("relocalization_max_speed_mps must be > 0");
    if (ground_mode_ != "none" && ground_mode_ != "plane_ransac")
      throw std::invalid_argument("ground_mode must be 'none' or 'plane_ransac'");
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {
    const rclcpp::Time stamp(msg->header.stamp);
    const bool accum = (mode_ == "map_localized");

    // sensor_only: pure single-frame snapshot in the input cloud's own frame;
    // raycast from that frame's origin (no TF, no accumulation, S3 semantics).
    if (!accum) {
      out_frame_ = msg->header.frame_id;
      map_->reset();
      const auto pts = collect_points(*msg);
      for (const auto &p : pts) map_->add_ray(0.0, 0.0, 0.0, p.x, p.y, p.z);
      map_->flush();
      const auto occ = map_->occupied();
      const auto infl = map_->inflated();
      publish(occ, occupied_pub_, stamp);
      publish(infl, inflated_pub_, stamp);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "S3 snapshot: cells=%zu occ=%zu infl=%zu stamp=%u.%u", map_->size(), occ.size(),
        infl.size(), msg->header.stamp.sec, msg->header.stamp.nanosec);
      return;
    }

    // map_localized: points must already be in map frame.
    if (msg->header.frame_id != map_frame_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "Refusing frame '%s'; expected map frame '%s'. No fake transform.",
        msg->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    out_frame_ = map_frame_;

    // Lidar position in map at the cloud's stamp (never latest/stale TF).
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(map_frame_, lidar_frame_, rclcpp::Time(msg->header.stamp));
    } catch (const tf2::TransformException &ex) {
      (void)policy_->decide(false, 0.0, 0.0, stamp);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "No TF %s <- %s at cloud stamp %u.%u; frame skipped (no wrong update): %s",
        map_frame_.c_str(), lidar_frame_.c_str(), msg->header.stamp.sec, msg->header.stamp.nanosec, ex.what());
      return;
    }
    const double ox = tf.transform.translation.x;
    const double oy = tf.transform.translation.y;
    const double oz = tf.transform.translation.z;

    const MapLocalizedPolicy::Action action = policy_->decide(true, ox, oy, stamp);

    map_->begin_frame();
    if (action == MapLocalizedPolicy::Action::ResetThenUpdate) map_->reset();
    map_->set_window_center(ox, oy);

    std::vector<Point3> pts = collect_points(*msg);
    if (pts.empty()) {
      map_->flush();  // clears the per-frame occlusion set; nothing else changes
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "Empty finite-point cloud; map kept frozen.");
      return;
    }

    const GroundResult gr = ground_->classify(pts, {ox, oy, oz});
    if (!gr.valid) {
      map_->flush();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Ground classification failed (method=%s); frame skipped, no wrong update.",
        gr.method.c_str());
      return;
    }

    for (std::size_t i = 0; i < pts.size(); ++i) {
      map_->add_ray(ox, oy, oz, pts[i].x, pts[i].y, pts[i].z, /*free=*/gr.is_ground[i]);
    }
    map_->flush();

    const auto occ = map_->occupied();
    const auto infl = map_->inflated();
    publish(occ, occupied_pub_, stamp);
    publish(infl, inflated_pub_, stamp);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "S4 update[%s]: cells=%zu occ=%zu infl=%zu window=(%.2f,%.2f) ground_z=%.3f(method=%s) stamp=%u.%u",
      action == MapLocalizedPolicy::Action::ResetThenUpdate ? "RESET" : "acc",
      map_->size(), occ.size(), infl.size(), map_->window_center_x(), map_->window_center_y(),
      gr.ground_z, gr.method.c_str(), msg->header.stamp.sec, msg->header.stamp.nanosec);
  }

  // Collect finite x/y/z points from a PointCloud2 (map or lidar frame).
  static std::vector<Point3> collect_points(const sensor_msgs::msg::PointCloud2 &msg) {
    std::vector<Point3> pts;
    sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      if (!std::isfinite(*it_x) || !std::isfinite(*it_y) || !std::isfinite(*it_z)) continue;
      pts.push_back({static_cast<double>(*it_x), static_cast<double>(*it_y),
                     static_cast<double>(*it_z)});
    }
    return pts;
  }

  void publish(const std::vector<Voxel> &voxels,
               const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub,
               const rclcpp::Time &stamp) {
    sensor_msgs::msg::PointCloud2 out;
    out.header.stamp = stamp;
    out.header.frame_id = out_frame_;
    out.height = 1;
    out.width = static_cast<uint32_t>(voxels.size());
    out.fields.resize(3);
    const char *names[] = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i) {
      out.fields[i].name = names[i];
      out.fields[i].offset = i * sizeof(float);
      out.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
      out.fields[i].count = 1;
    }
    out.is_bigendian = false;
    out.point_step = 3 * sizeof(float);
    out.row_step = out.point_step * out.width;
    out.is_dense = true;
    out.data.resize(out.row_step);
    const double cx = map_->window_center_x(), cy = map_->window_center_y();
    for (std::size_t i = 0; i < voxels.size(); ++i) {
      const float values[] = {
        static_cast<float>(cx + (voxels[i].x + 0.5) * resolution_),
        static_cast<float>(cy + (voxels[i].y + 0.5) * resolution_),
        static_cast<float>(min_z_ + (voxels[i].z + 0.5) * resolution_)};
      std::memcpy(out.data.data() + i * out.point_step, values, out.point_step);
    }
    pub->publish(out);
  }

  std::string mode_, map_frame_, lidar_frame_, ground_mode_, ground_fallback_;
  std::string out_frame_ = "map";  // per-frame publish frame (set on every on_cloud)
  double resolution_{}, half_x_{}, half_y_{}, min_z_{}, max_z_{};
  double p_hit_{}, p_miss_{}, p_min_{}, p_max_{}, p_occ_{};
  int inflation_radius_{}, max_cells_{};
  double max_range_{}, tf_timeout_sec_{};
  int tf_loss_frames_before_reset_{};
  double relocalization_jump_m_{}, relocalization_max_speed_mps_{};
  int ground_subsample_stride_{}, ground_max_points_{}, ground_iterations_{};
  double ground_inlier_dist_m_{}, ground_normal_max_cos_{}, ground_min_inlier_frac_{};
  double ground_fallback_percentile_q_{}, ground_fallback_radius_m_{};

  std::unique_ptr<BoundedVoxelMap> map_;
  std::unique_ptr<GroundFilter> ground_;
  std::unique_ptr<MapLocalizedPolicy> policy_;
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_pub_, inflated_pub_;
};

}  // namespace n01_scan

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<n01_scan::ScanGridMap>());
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("n01_scan_grid_map"), "Startup failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}