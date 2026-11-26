#include "plan_env/grid_fusion.h"
#include "plan_env/grid_map.h"
#include "plan_env/environment_inflation.h"
#include <unordered_set>
#include <nlohmann/json.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <chrono>
#include <limits>
#include <string>
#include <stdexcept>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace
{
template <typename T>
void load_parameter(rclcpp::Node *node, const std::string &name, T &value, const T &default_value)
{
  if (!node->has_parameter(name))
    node->declare_parameter<T>(name, default_value);
  node->get_parameter(name, value);
}
}  // namespace

void GridMap::initMap(rclcpp::Node *node)
{
  node_ = node;

  load_parameter(node_, "grid_map.use_urdf_footprint", urdf_footprint_enabled_, false);
  load_parameter(node_, "grid_map.footprint_max_age_s", footprint_max_age_s_, 3.0);
  if(!std::isfinite(footprint_max_age_s_) || footprint_max_age_s_<.1 || footprint_max_age_s_>3.)
    throw std::invalid_argument("grid_map.footprint_max_age_s must be within [0.1,3.0]");
  if(urdf_footprint_enabled_) {
    footprint_sub_=node_->create_subscription<geometry_msgs::msg::PolygonStamped>(
      "/n01/robot_footprint",rclcpp::QoS(1).reliable().transient_local(),
      [this](geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg) {
        footprint_valid_=false;footprint_cached_offsets_.clear();
        const auto update=[&]() {
        if(msg->header.frame_id!="base_link" || msg->polygon.points.size()<3 || msg->polygon.points.size()>256)return;
        std::vector<Eigen::Vector2d> polygon;
        for(const auto &v:msg->polygon.points) {
          if(!std::isfinite(v.x)||!std::isfinite(v.y)||std::abs(v.x)>3.||std::abs(v.y)>3.)return;
          polygon.emplace_back(v.x,v.y);
        }
        double area=0.;
        for(size_t i=0;i<polygon.size();++i){
          auto a=polygon[i],b=polygon[(i+1)%polygon.size()],c=polygon[(i+2)%polygon.size()];
          if((b-a).squaredNorm()<1e-12 || (b.x()-a.x())*(c.y()-b.y())-(b.y()-a.y())*(c.x()-b.x()) < -1e-8)return;
          area+=a.x()*b.y()-a.y()*b.x();
        }
        if(area<.01)return;
        footprint_polygon_=std::move(polygon);
        footprint_stamp_ns_=rclcpp::Time(msg->header.stamp).nanoseconds();footprint_valid_=true;
        };
        update();
        publishStaticOverlayStatus(static_source_valid_ && static_window_intersects_source_bounds_,
          !static_source_valid_ ? "waiting_for_static_cloud" :
          (static_window_intersects_source_bounds_ ? "ready" : "active_window_outside_static_map_bounds"));
      });
  }
  /* get parameter */
  double x_size, y_size, z_size;
  load_parameter(node_, "grid_map.resolution", mp_.resolution_, -1.0);
  load_parameter(node_, "grid_map.sliding_map_size_x", x_size, -1.0);
  load_parameter(node_, "grid_map.sliding_map_size_y", y_size, -1.0);
  load_parameter(node_, "grid_map.sliding_map_size_z", z_size, -1.0);
  load_parameter(node_, "grid_map.local_update_range_x", mp_.local_update_range_(0), x_size / 2.0);
  load_parameter(node_, "grid_map.local_update_range_y", mp_.local_update_range_(1), y_size / 2.0);
  load_parameter(node_, "grid_map.local_update_range_z", mp_.local_update_range_(2), z_size / 2.0);
  
  load_parameter(node_, "grid_map.obstacles_inflation_z_up", mp_.obstacles_inflation_z_up, -1.0);
  load_parameter(node_, "grid_map.obstacles_inflation_z_down", mp_.obstacles_inflation_z_down, -1.0);
  load_parameter(node_, "grid_map.double_cylinder_radius", mp_.double_cylinder_radius_, -1.0);
  load_parameter(node_, "grid_map.environment_margin", mp_.environment_margin_, 0.0);
  if(!std::isfinite(mp_.environment_margin_) || mp_.environment_margin_<0 || mp_.environment_margin_>.2)
    throw std::invalid_argument("environment margin outside [0,0.2]m");
  load_parameter(node_, "grid_map.fixed_body_heading", mp_.fixed_body_heading_, false);
  load_parameter(node_, "grid_map.double_cylinder_offset", mp_.double_cylinder_offset_, 0.0);
  load_parameter(node_, "grid_map.footprint_centers_x", mp_.footprint_centers_x_, std::vector<double>{});
  if (mp_.footprint_centers_x_.empty())
    mp_.footprint_centers_x_ = {-mp_.double_cylinder_offset_, mp_.double_cylinder_offset_};
  load_parameter(node_, "grid_map.map_sliding_en", mp_.map_sliding_en_, true);
  load_parameter(node_, "grid_map.map_sliding_thresh", mp_.map_sliding_thresh_, mp_.resolution_);

  load_parameter(node_, "grid_map.fx", mp_.fx_, -1.0);
  load_parameter(node_, "grid_map.fy", mp_.fy_, -1.0);
  load_parameter(node_, "grid_map.cx", mp_.cx_, -1.0);
  load_parameter(node_, "grid_map.cy", mp_.cy_, -1.0);

  load_parameter(node_, "grid_map.depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);
  load_parameter(node_, "grid_map.depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);
  load_parameter(node_, "grid_map.depth_filter_margin", mp_.depth_filter_margin_, -1);
  load_parameter(node_, "grid_map.k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  load_parameter(node_, "grid_map.skip_pixel", mp_.skip_pixel_, -1);

  load_parameter(node_, "grid_map.p_hit", mp_.p_hit_, -1.0);
  load_parameter(node_, "grid_map.p_miss", mp_.p_miss_, -1.0);
  load_parameter(node_, "grid_map.p_min", mp_.p_min_, -1.0);
  load_parameter(node_, "grid_map.p_max", mp_.p_max_, -1.0);
  load_parameter(node_, "grid_map.p_occ", mp_.p_occ_, -1.0);
  load_parameter(node_, "grid_map.max_ray_length", mp_.max_ray_length_, -0.1);

  load_parameter(node_, "grid_map.vis_height", mp_.vis_height_, 0.3);
  load_parameter(node_, "grid_map.show_occ_time", mp_.show_occ_time_, false);

  load_parameter(node_, "grid_map.frame_id", mp_.frame_id_, string("world"));
  load_parameter(node_, "grid_map.sliding_map_frame_id", mp_.sliding_map_frame_id_, string("sliding_map"));
  load_parameter(node_, "grid_map.ground_height", mp_.ground_height_, 0.0);
  load_parameter(node_, "grid_map.ground_clearance", mp_.ground_clearance_, 0.03);
  load_parameter(node_, "grid_map.ground_slope_x", mp_.ground_slope_x_, 0.0);
  load_parameter(node_, "grid_map.ground_slope_y", mp_.ground_slope_y_, 0.0);
  // Static PCD is optional, explicit and kept independent from live rays.
  // Default OFF so legacy SCAN planners preserve original behavior.
  load_parameter(node_, "grid_map.static_cloud_enabled", static_cloud_enabled_, false);
  load_parameter(node_, "grid_map.reference_updates_enabled", reference_updates_enabled_, false);
  load_parameter(node_, "grid_map.require_observed_local_space", require_observed_space_, false);
  load_parameter(node_, "grid_map.observation_ttl_s", observation_ttl_s_, 3.);
  if(!std::isfinite(observation_ttl_s_) || observation_ttl_s_<1. || observation_ttl_s_>30.)throw std::runtime_error("invalid observation TTL");
  load_parameter(node_, "grid_map.static_observation_refresh", static_observation_refresh_, false);
  load_parameter(node_, "grid_map.fused_2d_enabled", fused_2d_enabled_, false);
  load_parameter(node_, "grid_map.pcd_bounds_free", pcd_bounds_free_enabled_, false);
  load_parameter(node_, "grid_map.static_ego_filter_enabled", static_ego_filter_enabled_, true);
  load_parameter(node_, "grid_map.static_ego_filter_radius", static_ego_filter_radius_, 0.30);
  load_parameter(node_, "grid_map.static_ego_filter_margin", static_ego_filter_margin_, 0.06);
  load_parameter(node_, "grid_map.static_ego_filter_z_down", static_ego_filter_z_down_, 0.80);
  load_parameter(node_, "grid_map.static_ego_filter_z_up", static_ego_filter_z_up_, 0.90);
  load_parameter(node_, "grid_map.live_ego_filter_enabled", live_ego_filter_enabled_, true);
  load_parameter(node_, "grid_map.live_ego_filter_margin", live_ego_filter_margin_, 0.06);
  load_parameter(node_, "grid_map.raycast_dedup_resolution", raycast_dedup_resolution_, 0.10);
  if(!std::isfinite(raycast_dedup_resolution_)||raycast_dedup_resolution_<0.||raycast_dedup_resolution_>0.2)
    throw std::runtime_error("invalid raycast dedup resolution");
  load_parameter(node_, "grid_map.ego_history_enabled", ego_history_enabled_, true);
  load_parameter(node_, "grid_map.ego_history_step_m", ego_history_step_m_, 0.25);
  if(!std::isfinite(ego_history_step_m_)||ego_history_step_m_<0.05||ego_history_step_m_>1.0)
    throw std::runtime_error("invalid ego history step");
  if(!std::isfinite(live_ego_filter_margin_)||live_ego_filter_margin_<0.||live_ego_filter_margin_>.1)
    throw std::runtime_error("invalid live ego filter margin");
  if(!std::isfinite(static_ego_filter_radius_)||static_ego_filter_radius_<.1||static_ego_filter_radius_>.6)
    throw std::runtime_error("grid_map.static_ego_filter_radius must be in [0.1, 0.6]");
  if(!std::isfinite(static_ego_filter_margin_)||static_ego_filter_margin_<0.||static_ego_filter_margin_>.1)
    throw std::runtime_error("invalid static ego filter margin");
  if(!std::isfinite(static_ego_filter_z_down_)||!std::isfinite(static_ego_filter_z_up_)||
     static_ego_filter_z_down_<=0.||static_ego_filter_z_up_<=0.)
    throw std::runtime_error("invalid static ego filter z range");
  load_parameter(node_, "grid_map.static_prior_as_cost", static_prior_as_cost_, false);
  load_parameter(node_, "grid_map.unknown_as_obstacle", unknown_as_obstacle_, true);
  load_parameter(node_, "grid_map.extend_reference_to_goal", extend_reference_to_goal_, false);
  load_parameter(node_, "grid_map.static_cloud_topic", static_cloud_topic_,
                 std::string("/grid_map/static_occupancy"));
  load_parameter(node_, "grid_map.static_cloud_max_points", static_cloud_max_points_, 1000000);
  if (static_cloud_enabled_ &&
      (static_cloud_max_points_ < 1000 || static_cloud_max_points_ > 2000000 ||
       static_cloud_topic_.empty())) {
    throw std::invalid_argument("[GridMap] invalid static PCD overlay parameters");
  }

  load_parameter(node_, "grid_map.sensor_type", mp_.sensor_type_, string("lidar"));
  load_parameter(node_, "grid_map.cloud_is_world", mp_.cloud_is_world_, true);
  load_parameter(node_, "grid_map.need_extrinsic", mp_.need_extrinsic_, true);

  mp_.lidar_extrinsic_ <<
      1.0, 0.0, 0.0, -0.01100,
      0.0, 1.0, 0.0, -0.02329,
      0.0, 0.0, 1.0,  0.04412,
      0.0, 0.0, 0.0,  1.00000;

  mp_.depth_extrinsic_ <<
      0.0,  0.707107, 0.707107, -0.15170,
     -1.0,  0.000000, 0.000000,  0.00000,
      0.0, -0.707107, 0.707107,  0.07510,
      0.0,  0.000000, 0.000000,  1.00000;
      
  if (mp_.sensor_type_ != "lidar" && mp_.sensor_type_ != "depth")
  {
    RCLCPP_ERROR(node_->get_logger(), "[GridMap] invalid grid_map.sensor_type: %s; falling back to lidar",
                 mp_.sensor_type_.c_str());
    mp_.sensor_type_ = "lidar";
  }

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;
  mp_.map_sliding_thresh_vox_ = std::max(1, static_cast<int>(std::ceil(mp_.map_sliding_thresh_ * mp_.resolution_inv_)));

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;
  posToIndex(mp_.map_origin_, mp_.map_bound_min_idx_);
  mp_.map_bound_max_idx_ = mp_.map_bound_min_idx_ + mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
  mp_.map_origin_idx_ = mp_.map_bound_min_idx_ + mp_.map_voxel_num_ / 2;
  updateMapBoundaryFromIndex();

  // initialize data buffers

  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
  md_.occupancy_buffer_inflate_cnt_ = vector<int>(buffer_size, 0);
  static_occupancy_buffer_ = vector<char>(buffer_size, 0);
  static_prior_mask_ = vector<char>(buffer_size, 0);
  static_inflate_buffer_ = vector<char>(buffer_size, 0);
  rebuildInflationOffsets();

  md_.count_hit_and_miss_ = vector<std::uint32_t>(buffer_size, 0);
  md_.count_hit_ = vector<std::uint32_t>(buffer_size, 0);
  md_.flag_rayend_ = vector<std::uint32_t>(buffer_size, std::numeric_limits<std::uint32_t>::max());
  md_.flag_traverse_ = vector<std::uint32_t>(buffer_size, std::numeric_limits<std::uint32_t>::max());

  md_.raycast_num_ = 0;

  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_cnt = 0;

  /* init callback */
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*node_);

  if (mp_.sensor_type_ == "depth")
  {
    depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>();
    depth_pose_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>();
    depth_sub_->subscribe(node_, "depth", rmw_qos_profile_sensor_data);
    depth_pose_sub_->subscribe(node_, "sensor_pose", rmw_qos_profile_sensor_data);

    sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
        SyncPolicyImagePose(100), *depth_sub_, *depth_pose_sub_));
    sync_image_pose_->registerCallback(
        std::bind(&GridMap::depthPoseCallback, this, std::placeholders::_1, std::placeholders::_2));
  }
  else if (mp_.sensor_type_ == "lidar")
  {
    // The live bridge emits /cloud and /sensor_pose with the exact same scan
    // stamp. Never pair a cloud with a pose left over from a previous scan.
    cloud_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>();
    lidar_pose_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>();
    cloud_sub_->subscribe(node_, "cloud", rmw_qos_profile_sensor_data);
    lidar_pose_sub_->subscribe(node_, "sensor_pose", rmw_qos_profile_sensor_data);
    sync_cloud_pose_.reset(new message_filters::Synchronizer<SyncPolicyCloudPose>(
        SyncPolicyCloudPose(100), *cloud_sub_, *lidar_pose_sub_));
    sync_cloud_pose_->registerCallback(
        std::bind(&GridMap::cloudPoseCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

  if (static_cloud_enabled_) {
    static_ready_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
        "/grid_map/static_overlay_ready",
        rclcpp::QoS(1).reliable().transient_local());
    static_status_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "/grid_map/static_overlay_status",
        rclcpp::QoS(1).reliable().transient_local());
    publishStaticOverlayStatus(false, "waiting_for_static_cloud");
    // Static PCD is immutable and published once with transient-local QoS.
    // Request the retained sample even when SCAN starts after the map provider.
    static_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        static_cloud_topic_, rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&GridMap::staticCloudCallback, this, std::placeholders::_1));
    RCLCPP_WARN(node_->get_logger(),
                "[GridMap] static PCD occupancy overlay ENABLED: %s; "
                "original PCD map frame required, no floor/whole-body validation",
                static_cloud_topic_.c_str());
  }

  sliding_map_frame_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "body_pose", rclcpp::SensorDataQoS(),
      std::bind(&GridMap::slidingMapFrameCallback, this, std::placeholders::_1));

  occ_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                        std::bind(&GridMap::updateOccupancyCallback, this));
  // Visualization shares the executor with collision/map updates and the FSM.
  // Keep the control/map update rates intact; only decimate display exports.
  double vis_period_s = 0.5;
  load_parameter(node_, "grid_map.visualization_period_s", vis_period_s, 0.5);
  if (!std::isfinite(vis_period_s) || vis_period_s < 0.05 || vis_period_s > 2.0)
    throw std::runtime_error("grid_map.visualization_period_s must be in [0.05, 2.0]");
  vis_timer_ = node_->create_wall_timer(std::chrono::duration<double>(vis_period_s),
                                        std::bind(&GridMap::visCallback, this));
  reference_grid_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/grid_map/reference_grid", rclcpp::QoS(1).reliable().transient_local());
  live_grid_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/grid_map/live_grid", rclcpp::QoS(1).reliable().transient_local());
  global_grid_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/grid_map/global_grid", rclcpp::QoS(1).reliable().transient_local());

  ground_reference_sub_=node_->create_subscription<std_msgs::msg::String>("/n01/ground_reference/status",10,
    [this](std_msgs::msg::String::ConstSharedPtr msg){
      try {
        auto data=nlohmann::json::parse(msg->data);const auto& p=data.at("plane");
        double a=p.at("a"),b=p.at("b"),c=p.at("c"),residual=p.at("residual");
        if(data.at("state")!="ground_projection_ready" || !std::isfinite(a+b+c+residual) || residual<0 || residual>.04 || std::hypot(a,b)>std::tan(2.*M_PI/180.))return;
        if (fused_2d_enabled_ && !reference_updates_enabled_) return;
        const double delta=std::abs(c-mp_.ground_height_)+std::abs(a-mp_.ground_slope_x_)+std::abs(b-mp_.ground_slope_y_);
        mp_.ground_slope_x_=a;mp_.ground_slope_y_=b;mp_.ground_height_=c;
        if(delta>1e-5){
          static_refresh_pending_=true;static_restore_urgent_=true;
          // Ground plane changed: the persistent 2-D layer must not keep the
          // previous floor/ceiling classification.
          global_grid_.clear();global_tile_version_.clear();
          global_w_=global_h_=0;global_version_++;
        }
      }catch(const nlohmann::json::exception&){return;}
    });
  predicted_sub_=node_->create_subscription<sensor_msgs::msg::PointCloud2>("/n01/live_navigation/predicted_obstacles",rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){
      int64_t stamp=rclcpp::Time(msg->header.stamp).nanoseconds();
      double age=(node_->now().nanoseconds()-stamp)*1e-9;
      if(msg->header.frame_id!="map" || stamp<=prediction_stamp_ns_ || age<-.1 || age>2.0 || msg->width>20000)return;
      if(msg->height!=1 || msg->is_bigendian || msg->point_step<12 || msg->row_step<msg->width*msg->point_step || msg->data.size()<msg->row_step)return;
      for(const char* name:{"x","y","z"}) {
        bool valid=false;for(const auto& field:msg->fields)if(field.name==name && field.datatype==sensor_msgs::msg::PointField::FLOAT32 && field.count==1 && field.offset+4<=msg->point_step)valid=true;
        if(!valid)return;
      }
      pcl::PointCloud<pcl::PointXYZ> cloud;
      try{pcl::fromROSMsg(*msg,cloud);}catch(const std::exception& e){RCLCPP_WARN(node_->get_logger(),"Rejected forecast cloud: %s",e.what());return;}
      std::vector<Eigen::Vector3d> points;
      for(const auto& p:cloud)if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z))points.emplace_back(p.x,p.y,p.z);
      predicted_points_=std::move(points);prediction_stamp_ns_=stamp;
    });
  fused_view_pub_=node_->create_publisher<sensor_msgs::msg::PointCloud2>("/grid_map/fused_reference",rclcpp::QoS(1).reliable().transient_local());
  risk_view_pub_=node_->create_publisher<sensor_msgs::msg::PointCloud2>("/grid_map/unconfirmed_risk",rclcpp::SensorDataQoS());
  observed_free_pub_=node_->create_publisher<sensor_msgs::msg::PointCloud2>("/grid_map/observed_free",rclcpp::SensorDataQoS());
  map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy", rclcpp::SensorDataQoS());
  environment_inf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/environment_inflate", rclcpp::SensorDataQoS());
  map_inf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy_inflate", rclcpp::SensorDataQoS());
  sliding_map_bbox_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("grid_map/sliding_map_bbox", 10);

  unknown_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/unknown", rclcpp::SensorDataQoS());
  depth_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/depth_cloud", rclcpp::SensorDataQoS());
  extrinsic_pose_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("grid_map/sensor_pose_extrinsic", 10);

  md_.occ_need_update_ = false;
  md_.use_cloud_update_ = false;
  md_.has_first_depth_ = false;
  md_.has_ray_pose_ = false;
  md_.has_cloud_ = false;
  md_.image_cnt_ = 0;
  md_.ray_pos_.setZero();
  md_.sliding_map_frame_pos_.setZero();
  md_.ray_q_ = Eigen::Quaterniond::Identity();

  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_fuse_time_ = 0.0;
  md_.local_bound_min_ = mp_.map_bound_min_idx_;
  md_.local_bound_max_ = mp_.map_bound_max_idx_;

  // rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  // rand_noise2_ = normal_distribution<double>(0, 0.2);
  // random_device rd;
  // eng_ = default_random_engine(rd());
}

void GridMap::updateMapBoundaryFromIndex()
{
  mp_.map_bound_min_idx_ = mp_.map_origin_idx_ - mp_.map_voxel_num_ / 2;
  mp_.map_bound_max_idx_ = mp_.map_bound_min_idx_ + mp_.map_voxel_num_ - Eigen::Vector3i::Ones();

  mp_.map_min_boundary_ = mp_.map_bound_min_idx_.cast<double>() * mp_.resolution_;
  mp_.map_max_boundary_ = (mp_.map_bound_max_idx_.cast<double>() + Eigen::Vector3d::Ones()) * mp_.resolution_;
  mp_.map_origin_ = mp_.map_min_boundary_;
}

void GridMap::rebuildInflationOffsets()
{
  const double double_radius = urdf_footprint_enabled_ ? 0.0 : std::max(0.0, getFootprintRadius());
  const int inf_step_xy = ceil(double_radius / mp_.resolution_);
  static_inflate_z_up_steps_ =
      std::max(0, static_cast<int>(ceil(mp_.obstacles_inflation_z_up / mp_.resolution_)));
  static_inflate_z_down_steps_ =
      std::max(0, static_cast<int>(ceil(mp_.obstacles_inflation_z_down / mp_.resolution_)));

  md_.inflate_offsets_.clear();
  static_inflate_xy_offsets_.clear();
  for (int x = -inf_step_xy; x <= inf_step_xy; ++x)
    for (int y = -inf_step_xy; y <= inf_step_xy; ++y)
    {
      Eigen::Vector2d offset_xy(x * mp_.resolution_, y * mp_.resolution_);
      if (offset_xy.norm() > double_radius + 1e-12)
        continue;

      static_inflate_xy_offsets_.push_back({x, y});
      for (int z = -static_inflate_z_down_steps_; z <= static_inflate_z_up_steps_; ++z)
        md_.inflate_offsets_.push_back(Eigen::Vector3i(x, y, z));
    }
}

void GridMap::resetAllMapData()
{
  observation_times_.clear();observed_columns_.clear();latest_scan_stamp_ns_=processed_scan_stamp_ns_=0;
  prior_observation_since_.clear();static_window_prior_cells_.clear();last_prior_scan_stamp_ns_=0;
  world_changes_.clear(); // Whole-map/coordinate discontinuities revoke historical evidence.
  predicted_points_.clear();prediction_stamp_ns_=0;
  static_free_evidence_.clear();
  static_refresh_pending_=true;static_restore_urgent_=true;
  std::fill(md_.occupancy_buffer_.begin(), md_.occupancy_buffer_.end(), mp_.clamp_min_log_ - mp_.unknown_flag_);
  std::fill(md_.occupancy_buffer_inflate_.begin(), md_.occupancy_buffer_inflate_.end(), 0);
  std::fill(md_.occupancy_buffer_inflate_cnt_.begin(), md_.occupancy_buffer_inflate_cnt_.end(), 0);
  std::fill(md_.count_hit_and_miss_.begin(), md_.count_hit_and_miss_.end(), 0);
  std::fill(md_.count_hit_.begin(), md_.count_hit_.end(), 0);
  std::fill(md_.flag_rayend_.begin(), md_.flag_rayend_.end(), -1);
  std::fill(md_.flag_traverse_.begin(), md_.flag_traverse_.end(), -1);
  std::queue<Eigen::Vector3i> empty;
  std::swap(md_.cache_voxel_, empty);
}

void GridMap::hashIdToGlobalIndex(int addr, Eigen::Vector3i& id_g) const
{
  Eigen::Vector3i id_l;
  id_l(0) = addr / (mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2));
  id_l(1) = (addr - id_l(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2)) / mp_.map_voxel_num_(2);
  id_l(2) = addr - id_l(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) -
            id_l(1) * mp_.map_voxel_num_(2);

  for (int i = 0; i < 3; ++i)
  {
    const int min_l = getLocalIndex(mp_.map_bound_min_idx_(i), i);
    int dist = id_l(i) - min_l;
    if (dist < 0)
      dist += mp_.map_voxel_num_(i);
    id_g(i) = mp_.map_bound_min_idx_(i) + dist;
  }
}

void GridMap::updateInflationLayer(const Eigen::Vector3i& id, int delta,
                                   const vector<Eigen::Vector3i>& offsets,
                                   std::vector<int>& cnt_buffer,
                                   std::vector<char>& flag_buffer,
                                   const std::vector<char>* ignore_mask)
{
  for (const auto& offset : offsets)
  {
    const Eigen::Vector3i inf_id = id + offset;
    if (!isInMap(inf_id))
      continue;

    const int addr = toAddress(inf_id);
    if (ignore_mask && (*ignore_mask)[addr])
      continue;

    cnt_buffer[addr] += delta;
    if (cnt_buffer[addr] < 0)
      cnt_buffer[addr] = 0;
    flag_buffer[addr] = cnt_buffer[addr] > 0 ? 1 : 0;
  }
}

void GridMap::updateInflation(const Eigen::Vector3i& id, int delta, const std::vector<char>* ignore_mask)
{
  updateInflationLayer(id, delta, md_.inflate_offsets_, md_.occupancy_buffer_inflate_cnt_,
                       md_.occupancy_buffer_inflate_, ignore_mask);
}

void GridMap::applyOccupancyUpdate(const Eigen::Vector3i& id, double new_log_odds)
{
  if (!isInMap(id))
    return;

  const int addr = toAddress(id);
  const bool was_occ = collisionRisk(md_.occupancy_buffer_[addr]);
  const bool now_occ = collisionRisk(new_log_odds);

  md_.occupancy_buffer_[addr] = new_log_odds;
  if (was_occ != now_occ){
    updateInflation(id, now_occ ? 1 : -1);
    if(now_occ)route_map_changed_=true;
  }
}

void GridMap::resetCellByAddress(int addr)
{
  observation_times_.erase(addr);prior_observation_since_.erase(addr);
  static_free_evidence_.reset(addr);
  static_refresh_pending_=true;static_restore_urgent_=true;
  Eigen::Vector3i id_g;
  hashIdToGlobalIndex(addr, id_g);
  if (collisionRisk(md_.occupancy_buffer_[addr]))
    updateInflation(id_g, -1);

  md_.occupancy_buffer_[addr] = mp_.clamp_min_log_ - mp_.unknown_flag_;
  md_.count_hit_[addr] = 0;
  md_.count_hit_and_miss_[addr] = 0;
  md_.flag_rayend_[addr] = -1;
  md_.flag_traverse_[addr] = -1;
}

void GridMap::resetCellByAddressForSliding(int addr, const std::vector<char>& clear_mask)
{
  Eigen::Vector3i id_g;
  hashIdToGlobalIndex(addr, id_g);
  if (collisionRisk(md_.occupancy_buffer_[addr]))
    updateInflation(id_g, -1, &clear_mask);
}

void GridMap::updateSlidingMap(const Eigen::Vector3d& center)
{
  if (!mp_.map_sliding_en_)
    return;

  Eigen::Vector3i new_origin_idx;
  posToIndex(center, new_origin_idx);
  const Eigen::Vector3i shift_num = new_origin_idx - mp_.map_origin_idx_;
  if (shift_num.cwiseAbs().maxCoeff() < mp_.map_sliding_thresh_vox_)
    return;

  if ((shift_num.cwiseAbs().array() >= mp_.map_voxel_num_.array()).any())
  {
    resetAllMapData();
    mp_.map_origin_idx_ = new_origin_idx;
    updateMapBoundaryFromIndex();
    md_.local_bound_min_ = mp_.map_bound_min_idx_;
    md_.local_bound_max_ = mp_.map_bound_max_idx_;
    rebuildStaticOverlay(); // restore static map when the entire rolling window resets
    return;
  }

  const int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);
  std::vector<char> clear_mask(buffer_size, 0);
  std::vector<int> clear_addrs;
  clear_addrs.reserve(buffer_size / 8);

  auto add_clear_addr = [&](const Eigen::Vector3i& id_l) {
    const int addr = toAddressLocal(id_l);
    if (!clear_mask[addr])
    {
      clear_mask[addr] = 1;
      clear_addrs.push_back(addr);
    }
  };

  for (int dim = 0; dim < 3; ++dim)
  {
    const int shift = shift_num(dim);
    if (shift == 0)
      continue;

    if (shift > 0)
    {
      for (int k = 0; k < shift; ++k)
      {
        const int clear_g = mp_.map_bound_min_idx_(dim) + k;
        const int clear_l = getLocalIndex(clear_g, dim);

        for (int a = 0; a < mp_.map_voxel_num_((dim + 1) % 3); ++a)
          for (int b = 0; b < mp_.map_voxel_num_((dim + 2) % 3); ++b)
          {
            Eigen::Vector3i id_l;
            id_l(dim) = clear_l;
            id_l((dim + 1) % 3) = a;
            id_l((dim + 2) % 3) = b;
            add_clear_addr(id_l);
          }
      }
    }
    else
    {
      for (int k = 0; k < -shift; ++k)
      {
        const int clear_g = mp_.map_bound_max_idx_(dim) - k;
        const int clear_l = getLocalIndex(clear_g, dim);

        for (int a = 0; a < mp_.map_voxel_num_((dim + 1) % 3); ++a)
          for (int b = 0; b < mp_.map_voxel_num_((dim + 2) % 3); ++b)
          {
            Eigen::Vector3i id_l;
            id_l(dim) = clear_l;
            id_l((dim + 1) % 3) = a;
            id_l((dim + 2) % 3) = b;
            add_clear_addr(id_l);
          }
      }
    }
  }

  for (int addr : clear_addrs)
    resetCellByAddressForSliding(addr, clear_mask);

  for (int addr : clear_addrs)
  {
    observation_times_.erase(addr);prior_observation_since_.erase(addr);
  static_free_evidence_.reset(addr);
    md_.occupancy_buffer_[addr] = mp_.clamp_min_log_ - mp_.unknown_flag_;
    md_.occupancy_buffer_inflate_cnt_[addr] = 0;
    md_.occupancy_buffer_inflate_[addr] = 0;
    md_.count_hit_[addr] = 0;
    md_.count_hit_and_miss_[addr] = 0;
    md_.flag_rayend_[addr] = -1;
    md_.flag_traverse_[addr] = -1;
  }

  mp_.map_origin_idx_ = new_origin_idx;
  updateMapBoundaryFromIndex();
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);
  rebuildStaticOverlay(); // retained PCD reprojected into new sliding frame
}

void GridMap::resetBuffer()
{
  resetAllMapData();
  md_.local_bound_min_ = mp_.map_bound_min_idx_;
  md_.local_bound_max_ = mp_.map_bound_max_idx_;
}

void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{
  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
    {
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        resetCellByAddress(toAddress(x, y, z));
      }
    }
}

int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  if (!isInMap(id))
    return INVALID_IDX;

  int idx_ctns = toAddress(id);

  if(md_.count_hit_and_miss_[idx_ctns]<std::numeric_limits<std::uint32_t>::max())md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1)
  {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1 && md_.count_hit_[idx_ctns]<std::numeric_limits<std::uint32_t>::max())
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void GridMap::projectDepthImage()
{
  md_.proj_ground_.clear();
  // md_.proj_points_.clear();
  md_.proj_points_cnt = 0;

  uint16_t *row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;

  double depth;

  Eigen::Matrix3d sensor_r = md_.ray_q_.toRotationMatrix();

  // cout << "rotate: " << md_.ray_q_.toRotationMatrix() << endl;
  // std::cout << "pos in proj: " << md_.ray_pos_ << std::endl;

  if (!md_.has_first_depth_)
  {
    md_.has_first_depth_ = true;
    return;
  }

  Eigen::Vector3d pt_cur, pt_world;
  const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

  for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
  {
    row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

    for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_; u += mp_.skip_pixel_)
    {
      const uint16_t raw_depth = *row_ptr;
      depth = raw_depth * inv_factor;
      row_ptr = row_ptr + mp_.skip_pixel_;

      // filter depth
      // depth += rand_noise_(eng_);
      // if (depth > 0.01) depth += rand_noise2_(eng_);

      if (raw_depth == 0)
      {
        depth = mp_.max_ray_length_ + 0.1;
      }
      else if (depth < mp_.depth_filter_mindist_)
      {
        continue;
      }
      else if (depth > mp_.depth_filter_maxdist_)
      {
        depth = mp_.max_ray_length_ + 0.1;
      }

      // project to world frame
      pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
      pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
      pt_cur(2) = depth;

      pt_world = sensor_r * pt_cur + md_.ray_pos_;
      // if (!isInMap(pt_world)) {
      //   pt_world = closetPointInMap(pt_world, md_.ray_pos_);
      // }

      md_.proj_points_[md_.proj_points_cnt++] = pt_world;
    }
  }
}

void GridMap::raycastProcess()
{
  // if (md_.proj_points_.size() == 0)
  if (md_.proj_points_cnt == 0)
    return;

  updateSlidingMap(md_.ray_pos_);

  md_.raycast_num_ += 1;

  int vox_idx;
  double length;

  // bounding box of updated region
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;
  std::unordered_set<uint64_t> projected_ray_ends;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt_w = md_.proj_points_[i];

    // set flag for projected point

    if (!isInMap(pt_w))
    {
      pt_w = closetPointInMap(pt_w, md_.ray_pos_);

      length = (pt_w - md_.ray_pos_).norm();
      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.ray_pos_) / length * mp_.max_ray_length_ + md_.ray_pos_;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);
    }
    else
    {
      length = (pt_w - md_.ray_pos_).norm();

      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.ray_pos_) / length * mp_.max_ray_length_ + md_.ray_pos_;
        vox_idx = setCacheOccupancy(pt_w, 0);
      }
      else
      {
        // Ground returns still provide clearing rays, but never occupied ends.
        const bool ground=md_.use_cloud_update_ && i<static_cast<int>(md_.proj_ground_.size())
            ? md_.proj_ground_[i] : pt_w.z()<=getObstacleGroundCutoff(pt_w.x(),pt_w.y());
        vox_idx = setCacheOccupancy(pt_w, ground ? 0 : 1);
      }
    }

    const bool coverage_ray=pt_w.z()<=md_.sliding_map_frame_pos_.z()+mp_.obstacles_inflation_z_down;
    const double scan_time=latest_scan_stamp_ns_>0 ? latest_scan_stamp_ns_*1e-9 : node_->now().seconds();
    if(coverage_ray && projected_ray_ends.insert(coverageKey(pt_w.x(),pt_w.y())).second){
      // Projected visibility must not stop at a 3D traversal flag set by a
      // higher ray which never contributed coverage at collision height.
      RayCaster projected;Eigen::Vector3d cell;
      projected.setInput(Eigen::Vector3d(pt_w.x()/.1,pt_w.y()/.1,0.),
                         Eigen::Vector3d(md_.ray_pos_.x()/.1,md_.ray_pos_.y()/.1,0.));
      observed_columns_[coverageKey(pt_w.x(),pt_w.y())]=scan_time;
      while(projected.step(cell))observed_columns_[coverageKey((cell.x()+.5)*.1,(cell.y()+.5)*.1)]=scan_time;
    }
    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    // raycasting between ray origin and point

    if (vox_idx != INVALID_IDX)
    {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
      {
        continue;
      }
      else
      {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    raycaster.setInput(pt_w / mp_.resolution_, md_.ray_pos_ / mp_.resolution_);

    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      length = (tmp - md_.ray_pos_).norm();

      if(coverage_ray)observed_columns_[coverageKey(tmp.x(),tmp.y())]=scan_time;
      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX)
      {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
        {
          break;
        }
        else
        {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  min_x = min(min_x, md_.ray_pos_(0));
  min_y = min(min_y, md_.ray_pos_(1));
  min_z = min(min_z, md_.ray_pos_(2));

  max_x = max(max_x, md_.ray_pos_(0));
  max_y = max(max_y, md_.ray_pos_(1));
  max_z = max(max_z, md_.ray_pos_(2));
  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  // update occupancy cached in queue
  Eigen::Vector3d local_range_min = md_.ray_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.ray_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  while (!md_.cache_voxel_.empty())
  {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();
    observation_times_[idx_ctns]=latest_scan_stamp_ns_>0 ? latest_scan_stamp_ns_*1e-9 : node_->now().seconds();
    const plan_env_detail::WorldCell world_key={idx.x(),idx.y(),idx.z()};
    const bool measured_hit=md_.count_hit_[idx_ctns]>0;
    if(free_evidence_frame_valid_ && measured_hit){
      bool altered=world_changes_.suppressed(world_key)||world_changes_.uncertain(world_key);
      world_changes_.observe(world_key,true,free_evidence_stamp_);
      if(altered && static_observation_refresh_){
        static_refresh_pending_=true;static_restore_urgent_=true;route_map_changed_=true;
      }
    }
    if(free_evidence_frame_valid_ && (static_prior_mask_[idx_ctns] || world_changes_.hasHistory(world_key))){
      double now=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
      if(static_observation_refresh_ &&
         static_free_evidence_.observe(idx_ctns,measured_hit,now,free_evidence_stamp_)){
        static_refresh_pending_=true;static_restore_urgent_=true;
      }
      if(!measured_hit && static_free_evidence_.suppressesPrior(idx_ctns)){
        if(static_observation_refresh_ &&
           world_changes_.observe(world_key,false,free_evidence_stamp_)){
          static_refresh_pending_=true;static_restore_urgent_=true;route_map_changed_=true;
        }
        applyOccupancyUpdate(idx,mp_.clamp_min_log_);
      }
    }

    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
    {
      continue;
    }
    else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
    {
      applyOccupancyUpdate(idx, mp_.clamp_min_log_);
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
                    idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!in_local)
    {
      applyOccupancyUpdate(idx, mp_.clamp_min_log_);
    }

    const double new_log_odds =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
    applyOccupancyUpdate(idx, new_log_odds);
  }

}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &ray_pos)
{
  Eigen::Vector3d diff = pt - ray_pos;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - ray_pos;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - ray_pos;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return ray_pos + (min_t - 1e-3) * diff;
}

void GridMap::visCallback()
{
  refreshStaticObservations();
  const auto started = std::chrono::steady_clock::now();
  publishMap();
  publishPlanningView();
  publishMapInflate(true);
  publishSlidingMapFrame();
  publishSlidingMapBBox();
  publishDepthCloud();
  publishProjectedGrids();
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "[GridMap performance] visualization_ms=%.3f", elapsed_ms);
}


void GridMap::refreshGlobalGrid(bool force)
{
  const double now=node_->now().seconds();
  if(!force && now-last_global_rebuild_s_<1.0 && !global_grid_.empty()) return;
  last_global_rebuild_s_=now;
  rebuildGlobalGrid();
}

int GridMap::globalCellState(double x, double y) const
{
  if(!hasGlobalGrid())return -2;
  const int ix=static_cast<int>(std::floor((x-global_origin_.x())/global_res_));
  const int iy=static_cast<int>(std::floor((y-global_origin_.y())/global_res_));
  if(ix<0||ix>=global_w_||iy<0||iy>=global_h_)return -2;
  const int8_t v=global_grid_[static_cast<size_t>(iy)*global_w_+ix];
  if(v<0)return -1;
  return v>=100?100:0;
}

int GridMap::fusedCellState(double x, double y) const
{
  if(!fused_2d_enabled_ || !std::isfinite(x) || !std::isfinite(y) ||
     fused_w_<=0 || fused_h_<=0 || fused_2d_.empty()) return -2;
  const int ix=static_cast<int>(std::floor((x-fused_origin_.x())/fused_res_));
  const int iy=static_cast<int>(std::floor((y-fused_origin_.y())/fused_res_));
  if(ix<0||ix>=fused_w_||iy<0||iy>=fused_h_)return -2;
  const int8_t v=fused_2d_[static_cast<size_t>(iy)*fused_w_+ix];
  return v<0 ? -1 : (v>=100 ? 100 : 0);
}

void GridMap::rebuildFusedGrid()
{
  if(!fused_2d_enabled_)return;
  const double res=mp_.resolution_;
  if(!(res>1e-3))return;
  const Eigen::Vector3d lo=mp_.map_min_boundary_;
  const Eigen::Vector3d hi=mp_.map_max_boundary_;
  const int w=std::max(1,static_cast<int>(std::ceil((hi.x()-lo.x())/res)));
  const int h=std::max(1,static_cast<int>(std::ceil((hi.y()-lo.y())/res)));
  if(static_cast<long long>(w)*h>1000000LL)return;
  std::vector<int8_t> local(static_cast<size_t>(w)*h,-1);
  const auto set=[&](double x,double y,int8_t value){
    if(!std::isfinite(x)||!std::isfinite(y))return;
    const int ix=static_cast<int>(std::floor((x-lo.x())/res));
    const int iy=static_cast<int>(std::floor((y-lo.y())/res));
    if(ix<0||iy<0||ix>=w||iy>=h)return;
    local[static_cast<size_t>(iy)*w+ix]=value;
  };
  // Short-term local free evidence. It is never written to the global grid.
  for(const auto& entry:observed_columns_){
    const int32_t kx=static_cast<int32_t>(entry.first>>32);
    const int32_t ky=static_cast<int32_t>(entry.first&0xffffffffULL);
    set(kx*.1+.05,ky*.1+.05,0);
  }
  // Current local returns always win over the global reference.
  for(const auto& p:getLiveObstaclePoints()){
    if(!p.allFinite())continue;
    set(p.x(),p.y(),100);
  }
  // Feed short-horizon dynamic obstacle predictions into the local fused
  // planning grid (excluding the robot's immediate ego radius so predictions
  // never falsely trap the start cell) so SCAN proactively plans around
  // approaching/crossing humans before close-range emergency stops.
  if(predictionFresh()){
    const auto& body=md_.sliding_map_frame_pos_;
    for(const auto& p:predicted_points_){
      if(!p.allFinite())continue;
      if(std::hypot(p.x()-body.x(),p.y()-body.y())<0.65)continue;
      set(p.x(),p.y(),100);
    }
  }
  fused_origin_=lo.head<2>();fused_res_=res;fused_w_=w;fused_h_=h;
  fused_2d_.assign(static_cast<size_t>(w)*h,-1);
  for(int iy=0;iy<h;++iy)for(int ix=0;ix<w;++ix){
    const int8_t l=local[static_cast<size_t>(iy)*w+ix];
    if(l==100){fused_2d_[static_cast<size_t>(iy)*w+ix]=100;continue;}
    if(l==0){fused_2d_[static_cast<size_t>(iy)*w+ix]=0;continue;}
    const double wx=lo.x()+(ix+.5)*res,wy=lo.y()+(iy+.5)*res;
    const int g=globalCellState(wx,wy);
    fused_2d_[static_cast<size_t>(iy)*w+ix]=static_cast<int8_t>(fuseOccupancy2D(l,g==-2?-1:g));
    if(l==0 && known_free_cells_.size()<500000) known_free_cells_.insert(coverageKey(wx,wy));
  }
  planning_dirty_=true;
  rebuildPlanningGrid();
}

void GridMap::rebuildPlanningGrid()
{
  if(!fused_2d_enabled_) return;
  const bool has_global=hasGlobalGrid();
  const bool has_local=!fused_2d_.empty() && fused_w_>0 && fused_h_>0 && fused_res_>1e-6;
  if(!has_global && !has_local) return;
  planning_res_=has_global ? global_res_ : fused_res_;
  Eigen::Vector2d lo,hi;
  if(has_global){lo=global_origin_;hi=global_origin_+Eigen::Vector2d(global_w_*global_res_,global_h_*global_res_);}
  if(has_local){
    const Eigen::Vector2d llo=fused_origin_,lhi=fused_origin_+Eigen::Vector2d(fused_w_*fused_res_,fused_h_*fused_res_);
    if(has_global){lo=lo.cwiseMin(llo);hi=hi.cwiseMax(lhi);}else{lo=llo;hi=lhi;}
  }
  planning_origin_=lo;
  planning_w_=std::max(1,static_cast<int>(std::ceil((hi.x()-lo.x())/planning_res_)));
  planning_h_=std::max(1,static_cast<int>(std::ceil((hi.y()-lo.y())/planning_res_)));
  if(static_cast<long long>(planning_w_)*planning_h_>4000000LL)return;
  planning_grid_.assign(static_cast<size_t>(planning_w_)*planning_h_,-1);
  const auto put=[&](double x,double y,int8_t value){
    const int ix=static_cast<int>(std::floor((x-lo.x())/planning_res_));
    const int iy=static_cast<int>(std::floor((y-lo.y())/planning_res_));
    if(ix<0||iy<0||ix>=planning_w_||iy>=planning_h_)return;
    planning_grid_[static_cast<size_t>(iy)*planning_w_+ix]=value;
  };
  // Global base is copied, never modified.
  if(has_global){
    for(int iy=0;iy<global_h_;++iy)for(int ix=0;ix<global_w_;++ix){
      const int8_t v=global_grid_[static_cast<size_t>(iy)*global_w_+ix];
      put(global_origin_.x()+ix*global_res_,global_origin_.y()+iy*global_res_,v);
    }
  }
  // Session-only free memory: unknown -> free, but never clear a global 100.
  for(const auto key:known_free_cells_){
    const int32_t kx=static_cast<int32_t>(key>>32);
    const int32_t ky=static_cast<int32_t>(key&0xffffffffULL);
    const int ix=static_cast<int>(std::floor((kx*.1+.05-lo.x())/planning_res_));
    const int iy=static_cast<int>(std::floor((ky*.1+.05-lo.y())/planning_res_));
    if(ix>=0&&ix<planning_w_&&iy>=0&&iy<planning_h_){
      auto&c=planning_grid_[static_cast<size_t>(iy)*planning_w_+ix];
      if(c<100)c=0;
    }
  }
  // Local overlay: explicit 0/100 wins; -1 leaves global unchanged. This is
  // what lets local free outside the PCD extent participate in planning.
  if(has_local){
    for(int ly=0;ly<fused_h_;++ly)for(int lx=0;lx<fused_w_;++lx){
      const int8_t v=fused_2d_[static_cast<size_t>(ly)*fused_w_+lx];
      if(v<0)continue;
      put(fused_origin_.x()+(lx+.5)*fused_res_,fused_origin_.y()+(ly+.5)*fused_res_,v);
    }
  }
  planning_cloud_.clear();
  planning_cloud_.reserve(static_cast<size_t>(planning_w_)*planning_h_/16+16);
  const auto z=md_.sliding_map_frame_pos_.z();
  for(int iy=0;iy<planning_h_;++iy)for(int ix=0;ix<planning_w_;++ix){
    if(planning_grid_[static_cast<size_t>(iy)*planning_w_+ix]!=100)continue;
    planning_cloud_.push_back(pcl::PointXYZ(lo.x()+ix*planning_res_,lo.y()+iy*planning_res_,z));
  }
  planning_cloud_.width=planning_cloud_.size();planning_cloud_.height=1;planning_cloud_.is_dense=true;
  planning_dirty_=false;
}

const pcl::PointCloud<pcl::PointXYZ>& GridMap::getPlanningOccupancyCloud()
{
  if(planning_dirty_)rebuildPlanningGrid();
  return planning_cloud_;
}

int GridMap::globalFreeComponentSize(double x, double y) const
{
  if(globalCellState(x,y)!=0)return 0;
  const int sx=static_cast<int>(std::floor((x-global_origin_.x())/global_res_));
  const int sy=static_cast<int>(std::floor((y-global_origin_.y())/global_res_));
  const int n=global_w_*global_h_;
  std::vector<uint8_t> seen(static_cast<size_t>(n),0);
  std::vector<int> stack;stack.reserve(4096);
  const int start=sy*global_w_+sx;stack.push_back(start);seen[start]=1;
  int count=0;
  while(!stack.empty()){
    const int id=stack.back();stack.pop_back();++count;
    const int cx=id%global_w_,cy=id/global_w_;
    const int nb[4]={id-1,id+1,id-global_w_,id+global_w_};
    for(int k=0;k<4;++k){
      if(k==0&&cx==0)continue;if(k==1&&cx==global_w_-1)continue;
      if(k==2&&cy==0)continue;if(k==3&&cy==global_h_-1)continue;
      if(nb[k]<0||nb[k]>=n||seen[nb[k]])continue;
      if(global_grid_[nb[k]]!=0)continue;
      seen[nb[k]]=1;stack.push_back(nb[k]);
    }
  }
  return count;
}

bool GridMap::clearGlobalStart(double x, double y, double radius)
{
  if (fused_2d_enabled_) return false;
  if(!hasGlobalGrid())return false;
  const int cx=static_cast<int>(std::floor((x-global_origin_.x())/global_res_));
  const int cy=static_cast<int>(std::floor((y-global_origin_.y())/global_res_));
  const int r=std::max(1,static_cast<int>(std::ceil(radius/global_res_)));
  bool changed=false;
  for(int dy=-r;dy<=r;++dy)for(int dx=-r;dx<=r;++dx){
    if(dx*dx+dy*dy>r*r)continue;
    const int ix=cx+dx,iy=cy+dy;
    if(ix<0||iy<0||ix>=global_w_||iy>=global_h_)continue;
    auto&v=global_grid_[static_cast<size_t>(iy)*global_w_+ix];
    if(v!=0){v=0;changed=true;}
  }
  if(changed){
    global_version_++;
    global_cloud_.clear();global_cloud_.reserve(static_cast<size_t>(global_w_)*global_h_/16+16);
    const double z=md_.sliding_map_frame_pos_.z();
    for(int iy=0;iy<global_h_;++iy)for(int ix=0;ix<global_w_;++ix){
      if(global_grid_[static_cast<size_t>(iy)*global_w_+ix]!=100)continue;
      global_cloud_.push_back(pcl::PointXYZ(global_origin_.x()+ix*global_res_,
                                            global_origin_.y()+iy*global_res_,z));
    }
    global_cloud_.width=global_cloud_.size();global_cloud_.height=1;global_cloud_.is_dense=true;
  }
  return changed;
}

void GridMap::globalCounts(int& free_cells,int& occupied_cells,int& unknown_cells) const
{
  free_cells=occupied_cells=unknown_cells=0;
  for(const int8_t v:global_grid_){
    if(v<0)++unknown_cells;else if(v>=100)++occupied_cells;else ++free_cells;
  }
}

uint32_t GridMap::globalTileVersion(double x,double y) const
{
  if(global_tile_version_.empty()||global_res_<=0.)return 0;
  const int tx=static_cast<int>(std::floor((x-global_origin_.x())/(global_res_*global_tile_m_)));
  const int ty=static_cast<int>(std::floor((y-global_origin_.y())/(global_res_*global_tile_m_)));
  const int tw=std::max(1,static_cast<int>(std::ceil(global_w_/double(global_tile_m_))));
  const int th=std::max(1,static_cast<int>(std::ceil(global_h_/double(global_tile_m_))));
  if(tx<0||ty<0||tx>=tw||ty>=th)return 0;
  return global_tile_version_[static_cast<size_t>(ty)*tw+tx];
}

void GridMap::rebuildGlobalGrid()
{
  if (fused_2d_enabled_ && !reference_updates_enabled_ && hasGlobalGrid()) return;
  if(!static_cloud_enabled_||!static_source_valid_||static_source_decoded_.empty())return;
  const double res=mp_.resolution_;
  if(!(res>1e-3))return;
  Eigen::Vector2d lo(static_source_min_.x()-1.,static_source_min_.y()-1.);
  Eigen::Vector2d hi(static_source_max_.x()+1.,static_source_max_.y()+1.);
  const double cells=(hi.x()-lo.x())/res*(hi.y()-lo.y())/res;
  if(!std::isfinite(cells)||cells>4000000.){
    RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),5000,
      "Persistent global grid skipped: %.0f cells exceeds budget",cells);
    return;
  }
  const int w=std::max(1,static_cast<int>(std::ceil((hi.x()-lo.x())/res)));
  const int h=std::max(1,static_cast<int>(std::ceil((hi.y()-lo.y())/res)));
  std::vector<int8_t> next(static_cast<size_t>(w)*h,-1);
  const auto set=[&](double x,double y,int8_t value){
    const int ix=static_cast<int>(std::floor((x-lo.x())/res));
    const int iy=static_cast<int>(std::floor((y-lo.y())/res));
    if(ix<0||iy<0||ix>=w||iy>=h)return;
    next[static_cast<size_t>(iy)*w+ix]=value;
  };
  const auto& static_points = fused_2d_enabled_ ? static_source_decoded_ : effective_static_source_;
  const auto& global_body=md_.sliding_map_frame_pos_;
  std::vector<Eigen::Vector2d> global_ego_poly;
  if(static_ego_filter_enabled_ && urdf_footprint_enabled_ && footprintReady())
    global_ego_poly=footprintPolygon(md_.body_yaw_);
  for(const auto& pt:static_points.points){
    if(!std::isfinite(pt.x)||!std::isfinite(pt.y)||!std::isfinite(pt.z))continue;
    if(static_ego_filter_enabled_ && !fused_2d_enabled_){
      const double dx=pt.x-global_body.x(),dy=pt.y-global_body.y();
      const double dxy2=dx*dx+dy*dy;
      bool self=false;
      if(dxy2<0.64 && pt.z>=global_body.z()-static_ego_filter_z_down_ && pt.z<=global_body.z()+static_ego_filter_z_up_){
        if(!global_ego_poly.empty())
          self=plan_env_detail::polygonHit(Eigen::Vector2d(pt.x-global_body.x(),pt.y-global_body.y()),
                                           global_ego_poly,static_ego_filter_margin_);
        if(!self)self=dxy2<static_ego_filter_radius_*static_ego_filter_radius_;
      }
      if(!self && ego_history_enabled_ && !ego_trail_z_.empty()){
        const uint64_t key=(static_cast<uint64_t>(static_cast<uint32_t>(
            static_cast<int32_t>(std::floor(pt.x/std::max(0.05,mp_.resolution_)))))<<32) |
            static_cast<uint32_t>(static_cast<int32_t>(std::floor(pt.y/std::max(0.05,mp_.resolution_))));
        auto hit=ego_trail_z_.find(key);
        if(hit!=ego_trail_z_.end() && pt.z>=hit->second.first && pt.z<=hit->second.second)self=true;
      }
      if(self)continue;
    }
    if(pt.z<=getObstacleGroundCutoff(pt.x,pt.y))continue;
    set(pt.x,pt.y,100);
  }
  // Static-map semantics: inside the actual PCD XY extent, absence of an
  // obstacle is treated as free. Outside the PCD extent remains unknown.
  if(pcd_bounds_free_enabled_ && static_source_valid_){
    const int x0=static_cast<int>(std::floor((static_source_min_.x()-lo.x())/res));
    const int x1=static_cast<int>(std::ceil((static_source_max_.x()-lo.x())/res));
    const int y0=static_cast<int>(std::floor((static_source_min_.y()-lo.y())/res));
    const int y1=static_cast<int>(std::ceil((static_source_max_.y()-lo.y())/res));
    for(int iy=std::max(0,y0);iy<std::min(h,y1+1);++iy)
      for(int ix=std::max(0,x0);ix<std::min(w,x1+1);++ix){
        auto&c=next[static_cast<size_t>(iy)*w+ix];
        if(c<0)c=0;
      }
  }
  // In fused 2D mode the global layer is READ-ONLY: local free/obstacle
  // evidence only affects fusedCellState(), never this persistent layer.
  if(!fused_2d_enabled_){
  if(std::abs(res-global_hits_res_)>1e-9){global_live_hits_.clear();global_hits_res_=res;}
  const double now_s=node_->now().seconds();
  const double live_ttl=10.0;
  const auto key_for=[&](double x,double y)->uint64_t{
    const int32_t cx=static_cast<int32_t>(std::floor(x/res));
    const int32_t cy=static_cast<int32_t>(std::floor(y/res));
    return (static_cast<uint64_t>(static_cast<uint32_t>(cx))<<32)|static_cast<uint32_t>(cy);
  };
  for(auto it=global_live_hits_.begin();it!=global_live_hits_.end();){
    if(now_s-it->second>live_ttl)it=global_live_hits_.erase(it);else ++it;
  }
  const auto body_z=md_.sliding_map_frame_pos_.z();
  const auto& live_body=md_.sliding_map_frame_pos_;
  std::vector<Eigen::Vector2d> live_ego_poly;
  if(static_ego_filter_enabled_ && urdf_footprint_enabled_ && footprintReady())
    live_ego_poly=footprintPolygon(md_.body_yaw_);
  const auto ego_2d=[&](double x,double y){
    if(!static_ego_filter_enabled_)return false;
    const double dx=x-live_body.x(),dy=y-live_body.y();
    const double dxy2=dx*dx+dy*dy;
    if(dxy2>0.64)return false;
    if(!live_ego_poly.empty() &&
       plan_env_detail::polygonHit(Eigen::Vector2d(x-live_body.x(),y-live_body.y()),
                                   live_ego_poly,static_ego_filter_margin_))return true;
    return dxy2<static_ego_filter_radius_*static_ego_filter_radius_;
  };
  for(const auto& p:getLiveObstaclePoints()){
    if(!std::isfinite(p.z())||p.z()<=getObstacleGroundCutoff(p.x(),p.y()))continue;
    if(ego_2d(p.x(),p.y()))continue;
    global_live_hits_[key_for(p.x(),p.y())]=now_s;
  }
  for(const auto& e:observed_columns_){
    const int32_t kx=static_cast<int32_t>(e.first>>32);
    const int32_t ky=static_cast<int32_t>(e.first&0xffffffffULL);
    if(next.empty())continue;
    set(kx*0.1+0.05,ky*0.1+0.05,0);
    global_live_hits_.erase(key_for(kx*0.1+0.05,ky*0.1+0.05));
  }
  for(const auto& hit:global_live_hits_){
    const int32_t cx=static_cast<int32_t>(hit.first>>32);
    const int32_t cy=static_cast<int32_t>(hit.first&0xffffffffULL);
    const double x=cx*res,y=cy*res;
    if(ego_2d(x,y))continue;
    const int ix=static_cast<int>(std::floor((x-lo.x())/res));
    const int iy=static_cast<int>(std::floor((y-lo.y())/res));
    if(ix<0||iy<0||ix>=w||iy>=h)continue;
    next[static_cast<size_t>(iy)*w+ix]=100;
  }
  // Single-map authority: inside the local window, the local reference/live
  // grids override the independently filtered global layer. Outside the
  // window the global static layer remains the far-field source.
  if(!reference_grid_data_.empty() && reference_grid_res_>1e-6){
    const auto overlay_local=[&](const std::vector<int8_t>& src,bool live){
      if(src.empty())return;
      const double lres=reference_grid_res_;
      const int lw=reference_grid_w_,lh=reference_grid_h_;
      const double lx0=reference_grid_origin_.x(),ly0=reference_grid_origin_.y();
      for(int iy=0;iy<global_h_;++iy)for(int ix=0;ix<global_w_;++ix){
        const double wx=global_origin_.x()+(ix+.5)*global_res_;
        const double wy=global_origin_.y()+(iy+.5)*global_res_;
        const int lx=static_cast<int>(std::floor((wx-lx0)/lres));
        const int ly=static_cast<int>(std::floor((wy-ly0)/lres));
        if(lx<0||ly<0||lx>=lw||ly>=lh)continue;
        const int8_t v=src[static_cast<size_t>(ly)*lw+lx];
        if(v<0)continue;
        if(v>=100){
          next[static_cast<size_t>(iy)*global_w_+ix]=100;
        }else if(!live){
          next[static_cast<size_t>(iy)*global_w_+ix]=0;
        }else{
          // live free is authoritative inside the window as well
          next[static_cast<size_t>(iy)*global_w_+ix]=0;
        }
      }
    };
    overlay_local(reference_grid_data_,false);
    overlay_local(live_grid_data_,true);
  }
  }
  const int tw=std::max(1,static_cast<int>(std::ceil(w/double(global_tile_m_))));
  const int th=std::max(1,static_cast<int>(std::ceil(h/double(global_tile_m_))));
  std::vector<uint32_t> versions(static_cast<size_t>(tw)*th,0);
  const bool compare=!global_grid_.empty()&&global_w_==w&&global_h_==h;
  for(int ty=0;ty<th;++ty)for(int tx=0;tx<tw;++tx){
    bool changed=!compare;
    if(compare){
      for(int dy=0;dy<global_tile_m_&&!changed;++dy)for(int dx=0;dx<global_tile_m_;++dx){
        const int x=tx*global_tile_m_+dx,y=ty*global_tile_m_+dy;
        if(x>=w||y>=h)continue;
        if(next[static_cast<size_t>(y)*w+x]!=global_grid_[static_cast<size_t>(y)*w+x]){changed=true;break;}
      }
    }
    const uint32_t previous=(compare&&!global_tile_version_.empty())
        ? global_tile_version_[static_cast<size_t>(ty)*tw+tx] : 0u;
    versions[static_cast<size_t>(ty)*tw+tx]=previous+(changed?1u:0u);
  }
  global_grid_.swap(next);global_tile_version_.swap(versions);
  planning_dirty_=true;
  global_origin_=lo;global_w_=w;global_h_=h;global_res_=res;global_version_++;
  global_cloud_.clear();global_cloud_.reserve(static_cast<size_t>(w)*h/16+16);
  for(int iy=0;iy<h;++iy)for(int ix=0;ix<w;++ix){
    if(global_grid_[static_cast<size_t>(iy)*w+ix]!=100)continue;
    global_cloud_.push_back(pcl::PointXYZ(lo.x()+ix*res,lo.y()+iy*res,md_.sliding_map_frame_pos_.z()));
  }
  global_cloud_.width=global_cloud_.size();global_cloud_.height=1;global_cloud_.is_dense=true;
  if(global_grid_pub_){
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.frame_id=mp_.frame_id_;msg.header.stamp=node_->now();
    msg.info.resolution=res;msg.info.width=w;msg.info.height=h;
    msg.info.origin.position.x=lo.x();msg.info.origin.position.y=lo.y();
    msg.info.origin.orientation.w=1.0;
    msg.data.resize(global_grid_.size());
    for(size_t i=0;i<global_grid_.size();++i)msg.data[i]=global_grid_[i];
    global_grid_pub_->publish(msg);
  }
  RCLCPP_INFO_THROTTLE(node_->get_logger(),*node_->get_clock(),5000,
    "Persistent global grid: %dx%d res=%.2f occupied=%zu version=%llu",
    w,h,res,global_cloud_.size(),static_cast<unsigned long long>(global_version_));
}

void GridMap::publishProjectedGrids()
{
  const double res = mp_.resolution_;
  const Eigen::Vector3d lo = mp_.map_min_boundary_;
  const int w = std::max(1, static_cast<int>(std::ceil((mp_.map_max_boundary_.x() - lo.x()) / res)));
  const int h = std::max(1, static_cast<int>(std::ceil((mp_.map_max_boundary_.y() - lo.y()) / res)));
  if (static_cast<long long>(w) * h > 1000000LL) return;
  const auto now = node_->now();
  auto build = [&](bool live) {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = mp_.frame_id_;
    grid.header.stamp = now;
    grid.info.resolution = res;
    grid.info.width = static_cast<uint32_t>(w);
    grid.info.height = static_cast<uint32_t>(h);
    grid.info.origin.position.x = lo.x();
    grid.info.origin.position.y = lo.y();
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(static_cast<size_t>(w) * h, -1);
    // Observed columns are known free at display resolution (0.1 m cells).
    const double cell = 0.1;
    const int span = std::max(1, static_cast<int>(std::round(cell / res)));
    for (const auto& e : observed_columns_) {
      const int32_t kx = static_cast<int32_t>(e.first >> 32);
      const int32_t ky = static_cast<int32_t>(e.first & 0xffffffffULL);
      const int x0 = static_cast<int>(std::floor((kx * cell - lo.x()) / res));
      const int y0 = static_cast<int>(std::floor((ky * cell - lo.y()) / res));
      for (int dy = 0; dy < span; ++dy)
        for (int dx = 0; dx < span; ++dx) {
          const int x = x0 + dx, y = y0 + dy;
          if (x >= 0 && y >= 0 && x < w && y < h)
            grid.data[static_cast<size_t>(y) * w + x] = 0;
        }
    }
    std::vector<Eigen::Vector3d> points;
    if (live) points = getLiveObstaclePoints();
    else {
      points.reserve(static_window_prior_cells_.size());
      for (const auto& id : static_window_prior_cells_) {
        Eigen::Vector3d p; indexToPos(id, p); points.push_back(p);
      }
    }
    for (const auto& p : points) {
      if (!p.allFinite() || !isInMap(p)) continue;
      const double cutoff = getObstacleGroundCutoff(p.x(), p.y());
      if (p.z() <= cutoff || p.z() > getObstacleHeightMax(p.x(), p.y())) continue;
      const int x = static_cast<int>(std::floor((p.x() - lo.x()) / res));
      const int y = static_cast<int>(std::floor((p.y() - lo.y()) / res));
      if (x >= 0 && y >= 0 && x < w && y < h)
        grid.data[static_cast<size_t>(y) * w + x] = 100;
    }
    if (live) {
      live_grid_data_ = grid.data;
      reference_grid_origin_ = lo.head<2>();
      reference_grid_res_ = res;
    } else {
      reference_grid_data_ = grid.data;
      reference_grid_origin_ = lo.head<2>();
      reference_grid_res_ = res;
    }
    if (!live) {
      reference_grid_w_ = w; reference_grid_h_ = h;
    } else {
      // Both local grids share the same window.
      reference_grid_w_ = w; reference_grid_h_ = h;
    }
    return grid;
  };
  auto ref_grid = build(false);
  if (reference_grid_pub_) reference_grid_pub_->publish(ref_grid);
  auto live_grid = build(true);
  if (live_grid_pub_) live_grid_pub_->publish(live_grid);
}

void GridMap::ageStaticPriors(){
  if(fused_2d_enabled_ || !static_observation_refresh_ || latest_scan_stamp_ns_<=last_prior_scan_stamp_ns_)return;
  last_prior_scan_stamp_ns_=latest_scan_stamp_ns_;
  for(const auto& id:static_window_prior_cells_){
    if(!isInMap(id))continue;
    const int addr=toAddress(id);Eigen::Vector3d p;indexToPos(id,p);
    if(!hasObservedCoverage(p) || (observationFresh(addr) && md_.occupancy_buffer_[addr]>std::min(-.01,mp_.prob_miss_log_))){
      prior_observation_since_.erase(addr);continue;
    }
    auto inserted=prior_observation_since_.emplace(addr,latest_scan_stamp_ns_);
    if((latest_scan_stamp_ns_-inserted.first->second)*1e-9>=observation_ttl_s_){
      if(world_changes_.ageHit({id.x(),id.y(),id.z()},latest_scan_stamp_ns_)){
        static_refresh_pending_=true;static_restore_urgent_=true;route_map_changed_=true;
      }
    }
  }
}

void GridMap::expireObservations() {
  const double now=node_->now().seconds();observation_now_s_=now;
  for(auto it=observation_times_.begin();it!=observation_times_.end();) {
    if(now<it->second || now-it->second>observation_ttl_s_) {
      Eigen::Vector3i id;hashIdToGlobalIndex(it->first,id);
      // Age is a quality downgrade, never fabricated free evidence. Keep a
      // world-keyed soft risk, rather than an everlasting hard collision cell.
      if(md_.occupancy_buffer_[it->first]>std::min(-.01,mp_.prob_miss_log_)){
        if(world_changes_.ageHit({id.x(),id.y(),id.z()})){
          static_refresh_pending_=true;static_restore_urgent_=true;route_map_changed_=true;
        }
      }
      applyOccupancyUpdate(id,mp_.clamp_min_log_-mp_.unknown_flag_);
      it=observation_times_.erase(it);
    }else ++it;
  }
  for(auto it=observed_columns_.begin();it!=observed_columns_.end();)
    if(now<it->second || now-it->second>observation_ttl_s_)it=observed_columns_.erase(it);else ++it;
  ageStaticPriors();
}

void GridMap::updateOccupancyCallback()
{
  expireObservations();
  if (!md_.occ_need_update_)
    return;
  const auto started = std::chrono::steady_clock::now();

  /* update occupancy */
  // ros::Time t1, t2, t3, t4;
  // t1 = ros::Time::now();

  if (!md_.use_cloud_update_)
    projectDepthImage();
  // t2 = ros::Time::now();
  raycastProcess();
  processed_scan_stamp_ns_=latest_scan_stamp_ns_;
  refreshStaticObservations();
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "[GridMap performance] occupancy_ms=%.3f points=%d", elapsed_ms,
      md_.proj_points_cnt);
  // t3 = ros::Time::now();

  // t4 = ros::Time::now();

  // cout << setprecision(7);
  // cout << "t2=" << (t2-t1).toSec() << " t3=" << (t3-t2).toSec() << " t4=" << (t4-t3).toSec() << endl;;

  // md_.fuse_time_ += (t2 - t1).toSec();
  // md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

  // if (mp_.show_occ_time_)
  //   ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
  //            md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  md_.occ_need_update_ = false;
  md_.use_cloud_update_ = false;
}

void GridMap::depthPoseCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img,
                                const nav_msgs::msg::Odometry::ConstSharedPtr &pose)
{
  if (mp_.sensor_type_ != "depth")
    return;

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

  /* get pose */
  const geometry_msgs::msg::Pose &sensor_pose = pose->pose.pose;
  Eigen::Quaterniond ray_q(sensor_pose.orientation.w, sensor_pose.orientation.x,
                           sensor_pose.orientation.y, sensor_pose.orientation.z);
  if (ray_q.norm() < 1e-6)
    return;
  ray_q.normalize();

  Eigen::Vector3d ray_pos(sensor_pose.position.x, sensor_pose.position.y, sensor_pose.position.z);
  if (mp_.need_extrinsic_)
  {
    const Eigen::Matrix3d pose_r = ray_q.toRotationMatrix();
    ray_pos += pose_r * mp_.depth_extrinsic_.block<3, 1>(0, 3);
    ray_q = Eigen::Quaterniond(pose_r * mp_.depth_extrinsic_.block<3, 3>(0, 0));
    ray_q.normalize();
  }

  nav_msgs::msg::Odometry extrinsic_pose = *pose;
  extrinsic_pose.pose.pose.position.x = ray_pos.x();
  extrinsic_pose.pose.pose.position.y = ray_pos.y();
  extrinsic_pose.pose.pose.position.z = ray_pos.z();
  extrinsic_pose.pose.pose.orientation.x = ray_q.x();
  extrinsic_pose.pose.pose.orientation.y = ray_q.y();
  extrinsic_pose.pose.pose.orientation.z = ray_q.z();
  extrinsic_pose.pose.pose.orientation.w = ray_q.w();
  extrinsic_pose.child_frame_id =
      pose->child_frame_id.empty() ? "sensor_extrinsic" : pose->child_frame_id + "_extrinsic";
  extrinsic_pose_pub_->publish(extrinsic_pose);

  md_.ray_pos_ = ray_pos;
  md_.ray_q_ = ray_q;
  md_.use_cloud_update_ = false;
  updateSlidingMap(md_.ray_pos_);
  if (isInMap(md_.ray_pos_))
  {
    md_.has_ray_pose_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  }
  else
  {
    md_.occ_need_update_ = false;
  }
}

void GridMap::sensorPoseCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg)
{
  if (mp_.sensor_type_ != "lidar")
    return;

  const geometry_msgs::msg::Pose &sensor_pose = pose_msg->pose.pose;
  Eigen::Quaterniond ray_q(sensor_pose.orientation.w, sensor_pose.orientation.x,
                           sensor_pose.orientation.y, sensor_pose.orientation.z);
  if (ray_q.norm() < 1e-6)
    return;
  ray_q.normalize();

  Eigen::Vector3d ray_pos(sensor_pose.position.x, sensor_pose.position.y, sensor_pose.position.z);
  if (mp_.need_extrinsic_)
  {
    const Eigen::Matrix3d pose_r = ray_q.toRotationMatrix();
    ray_pos += pose_r * mp_.lidar_extrinsic_.block<3, 1>(0, 3);
    ray_q = Eigen::Quaterniond(pose_r * mp_.lidar_extrinsic_.block<3, 3>(0, 0));
    ray_q.normalize();
  }
  if (!std::isfinite(ray_pos.x()) || !std::isfinite(ray_pos.y()) || !std::isfinite(ray_pos.z()))
    return;

  md_.ray_pos_ = ray_pos;
  md_.ray_q_ = ray_q;
  md_.has_ray_pose_ = true;
  updateSlidingMap(md_.ray_pos_);
}

void GridMap::cloudPoseCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud,
    const nav_msgs::msg::Odometry::ConstSharedPtr& pose)
{
  if (!cloud || !pose) return;
  if (cloud->header.stamp.sec != pose->header.stamp.sec ||
      cloud->header.stamp.nanosec != pose->header.stamp.nanosec) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "[GridMap] reject unsynchronized cloud/sensor_pose pair");
    return;
  }
  sensorPoseCallback(pose);
  const int64_t stamp=static_cast<int64_t>(cloud->header.stamp.sec)*1000000000LL+cloud->header.stamp.nanosec;
  const double age=(node_->now().nanoseconds()-stamp)*1e-9;
  free_evidence_frame_valid_=cloud->header.frame_id==mp_.frame_id_ && stamp>last_free_evidence_stamp_ && age>=-0.1 && age<=2.0;
  free_evidence_stamp_=stamp;
  if(free_evidence_frame_valid_) last_free_evidence_stamp_=stamp;
  cloudCallback(cloud);
}

void GridMap::noteEgoTrailPose()
{
  if(!ego_history_enabled_ || !static_ego_filter_enabled_ || !urdf_footprint_enabled_ ||
     !footprintReady()) return;
  const Eigen::Vector3d body=md_.sliding_map_frame_pos_;
  const double yaw=md_.body_yaw_;
  if(last_ego_history_valid_){
    const double dxy=(body-last_ego_history_pose_).head<2>().norm();
    const double dyaw=std::atan2(std::sin(yaw-last_ego_history_yaw_),
                                 std::cos(yaw-last_ego_history_yaw_));
    if(dxy<ego_history_step_m_ && std::abs(dyaw)<0.20) return;
  }
  last_ego_history_pose_=body;last_ego_history_yaw_=yaw;last_ego_history_valid_=true;
  const auto poly=footprintPolygon(yaw);
  if(poly.size()<3) return;
  Eigen::Vector2d lo=poly.front(),hi=poly.front();
  for(const auto&v:poly){lo=lo.cwiseMin(v);hi=hi.cwiseMax(v);}
  const double res=std::max(0.05,mp_.resolution_);
  const int x0=static_cast<int>(std::floor(lo.x()/res));
  const int x1=static_cast<int>(std::ceil(hi.x()/res));
  const int y0=static_cast<int>(std::floor(lo.y()/res));
  const int y1=static_cast<int>(std::ceil(hi.y()/res));
  const double z0=body.z()-static_ego_filter_z_down_;
  const double z1=body.z()+static_ego_filter_z_up_;
  for(int iy=y0;iy<=y1;++iy)for(int ix=x0;ix<=x1;++ix){
    const Eigen::Vector2d local(ix*res,iy*res);
    if(!plan_env_detail::polygonHit(local,poly,static_ego_filter_margin_))continue;
    const double wx=body.x()+local.x(),wy=body.y()+local.y();
    const uint64_t key=(static_cast<uint64_t>(static_cast<uint32_t>(
        static_cast<int32_t>(std::floor(wx/res))))<<32) |
        static_cast<uint32_t>(static_cast<int32_t>(std::floor(wy/res)));
    auto it=ego_trail_z_.find(key);
    if(it==ego_trail_z_.end())ego_trail_z_[key]={z0,z1};
    else{it->second.first=std::min(it->second.first,z0);it->second.second=std::max(it->second.second,z1);}
  }
  if(ego_trail_z_.size()>ego_history_max_cells_){
    RCLCPP_WARN(node_->get_logger(),"[GridMap] ego trail cell cap reached; clearing history (%zu)",ego_trail_z_.size());
    ego_trail_z_.clear();
  }
}

void GridMap::slidingMapFrameCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose)
{
  const geometry_msgs::msg::Point &pos = pose->pose.pose.position;
  md_.sliding_map_frame_pos_ = Eigen::Vector3d(pos.x, pos.y, pos.z);
  const auto &q=pose->pose.pose.orientation;
  const double previous_yaw=md_.body_yaw_;
  md_.body_yaw_=std::atan2(2.0*(q.w*q.z+q.x*q.y),1.0-2.0*(q.y*q.y+q.z*q.z));
  const double dyaw=std::atan2(std::sin(md_.body_yaw_-previous_yaw),std::cos(md_.body_yaw_-previous_yaw));
  if(static_ego_filter_enabled_ && std::abs(dyaw)>0.20){
    // Rotation changes which static points are ego; throttle via the normal
    // static refresh instead of rebuilding every pose callback.
    static_refresh_pending_=true;
  }
  noteEgoTrailPose();
}

void GridMap::staticCloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud)
{
  if (!static_cloud_enabled_ || (static_source_cloud_ && !reference_updates_enabled_)) return;
  if (reference_updates_enabled_ && static_source_cloud_ &&
      rclcpp::Time(cloud->header.stamp) <= rclcpp::Time(static_source_cloud_->header.stamp)) return;
  if (cloud->header.frame_id != mp_.frame_id_) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[GridMap] refuse static PCD in frame %s; expected %s",
                 cloud->header.frame_id.c_str(), mp_.frame_id_.c_str());
    publishStaticOverlayStatus(false, "static_map_frame_mismatch");
    return;
  }
  const uint64_t points =
      static_cast<uint64_t>(cloud->width) * static_cast<uint64_t>(cloud->height);
  if (points == 0 || points > static_cast<uint64_t>(static_cloud_max_points_)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[GridMap] static PCD point count %llu exceeds bounded limit %d; "
                 "no partial map silently accepted",
                 static_cast<unsigned long long>(points), static_cloud_max_points_);
    publishStaticOverlayStatus(false, "static_map_point_count_out_of_bounds");
    return;
  }
  pcl::PointCloud<pcl::PointXYZ> decoded;
  try {
    pcl::fromROSMsg(*cloud, decoded);
  } catch (const std::exception& exc) {
    RCLCPP_ERROR(node_->get_logger(), "[GridMap] bad static PCD: %s", exc.what());
    publishStaticOverlayStatus(false, "static_map_decode_failed");
    return;
  }
  if (decoded.empty()) {
    publishStaticOverlayStatus(false, "static_map_empty");
    return;
  }

  Eigen::Vector3d source_min = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  Eigen::Vector3d source_max = Eigen::Vector3d::Constant(
      -std::numeric_limits<double>::infinity());
  size_t finite_points = 0;
  pcl::PointCloud<pcl::PointXYZ> filtered;
  filtered.points.reserve(decoded.points.size());
  for (const auto& pt : decoded.points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
      continue;
    source_min = source_min.cwiseMin(Eigen::Vector3d(pt.x, pt.y, pt.z));
    source_max = source_max.cwiseMax(Eigen::Vector3d(pt.x, pt.y, pt.z));
    filtered.points.push_back(pt);
    ++finite_points;
  }
  filtered.width = static_cast<uint32_t>(finite_points);
  filtered.height = 1;
  filtered.is_dense = true;
  if (finite_points == 0) {
    publishStaticOverlayStatus(false, "static_map_has_no_finite_points");
    return;
  }

  static_source_decoded_ = std::move(filtered);
  static_source_cloud_ = cloud;
  static_source_min_ = source_min;
  static_source_max_ = source_max;
  static_source_points_ = finite_points;
  static_source_valid_ = true;
  rebuildStaticOverlay();
}

void GridMap::publishStaticOverlayStatus(bool ready, const std::string& reason)
{
  if (static_ready_pub_) {
    std_msgs::msg::Bool ready_msg;
    ready_msg.data = ready;
    static_ready_pub_->publish(ready_msg);
  }
  if (!static_status_pub_) return;

  std::ostringstream out;
  out << std::fixed << std::setprecision(3)
      << "{\"schema\":\"n01.static_overlay_status.v1\","
      << "\"enabled\":" << (static_cloud_enabled_ ? "true" : "false")
      << ",\"source_received\":" << (static_source_valid_ ? "true" : "false")
      << ",\"source_frame\":\"" << mp_.frame_id_ << "\""
      << ",\"source_points\":" << static_source_points_
      << ",\"observation_refresh_enabled\":" << (static_observation_refresh_ ? "true" : "false")
      << ",\"static_reference_cost_in_local_window\":" << (static_prior_as_cost_ ? "true" : "false")
      << ",\"static_points_overridden_by_free_rays\":"
      << (static_observation_refresh_ ? static_source_decoded_.size()-effective_static_source_.size() : 0)
      << ",\"historical_change_cells\":" << world_changes_.size()
      << ",\"window_points\":" << static_window_points_
      << ",\"occupied_voxels_in_window\":" << static_occupied_voxels_
      << ",\"rebuild_duration_ms\":" << static_overlay_rebuild_ms_
      << ",\"inflation_xy_kernel_size\":" << static_inflate_xy_offsets_.size()
      << ",\"inflation_z_down_voxels\":" << static_inflate_z_down_steps_
      << ",\"inflation_z_up_voxels\":" << static_inflate_z_up_steps_
      << ",\"unconfirmed_risk_points\":" << world_changes_.uncertainCells().size()
      << ",\"observed_columns\":" << observed_columns_.size()
      << ",\"environment_margin_m\":" << mp_.environment_margin_
      << ",\"robot_footprint_model\":\"" << (urdf_footprint_enabled_ ? "urdf_steering_hull" : "three_disks") << "\""
      << ",\"robot_footprint_ready\":" << (footprintReady() ? "true" : "false")
      << ",\"robot_footprint_radius_m\":" << (urdf_footprint_enabled_ ? 0. : mp_.double_cylinder_radius_)
      << ",\"collision_query_radius_m\":" << getFootprintRadius()
      << ",\"inflation_strategy\":\"xy_row_span_fill\""
      << ",\"window_intersects_source_bounds\":"
      << (static_window_intersects_source_bounds_ ? "true" : "false")
      << ",\"ready\":" << (ready ? "true" : "false")
      << ",\"reason\":\"" << reason << "\""
      << ",\"window_min_xyz\":["
      << mp_.map_min_boundary_.x() << "," << mp_.map_min_boundary_.y() << ","
      << mp_.map_min_boundary_.z() << "]"
      << ",\"window_max_xyz\":["
      << mp_.map_max_boundary_.x() << "," << mp_.map_max_boundary_.y() << ","
      << mp_.map_max_boundary_.z() << "]";
  if (static_source_valid_) {
    out << ",\"source_min_xyz\":["
        << static_source_min_.x() << "," << static_source_min_.y() << ","
        << static_source_min_.z() << "]"
        << ",\"source_max_xyz\":["
        << static_source_max_.x() << "," << static_source_max_.y() << ","
        << static_source_max_.z() << "]";
  }
  out << "}";
  std_msgs::msg::String status_msg;
  status_msg.data = out.str();
  static_status_pub_->publish(status_msg);
}

void GridMap::refreshStaticObservations(bool global_request)
{
  if(!static_source_valid_) return;
  const double now=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  if(!static_observation_refresh_){
    // Offline PCD has no live reference refresh, but the ego filter still has
    // to follow the current pose. Rebuild at a bounded rate, never per scan.
    if(static_refresh_pending_ && now-last_static_rebuild_s_>=0.5)
      rebuildStaticOverlay();
    return;
  }
  static_free_evidence_.expire(now); // timeout never resurrects historical occupancy
  // Local collision checking uses the live layer in reference mode. Refresh
  // historical costs lazily for global planning, not on every sensor frame.
  if(static_prior_as_cost_ && !global_request) return;
  if ((fused_2d_enabled_ && now-last_static_rebuild_s_>=0.5) ||
      (static_refresh_pending_ && (static_restore_urgent_ || now-last_static_rebuild_s_>=0.5)))
    rebuildStaticOverlay();
}

void GridMap::rebuildStaticOverlay()
{
  if (!static_cloud_enabled_ || !static_source_cloud_ || !static_source_valid_) return;
  const auto rebuild_start = std::chrono::steady_clock::now();
  const double evidence_now=std::chrono::duration<double>(rebuild_start.time_since_epoch()).count();
  static_refresh_pending_=false;static_restore_urgent_=false;last_static_rebuild_s_=evidence_now;

  // Static layer is independent of dynamic occupancy probabilities. Cache the
  // decoded PCD once; sliding-window rebuilds only reproject the cached points.
  std::fill(static_occupancy_buffer_.begin(), static_occupancy_buffer_.end(), 0);
  std::fill(static_inflate_buffer_.begin(), static_inflate_buffer_.end(), 0);
  std::fill(static_prior_mask_.begin(),static_prior_mask_.end(),0);
  effective_static_source_.clear();
  if(static_observation_refresh_) effective_static_source_.reserve(static_source_decoded_.size());
  static_window_points_ = 0;
  const auto& cloud = static_source_decoded_;
  const auto& local_body=md_.sliding_map_frame_pos_;
  std::vector<Eigen::Vector2d> local_ego_poly;
  if(static_ego_filter_enabled_ && urdf_footprint_enabled_ && footprintReady())
    local_ego_poly=footprintPolygon(md_.body_yaw_);
  const auto ego_point=[&](const pcl::PointXYZ& pt){
    if(!static_ego_filter_enabled_)return false;
    const double dx=pt.x-local_body.x(),dy=pt.y-local_body.y();
    const double dxy2=dx*dx+dy*dy;
    if(dxy2>0.64)return false;
    if(pt.z<local_body.z()-static_ego_filter_z_down_||pt.z>local_body.z()+static_ego_filter_z_up_)return false;
    if(!local_ego_poly.empty() &&
       plan_env_detail::polygonHit(Eigen::Vector2d(pt.x-local_body.x(),pt.y-local_body.y()),
                                   local_ego_poly,static_ego_filter_margin_))return true;
    if(ego_history_enabled_ && !ego_trail_z_.empty()){
      const uint64_t key=(static_cast<uint64_t>(static_cast<uint32_t>(
          static_cast<int32_t>(std::floor(pt.x/std::max(0.05,mp_.resolution_)))))<<32) |
          static_cast<uint32_t>(static_cast<int32_t>(std::floor(pt.y/std::max(0.05,mp_.resolution_))));
      auto hit=ego_trail_z_.find(key);
      if(hit!=ego_trail_z_.end() && pt.z>=hit->second.first && pt.z<=hit->second.second)return true;
    }
    return dxy2<static_ego_filter_radius_*static_ego_filter_radius_;
  };

  // A sparse occupancy map can legitimately contain no obstacle returns in
  // the active local window. Readiness therefore means that a valid, finite,
  // correctly framed source map was loaded and its bounds overlap this window;
  // the local occupied-voxel count remains a separate diagnostic.
  static_window_intersects_source_bounds_ =
      (static_source_max_.array() >= mp_.map_min_boundary_.array()).all() &&
      (static_source_min_.array() <= mp_.map_max_boundary_.array()).all();

  std::vector<Eigen::Vector3i> occupied;
  occupied.reserve(std::min(cloud.points.size(), static_occupancy_buffer_.size()));
  for (const auto& pt : cloud.points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
      continue;
    if (!fused_2d_enabled_ && (pt.x < mp_.map_min_boundary_.x() || pt.x > mp_.map_max_boundary_.x() ||
        pt.y < mp_.map_min_boundary_.y() || pt.y > mp_.map_max_boundary_.y()))
      continue;
    if (ego_point(pt)) continue;
    const Eigen::Vector3d p(pt.x, pt.y, pt.z);
    Eigen::Vector3i world;posToIndex(p,world);
    bool observed_free=!fused_2d_enabled_ && static_observation_refresh_ && world_changes_.suppressed({world.x(),world.y(),world.z()});
    if(isInMap(p)) {
      Eigen::Vector3i cell;posToIndex(p,cell);const int addr=toAddress(cell);
      if(p.z()>getObstacleGroundCutoff(p.x(),p.y()) &&
         p.z()<=getObstacleHeightMax(p.x(),p.y())) {
        static_prior_mask_[addr]=1;
      }
      if (static_observation_refresh_) {
        observed_free = observed_free || (fused_2d_enabled_
            ? static_free_evidence_.free(addr, evidence_now)
            : static_free_evidence_.suppressesPrior(addr));
      }
    }
    const bool unconfirmed=!fused_2d_enabled_ && isInMap(p) && world_changes_.uncertain({world.x(),world.y(),world.z()});
    if(!observed_free && !unconfirmed) effective_static_source_.push_back(pt);
    if(observed_free || unconfirmed) continue;
    if (p.z() <= getObstacleGroundCutoff(p.x(), p.y()) ||
        p.z() > getObstacleHeightMax(p.x(), p.y())) continue;
    if (!isInMap(p)) continue;
    ++static_window_points_;
    Eigen::Vector3i id;
    posToIndex(p, id);
    const int addr = toAddress(id);
    if (static_occupancy_buffer_[addr] == 0) {
      static_occupancy_buffer_[addr] = 1;
      occupied.push_back(id);
    }
  }
  if(!static_prior_as_cost_) plan_env_detail::markMergedStaticInflationRows(
        static_inflate_buffer_,occupied,
        mp_.map_voxel_num_(0), mp_.map_voxel_num_(1), mp_.map_voxel_num_(2),
        mp_.map_bound_min_idx_(0), mp_.map_bound_min_idx_(1), mp_.map_bound_min_idx_(2),
        mp_.map_bound_max_idx_(0), mp_.map_bound_max_idx_(1), mp_.map_bound_max_idx_(2),
        static_inflate_xy_offsets_,
        static_inflate_z_down_steps_, static_inflate_z_up_steps_);
  static_window_prior_cells_=occupied;
  static_occupied_voxels_ = occupied.size();
  static_overlay_rebuild_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - rebuild_start).count();

  const bool ready = static_source_valid_ && static_window_intersects_source_bounds_;
  const std::string reason = !static_source_valid_
      ? "waiting_for_static_cloud"
      : (!static_window_intersects_source_bounds_
          ? "active_window_outside_static_map_bounds"
          : (static_occupied_voxels_ == 0
              ? "map_loaded_window_has_no_occupied_voxels"
              : "ready"));
  publishStaticOverlayStatus(ready, reason);

  RCLCPP_INFO(node_->get_logger(),
              "[GridMap] static PCD prior overlay: source=%zu points, "
              "window=%zu points/%zu occupied voxels, rebuild=%.3f ms, ready=%s (%s)",
              static_source_points_, static_window_points_, static_occupied_voxels_,
              static_overlay_rebuild_ms_, ready ? "true" : "false", reason.c_str());
}

void GridMap::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &img)
{
  if (mp_.sensor_type_ != "lidar")
    return;

  if (!md_.has_ray_pose_)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "[GridMap] no sensor_pose received for lidar cloud update");
    return;
  }

  const int64_t stamp=rclcpp::Time(img->header.stamp).nanoseconds();
  const double age=(node_->now().nanoseconds()-stamp)*1e-9;
  if(stamp<=latest_scan_stamp_ns_ || age<-.1 || age>1.)return;
  latest_scan_stamp_ns_=stamp;
  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  md_.has_cloud_ = true;

  if (latest_cloud.points.size() == 0)
    return;

  const Eigen::Matrix3d sensor_r = md_.ray_q_.toRotationMatrix();
  const Eigen::Vector3d ray_pos = md_.ray_pos_;
  if (!std::isfinite(ray_pos.x()) || !std::isfinite(ray_pos.y()) || !std::isfinite(ray_pos.z()))
    return;

  updateSlidingMap(ray_pos);

  std::vector<Eigen::Vector2d> live_ego_poly;
  if(live_ego_filter_enabled_ && urdf_footprint_enabled_ && footprintReady())
    live_ego_poly=footprintPolygon(md_.body_yaw_);
  md_.proj_points_cnt = 0;
  md_.proj_ground_.clear();
  std::unordered_set<uint64_t> raycast_voxels;
  if(raycast_dedup_resolution_>1e-6)
    raycast_voxels.reserve(static_cast<size_t>(latest_cloud.points.size()));
  std::vector<float> ground_labels;
  for(const auto& field:img->fields)if(field.name=="ground" && field.datatype==sensor_msgs::msg::PointField::FLOAT32) {
    sensor_msgs::PointCloud2ConstIterator<float> label(*img,"ground");
    for(size_t i=0;i<latest_cloud.size();++i,++label)ground_labels.push_back(*label);
    break;
  }

  for (size_t i = 0; i < latest_cloud.points.size(); ++i)
  {
    const pcl::PointXYZ &pt = latest_cloud.points[i];
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
      continue;

    Eigen::Vector3d pt_world;
    if (mp_.cloud_is_world_)
    {
      pt_world = Eigen::Vector3d(pt.x, pt.y, pt.z);
    }
    else
    {
      const Eigen::Vector3d pt_sensor(pt.x, pt.y, pt.z);
      pt_world = sensor_r * pt_sensor + ray_pos;
    }
    // Keep ground returns for free-space ray tracing; raycastProcess prevents
    // their endpoints from becoming obstacles. Dropping them here leaves an
    // empty corridor unable to refute stale static occupancy.
    const Eigen::Vector3d devi = pt_world - ray_pos;
    const double ray_length = devi.norm();
    const bool in_local_range =
        fabs(devi(0)) <= mp_.local_update_range_(0) && fabs(devi(1)) <= mp_.local_update_range_(1) &&
        fabs(devi(2)) <= mp_.local_update_range_(2);
    if (!in_local_range && ray_length <= mp_.max_ray_length_)
      continue;
    if(raycast_dedup_resolution_>1e-6){
      const int32_t ix=static_cast<int32_t>(std::floor(pt_world.x()/raycast_dedup_resolution_));
      const int32_t iy=static_cast<int32_t>(std::floor(pt_world.y()/raycast_dedup_resolution_));
      const int32_t iz=static_cast<int32_t>(std::floor(pt_world.z()/raycast_dedup_resolution_));
      const uint64_t key=(static_cast<uint64_t>(static_cast<uint32_t>(ix))<<42) ^
                         (static_cast<uint64_t>(static_cast<uint32_t>(iy))<<21) ^
                         static_cast<uint32_t>(iz);
      if(!raycast_voxels.insert(key).second)continue;
    }

    if (md_.proj_points_cnt >= static_cast<int>(md_.proj_points_.size()))
      md_.proj_points_.push_back(pt_world);
    else
      md_.proj_points_[md_.proj_points_cnt] = pt_world;

    const bool ground=ground_labels.empty() ? pt_world.z()<=getObstacleGroundCutoff(pt_world.x(),pt_world.y())
        : std::isfinite(ground_labels[i]) && ground_labels[i]>.5f;
    // Remove the robot's own live returns before they enter the local live
    // layer. Static-layer ego filtering cannot cover the current /cloud frame.
    if(live_ego_filter_enabled_){
      const auto& body=md_.sliding_map_frame_pos_;
      bool self=false;
      const double dx=pt_world.x()-body.x(),dy=pt_world.y()-body.y();
      const double dxy2=dx*dx+dy*dy;
      if(dxy2<0.64 && pt_world.z()>=body.z()-static_ego_filter_z_down_ &&
         pt_world.z()<=body.z()+static_ego_filter_z_up_){
        if(!live_ego_poly.empty()){
          self=plan_env_detail::polygonHit(Eigen::Vector2d(pt_world.x()-body.x(),pt_world.y()-body.y()),
                                           live_ego_poly,live_ego_filter_margin_);
        }
        if(!self)self=dxy2<static_ego_filter_radius_*static_ego_filter_radius_;
      }
      if(!self && ego_history_enabled_ && !ego_trail_z_.empty()){
        const uint64_t key=(static_cast<uint64_t>(static_cast<uint32_t>(
            static_cast<int32_t>(std::floor(pt.x/std::max(0.05,mp_.resolution_)))))<<32) |
            static_cast<uint32_t>(static_cast<int32_t>(std::floor(pt.y/std::max(0.05,mp_.resolution_))));
        auto hit=ego_trail_z_.find(key);
        if(hit!=ego_trail_z_.end() && pt.z>=hit->second.first && pt.z<=hit->second.second)self=true;
      }
      if(self)continue;
    }
    md_.proj_ground_.push_back(ground);
    md_.proj_points_cnt++;
  }

  if (md_.proj_points_cnt == 0)
    return;

  md_.use_cloud_update_ = true;
  md_.occ_need_update_ = true;
  rebuildFusedGrid();
}

void GridMap::publishPlanningView(){
  const double now=node_->now().seconds();
  if(now>=last_planning_view_s_ && now-last_planning_view_s_<1.)return;
  if(fused_view_pub_->get_subscription_count()==0 && risk_view_pub_->get_subscription_count()==0)return;
  last_planning_view_s_=now;
  const auto emit=[&](const auto& points,const auto& pub){
    pcl::PointCloud<pcl::PointXYZ> view;
    for(const auto& p:points){
      Eigen::Vector3d v(p.x,p.y,p.z);
      if(v.z()<=getObstacleGroundCutoff(v.x(),v.y()) ||
         v.z()>getObstacleHeightMax(v.x(),v.y()))continue;
      view.push_back(p);
    }
    sensor_msgs::msg::PointCloud2 msg;pcl::toROSMsg(view,msg);msg.header.frame_id=mp_.frame_id_;msg.header.stamp=node_->now();pub->publish(msg);
  };
  if(fused_view_pub_->get_subscription_count()>0){
    const auto& background=getStaticSourceCloud();emit(background,fused_view_pub_);
  }
  if(risk_view_pub_->get_subscription_count()>0){
    pcl::PointCloud<pcl::PointXYZ> risk;for(const auto& p:getUnconfirmedPoints())risk.push_back(pcl::PointXYZ(p.x(),p.y(),p.z()));
    emit(risk,risk_view_pub_);
  }
}

void GridMap::publishMap()
{

  if (map_pub_->get_subscription_count() == 0 && environment_inf_pub_->get_subscription_count()==0 && observed_free_pub_->get_subscription_count()==0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  std::unordered_set<int> addresses;
  for(const auto& observed:observation_times_)if(collisionRisk(md_.occupancy_buffer_[observed.first]))addresses.insert(observed.first);
  if(static_cloud_enabled_ && !static_prior_as_cost_)for(const auto& p:static_source_decoded_) {
    Eigen::Vector3d point(p.x,p.y,p.z);if(!isInMap(point))continue;
    Eigen::Vector3i id;posToIndex(point,id);int a=toAddress(id);if(static_occupancy_buffer_[a])addresses.insert(a);
  }
  for(int addr:addresses) {
    Eigen::Vector3i id;hashIdToGlobalIndex(addr,id);Eigen::Vector3d pos;indexToPos(id,pos);
    if(md_.has_ray_pose_ && pos.z()>md_.ray_pos_.z()+mp_.vis_height_)continue;
    cloud.push_back(pcl::PointXYZ(pos.x(),pos.y(),pos.z()));
  }


  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp.sec=processed_scan_stamp_ns_/1000000000LL;
  cloud_msg.header.stamp.nanosec=processed_scan_stamp_ns_%1000000000LL;
  map_pub_->publish(cloud_msg);
  if(observed_free_pub_->get_subscription_count()>0 && require_observed_space_ && md_.has_ray_pose_) {
    pcl::PointCloud<pcl::PointXYZ> free_cloud;
    const double step=.1,z=md_.sliding_map_frame_pos_.z();
    for(double x=std::ceil(mp_.map_min_boundary_.x()/step)*step+.05;x<mp_.map_max_boundary_.x();x+=step)
      for(double y=std::ceil(mp_.map_min_boundary_.y()/step)*step+.05;y<mp_.map_max_boundary_.y();y+=step) {
        bool free=true;
        for(double dx:{-.025,.025})for(double dy:{-.025,.025})
          free=free && observedFree(Eigen::Vector3d(x+dx,y+dy,z));
        if(free)free_cloud.push_back(pcl::PointXYZ(x,y,z));
      }
    sensor_msgs::msg::PointCloud2 free_msg;pcl::toROSMsg(free_cloud,free_msg);
    free_msg.header=cloud_msg.header;observed_free_pub_->publish(free_msg);
  }
  // This layer contains ONLY the environmental margin, not the robot's
  // configuration-space kernel. Reuse the raw export; no second volume scan.
  if(environment_inf_pub_->get_subscription_count()>0 && md_.has_ray_pose_) {
    const auto offsets=plan_env_detail::environmentDisk(mp_.environment_margin_,mp_.resolution_);
    std::unordered_set<uint64_t> cells;
    pcl::PointCloud<pcl::PointXYZ> environment;
    const double body_z=md_.sliding_map_frame_pos_.z();
    for(const auto& p:cloud.points) {
      if(p.z<=getObstacleGroundCutoff(p.x,p.y) ||
         p.z>getObstacleHeightMax(p.x,p.y)) continue;
      const int ix=static_cast<int>(std::floor(p.x/mp_.resolution_));
      const int iy=static_cast<int>(std::floor(p.y/mp_.resolution_));
      for(const auto& offset:offsets) {
        const int x=ix+offset.x,y=iy+offset.y;
        const uint64_t key=(static_cast<uint64_t>(static_cast<uint32_t>(x))<<32) |
                           static_cast<uint32_t>(y);
        if(cells.insert(key).second) {
          pcl::PointXYZ q;
          q.x=(x+.5)*mp_.resolution_;q.y=(y+.5)*mp_.resolution_;q.z=body_z;
          environment.push_back(q);
        }
      }
    }
    environment.width=environment.size();environment.height=1;
    sensor_msgs::msg::PointCloud2 output;pcl::toROSMsg(environment,output);
    output.header=cloud_msg.header;environment_inf_pub_->publish(output);
  }
}

void GridMap::publishMapInflate(bool all_info)
{

  if (map_inf_pub_->get_subscription_count() == 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = mp_.map_bound_min_idx_;
  Eigen::Vector3i max_cut = mp_.map_bound_max_idx_;

  const std::vector<char> &inflate_buffer = md_.occupancy_buffer_inflate_;
  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        const int addr = toAddress(x, y, z);
        if (inflate_buffer[addr] == 0 &&
            (!static_cloud_enabled_ || static_prior_as_cost_ || static_inflate_buffer_[addr] == 0))
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (md_.has_ray_pose_ && pos(2) > md_.ray_pos_(2) + mp_.vis_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = node_->now();
  map_inf_pub_->publish(cloud_msg);

  // ROS_INFO("pub map");
}

void GridMap::publishSlidingMapFrame()
{
  if (mp_.sliding_map_frame_id_.empty())
    return;

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node_->now();
  transform.header.frame_id = mp_.frame_id_;
  transform.child_frame_id = mp_.sliding_map_frame_id_;
  transform.transform.translation.x = md_.sliding_map_frame_pos_.x();
  transform.transform.translation.y = md_.sliding_map_frame_pos_.y();
  transform.transform.translation.z = md_.sliding_map_frame_pos_.z();
  transform.transform.rotation.w = 1.0;
  tf_broadcaster_->sendTransform(transform);
}

void GridMap::publishSlidingMapBBox()
{
  if (sliding_map_bbox_pub_->get_subscription_count() == 0)
    return;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = mp_.frame_id_;
  marker.header.stamp = node_->now();
  marker.ns = "sliding_map";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.04;
  marker.color.r = 0.0;
  marker.color.g = 0.8;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  const Eigen::Vector3d& min_pt = mp_.map_min_boundary_;
  const Eigen::Vector3d& max_pt = mp_.map_max_boundary_;
  Eigen::Vector3d corners[8] = {
      {min_pt.x(), min_pt.y(), min_pt.z()},
      {max_pt.x(), min_pt.y(), min_pt.z()},
      {max_pt.x(), max_pt.y(), min_pt.z()},
      {min_pt.x(), max_pt.y(), min_pt.z()},
      {min_pt.x(), min_pt.y(), max_pt.z()},
      {max_pt.x(), min_pt.y(), max_pt.z()},
      {max_pt.x(), max_pt.y(), max_pt.z()},
      {min_pt.x(), max_pt.y(), max_pt.z()},
  };

  auto pushPoint = [&](const Eigen::Vector3d& p) {
    geometry_msgs::msg::Point point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    marker.points.push_back(point);
  };

  const int edges[12][2] = {
      {0, 1}, {1, 2}, {2, 3}, {3, 0},
      {4, 5}, {5, 6}, {6, 7}, {7, 4},
      {0, 4}, {1, 5}, {2, 6}, {3, 7},
  };

  for (const auto& edge : edges)
  {
    pushPoint(corners[edge[0]]);
    pushPoint(corners[edge[1]]);
  }

  sliding_map_bbox_pub_->publish(marker);
}

void GridMap::publishUnknown()
{
  // This is visualization only: avoid a full voxel traversal when unused.
  if (unknown_pub_->get_subscription_count() == 0)
    return;
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  boundIndex(max_cut);
  boundIndex(min_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.clamp_min_log_ - 1e-3)
        {
          Eigen::Vector3d pos;
          indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (md_.has_ray_pose_ && pos(2) > md_.ray_pos_(2) + mp_.vis_height_)
            continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = node_->now();
  unknown_pub_->publish(cloud_msg);
}

bool GridMap::odomValid() { return md_.has_ray_pose_; }

bool GridMap::hasDepthObservation() { return md_.has_first_depth_; }

Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

// int GridMap::getVoxelNum() {
//   return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
// }

void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

// GridMap

void GridMap::publishDepthCloud()
{
  if (depth_cloud_pub_->get_subscription_count() == 0)
    return;

  if (md_.proj_points_cnt == 0)
    return;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ pt;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt.x = md_.proj_points_[i](0);
    pt.y = md_.proj_points_[i](1);
    pt.z = md_.proj_points_[i](2);
    cloud.push_back(pt);
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = node_->now();
  depth_cloud_pub_->publish(cloud_msg);
}
