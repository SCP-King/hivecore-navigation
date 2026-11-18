#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <algorithm>
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <plan_env/polygon_footprint.h>
#include <iostream>
#include <limits>
#include <random>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <tuple>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <plan_env/raycast.h>
#include <plan_env/static_inflation.hpp>
#include <plan_env/static_free_evidence.h>
#include <plan_env/world_evidence.h>
#include <unordered_map>

#define logit(x) (log((x) / (1 - (x))))

using namespace std;
// voxel hashing
template <typename T>
struct matrix_hash {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

// constant parameters

struct MappingParameters {

  /* map properties */
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;  // map range in pos
  Eigen::Vector3i map_voxel_num_;                        // map range in index
  Eigen::Vector3i map_bound_min_idx_, map_bound_max_idx_;
  Eigen::Vector3i map_origin_idx_;
  Eigen::Vector3d local_update_range_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_z_up, obstacles_inflation_z_down;
  double ground_clearance_;
  double ground_slope_x_, ground_slope_y_;
  double double_cylinder_radius_, double_cylinder_offset_;
  double environment_margin_{0.0};
  std::vector<double> footprint_centers_x_;
  bool fixed_body_heading_{false};
  bool map_sliding_en_;
  double map_sliding_thresh_;
  int map_sliding_thresh_vox_;
  string frame_id_, sliding_map_frame_id_;

  /* depth camera intrinsics */
  double cx_, cy_, fx_, fy_;

  /* depth image projection filtering */
  double depth_filter_maxdist_, depth_filter_mindist_;
  int depth_filter_margin_;
  double k_depth_scaling_factor_;
  int skip_pixel_;

  /* raycasting */
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;  // occupancy probability
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
      min_occupancy_log_;                   // logit of occupancy probability
  double min_ray_length_, max_ray_length_;  // range of doing raycasting

  /* visualization and computation time display */
  double vis_height_, ground_height_;
  bool show_occ_time_;

  /* mapping sensor input */
  string sensor_type_;
  bool cloud_is_world_;
  bool need_extrinsic_;
  Eigen::Matrix4d lidar_extrinsic_;
  Eigen::Matrix4d depth_extrinsic_;

  /* active mapping */
  double unknown_flag_;
};

// intermediate mapping data for fusion

struct MappingData {
  // main map data, occupancy of each voxel and Euclidean distance

  std::vector<double> occupancy_buffer_;
  std::vector<char> occupancy_buffer_inflate_;
  std::vector<int> occupancy_buffer_inflate_cnt_;
  vector<Eigen::Vector3i> inflate_offsets_;

  // raycast origin and sensor pose data

  Eigen::Vector3d ray_pos_;
  Eigen::Quaterniond ray_q_;
  Eigen::Vector3d sliding_map_frame_pos_;
  double body_yaw_{0.0};

  // depth image data

  cv::Mat depth_image_;
  int image_cnt_;
  // flags of map state

  bool occ_need_update_;
  bool use_cloud_update_;
  bool has_first_depth_;
  bool has_ray_pose_, has_cloud_;

  // depth image projected point cloud

  vector<Eigen::Vector3d> proj_points_;
  std::vector<char> proj_ground_; // scan-time classification, not synthetic endpoints
  int proj_points_cnt;

  // flag buffers for speeding up raycasting

  vector<std::uint32_t> count_hit_, count_hit_and_miss_;
  vector<std::uint32_t> flag_traverse_, flag_rayend_;
  std::uint32_t raycast_num_;
  queue<Eigen::Vector3i> cache_voxel_;

  // range of updating grid

  Eigen::Vector3i local_bound_min_, local_bound_max_;

  // computation time

  double fuse_time_, max_fuse_time_;
  int update_num_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GridMap {
public:
  GridMap() {}
  ~GridMap() {}

  enum { INVALID_IDX = -10000 };

  // occupancy map management
  void resetBuffer();
  void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);

  inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
  inline int toAddress(const Eigen::Vector3i& id);
  inline int toAddress(int& x, int& y, int& z);
  inline bool isInMap(const Eigen::Vector3d& pos);
  inline bool isInMap(const Eigen::Vector3i& idx);

  inline void setOccupancy(Eigen::Vector3d pos, double occ = 1);
  inline void setOccupied(Eigen::Vector3d pos);
  inline int getOccupancy(Eigen::Vector3d pos);
  inline int getOccupancy(Eigen::Vector3i id);
  inline int getInflateOccupancy(Eigen::Vector3d pos, double yaw, bool allow_unknown=false);

  inline void boundIndex(Eigen::Vector3i& id);
  inline bool isUnknown(const Eigen::Vector3i& id);
  inline bool isUnknown(const Eigen::Vector3d& pos);
  inline bool isKnownFree(const Eigen::Vector3i& id);
  inline bool isKnownOccupied(const Eigen::Vector3i& id);

  void initMap(rclcpp::Node* node);

  // Complete static PCD source (map frame), not the sliding-window copy.
  bool requiresObservedSpace() const { return require_observed_space_; }
  static uint64_t coverageKey(double x,double y) {
    return (uint64_t(uint32_t(int64_t(std::floor(x/.1))))<<32) |
           uint32_t(int64_t(std::floor(y/.1)));
  }
  bool hasObservedCoverage(const Eigen::Vector3d& p) const {
    auto it=observed_columns_.find(coverageKey(p.x(),p.y()));
    const double age=observation_now_s_- (it==observed_columns_.end()?0.:it->second);
    return it!=observed_columns_.end() && age>=-.1 && age<=observation_ttl_s_;
  }
  double collisionLogThreshold() const {
    return mp_.min_occupancy_log_;
  }
  bool collisionRisk(double value) const {
    return value>=mp_.clamp_min_log_ && value>collisionLogThreshold();
  }
  bool observationFresh(int addr) const {
    auto it=observation_times_.find(addr);
    return it!=observation_times_.end() && observation_now_s_>=it->second-.1 && observation_now_s_-it->second<=observation_ttl_s_;
  }
  bool observedFree(const Eigen::Vector3d& p,
                    const Eigen::Vector3d* envelope_center=nullptr) {
    if(!isInMap(p))return false;
    Eigen::Vector3i observed_id;posToIndex(p,observed_id);
    const int addr=toAddress(observed_id);
    // Visibility is 2D projected ray coverage, not a dense slice at body Z.
    // Occupancy remains 3D: a ground ray never erases an upper chair voxel.
    if(md_.occupancy_buffer_inflate_[addr] || collisionRisk(md_.occupancy_buffer_[addr]))return false;
    if(hasObservedCoverage(p))return true;
    // The current physical robot footprint is known to contain the robot,
    // not unobserved forward travel. Do not erase any occupied voxel here.
    // The collision query samples the environment-margin envelope, so the
    // exemption must cover that same ring: otherwise every sample just
    // outside the raw hull (LiDAR blind wedge at the rear/side) reads as
    // unobserved and the robot can never be planned out of its start pose.
    // Planning starts at a control point that can sit a few centimetres ahead
    // of the measured body pose, so the exemption is anchored to the queried
    // envelope while staying bounded to the real robot location.
    const Eigen::Vector3d origin=envelope_center?*envelope_center:md_.sliding_map_frame_pos_;
    // The rebound start can lead the measured pose by one control-point
    // spacing (0.2 m), so bound the anchor offset at 0.3 m.
    if((origin-md_.sliding_map_frame_pos_).head<2>().norm()>.30)return false;
    const Eigen::Vector2d d=(p-origin).head<2>();
    if(d.norm()>.85)return false;
    auto polygon=footprintPolygon(md_.body_yaw_);
    if(!urdf_footprint_enabled_ || polygon.size()<3)return false;
    bool inside=true;
    double edge_distance=std::numeric_limits<double>::infinity();
    for(size_t i=0;i<polygon.size();++i){
      const Eigen::Vector2d a=polygon[i],e=polygon[(i+1)%polygon.size()]-a,w=d-a;
      if(e.x()*w.y()-e.y()*w.x()<0)inside=false;
      const double len2=e.squaredNorm();
      const double t=len2>1e-12?std::max(0.,std::min(1.,w.dot(e)/len2)):0.;
      edge_distance=std::min(edge_distance,(w-t*e).norm());
    }
    return inside || edge_distance<=mp_.environment_margin_+1e-9;
  }
  bool hasStaticSource() const { return static_source_valid_; }
  const pcl::PointCloud<pcl::PointXYZ>& getStaticSourceCloud() {
    refreshStaticObservations(true);
    return static_observation_refresh_ ? effective_static_source_ : static_source_decoded_;
  }
  Eigen::Vector3d getStaticSourceMin() const { return static_source_min_; }
  Eigen::Vector3d getStaticSourceMax() const { return static_source_max_; }
  bool usesFixedBodyHeading() const { return mp_.fixed_body_heading_; }
  bool usesRealtimeReference() const { return reference_updates_enabled_; }
  // Goals outside the loaded map are plannable: unknown space is not an
  // obstacle. The search grid extends to include start and goal.
  bool extendsReferenceToGoal() const { return extend_reference_to_goal_; }
  bool usesStaticReferenceCost() const { return static_prior_as_cost_; }
  // First concrete blocker of the robot envelope at pos/yaw. A failed local
  // plan must name a cause and a location instead of a generic optimizer
  // error; an empty code means the envelope is clear for planning.
  struct EnvelopeBlocker { std::string code; double x=0., y=0., z=0.;
                           double yaw=0., dnorm=0.; int polygon_points=0; };
  EnvelopeBlocker envelopeBlocker(const Eigen::Vector3d& pos, double yaw) {
    EnvelopeBlocker out;
    const auto probe=[&](const Eigen::Vector3d& p)->const char* {
      if(!isInMap(p))return "outside_map";
      Eigen::Vector3i id;posToIndex(p,id);const int adr=toAddress(id);
      if(md_.occupancy_buffer_inflate_[adr])return "live_occupied";
      if(static_cloud_enabled_&&!static_prior_as_cost_&&!static_inflate_buffer_.empty()&&
         static_inflate_buffer_[adr])return "static_occupied";
      if(require_observed_space_&&!observedFree(p))return "not_observed";
      return nullptr;
    };
    const auto record=[&](const Eigen::Vector3d& p){
      const char* code=probe(p);
      if(!code)return false;
      out.code=code;out.x=p.x();out.y=p.y();out.z=p.z();
      out.yaw=yaw;out.dnorm=(p-md_.sliding_map_frame_pos_).head<2>().norm();
      out.polygon_points=int(footprint_polygon_.size());return true;
    };
    if(record(pos))return out;
    if(mp_.fixed_body_heading_)yaw=md_.body_yaw_;
    if(urdf_footprint_enabled_){
      if(!footprintReady()){
        out.code="footprint_missing";out.x=pos.x();out.y=pos.y();out.z=pos.z();return out;
      }
      const auto offsets=plan_env_detail::polygonOffsets(
          footprintPolygon(yaw),mp_.environment_margin_,mp_.resolution_);
      for(const auto& offset:offsets){
        const Eigen::Vector3d p=pos+Eigen::Vector3d(offset.x()*mp_.resolution_,
                                                    offset.y()*mp_.resolution_,0.);
        if(record(p))return out;
      }
      return out;
    }
    const Eigen::Vector3d heading(std::cos(yaw),std::sin(yaw),0.);
    for(double x:mp_.footprint_centers_x_){
      if(record(pos+x*heading))return out;
    }
    return out;
  }
  Eigen::Vector3d getLocalMinimum() const { return mp_.map_min_boundary_; }
  Eigen::Vector3d getLocalMaximum() const { return mp_.map_max_boundary_; }
  std::vector<Eigen::Vector3d> getLiveObstaclePoints() {
    std::vector<Eigen::Vector3d> points;points.reserve(observation_times_.size()/4);
    // Global and local use accumulated measured occupancy, not only one scan.
    for(const auto& e:observation_times_)if(collisionRisk(md_.occupancy_buffer_[e.first])){
      Eigen::Vector3i id;hashIdToGlobalIndex(e.first,id);Eigen::Vector3d p;indexToPos(id,p);
      if(isInMap(p))points.push_back(p);
    }
    return points;
  }
  std::vector<Eigen::Vector3d> getUnconfirmedPoints() {
    std::vector<Eigen::Vector3d> points;
    for(const auto& id:world_changes_.uncertainCells()){
      Eigen::Vector3d p;indexToPos(Eigen::Vector3i(id[0],id[1],id[2]),p);
      if(isInMap(p))points.push_back(p);
    }
    for(const auto& e:observation_times_){double v=md_.occupancy_buffer_[e.first];
      if(v>std::min(-.01,mp_.prob_miss_log_) && !collisionRisk(v)){
        Eigen::Vector3i id;hashIdToGlobalIndex(e.first,id);Eigen::Vector3d p;indexToPos(id,p);points.push_back(p);
      }
    }
    if(predictionFresh())points.insert(points.end(),predicted_points_.begin(),predicted_points_.end());
    return points;
  }
  int64_t sceneStamp() const{return processed_scan_stamp_ns_;}
  bool takeRouteMapChange(){bool changed=route_map_changed_;route_map_changed_=false;return changed;}

  void publishMap();
  // 2D projection of the same voxel buffers for display only (never planning input).
  void publishProjectedGrids();
  void publishMapInflate(bool all_info = false);

  void publishUnknown();
  void publishDepth();
  void publishDepthCloud();
  void publishSlidingMapBBox();
  void publishSlidingMapFrame();

  bool hasDepthObservation();
  bool odomValid();
  void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size);
  inline double getResolution();
  double getFootprintRadius() const { return urdf_footprint_enabled_ ? mp_.environment_margin_ : mp_.double_cylinder_radius_ + mp_.environment_margin_; }
  bool footprintReady() const {
    return !urdf_footprint_enabled_ || (footprint_valid_ &&
      (node_->now().nanoseconds()-footprint_stamp_ns_)*1e-9 >= -.1 &&
      (node_->now().nanoseconds()-footprint_stamp_ns_)*1e-9 <= footprint_max_age_s_);
  }
  std::vector<Eigen::Vector2d> footprintPolygon(double yaw) const {
    std::vector<Eigen::Vector2d> p;
    const Eigen::Rotation2Dd r(yaw);
    for(const auto &v:footprint_polygon_)p.push_back(r*v);
    return p;
  }
  std::vector<Eigen::Vector2d> getFootprintPolygon() const {return urdf_footprint_enabled_ ? footprintPolygon(md_.body_yaw_) : std::vector<Eigen::Vector2d>();}
  double getBodyDown() const { return mp_.obstacles_inflation_z_up; }
  double getBodyUp() const { return mp_.obstacles_inflation_z_down; }
  double getObstacleGroundCutoff(double x=0.0, double y=0.0) const {
    return mp_.ground_height_ + mp_.ground_slope_x_*x + mp_.ground_slope_y_*y + mp_.ground_clearance_;
  }
  Eigen::Vector3d getGroundCutoffPlane() const {
    return Eigen::Vector3d(mp_.ground_slope_x_, mp_.ground_slope_y_, mp_.ground_height_+mp_.ground_clearance_);
  }
  std::vector<Eigen::Vector2d> getFootprintCenters() const {
    if(urdf_footprint_enabled_) return {Eigen::Vector2d::Zero()};
    std::vector<Eigen::Vector2d> result;
    for (double x : mp_.footprint_centers_x_)
      result.emplace_back(x*std::cos(md_.body_yaw_), x*std::sin(md_.body_yaw_));
    return result;
  }
  Eigen::Vector3d getOrigin();
  int getVoxelNum();

  typedef std::shared_ptr<GridMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mp_;
  MappingData md_;

  // get depth image and sensor pose
  void depthPoseCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img,
                         const nav_msgs::msg::Odometry::ConstSharedPtr& pose);
  void sensorPoseCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& pose);
  void cloudPoseCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud,
                         const nav_msgs::msg::Odometry::ConstSharedPtr& pose);
  void slidingMapFrameCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& pose);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& img);
  // Static prior, locally overridden only by repeated fresh traversed-free rays.
  void staticCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud);
  void rebuildStaticOverlay();
  void refreshStaticObservations(bool global_request=false);
  void publishStaticOverlayStatus(bool ready, const std::string& reason);

  // update occupancy by raycasting
  void updateOccupancyCallback();
  void visCallback();

  // main update process
  void projectDepthImage();
  void raycastProcess();

  inline void inflatePoint(const Eigen::Vector3i& pt, int inf_step_xy, int inf_step_z_up, int inf_step_z_down, vector<Eigen::Vector3i>& pts);
  inline int getInflateOccupancyFromBuffer(Eigen::Vector3d pos, const std::vector<char>& buffer);
  inline int getLocalIndex(int id, int dim) const;
  inline int toAddressLocal(const Eigen::Vector3i& id_l) const;
  inline int toAddressLocal(int x, int y, int z) const;
  int setCacheOccupancy(Eigen::Vector3d pos, int occ);
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& ray_pos);
  void updateSlidingMap(const Eigen::Vector3d& center);
  void updateMapBoundaryFromIndex();
  void resetAllMapData();
  void resetCellByAddress(int addr);
  void resetCellByAddressForSliding(int addr, const std::vector<char>& clear_mask);
  void hashIdToGlobalIndex(int addr, Eigen::Vector3i& id_g) const;
  void applyOccupancyUpdate(const Eigen::Vector3i& id, double new_log_odds);
  void rebuildInflationOffsets();
  void updateInflation(const Eigen::Vector3i& id, int delta, const std::vector<char>* ignore_mask = nullptr);
  void updateInflationLayer(const Eigen::Vector3i& id, int delta,
                            const vector<Eigen::Vector3i>& offsets,
                            std::vector<int>& cnt_buffer,
                            std::vector<char>& flag_buffer,
                            const std::vector<char>* ignore_mask);

  // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // nav_msgs::Odometry> SyncPolicyImageOdom; typedef
  // message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // nav_msgs::Odometry> SyncPolicyImagePose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, nav_msgs::msg::Odometry>
      SyncPolicyImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
  // Pair live LiDAR and its map-frame pose at the identical original scan stamp.
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>
      SyncPolicyCloudPose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudPose>> SynchronizerCloudPose;

  bool urdf_footprint_enabled_{false},footprint_valid_{false};
  int64_t footprint_stamp_ns_{0};
  double footprint_max_age_s_{3.};
  std::vector<Eigen::Vector2d> footprint_polygon_;
  double footprint_cached_yaw_{1e30};
  std::vector<Eigen::Vector2i> footprint_cached_offsets_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;
  rclcpp::Node* node_{nullptr};
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> depth_pose_sub_;
  SynchronizerImagePose sync_image_pose_;

  shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> lidar_pose_sub_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> cloud_sub_;
  SynchronizerCloudPose sync_cloud_pose_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sliding_map_frame_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr static_cloud_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr static_ready_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr static_status_pub_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr static_source_cloud_;
  pcl::PointCloud<pcl::PointXYZ> static_source_decoded_;
  pcl::PointCloud<pcl::PointXYZ> effective_static_source_;
  plan_env_detail::StaticFreeEvidence static_free_evidence_;
  plan_env_detail::WorldChanges world_changes_;
  std::vector<char> static_prior_mask_;
  bool static_observation_refresh_{false}, static_refresh_pending_{false}, static_restore_urgent_{false};
  bool static_prior_as_cost_{false};
  bool unknown_as_obstacle_{true};
  bool extend_reference_to_goal_{false};
  bool route_map_changed_{false};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr predicted_sub_;
  std::vector<Eigen::Vector3d> predicted_points_;
  int64_t prediction_stamp_ns_{0};
  bool predictionFresh() const {
    double age=(node_->now().nanoseconds()-prediction_stamp_ns_)*1e-9;
    return prediction_stamp_ns_>0 && age>=-.1 && age<=.6;
  }
  bool require_observed_space_{false};
  double observation_ttl_s_{3.};
  int64_t latest_scan_stamp_ns_{0};
  std::unordered_map<int,double> observation_times_;
  std::unordered_map<uint64_t,double> observed_columns_;
  double observation_now_s_{0.};
  void expireObservations();
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr observed_free_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ground_reference_sub_;
  bool free_evidence_frame_valid_{false};
  int64_t free_evidence_stamp_{0}, last_free_evidence_stamp_{0};
  double last_static_rebuild_s_{0};
  Eigen::Vector3d static_source_min_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d static_source_max_{Eigen::Vector3d::Zero()};
  std::vector<char> static_occupancy_buffer_;
  std::vector<char> static_inflate_buffer_;
  std::vector<plan_env_detail::StaticInflationXYOffset> static_inflate_xy_offsets_;
  bool static_cloud_enabled_{false};
  bool reference_updates_enabled_{false};
  bool static_source_valid_{false};
  bool static_window_intersects_source_bounds_{false};
  int static_cloud_max_points_{1000000};
  int static_inflate_z_up_steps_{0};
  int static_inflate_z_down_steps_{0};
  std::string static_cloud_topic_;
  size_t static_source_points_{0};
  size_t static_window_points_{0};
  size_t static_occupied_voxels_{0};
  double static_overlay_rebuild_ms_{0.0};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_inf_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr environment_inf_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sliding_map_bbox_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr extrinsic_pose_pub_;
  rclcpp::TimerBase::SharedPtr occ_timer_, vis_timer_;

  //
  uniform_real_distribution<double> rand_noise_;
  normal_distribution<double> rand_noise2_;
  default_random_engine eng_;
  void publishPlanningView();
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fused_view_pub_,risk_view_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr reference_grid_pub_, live_grid_pub_;
  double last_planning_view_s_{-1.};
  std::vector<Eigen::Vector3i> static_window_prior_cells_;
  std::unordered_map<int,int64_t> prior_observation_since_;
  int64_t last_prior_scan_stamp_ns_{0},processed_scan_stamp_ns_{0};
  void ageStaticPriors();
};

/* ============================== definition of inline function
 * ============================== */

inline int GridMap::toAddress(const Eigen::Vector3i& id) {
  return getLocalIndex(id(0), 0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
         getLocalIndex(id(1), 1) * mp_.map_voxel_num_(2) + getLocalIndex(id(2), 2);
}

inline int GridMap::toAddress(int& x, int& y, int& z) {
  return getLocalIndex(x, 0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
         getLocalIndex(y, 1) * mp_.map_voxel_num_(2) + getLocalIndex(z, 2);
}

inline int GridMap::getLocalIndex(int id, int dim) const {
  int local_id = id % mp_.map_voxel_num_(dim);
  if (local_id < 0) local_id += mp_.map_voxel_num_(dim);
  return local_id;
}

inline int GridMap::toAddressLocal(const Eigen::Vector3i& id_l) const {
  return id_l(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + id_l(1) * mp_.map_voxel_num_(2) + id_l(2);
}

inline int GridMap::toAddressLocal(int x, int y, int z) const {
  return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
}

inline void GridMap::boundIndex(Eigen::Vector3i& id) {
  Eigen::Vector3i id1;
  id1(0) = max(min(id(0), mp_.map_bound_max_idx_(0)), mp_.map_bound_min_idx_(0));
  id1(1) = max(min(id(1), mp_.map_bound_max_idx_(1)), mp_.map_bound_min_idx_(1));
  id1(2) = max(min(id(2), mp_.map_bound_max_idx_(2)), mp_.map_bound_min_idx_(2));
  id = id1;
}

inline bool GridMap::isUnknown(const Eigen::Vector3i& id) {
  if(!isInMap(id))return true;
  Eigen::Vector3i id1 = id;
  const double value=md_.occupancy_buffer_[toAddress(id1)];
  return (require_observed_space_ && !observationFresh(toAddress(id1))) || value<mp_.clamp_min_log_-1e-3 ||
      (value>std::min(-.01,mp_.prob_miss_log_) && value<mp_.min_occupancy_log_);
}

inline bool GridMap::isUnknown(const Eigen::Vector3d& pos) {
  Eigen::Vector3i idc;
  posToIndex(pos, idc);
  return isUnknown(idc);
}

inline bool GridMap::isKnownFree(const Eigen::Vector3i& id) {
  if(!isInMap(id))return false;
  const int adr=toAddress(id);
  return md_.occupancy_buffer_[adr]>=mp_.clamp_min_log_ &&
         md_.occupancy_buffer_[adr]<=std::min(-.01,mp_.prob_miss_log_) &&
         md_.occupancy_buffer_inflate_[adr]==0 &&
         (!static_cloud_enabled_ || static_prior_as_cost_ || static_inflate_buffer_.empty() || static_inflate_buffer_[adr]==0);
}

inline bool GridMap::isKnownOccupied(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  return md_.occupancy_buffer_inflate_[adr] == 1 ||
         (static_cloud_enabled_ && !static_prior_as_cost_ && !static_inflate_buffer_.empty() &&
          static_inflate_buffer_[adr] == 1);
}

inline void GridMap::setOccupied(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  applyOccupancyUpdate(id, mp_.clamp_max_log_);
}

inline void GridMap::setOccupancy(Eigen::Vector3d pos, double occ) {
  if (occ != 1 && occ != 0) {
    cout << "occ value error!" << endl;
    return;
  }

  if (!isInMap(pos)) return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  applyOccupancyUpdate(id, occ > 0.5 ? mp_.clamp_max_log_ : mp_.clamp_min_log_);
}

inline int GridMap::getOccupancy(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  const int addr = toAddress(id);
  return (md_.occupancy_buffer_[addr] > mp_.min_occupancy_log_ ||
          (static_cloud_enabled_ && !static_prior_as_cost_ && !static_occupancy_buffer_.empty() &&
           static_occupancy_buffer_[addr] == 1)) ? 1 : 0;
}

inline int GridMap::getInflateOccupancy(Eigen::Vector3d pos, double yaw, bool allow_unknown) {
  // Hard occupancy and observation certainty are separate decisions. Real
  // obstacles always block; an unobserved cell only blocks when the caller
  // asks for strict observation (execution/admission) or when the configured
  // planning policy still treats unknown as an obstacle. Planning may relax
  // unknown to a cost/speed influence while execution keeps the strict gate.
  if (mp_.fixed_body_heading_) yaw=md_.body_yaw_;
  const bool check_observed=require_observed_space_ && !allow_unknown;
  bool unobserved=false;
  const auto sample=[&](const Eigen::Vector3d& point)->int {
    const int live=getInflateOccupancyFromBuffer(point,md_.occupancy_buffer_inflate_);
    if(live!=0)return live;
    if(static_cloud_enabled_ && !static_prior_as_cost_ && !static_inflate_buffer_.empty()) {
      const int stat=getInflateOccupancyFromBuffer(point,static_inflate_buffer_);
      if(stat!=0)return stat;
    }
    if(check_observed && !observedFree(point,&pos))unobserved=true;
    return 0;
  };
  if(urdf_footprint_enabled_) {
    if(!footprintReady())return -1;
    if(std::abs(yaw-footprint_cached_yaw_)>1e-9 || footprint_cached_offsets_.empty()) {
      footprint_cached_offsets_=plan_env_detail::polygonOffsets(footprintPolygon(yaw),mp_.environment_margin_,mp_.resolution_);
      footprint_cached_yaw_=yaw;
    }
    for(const auto &offset:footprint_cached_offsets_) {
      const Eigen::Vector3d point=pos+Eigen::Vector3d(offset.x()*mp_.resolution_,offset.y()*mp_.resolution_,0.);
      const int occupied=sample(point);
      if(occupied!=0)return occupied;
    }
  } else {
    const Eigen::Vector3d heading(std::cos(yaw), std::sin(yaw), 0.0);
    for (double x : mp_.footprint_centers_x_) {
      const int occupied=sample(pos + x*heading);
      if (occupied != 0) return occupied;
    }
  }
  if(unobserved)return unknown_as_obstacle_ ? -1 : 0;
  return 0;
}

inline int GridMap::getInflateOccupancyFromBuffer(Eigen::Vector3d pos, const std::vector<char>& buffer) {
  if (!isInMap(pos)) return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);


  return int(buffer[toAddress(id)]);
}

inline int GridMap::getOccupancy(Eigen::Vector3i id) {
  if (!isInMap(id))
    return -1;

  const int addr = toAddress(id);
  return (md_.occupancy_buffer_[addr] > mp_.min_occupancy_log_ ||
          (static_cloud_enabled_ && !static_prior_as_cost_ && !static_occupancy_buffer_.empty() &&
           static_occupancy_buffer_[addr] == 1)) ? 1 : 0;
}

inline bool GridMap::isInMap(const Eigen::Vector3d& pos) {
  if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
      pos(2) < mp_.map_min_boundary_(2) + 1e-4) {
    // cout << "less than min range!" << endl;
    return false;
  }
  if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
      pos(2) > mp_.map_max_boundary_(2) - 1e-4) {
    return false;
  }
  return true;
}

inline bool GridMap::isInMap(const Eigen::Vector3i& idx) {
  if (idx(0) < mp_.map_bound_min_idx_(0) || idx(1) < mp_.map_bound_min_idx_(1) ||
      idx(2) < mp_.map_bound_min_idx_(2)) {
    return false;
  }
  if (idx(0) > mp_.map_bound_max_idx_(0) || idx(1) > mp_.map_bound_max_idx_(1) ||
      idx(2) > mp_.map_bound_max_idx_(2)) {
    return false;
  }
  return true;
}

inline void GridMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i) id(i) = floor(pos(i) * mp_.resolution_inv_);
}

inline void GridMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
  for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * mp_.resolution_;
}

inline void GridMap::inflatePoint(const Eigen::Vector3i& pt, int inf_step_xy, int inf_step_z_up, int inf_step_z_down, vector<Eigen::Vector3i>& pts) {

  /* ---------- + shape inflate ---------- */
  // for (int x = -step; x <= step; ++x)
  // {
  //   if (x == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
  // }
  // for (int y = -step; y <= step; ++y)
  // {
  //   if (y == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
  // }
  // for (int z = -1; z <= 1; ++z)
  // {
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
  // }

  /* ---------- all inflate ---------- */
  pts.clear();
  for (int x = -inf_step_xy; x <= inf_step_xy; ++x)
    for (int y = -inf_step_xy; y <= inf_step_xy; ++y)
    {
      if (std::sqrt(x * x + y * y) > inf_step_xy)
        continue;

      for (int z = -inf_step_z_down; z <= inf_step_z_up; ++z) {
        pts.push_back(Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z));
      }
    }
}

inline double GridMap::getResolution() { return mp_.resolution_; }

#endif
