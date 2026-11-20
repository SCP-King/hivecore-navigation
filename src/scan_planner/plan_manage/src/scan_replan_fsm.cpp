
#include <nlohmann/json.hpp>

#include <plan_manage/scan_replan_fsm.h>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace
{
  template <typename T>
  T load_parameter(rclcpp::Node *node, const std::string &name, const T &default_value)
  {
    if (!node->has_parameter(name)) node->declare_parameter<T>(name, default_value);
    return node->get_parameter(name).get_value<T>();
  }

  std::string faultDomain(const std::string &reason)
  {
    const auto has=[&](const char* key){return reason.find(key)!=std::string::npos;};
    if(reason.empty())return "unknown";
    if(reason.find("not_ready")==std::string::npos &&
       reason.find("missing")==std::string::npos &&
       reason.find("stale")==std::string::npos &&
       (reason.find("_ready")!=std::string::npos ||
        reason.find("_available")!=std::string::npos ||
        reason=="ready" || reason=="scan_computing"))return "none";
    if(has("localization")||has("robot_pose")||has("pose_scan")||has("body_pose")||
       has("odom_")||has("reloc")||has("anchor")||has("scan_motion_compensation")||
       has("control_pose")||has("tf_"))return "localization";
    if(has("estop")||has("emergency")||has("manual_override")||has("wheel_")||
       has("chassis_twist")||has("native_ik")||has("native_translation")||
       has("steering")||has("cmd_"))return "chassis";
    if(has("ground_")||has("scene_not_ready")||has("occupancy_stale")||has("live_")||
       has("point_cloud")||has("cloud_")||has("sensor_"))return "perception";
    if(has("urdf")||has("footprint_missing")||has("invalid_global_inputs")||
       has("packet_geometry")||has("geometry_or_scene_size"))return "config";
    if(has("goal_invalid")||has("user_stop")||has("manual_stop"))return "operator";
    if(has("packet_")||has("scene_or_admission")||has("corrected_command")||
       has("feedback_")||has("waiting_for_replan")||has("trajectory_")||
       has("execution_")||has("arrival")||has("no_safe_command")||
       has("no_verified_initial"))return "execution";
    if(has("global_")||has("scan_")||has("local_")||has("optimizer")||
       has("frontier")||has("reference_")||has("planning")||has("budget")||
       has("route"))return "planning";
    if(has("goal_")||has("start_")||has("map_")||has("static_")||has("occupied")||
       has("no_route")||has("reference_unavailable"))return "map";
    return "unknown";
  }
  std::string faultAction(const std::string &domain)
  {
    if(domain=="map")return "检查目标/起点占用与地图版本；更新PCD或重选目标";
    if(domain=="localization")return "检查定位tracking/TF/时间戳；重定位后再试";
    if(domain=="perception")return "检查/cloud、地面与场景时效";
    if(domain=="planning")return "检查窄通道/搜索预算并重规划";
    if(domain=="execution")return "检查轨迹包/场景/反馈；等重规划或人工挪车";
    if(domain=="chassis")return "释放急停/手动接管；检查轮组与底盘";
    if(domain=="config")return "检查URDF/包络/参数配置";
    if(domain=="none")return "无需动作";
    if(domain=="operator")return "确认停止/取消/改目标";
    return "查日志与话题，不自动重试";
  }

} // namespace

namespace scan_planner
{

  void SCANReplanFSM::init(rclcpp::Node *node)
  {
    node_ = node;
    planning_status_pub_=node_->create_publisher<std_msgs::msg::String>("/planning/status",rclcpp::QoS(1).reliable().transient_local());
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    trigger_ = false;
    have_target_ = false;
    have_odom_ = false;
    have_new_target_ = false;
    rviz_height_ready_ = false;
    navigation_replan_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/n01/navigation/replan", 10, [this](std_msgs::msg::Bool::ConstSharedPtr p){
        if(p->data && have_target_ && exec_state_ == EXEC_TRAJ) changeFSMExecState(REPLAN_TRAJ,"CONTROLLER_PROGRESS");
      });
    execution_progress_sub_ = node_->create_subscription<scan_planner_msgs::msg::ExecutionProgress>(
      "/n01/navigation/progress", 10, [this](scan_planner_msgs::msg::ExecutionProgress::ConstSharedPtr p) {
        if (p->goal_id == navigation_goal_id_) execution_progress_ = p;
      });
    flag_escape_emergency_ = true;
    need_hover_stop_ = false;
    replan_fail_count_ = 0;
    resetRecovery();
    goal_reached_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
        "/n01/navigation/goal_reached",10,
        [this](const geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
          if (!have_target_ || !have_odom_ || msg->header.frame_id!="map") return;
          const Eigen::Vector3d goal(msg->point.x,msg->point.y,msg->point.z);
          const double age=(node_->now()-rclcpp::Time(msg->header.stamp)).seconds();
          if (!goal.allFinite() || (goal-end_pt_).norm()>.02 ||
              (odom_pos_-end_pt_).head<2>().norm()>.12 || age<-.2 || age>1.) return;
          have_target_=false;have_new_target_=false;replan_fail_count_=0;
          changeFSMExecState(WAIT_TARGET,"EXECUTOR_CONFIRMED_ARRIVAL");
        });

    /*  fsm param  */
    navi_mode_ = load_parameter<int>(node_, "fsm.navi_mode", -1);
    replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_replan", -1.0);
    max_replan_interval_s_ = load_parameter<double>(node_, "fsm.max_replan_interval_s", 0.5);
    max_replan_interval_s_ = std::max(0.2, max_replan_interval_s_);
    // Recovery limits are read live so the GUI can tune them at runtime.
    load_parameter<int>(node_,"recovery.local_attempts",15);
    load_parameter<int>(node_,"recovery.global_attempts",5);
    load_parameter<double>(node_,"recovery.wait_timeout_s",0.);
    load_parameter<bool>(node_,"recovery.auto_resume",true);
    recovery_resume_sub_=node_->create_subscription<std_msgs::msg::Bool>(
      "/n01/navigation/resume",10,[this](std_msgs::msg::Bool::ConstSharedPtr p){
        if(!p->data || !have_target_ || recovery_ladder_.stage!=navigation::RecoveryLadder::WAIT)return;
        resetRecovery();need_hover_stop_=false;
        RCLCPP_INFO(node_->get_logger(),"Recovery resumed by operator");
        changeFSMExecState(GEN_NEW_TRAJ,"RECOVERY_RESUME");
      });
    no_replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_no_replan", -1.0);
    planning_horizon_ = load_parameter<double>(node_, "fsm.planning_horizon", -1.0);
    emergency_time_ = load_parameter<double>(node_, "fsm.emergency_time", 1.0);
    enable_fail_safe_ = load_parameter<bool>(node_, "fsm.fail_safe", true);
    self_inflation_z_up_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_up", 0.0);
    self_inflation_z_down_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_down", 0.0);
    self_double_cylinder_radius_ = load_parameter<double>(node_, "grid_map.double_cylinder_radius", 0.0);
    self_double_cylinder_offset_ = load_parameter<double>(node_, "grid_map.double_cylinder_offset", 0.0);
    self_footprint_centers_x_ = load_parameter<std::vector<double>>(node_, "grid_map.footprint_centers_x", std::vector<double>{});
    if (self_footprint_centers_x_.empty())
      self_footprint_centers_x_ = {-self_double_cylinder_offset_, self_double_cylinder_offset_};
    body_height_ = load_parameter<double>(node_, "grid_map.body_height", 0.4);
    self_inflation_frame_id_ = load_parameter<std::string>(node_, "grid_map.frame_id", "world");

    if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      const auto flat_waypoints = load_parameter<std::vector<double>>(node_, "fsm.waypoints", {});
      if (flat_waypoints.empty() || flat_waypoints.size() % 3 != 0)
        throw std::runtime_error("navi_mode=2 requires non-empty fsm.waypoints with x,y,z triples");
      waypoint_num_ = static_cast<int>(flat_waypoints.size() / 3);
      preset_waypoints_.resize(waypoint_num_);
      for (int i = 0; i < waypoint_num_; i++)
      {
        preset_waypoints_[i] = Eigen::Vector3d(flat_waypoints[3 * i], flat_waypoints[3 * i + 1],
                                               flat_waypoints[3 * i + 2]);
      }
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(node_));
    planner_manager_.reset(new SCANPlannerManager);
    planner_manager_->initPlanModules(node_, visualization_);

    /* callback */
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&SCANReplanFSM::execFSMCallback, this));
    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             std::bind(&SCANReplanFSM::checkCollisionCallback, this));
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&SCANReplanFSM::odometryCallback, this, std::placeholders::_1));
    bspline_pub_ = node_->create_publisher<scan_planner_msgs::msg::Bspline>("planning/bspline", 10);
    global_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
        "planning/global_path", rclcpp::QoS(1).reliable().transient_local());
    data_disp_pub_ = node_->create_publisher<scan_planner_msgs::msg::DataDisp>("planning/data_display", 100);
    self_inflation_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "self_inflation", rclcpp::QoS(1).reliable().transient_local());

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET)
      goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "move_base_simple/goal", 1,
          std::bind(&SCANReplanFSM::rvizGoalCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
      path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
          "initial_path", 1, std::bind(&SCANReplanFSM::pathCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
      RCLCPP_INFO(node_->get_logger(), "Preset waypoint mode will start after the first odometry message");
    else
      throw std::runtime_error("fsm.navi_mode must be 1, 2, or 3");
  }

  void SCANReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps = preset_waypoints_;

    for (size_t i = 0; i < wps.size(); i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    active_waypoints_ = wps;
    current_wp_ = 0;
    trigger_ = true;
    init_pt_ = odom_pos_;

    if (planNextWaypoint())
    {
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory to first preset waypoint");
    }
  }

  void SCANReplanFSM::rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
  {
    if (!msg)
      return;

    if (!rviz_height_ready_)
    {
      RCLCPP_WARN(node_->get_logger(), "Ignore RViz goal before receiving initial body pose");
      return;
    }

    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header = msg->header;
    path->poses.push_back(*msg);
    waypointCallback(path);
  }

  void SCANReplanFSM::waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Empty waypoint message; ignoring");
      return;
    }

    const auto& position=msg->poses[0].pose.position;
    // Map origin is established by SLAM, not by ground height. Negative map
    // coordinates are valid; only non-finite payloads are invalid.
    if(!std::isfinite(position.x)||!std::isfinite(position.y)||!std::isfinite(position.z))return;

    const uint64_t incoming_goal = static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL + msg->header.stamp.nanosec;
    if (incoming_goal == 0 || incoming_goal <= navigation_goal_id_) return;
    navigation_goal_id_ = incoming_goal;
    clearGlobalPath();
    resetRecovery();

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, rviz_goal_height_;
    const auto &q=msg->poses[0].pose.orientation;
    end_yaw_=std::atan2(2.0*(q.w*q.z+q.x*q.y),1.0-2.0*(q.y*q.y+q.z*q.z));
    if(planner_manager_->grid_map_ &&
       planner_manager_->grid_map_->getInflateOccupancy(end_pt_,end_yaw_,false,false)>0){
      planner_manager_->last_global_error_="goal_footprint_occupied";
      publishPlanningStatus("GLOBAL_FAILED","goal_footprint_occupied");
      return;
    }
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
      success = adjustGlobalTargetIfOccupied();

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);
    if(!success)publishPlanningStatus("GLOBAL_FAILED",planner_manager_->last_global_error_.empty()?"goal_collision_or_unavailable":planner_manager_->last_global_error_);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;
      publishGlobalPath();

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory");
    }
  }

  bool SCANReplanFSM::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints)
  {
    if (waypoints.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "No waypoint supplied for global trajectory");
      return false;
    }

    end_pt_ = waypoints.back();

    for (size_t i = 0; i < waypoints.size(); i++)
    {
      visualization_->displayGoalPoint(waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    bool success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_,
        odom_vel_,
        Eigen::Vector3d::Zero(),
        waypoints,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from waypoints");
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    publishGlobalPath();
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, static_cast<int>(waypoints.size()) - 1);

    return true;
  }

  bool SCANReplanFSM::planNextWaypoint()
  {
    if (current_wp_ < 0 || current_wp_ >= (int)active_waypoints_.size())
    {
      RCLCPP_WARN(node_->get_logger(), "[navi_mode=%d] No active waypoint to plan", navi_mode_);
      return false;
    }

    end_pt_ = active_waypoints_[current_wp_];
    setStartStateFromOdomOrCurrentTraj();

    bool success = planner_manager_->planGlobalTraj(
        start_pt_,
        start_vel_,
        start_acc_,
        end_pt_,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "[navi_mode=%d] Unable to generate trajectory to waypoint %d",
                   navi_mode_, current_wp_ + 1);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    publishGlobalPath();
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, current_wp_);
    RCLCPP_INFO(node_->get_logger(), "[navi_mode=%d] Planning to waypoint %d/%zu: [%.2f, %.2f, %.2f]",
                navi_mode_, current_wp_ + 1, active_waypoints_.size(), end_pt_(0), end_pt_(1), end_pt_(2));

    return true;
  }

  bool SCANReplanFSM::isWaypointSequenceMode() const
  {
    return navi_mode_ == NAVI_MODE::PRESET_TARGET;
  }

  bool SCANReplanFSM::adjustGlobalTargetIfOccupied()
  {
    auto map = planner_manager_->grid_map_;
    auto &global_data = planner_manager_->global_data_;
    const double duration = global_data.global_duration_;
    if (!map || duration < 1e-3)
      return true;

    constexpr double sample_dt = 0.05;
    const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
    const Eigen::Vector3d final_pt = global_data.global_traj_.evaluate(duration);
    const Eigen::Vector3d final_prev = global_data.global_traj_.evaluate(duration * (sample_num - 1) / sample_num);
    const int final_occ = map->getInflateOccupancy(final_pt, end_yaw_, false, false);
    if (final_occ <= 0)
      return true;

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET)
    {
      // A temporary occupied goal is not permission to silently move the
      // requested destination backwards along the route on each replan.
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
          "Requested goal is occupied; preserving destination and refusing this plan");
      return false;
    }

    for (int i = sample_num; i >= 0; --i)
    {
      const double t = duration * i / sample_num;
      const double prev_t = duration * std::max(0, i - 1) / sample_num;
      const Eigen::Vector3d pt = global_data.global_traj_.evaluate(t);
      const Eigen::Vector3d prev_pt = global_data.global_traj_.evaluate(prev_t);

      if (map->getInflateOccupancy(pt, estimateYawFromSegment(prev_pt, pt)) == 0)
      {
        const Eigen::Vector3d raw_end = end_pt_;
        end_pt_ = pt;
        global_data.global_duration_ = t;
        global_data.last_progress_time_ = std::min(global_data.last_progress_time_, t);
        RCLCPP_WARN(node_->get_logger(),
                    "Target [%.2f, %.2f, %.2f] is occupied; using [%.2f, %.2f, %.2f]",
                    raw_end(0), raw_end(1), raw_end(2), end_pt_(0), end_pt_(1), end_pt_(2));
        return true;
      }
    }

    RCLCPP_ERROR(node_->get_logger(),
                 "Target is occupied and no collision-free point was found on the global trajectory");
    return false;
  }

  void SCANReplanFSM::pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Received empty initial_path; ignoring");
      return;
    }

    trigger_ = true;

    std::vector<Eigen::Vector3d> waypoints;
    waypoints.reserve(msg->poses.size());

    for (const auto& pose_stamped : msg->poses)
    {
      Eigen::Vector3d wp;
      wp(0) = pose_stamped.pose.position.x;
      wp(1) = pose_stamped.pose.position.y;
      wp(2) = pose_stamped.pose.position.z + body_height_; // Adjust for body height
      waypoints.push_back(wp);
    }
    bool success = planGlobalTrajByWaypoints(waypoints);

    if (success)
    {
      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }
      else if (exec_state_ == EXEC_TRAJ)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      RCLCPP_INFO(node_->get_logger(), "Reference path accepted");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from reference path");
    }
  }

  void SCANReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_)
    {
      rviz_goal_height_ = odom_pos_(2);
      rviz_height_ready_ = true;
      RCLCPP_INFO(node_->get_logger(), "Set RViz goal height from initial body_pose z: %.3f", rviz_goal_height_);
    }

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
    publishSelfInflationMarker();
    if (navi_mode_ == NAVI_MODE::PRESET_TARGET && !preset_started_)
    {
      preset_started_ = true;
      planGlobalTrajbyGivenWps();
    }
  }

  double SCANReplanFSM::executionParameter() const
  {
    const auto &info = planner_manager_->local_data_;
    if (execution_progress_ && execution_progress_->goal_id == navigation_goal_id_ &&
        execution_progress_->revision == info.traj_id_ &&
        (node_->now() - rclcpp::Time(execution_progress_->header.stamp)).seconds() <= .6)
      return std::clamp(execution_progress_->parameter_time, 0.0, info.duration_);
    // Before execution / after revision: measure actual geometric progress,
    // not elapsed wall time. Waiting at start cannot consume the trajectory.
    double best = 1e100, parameter = 0.;
    for (int i=0; i<=100; ++i) {
      const double t = info.duration_*i/100.;
      const double d = (info.position_traj_.evaluateDeBoorT(t)-odom_pos_).squaredNorm();
      if (d < best) { best=d; parameter=t; }
    }
    return parameter;
  }

  double SCANReplanFSM::getOdomYaw() const
  {
    Eigen::Vector3d heading = odom_orient_.toRotationMatrix().col(0);
    if (heading.head<2>().squaredNorm() < 1e-8)
      return 0.0;
    return std::atan2(heading(1), heading(0));
  }

  double SCANReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    if (planner_manager_->grid_map_->usesFixedBodyHeading()) return getOdomYaw();
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    if (diff.squaredNorm() < 1e-8)
      return getOdomYaw();
    return std::atan2(diff(1), diff(0));
  }

  void SCANReplanFSM::publishSelfInflationMarker()
  {
    const auto polygon=planner_manager_->grid_map_->getFootprintPolygon();
    if(!polygon.empty()) {
      visualization_msgs::msg::Marker wire;
      wire.header.frame_id=self_inflation_frame_id_;wire.header.stamp=node_->now();
      wire.ns="self_inflation";wire.id=0;wire.type=visualization_msgs::msg::Marker::LINE_LIST;
      wire.action=visualization_msgs::msg::Marker::ADD;wire.pose.orientation.w=1.;
      wire.scale.x=.012;wire.color.r=.72;wire.color.g=.40;wire.color.b=1.;wire.color.a=.85;
      wire.lifetime=rclcpp::Duration::from_seconds(.25);
      const auto point=[&](const Eigen::Vector2d &p,double z) {
        geometry_msgs::msg::Point q;q.x=odom_pos_.x()+p.x();q.y=odom_pos_.y()+p.y();q.z=z;return q;
      };
      for(size_t i=0;i<polygon.size();++i) {
        const auto &a=polygon[i];const auto &b=polygon[(i+1)%polygon.size()];
        for(double z:{odom_pos_.z()-self_inflation_z_up_,odom_pos_.z()+self_inflation_z_down_}) {
          wire.points.push_back(point(a,z));wire.points.push_back(point(b,z));
        }
        wire.points.push_back(point(a,odom_pos_.z()-self_inflation_z_up_));
        wire.points.push_back(point(a,odom_pos_.z()+self_inflation_z_down_));
      }
      self_inflation_pub_->publish(wire);return;
    }
    const double radius = std::max(0.0, self_double_cylinder_radius_);
    const double z_up = std::max(0.0, self_inflation_z_up_);
    const double z_down = std::max(0.0, self_inflation_z_down_);
    const double height = std::max(1e-3, z_up + z_down);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "self_inflation";
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = height;
    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.4;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    Eigen::Vector3d center = odom_pos_;
    // Obstacle inflation is the reflected robot extent.
    center(2) += 0.5 * (z_down - z_up);

    Eigen::Vector3d heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()), 0.0);
    for (size_t i=0; i<self_footprint_centers_x_.size(); ++i) {
      Eigen::Vector3d point=center+self_footprint_centers_x_[i]*heading;
      marker.id=static_cast<int>(i);
      marker.pose.position.x=point.x(); marker.pose.position.y=point.y(); marker.pose.position.z=point.z();
      self_inflation_pub_->publish(marker);
    }
  }

  void SCANReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continuously_called_times_++;
    else
      continuously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> SCANReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);
  }

  void SCANReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  bool SCANReplanFSM::globalRouteBlocked()
  {
    auto map=planner_manager_->grid_map_;
    auto& data=planner_manager_->global_data_;
    const double duration=data.global_duration_;
    if(!(duration>0.))return false;
    const auto lo=map->getLocalMinimum(),hi=map->getLocalMaximum();
    const double step=std::max(.01,map->getResolution()/(2.*planner_manager_->pp_.max_vel_));
    for(double t=data.last_progress_time_;t<duration;t+=step){
      auto p=data.getPosition(t);
      if(p.x()<=lo.x()||p.x()>=hi.x()||p.y()<=lo.y()||p.y()>=hi.y())continue;
      if(map->getInflateOccupancy(p,getOdomYaw(),true)==1)return true;
    }
    return false;
  }

  void SCANReplanFSM::execFSMCallback()
  {
    if(!have_target_ && !global_path_samples_.empty() &&
       planner_manager_->grid_map_->takeRouteMapChange()){
      clearGlobalPath();
      publishPlanningStatus("GLOBAL_STALE","map_changed_without_target");
    }
    if(have_target_ && checkAndRepairDirtyGlobal()){
      if(force_global_refresh_ && exec_state_==EXEC_TRAJ)changeFSMExecState(REPLAN_TRAJ,"DIRTY_SEGMENT_FULL_REPLAN");
    }

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      // Arrival is measured from live odometry, never from spline time.
      // Do not count an already reached target as a planning failure.
      if (navi_mode_ == NAVI_MODE::MANUAL_TARGET &&
          (end_pt_ - odom_pos_).head<2>().norm() <= .08)
      {
        have_target_ = false;
        have_new_target_ = false;
        replan_fail_count_ = 0;
        resetRecovery();
        changeFSMExecState(WAIT_TARGET, "MEASURED_ARRIVAL");
        break;
      }
      // Failed local optimization must not busy-loop at the 100Hz FSM rate.
      // Keep map/pose/goal callbacks responsive; a new target bypasses backoff.
      const auto attempt_time=std::chrono::steady_clock::now();
      if(replan_fail_count_>0 && !have_new_target_ &&
          std::chrono::duration<double>(attempt_time-last_replan_attempt_).count()<navigation::Recovery::retryDelay(replan_fail_count_))break;
      last_replan_attempt_=attempt_time;
      setStartStateFromOdomOrCurrentTraj();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = replan_fail_count_>=2 ? planFromCurrentTraj() : callReboundReplan(true, flag_random_poly_init);
      if (success)
      {

        replan_fail_count_ = 0;
        resetRecovery();
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        replan_fail_count_++;
        noteRecoveryFailure();
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      const auto attempt_time=std::chrono::steady_clock::now();
      if(replan_fail_count_>0 && !have_new_target_ &&
          std::chrono::duration<double>(attempt_time-last_replan_attempt_).count()<navigation::Recovery::retryDelay(replan_fail_count_))break;
      last_replan_attempt_=attempt_time;
      if (planFromCurrentTraj())
      {
        replan_fail_count_ = 0;
        resetRecovery();
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        replan_fail_count_++;
        noteRecoveryFailure();
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      double t_cur = executionParameter();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);
      if (navi_mode_ == NAVI_MODE::MANUAL_TARGET &&
          (end_pt_ - odom_pos_).head<2>().norm() <= .08)
      {
        have_target_ = false;
        replan_fail_count_ = 0;
        resetRecovery();
        changeFSMExecState(WAIT_TARGET, "MEASURED_ARRIVAL");
        return;
      }

      if (isWaypointSequenceMode() &&
          current_wp_ + 1 < (int)active_waypoints_.size() &&
          (end_pt_ - odom_pos_).norm() < 0.5)
      {
        current_wp_++;
        if (planNextWaypoint())
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }
        replan_fail_count_++;
        noteRecoveryFailure();
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        return;
      }

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2)
      {
        // A slow/external executor may lag the nominal spline schedule. A
        // completed time parameter is not evidence of physical goal arrival.
        if (navi_mode_ == NAVI_MODE::MANUAL_TARGET &&
            (end_pt_ - odom_pos_).norm() > no_replan_thresh_)
        {
          RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
              "Trajectory window ended before measured arrival (remaining=%.3f m); replanning from odometry",
              (end_pt_ - odom_pos_).norm());
          have_new_target_ = true;
          changeFSMExecState(GEN_NEW_TRAJ, "MEASURED_PROGRESS");
          return;
        }
        if (isWaypointSequenceMode() && current_wp_ + 1 < (int)active_waypoints_.size())
        {
          current_wp_++;
          if (planNextWaypoint())
          {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            return;
          }
          replan_fail_count_++;
          noteRecoveryFailure();
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }

        if (isWaypointSequenceMode())
        {
          active_waypoints_.clear();
          current_wp_ = 0;
        }

        have_target_ = false;clearGlobalPath();

        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      else if (info->duration_ - t_cur <= max_replan_interval_s_)
      {
        changeFSMExecState(REPLAN_TRAJ, "LIVE_ENVIRONMENT_REFRESH");
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && !need_hover_stop_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        else if (enable_fail_safe_ && need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          if (recovery_ladder_.stage == navigation::RecoveryLadder::WAIT) {
            maybeAutoResume();
            break;
          }
          RCLCPP_INFO(node_->get_logger(),
                      "Stopped safely; retaining target and retrying recovery");
          need_hover_stop_ = false;
          changeFSMExecState(have_target_?GEN_NEW_TRAJ:WAIT_TARGET, "RECOVERY_RETRY");
        }
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = node_->now();
    data_disp_pub_->publish(data_disp_);
  }

  bool SCANReplanFSM::planFromCurrentTraj()
  {
    setStartStateFromOdomOrCurrentTraj();

    // Ordinary local updates retain the reference and its bypass side.
    // A blocked/failed local solution may refresh A* from the measured state.
    if (force_global_refresh_ || replan_fail_count_ >= 2) {
    if (!planner_manager_->planGlobalTraj(
            start_pt_,
            start_vel_,
            start_acc_,
            end_pt_,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero()))
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "[navi_mode=%d] Unable to refresh global trajectory from odom to current target", navi_mode_);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    ++global_replans_;
    publishGlobalPath();
    force_global_refresh_ = false;
    }
    bool success = callReboundReplan(true, false);
    if (!success)
    {
      success = callReboundReplan(true, true);
      if (!success)
        return false;
    }

    return true;
  }

  void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();
    // External wheel execution may stop or lag the nominal spline. Pair the
    // measured start position with measured velocity, never spline-time velocity.
    if (execution_progress_ && execution_progress_->goal_id==navigation_goal_id_ &&
        execution_progress_->phase=="ALIGN" && start_vel_.head<2>().norm()<.01) {
      const double seed=std::min(.03,planner_manager_->pp_.max_vel_);
      start_vel_.x()=seed*std::cos(getOdomYaw());start_vel_.y()=seed*std::sin(getOdomYaw());
    }
    const double measured_speed=start_vel_.norm();
    const double planning_speed_limit=planner_manager_->pp_.max_vel_;
    if (measured_speed>planning_speed_limit && measured_speed>1e-6)
      start_vel_*=planning_speed_limit/measured_speed;
    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0 &&
        !(execution_progress_ && execution_progress_->phase=="ALIGN"))
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }
  }

  void SCANReplanFSM::checkCollisionCallback()
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.seconds() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = executionParameter();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t);
      Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(std::min(t + time_step, info->duration_));
      if (map->getInflateOccupancy(pos, estimateYawFromSegment(pos, pos_next)))
      {
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            RCLCPP_WARN(node_->get_logger(), "Obstacle discovered; emergency stop in %.3fs", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    ++local_attempts_;
    if(!getLocalTarget())return false;
    publishPlanningStatus("LOCAL_PLANNING","scan_computing");
    // An obstructed ALIGN may need a non-collinear seed. Use SCAN's own
    // optimizer random initialization, never another downstream search.
    if(execution_progress_ && execution_progress_->goal_id==navigation_goal_id_ &&
       execution_progress_->phase=="ALIGN") flag_randomPolyTraj=true;

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    if(!plan_success) {
      auto blocker=planner_manager_->grid_map_->envelopeBlocker(start_pt_,getOdomYaw());
      if(!blocker.code.empty()) {
        char detail[160];
        std::snprintf(detail,sizeof(detail),"start_envelope_%s@(%.2f,%.2f,%.2f)",
                      blocker.code.c_str(),blocker.x,blocker.y,blocker.z);
        planner_manager_->last_local_error_=detail;
        RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),2000,
            "Local plan blocked inside the robot envelope: %s",detail);
      }
    }
    publishPlanningStatus(plan_success?"LOCAL_READY":"LOCAL_FAILED",plan_success?"scan_trajectory_available":
        (planner_manager_->last_local_error_.empty()?"scan_no_solution":planner_manager_->last_local_error_));

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {

      auto info = &planner_manager_->local_data_;

      /* publish traj */
      scan_planner_msgs::msg::Bspline bspline;
      bspline.order = 3;
      bspline.goal_id = navigation_goal_id_;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      bspline_pub_->publish(bspline);

      visualization_->displayOptimalTraj(info->position_traj_, 0);
    }

    return plan_success;
  }

  bool SCANReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    scan_planner_msgs::msg::Bspline bspline;
    bspline.order = 3;
    bspline.goal_id = navigation_goal_id_;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_->publish(bspline);

    return true;
  }

  int SCANReplanFSM::recoveryLimit(const char *name, int fallback)
  {
    const int value = node_->get_parameter(name).as_int();
    return value >= 1 && value <= 50 ? value : fallback;
  }

  void SCANReplanFSM::resetRecovery()
  {
    recovery_ladder_.reset();
    recovery_wait_since_ = 0.;
  }

  void SCANReplanFSM::noteRecoveryFailure()
  {
    if (recovery_ladder_.stage == navigation::RecoveryLadder::WAIT) return;
    // Starting up or losing the scene is a data problem, not a planning failure.
    auto grid_map = planner_manager_->grid_map_;
    const int64_t scene_stamp = grid_map->sceneStamp();
    const double scene_age = scene_stamp > 0
        ? (node_->now().nanoseconds() - scene_stamp) * 1e-9 : -1.;
    if (scene_age < 0. || scene_age > 2.0) {
      recovery_ladder_.reset();
      publishPlanningStatus("WAITING_FOR_SCENE", "scene_not_ready");
      return;
    }
    recovery_ladder_.configure(recoveryLimit("recovery.local_attempts", 15),
                               recoveryLimit("recovery.global_attempts", 5));
    const auto step = recovery_ladder_.noteFailure();
    if (step.stage == navigation::RecoveryLadder::WAIT) {
      recovery_wait_since_ = node_->now().seconds();
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      publishPlanningStatus("WAITING_FOR_CLEARANCE", "goal_unreachable_observed");
      RCLCPP_WARN(node_->get_logger(),
                  "Recovery exhausted (local=%d, global=%d); holding at zero speed with target retained",
                  recovery_ladder_.local_limit, recovery_ladder_.global_limit);
      changeFSMExecState(EMERGENCY_STOP, "RECOVERY_EXHAUSTED");
      return;
    }
    if (step.stage == navigation::RecoveryLadder::GLOBAL) {
      force_global_refresh_ = true;
      publishPlanningStatus("RECOVERY_GLOBAL_REPLAN",
                            recovery_ladder_.global_attempts <= 1 ? "local_detour_exhausted"
                                                                  : "global_replan_retry");
    } else {
      publishPlanningStatus("RECOVERY_LOCAL_DETOUR", "local_detour_retry");
    }
  }

  void SCANReplanFSM::maybeAutoResume()
  {
    if (recovery_ladder_.stage != navigation::RecoveryLadder::WAIT) return;
    if (!node_->get_parameter("recovery.auto_resume").as_bool()) return;
    const double timeout = node_->get_parameter("recovery.wait_timeout_s").as_double();
    if (!(timeout > 0.) || node_->now().seconds() - recovery_wait_since_ < timeout) return;
    resetRecovery();
    need_hover_stop_ = false;
    RCLCPP_INFO(node_->get_logger(), "Recovery wait timeout elapsed; retrying the retained target");
    changeFSMExecState(have_target_ ? GEN_NEW_TRAJ : WAIT_TARGET, "RECOVERY_TIMEOUT");
  }

  void SCANReplanFSM::publishPlanningStatus(const std::string& phase,const std::string& reason) {
    const bool global=phase.rfind("GLOBAL",0)==0;
    auto& state=global?global_phase_:local_phase_;auto& why=global?global_reason_:local_reason_;
    const double now=node_->now().seconds();
    if(state!=phase || why!=reason)planning_phase_since_=now;
    state=phase;why=reason;
    auto map=planner_manager_->grid_map_;
    const bool global_diag = reason.find("global_") != std::string::npos;
    int g_start_cell=-2,g_goal_cell=-2,g_start_comp=0,g_goal_comp=0;
    int g_free=0,g_occ=0,g_unknown=0;
    if(global_diag && map->hasGlobalGrid()){
      g_start_cell=map->globalCellState(start_pt_.x(),start_pt_.y());
      g_goal_cell=map->globalCellState(end_pt_.x(),end_pt_.y());
      g_start_comp=map->globalFreeComponentSize(start_pt_.x(),start_pt_.y());
      g_goal_comp=map->globalFreeComponentSize(end_pt_.x(),end_pt_.y());
      map->globalCounts(g_free,g_occ,g_unknown);
    }
    nlohmann::json data={{"goal_id",navigation_goal_id_},{"phase",phase},{"reason",reason},
      {"fault_domain",faultDomain(reason)},
      {"fault_action",faultAction(faultDomain(reason))},
      {"fault_evidence",{{"stage",phase},{"reason",reason},
                          {"scene_age_s",map->sceneStamp()>0?(node_->now().nanoseconds()-map->sceneStamp())*1e-9:-1.},
                          {"global_start_cell",g_start_cell},{"global_goal_cell",g_goal_cell},
                          {"global_start_component",g_start_comp},{"global_goal_component",g_goal_comp},
                          {"global_free",g_free},{"global_occupied",g_occ},{"global_unknown",g_unknown}}},
      {"global_phase",global_phase_},{"global_reason",global_reason_},{"local_phase",local_phase_},{"local_reason",local_reason_},
      {"stamp_ns",node_->now().nanoseconds()},{"scene_stamp_ns",map->sceneStamp()},
      {"scene_age_s",map->sceneStamp()>0?(node_->now().nanoseconds()-map->sceneStamp())*1e-9:-1.},
      {"stage",global?"global":"scan"},
      {"waiting_since_s",planning_phase_since_},{"local_attempts",local_attempts_},{"global_replans",global_replans_},
      {"local_target",{local_target_pt_.x(),local_target_pt_.y(),local_target_pt_.z()}},
      {"final_goal",{end_pt_.x(),end_pt_.y(),end_pt_.z()}},
      {"recovery",global?"update_goal_or_scene":"retry_on_fused_scene"},
      {"recovery_stage",recovery_ladder_.stage==navigation::RecoveryLadder::WAIT?"wait":(recovery_ladder_.stage==navigation::RecoveryLadder::GLOBAL?"global_replan":"local_detour")},
      {"recovery_local_attempts",recovery_ladder_.local_attempts},
      {"recovery_global_attempts",recovery_ladder_.global_attempts},
      {"recovery_local_limit",recoveryLimit("recovery.local_attempts",15)},
      {"recovery_global_limit",recoveryLimit("recovery.global_attempts",5)},
      {"recovery_wait_s",recovery_ladder_.stage==navigation::RecoveryLadder::WAIT?node_->now().seconds()-recovery_wait_since_:0.}};
    std_msgs::msg::String msg;msg.data=data.dump();planning_status_pub_->publish(msg);
  }

  void SCANReplanFSM::clearGlobalPath()
  {
    global_path_samples_.clear();global_path_versions_.clear();
    nav_msgs::msg::Path empty;
    empty.header.frame_id="map";empty.header.stamp=node_->now();
    global_path_pub_->publish(empty);
  }

  void SCANReplanFSM::refreshGlobalPathSamples()
  {
    global_path_samples_.clear();global_path_versions_.clear();
    auto& gd=planner_manager_->global_data_;
    const double duration=gd.global_duration_;
    if(!(duration>0.)||!planner_manager_->grid_map_)return;
    const double step=std::max(.2,planner_manager_->pp_.global_waypoint_spacing_);
    auto map=planner_manager_->grid_map_;
    for(double t=0.;t<=duration+1e-6;t+=step){
      const auto p=gd.global_traj_.evaluate(std::min(duration,t));
      global_path_samples_.push_back(p);
      global_path_versions_.push_back(map->hasGlobalGrid()?map->globalTileVersion(p.x(),p.y()):0u);
    }
  }

  bool SCANReplanFSM::checkAndRepairDirtyGlobal()
  {
    if(!have_target_||global_path_samples_.size()<4||!planner_manager_->grid_map_)return false;
    const double now=node_->now().seconds();
    if(now-last_dirty_check_s_<0.5)return false;
    last_dirty_check_s_=now;
    auto map=planner_manager_->grid_map_;
    map->refreshGlobalGrid();
    if(!map->hasGlobalGrid())return false;
    int nearest=0;double best=1e9;
    for(size_t i=0;i<global_path_samples_.size();++i){
      const double d=(global_path_samples_[i].head<2>()-odom_pos_.head<2>()).norm();
      if(d<best){best=d;nearest=int(i);}
    }
    int first=-1,last=-1;
    for(size_t i=size_t(nearest);i<global_path_samples_.size();++i){
      if(map->globalTileVersion(global_path_samples_[i].x(),global_path_samples_[i].y())!=global_path_versions_[i]){
        if(first<0)first=int(i);last=int(i);
      }
    }
    if(first<0)return false;
    if(last-first+1>int(global_path_samples_.size()*0.5)){
      force_global_refresh_=true;
      publishPlanningStatus("GLOBAL_REPAIR","dirty_segment_too_large");
      return true;
    }
    for(int expand=0;expand<=2;++expand){
      const int i0=std::max(1,first-1-expand),i1=std::min(int(global_path_samples_.size())-1,last+1+expand);
      if(i1<=i0+1)return false;
      const Eigen::Vector3d A=global_path_samples_[i0],B=global_path_samples_[i1];
      if(!planner_manager_->planGlobalTraj(A,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),
                                           B,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()))continue;
      auto& gd=planner_manager_->global_data_;
      std::vector<Eigen::Vector3d> seg;
      const double step=std::max(.2,planner_manager_->pp_.global_waypoint_spacing_);
      for(double t=0.;t<=gd.global_duration_+1e-6;t+=step)seg.push_back(gd.global_traj_.evaluate(std::min(gd.global_duration_,t)));
      if(seg.size()<2)continue;
      std::vector<Eigen::Vector3d> combined(global_path_samples_.begin(),global_path_samples_.begin()+i0+1);
      combined.insert(combined.end(),seg.begin()+1,seg.end());
      combined.insert(combined.end(),global_path_samples_.begin()+i1,global_path_samples_.end());
      if(!planner_manager_->planGlobalTrajWaypoints(odom_pos_,odom_vel_,Eigen::Vector3d::Zero(),
                                                    combined,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()))continue;
      publishGlobalPath();
      publishPlanningStatus("GLOBAL_REPAIRED","dirty_segment_repaired");
      RCLCPP_INFO(node_->get_logger(),"Global path repaired segment [%d,%d] (%zu samples)",i0,i1,combined.size());
      return true;
    }
    force_global_refresh_=true;
    publishPlanningStatus("GLOBAL_REPAIR","dirty_segment_repair_failed");
    return true;
  }

  void SCANReplanFSM::publishGlobalPath()
  {
    auto &gd = planner_manager_->global_data_;
    const double duration = gd.global_traj_.getTimeSum();
    if (!(duration > 1e-3)) return;
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = node_->now();
    const int samples = std::max(20, std::min(2000, static_cast<int>(duration / 0.05)));
    const double dt = duration / static_cast<double>(samples);
    for (int i = 0; i <= samples; ++i) {
      const double t = std::min(duration, dt * static_cast<double>(i));
      const Eigen::Vector3d p = gd.global_traj_.evaluate(t);
      const Eigen::Vector3d p_next = gd.global_traj_.evaluate(std::min(duration, t + dt));
      const double yaw = std::atan2(p_next.y() - p.y(), p_next.x() - p.x());
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = p.x();
      pose.pose.position.y = p.y();
      pose.pose.position.z = p.z();
      pose.pose.orientation.z = std::sin(yaw * 0.5);
      pose.pose.orientation.w = std::cos(yaw * 0.5);
      path.poses.push_back(pose);
    }
    global_path_pub_->publish(path);
    publishPlanningStatus("GLOBAL_READY","reference_available");
    refreshGlobalPathSamples();
  }

  bool SCANReplanFSM::getLocalTarget()
  {
    double t;

    double t_step = planning_horizon_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    double target_t = planner_manager_->global_data_.global_duration_;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizon_)
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "Local target progress mismatch: distance=%.3f horizon=%.3f progress_time=%.3f",
                     dist, planning_horizon_, planner_manager_->global_data_.last_progress_time_);
        local_target_pt_ = pos_t;
        target_t = t;
        planner_manager_->global_data_.last_progress_time_ = t;
        break;
      }
      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }
      if (dist >= planning_horizon_)
      {
        local_target_pt_ = pos_t;
        target_t = t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t >= planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
      target_t = planner_manager_->global_data_.global_duration_;
    }

    auto targetOccupancy = [&](const Eigen::Vector3d &pt) {
      return planner_manager_->grid_map_->getInflateOccupancy(pt, estimateYawFromSegment(odom_pos_, pt));
    };

    if (planner_manager_->grid_map_->usesRealtimeReference()) {
      const auto map=planner_manager_->grid_map_;
      const auto lo=map->getLocalMinimum(),hi=map->getLocalMaximum();
      // Leave room for the robot and braking instead of selecting a SCAN
      // endpoint outside the rolling map when the ultimate goal is far away.
      double extent=0.;
      for(const auto& vertex:map->getFootprintPolygon())extent=std::max(extent,vertex.norm());
      const double inset=extent+map->getFootprintRadius()+.15;
      const auto inside=[&](const Eigen::Vector3d& p) {
        return p.x()>lo.x()+inset && p.x()<hi.x()-inset &&
               p.y()>lo.y()+inset && p.y()<hi.y()-inset;
      };
      if(!inside(local_target_pt_)) {
        bool found=false;
        for(double tt=std::min(target_t,planner_manager_->global_data_.global_duration_);
            tt>=dist_min_t;tt-=t_step) {
          const auto candidate=planner_manager_->global_data_.getPosition(tt);
          if(inside(candidate) && targetOccupancy(candidate)==0) {
            local_target_pt_=candidate;target_t=tt;found=true;break;
          }
        }
        if(!found) {
          local_target_pt_=start_pt_;local_target_vel_.setZero();
          RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),2000,
              "Realtime goal retained; waiting for a reachable local observation window");
          return false;
        }
      }
    }

    if (targetOccupancy(local_target_pt_) != 0)
    {
      bool found_free_target = false;
      double adjusted_t = target_t;

      for (double dt = 0.0; dt <= planner_manager_->global_data_.global_duration_; dt += t_step)
      {
        double t_forward = target_t + dt;
        if (t_forward <= planner_manager_->global_data_.global_duration_)
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_forward);
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_forward;
            found_free_target = true;
            break;
          }
        }

        double t_backward = target_t - dt;
        if (t_backward >= std::max(0.0, dist_min_t))
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_backward);
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_backward;
            found_free_target = true;
            break;
          }
        }
      }

      if (found_free_target)
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target was adjusted to a nearby collision-free point");
        target_t = adjusted_t;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target is in collision and no nearby free target was found");
      }
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(target_t);
      // cout << "AA" << endl;
    }
    return true;
  }

} // namespace scan_planner
