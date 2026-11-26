// The only production navigation command owner. No path search runs here.
#include <navigation/tracking.hpp>
#include <chassis/chassis_kinematics.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <csignal>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <scan_planner_msgs/msg/navigation_trajectory.hpp>
#include <scan_planner_msgs/msg/execution_progress.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <cstring>
#include <optional>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <iostream>
using Packet=scan_planner_msgs::msg::NavigationTrajectory;
using Progress=scan_planner_msgs::msg::ExecutionProgress;
using Json=nlohmann::json;
using navigation::wrap;
class NavigationController:public rclcpp::Node {
 public:
  NavigationController():Node("n01_navigation_controller"){
    enabled_=declare_parameter("enabled",false);dry_=declare_parameter("dry_run",false);
    task_=declare_parameter<std::string>("task_id","");
    vmax_=declare_parameter("max_speed_mps",.05);wmax_=declare_parameter("max_yaw_rate_radps",.12);
    accel_=declare_parameter("max_accel_mps2",.04);
    final_yaw_=declare_parameter("require_final_yaw",false);follow_updates_=declare_parameter("follow_goal_updates",false);
    final_yaw_tolerance_=declare_parameter("final_yaw_tolerance_rad",0.087266);
    if(!std::isfinite(final_yaw_tolerance_)||final_yaw_tolerance_<=0.||final_yaw_tolerance_>0.5)
      throw std::invalid_argument("invalid final_yaw_tolerance_rad");
    expected_=declare_parameter("require_expected_goal",false);
    expected_x_=declare_parameter("expected_goal_x",0.);expected_y_=declare_parameter("expected_goal_y",0.);
    bool cbf=declare_parameter("cbf_enabled",false);
    if(cbf)throw std::invalid_argument("CBF is not part of the SCAN production chain; disable CBF");
    // Feedback freshness is our own gate, now a declared parameter so it can
    // be tuned at runtime. Defaults keep the previous behaviour.
    const auto fresh_limit=[&](const char* name,double fallback){
      const double v=declare_parameter(name,fallback);
      if(!std::isfinite(v)||v<.1||v>5.)throw std::invalid_argument(std::string("invalid ")+name);
      return v;};
    fresh_limit("feedback.pose_max_age_s",3.0);
    fresh_limit("feedback.pose_stamp_max_age_s",3.0);
    fresh_limit("feedback.odom_max_age_s",3.0);
    fresh_limit("feedback.joints_max_age_s",3.0);
    fresh_limit("feedback.packet_max_age_s",3.0);
    fresh_limit("feedback.scene_max_age_s",3.0);
    fresh_limit("feedback.observed_space_max_age_s",3.0);
    fresh_limit("arrival.confirm_s",0.3);

    speed_safety_scale_enabled_=declare_parameter("speed_safety_scale_enabled",true);
    low_speed_scrub_guard_s_=declare_parameter("low_speed_scrub_guard_s",1.0);
    min_moving_speed_=declare_parameter("min_moving_speed_mps",.05);
    if(!std::isfinite(min_moving_speed_)||min_moving_speed_<.01||min_moving_speed_>.3)
      throw std::invalid_argument("invalid min_moving_speed_mps");
    final_approach_radius_=declare_parameter("final_approach_radius_m",.50);
    final_approach_stop_=declare_parameter("final_approach_stop_m",.08);
    final_approach_speed_=declare_parameter("final_approach_speed_mps",.08);
    watchdog_feedback_s_=declare_parameter("watchdog.feedback_timeout_s",3.0);
    watchdog_replan_s_=declare_parameter("watchdog.replan_timeout_s",3.0);
    watchdog_steering_s_=declare_parameter("watchdog.steering_timeout_s",2.0);
    watchdog_align_s_=declare_parameter("watchdog.align_timeout_s",5.0);
    reverse_enabled_=declare_parameter("recovery.reverse_enabled",true);
    reverse_speed_=declare_parameter("recovery.reverse_speed_mps",.12);
    reverse_distance_=declare_parameter("recovery.reverse_distance_m",1.50);
    reverse_timeout_=declare_parameter("recovery.reverse_timeout_s",15.0);
    reverse_max_attempts_=declare_parameter("recovery.reverse_max_attempts",2);
    reverse_require_observed_=declare_parameter("recovery.reverse_require_observed_free",true);
    reverse_use_history_=declare_parameter("recovery.reverse_use_trajectory_history",true);
    reverse_history_age_s_=declare_parameter("recovery.reverse_history_age_s",10.0);
    if(!std::isfinite(reverse_history_age_s_)||reverse_history_age_s_<1.||reverse_history_age_s_>15.)
      reverse_history_age_s_=10.0;
    if(!std::isfinite(reverse_speed_)||reverse_speed_<=0.||reverse_speed_>.2||
       !std::isfinite(reverse_distance_)||reverse_distance_<.1||reverse_distance_>3.0||
       !std::isfinite(reverse_timeout_)||reverse_timeout_<.5||reverse_timeout_>30.||
       reverse_max_attempts_<1||reverse_max_attempts_>5)
      throw std::invalid_argument("invalid reverse recovery parameters");
    if(!std::isfinite(final_approach_radius_)||final_approach_radius_<.2||final_approach_radius_>1.0||
       !std::isfinite(final_approach_stop_)||final_approach_stop_<.05||final_approach_stop_>=final_approach_radius_||
       !std::isfinite(final_approach_speed_)||final_approach_speed_<=0.||final_approach_speed_>.3)
      throw std::invalid_argument("invalid final approach parameters");
    if(!std::isfinite(watchdog_feedback_s_)||watchdog_feedback_s_<.5||
       !std::isfinite(watchdog_replan_s_)||watchdog_replan_s_<.5||
       !std::isfinite(watchdog_steering_s_)||watchdog_steering_s_<.5||
       !std::isfinite(watchdog_align_s_)||watchdog_align_s_<.5)
      throw std::invalid_argument("invalid watchdog parameters");
    if(!std::isfinite(low_speed_scrub_guard_s_)||low_speed_scrub_guard_s_<.2||low_speed_scrub_guard_s_>5.)
      throw std::invalid_argument("invalid low_speed_scrub_guard_s");
    leg_gate_enabled_=declare_parameter("leg_gate_enabled",true);
    leg_home_tolerance_=declare_parameter("leg_home_tolerance_rad",.08);
    leg_home_names_=declare_parameter<std::vector<std::string>>("leg_home_joint_names",
      std::vector<std::string>{"fl_leg_pitch1","fl_leg_pitch2","fr_leg_pitch1","fr_leg_pitch2",
        "rl_leg_pitch1","rl_leg_pitch2","rr_leg_pitch1","rr_leg_pitch2"});
    leg_home_positions_=declare_parameter<std::vector<double>>("leg_home_joint_positions",
      std::vector<double>{.7853981633974483,-1.3089969389957472,.7853981633974483,-1.3089969389957472,
        .7853981633974483,-1.3089969389957472,.7853981633974483,-1.3089969389957472});
    if(!std::isfinite(leg_home_tolerance_)||leg_home_tolerance_<=0.||leg_home_tolerance_>.2||
       leg_home_names_.size()!=leg_home_positions_.size()||leg_home_names_.empty())
      throw std::invalid_argument("invalid leg home gate parameters");

    if(task_.empty()||!std::isfinite(vmax_)||vmax_<.01||vmax_>1.0||!std::isfinite(wmax_)||wmax_<=0||wmax_>.5||!std::isfinite(accel_)||accel_<=0)throw std::invalid_argument("invalid navigation task parameters");
    display_=create_publisher<nav_msgs::msg::Path>("/n01/navigation/path",rclcpp::QoS(1).reliable().transient_local());
    cmd_=create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
    status_=create_publisher<std_msgs::msg::String>("/n01/navigation/execution_status",10);
    progress_=create_publisher<Progress>("/n01/navigation/progress",10);
    reached_=create_publisher<geometry_msgs::msg::PointStamped>("/n01/navigation/goal_reached",10);
    replan_=create_publisher<std_msgs::msg::Bool>("/n01/navigation/replan",10);
    own_gid_=cmd_->get_gid();
    auto sensor=rclcpp::SensorDataQoS();
    observe_=create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",sensor,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr,const rclcpp::MessageInfo&i){
        const auto &g=i.get_rmw_message_info().publisher_gid;
        if(std::memcmp(g.data,own_gid_.data,RMW_GID_STORAGE_SIZE)!=0){manual_=true;last_manual_=steady();v_=w_=0.;}
      });
    pose_sub_=create_subscription<nav_msgs::msg::Odometry>("/body_pose",sensor,[this](nav_msgs::msg::Odometry::ConstSharedPtr m){
      auto p=m->pose.pose.position;auto q=m->pose.pose.orientation;
      double age=(now()-rclcpp::Time(m->header.stamp)).seconds();
      if(m->header.frame_id!="map"||m->child_frame_id!="base_link"||!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||age<-.1||age>live_limit("feedback.pose_stamp_max_age_s"))return;
      double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;if(!std::isfinite(norm)||std::abs(norm-1.)>.02)return;
      x_=p.x;y_=p.y;z_=p.z;yaw_=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
      pose_rx_=steady();pose_stamp_=rclcpp::Time(m->header.stamp).nanoseconds();have_pose_=true;
    });
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::QoS(64).best_effort(),[this](nav_msgs::msg::Odometry::ConstSharedPtr m){
      double age=(now()-rclcpp::Time(m->header.stamp)).seconds();auto t=m->twist.twist;
      if(m->header.frame_id!="odom"||m->child_frame_id!="base_link"||age<-.1||age>live_limit("feedback.odom_max_age_s")||!std::isfinite(t.linear.x)||!std::isfinite(t.linear.y)||!std::isfinite(t.angular.z))return;
      double speed=std::hypot(t.linear.x,t.linear.y);int64_t ns=rclcpp::Time(m->header.stamp).nanoseconds();
      if(ns<=odom_stamp_)return;
      if(odom_stamp_&&ns-odom_stamp_<=80000000)travel_+=(speed+measured_v_)*.5*(ns-odom_stamp_)/1e9;
      measured_v_=speed;measured_w_=t.angular.z;odom_stamp_=ns;odom_rx_=steady();
    });
    joints_=create_subscription<sensor_msgs::msg::JointState>("/joint_states",sensor,[this](sensor_msgs::msg::JointState::ConstSharedPtr m){
      const std::array<std::string,4> names={"fl_ankle_yaw","fr_ankle_yaw","rl_ankle_yaw","rr_ankle_yaw"};
      double age=(now()-rclcpp::Time(m->header.stamp)).seconds();if(age<-.2||age>live_limit("feedback.joints_max_age_s"))return;
      for(size_t i=0;i<m->name.size()&&i<m->position.size();i++){
        if(!std::isfinite(m->position[i]))continue;
        for(size_t j=0;j<4;j++)if(m->name[i]==names[j]){
          int64_t stamp=rclcpp::Time(m->header.stamp).nanoseconds();
          if(stamp<=joint_stamp_[j])continue;
          double rate=std::numeric_limits<double>::quiet_NaN();
          if(i<m->velocity.size()&&std::isfinite(m->velocity[i]))rate=m->velocity[i];
          else if(joint_stamp_[j]&&stamp-joint_stamp_[j]<=500000000)rate=(m->position[i]-q_[j])/((stamp-joint_stamp_[j])/1e9);
          q_[j]=m->position[i];joint_stamp_[j]=stamp;
          if(std::isfinite(rate)){qd_[j]=rate;joint_rx_[j]=steady();}
        }
        for(size_t h=0;h<leg_home_names_.size();h++)if(m->name[i]==leg_home_names_[h]){
          leg_positions_[leg_home_names_[h]]=m->position[i];leg_rx_=steady();
        }
      }
    });
    auto stopq=rclcpp::QoS(10).reliable().transient_local();
    estop_sub_=create_subscription<std_msgs::msg::Bool>("/estop",stopq,[this](std_msgs::msg::Bool::ConstSharedPtr m){estop_|=m->data;});
    dec_sub_=create_subscription<std_msgs::msg::UInt8>("/chassis_dec_stop/state",stopq,[this](std_msgs::msg::UInt8::ConstSharedPtr m){dec_=m->data;});
    enable_sub_=create_subscription<std_msgs::msg::Bool>("/n01/navigation/enable",10,[this](std_msgs::msg::Bool::ConstSharedPtr m){
      if(m->data&&!enabled_&&!finished_&&steady()-last_manual_>1.)manual_=false;
      enabled_=m->data; if(!enabled_)v_=w_=0.;
    });
    pause_sub_=create_subscription<std_msgs::msg::Bool>("/n01/navigation/pause",10,[this](std_msgs::msg::Bool::ConstSharedPtr m){
      paused_=m->data;
      if(paused_){v_=w_=0.;}
    });
    speed_sub_=create_subscription<std_msgs::msg::Float64>("/n01/navigation/speed_limit",10,[this](std_msgs::msg::Float64::ConstSharedPtr m){
      const double value=m->data;
      if(std::isfinite(value)&&value>=.01&&value<=1.)vmax_=value;
    });
    goal_sub_=create_subscription<geometry_msgs::msg::PoseStamped>("/n01/live_navigation/goal_gated",10,[this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m){
      uint64_t id=rclcpp::Time(m->header.stamp).nanoseconds();
      if(coordinate_fault_||m->header.frame_id!="map"||id<=goal_)return;
      if(goal_&&!follow_updates_){finished_=true;state_="GOAL_CHANGED";terminal_reason_="accepted_goal_changed";return;}
      goal_=id;curve_.reset();packet_.reset();arrived_=false;finished_=false;arrival_zone_=false;arrival_since_=0.;execution_time_=0.;reverse_attempts_=0;reverse_active_=false;last_blocked_reason_.clear();
    });
    health_sub_=create_subscription<std_msgs::msg::String>("/n01/live_navigation/system_health",10,[this](std_msgs::msg::String::ConstSharedPtr m){
      try{health_=Json::parse(m->data);health_rx_=steady();}catch(...){health_=Json();}
    });
    trajectory_sub_=create_subscription<Packet>("/n01/navigation/trajectory",10,[this](Packet::ConstSharedPtr m){receive(m);});
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this](){tick();});
    RCLCPP_INFO(get_logger(),"Unified controller: %s; task=%s; dry_run=%s",enabled_?"enabled":"disabled",task_.c_str(),dry_?"true":"false");
  }
  void stop(){if(!manual_&&!dry_)cmd_->publish(geometry_msgs::msg::Twist());}
 private:
  static double steady(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  void request_replan(){if(steady()-replan_at_>.5){std_msgs::msg::Bool b;b.data=true;replan_->publish(b);replan_at_=steady();}}
  void receive(Packet::ConstSharedPtr m){
    if(goal_&&m->goal_id<goal_)return;
    auto rejected=[&](const char* code){packet_rejection_={{"reason",code},{"goal_id",m->goal_id},{"revision",m->revision},{"source_reason",m->reason},
      {"scene_valid",m->scene_valid},{"frame",m->header.frame_id},{"scene_age_s",(now()-rclcpp::Time(m->scene_stamp)).seconds()},
      {"packet_age_s",(now()-rclcpp::Time(m->header.stamp)).seconds()},
      {"scene_limit_s",live_limit("feedback.scene_max_age_s")}};};
    if(m->map_id.empty()){rejected("packet_map_id_missing");return;}
    if(m->header.frame_id!="map"||m->goal_id==0||m->spline.goal_id!=m->goal_id||m->revision!=m->spline.traj_id){rejected("packet_goal_frame_or_revision_mismatch");return;}
    if(!m->scene_valid){rejected("packet_scene_inputs_not_ready");return;}
    const double age=(now()-rclcpp::Time(m->header.stamp)).seconds();
    const double scene_age=(now()-rclcpp::Time(m->scene_stamp)).seconds();
    if(age<-.1||age>live_limit("feedback.scene_max_age_s")||scene_age<-.1||scene_age>live_limit("feedback.scene_max_age_s")){rejected("packet_or_scene_stale");return;}
    if(expected_&&std::hypot(m->goal.x-expected_x_,m->goal.y-expected_y_)>.12){rejected("packet_expected_goal_mismatch");return;}
    if(goal_&&m->goal_id<goal_)return;
    if(goal_&&m->goal_id!=goal_&&!follow_updates_){finished_=true;state_="GOAL_CHANGED";terminal_reason_="accepted_goal_changed";return;}
    if(m->chassis_geometry.size()!=9||m->footprint.size()<3||m->footprint.size()>256||m->obstacles.width>120000){rejected("packet_geometry_or_scene_size_invalid");return;}
    for(auto p:m->chassis_geometry)if(!std::isfinite(p))return;
    const auto &g=m->chassis_geometry;
    if(g[0]<=0||g[1]<=0||g[2]<=0||g[3]<0||g[4]<=0||g[5]<=0||g[6]<=0||g[7]<=0||g[7]>3.141593||g[8]<0)return;
    for(auto p:m->footprint)if(!std::isfinite(p.x)||!std::isfinite(p.y))return;
    const auto &cloud=m->obstacles;
    if(cloud.header.frame_id!="map"||cloud.height!=1||cloud.point_step!=12||cloud.is_bigendian||
      cloud.row_step!=cloud.width*12||cloud.data.size()!=size_t(cloud.row_step)||cloud.fields.size()!=3)return;
    const std::array<std::string,3> names={"x","y","z"};
    for(int i=0;i<3;i++)if(cloud.fields[i].name!=names[i]||cloud.fields[i].offset!=unsigned(i*4)||cloud.fields[i].datatype!=7||cloud.fields[i].count!=1)return;
    for(size_t i=0;i<cloud.data.size();i+=4){float p;std::memcpy(&p,&cloud.data[i],4);if(!std::isfinite(p)||std::abs(p)>1e6)return;}
    for(auto p:m->window_bounds)if(!std::isfinite(p))return;
    if(m->require_observed_space){
      const auto& f=m->observed_free;
      if(!std::isfinite(m->observed_resolution)||m->observed_resolution<.04||m->observed_resolution>.2||f.header.frame_id!="map"||f.height!=1||f.width>200000||f.point_step!=12||f.row_step!=f.width*12||f.data.size()!=size_t(f.row_step)||f.is_bigendian||f.fields.size()!=3)return;
      double free_age=(now()-rclcpp::Time(f.header.stamp)).seconds();if(free_age<-.1||free_age>live_limit("feedback.observed_space_max_age_s")){rejected("observed_space_stale");return;}
      for(int i=0;i<3;i++)if(f.fields[i].name!=names[i]||f.fields[i].offset!=unsigned(i*4)||f.fields[i].datatype!=7||f.fields[i].count!=1)return;
      for(size_t i=0;i<f.data.size();i+=4){float v;std::memcpy(&v,&f.data[i],4);if(!std::isfinite(v)||std::abs(v)>1e6)return;}
    }
    if(m->window_bounds[2]<=m->window_bounds[0]||m->window_bounds[3]<=m->window_bounds[1])return;
    if(!std::isfinite(m->scene_radius)||m->scene_radius<=0||m->scene_radius>12.)return;
    if(!std::isfinite(m->max_steering_rate)||!std::isfinite(m->max_wheel_rate)||m->max_steering_rate<=0||m->max_wheel_rate<=0)return;
    if(m->max_speed<=0||m->max_accel<=0||!std::isfinite(m->max_accel)||!std::isfinite(m->environment_margin)||!std::isfinite(m->goal.x)||!std::isfinite(m->goal.y)||!std::isfinite(m->scene_center.x)||!std::isfinite(m->scene_center.y)||!std::isfinite(m->max_speed)||!std::isfinite(m->goal_yaw)||m->environment_margin<0||m->environment_margin>.1)return;
    if(!map_id_.empty()&&m->map_id!=map_id_){coordinate_fault_=true;finished_=true;state_="FAILED";terminal_reason_="map_or_coordinate_model_changed";return;}
    packet_rejection_=Json::object();
    map_id_=m->map_id;
    if(!goal_||m->goal_id!=goal_){goal_=m->goal_id;curve_.reset();arrived_=finished_=false;arrival_zone_=false;execution_time_=0.;reverse_attempts_=0;reverse_active_=false;last_blocked_reason_.clear();start_aligned_=false;last_blocked_reason_.clear();}
    if(!m->valid){
      // A new invalid packet means the current valid curve is no longer backed
      // by a fresh scene. Keep it only for diagnostics; the freshness gate must
      // stop motion until a new valid packet arrives.
      latest_packet_valid_=false;
      adopt_scene(m);packet_rx_=steady();request_replan();return;
    }
    latest_packet_valid_=true;
    if(m->hold_position){
      // The monitor switches to hold packets inside its final handoff radius
      // (0.5 m). Accept them across the whole FINAL_APPROACH/ALIGN zone, or
      // the scene goes stale between 0.10 m and 0.50 m and execution deadlocks.
      if(!have_pose_||std::hypot(m->goal.x-x_,m->goal.y-y_)>final_approach_radius_+.05)return;
      adopt_scene(m);packet_rx_=steady();hold_=true;revision_=m->revision;admitted_speed_=m->max_speed;return;
    }
    hold_=false;
    if(curve_&&m->revision<revision_)return;
    auto candidate=std::make_shared<navigation::Curve>();
    if(curve_&&m->revision==revision_){
      if(m->spline.knots!=curve_->knots||m->spline.pos_pts.size()!=curve_->control.size())return;
      for(size_t i=0;i<curve_->control.size();i++)if(m->spline.pos_pts[i].x!=curve_->control[i][0]||m->spline.pos_pts[i].y!=curve_->control[i][1]||m->spline.pos_pts[i].z!=curve_->control[i][2])return;
      adopt_scene(m);admitted_speed_=m->max_speed;packet_rx_=steady();return; // Same geometry: update speed advice, preserve progress.
    }
    try{candidate->degree=m->spline.order;candidate->knots=m->spline.knots;for(auto p:m->spline.pos_pts)candidate->control.push_back({p.x,p.y,p.z});candidate->build();}catch(...){return;}
    // New scene applies to BOTH the held and candidate curve. Keep the current
    // safe route while extending, avoiding equivalent left/right oscillation.
    adopt_scene(m);
    if(curve_&&have_pose_&&curve_->length-progress_s_>.5&&
        clear_curve(*curve_,progress_s_,std::min(curve_->length,progress_s_+1.))){
      // Keep the already-validated geometry, but never keep its old speed
      // advice. Otherwise a low-speed curve can pin admitted_speed below the
      // planner's fresh max_speed indefinitely while the robot keeps moving.
      admitted_speed_=m->max_speed;
      packet_rx_=steady();return;
    }
    if(have_pose_){
      double s=candidate->nearest(x_,y_);auto p=candidate->at(s);
      if(std::hypot(p.x-x_,p.y-y_)>.35){packet_rx_=steady();request_replan();return;}
      if(curve_&&measured_v_>.01&&std::abs(wrap(p.yaw-curve_->at(curve_->nearest(x_,y_)).yaw))>.35){
        packet_rx_=steady();request_replan();return;
      }
      progress_s_=s;
    }else progress_s_=0.;
    curve_=candidate;revision_=m->revision;admitted_speed_=m->max_speed;packet_rx_=steady();arrival_since_=0.;
  }
  double live_limit(const char* name) const { return get_parameter(name).as_double(); }
  bool fresh(){
    double t=steady();
    if(!have_pose_||t-pose_rx_>live_limit("feedback.pose_max_age_s")||
       (now().nanoseconds()-pose_stamp_)/1e9>live_limit("feedback.pose_stamp_max_age_s")){feedback_reason_="control_pose_stale";return false;}
    if(t-odom_rx_>live_limit("feedback.odom_max_age_s")){feedback_reason_="chassis_twist_stale";return false;}
    for(double rx:joint_rx_)if(t-rx>live_limit("feedback.joints_max_age_s")){feedback_reason_="wheel_feedback_stale";return false;}
    if(!packet_||(!curve_&&!hold_)){feedback_reason_=packet_rejection_.value("reason",std::string("waiting_for_native_scan_trajectory"));return false;}
    if(!latest_packet_valid_){feedback_reason_="latest_trajectory_invalid_or_scene_stale";return false;}
    if(t-packet_rx_>live_limit("feedback.packet_max_age_s")||
       (now()-rclcpp::Time(packet_->header.stamp)).seconds()>live_limit("feedback.scene_max_age_s")||
       (now()-rclcpp::Time(packet_->scene_stamp)).seconds()>live_limit("feedback.scene_max_age_s")){feedback_reason_="scene_or_admission_stale";return false;}
    // Inactive while require_observed_space is false (coverage gating removed).
    if(packet_->require_observed_space && (now()-rclcpp::Time(packet_->observed_free.header.stamp)).seconds()>live_limit("feedback.observed_space_max_age_s")){feedback_reason_="observed_space_stale";return false;}
    feedback_reason_.clear();return true;
  }
  static uint64_t cell_key(int x,int y){return (uint64_t(uint32_t(x))<<32)|uint32_t(y);}
  void adopt_scene(Packet::ConstSharedPtr p){
    packet_=p;scene_grid_.clear();observed_free_.clear();if(!p)return;
    if(p->require_observed_space)for(size_t i=0;i<p->observed_free.data.size();i+=12){
      float x,y;std::memcpy(&x,&p->observed_free.data[i],4);std::memcpy(&y,&p->observed_free.data[i+4],4);
      observed_free_.insert(cell_key(int(std::floor(double(x)/p->observed_resolution)),int(std::floor(double(y)/p->observed_resolution))));
    }
    const auto &cloud=p->obstacles;
    for(size_t i=0;i<cloud.data.size();i+=12){
      float x,y;std::memcpy(&x,&cloud.data[i],4);std::memcpy(&y,&cloud.data[i+4],4);
      scene_grid_[cell_key(int(std::floor(x/.25)),int(std::floor(y/.25)))].push_back({x,y});
    }
  }
  bool clear(double x,double y,double yaw,double extra=0.)const{
    if(!packet_)return false;
    if(packet_->require_observed_space && !observed_free_.count(cell_key(int(std::floor(x/packet_->observed_resolution)),int(std::floor(y/packet_->observed_resolution)))))return false;
    const auto &b=packet_->window_bounds;double c=std::cos(yaw),s=std::sin(yaw),rho=0.;
    std::vector<std::array<double,2>> hull;
    for(auto p:packet_->footprint){
      hull.push_back({p.x,p.y});rho=std::max(rho,std::hypot(p.x,p.y));
      double wx=x+c*p.x-s*p.y,wy=y+s*p.x+c*p.y;
      if(wx-packet_->environment_margin<b[0]||wy-packet_->environment_margin<b[1]||wx+packet_->environment_margin>b[2]||wy+packet_->environment_margin>b[3])return false;
    }
    if(packet_->require_observed_space){
      const double step=packet_->observed_resolution;
      for(int gx=int(std::floor((x-rho-packet_->environment_margin-extra)/step));gx<=int(std::floor((x+rho+packet_->environment_margin+extra)/step));++gx)
        for(int gy=int(std::floor((y-rho-packet_->environment_margin-extra)/step));gy<=int(std::floor((y+rho+packet_->environment_margin+extra)/step));++gy){
          double dx=(gx+.5)*step-x,dy=(gy+.5)*step-y;
          if(navigation::hits(c*dx+s*dy,-s*dx+c*dy,hull,packet_->environment_margin+extra) && !observed_free_.count(cell_key(gx,gy)))return false;
        }
    }
    if(std::hypot(x-packet_->scene_center.x,y-packet_->scene_center.y)+rho+packet_->environment_margin+extra>packet_->scene_radius)return false;
    const double range=rho+packet_->environment_margin+extra;
    for(int gx=int(std::floor((x-range)/.25));gx<=int(std::floor((x+range)/.25));gx++)
      for(int gy=int(std::floor((y-range)/.25));gy<=int(std::floor((y+range)/.25));gy++){
        auto cell=scene_grid_.find(cell_key(gx,gy));if(cell==scene_grid_.end())continue;
        for(auto p:cell->second){
          double dx=p[0]-x,dy=p[1]-y;if(dx*dx+dy*dy>range*range)continue;
          if(navigation::hits(c*dx+s*dy,-s*dx+c*dy,hull,packet_->environment_margin+extra))return false;
        }
    }return true;
  }
  bool clear_curve(const navigation::Curve& curve,double from,double to){
    for(double s=from;s<=to+1e-6;s+=.04){auto p=curve.at(s);if(!clear(p.x,p.y,p.yaw,.02))return false;}return true;
  }
  bool sweep(double target){
    const double age=(now()-rclcpp::Time(packet_->rotation_stamp)).seconds();
    if(!packet_->rotation_observed||age<-.1||age>live_limit("feedback.scene_max_age_s"))return false;
    double d=wrap(target-yaw_);int count=std::max(1,int(std::ceil(std::abs(d)/.035)));
    double rho=0;for(auto p:packet_->footprint)rho=std::max(rho,std::hypot(p.x,p.y));
    for(int i=0;i<=count;i++)if(!clear(x_,y_,yaw_+d*i/count,rho*std::abs(d)/count/2))return false;
    return true;
  }
  bool model(double v,double w){
    physical_reason_.clear();
    const auto&p=packet_->chassis_geometry;
    wheel_control::ChassisKinematics::ChassisConfig c{};
    c.wheel_radius=p[0];c.half_base_x=p[1]/2;c.half_base_y=p[2]/2;c.delta_y=p[3];c.max_linear_speed=p[4];c.max_angular_speed=p[5];c.spin_v_threshold=p[6];c.yaw_soft_limit_rad=p[7];c.steer_near_zero_bias=p[8];
    wheel_control::ChassisKinematics k(c);
    auto target=k.ComputeSteerTargets(v,0.,w,q_);
    auto drive=k.ComputeDriveFromSteer(v,0.,w,target);
    auto f=k.ComputeFeedbackVelocities(target,wheel_control::WheelArray{},drive);
    // ComputeSteerTargets uses pivot geometry while ComputeDriveFromSteer / FK
    // project onto ContactPoint(theta, delta_y). With delta_y > 0, in-place
    // rotation (v=0) has a fixed proportional omega projection offset (>2%).
    const bool pure_pivot=(std::abs(v)<1e-4&&std::abs(w)>1e-4);
    const bool fk_ok=pure_pivot
      ? (f.valid&&std::abs(f.vx)<=.02&&std::abs(f.vy)<=.02&&(f.omega*w>0.)&&std::abs(f.omega)>=.5*std::abs(w))
      : (f.valid&&std::abs(f.vx-v)<=std::max(1e-3,.05*std::abs(v))&&std::abs(f.vy)<=.01&&std::abs(f.omega-w)<=std::max(1e-3,.15*std::abs(w)));
    if(!fk_ok){physical_reason_="native_ik_fk_twist_mismatch";return false;}
    double max_change=0.;for(int j=0;j<4;j++){
      // The measured steer angle can overshoot the soft limit by a small
      // amount (backlash/closed-loop overshoot). Refusing every target then
      // deadlocks the controller outside the limit. Allow a target only when
      // it moves the measured angle back inside the soft limit.
      const bool measured_out=std::abs(q_[j])>p[7]+1e-6;
      const bool target_out=std::abs(target[j])>p[7]+1e-6;
      const bool inward=!measured_out||std::abs(target[j])<std::abs(q_[j])-1e-4;
      if(target_out||!inward||std::abs(drive[j])>packet_->max_wheel_rate||std::abs(qd_[j])>packet_->max_steering_rate*2.){physical_reason_=measured_out&&!target_out?"steering_angle_recovery_required":"live_wheel_angle_speed_or_steering_rate_limit";return false;}
      max_change=std::max(max_change,std::abs(target[j]-q_[j]));
    }
    // Slew mismatch slows motion; native control handles steering transitions.
    double rate=0.;for(double r:qd_)rate=std::max(rate,std::abs(r));
    steering_scale_=std::min(std::clamp(1.-max_change/3.141592653589793,.15,1.),
      rate>packet_->max_steering_rate?packet_->max_steering_rate/rate:1.);return true;
  }
  // Search the admissible angular-rate interval before discarding a command.
  // The native 4WS solver can reject a requested w because one wheel target
  // crosses the soft steering limit; a smaller |w| is often still feasible.
  bool model_limited(double v, double requested_w, double& out_w){
    if(model(v,requested_w)){out_w=requested_w;return true;}
    const double sign=requested_w<0.?-1.:1.;
    const double mag=std::abs(requested_w);
    for(int i=1;i<=12;i++){
      const double cand=sign*mag*(1.-double(i)/12.);
      if(std::abs(cand)<1e-4)break;
      if(model(v,cand)){out_w=cand;return true;}
    }
    out_w=0.;return false;
  }
  bool measured_arrival(int64_t /*after*/){
    if(!have_pose_||!packet_)return false;
    if(steady()-pose_rx_>live_limit("feedback.pose_max_age_s"))return false;
    try{
      if(steady()-health_rx_<=3.0&&health_.contains("bridge")){
        auto b=health_.at("bridge");
        if(b.contains("pose_prediction")){
          auto p=b.at("pose_prediction");
          if(!p.at("fault").is_null()&&p.at("fault")!=false&&p.at("fault")!="")return false;
        }
      }
    }catch(...){}
    if(std::hypot(x_-packet_->goal.x,y_-packet_->goal.y)>.15)return false;
    const double yaw_confirm_tol=final_yaw_tolerance_+0.052;
    if(final_yaw_&&std::abs(wrap(yaw_-packet_->goal_yaw))>yaw_confirm_tol)return false;
    return true;
  }
  bool legs_home() const {
    if(!leg_gate_enabled_)return true;
    if(steady()-leg_rx_>live_limit("feedback.joints_max_age_s"))return false;
    for(size_t i=0;i<leg_home_names_.size();i++){
      auto it=leg_positions_.find(leg_home_names_[i]);
      if(it==leg_positions_.end()||!std::isfinite(it->second)||
         std::abs(it->second-leg_home_positions_[i])>leg_home_tolerance_)return false;
    }
    return true;
  }
  bool history_reverse_safe(double now_s) const {
    if(!reverse_use_history_ || history_.size()<4) return false;
    const double c=std::cos(yaw_),s=std::sin(yaw_);
    int behind=0;
    for(auto it=history_.rbegin();it!=history_.rend();++it){
      if(now_s-it->t>reverse_history_age_s_) break;
      const double dx=it->x-x_,dy=it->y-y_;
      if(std::hypot(dx,dy)>reverse_distance_+.25) continue;
      if(dx*c+dy*s < -.02) ++behind;
      if(behind>=4) return true;
    }
    return false;
  }
  void record_history(double now_s){
    if(!have_pose_)return;
    if(!history_.empty() && std::hypot(x_-history_.back().x,y_-history_.back().y)<.01)return;
    history_.push_back({x_,y_,yaw_,now_s});
    while(!history_.empty() && (now_s-history_.front().t>10. ||
          std::hypot(x_-history_.front().x,y_-history_.front().y)>3.0)) history_.pop_front();
  }
  void tick(){
    const double began=steady(),dt=last_tick_?std::clamp(began-last_tick_,0.,.1):.02;last_tick_=began;
    double v_before_scale=0.;int safety_reductions=0;std::string speed_limit_sources;
    bool ready=fresh()&&!manual_&&!estop_&&dec_==0&&!finished_;
    double v=0,w=0;std::string reason=finished_?terminal_reason_:(ready?"":feedback_reason_);
    // Preserve the controller state between 20 ms ticks. Resetting to DISABLED
    // here made SETTLING/WAITING unreachable and reset settle timers every tick.
    if(!enabled_ && !finished_) state_="DISABLED";
    if(paused_&&!finished_&&enabled_){state_="PAUSED";reason="user_paused";v=w=0.;}
    else if(manual_){state_="MANUAL_OVERRIDE";reason="external command traffic; explicit resume required";}
    else if(estop_||dec_!=0){state_="ESTOP";reason="chassis stop or emergency stop";}
    else if(!finished_&&enabled_){
      // Highest priority after ESTOP/MANUAL/PAUSED: once the robot has reached
      // the goal position and yaw, immediately stop and confirm ARRIVED without
      // being interrupted by scene_or_admission_stale or SAFETY_HOLD.
      const bool pose_fresh_now=have_pose_&&packet_&&(began-pose_rx_<=live_limit("feedback.pose_max_age_s"));
      bool goal_arrived_now=false;
      if(pose_fresh_now){
        const double d_goal=std::hypot(packet_->goal.x-x_,packet_->goal.y-y_);
        if(d_goal<=.10)arrival_zone_=true;
        if(d_goal>=.15)arrival_zone_=false;
        const double yaw_gate=(state_=="VERIFYING_ARRIVAL")?(final_yaw_tolerance_+0.052):final_yaw_tolerance_;
        const bool yaw_ok=!final_yaw_||std::abs(wrap(packet_->goal_yaw-yaw_))<=yaw_gate;
        goal_arrived_now=arrival_zone_&&yaw_ok;
      }
      if(goal_arrived_now){
        v=w=0.;state_="VERIFYING_ARRIVAL";reason="";pivot_dir_=0;final_approach_active_=false;
        if(measured_v_<=.02&&std::abs(measured_w_)<=.03){
          if(!arrival_since_){arrival_since_=began;arrival_stamp_=now().nanoseconds();}
          if(began-arrival_since_>=live_limit("arrival.confirm_s")&&measured_arrival(arrival_stamp_)){
            state_="ARRIVED";finished_=arrived_=true;
            geometry_msgs::msg::PointStamped gp;gp.header.stamp=now();gp.header.frame_id="map";gp.point=packet_->goal;
            reached_->publish(gp);
          }
        }else arrival_since_=0.;
      }else{
      const double state_elapsed=state_since_>0.?began-state_since_:0.;
      if(state_=="WAITING_FOR_FEEDBACK"&&state_elapsed>=watchdog_feedback_s_){
        state_="SAFETY_HOLD";safety_hold_reason_="feedback_timeout";safety_release_since_=0.;
      }else if(state_=="WAITING_FOR_REPLAN"&&state_elapsed>=watchdog_replan_s_){
        state_="SAFETY_HOLD";safety_hold_reason_="replan_timeout";safety_release_since_=0.;
      }else if(state_=="WAITING_FOR_STEERING"&&state_elapsed>=watchdog_steering_s_){
        state_="SAFETY_HOLD";safety_hold_reason_="steering_timeout";safety_release_since_=0.;
      }else if((state_=="ALIGN"&&state_elapsed>=watchdog_align_s_)||
               (state_=="ROTATING"&&state_elapsed>=std::max(15.0,watchdog_align_s_*3.0))){
        state_="SAFETY_HOLD";safety_hold_reason_="alignment_timeout";safety_release_since_=0.;
      }
      if(state_=="SAFETY_HOLD"){
        v=w=0.;reason=safety_hold_reason_;
        const bool blockedish =
          (physical_reason_.find("collision")!=std::string::npos||
           physical_reason_.find("curvature")!=std::string::npos||
           physical_reason_.find("yaw_rate")!=std::string::npos||
           last_blocked_reason_.find("collision")!=std::string::npos||
           last_blocked_reason_.find("curvature")!=std::string::npos||
           last_blocked_reason_.find("yaw_rate")!=std::string::npos||
           last_blocked_reason_.find("scrub")!=std::string::npos||
           last_blocked_reason_.find("occupied")!=std::string::npos);
        if(reverse_enabled_&&reverse_attempts_<reverse_max_attempts_&&blockedish&&
           history_reverse_safe(began)){
          reverse_active_=true;reverse_start_x_=x_;reverse_start_y_=y_;reverse_start_s_=began;
          reverse_reset_travel_=travel_;
          ++reverse_attempts_;state_="REVERSING";reason="reverse_from_safety_hold";
          safety_hold_reason_.clear();safety_release_since_=0.;
        } else if(ready&&fresh()&&latest_packet_valid_){
          if(!safety_release_since_)safety_release_since_=began;
          else if(began-safety_release_since_>=.5){
            state_="MOVING";safety_release_since_=0.;reason="safety_hold_released";request_replan();
          }
        }else{
          safety_release_since_=0.;
          // Keep asking for a fresh trajectory (throttled to 2 Hz); a hold
          // that only waits passively can never recover once packets stop.
          request_replan();
        }
      } else if(!ready){state_="WAITING_FOR_FEEDBACK";reason=feedback_reason_;request_replan();}
      else {
        double measured=curve_?curve_->nearest(x_,y_):0.;progress_s_=std::max(progress_s_,measured);
        auto p=curve_?curve_->at(progress_s_):navigation::Sample{x_,y_,z_,yaw_,0.,0.};double distance=std::hypot(packet_->goal.x-x_,packet_->goal.y-y_);
        if(distance<=.10)arrival_zone_=true;
        if(distance>=.15)arrival_zone_=false;
        bool at_goal=arrival_zone_;
        const auto end=curve_?curve_->samples.back():p;
        const bool terminal=std::hypot(end.x-packet_->goal.x,end.y-packet_->goal.y)<=.12;
        // Terminal homing closes cross-track error instead of exhausting arc
        // length a millimetre outside the arrival zone.
        if(terminal&&distance<.35&&!at_goal)p.yaw=std::atan2(packet_->goal.y-y_,packet_->goal.x-x_);
        double yaw_error=wrap(p.yaw-yaw_);
        const bool can_pivot=legs_home();
        const bool near_goal=!at_goal&&distance<=final_approach_radius_;
        const double approach_heading=std::atan2(packet_->goal.y-y_,packet_->goal.x-x_);
        const bool approach_pivot=near_goal&&distance>.20&&can_pivot&&std::abs(wrap(approach_heading-yaw_))>.65;
        // Finish position convergence (into arrival_zone_) first, then perform
        // a single in-place final yaw alignment at the goal. Add +3 deg
        // hysteresis once inside VERIFYING_ARRIVAL so tiny pose noise does not
        // kick a stopped robot back into ROTATING.
        const double goal_yaw_gate=(state_=="VERIFYING_ARRIVAL")?(final_yaw_tolerance_+0.052):final_yaw_tolerance_;
        const bool goal_yaw_align=at_goal&&final_yaw_&&
          std::abs(wrap(packet_->goal_yaw-yaw_))>goal_yaw_gate;
        double desired=at_goal&&final_yaw_?packet_->goal_yaw:(approach_pivot?approach_heading:p.yaw);
        const double align_tol=at_goal&&final_yaw_?final_yaw_tolerance_:.08;
        // Bounded reverse escape: only after a blocked/collision forward
        // command, and only while the rear footprint is clear.
        const std::string blocked=physical_reason_.empty()?last_blocked_reason_:physical_reason_;
        if(reverse_enabled_&&!reverse_active_&&state_=="WAITING_FOR_REPLAN"&&
           reverse_attempts_<reverse_max_attempts_&&
           (blocked.find("collision")!=std::string::npos||
            blocked.find("curvature")!=std::string::npos||
            blocked.find("yaw_rate")!=std::string::npos||
            blocked.find("scrub")!=std::string::npos||
            blocked.find("no_safe")!=std::string::npos)){
          reverse_active_=true;reverse_start_x_=x_;reverse_start_y_=y_;reverse_start_s_=began;
          reverse_reset_travel_=travel_;
          ++reverse_attempts_;state_="REVERSING";reason="reverse_recovery_start";
        }
        if(state_=="REVERSING"){
          v=w=0.;
          const double travelled=std::hypot(x_-reverse_start_x_,y_-reverse_start_y_);
          const double elapsed=began-reverse_start_s_;
          bool safe=can_pivot&&travelled<reverse_distance_&&elapsed<reverse_timeout_;
          double feasible=0.;
          if(safe){
            if(reverse_require_observed_ && observed_free_.empty() &&
               !history_reverse_safe(began)){
              safe=false;physical_reason_="reverse_rear_not_observed";
            }
            const double vrev=-std::min(reverse_speed_,std::max(.03,vmax_*.5));
            if(model_limited(vrev,0.,feasible)){
              const double horizon=std::max(.6,reverse_distance_*1.1);
              double px=x_,py=y_;
              for(double t=0;t<=horizon;t+=.04){
                if(!clear(px,py,yaw_,.01)){safe=false;physical_reason_="reverse_rear_footprint_not_clear";break;}
                px+=vrev*std::cos(yaw_)*.04;py+=vrev*std::sin(yaw_)*.04;
              }
              if(safe){v=vrev;w=feasible;state_="REVERSING";reason="reverse_recovery";}
            }else{safe=false;physical_reason_="reverse_native_model_rejected";}
          }
          if(!safe||travelled>=reverse_distance_||elapsed>=reverse_timeout_){
            v=w=0.;state_="WAITING_FOR_REPLAN";
            reason=travelled>=reverse_distance_?"reverse_complete_replan":
                   (elapsed>=reverse_timeout_?"reverse_timeout_replan":"reverse_not_safe");
            reverse_active_=false;request_replan();
          }
        } else if(state_=="SETTLING"){
          v=w=0.;
          if(!can_pivot){state_="MOVING";settle_since_=0.;reason="leg_home_gate_released_settle";}
          else if(std::abs(wrap(desired-yaw_))>std::max(.10,align_tol+.02)){state_="ALIGN";settle_since_=0.;reason="heading_changed_during_settle";}
          else if(measured_v_>.01||std::abs(measured_w_)>.02){settle_since_=0.;reason="braking_after_rotation";}
          else if(!settle_since_){settle_since_=began;reason="settling_after_rotation";}
          else if(began-settle_since_>=.20 ||
                  (state_elapsed>=1.0&&measured_v_<.03&&std::abs(measured_w_)<.05)){
            state_=at_goal?"VERIFYING_ARRIVAL":"MOVING";settle_since_=0.;settle_enter_s_=0.;start_aligned_=true;reason="";
          }
          else{reason="settling_after_rotation";}
        } else {
        const bool start_align = can_pivot && !start_aligned_ && progress_s_<.20 && std::abs(yaw_error)>.08;
        if(can_pivot&&(goal_yaw_align||approach_pivot||(!at_goal&&!near_goal&&(std::abs(yaw_error)>.45||start_align||std::min({vmax_,packet_->max_speed,admitted_speed_})*std::max(0.,std::cos(yaw_error))<=packet_->chassis_geometry[6]+.001)))){
          state_="ALIGN";
          if(measured_v_>.01||std::abs(v_)>.005){reason="braking before alignment";}
          else if(sweep(desired)){
            double err=wrap(desired-yaw_);
            // Latch turn direction near +-pi so small pose jitter cannot flip
            // rotation back and forth between +wmax and -wmax.
            if(pivot_dir_!=0&&std::abs(err)>2.4){
              if(err*pivot_dir_<0.)err+=(pivot_dir_>0?2.*M_PI:-2.*M_PI);
            }else if(std::abs(err)>1e-3){
              pivot_dir_=(err>=0.?1:-1);
            }
            const double requested=navigation::limit(1.2*err,wmax_);
            double feasible=0.;
            if(model_limited(0.,requested,feasible)&&std::abs(feasible)>.01){
              w=feasible;state_="ROTATING";
              if(std::abs(feasible-requested)>.02)reason="steering_limited_rotation";
            }else{reason="native wheel model rejected rotation at all feasible rates";request_replan();}
          }
          else {
            reason=packet_->rotation_observed?"rotation swept volume occupied; SCAN replan requested":"waiting for full-azimuth scan";
            if(packet_->rotation_observed&&reverse_enabled_&&reverse_attempts_<reverse_max_attempts_&&
               state_=="ALIGN"&&state_elapsed>=1.5&&history_reverse_safe(began)){
              reverse_active_=true;reverse_start_x_=x_;reverse_start_y_=y_;reverse_start_s_=began;
              reverse_reset_travel_=travel_;
              ++reverse_attempts_;state_="REVERSING";reason="reverse_for_rotation_clearance";
            }else{
              request_replan();
            }
          }
        }else if(near_goal){
          // Terminal approach: do not let the general local B-spline planner
          // wander around the goal. Drive slowly along the direct bearing and
          // hand over to ALIGN only at the final yaw stop radius.
          if(!final_approach_active_){final_approach_best_=distance;final_approach_bad_since_=0.;}
          final_approach_active_=true;
          final_approach_best_=std::min(final_approach_best_,distance);
          if(distance>final_approach_best_+.08){
            if(!final_approach_bad_since_)final_approach_bad_since_=began;
          }else final_approach_bad_since_=0.;
          if(final_approach_bad_since_&&began-final_approach_bad_since_>=.8){
            v=w=0.;state_="WAITING_FOR_REPLAN";reason="final_approach_distance_increased";
            physical_reason_=reason;final_approach_active_=false;request_replan();
          }else{
            const double heading=std::atan2(packet_->goal.y-y_,packet_->goal.x-x_);
            const double heading_err=wrap(heading-yaw_);
            const double cos_h=std::cos(heading_err);
            double cap=std::min({vmax_,packet_->max_speed,admitted_speed_,final_approach_speed_});
            double steer_err=heading_err;
            if(cos_h>=0.){
              v=cap*cos_h;
            }else{
              // Overshot slightly past the goal (goal is behind the robot):
              // gently back up instead of deadlocking or spinning 180 deg.
              v=-std::min(cap,.08)*(-cos_h);
              steer_err=wrap(heading+M_PI-yaw_);
            }
            const double v_ratio=std::clamp(std::abs(v)/std::max(.001,vmax_),0.,1.);
            w=navigation::limit(1.5*steer_err,wmax_*v_ratio);
            double limited_w=0.;
            bool safe=std::abs(v)>=packet_->chassis_geometry[6]&&model_limited(v,w,limited_w);
            if(safe){
              w=limited_w;double px=x_,py=y_,a=yaw_,horizon=.4+std::abs(v)/std::min(accel_,packet_->max_accel);
              for(double t=0;t<=horizon;t+=.04){if(!clear(px,py,a,.01)){safe=false;break;}a+=w*.04;px+=v*std::cos(a)*.04;py+=v*std::sin(a)*.04;}
            }
            if(!safe){
              v=w=0.;state_="WAITING_FOR_REPLAN";reason="final_approach_no_safe_direct_command";physical_reason_=reason;final_approach_active_=false;request_replan();
            }else{state_="FINAL_APPROACH";start_aligned_=true;execution_time_+=dt;}
          }
        }else if(at_goal){
          state_="VERIFYING_ARRIVAL";
          if(measured_v_<=.01&&std::abs(measured_w_)<=.02){
            if(!arrival_since_){arrival_since_=began;arrival_stamp_=now().nanoseconds();}
            if(began-arrival_since_>=live_limit("arrival.confirm_s")&&measured_arrival(arrival_stamp_)){
              state_="ARRIVED";finished_=arrived_=true;geometry_msgs::msg::PointStamped p;p.header.stamp=now();p.header.frame_id="map";p.point=packet_->goal;reached_->publish(p);
            }
          }else arrival_since_=0.;
        }else if(!curve_||hold_){state_="WAITING_FOR_REPLAN";reason="station position changed; SCAN trajectory required";request_replan();}
        else{
          arrival_since_=0.;double remaining=curve_->length-progress_s_;
          double cap=std::min({vmax_,packet_->max_speed,admitted_speed_});
          // Apply steering_scale_ to the target speed cap (not after v_+accel*dt)
          // so moderate steering mismatch scales cruise speed smoothly instead of
          // trapping the acceleration ramp at 0.012 m/s forever.
          if(speed_safety_scale_enabled_)cap=std::max(min_moving_speed_,cap*steering_scale_);
          try{auto bridge=health_.at("bridge");if(bridge.contains("pose_prediction")&&bridge["pose_prediction"].value("source_age_s",0.)>1.5)cap=std::min(cap,.10);}catch(...){}
          double stop=terminal?std::max(0.,distance-.04):remaining;
          cap=navigation::braking_speed(cap,std::min(accel_,packet_->max_accel),std::max(0.,stop-.02));
          auto ahead=curve_->at(std::min(curve_->length,progress_s_+.25));
          if(terminal&&distance<.35){ahead.x=packet_->goal.x;ahead.y=packet_->goal.y;ahead.yaw=p.yaw;}
          auto command=navigation::feedback(p,ahead,x_,y_,yaw_,cap,wmax_);
          v=std::min(command.v,std::max(v_+accel_*dt,packet_->chassis_geometry[6]+.002));w=command.w;
          // Check the actual feedback-corrected command, not only the planned tangent.
          bool safe=false;
          for(int reduction=0;reduction<7;reduction++){
            double limited_w=0.;
            if(model_limited(v,w,limited_w)){
              const double trial_w=limited_w;
              // Always check at least ~0.35 m ahead even at crawl/startup speed
              // (v = 0.012 m/s) so the robot stops cleanly in front of a human
              // instead of micro-inching and braking repeatedly.
              const double check_v=std::max(v,.25);
              double px=x_,py=y_,a=yaw_,horizon=.6+check_v/std::min(accel_,packet_->max_accel);safe=true;
              for(double t=0;t<=horizon;t+=.04){if(!clear(px,py,a,.01)){safe=false;physical_reason_="corrected_command_collision_unknown_or_outside_scene";break;}a+=trial_w*.04;px+=check_v*std::cos(a)*.04;py+=check_v*std::sin(a)*.04;}
              if(safe){w=trial_w;break;}
            }
            v*=.5;w*=.5;safety_reductions++;
            if(distance>final_approach_radius_&&v<min_moving_speed_)break;
          }
          if(!safe||v<packet_->chassis_geometry[6]){v=w=0.;state_="WAITING_FOR_REPLAN";reason=physical_reason_.empty()?"no_safe_command_above_native_translation_threshold":physical_reason_;request_replan();}
          else{v_before_scale=v;
            w=navigation::limit(w,wmax_);
            state_="MOVING";start_aligned_=true;execution_time_+=dt;
            if(reverse_attempts_>0&&travel_-reverse_reset_travel_>=1.0){reverse_attempts_=0;reverse_reset_travel_=travel_;}}
          // Do not crawl indefinitely below min_moving_speed_ in cruise mode:
          // stop linear motion and request a clearer/faster replan instead.
          const double min_forward=std::max(min_moving_speed_,packet_->chassis_geometry[6]+.002);
          if(state_=="MOVING"&&distance>final_approach_radius_&&v<min_forward){
            if(!low_speed_since_)low_speed_since_=began;
            if(began-low_speed_since_>=low_speed_scrub_guard_s_){
              v=w=0.;state_="WAITING_FOR_REPLAN";
              reason="cruise_speed_below_minimum_threshold";physical_reason_=reason;
              low_speed_since_=0.;request_replan();
            }
          }else low_speed_since_=0.;
          if(remaining<.5)request_replan();
        }
        if(state_!="FINAL_APPROACH")final_approach_active_=false;
        if(state_!="ALIGN"&&state_!="ROTATING")pivot_dir_=0;
        if((state_=="ALIGN"||state_=="ROTATING")&&std::abs(wrap(desired-yaw_))<=align_tol){
          state_="SETTLING";settle_since_=0.;settle_enter_s_=began;v=w=0.;reason="settling_after_rotation";
        }
        }
        if(w!=0&&v==0){
          double feasible=0.;
          if(model_limited(0.,w,feasible)&&std::abs(feasible)>.01){w=feasible;}
          else{w=0;state_="WAITING_FOR_STEERING";reason="native wheel model rejected rotation at all feasible rates";}
        }
      }
      }
    }
    w=navigation::limit(w,wmax_);v_=v;w_=w;
    if(state_=="WAITING_FOR_REPLAN")last_blocked_reason_=reason;
    record_history(began);
    if(state_!=last_state_){last_state_=state_;state_since_=began;}
    auto add_limit=[&](const char* r){
      if(!speed_limit_sources.empty())speed_limit_sources+=",";
      speed_limit_sources+=r;
    };
    if(speed_safety_scale_enabled_&&steering_scale_<.999)add_limit("steering_scale");
    if(safety_reductions>0)add_limit("collision_reduction");
    if(admitted_speed_>0.&&admitted_speed_<vmax_-.001)add_limit("admitted_speed");
    if(state_=="FINAL_APPROACH")add_limit("final_approach");
    if(v_before_scale>0.&&v<v_before_scale-1e-4)add_limit("downstream_scale");
    if(!speed_safety_scale_enabled_)add_limit("speed_safety_disabled");
    if(speed_limit_sources.empty())speed_limit_sources="none";
    if(!manual_&&!dry_){geometry_msgs::msg::Twist out;out.linear.x=v;out.angular.z=w;cmd_->publish(out);}
    double cost=(steady()-began)*1000.;max_cost_=std::max(max_cost_,cost);
    if(began-status_at_>=.1){status_at_=began;
      Progress progress;progress.header.stamp=now();progress.header.frame_id="map";progress.goal_id=goal_;progress.revision=revision_;progress.task_id=task_;progress.phase=state_;progress.arc_length=progress_s_;progress.execution_time=execution_time_;progress.parameter_time=curve_?curve_->at(progress_s_).u:0.;progress.frozen=state_!="MOVING";progress_->publish(progress);
      nav_msgs::msg::Path path;path.header=progress.header;
      if(curve_&&fresh()){
        // Publish the complete remaining local B-spline, not a 1 m preview.
        // Cap sampling so a pathological curve cannot flood the GUI.
        const double span=std::max(0.,curve_->length-progress_s_);
        const double step=std::max(.02,span/1200.);
        for(double s=progress_s_;s<=curve_->length+1e-6;s+=step){
        auto p=curve_->at(s);geometry_msgs::msg::PoseStamped pose;pose.header=path.header;
        pose.pose.position.x=p.x;pose.pose.position.y=p.y;pose.pose.position.z=p.z;
          pose.pose.orientation.z=std::sin(p.yaw/2);pose.pose.orientation.w=std::cos(p.yaw/2);path.poses.push_back(pose);
        }
      }
      display_->publish(path);
      std::string safety_state = estop_?"ESTOP":(manual_?"MANUAL_OVERRIDE":
        ((state_=="SAFETY_HOLD"||state_=="WAITING_FOR_FEEDBACK"||
          state_=="WAITING_FOR_REPLAN"||state_=="WAITING_FOR_STEERING")?"SAFETY_HOLD":
         ((state_=="SETTLING"||state_=="VERIFYING_ARRIVAL"||state_=="ALIGN"||state_=="ROTATING")?"BRAKE":"NORMAL")));
      std::string motion_state = state_=="REVERSING"?"REVERSE":
        ((state_=="ALIGN"||state_=="ROTATING")?"PIVOT":
         (state_=="FINAL_APPROACH"?"FINAL_APPROACH":
          (state_=="MOVING"?(std::abs(w)>.03?"CURVE":"STRAIGHT"):"STOP")));
      std::string planning_state = (state_=="WAITING_FOR_REPLAN"||state_=="FAILED")?"BLOCKED":
        ((state_=="WAITING_FOR_FEEDBACK"||state_=="WAITING_FOR_STEERING")?"PLAN_HOLD":
         (state_=="FINAL_APPROACH"||state_=="SETTLING"?"TERMINAL_HOLD":
          (state_=="MOVING"?"LOCAL":"IDLE")));
      Json status={{"task_id",task_},{"goal_id",goal_},{"state",state_},{"phase",state_},{"task_state",state_=="ARRIVED"?"ARRIVED":state_=="MOVING"||state_=="ROTATING"?"RUNNING":state_=="MANUAL_OVERRIDE"?"MANUAL":state_=="DISABLED"?"STARTING":"WAITING"},
        {"start_ready",ready},{"reason",reason},{"enabled",enabled_},{"paused",paused_},{"manual_override",manual_},{"speed_mps",v},{"cmd_linear_x",v},{"cmd_linear_y",0.},{"cmd_angular_z",w},{"travelled_m",travel_},{"distance_source","stamped_odom_twist_integral"},{"control_last_ms",cost},{"control_max_ms",max_cost_},{"trajectory_id",revision_},{"execution_time_s",execution_time_},{"progress_s",progress_s_},{"cbf_enabled",false},{"motion_mode",(state_=="ALIGN"||state_=="ROTATING")?"pivot":(state_=="FINAL_APPROACH"?"final_approach":"translate")},{"speed_limit_sources",speed_limit_sources},{"safety_state",safety_state},{"safety_hold_reason",safety_hold_reason_},{"motion_state",motion_state},{"planning_state",planning_state},{"state_elapsed_s",state_since_>0.?steady()-state_since_:0.},{"dry_run",dry_},{"max_speed_mps",vmax_},{"path_source_used","scan"},{"physical_reason",physical_reason_},{"legs_home",legs_home()},{"leg_home_gate",leg_gate_enabled_},{"latest_packet_valid",latest_packet_valid_},{"speed_safety_scale_enabled",speed_safety_scale_enabled_},{"low_speed_scrub_guard_s",low_speed_scrub_guard_s_},{"final_approach_radius_m",final_approach_radius_},{"final_approach_stop_m",final_approach_stop_},{"settle_elapsed_s",settle_enter_s_>0.?steady()-settle_enter_s_:0.},{"final_approach_speed_mps",final_approach_speed_},{"reverse_attempts",reverse_attempts_},{"reverse_active",reverse_active_},{"last_blocked_reason",last_blocked_reason_},{"steering_scale",steering_scale_},{"v_before_scale",v_before_scale},{"safety_reductions",safety_reductions},{"pose_x",x_},{"pose_y",y_},{"pose_yaw",yaw_},{"yaw_error_rad",curve_?wrap(curve_->at(progress_s_).yaw-yaw_):0.},{"goal_yaw_error_rad",(final_yaw_&&packet_)?wrap(packet_->goal_yaw-yaw_):0.}};
      status["trajectory_rejection"]=packet_rejection_;
      status["admitted_speed_mps"]=admitted_speed_;
      status["navigation_degraded"]=admitted_speed_>0. && admitted_speed_<vmax_;
      if(packet_)status["goal_distance_m"]=std::hypot(packet_->goal.x-x_,packet_->goal.y-y_);
      std_msgs::msg::String out;out.data=status.dump();status_->publish(out);
      if(finished_&&state_since_>0.&&began-state_since_>=1.0){
        RCLCPP_INFO(get_logger(),"Terminal state %s published; exiting controller cleanly",state_.c_str());
        rclcpp::shutdown();
      }
    }
  }
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr display_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_;rmw_gid_t own_gid_{};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Publisher<Progress>::SharedPtr progress_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr reached_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr replan_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr observe_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_,odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_,enable_sub_,pause_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr dec_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr health_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<Packet>::SharedPtr trajectory_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unordered_set<uint64_t> observed_free_;
  std::unordered_map<uint64_t,std::vector<std::array<double,2>>>scene_grid_;
  std::shared_ptr<navigation::Curve>curve_;Packet::ConstSharedPtr packet_;Json health_,packet_rejection_=Json::object();
  wheel_control::WheelArray q_{},qd_{};std::array<double,4> joint_rx_{};std::array<int64_t,4> joint_stamp_{};
  bool speed_safety_scale_enabled_=true;bool leg_gate_enabled_=true;double leg_home_tolerance_=.05;double leg_rx_=-1.;
  double low_speed_scrub_guard_s_=1.0;double low_speed_since_=0.;double min_moving_speed_=.05;int pivot_dir_=0;
  double final_approach_radius_=.50,final_approach_stop_=.12,final_approach_speed_=.08;
  double settle_enter_s_=0.;
  double watchdog_feedback_s_=3.0,watchdog_replan_s_=3.0,watchdog_steering_s_=2.0,watchdog_align_s_=5.0;
  std::string safety_hold_reason_;double safety_release_since_=0.;
  bool final_approach_active_=false;double final_approach_best_=1e9,final_approach_bad_since_=0.;
  bool reverse_enabled_=true,reverse_active_=false,reverse_require_observed_=true,reverse_use_history_=true;int reverse_attempts_=0,reverse_max_attempts_=2;
  double reverse_history_age_s_=10.0;
  struct HistoryPose{double x,y,yaw,t;};std::deque<HistoryPose>history_;
  double reverse_speed_=.08,reverse_distance_=1.50,reverse_timeout_=15.0;
  double reverse_start_x_=0.,reverse_start_y_=0.,reverse_start_s_=0.,reverse_reset_travel_=0.;
  std::string last_blocked_reason_,last_state_;double state_since_=0.;
  std::vector<std::string> leg_home_names_;std::vector<double> leg_home_positions_;
  std::unordered_map<std::string,double> leg_positions_;
  bool coordinate_fault_=false,hold_=false,arrival_zone_=false,latest_packet_valid_=false;
  bool enabled_=false,dry_=false,manual_=false,estop_=false,finished_=false,arrived_=false,have_pose_=false,final_yaw_=false,expected_=false,follow_updates_=false,paused_=false,start_aligned_=false;
  int dec_=0;uint64_t goal_=0;int64_t revision_=-1,pose_stamp_=0,odom_stamp_=0,arrival_stamp_=0;
  double x_=0,y_=0,z_=0,yaw_=0,v_=0,w_=0,vmax_,wmax_,accel_,expected_x_,expected_y_;
  double progress_s_=0,execution_time_=0,packet_rx_=0,pose_rx_=0,odom_rx_=0,health_rx_=0,measured_v_=0,measured_w_=0,travel_=0;
  double last_manual_=-100,last_tick_=0,status_at_=0,replan_at_=0,arrival_since_=0,max_cost_=0,steering_scale_=1.,admitted_speed_=0.,final_yaw_tolerance_=0.10472,settle_since_=0.;std::string physical_reason_,terminal_reason_,map_id_,feedback_reason_,task_,state_="DISABLED";
};
static volatile std::sig_atomic_t stop_requested=0;
static void request_stop(int){stop_requested=1;}
int main(int argc,char**argv){
  rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT,request_stop);std::signal(SIGTERM,request_stop);
  try{
    auto node=std::make_shared<NavigationController>();
    rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
    while(rclcpp::ok()&&!stop_requested){executor.spin_some(std::chrono::milliseconds(5));std::this_thread::sleep_for(std::chrono::milliseconds(1));}
    node->stop();executor.remove_node(node);
  }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
