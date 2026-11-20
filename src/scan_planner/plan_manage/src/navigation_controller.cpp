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
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <scan_planner_msgs/msg/navigation_trajectory.hpp>
#include <scan_planner_msgs/msg/execution_progress.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <cstring>
#include <optional>
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
    final_yaw_tolerance_=declare_parameter("final_yaw_tolerance_rad",0.10472);
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
    fresh_limit("arrival.confirm_s",3.0);

    if(task_.empty()||!std::isfinite(vmax_)||vmax_<.01||vmax_>.8||!std::isfinite(wmax_)||wmax_<=0||wmax_>.5||!std::isfinite(accel_)||accel_<=0)throw std::invalid_argument("invalid navigation task parameters");
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
      for(size_t i=0;i<m->name.size()&&i<m->position.size();i++)for(size_t j=0;j<4;j++)if(m->name[i]==names[j]&&std::isfinite(m->position[i])){int64_t stamp=rclcpp::Time(m->header.stamp).nanoseconds();
        if(stamp<=joint_stamp_[j])continue;
        double rate=std::numeric_limits<double>::quiet_NaN();
        if(i<m->velocity.size()&&std::isfinite(m->velocity[i]))rate=m->velocity[i];
        else if(joint_stamp_[j]&&stamp-joint_stamp_[j]<=500000000)rate=(m->position[i]-q_[j])/((stamp-joint_stamp_[j])/1e9);
        q_[j]=m->position[i];joint_stamp_[j]=stamp;
        if(std::isfinite(rate)){qd_[j]=rate;joint_rx_[j]=steady();}}
    });
    auto stopq=rclcpp::QoS(10).reliable().transient_local();
    estop_sub_=create_subscription<std_msgs::msg::Bool>("/estop",stopq,[this](std_msgs::msg::Bool::ConstSharedPtr m){estop_|=m->data;});
    dec_sub_=create_subscription<std_msgs::msg::UInt8>("/chassis_dec_stop/state",stopq,[this](std_msgs::msg::UInt8::ConstSharedPtr m){dec_=m->data;});
    enable_sub_=create_subscription<std_msgs::msg::Bool>("/n01/navigation/enable",10,[this](std_msgs::msg::Bool::ConstSharedPtr m){
      if(m->data&&!enabled_&&!finished_&&steady()-last_manual_>1.)manual_=false;
      enabled_=m->data; if(!enabled_)v_=w_=0.;
    });
    goal_sub_=create_subscription<geometry_msgs::msg::PoseStamped>("/n01/live_navigation/goal_gated",10,[this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m){
      uint64_t id=rclcpp::Time(m->header.stamp).nanoseconds();
      if(coordinate_fault_||m->header.frame_id!="map"||id<=goal_)return;
      if(goal_&&!follow_updates_){finished_=true;state_="GOAL_CHANGED";terminal_reason_="accepted_goal_changed";return;}
      goal_=id;curve_.reset();packet_.reset();arrived_=false;finished_=false;arrival_zone_=false;arrival_since_=0.;execution_time_=0.;
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
    if(!goal_||m->goal_id!=goal_){goal_=m->goal_id;curve_.reset();arrived_=finished_=false;arrival_zone_=false;execution_time_=0.;}
    if(!m->valid){
      // Fresh scene/model are useful even when the proposed geometry fails.
      // Keep the current curve and its admitted speed; never adopt this spline.
      adopt_scene(m);packet_rx_=steady();request_replan();return;
    }
    if(m->hold_position){
      if(!have_pose_||std::hypot(m->goal.x-x_,m->goal.y-y_)>.10)return;
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
    if(!f.valid||std::abs(f.vx-v)>std::max(1e-4,.02*std::abs(v))||std::abs(f.vy)>.005||std::abs(f.omega-w)>std::max(1e-4,.02*std::abs(w))){physical_reason_="native_ik_fk_twist_mismatch";return false;}
    double max_change=0.;for(int j=0;j<4;j++){if(std::abs(q_[j])>p[7]+1e-6||std::abs(target[j])>p[7]+1e-6||std::abs(drive[j])>packet_->max_wheel_rate||std::abs(qd_[j])>packet_->max_steering_rate*2.){physical_reason_="live_wheel_angle_speed_or_steering_rate_limit";return false;}max_change=std::max(max_change,std::abs(target[j]-q_[j]));}
    // Slew mismatch slows motion; native control handles steering transitions.
    double rate=0.;for(double r:qd_)rate=std::max(rate,std::abs(r));
    steering_scale_=std::min(std::clamp(1.-max_change/3.141592653589793,.15,1.),
      rate>packet_->max_steering_rate?packet_->max_steering_rate/rate:1.);return true;
  }
  bool measured_arrival(int64_t after){
    if(steady()-health_rx_>1.5)return false;
    try{
      auto b=health_.at("bridge");
      if(!b.contains("pose_prediction"))return pose_stamp_>=after;
      auto p=b.at("pose_prediction");if(!p.at("fault").is_null()&&p.at("fault")!=false&&p.at("fault")!="")return false;
      if(p.at("source_stamp_ns").get<int64_t>()<after)return false;
      auto q=p.at("source_pose");if(std::hypot(q.at("x").get<double>()-packet_->goal.x,q.at("y").get<double>()-packet_->goal.y)>.10)return false;
      if(final_yaw_){auto a=q.at("quaternion");double x=a[0],y=a[1],z=a[2],w=a[3];double yaw=std::atan2(2*(w*z+x*y),1-2*(y*y+z*z));if(std::abs(wrap(yaw-packet_->goal_yaw))>final_yaw_tolerance_)return false;}
      return true;
    }catch(...){return false;}
  }
  void tick(){
    const double began=steady(),dt=last_tick_?std::clamp(began-last_tick_,0.,.1):.02;last_tick_=began;
    bool ready=fresh()&&!manual_&&!estop_&&dec_==0&&!finished_;
    double v=0,w=0;std::string reason=finished_?terminal_reason_:(ready?"":feedback_reason_);state_=finished_?state_:"DISABLED";
    if(manual_){state_="MANUAL_OVERRIDE";reason="external command traffic; explicit resume required";}
    else if(estop_||dec_!=0){state_="ESTOP";reason="chassis stop or emergency stop";}
    else if(!finished_&&enabled_){
      if(!ready){state_="WAITING_FOR_FEEDBACK";reason=feedback_reason_;request_replan();}
      else {
        double measured=curve_?curve_->nearest(x_,y_):0.;progress_s_=std::max(progress_s_,measured);
        auto p=curve_?curve_->at(progress_s_):navigation::Sample{x_,y_,z_,yaw_,0.,0.};double distance=std::hypot(packet_->goal.x-x_,packet_->goal.y-y_);
        if(distance<=.08)arrival_zone_=true;
        if(distance>=.10)arrival_zone_=false;
        bool at_goal=arrival_zone_;
        const auto end=curve_?curve_->samples.back():p;
        const bool terminal=std::hypot(end.x-packet_->goal.x,end.y-packet_->goal.y)<=.12;
        // Terminal homing closes cross-track error instead of exhausting arc
        // length a millimetre outside the arrival zone.
        if(terminal&&distance<.35&&!at_goal)p.yaw=std::atan2(packet_->goal.y-y_,packet_->goal.x-x_);
        double yaw_error=wrap(p.yaw-yaw_);
        double desired=at_goal&&final_yaw_?packet_->goal_yaw:p.yaw;
        if((at_goal&&final_yaw_&&std::abs(wrap(desired-yaw_))>final_yaw_tolerance_)||(!at_goal&&(std::abs(yaw_error)>.85||std::min({vmax_,packet_->max_speed,admitted_speed_})*std::max(0.,std::cos(yaw_error))<=packet_->chassis_geometry[6]+.001))){
          state_="ALIGN";
          if(measured_v_>.01||std::abs(v_)>.005){reason="braking before alignment";}
          else if(sweep(desired)){w=navigation::limit(1.2*wrap(desired-yaw_),wmax_);state_="ROTATING";}
          else {reason=packet_->rotation_observed?"rotation swept volume occupied; SCAN replan requested":"waiting for full-azimuth scan";request_replan();}
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
          try{auto bridge=health_.at("bridge");if(bridge.contains("pose_prediction")&&bridge["pose_prediction"].value("source_age_s",0.)>.7)cap=std::min(cap,.03);}catch(...){}
          double stop=terminal?std::max(0.,distance-.04):remaining;
          cap=navigation::braking_speed(cap,std::min(accel_,packet_->max_accel),std::max(0.,stop-.02));
          auto ahead=curve_->at(std::min(curve_->length,progress_s_+.25));
          if(terminal&&distance<.35){ahead.x=packet_->goal.x;ahead.y=packet_->goal.y;ahead.yaw=p.yaw;}
          auto command=navigation::feedback(p,ahead,x_,y_,yaw_,cap,wmax_);
          v=std::min(command.v,std::max(v_+accel_*dt,packet_->chassis_geometry[6]+.002));w=command.w;
          // Check the actual feedback-corrected command, not only the planned tangent.
          bool safe=false;
          for(int reduction=0;reduction<7;reduction++){
            if(model(v,w)){
              double px=x_,py=y_,a=yaw_,horizon=.6+v/std::min(accel_,packet_->max_accel);safe=true;
              for(double t=0;t<=horizon;t+=.04){if(!clear(px,py,a,.01)){safe=false;physical_reason_="corrected_command_collision_unknown_or_outside_scene";break;}a+=w*.04;px+=v*std::cos(a)*.04;py+=v*std::sin(a)*.04;}
              if(safe)break;
            }v*=.5;w*=.5;
          }
          if(!safe||v<packet_->chassis_geometry[6]){v=w=0.;state_="WAITING_FOR_REPLAN";reason=physical_reason_.empty()?"no_safe_command_above_native_translation_threshold":physical_reason_;request_replan();}
          else{double scale=std::min(1.,std::max(steering_scale_,(packet_->chassis_geometry[6]+.002)/v));v*=scale;w*=scale;state_="MOVING";execution_time_+=dt;}
          if(remaining<.5)request_replan();
        }
        if(w!=0&&v==0&&!model(v,w)){w=0;state_="WAITING_FOR_STEERING";reason="native wheel model rejected rotation";}
      }
    }
    w=navigation::limit(w,wmax_);v_=v;w_=w;
    if(!manual_&&!dry_){geometry_msgs::msg::Twist out;out.linear.x=v;out.angular.z=w;cmd_->publish(out);}
    double cost=(steady()-began)*1000.;max_cost_=std::max(max_cost_,cost);
    if(began-status_at_>=.1){status_at_=began;
      Progress progress;progress.header.stamp=now();progress.header.frame_id="map";progress.goal_id=goal_;progress.revision=revision_;progress.task_id=task_;progress.phase=state_;progress.arc_length=progress_s_;progress.execution_time=execution_time_;progress.parameter_time=curve_?curve_->at(progress_s_).u:0.;progress.frozen=state_!="MOVING";progress_->publish(progress);
      nav_msgs::msg::Path path;path.header=progress.header;
      if(curve_&&fresh())for(double s=progress_s_;s<=std::min(curve_->length,progress_s_+1.)+1e-6;s+=.02){
        auto p=curve_->at(s);geometry_msgs::msg::PoseStamped pose;pose.header=path.header;
        pose.pose.position.x=p.x;pose.pose.position.y=p.y;pose.pose.position.z=p.z;
        pose.pose.orientation.z=std::sin(p.yaw/2);pose.pose.orientation.w=std::cos(p.yaw/2);path.poses.push_back(pose);
      }
      display_->publish(path);
      Json status={{"task_id",task_},{"goal_id",goal_},{"state",state_},{"phase",state_},{"task_state",state_=="ARRIVED"?"ARRIVED":state_=="MOVING"||state_=="ROTATING"?"RUNNING":state_=="MANUAL_OVERRIDE"?"MANUAL":state_=="DISABLED"?"STARTING":"WAITING"},
        {"start_ready",ready},{"reason",reason},{"enabled",enabled_},{"manual_override",manual_},{"speed_mps",v},{"cmd_linear_x",v},{"cmd_linear_y",0.},{"cmd_angular_z",w},{"travelled_m",travel_},{"distance_source","stamped_odom_twist_integral"},{"control_last_ms",cost},{"control_max_ms",max_cost_},{"trajectory_id",revision_},{"execution_time_s",execution_time_},{"progress_s",progress_s_},{"cbf_enabled",false},{"motion_mode","translate"},{"dry_run",dry_},{"max_speed_mps",vmax_},{"path_source_used","scan"},{"pose_x",x_},{"pose_y",y_},{"pose_yaw",yaw_},{"yaw_error_rad",curve_?wrap(curve_->at(progress_s_).yaw-yaw_):0.}};
      status["trajectory_rejection"]=packet_rejection_;
      status["admitted_speed_mps"]=admitted_speed_;
      status["navigation_degraded"]=admitted_speed_>0. && admitted_speed_<vmax_;
      if(packet_)status["goal_distance_m"]=std::hypot(packet_->goal.x-x_,packet_->goal.y-y_);
      std_msgs::msg::String out;out.data=status.dump();status_->publish(out);
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
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_,enable_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr dec_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr health_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<Packet>::SharedPtr trajectory_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unordered_set<uint64_t> observed_free_;
  std::unordered_map<uint64_t,std::vector<std::array<double,2>>>scene_grid_;
  std::shared_ptr<navigation::Curve>curve_;Packet::ConstSharedPtr packet_;Json health_,packet_rejection_=Json::object();
  wheel_control::WheelArray q_{},qd_{};std::array<double,4> joint_rx_{};std::array<int64_t,4> joint_stamp_{};
  bool coordinate_fault_=false,hold_=false,arrival_zone_=false;
  bool enabled_=false,dry_=false,manual_=false,estop_=false,finished_=false,arrived_=false,have_pose_=false,final_yaw_=false,expected_=false,follow_updates_=false;
  int dec_=-1;uint64_t goal_=0;int64_t revision_=-1,pose_stamp_=0,odom_stamp_=0,arrival_stamp_=0;
  double x_=0,y_=0,z_=0,yaw_=0,v_=0,w_=0,vmax_,wmax_,accel_,expected_x_,expected_y_;
  double progress_s_=0,execution_time_=0,packet_rx_=0,pose_rx_=0,odom_rx_=0,health_rx_=0,measured_v_=0,measured_w_=0,travel_=0;
  double last_manual_=-100,last_tick_=0,status_at_=0,replan_at_=0,arrival_since_=0,max_cost_=0,steering_scale_=1.,admitted_speed_=0.,final_yaw_tolerance_=0.10472;std::string physical_reason_,terminal_reason_,map_id_,feedback_reason_,task_,state_="DISABLED";
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
