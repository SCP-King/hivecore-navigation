#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include <bspline_opt/uniform_bspline.h>
#include <traj_utils/polynomial_traj.h>

using std::vector;

namespace scan_planner
{

  class GlobalTrajData
  {
  private:
  public:
    PolynomialTraj global_traj_;
    // Piecewise-linear/rounded global reference. When non-empty it is the
    // authoritative global route; min-snap is not used for this mode.
    std::vector<Eigen::Vector3d> polyline_;
    std::vector<double> polyline_s_;
    double polyline_speed_{0.2};
    vector<UniformBspline> local_traj_;

    double global_duration_;
    rclcpp::Time global_start_time_;
    double local_start_time_, local_end_time_;
    double time_increase_;
    double last_time_inc_;
    double last_progress_time_;

    GlobalTrajData() : global_duration_(0.),local_start_time_(0.),local_end_time_(0.),time_increase_(0.),last_time_inc_(0.),last_progress_time_(0.) {}

    ~GlobalTrajData() {}

    bool localTrajReachTarget() { return fabs(local_end_time_ - global_duration_) < 0.1; }

    void setGlobalTraj(const PolynomialTraj &traj, const rclcpp::Time &time)
    {
      global_traj_ = traj;
      global_traj_.init();
      polyline_.clear();
      polyline_s_.clear();
      global_duration_ = global_traj_.getTimeSum();
      global_start_time_ = time;

      local_traj_.clear();
      local_start_time_ = -1;
      local_end_time_ = -1;
      time_increase_ = 0.0;
      last_time_inc_ = 0.0;
      last_progress_time_ = 0.0;
    }

    void setPolyline(const std::vector<Eigen::Vector3d>& points, double speed,
                     const rclcpp::Time &time)
    {
      polyline_ = points;
      polyline_s_.clear();
      polyline_s_.reserve(polyline_.size());
      double s = 0.0;
      polyline_s_.push_back(0.0);
      for (size_t i = 1; i < polyline_.size(); ++i) {
        s += (polyline_[i] - polyline_[i - 1]).norm();
        polyline_s_.push_back(s);
      }
      polyline_speed_ = std::max(0.05, speed);
      global_duration_ = s / polyline_speed_;
      global_start_time_ = time;
      local_traj_.clear();
      local_start_time_ = -1;
      local_end_time_ = -1;
      time_increase_ = 0.0;
      last_time_inc_ = 0.0;
      last_progress_time_ = 0.0;
    }

    bool hasPolyline() const { return polyline_.size() >= 2; }

    Eigen::Vector3d polylineAt(double t) const
    {
      if (!hasPolyline()) return Eigen::Vector3d::Zero();
      const double total = polyline_s_.back();
      const double s = std::clamp(t * polyline_speed_, 0.0, total);
      auto it = std::upper_bound(polyline_s_.begin(), polyline_s_.end(), s);
      size_t i = (it == polyline_s_.begin()) ? 0 : size_t(it - polyline_s_.begin() - 1);
      if (i + 1 >= polyline_.size()) return polyline_.back();
      const double denom = polyline_s_[i + 1] - polyline_s_[i];
      const double f = denom > 1e-12 ? (s - polyline_s_[i]) / denom : 0.0;
      return polyline_[i] + f * (polyline_[i + 1] - polyline_[i]);
    }

    Eigen::Vector3d polylineVelocity(double t) const
    {
      if (!hasPolyline()) return Eigen::Vector3d::Zero();
      const double total = polyline_s_.back();
      const double s = std::clamp(t * polyline_speed_, 0.0, total);
      auto it = std::upper_bound(polyline_s_.begin(), polyline_s_.end(), s);
      size_t i = (it == polyline_s_.begin()) ? 0 : size_t(it - polyline_s_.begin() - 1);
      if (i + 1 >= polyline_.size()) return Eigen::Vector3d::Zero();
      const Eigen::Vector3d d = polyline_[i + 1] - polyline_[i];
      if (d.norm() <= 1e-9) return Eigen::Vector3d::Zero();
      return d.normalized() * polyline_speed_;
    }

    void setLocalTraj(UniformBspline traj, double local_ts, double local_te, double time_inc)
    {
      local_traj_.resize(3);
      local_traj_[0] = traj;
      local_traj_[1] = local_traj_[0].getDerivative();
      local_traj_[2] = local_traj_[1].getDerivative();

      local_start_time_ = local_ts;
      local_end_time_ = local_te;
      global_duration_ += time_inc;
      time_increase_ += time_inc;
      last_time_inc_ = time_inc;
    }

    Eigen::Vector3d getPosition(double t)
    {
      if (hasPolyline()) return polylineAt(t);
      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluate(t - time_increase_ + last_time_inc_);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluate(t - time_increase_);
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[0].evaluateDeBoor(tm + t - local_start_time_);
      }
    }

    Eigen::Vector3d getVelocity(double t)
    {
      if (hasPolyline()) return polylineVelocity(t);
      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluateVel(t);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluateVel(t - time_increase_);
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[1].evaluateDeBoor(tm + t - local_start_time_);
      }
    }

    Eigen::Vector3d getAcceleration(double t)
    {
      if (hasPolyline()) return Eigen::Vector3d::Zero();
      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluateAcc(t);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluateAcc(t - time_increase_);
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[2].evaluateDeBoor(tm + t - local_start_time_);
      }
    }

    // get Bspline parameterization data of a local trajectory within a sphere
    // start_t: start time of the trajectory
    // dist_pt: distance between the discretized points
    void getTrajByRadius(const double &start_t, const double &des_radius, const double &dist_pt,
                         vector<Eigen::Vector3d> &point_set, vector<Eigen::Vector3d> &start_end_derivative,
                         double &dt, double &seg_duration)
    {
      double seg_length = 0.0; // length of the truncated segment
      double seg_time = 0.0;   // duration of the truncated segment
      double radius = 0.0;     // distance to the first point of the segment

      double delta = 0.2;
      Eigen::Vector3d first_pt = getPosition(start_t); // first point of the segment
      Eigen::Vector3d prev_pt = first_pt;              // previous point
      Eigen::Vector3d cur_pt;                          // current point

      // go forward until the traj exceed radius or global time

      while (radius < des_radius && seg_time < global_duration_ - start_t - 1e-3)
      {
        seg_time += delta;
        seg_time = min(seg_time, global_duration_ - start_t);

        cur_pt = getPosition(start_t + seg_time);
        seg_length += (cur_pt - prev_pt).norm();
        prev_pt = cur_pt;
        radius = (cur_pt - first_pt).norm();
      }

      // get parameterization dt by desired density of points
      int seg_num = floor(seg_length / dist_pt);

      // get outputs

      seg_duration = seg_time; // duration of the truncated segment
      dt = seg_time / seg_num; // time difference between two points

      for (double tp = 0.0; tp <= seg_time + 1e-4; tp += dt)
      {
        cur_pt = getPosition(start_t + tp);
        point_set.push_back(cur_pt);
      }

      start_end_derivative.push_back(getVelocity(start_t));
      start_end_derivative.push_back(getVelocity(start_t + seg_time));
      start_end_derivative.push_back(getAcceleration(start_t));
      start_end_derivative.push_back(getAcceleration(start_t + seg_time));
    }

    // get Bspline parameterization data of a fixed duration local trajectory
    // start_t: start time of the trajectory
    // duration: time length of the segment
    // seg_num: discretized the segment into *seg_num* parts
    void getTrajByDuration(double start_t, double duration, int seg_num,
                           vector<Eigen::Vector3d> &point_set,
                           vector<Eigen::Vector3d> &start_end_derivative, double &dt)
    {
      dt = duration / seg_num;
      Eigen::Vector3d cur_pt;
      for (double tp = 0.0; tp <= duration + 1e-4; tp += dt)
      {
        cur_pt = getPosition(start_t + tp);
        point_set.push_back(cur_pt);
      }

      start_end_derivative.push_back(getVelocity(start_t));
      start_end_derivative.push_back(getVelocity(start_t + duration));
      start_end_derivative.push_back(getAcceleration(start_t));
      start_end_derivative.push_back(getAcceleration(start_t + duration));
    }
  };

  struct PlanParameters
  {
    /* planning algorithm parameters */
    double max_vel_, max_acc_, max_jerk_; // physical limits
    double vel_tolerance_, acc_tolerance_;
    double ctrl_pt_dist;                  // distance between adjacient B-spline control points
    double feasibility_tolerance_;        // permitted ratio of vel/acc exceeding limits
    double planning_horizon_;
    double global_waypoint_spacing_;      // max spacing between global A* waypoints
    double min_turn_radius_{0.6};         // global/local curvature floor
    double max_yaw_rate_{0.45};           // 4WS yaw-rate limit

    /* processing time */
    double time_search_ = 0.0;
    double time_optimize_ = 0.0;
    double time_adjust_ = 0.0;
  };

  struct LocalTrajData
  {
    /* info of generated traj */

    int traj_id_;
    double duration_;
    double global_time_offset; // This is because when the local traj finished and is going to switch back to the global traj, the global traj time is no longer matches the world time.
    rclcpp::Time start_time_;
    Eigen::Vector3d start_pos_;
    UniformBspline position_traj_, velocity_traj_, acceleration_traj_;
  };

} // namespace scan_planner

#endif
