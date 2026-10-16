#pragma once

// Localization-continuity policy for the map_localized occupancy node.
//
// Decides, per point-cloud frame, whether to update the sliding map, skip it,
// or reset it first. Extracted as a pure class so the tricky recovery/jump
// logic is unit-testable without spinning up a ROS graph:
//  - TF failure (or frame mismatch): the frame is SKIPPED -- no raycast, no
//    stale-map publish. After tf_loss_frames_before_reset consecutive
//    failures the node is declared LOST (updates stay frozen).
//  - First successful transform while LOST: RESET then update (the pose
//    continuity is unknown, so stale cells would be wrong). A single transient
//    miss does NOT wipe the map: with threshold >= 2 the node freezes one
//    frame and resumes on the next good frame without a reset.
//  - Relocalization jump: if the horizontal distance between two consecutive
//    PROCESSED origins exceeds relocalization_jump_m AND is implausible for
//    the stamp delta (distance > relocalization_max_speed_mps * dt), the map
//    is RESET and recentered before the jumping frame's rays are inserted.
//    A pure-yaw discontinuity does not reset. Time-awareness prevents a 2 m
//    legit motion at a degraded frame rate from wiping a good map. Note in the
//    delivery docs: a genuine loop-closure pose correction larger than the
//    threshold discards accumulated cells by design (conservative; world-
//    anchored keys are otherwise robust to small corrections).

#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace n01_scan {

struct MapLocalizedPolicy {
  enum class Action { Skip, Update, ResetThenUpdate };

  double jump_m = 2.0;            // m; horizontal distance threshold
  double max_speed_mps = 5.0;     // m/s; max plausible robot speed for jump timing
  int loss_threshold = 1;         // consecutive TF failures before declaring LOST

  int loss_count = 0;
  bool lost = false;
  bool have_prev = false;
  double prev_x = 0.0, prev_y = 0.0;
  rclcpp::Time prev_stamp{0, 0, RCL_ROS_TIME};
  std::string reason;

  Action decide(bool tf_ok, double ox, double oy, const rclcpp::Time &stamp) {
    if (!tf_ok) {
      ++loss_count;
      if (loss_count >= loss_threshold) lost = true;
      reason = "TF unavailable; frame skipped";
      return Action::Skip;
    }
    loss_count = 0;
    const bool was_lost = lost;
    lost = false;

    bool jump = false;
    if (have_prev) {
      const double dt = (stamp - prev_stamp).seconds();
      const double dist = std::hypot(ox - prev_x, oy - prev_y);
      jump = dist > jump_m && dist > max_speed_mps * std::max(dt, 0.0);
    }
    prev_x = ox;
    prev_y = oy;
    prev_stamp = stamp;
    have_prev = true;

    if (was_lost) {
      reason = "recovered after " + std::to_string(loss_threshold) +
               " TF loss frame(s): map reset and recentered at recovered origin";
      return Action::ResetThenUpdate;
    }
    if (jump) {
      reason = "relocalization jump detected: map reset and recentered";
      return Action::ResetThenUpdate;
    }
    reason = "update";
    return Action::Update;
  }
};

}  // namespace n01_scan