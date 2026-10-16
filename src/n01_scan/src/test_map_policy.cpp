#include <cmath>
#include <cstdio>

#include "map_policy.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {
int g_failures = 0;
#define CHECK(cond) do { \
  if (!(cond)) { std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); ++g_failures; } \
} while (0)

using n01_scan::MapLocalizedPolicy;
using Action = MapLocalizedPolicy::Action;

rclcpp::Time t(double sec) {
  return rclcpp::Time(static_cast<int32_t>(sec), static_cast<uint32_t>((sec - std::floor(sec)) * 1e9),
                      RCL_ROS_TIME);
}

void test_normal_updates() {
  MapLocalizedPolicy p;
  p.loss_threshold = 1;
  CHECK(p.decide(true, 0.0, 0.0, t(100.0)) == Action::Update);
  CHECK(p.decide(true, 0.2, 0.1, t(100.05)) == Action::Update);
  CHECK(!p.lost);
  CHECK(p.loss_count == 0);
}

void test_tf_loss_and_recovery() {
  MapLocalizedPolicy p;
  p.loss_threshold = 2;
  CHECK(p.decide(true, 0.0, 0.0, t(100.0)) == Action::Update);
  // One transient miss with threshold 2 freezes one frame but does NOT reset.
  CHECK(p.decide(false, 0.0, 0.0, t(100.05)) == Action::Skip);
  CHECK(!p.lost);
  CHECK(p.decide(true, 0.0, 0.0, t(100.10)) == Action::Update);
  // Two consecutive misses declare LOST; the next good frame resets+updates.
  CHECK(p.decide(false, 0.0, 0.0, t(100.15)) == Action::Skip);
  CHECK(p.decide(false, 0.0, 0.0, t(100.20)) == Action::Skip);
  CHECK(p.lost);
  CHECK(p.decide(true, 0.0, 0.0, t(100.25)) == Action::ResetThenUpdate);
  CHECK(!p.lost);
}

void test_relocalization_jump() {
  MapLocalizedPolicy p;
  p.jump_m = 2.0;
  p.max_speed_mps = 5.0;
  CHECK(p.decide(true, 0.0, 0.0, t(100.0)) == Action::Update);
  // 5 m jump in 100 ms is implausible for a 5 m/s base: reset+recenter.
  CHECK(p.decide(true, 5.0, 0.0, t(100.1)) == Action::ResetThenUpdate);
}

void test_time_aware_no_jump() {
  MapLocalizedPolicy p;
  p.jump_m = 2.0;
  p.max_speed_mps = 5.0;
  CHECK(p.decide(true, 0.0, 0.0, t(100.0)) == Action::Update);
  // 10 m over 2 s is exactly the speed limit; NOT a jump.
  CHECK(p.decide(true, 10.0, 0.0, t(102.0)) == Action::Update);
  CHECK(!p.lost);
}

void test_pure_yaw_no_reset() {
  MapLocalizedPolicy p;
  p.jump_m = 0.1;
  p.max_speed_mps = 5.0;
  CHECK(p.decide(true, 0.0, 0.0, t(100.0)) == Action::Update);
  // Only the horizontal position is compared; same xy is never a jump.
  CHECK(p.decide(true, 0.0, 0.0, t(100.1)) == Action::Update);
}

}  // namespace

int main() {
  test_normal_updates();
  test_tf_loss_and_recovery();
  test_relocalization_jump();
  test_time_aware_no_jump();
  test_pure_yaw_no_reset();
  if (g_failures) {
    std::fprintf(stderr, "%d check(s) failed\n", g_failures);
    return 1;
  }
  return 0;
}
