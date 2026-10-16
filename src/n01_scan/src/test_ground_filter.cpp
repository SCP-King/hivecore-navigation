#include <cmath>
#include <cstdio>
#include <vector>

#include "ground_filter.hpp"

namespace {
int g_failures = 0;
#define CHECK(cond) do { \
  if (!(cond)) { std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); ++g_failures; } \
} while (0)

n01_scan::GroundParams params() {
  n01_scan::GroundParams p;
  p.mode = "plane_ransac";
  p.subsample_stride = 1;
  p.max_points = 2000;
  p.iterations = 64;
  p.lcg_seed = 1337;
  p.inlier_dist_m = 0.04;
  p.normal_max_cos = 0.8660254;
  p.min_inlier_frac = 0.25;
  p.fallback = "robust_z_percentile";
  p.fallback_percentile_q = 2.0;
  p.fallback_radius_m = 3.0;
  return p;
}

void test_plane_and_obstacle() {
  std::vector<n01_scan::Point3> points;
  // Flat floor at z=0.02; noise-free so the Jacobi refit stays vertical.
  // (A checkerboard z-wobble would correlate with x/y and legitimately tilt the
  // best-fit normal, tripping the near-horizontal check and the fallback.)
  for (int ix = -5; ix <= 5; ++ix) {
    for (int iy = -5; iy <= 5; ++iy) {
      points.push_back({0.2 * ix, 0.2 * iy, 0.02});
    }
  }
  const std::size_t obstacle = points.size();
  points.push_back({0.8, 0.4, 0.35});
  points.push_back({0.9, 0.4, 0.35});
  points.push_back({0.8, 0.5, 0.35});

  n01_scan::GroundFilter filter(params());
  const auto result = filter.classify(points, {0.0, 0.0, 0.9});
  CHECK(result.valid);
  CHECK(result.method == "plane_ransac");
  CHECK(result.is_ground.size() == points.size());
  CHECK(std::fabs(result.ground_z - 0.02) < 0.04);
  for (std::size_t i = 0; i < obstacle; ++i) CHECK(result.is_ground[i]);
  for (std::size_t i = obstacle; i < points.size(); ++i) CHECK(!result.is_ground[i]);
}

void test_deterministic_result() {
  std::vector<n01_scan::Point3> points;
  for (int i = 0; i < 40; ++i) {
    const double x = -2.0 + 0.1 * (i % 20);
    const double y = -1.0 + 0.1 * (i / 20);
    points.push_back({x, y, 0.02});
  }
  points.push_back({0.0, 0.0, 0.5});
  n01_scan::GroundFilter filter(params());
  const auto a = filter.classify(points, {0.0, 0.0, 0.9});
  const auto b = filter.classify(points, {0.0, 0.0, 0.9});
  CHECK(a.valid && b.valid);
  CHECK(a.method == b.method);
  CHECK(a.is_ground == b.is_ground);
  CHECK(a.ground_z == b.ground_z);
}

void test_fallback_and_invalid_input() {
  auto p = params();
  p.iterations = 1;
  std::vector<n01_scan::Point3> points;
  for (int i = 0; i < 20; ++i) {
    points.push_back({0.1 * i, 0.0, i == 19 ? 0.5 : 0.0});
  }
  // This is a valid, deterministic fallback input if RANSAC cannot form a
  // qualifying plane; the low percentile identifies the floor.
  n01_scan::GroundFilter filter(p);
  const auto result = filter.classify(points, {0.0, 0.0, 0.9});
  CHECK(result.valid);
  CHECK(result.method == "plane_ransac" || result.method == "robust_z_percentile");
  CHECK(result.is_ground.size() == points.size());

  const auto empty = filter.classify({}, {0.0, 0.0, 0.9});
  CHECK(!empty.valid);
  CHECK(empty.method == "empty");
}
}  // namespace

int main() {
  test_plane_and_obstacle();
  test_deterministic_result();
  test_fallback_and_invalid_input();
  return g_failures ? 1 : 0;
}
