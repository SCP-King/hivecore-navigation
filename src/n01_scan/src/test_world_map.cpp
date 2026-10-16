#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>

#include "voxel_map.hpp"

namespace {

using n01_scan::BoundedVoxelMap;
using n01_scan::Voxel;

static int g_failures = 0;
#define CHECK(cond)                                                       \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      ++g_failures;                                                        \
    }                                                                      \
  } while (0)

static bool contains(const std::vector<Voxel> &vs, const Voxel &v) {
  return std::any_of(vs.begin(), vs.end(), [&](const Voxel &x) { return x == v; });
}

// World x of a voxel center, exactly as scan_grid_map publishes it.
static double world_x(const BoundedVoxelMap &map, const Voxel &v) {
  return map.window_center_x() + (static_cast<double>(v.x) + 0.5) * map.resolution();
}
static double world_y(const BoundedVoxelMap &map, const Voxel &v) {
  return map.window_center_y() + (static_cast<double>(v.y) + 0.5) * map.resolution();
}

// --- S4: world-origin accumulation, window follows the real sensor position,
// --- out-of-window return creates NO phantom ring on the window face ---
void test_world_origin_accumulation() {
  BoundedVoxelMap map(1.0, 4.0, 4.0, -1.0, 3.0, 0.85, 0.30, 0.12, 0.98, 0.80, 1,
                      /*max_cells=*/400000, /*max_range=*/30.0, /*inflate_2d=*/true);
  map.set_reject_out_of_window(true);

  // Frame 1: sensor at (0,0), obstacle at world (2.5, 0).
  map.set_window_center(0.0, 0.0);
  map.begin_frame();
  map.add_ray(0.0, 0.0, 0.0, 2.5, 0.0, 0.0);
  map.flush();
  const Voxel obstacle = map.to_voxel(2.5, 0.0, 0.0);
  CHECK(contains(map.occupied(), obstacle));

  // Frame 2: sensor moves to (1,0); obstacle keeps world coords and is
  // returned by the beam again (a beam cannot return past a static obstacle).
  map.set_window_center(1.0, 0.0);
  map.begin_frame();
  map.add_ray(1.0, 0.0, 0.0, 2.5, 0.0, 0.0);  // same world endpoint
  map.flush();
  const Voxel after_move = map.to_voxel(2.5, 0.0, 0.0);
  CHECK(contains(map.occupied(), after_move));
  CHECK(map.window_center_x() == 1.0);
  CHECK(std::fabs(world_x(map, after_move) - 2.5) <= 0.5 * map.resolution() + 1e-9);

  // A return from world (10,0) is far outside the window centered at (1,0):
  // with rejection it is ignored, so occupancy is unchanged and no phantom
  // ring appears on the +x window face.
  const auto occ_before = map.occupied();
  map.begin_frame();
  map.add_ray(1.0, 0.0, 0.0, 10.0, 0.0, 0.0);
  map.flush();
  const auto occ_after = map.occupied();
  const Voxel face = map.to_voxel(5.0, 0.0, 0.0);  // window +x face (center 1 + half 4)
  CHECK(!contains(occ_after, face));
  CHECK(occ_after.size() == occ_before.size());  // nothing new painted
}

// --- S4: cross-frame accumulation, static obstacle fixed in map across sensor motion ---
void test_cross_frame_accumulation() {
  BoundedVoxelMap map(1.0, 4.0, 4.0, -1.0, 3.0, 0.85, 0.30, 0.12, 0.98, 0.80, 1,
                      /*max_cells=*/400000, /*max_range=*/30.0, /*inflate_2d=*/true);
  map.set_reject_out_of_window(true);
  map.set_window_center(0.0, 0.0);
  map.begin_frame();
  map.add_ray(0.0, 0.0, 0.0, 2.5, 1.2, 0.0);
  map.flush();
  const auto first = map.occupied();
  CHECK(first.size() >= 1);

  // Sensor moves to (0.5,0) (window snaps to the voxel grid at 1.0); the static
  // obstacle is returned again by the beam. No reset between frames.
  map.set_window_center(0.5, 0.0);
  map.begin_frame();
  map.add_ray(0.5, 0.0, 0.0, 2.5, 1.2, 0.0);
  map.flush();
  const Voxel obs = map.to_voxel(2.5, 1.2, 0.0);
  CHECK(contains(map.occupied(), obs));

  // Without a reset, cells are NOT wiped between frames (accumulation).
  CHECK(map.size() >= first.size());
}

// --- S4: sliding window no obstacle-coordinate misalignment ---
// Moving the window by an integer number of voxels re-keys surviving cells;
// the published WORLD coordinates of a static obstacle must not change.
void test_sliding_window_no_misalignment() {
  BoundedVoxelMap map(0.5, 5.0, 5.0, -1.0, 3.0, 0.85, 0.30, 0.12, 0.98, 0.80, 1,
                      /*max_cells=*/400000, /*max_range=*/30.0, /*inflate_2d=*/true);
  map.set_reject_out_of_window(true);
  map.set_window_center(0.0, 0.0);
  map.begin_frame();
  map.add_ray(0.0, 0.0, 0.0, 2.4, 0.0, 0.0);
  map.flush();
  const Voxel obs0 = map.to_voxel(2.4, 0.0, 0.0);
  const double w0x = world_x(map, obs0);
  const double w0y = world_y(map, obs0);

  // Move the window by 2 voxels (1.0 m); the beam returns the SAME static
  // obstacle world point (2.4,0,0) again.
  map.set_window_center(1.0, 0.0);
  map.begin_frame();
  map.add_ray(1.0, 0.0, 0.0, 2.4, 0.0, 0.0);
  map.flush();
  const Voxel obs1 = map.to_voxel(2.4, 0.0, 0.0);
  CHECK(contains(map.occupied(), obs1));
  CHECK(std::fabs(world_x(map, obs1) - w0x) < 1e-9);  // world coord preserved
  CHECK(std::fabs(world_y(map, obs1) - w0y) < 1e-9);
  CHECK(std::fabs(w0x - 2.4) <= 0.5 * map.resolution() + 1e-9);  // within one voxel
}

// --- S4: bounded memory still honored under accumulation (cap reached) ---
void test_bounded_memory_accumulation() {
  BoundedVoxelMap map(0.25, 5.0, 5.0, -1.0, 3.0, 0.85, 0.30, 0.12, 0.98, 0.80, 1,
                      /*max_cells=*/1000, /*max_range=*/30.0, /*inflate_2d=*/true);
  map.set_reject_out_of_window(true);
  map.set_window_center(0.0, 0.0);
  // Distinct endpoints spread across the window: ~40x40 = 1600 distinct voxel
  // targets, well above the 1000-cell cap. All inside the window.
  for (int i = 0; i < 2000; ++i) {
    const double x = -4.9 + 0.25 * static_cast<double>(i % 40);
    const double y = -4.9 + 0.25 * static_cast<double>((i / 40) % 40);
    map.begin_frame();
    map.add_ray(0.0, 0.0, 0.0, x, y, 0.0);
    map.flush();
  }
  CHECK(map.size() <= 1000);
  CHECK(map.size() > 0);
}

// --- S4: static obstacle fixed in map across a diagonal sensor motion ---
void test_static_obstacle_fixed_across_motion() {
  BoundedVoxelMap map(0.5, 5.0, 5.0, -1.0, 3.0, 0.85, 0.30, 0.12, 0.98, 0.80, 1,
                      /*max_cells=*/400000, /*max_range=*/30.0, /*inflate_2d=*/true);
  map.set_reject_out_of_window(true);
  // Sensor at (0,0); obstacle at world (2.5, 1.5).
  map.set_window_center(0.0, 0.0);
  map.begin_frame();
  map.add_ray(0.0, 0.0, 0.0, 2.5, 1.5, 0.0);
  map.flush();
  // Sensor moves to (-2, 1); the beam returns the static obstacle's world
  // point (2.5, 1.5) again.
  map.set_window_center(-2.0, 1.0);
  map.begin_frame();
  map.add_ray(-2.0, 1.0, 0.0, 2.5, 1.5, 0.0);
  map.flush();
  CHECK(contains(map.occupied(), map.to_voxel(2.5, 1.5, 0.0)));
  CHECK(map.window_center_x() == -2.0);
  CHECK(map.window_center_y() == 1.0);
}

}  // namespace

int main() {
  test_world_origin_accumulation();
  test_cross_frame_accumulation();
  test_sliding_window_no_misalignment();
  test_bounded_memory_accumulation();
  test_static_obstacle_fixed_across_motion();
  if (g_failures) {
    std::fprintf(stderr, "%d check(s) failed\n", g_failures);
    return 1;
  }
  return 0;
}