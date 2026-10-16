#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "voxel_map.hpp"

// CHECK: an always-on assertion (unlike assert(), which compiles out under
// NDEBUG/Release). Failures are counted so every test runs to completion and
// the exit code reflects the failures.
static int g_failures = 0;
#define CHECK(cond)                                                       \
  do {                                                                    \
    if (!(cond)) {                                                        \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      ++g_failures;                                                       \
    }                                                                     \
  } while (0)


namespace {

using n01_scan::BoundedVoxelMap;
using n01_scan::Voxel;

static bool contains(const std::vector<Voxel> &vs, const Voxel &v) {
  return std::any_of(vs.begin(), vs.end(), [&](const Voxel &x) { return x == v; });
}

// --- free cells update via ray: cells strictly between origin and endpoint are not occupied ---
void test_free_ray_update() {
  BoundedVoxelMap map(1.0, 4.0, 4.0, -1.0, 3.0, 0.85, 0.30, 0.12, 0.98, 0.80, 1);
  map.add_ray(0.0, 0.0, 0.0, 3.2, 0.0, 0.0);
  map.flush();
  const Voxel end = map.to_voxel(3.2, 0.0, 0.0);
  CHECK(contains(map.occupied(), end));          // endpoint occupied
  const Voxel mid = map.to_voxel(1.2, 0.0, 0.0);
  CHECK(!contains(map.occupied(), mid));         // free cell NOT occupied
  // A free cell never becomes occupied even after an extra miss pass from the other side.
  map.add_ray(0.0, 0.0, 0.0, -3.2, 0.0, 0.0);
  map.flush();
  CHECK(!contains(map.occupied(), mid));
  CHECK(map.size() < 1000);
}

// --- occlusion: a near surface is NOT cleared by a far return on the same ray ---
void test_occlusion_near_surface_preserved() {
  // far processed SECOND (its free pass crosses the near voxel):
  {
    BoundedVoxelMap map(1.0, 8.0, 4.0, -1.0, 3.0, 0.85, 0.30, 0.12, 0.98, 0.80, 1);
    map.add_ray(0.0, 0.0, 0.0, 2.5, 0.0, 0.0);
    map.add_ray(0.0, 0.0, 0.0, 6.2, 0.0, 0.0);
    map.flush();
    CHECK(contains(map.occupied(), map.to_voxel(2.5, 0.0, 0.0)));
  }
  // far processed FIRST:
  {
    BoundedVoxelMap map2(1.0, 8.0, 4.0, -1.0, 3.0, 0.85, 0.30, 0.12, 0.98, 0.80, 1);
    map2.add_ray(0.0, 0.0, 0.0, 6.2, 0.0, 0.0);
    map2.add_ray(0.0, 0.0, 0.0, 2.5, 0.0, 0.0);
    map2.flush();
    CHECK(contains(map2.occupied(), map2.to_voxel(2.5, 0.0, 0.0)));
  }
}

// --- endpoint is never marked free (ray reaches its own endpoint voxel) ---
void test_endpoint_never_free() {
  BoundedVoxelMap map(1.0, 8.0, 8.0, -1.0, 5.0, 0.85, 0.30, 0.12, 0.98, 0.80, 1);
  map.add_ray(0.0, 0.0, 0.0, 7.2, 7.2, 3.2);
  map.flush();
  CHECK(contains(map.occupied(), map.to_voxel(7.2, 7.2, 3.2)));
}

// --- log-odds clamps: repeated hits saturate, repeated misses stay free ---
void test_log_odds_clamp() {
  BoundedVoxelMap map(1.0, 4.0, 4.0, -1.0, 3.0, 0.85, 0.30, 0.12, 0.98, 0.80, 1);
  for (int i = 0; i < 200; ++i) map.add_ray(0.0, 0.0, 0.0, 2.0, 0.0, 0.0);
  map.flush();
  const Voxel v = map.to_voxel(2.0, 0.0, 0.0);
  CHECK(contains(map.occupied(), v));            // saturates at p_max: still occupied

  BoundedVoxelMap map2(1.0, 4.0, 4.0, -1.0, 3.0, 0.85, 0.30, 0.12, 0.98, 0.80, 1);
  for (int i = 0; i < 200; ++i) map2.add_ray(0.0, 0.0, 0.0, 3.9, 0.0, 0.0);
  map2.flush();
  const Voxel mid = map2.to_voxel(1.5, 0.0, 0.0);
  CHECK(!contains(map2.occupied(), mid));        // free-only cells stay below occ threshold
}

// --- inflation: superset of occupied, includes neighbors ---
void test_inflated_superset() {
  BoundedVoxelMap map(1.0, 4.0, 4.0, -1.0, 3.0, 0.85, 0.30, 0.12, 0.98, 0.80, 1);
  map.add_ray(0.0, 0.0, 0.0, 3.2, 0.0, 0.0);
  map.flush();
  const auto occ = map.occupied();
  const auto infl = map.inflated();
  CHECK(!occ.empty());
  CHECK(infl.size() >= occ.size());
  for (const auto &v : occ) CHECK(contains(infl, v));
}

// --- out-of-bounds: no crash, bounded, partial free-clear to the boundary ---
void test_out_of_bounds_no_crash() {
  BoundedVoxelMap map(1.0, 4.0, 4.0, -1.0, 3.0, 0.85, 0.30, 0.12, 0.98, 0.80, 1);
  // within max_range (30) but outside the box: clamped, traversed to the face.
  map.add_ray(0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
  map.flush();
  const Voxel face = map.to_voxel(4.0, 0.0, 0.0);
  CHECK(contains(map.occupied(), face));               // clamped boundary voxel marked hit
  const Voxel inside_free = map.to_voxel(2.0, 0.0, 0.0);
  CHECK(!contains(map.occupied(), inside_free));       // free cells in-bounds cleared
  CHECK(map.size() < 100);
  // beyond range gate and huge finite: ignored, no crash, no overflow.
  map.reset();
  map.add_ray(0.0, 0.0, 0.0, 100.0, 100.0, 100.0);
  map.add_ray(0.0, 0.0, 0.0, 1e30, -1e30, 1e30);
  map.flush();
  CHECK(map.size() == 0);
}

// --- bounded memory: cell cap honored ---
void test_bounded_memory() {
  BoundedVoxelMap map(0.5, 50.0, 50.0, -1.0, 3.0, 0.85, 0.30, 0.12, 0.98, 0.80, 2,
                      1000, 5.0);
  for (int i = 0; i < 500; ++i) {
    map.add_ray(0.0, 0.0, 0.0, 3.4, 3.4 - static_cast<double>(i) * 0.01, 1.0);
  }
  map.flush();
  CHECK(map.size() <= 1000);
  CHECK(map.size() > 0);
}

// --- bounds: +half_x and -half_x, min_z and max_z all map onto valid voxels ---
void test_bounds() {
  BoundedVoxelMap map(1.0, 4.0, 4.0, -1.0, 3.0, 0.85, 0.30, 0.12, 0.98, 0.80, 1);
  map.add_ray(0.0, 0.0, 0.0,  4.0, 0.0, 0.0);   map.flush();
  CHECK(contains(map.occupied(), map.to_voxel( 4.0, 0.0, 0.0)));
  map.reset();
  map.add_ray(0.0, 0.0, 0.0, -4.0, 0.0, 0.0);   map.flush();
  CHECK(contains(map.occupied(), map.to_voxel(-4.0, 0.0, 0.0)));
  map.reset();
  map.add_ray(0.0, 0.0, 0.0, 0.0, 0.0,  3.0);   map.flush();
  CHECK(contains(map.occupied(), map.to_voxel(0.0, 0.0,  3.0)));
  map.reset();
  map.add_ray(0.0, 0.0, 0.0, 0.0, 0.0, -1.0);   map.flush();
  CHECK(contains(map.occupied(), map.to_voxel(0.0, 0.0, -1.0)));
}

// --- reset clears state (per-frame snapshot semantics) ---
void test_reset() {
  BoundedVoxelMap map(1.0, 4.0, 4.0, -1.0, 3.0, 0.85, 0.30, 0.12, 0.98, 0.80, 1);
  map.add_ray(0.0, 0.0, 0.0, 3.2, 0.0, 0.0);
  map.flush();
  CHECK(map.size() > 0);
  map.reset();
  CHECK(map.size() == 0);
  CHECK(map.occupied().empty());
}

// --- min-range: return inside the sensor voxel is ignored, sensor cell never occupied ---
void test_min_range() {
  BoundedVoxelMap map(1.0, 4.0, 4.0, -1.0, 3.0, 0.85, 0.30, 0.12, 0.98, 0.80, 1);
  map.add_ray(0.0, 0.0, 0.0, 0.1, 0.0, 0.0);     // r < resolution
  map.add_ray(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);     // exactly the sensor
  map.flush();
  CHECK(map.size() == 0);
}

// --- ray through an uninflated free corridor alongside an occupied cell ---
void test_free_not_reversed_along_path() {
  BoundedVoxelMap map(1.0, 8.0, 8.0, -1.0, 5.0, 0.85, 0.30, 0.12, 0.98, 0.80, 1);
  map.add_ray(0.0, 0.0, 0.0, 6.0, 3.0, 0.0);     // endpoint (6,3,?) occupied
  map.add_ray(0.0, 0.0, 0.0, 6.0, 0.0, 0.0);     // endpoint (6,0,?) occupied
  map.flush();
  // A voxel on ray B's path but far from ray A must not be spuriously occupied.
  CHECK(contains(map.occupied(), map.to_voxel(6.0, 0.0, 0.0)));
  const Voxel corridor = map.to_voxel(3.0, 0.0, 0.0);
  CHECK(!contains(map.occupied(), corridor));    // free cell between origin and endpoint
}

}  // namespace

int main() {
  test_free_ray_update();
  test_occlusion_near_surface_preserved();
  test_endpoint_never_free();
  test_log_odds_clamp();
  test_inflated_superset();
  test_out_of_bounds_no_crash();
  test_bounded_memory();
  test_bounds();
  test_reset();
  test_min_range();
  test_free_not_reversed_along_path();
  if (g_failures) {
    std::fprintf(stderr, "%d check(s) failed\n", g_failures);
    return 1;
  }
  return 0;
}