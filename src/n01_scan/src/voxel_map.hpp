#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace n01_scan {

struct Voxel {
  int x;
  int y;
  int z;
  bool operator==(const Voxel &other) const { return x == other.x && y == other.y && z == other.z; }
};

struct VoxelHash {
  std::size_t operator()(const Voxel &v) const {
    std::size_t h = static_cast<std::size_t>(v.x) * 73856093u;
    h ^= static_cast<std::size_t>(v.y) * 19349663u;
    h ^= static_cast<std::size_t>(v.z) * 83492791u;
    return h;
  }
};

// Bounded 3D log-odds occupancy grid, SCAN semantics ported to ROS2.
//
// Two operating modes share one class:
//  - sensor_only (S3): window center fixed at (0,0), reset() per frame => a
//    per-scan snapshot in a lidar-centered frame. With the default center this
//    reproduces the original fixed-origin voxel indices exactly, so the S3
//    unit tests pass unchanged.
//  - map_localized (S4): the window center follows the real lidar origin in
//    map coordinates via set_window_center(); cells accumulate across frames.
//    Moving the window re-keys surviving cells by the integer voxel shift and
//    prunes cells that fall outside the window, so a static obstacle keeps its
//    absolute world coordinate and never shifts (requirement: "no obstacle
//    coordinate misalignment on window move").
//
// Per frame: begin_frame() starts a fresh occlusion set, add_ray() is called
// once per return, flush() applies the update. A cell that is a hit endpoint
// in the CURRENT frame is never marked free by another ray (per-frame
// occlusion voting, SCAN grid_map.cpp), so a far return on the same beam
// cannot erase a nearer surface regardless of processing order.
//
// Ground/free rays (add_ray(..., free=true)) mark the traversal and their
// endpoint free but never enter the occlusion set: within one frame a ground
// beam stops at a same-frame obstacle endpoint (hit wins), while across frames
// a previous frame's hit does not block today's free ray (hit_voxels_ lives
// for exactly one frame and is cleared at the end of flush()).
//
// Rays are Amanatides-Woo 3D DDA between voxel centers; cells are capped
// (max_cells). In sensor_only, out-of-window endpoints are clamped to the box
// face (partial free-clear to the face, cells beyond max_range gated). In
// map_localized, set_reject_out_of_window(true) drops endpoints outside the
// window instead of clamping them to the face (a clamped return would paint a
// phantom occupied ring around the moving window).
class BoundedVoxelMap {
public:
  BoundedVoxelMap(double resolution, double half_x, double half_y, double min_z, double max_z,
                  double p_hit, double p_miss, double p_min, double p_max, double p_occ,
                  int inflation_radius, std::size_t max_cells = 400000, double max_range = 30.0,
                  bool inflate_2d = false)
  : resolution_(resolution), half_x_(half_x), half_y_(half_y), min_z_(min_z), max_z_(max_z),
    min_log_(logit(p_min)), max_log_(logit(p_max)), occ_log_(logit(p_occ)),
    hit_log_(logit(p_hit)), miss_log_(logit(p_miss)),
    inflation_radius_(inflation_radius), max_cells_(max_cells), max_range_(max_range),
    inflate_2d_(inflate_2d) {
    nx_ = static_cast<int>(std::round(half_x_ / resolution_));
    ny_ = static_cast<int>(std::round(half_y_ / resolution_));
    nz_ = static_cast<int>(std::round((max_z_ - min_z_) / resolution_));
    pending_.reserve(4096);
  }

  // Clears cells AND the window center: returns to the origin-centered (S3)
  // state. Accumulation mode uses begin_frame() below per frame instead.
  void reset() {
    cells_.clear();
    hit_voxels_.clear();
    pending_.clear();
    cx_ = 0.0;
    cy_ = 0.0;
  }

  // Start a new frame in accumulation mode: fresh per-frame occlusion set.
  void begin_frame() {
    hit_voxels_.clear();
    pending_.clear();
  }

  // Move the window center to the lidar origin in world coordinates. The
  // center is snapped to the voxel grid (sub-voxel drift cannot accumulate);
  // an unchanged center is a no-op. Cells outside the new window are pruned;
  // surviving cells are re-keyed by the integer voxel shift so their absolute
  // world coordinates are preserved (no obstacle coordinate misalignment).
  void set_window_center(double wx, double wy) {
    const double new_cx = std::round(wx / resolution_) * resolution_;
    const double new_cy = std::round(wy / resolution_) * resolution_;
    const int dxi = static_cast<int>(std::llround((new_cx - cx_) / resolution_));
    const int dyi = static_cast<int>(std::llround((new_cy - cy_) / resolution_));
    if (dxi == 0 && dyi == 0) {
      cx_ = new_cx;
      cy_ = new_cy;
      return;
    }
    // Re-key + prune: the integer shift is a bijection on keys, so no
    // collisions can occur; only bounds pruning removes cells.
    std::unordered_map<Voxel, double, VoxelHash> shifted;
    shifted.reserve(cells_.size());
    for (const auto &kv : cells_) {
      const Voxel n{kv.first.x - dxi, kv.first.y - dyi, kv.first.z};
      if (inside(n)) shifted.emplace(n, kv.second);
    }
    cells_.swap(shifted);
    cx_ = new_cx;
    cy_ = new_cy;
  }

  // map_localized rejects out-of-window returns instead of clamping them to
  // the window face (a clamp would paint a phantom occupied ring). sensor_only
  // keeps clamping (S3 behavior, asserted by test_out_of_bounds_no_crash).
  void set_reject_out_of_window(bool reject) { reject_out_of_window_ = reject; }

  std::size_t size() const { return cells_.size(); }
  double window_center_x() const { return cx_; }
  double window_center_y() const { return cy_; }

  // Bounds-inclusive voxel index relative to the current window center;
  // out-of-range inputs are clamped so the map is always bounded and int-cast
  // overflow is impossible for huge finite values. With cx_=cy_=0 this is
  // byte-for-byte the S3 index mapping.
  Voxel to_voxel(double x, double y, double z) const {
    constexpr double kMaxIndex = 1e6;
    const int ix = static_cast<int>(
      std::floor(std::clamp((x - cx_) / resolution_, -kMaxIndex, kMaxIndex)));
    const int iy = static_cast<int>(
      std::floor(std::clamp((y - cy_) / resolution_, -kMaxIndex, kMaxIndex)));
    const int iz = static_cast<int>(
      std::floor(std::clamp((z - min_z_) / resolution_, -kMaxIndex, kMaxIndex)));
    return {std::clamp(ix, -nx_, nx_), std::clamp(iy, -ny_, ny_), std::clamp(iz, 0, nz_ - 1)};
  }

  bool inside(const Voxel &v) const {
    return v.x >= -nx_ && v.x <= nx_ && v.y >= -ny_ && v.y <= ny_ && v.z >= 0 && v.z < nz_;
  }

  // Whether the raw (unclamped) voxel index of a world point lies inside the
  // window. Distinct from inside(v), which takes already-clamped indices.
  bool endpoint_inside_window(double px, double py, double pz) const {
    const int ix = static_cast<int>(std::floor((px - cx_) / resolution_));
    const int iy = static_cast<int>(std::floor((py - cy_) / resolution_));
    const int iz = static_cast<int>(std::floor((pz - min_z_) / resolution_));
    return ix >= -nx_ && ix <= nx_ && iy >= -ny_ && iy <= ny_ && iz >= 0 && iz < nz_;
  }

  // Register one return (sensor origin -> point). free=true is a ground/free
  // ray: its endpoint is not added to the occlusion set and gets the miss
  // delta. Points closer than one voxel to the origin or beyond max_range are
  // ignored; in map_localized (reject_out_of_window) returns outside the
  // window are ignored instead of being clamped to the face.
  void add_ray(double ox, double oy, double oz, double px, double py, double pz,
               bool free = false) {
    const double rx = px - ox, ry = py - oy, rz = pz - oz;
    const double r2 = rx * rx + ry * ry + rz * rz;
    if (r2 < resolution_ * resolution_) return;              // inside the sensor voxel
    if (max_range_ > 0.0 && r2 > max_range_ * max_range_) return;  // range gate
    // Rejection must be evaluated on the UNCLAMPED endpoint index: to_voxel()
    // clamps into the window, which would make inside() always true and the
    // rejection dead code. A clamped return would paint a phantom occupied
    // ring on the moving window face.
    if (reject_out_of_window_ && !endpoint_inside_window(px, py, pz)) return;
    const Voxel origin = to_voxel(ox, oy, oz);
    const Voxel endpoint = to_voxel(px, py, pz);
    if (endpoint == origin) return;
    pending_.emplace_back(Ray{origin, endpoint, free});
    if (!free) hit_voxels_.insert(endpoint);  // flag hits BEFORE traversal: order-independent occlusion
  }

  // Apply the frame: free traversal for all rays (skipping any flagged hit
  // voxel), then the endpoint update per ray. Hit rays get the hit delta,
  // free rays get the miss delta (unless another ray hit that exact voxel this
  // frame -- a hit wins). Call once per frame. The per-frame occlusion set is
  // cleared here so a future frame starts fresh.
  void flush() {
    for (const auto &ray : pending_) {
      raycast_free(ray.origin, ray.endpoint);
      if (ray.free) {
        if (!hit_voxels_.count(ray.endpoint)) update(ray.endpoint, miss_log_);
      } else {
        update(ray.endpoint, hit_log_);
      }
    }
    pending_.clear();
    hit_voxels_.clear();
  }

  std::vector<Voxel> occupied() const {
    std::vector<Voxel> out;
    out.reserve(cells_.size());
    for (const auto &entry : cells_) if (entry.second >= occ_log_) out.push_back(entry.first);
    return out;
  }

  // Inflated set. inflate_2d restricts dilation to the same z plane (dz=0),
  // which keeps the output bounded in accumulation mode; sensor_only keeps the
  // full 3D dilation.
  std::vector<Voxel> inflated() const {
    std::vector<Voxel> out;
    const auto occ = occupied();
    std::unordered_map<Voxel, bool, VoxelHash> seen;
    seen.reserve(occ.size() * 8);
    const int z0 = inflate_2d_ ? 0 : -inflation_radius_;
    const int z1 = inflate_2d_ ? 0 : inflation_radius_;
    for (const auto &v : occ) {
      for (int dx = -inflation_radius_; dx <= inflation_radius_; ++dx) {
        for (int dy = -inflation_radius_; dy <= inflation_radius_; ++dy) {
          for (int dz = z0; dz <= z1; ++dz) {
            if (dx * dx + dy * dy + dz * dz > inflation_radius_ * inflation_radius_) continue;
            Voxel q{v.x + dx, v.y + dy, v.z + dz};
            if (inside(q) && seen.emplace(q, true).second) out.push_back(q);
          }
        }
      }
    }
    return out;
  }

  double resolution() const { return resolution_; }
  double min_z() const { return min_z_; }
  int nx() const { return nx_; }
  int ny() const { return ny_; }
  int nz() const { return nz_; }

private:
  struct Ray {
    Voxel origin;
    Voxel endpoint;
    bool free;
  };

  static double logit(double p) {
    const double q = std::clamp(p, 1e-6, 1.0 - 1e-6);
    return std::log(q / (1.0 - q));
  }

  void update(const Voxel &v, double delta) {
    if (!inside(v)) return;
    auto it = cells_.find(v);
    if (it == cells_.end()) {
      if (cells_.size() >= max_cells_) return;  // bounded memory: no new cells beyond cap
      it = cells_.emplace(v, 0.0).first;
    }
    it->second = std::clamp(it->second + delta, min_log_, max_log_);
  }

  // Amanatides-Woo 3D DDA between voxel centers: cells strictly between a and b
  // are updated with the miss delta; traversal stops at the endpoint voxel
  // (never marked free) and at the first voxel already flagged as a hit
  // endpoint this frame (a nearer surface occludes later returns).
  void raycast_free(const Voxel &a, const Voxel &b) {
    if (a == b) return;
    const int dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
    const int stepX = dx ? (dx > 0 ? 1 : -1) : 0;
    const int stepY = dy ? (dy > 0 ? 1 : -1) : 0;
    const int stepZ = dz ? (dz > 0 ? 1 : -1) : 0;
    const double inf = std::numeric_limits<double>::infinity();
    // Rays start at a voxel center (offset 0.5), so the first boundary along an
    // axis is 0.5 voxels away and every subsequent one 1.0 voxels away.
    double tMaxX = dx ? 0.5 / std::abs(static_cast<double>(dx)) : inf;
    double tMaxY = dy ? 0.5 / std::abs(static_cast<double>(dy)) : inf;
    double tMaxZ = dz ? 0.5 / std::abs(static_cast<double>(dz)) : inf;
    const double tDeltaX = dx ? 1.0 / std::abs(static_cast<double>(dx)) : inf;
    const double tDeltaY = dy ? 1.0 / std::abs(static_cast<double>(dy)) : inf;
    const double tDeltaZ = dz ? 1.0 / std::abs(static_cast<double>(dz)) : inf;

    int x = a.x, y = a.y, z = a.z;
    for (;;) {
      if (tMaxX < tMaxY && tMaxX < tMaxZ)      { x += stepX; tMaxX += tDeltaX; }
      else if (tMaxY < tMaxZ)                  { y += stepY; tMaxY += tDeltaY; }
      else                                     { z += stepZ; tMaxZ += tDeltaZ; }
      if (x == b.x && y == b.y && z == b.z) break;  // endpoint reached: never marked free
      const Voxel v{x, y, z};
      if (hit_voxels_.count(v)) break;               // nearer hit this frame occludes
      update(v, miss_log_);
    }
  }

  double resolution_, half_x_, half_y_, min_z_, max_z_;
  double min_log_, max_log_, occ_log_, hit_log_, miss_log_;
  int inflation_radius_, nx_, ny_, nz_;
  std::size_t max_cells_;
  double max_range_;
  bool inflate_2d_{};
  bool reject_out_of_window_{};
  double cx_{}, cy_{};  // window center in world meters (snapped to grid)
  std::unordered_map<Voxel, double, VoxelHash> cells_;
  std::unordered_set<Voxel, VoxelHash> hit_voxels_;  // per-frame occlusion set
  std::vector<Ray> pending_;
};

}  // namespace n01_scan
