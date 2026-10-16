#pragma once

// Deterministic ground classification for the map_localized occupancy map.
//
// Design notes (units / frame / source):
//  - Points are always in the MAP frame (the perception node already
//    transformed /livox/lidar/geo_filtered -> map at the message stamp).
//  - The classifier consumes only measured points; it never reads a manual
//    robot height. The lidar origin passed in is the full translation of
//    map->lidar_link from TF (authoritative 甲方 geometry, ~z 0.9 m), used
//    only to require that the fitted ground plane lies below the lidar and to
//    bound the z-percentile fallback radius.
//  - "plane_ransac": deterministic RANSAC (fixed LCG seed, fixed iteration
//    count, deterministic stride/cap subsampling) finds the dominant
//    near-horizontal plane (|normal_z| >= cos(ground_normal_max_angle_deg),
//    evaluated in map frame; assumes the SLAM map z axis is gravity-aligned --
//    requires on-site confirmation). Plane inliers closer than
//    ground_inlier_dist_m are ground. Points within ground_inlier_dist_m of a
//    flush-with-ground low obstacle are intrinsically ambiguous (documented);
//    the default 0.04 m is ~1 voxel at 0.05 m resolution so a >= ~5 cm object
//    is preserved. A deterministic least-squares refit over the best inlier
//    set (3x3 Jacobi eigen decomposition) reclassifies all points.
//  - Fallback "robust_z_percentile": when no near-vertical dominant plane is
//    accepted (wall-dominant scene, steep slope, dense vegetation), the ground
//    height is the q-th percentile of point z within ground_fallback_radius_m
//    of the lidar's map xy projection; points within ground_inlier_dist_m of
//    it are ground. This keeps a flat floor from being marked occupied when
//    RANSAC fails. If even that yields too few points, the frame is skipped
//    (no wrong update) and a diagnostic is published.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace n01_scan {

struct Point3 {
  double x, y, z;
};

struct GroundParams {
  std::string mode = "none";             // none | plane_ransac
  int subsample_stride = 5;              // points; deterministic ordered sampling
  std::size_t max_points = 20000;        // points; cap for RANSAC candidate set
  int iterations = 24;                   // fixed RANSAC iteration count
  std::uint32_t lcg_seed = 1337;         // fixed seed => candidate sequence deterministic
  double inlier_dist_m = 0.04;           // m (map frame); ~1 voxel at 0.05 m
  double normal_max_cos = 0.8660254;     // cos(30 deg); |normal_z| floor in map frame
  double min_inlier_frac = 0.25;         // 0..1; candidate plane minimum inlier ratio
  std::string fallback = "robust_z_percentile";  // none | robust_z_percentile
  double fallback_percentile_q = 2.0;    // percent 0..100 of point z within radius
  double fallback_radius_m = 3.0;        // m; radius around lidar map xy projection
};

struct GroundResult {
  bool valid = false;                    // false => caller must not update the map
  std::string method = "none";           // plane_ransac | robust_z_percentile | none | ...
  std::vector<bool> is_ground;           // parallel to the input points
  double ground_z = 0.0;                 // estimated ground height in map z (m)
};

class GroundFilter {
public:
  explicit GroundFilter(const GroundParams &p) : p_(p) {}

  // origin = lidar position in map (full TF translation). Returns one flag per
  // input point. Never returns a partially-valid result: on failure valid is
  // false and the caller skips the frame entirely (no wrong update).
  GroundResult classify(const std::vector<Point3> &pts, const Point3 &origin) const {
    GroundResult res;
    res.is_ground.assign(pts.size(), false);
    if (pts.empty()) {
      res.method = "empty";
      return res;
    }
    if (p_.mode == "none") {
      res.method = "none";
      res.valid = true;  // no ground class: every point is a hit ray
      return res;
    }
    if (p_.mode != "plane_ransac") {
      res.method = "bad_mode";
      return res;
    }

    // Deterministic subsample: ordered, fixed stride, fixed cap.
    std::vector<std::size_t> sample;
    sample.reserve(std::min(pts.size(), p_.max_points));
    for (std::size_t i = 0; i < pts.size(); i += static_cast<std::size_t>(p_.subsample_stride)) {
      if (!std::isfinite(pts[i].x) || !std::isfinite(pts[i].y) || !std::isfinite(pts[i].z)) continue;
      sample.push_back(i);
      if (sample.size() >= p_.max_points) break;
    }
    if (sample.size() < 3) return run_fallback(pts, origin, res);

    // Deterministic RANSAC candidate search.
    std::uint32_t s = p_.lcg_seed;
    const auto rnd = [&s]() {
      s = s * 1103515245u + 12345u;
      return (s >> 16) & 0x7fff;
    };

    double best_nx = 0.0, best_ny = 0.0, best_nz = 0.0, best_d = 0.0;
    std::size_t best_inliers = 0;
    const std::size_t n = sample.size();

    for (int it = 0; it < p_.iterations; ++it) {
      std::size_t i0 = rnd() % n, i1 = rnd() % n, i2 = rnd() % n;
      if (i0 == i1 || i1 == i2 || i0 == i2) {
        i1 = (i1 + 1) % n;
        i2 = (i2 + 2) % n;
        if (i0 == i1 || i1 == i2 || i0 == i2) continue;
      }
      const Point3 &p0 = pts[sample[i0]];
      const Point3 &p1 = pts[sample[i1]];
      const Point3 &p2 = pts[sample[i2]];
      const double ux = p1.x - p0.x, uy = p1.y - p0.y, uz = p1.z - p0.z;
      const double vx = p2.x - p0.x, vy = p2.y - p0.y, vz = p2.z - p0.z;
      double nx = uy * vz - uz * vy;
      double ny = uz * vx - ux * vz;
      double nz = ux * vy - uy * vx;
      const double norm = std::sqrt(nx * nx + ny * ny + nz * nz);
      if (!(norm > 1e-9)) continue;                 // degenerate triangle
      nx /= norm; ny /= norm; nz /= norm;
      if (nz < 0.0) { nx = -nx; ny = -ny; nz = -nz; }  // orient +z (map gravity-aligned)
      if (nz < p_.normal_max_cos) continue;         // not near-horizontal
      if (!(nz > 1e-6)) continue;
      const double d = nx * p0.x + ny * p0.y + nz * p0.z;
      // Ground plane must lie below the lidar origin (rejects ceilings).
      const double z_at_origin = (d - nx * origin.x - ny * origin.y) / nz;
      if (!(z_at_origin < origin.z - 1e-3)) continue;

      std::size_t count = 0;
      for (std::size_t i = 0; i < pts.size(); ++i) {
        const double dist = std::fabs(nx * pts[i].x + ny * pts[i].y + nz * pts[i].z - d);
        if (dist <= p_.inlier_dist_m) ++count;
      }
      if (count > best_inliers) {
        best_inliers = count;
        best_nx = nx; best_ny = ny; best_nz = nz; best_d = d;
      }
    }

    if (best_inliers == 0 ||
        static_cast<double>(best_inliers) / static_cast<double>(pts.size()) < p_.min_inlier_frac) {
      return run_fallback(pts, origin, res);
    }

    // Deterministic least-squares refit over the best candidate's inliers.
    double ni = best_nx, nj = best_ny, nk = best_nz, dref = best_d;
    std::size_t cnt = 0;
    double cx = 0.0, cy = 0.0, cz = 0.0;
    for (std::size_t i = 0; i < pts.size(); ++i) {
      if (std::fabs(ni * pts[i].x + nj * pts[i].y + nk * pts[i].z - dref) <= p_.inlier_dist_m) {
        cx += pts[i].x; cy += pts[i].y; cz += pts[i].z; ++cnt;
      }
    }
    if (cnt >= 3) {
      cx /= static_cast<double>(cnt);
      cy /= static_cast<double>(cnt);
      cz /= static_cast<double>(cnt);
      double xx = 0.0, yy = 0.0, zz = 0.0, xy = 0.0, xz = 0.0, yz = 0.0;
      for (std::size_t i = 0; i < pts.size(); ++i) {
        if (std::fabs(ni * pts[i].x + nj * pts[i].y + nk * pts[i].z - dref) > p_.inlier_dist_m) continue;
        const double dx = pts[i].x - cx, dy = pts[i].y - cy, dz = pts[i].z - cz;
        xx += dx * dx; yy += dy * dy; zz += dz * dz;
        xy += dx * dy; xz += dx * dz; yz += dy * dz;
      }
      const double m[3][3] = {{xx, xy, xz}, {xy, yy, yz}, {xz, yz, zz}};
      double ev[3];
      smallest_eigenvector(m, ev);
      ni = ev[0]; nj = ev[1]; nk = ev[2];
      const double norm = std::sqrt(ni * ni + nj * nj + nk * nk);
      if (norm > 1e-9) { ni /= norm; nj /= norm; nk /= norm; }
      if (nk < 0.0) { ni = -ni; nj = -nj; nk = -nk; }
    }
    if (nk < p_.normal_max_cos) return run_fallback(pts, origin, res);
    dref = ni * cx + nj * cy + nk * cz;

    for (std::size_t i = 0; i < pts.size(); ++i) {
      res.is_ground[i] =
        std::fabs(ni * pts[i].x + nj * pts[i].y + nk * pts[i].z - dref) <= p_.inlier_dist_m;
    }
    res.valid = true;
    res.method = "plane_ransac";
    res.ground_z = (dref - ni * origin.x - nj * origin.y) / nk;
    return res;
  }

private:
  // Jacobi rotations on a symmetric 3x3 matrix: returns the eigenvector of the
  // smallest eigenvalue (the plane normal of best fit). Deterministic.
  static void smallest_eigenvector(const double m[3][3], double out[3]) {
    double a[3][3] = {{m[0][0], m[0][1], m[0][2]},
                      {m[1][0], m[1][1], m[1][2]},
                      {m[2][0], m[2][1], m[2][2]}};
    double v[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    for (int sweep = 0; sweep < 24; ++sweep) {
      const double off = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
      if (off < 1e-20) break;
      for (int p = 0; p < 2; ++p) {
        for (int q = p + 1; q < 3; ++q) {
          if (std::fabs(a[p][q]) < 1e-20) continue;
          const double phi = 0.5 * std::atan2(2.0 * a[p][q], a[q][q] - a[p][p]);
          const double c = std::cos(phi), s = std::sin(phi);
          for (int k = 0; k < 3; ++k) {
            const double akp = a[k][p], akq = a[k][q];
            a[k][p] = c * akp - s * akq;
            a[k][q] = s * akp + c * akq;
          }
          for (int k = 0; k < 3; ++k) {
            a[p][k] = a[k][p];
            a[q][k] = a[k][q];
          }
          for (int k = 0; k < 3; ++k) {
            const double vkp = v[k][p], vkq = v[k][q];
            v[k][p] = c * vkp - s * vkq;
            v[k][q] = s * vkp + c * vkq;
          }
        }
      }
    }
    int idx = 0;
    double ev = a[0][0];
    for (int i = 1; i < 3; ++i) if (a[i][i] < ev) { ev = a[i][i]; idx = i; }
    out[0] = v[0][idx]; out[1] = v[1][idx]; out[2] = v[2][idx];
  }

  GroundResult run_fallback(const std::vector<Point3> &pts, const Point3 &origin,
                            GroundResult &res) const {
    if (p_.fallback != "robust_z_percentile") {
      res.method = "plane_ransac_no_fallback";
      return res;
    }
    std::vector<double> zs;
    zs.reserve(pts.size());
    for (const auto &p : pts) {
      if (std::hypot(p.x - origin.x, p.y - origin.y) <= p_.fallback_radius_m &&
          std::isfinite(p.z)) zs.push_back(p.z);
    }
    if (zs.size() < 10) {
      res.method = "robust_z_percentile_insufficient";
      return res;  // valid stays false: caller skips the frame (no wrong update)
    }
    std::sort(zs.begin(), zs.end());
    const std::size_t idx = static_cast<std::size_t>(std::round(
      p_.fallback_percentile_q / 100.0 * static_cast<double>(zs.size() - 1)));
    const double gz = zs[idx];
    for (std::size_t i = 0; i < pts.size(); ++i) {
      res.is_ground[i] = std::fabs(pts[i].z - gz) <= p_.inlier_dist_m;
    }
    res.valid = true;
    res.method = "robust_z_percentile";
    res.ground_z = gz;
    return res;
  }

  GroundParams p_;
};

}  // namespace n01_scan