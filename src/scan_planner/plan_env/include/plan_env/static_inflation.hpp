#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

namespace plan_env_detail {

struct StaticInflationXYOffset {
  int x;
  int y;
};

inline int wrapStaticIndex(int value, int size)
{
  const int wrapped = value % size;
  return wrapped < 0 ? wrapped + size : wrapped;
}

// Mark the same clipped cylindrical inflation volume as the original 3-D
// offset loop, but fill contiguous Z rows instead of testing every voxel.
inline void markStaticInflationRows(
    std::vector<char>& mask,
    int nx, int ny, int nz,
    int min_x, int min_y, int min_z,
    int max_x, int max_y, int max_z,
    int gx, int gy, int gz,
    const std::vector<StaticInflationXYOffset>& xy_offsets,
    int z_down, int z_up)
{
  if (nx <= 0 || ny <= 0 || nz <= 0 ||
      max_x - min_x + 1 != nx ||
      max_y - min_y + 1 != ny ||
      max_z - min_z + 1 != nz ||
      mask.size() != static_cast<std::size_t>(nx) * ny * nz ||
      gx < min_x || gx > max_x ||
      gy < min_y || gy > max_y ||
      gz < min_z || gz > max_z ||
      xy_offsets.empty()) {
    return;
  }

  const int z_low = std::max(min_z, gz - std::max(0, z_down));
  const int z_high = std::min(max_z, gz + std::max(0, z_up));
  if (z_low > z_high) return;

  const int local_z = wrapStaticIndex(z_low, nz);
  const int z_count = z_high - z_low + 1;
  const int first_count = std::min(z_count, nz - local_z);
  const int remaining = z_count - first_count;

  for (const auto& offset : xy_offsets) {
    const int target_x = gx + offset.x;
    const int target_y = gy + offset.y;
    if (target_x < min_x || target_x > max_x ||
        target_y < min_y || target_y > max_y) {
      continue;
    }

    const int local_x = wrapStaticIndex(target_x, nx);
    const int local_y = wrapStaticIndex(target_y, ny);
    const std::size_t row_base =
        (static_cast<std::size_t>(local_x) * ny + local_y) * nz;

    std::fill_n(mask.begin() + row_base + local_z, first_count, static_cast<char>(1));
    if (remaining > 0) {
      std::fill_n(mask.begin() + row_base, remaining, static_cast<char>(1));
    }
  }
}

}  // namespace plan_env_detail
