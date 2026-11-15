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

// Identical union, merging overlapping vertical spans before XY dilation.
template<class Point>
inline void markMergedStaticInflationRows(
    std::vector<char>& mask, std::vector<Point> points,
    int nx,int ny,int nz,int min_x,int min_y,int min_z,int max_x,int max_y,int max_z,
    const std::vector<StaticInflationXYOffset>& offsets,int down,int up)
{
  points.erase(std::remove_if(points.begin(),points.end(),[=](const Point& p) {
    return p[0]<min_x || p[0]>max_x || p[1]<min_y || p[1]>max_y || p[2]<min_z || p[2]>max_z;
  }),points.end());
  std::sort(points.begin(),points.end(),[](const Point& a,const Point& b) {
    if(a[0]!=b[0]) return a[0]<b[0];
    if(a[1]!=b[1]) return a[1]<b[1];
    return a[2]<b[2];
  });
  std::size_t i=0;
  while(i<points.size()) {
    const int x=points[i][0],y=points[i][1],low=points[i][2];
    int high=low;++i;
    while(i<points.size() && points[i][0]==x && points[i][1]==y &&
          points[i][2]-high<=std::max(0,down)+std::max(0,up)+1) {
      high=points[i][2];++i;
    }
    markStaticInflationRows(mask,nx,ny,nz,min_x,min_y,min_z,max_x,max_y,max_z,
                           x,y,low,offsets,down,high-low+std::max(0,up));
  }
}
}  // namespace plan_env_detail
