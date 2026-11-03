#include "plan_env/static_inflation.hpp"

#include <array>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace
{
int wrap(int value, int size)
{
  const int wrapped = value % size;
  return wrapped < 0 ? wrapped + size : wrapped;
}

std::size_t address(int x, int y, int z, int nx, int ny, int nz)
{
  return (static_cast<std::size_t>(wrap(x, nx)) * ny + wrap(y, ny)) * nz +
         wrap(z, nz);
}

void checkCase(
    int nx, int ny, int nz,
    int min_x, int min_y, int min_z,
    const std::vector<plan_env_detail::StaticInflationXYOffset>& offsets,
    int z_down, int z_up,
    const std::vector<std::array<int, 3>>& occupied)
{
  const int max_x = min_x + nx - 1;
  const int max_y = min_y + ny - 1;
  const int max_z = min_z + nz - 1;
  const std::size_t size = static_cast<std::size_t>(nx) * ny * nz;
  std::vector<char> expected(size, 0);
  std::vector<char> actual(size, 0);

  for (const auto& point : occupied) {
    for (const auto& offset : offsets) {
      for (int dz = -z_down; dz <= z_up; ++dz) {
        const int x = point[0] + offset.x;
        const int y = point[1] + offset.y;
        const int z = point[2] + dz;
        if (x < min_x || x > max_x ||
            y < min_y || y > max_y ||
            z < min_z || z > max_z) {
          continue;
        }
        expected[address(x, y, z, nx, ny, nz)] = 1;
      }
    }

    plan_env_detail::markStaticInflationRows(
        actual, nx, ny, nz,
        min_x, min_y, min_z, max_x, max_y, max_z,
        point[0], point[1], point[2],
        offsets, z_down, z_up);
  }

  if (expected != actual) {
    std::cerr << "static inflation row-span output differs from reference\n";
    std::exit(EXIT_FAILURE);
  }
}
}  // namespace

int main()
{
  using plan_env_detail::StaticInflationXYOffset;
  const std::vector<StaticInflationXYOffset> disk{
      {0, 0}, {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, -1}};

  // Negative global indices and clipping on all six map boundaries.
  checkCase(
      7, 6, 5, -3, 2, 3, disk, 2, 1,
      {{{-3, 2, 3}}, {{0, 4, 5}}, {{3, 7, 7}}, {{-1, 5, 6}}});

  // The active window crosses modulo wrap boundaries in X, Y and Z.
  checkCase(
      8, 7, 6, 3, -5, -4, disk, 1, 2,
      {{{3, -5, -4}}, {{7, -1, 1}}, {{5, -3, 0}}, {{10, 0, 0}}});

  // Zero-radius/empty XY kernel must not mark any voxel.
  checkCase(
      4, 4, 4, -2, -2, -2, {}, 0, 0,
      {{{-2, -2, -2}}, {{0, 0, 0}}});

  std::cout << "static inflation row-span equivalence: PASS\n";
  return 0;
}
