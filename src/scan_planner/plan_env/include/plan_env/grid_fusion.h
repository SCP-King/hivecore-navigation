#pragma once

// Read-only fusion of a short-term local cell and the immutable global cell.
// Values: -1 unknown, 0 free, 100 occupied. Local explicit observations win;
// local unknown must never override the global reference.
inline int fuseOccupancy2D(int local, int global)
{
  if (local == 100) return 100;
  if (local == 0) return 0;
  if (global == 100) return 100;
  if (global == 0) return 0;
  return -1;
}
