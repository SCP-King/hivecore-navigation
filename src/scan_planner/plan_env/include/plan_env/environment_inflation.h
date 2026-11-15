#pragma once
#include <cmath>
#include <stdexcept>
#include <vector>
#include "plan_env/static_inflation.hpp"
namespace plan_env_detail {
inline std::vector<StaticInflationXYOffset> environmentDisk(double margin,double resolution) {
  if(!std::isfinite(margin)||margin<0||!std::isfinite(resolution)||resolution<=0)
    throw std::invalid_argument("invalid environment inflation geometry");
  const int steps=static_cast<int>(std::ceil(margin/resolution));
  std::vector<StaticInflationXYOffset> offsets;
  for(int x=-steps;x<=steps;++x) for(int y=-steps;y<=steps;++y)
    if(std::hypot(x*resolution,y*resolution)<=margin+1e-9) offsets.push_back({x,y});
  return offsets;
}
}
