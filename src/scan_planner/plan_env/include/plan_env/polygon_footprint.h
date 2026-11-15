#pragma once
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
namespace plan_env_detail {
inline bool polygonHit(const Eigen::Vector2d &q,const std::vector<Eigen::Vector2d>&p,double margin) {
  bool inside=true;double distance2=1e30;
  for(size_t i=0;i<p.size();++i){
    const auto a=p[i];const auto e=(p[(i+1)%p.size()]-a).eval();const auto v=(q-a).eval();
    if(e.x()*v.y()-e.y()*v.x() < -1e-10)inside=false;
    const double t=std::clamp(v.dot(e)/e.squaredNorm(),0.,1.);
    distance2=std::min(distance2,(v-t*e).squaredNorm());
  }
  return inside || distance2<=margin*margin+1e-12;
}
inline std::vector<Eigen::Vector2i> polygonOffsets(const std::vector<Eigen::Vector2d>&p,double margin,double resolution) {
  Eigen::Vector2d lo=p.front(),hi=p.front();
  for(const auto &v:p){lo=lo.cwiseMin(v);hi=hi.cwiseMax(v);}
  std::vector<Eigen::Vector2i> result;
  for(int y=std::floor((lo.y()-margin)/resolution);y<=std::ceil((hi.y()+margin)/resolution);++y)
    for(int x=std::floor((lo.x()-margin)/resolution);x<=std::ceil((hi.x()+margin)/resolution);++x)
      if(polygonHit({x*resolution,y*resolution},p,margin))result.emplace_back(x,y);
  return result;
}
}
