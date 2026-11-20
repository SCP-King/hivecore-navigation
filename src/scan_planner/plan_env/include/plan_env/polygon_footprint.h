#pragma once
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
namespace plan_env_detail {
inline bool polygonHit(const Eigen::Vector2d &q,const std::vector<Eigen::Vector2d>&p,double margin) {
  // Winding-independent point-in-polygon: URDF hulls may be authored CW or CCW.
  bool inside=false;double distance2=1e30;
  for(size_t i=0,j=p.size()-1;i<p.size();j=i++){
    const auto &a=p[i],&b=p[j];
    if(((a.y()>q.y())!=(b.y()>q.y())) &&
       (q.x() < (b.x()-a.x())*(q.y()-a.y())/(b.y()-a.y()+1e-12)+a.x()))
      inside=!inside;
    const auto e=(p[(i+1)%p.size()]-p[i]).eval();const auto v=(q-p[i]).eval();
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
