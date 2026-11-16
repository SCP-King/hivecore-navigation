#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <array>
namespace navigation {
inline double wrap(double a) { return std::atan2(std::sin(a),std::cos(a)); }
inline double limit(double x,double m) { return std::clamp(x,-m,m); }
struct Sample {double x,y,z,yaw,s,u;};
class Curve {
 public:
  int degree=0;
  std::vector<double> knots;
  std::vector<std::array<double,3>> control;
  std::vector<Sample> samples;
  double begin=0,end=0,length=0;
  std::array<double,3> evaluate(double u) const {
    u=std::clamp(u,begin,end);
    int n=control.size(),k=u>=end?n-1:std::clamp(int(std::upper_bound(knots.begin(),knots.end(),u)-knots.begin())-1,degree,n-1);
    std::vector<std::array<double,3>> d;
    for(int j=0;j<=degree;j++)d.push_back(control[k-degree+j]);
    for(int r=1;r<=degree;r++)for(int j=degree;j>=r;j--){
      int i=k-degree+j;double den=knots[i+degree-r+1]-knots[i];double a=den>1e-12?(u-knots[i])/den:0.;
      for(int v=0;v<3;v++)d[j][v]=(1-a)*d[j-1][v]+a*d[j][v];
    }
    return d[degree];
  }
  void build() {
    const int n=control.size();
    if(degree<1||degree>6||n<=degree||n>4096||knots.size()!=size_t(n+degree+1))throw std::invalid_argument("invalid spline shape");
    for(auto v:knots)if(!std::isfinite(v))throw std::invalid_argument("invalid knot");
    if(!std::is_sorted(knots.begin(),knots.end()))throw std::invalid_argument("unordered knots");
    for(auto p:control)for(auto v:p)if(!std::isfinite(v))throw std::invalid_argument("invalid control point");
    begin=knots[degree];end=knots[n];if(end<=begin)throw std::invalid_argument("empty spline");
    samples.clear();length=0.;
    for(int i=0;i<=500;i++){
      double u=begin+(end-begin)*i/500.;auto p=evaluate(u);
      auto a=evaluate(std::max(begin,u-1e-4)),b=evaluate(std::min(end,u+1e-4));
      double yaw=std::atan2(b[1]-a[1],b[0]-a[0]);
      if(!samples.empty())length+=std::hypot(p[0]-samples.back().x,p[1]-samples.back().y);
      samples.push_back({p[0],p[1],p[2],yaw,length,u-begin});
    }
    if(length<1e-6)throw std::invalid_argument("zero spline length");
  }
  Sample at(double s) const {
    s=std::clamp(s,0.,length);
    auto b=std::lower_bound(samples.begin(),samples.end(),s,[](const Sample&p,double v){return p.s<v;});
    if(b==samples.begin())return *b;
    if(b==samples.end())return samples.back();
    auto a=b-1;
    double f=(s-a->s)/std::max(1e-12,b->s-a->s);double u=a->u+f*(b->u-a->u);auto p=evaluate(begin+u);
    return {p[0],p[1],p[2],a->yaw+f*wrap(b->yaw-a->yaw),s,u};
  }
  double nearest(double x,double y) const {
    double best=1e100,s=0.;
    for(size_t i=1;i<samples.size();i++){
      auto a=samples[i-1],b=samples[i];double dx=b.x-a.x,dy=b.y-a.y;
      double f=std::clamp(((x-a.x)*dx+(y-a.y)*dy)/std::max(1e-18,dx*dx+dy*dy),0.,1.);
      double d=std::hypot(x-a.x-f*dx,y-a.y-f*dy);
      if(d<best){best=d;s=a.s+f*(b.s-a.s);}
    }return s;
  }
};
inline double braking_speed(double limit,double accel,double available,double reaction=.6){
  if(!std::isfinite(limit)||!std::isfinite(accel)||!std::isfinite(available)||limit<0||accel<=0||reaction<0)throw std::invalid_argument("invalid braking budget");
  return std::min(limit,std::max(0.,std::sqrt(accel*accel*reaction*reaction+2*accel*std::max(0.,available))-accel*reaction));
}
struct Command {double v=0,w=0;};
inline Command feedback(const Sample& p,const Sample& ahead,double x,double y,double yaw,
                        double speed,double yaw_limit,double gain=1.2) {
  double dx=ahead.x-x,dy=ahead.y-y;
  double error=wrap(p.yaw-yaw),cross=-std::sin(yaw)*dx+std::cos(yaw)*dy;
  double curvature=wrap(ahead.yaw-p.yaw)/std::max(.02,ahead.s-p.s);
  double v=speed*std::max(0.,std::cos(error));
  return {v,limit(v*curvature+gain*error+1.1*cross,yaw_limit)};
}
// Polygon Minkowski query: physical URDF hull plus the environment margin once.
inline bool hits(double x,double y,const std::vector<std::array<double,2>>& polygon,double margin){
  bool inside=false;double distance=1e100;
  for(size_t i=0,j=polygon.size()-1;i<polygon.size();j=i++){
    auto a=polygon[j],b=polygon[i];double dx=b[0]-a[0],dy=b[1]-a[1];
    double f=std::clamp(((x-a[0])*dx+(y-a[1])*dy)/std::max(1e-18,dx*dx+dy*dy),0.,1.);
    distance=std::min(distance,std::hypot(x-a[0]-f*dx,y-a[1]-f*dy));
    if((a[1]>y)!=(b[1]>y) && x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0])inside=!inside;
  }return inside||distance<=margin;
}
}
