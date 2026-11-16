#include <navigation/tracking.hpp>
#include <cassert>
#include <limits>
int main(){
  using namespace navigation;
  Curve curve;curve.degree=3;curve.knots={0,0,0,0,1,1,1,1};curve.control={{{0,0,0}},{{.3,0,0}},{{.7,0,0}},{{1,0,0}}};curve.build();
  assert(std::abs(curve.length-1)<1e-8);assert(std::abs(curve.nearest(.4,.1)-.4)<1e-6);
  auto p=curve.at(.4),ahead=curve.at(.65);auto command=feedback(p,ahead,.4,.1,0.,.1,.4);
  assert(command.v>0&&command.w<0); // lateral error corrects toward the reference
  assert(braking_speed(.8,.04,.2)<.13);assert(braking_speed(.8,.04,0.)==0.);
  assert(braking_speed(.03,.04,.04)>.01); // no premature stall outside arrival zone
  bool rejected=false;try{braking_speed(.8,0.,1.);}catch(...){rejected=true;}assert(rejected);
  std::vector<std::array<double,2>> hull={{{-.4,-.15}},{{.4,-.15}},{{.4,.15}},{{-.4,.15}}};
  assert(hits(0,.209,hull,.06));assert(!hits(0,.211,hull,.06));
  assert(!hits(.45,.20,hull,.06)); // no double inflation or square corner expansion
  curve.control[0][0]=std::numeric_limits<double>::quiet_NaN();rejected=false;
  try{curve.build();}catch(...){rejected=true;}assert(rejected);
}
