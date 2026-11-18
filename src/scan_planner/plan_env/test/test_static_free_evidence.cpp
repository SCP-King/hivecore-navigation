#include "plan_env/static_free_evidence.h"
#include <stdexcept>

void check(bool ok) { if(!ok) throw std::runtime_error("static free evidence contract failed"); }
int main() {
  plan_env_detail::StaticFreeEvidence e;
  check(!e.free(4,1.)); // unobserved is not free
  e.observe(4,false,1.,100);
  e.observe(4,false,1.2,100); // duplicate sensor stamp cannot build confidence
  e.observe(4,false,1.4,100);
  check(!e.free(4,1.4));
  e.observe(4,false,1.4,200);e.observe(4,false,1.6,300);
  check(e.free(4,1.6));
  check(!e.free(4,3.2)); // stale observations are unknown, not fresh free space
  check(e.suppressesPrior(4));
  check(!e.expire(3.2));
  check(e.suppressesPrior(4)); // no historical resurrection on timeout
  check(!e.free(4,3.2));
  e.observe(4,false,4.,400);e.observe(4,false,4.2,500);e.observe(4,false,4.4,600);
  check(e.free(4,4.4));
  check(e.observe(4,true,4.5,700)); // a new hit immediately revokes clearance
  check(!e.free(4,4.5));
  check(!e.suppressesPrior(4));
  e.observe(4,false,4.6,600); // delayed miss before the hit cannot undo it
  check(!e.suppressesPrior(4));
  e.observe(4,false,5.,800);e.observe(4,false,6.,900);e.observe(4,false,6.2,1000);
  check(!e.free(4,6.2)); // intermittent rays are not continuous free evidence
  e.observe(4,false,6.4,1100);check(e.free(4,6.4));
  check(!e.observe(4,true,6.5,1000)); // old hit cannot erase newer clearance
  check(e.suppressesPrior(4));
  check(!e.suppressesPrior(5)); // nearby higher voxel is independent
  check(!e.observe(4,true,6.3,1200)); // backwards receipt clock rejected
  check(e.suppressesPrior(4));
  check(e.observe(4,true,6.6,1200));
  check(!e.suppressesPrior(4));
  e.observe(4,false,7.,1300);e.observe(4,false,7.2,1400);e.observe(4,false,7.4,1500);
  check(e.suppressesPrior(4));
  e.reset(4);check(!e.free(4,7.4) && !e.suppressesPrior(4)); // ring-buffer cell reuse must not inherit clearance
  return 0;
}
