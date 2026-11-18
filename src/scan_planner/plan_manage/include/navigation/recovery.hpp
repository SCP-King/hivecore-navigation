#pragma once
// Retry backoff shared by the planner FSM.
#include <algorithm>
#include <cmath>
#include <limits>
namespace navigation {
struct Recovery {
  double best=std::numeric_limits<double>::infinity(),changed=0;
  void reset(double now){best=std::numeric_limits<double>::infinity();changed=now;}
  bool stagnant(double remaining,double now,double timeout=15.){
    if(remaining+.10<best){best=remaining;changed=now;return false;}
    return now-changed>timeout;
  }
  static double retryDelay(int failures){return std::min(2.,.2*std::pow(2.,std::min(4,failures/3)));}
};
}
