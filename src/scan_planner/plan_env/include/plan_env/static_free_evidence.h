#pragma once
#include <cstdint>
#include <unordered_map>
#include <cmath>
#include <algorithm>

namespace plan_env_detail {
// Only traversed, hit-free rays are evidence. Missing returns are not free space.
class StaticFreeEvidence {
 public:
  bool free(int cell, double now) const {
    auto it=data_.find(cell);
    if(it==data_.end()) return false;
    const auto& o=it->second;
    return o.count>=3 && o.last-o.first>=0.3 && now>=o.last && now-o.last<=1.5;
  }
  // A stale observation is not fresh free space. Its confirmed contradiction
  // still suppresses the historical prior until a newer hit or cell reset.
  bool suppressesPrior(int cell) const {
    auto it=data_.find(cell);
    return it!=data_.end() && it->second.confirmed;
  }
  bool observe(int cell, bool hit, double now, std::int64_t stamp) {
    if(!std::isfinite(now) || stamp<=0) return false;
    auto it=data_.find(cell);
    if(it!=data_.end() && (stamp<=it->second.stamp || now<it->second.last)) return false;
    const bool before=suppressesPrior(cell);
    auto& o=data_[cell];
    if(hit) {
      o={};o.last=now;o.stamp=stamp;
      return before;
    }
    if(o.count==0 || now-o.last>0.6) { o.count=0; o.first=now; }
    o.count=std::min(255,o.count+1);o.last=now;o.stamp=stamp;
    o.confirmed=o.confirmed || (o.count>=3 && o.last-o.first>=0.3);
    return before!=o.confirmed;
  }
  void reset(int cell) { data_.erase(cell); }
  void clear() { data_.clear(); }
  bool expire(double now) {
    // Bound transient counters; confirmed contradictions remain bounded by
    // the sliding-map cell lifecycle. No prior rebuild is needed on timeout.
    for(auto it=data_.begin();it!=data_.end();) {
      if(now>=it->second.last && now-it->second.last>1.5 && !it->second.confirmed)
        it=data_.erase(it);
      else ++it;
    }
    return false;
  }
 private:
  struct Observation { int count=0; double first=0,last=0;std::int64_t stamp=0;bool confirmed=false; };
  std::unordered_map<int,Observation> data_;
};
}
