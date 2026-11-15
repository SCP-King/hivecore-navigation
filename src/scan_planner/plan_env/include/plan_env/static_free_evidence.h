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
  bool observe(int cell, bool hit, double now, std::int64_t stamp) {
    const bool before=free(cell,now);
    if(hit) { data_.erase(cell); return before; }
    auto& o=data_[cell];
    if(stamp<=o.stamp) return false;
    if(o.count==0 || now<o.last || now-o.last>0.6) { o.count=0; o.first=now; }
    o.count=std::min(255,o.count+1);o.last=now;o.stamp=stamp;
    return before!=free(cell,now);
  }
  void reset(int cell) { data_.erase(cell); }
  void clear() { data_.clear(); }
  bool expire(double now) {
    bool changed=false;
    for(auto it=data_.begin();it!=data_.end();) {
      if(now<it->second.last || now-it->second.last>1.5) {
        changed=changed || (it->second.count>=3 && it->second.last-it->second.first>=0.3);
        it=data_.erase(it);
      } else ++it;
    }
    return changed;
  }
 private:
  struct Observation { int count=0; double first=0,last=0;std::int64_t stamp=0; };
  std::unordered_map<int,Observation> data_;
};
}
