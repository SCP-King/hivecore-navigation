#pragma once
#include <array>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>
namespace plan_env_detail {
using WorldCell=std::array<int,3>;
struct WorldCellHash {
  size_t operator()(const WorldCell& a)const {
    size_t h=0;for(int v:a)h^=std::hash<int>{}(v)+0x9e3779b9+(h<<6)+(h>>2);return h;
  }
};
// Historical changes have coordinate keys, never rotating buffer addresses.
// This is a contradiction to a map prior, NOT fresh free-space certification.
class WorldChanges {
 public:
  bool hasHistory(const WorldCell& key)const{return changes_.find(key)!=changes_.end();}
  bool suppressed(const WorldCell& key)const {
    auto it=changes_.find(key);return it!=changes_.end()&&it->second.state==State::Free;
  }
  bool uncertain(const WorldCell& key)const {
    auto it=changes_.find(key);return it!=changes_.end()&&it->second.state==State::Unconfirmed;
  }
  std::vector<WorldCell> uncertainCells()const {
    std::vector<WorldCell> cells;for(const auto& e:changes_)if(e.second.state==State::Unconfirmed)cells.push_back(e.first);return cells;
  }
  bool ageHit(const WorldCell& key,int64_t stamp=0){
    auto it=changes_.find(key);
    if(stamp==0){if(it==changes_.end())return false;stamp=it->second.stamp;}
    if(it!=changes_.end()&&(stamp<it->second.stamp || it->second.state!=State::Hit))return false;
    if(it==changes_.end()&&changes_.size()>=2000000)return false;
    changes_[key]={stamp,State::Unconfirmed};return true;
  }
  bool observe(const WorldCell& key,bool hit,int64_t stamp){
    if(stamp<=0)return false;
    auto it=changes_.find(key);if(it!=changes_.end() && stamp<=it->second.stamp)return false;
    const bool before=suppressed(key);
    if(it==changes_.end()&&changes_.size()>=2000000)return false;
    changes_[key]={stamp,hit?State::Hit:State::Free};return before!=!hit;
  }
  size_t size()const{return changes_.size();}
  void clear(){changes_.clear();}
 private:
  enum class State {Hit,Free,Unconfirmed};
  struct Change{int64_t stamp;State state;};
  std::unordered_map<WorldCell,Change,WorldCellHash> changes_;
};
}
