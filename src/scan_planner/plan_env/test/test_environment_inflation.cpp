#include "plan_env/environment_inflation.h"
#include <stdexcept>
#include <iostream>
void require(bool value) { if(!value) throw std::runtime_error("environment inflation check failed"); }
int main() {
 const auto offsets=plan_env_detail::environmentDisk(.06,.05);
 require(offsets.size()==5); // 5cm grid: centre plus axial neighbours.
 for(const auto& p:offsets) require(std::hypot(p.x*.05,p.y*.05)<=.060000001);
 const auto fine=plan_env_detail::environmentDisk(.06,.01);
 bool edge=false;
 for(const auto& p:fine) {
   require(std::hypot(p.x*.01,p.y*.01)<=.060000001);
   if(p.x==6 && p.y==0) edge=true;
 }
 require(edge);
 require(plan_env_detail::environmentDisk(0,.05).size()==1);
 bool rejected=false;
 try{plan_env_detail::environmentDisk(-.1,.05);}catch(const std::invalid_argument&){rejected=true;}
 require(rejected);
 std::cout<<"environment 6cm disk verified independently of robot size\n";
}
