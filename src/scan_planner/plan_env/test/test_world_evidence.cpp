#include <plan_env/world_evidence.h>
#include <cassert>
int main(){
 plan_env_detail::WorldChanges e;
 plan_env_detail::WorldCell old={-10,4,2},reused={10,4,2},above={-10,4,3};
 assert(e.observe(old,false,10));assert(e.suppressed(old));
 assert(!e.suppressed(reused));assert(!e.suppressed(above));
 // A ring-buffer address can be recycled without inheriting the world key.
 assert(e.suppressed(old));
 assert(!e.observe(old,true,9));assert(e.suppressed(old));
 assert(e.observe(old,true,11));assert(!e.suppressed(old));
 assert(!e.observe(old,false,10));assert(!e.suppressed(old));
 assert(e.observe(old,false,12));assert(e.suppressed(old));
 assert(e.ageHit(old,12)==false); // Free evidence cannot age into a phantom hit.
 assert(e.observe(old,true,13));assert(!e.uncertain(old));
 assert(e.ageHit(old,13));assert(e.uncertain(old));assert(!e.suppressed(old));
 assert(e.uncertainCells().size()==1);
 assert(!e.observe(old,true,12));assert(e.uncertain(old));
 e.observe(old,true,14);assert(!e.uncertain(old));
 assert(e.ageHit(old,14));assert(e.observe(old,false,15));assert(e.suppressed(old));
 e.clear();assert(!e.suppressed(old)&&e.size()==0);
}
