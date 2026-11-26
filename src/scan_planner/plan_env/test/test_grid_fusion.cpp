#include <plan_env/grid_fusion.h>
#include <cassert>
int main(){
  assert(fuseOccupancy2D(100,0)==100);
  assert(fuseOccupancy2D(100,100)==100);
  assert(fuseOccupancy2D(0,100)==0);
  assert(fuseOccupancy2D(0,0)==0);
  assert(fuseOccupancy2D(-1,100)==100);
  assert(fuseOccupancy2D(-1,0)==0);
  assert(fuseOccupancy2D(-1,-1)==-1);
  return 0;
}
