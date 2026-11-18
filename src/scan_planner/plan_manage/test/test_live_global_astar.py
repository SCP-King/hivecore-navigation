"""Compile the production pure A* function and test live obstacle rerouting.

No ROS graph, sensor publishing or hardware; logging alone is stubbed out.
"""
import pathlib
import shlex
import subprocess
import tempfile


def test_production_astar_reroutes_current_hits_and_recovers_when_removed():
    root=pathlib.Path(__file__).resolve().parents[1]
    source=(root/'src/planner_manager.cpp').read_text()
    actual=source[source.index('  struct GlobalGridAStarNode'):source.index('  } // namespace')]
    preamble='''
#include <plan_env/polygon_footprint.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <cassert>
#include <iostream>
#include <functional>
#define RCLCPP_ERROR(...) ((void)0)
'''
    main='''
int main(){
 pcl::PointCloud<pcl::PointXYZ> cloud;
 cloud.push_back(pcl::PointXYZ(-5,-3,.9));cloud.push_back(pcl::PointXYZ(5,3,.9));
 Eigen::Vector3d lo(-5,-3,0),hi(5,3,2),start(-2,0,.9),goal(2,0,.9);
 const auto plan=[&](const std::vector<Eigen::Vector3d>&live){
  return globalAStar2D(cloud,live,lo,hi,.1,.25,.35,1.45,.4,start,goal);
 };
 auto clear=plan({});assert(clear.size()>1);
 std::vector<Eigen::Vector3d> wall;
 for(double y=-.8;y<=.81;y+=.05)wall.emplace_back(0,y,.9);
 auto blocked=plan(wall);assert(blocked.size()>1);
 double excursion=0.;for(const auto&p:blocked)excursion=std::max(excursion,std::abs(p.y()));
 assert(excursion>1.0);assert((blocked.back()-goal).norm()<1e-10);
 auto restored=plan({});assert(restored.size()==clear.size());
 for(size_t i=0;i<clear.size();++i)assert((restored[i]-clear[i]).norm()<1e-10);
 assert(plan({goal}).empty());
 // 36cm query radius at 5cm resolution must not become a 40cm disk.
 // Coordinates avoid floor/index ambiguity at cell boundaries.
 Eigen::Vector3d edge_start(-2.025,.025,.9),edge_goal(2.025,.025,.9);
 auto outside=globalAStar2D(cloud,{Eigen::Vector3d(2.405,.025,.9)},lo,hi,
     .05,.36,.35,1.45,.4,edge_start,edge_goal);
 assert(outside.size()>1);
 auto inside=globalAStar2D(cloud,{Eigen::Vector3d(2.365,.025,.9)},lo,hi,
     .05,.36,.35,1.45,.4,edge_start,edge_goal);
 assert(inside.empty());
 // Do not round the 23cm disk offset to 25cm, including after yaw rotation.
 for(const auto& offset : std::vector<Eigen::Vector2d>{{.23,0.},{0.,.23}}){
   Eigen::Vector3d direction(offset.x()/.23,offset.y()/.23,0.);
   auto outside_offset=globalAStar2D(cloud,{edge_goal+.41*direction},lo,hi,
       .05,.15,.35,1.45,.4,edge_start,edge_goal,{offset});
   assert(outside_offset.size()>1);
   auto inside_offset=globalAStar2D(cloud,{edge_goal+.34*direction},lo,hi,
       .05,.15,.35,1.45,.4,edge_start,edge_goal,{offset});
   assert(inside_offset.empty());
 }
 std::vector<Eigen::Vector2d> polygon{{-.4,-.15},{.4,-.15},{.4,.15},{-.4,.15}};
 std::vector<Eigen::Vector3d> corridor;
 for(double x=-3.;x<3.;x+=.025)for(double y:{-.3,.3})corridor.emplace_back(x,y,.9);
 auto rectangle=globalAStar2D(cloud,corridor,lo,hi,.05,.06,.35,1.45,.4,
     start,goal,{Eigen::Vector2d::Zero()},Eigen::Vector3d(0,0,-10),false,
     Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),polygon);
 assert(rectangle.size()>1);
 assert(globalAStar2D(cloud,corridor,lo,hi,.05,.36,.35,1.45,.4,start,goal).empty());
 // An obstacle behind the reference point can hit the asymmetric rear disk.
 std::vector<Eigen::Vector3d> rear_hit{Eigen::Vector3d(-2.5,0,.9)};
 auto circular=globalAStar2D(cloud,rear_hit,lo,hi,.05,.33,.35,1.45,.4,start,goal);
 assert(circular.size()>1);
 std::vector<Eigen::Vector2d> centers{{-.43,0.},{-.1,0.},{.23,0.}};
 auto actual=globalAStar2D(cloud,rear_hit,lo,hi,.05,.33,.35,1.45,.4,start,goal,centers);
 assert(actual.empty());
 auto floor=plan({Eigen::Vector3d(0,0,0)});assert(floor.size()==clear.size());
 // Tilted map floor must not become a solid obstacle after vertical inflation.
 pcl::PointCloud<pcl::PointXYZ> sloped;
 for(double x=-3;x<3;x+=.1)for(double y=-2;y<2;y+=.1)
   sloped.push_back(pcl::PointXYZ(x,y,.02*x-.89));
 // A 4cm scatter return must be handled by the configured 6cm floor band.
 sloped.push_back(pcl::PointXYZ(-2,0,-.89-.04+.04));
 Eigen::Vector3d floor_cutoff(.02,0,-.83);
 auto slope_path=globalAStar2D(sloped,{},lo,hi,.1,.25,-1.1,.3,.4,
     start,goal,{Eigen::Vector2d::Zero()},floor_cutoff);
 assert(slope_path.size()>1);
 // Above-floor hits still block a goal; live and static use the same plane.
 auto slope_blocked=globalAStar2D(sloped,{goal},lo,hi,.1,.25,-1.1,1.1,.4,
     start,goal,{Eigen::Vector2d::Zero()},floor_cutoff);
 assert(slope_blocked.empty());
 auto low_obstacle=globalAStar2D(sloped,{Eigen::Vector3d(2,0,-.89+.04+.08)},lo,hi,
     .1,.25,-1.1,.3,.4,start,goal,{Eigen::Vector2d::Zero()},floor_cutoff);
 assert(low_obstacle.empty());
 // Known historical occupancy remains hard inside and outside the local window;
 // live occupancy is always hard, including at the start and goal.
 pcl::PointCloud<pcl::PointXYZ> historical=cloud;
 historical.push_back(pcl::PointXYZ(start.x(),start.y(),start.z()));
 historical.push_back(pcl::PointXYZ(goal.x(),goal.y(),goal.z()));
 auto reference=globalAStar2D(historical,{},lo,hi,.1,.25,.35,1.45,.4,
     start,goal,{Eigen::Vector2d::Zero()},Eigen::Vector3d(0,0,-10),true,lo,hi);
 assert(reference.empty()); // Entering the window cannot make known start/goal obstacles traversable.
 auto current_block=globalAStar2D(historical,{goal},lo,hi,.1,.25,.35,1.45,.4,
     start,goal,{Eigen::Vector2d::Zero()},Eigen::Vector3d(0,0,-10),true,lo,hi);
 assert(current_block.empty());
 auto unobserved=globalAStar2D(historical,{},lo,hi,.1,.25,.35,1.45,.4,
     start,goal,{Eigen::Vector2d::Zero()},Eigen::Vector3d(0,0,-10),true,
     Eigen::Vector3d(-3,-1,0),Eigen::Vector3d(0,1,2));
 assert(unobserved.empty());
 // Offline bounds stay immutable; realtime reference can retain a far goal.
 Eigen::Vector3d far(12.,0.,.9);
 auto offline_far=globalAStar2D(cloud,{},lo,hi,.1,.25,.35,1.45,.4,start,far);
 assert(offline_far.empty());
 auto realtime=[&](const Eigen::Vector3d& target,const std::vector<Eigen::Vector3d>& hits){
   return globalAStar2D(cloud,hits,lo,hi,.1,.25,.35,1.45,.4,start,target,
     {Eigen::Vector2d::Zero()},Eigen::Vector3d(0,0,-10),true,
     Eigen::Vector3d(-3,-2,0),Eigen::Vector3d(3,2,2),{},true);
 };
 auto far_path=realtime(far,{});assert(far_path.size()>2);
 assert((far_path.front()-start).norm()<1e-10);
 assert((far_path.back()-far).norm()<1e-10);
 assert(realtime(far,{far}).empty()); // extending a reference never overrides a hit
 assert(realtime(Eigen::Vector3d(1e100,0,.9),{}).empty());
 assert(realtime(Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(),0,.9),{}).empty());
 std::cout<<"live obstacle detour="<<excursion<<" m; removal restored route; occupied goal refused\\n";
}
'''
    try:
        flags=shlex.split(subprocess.check_output(
            ['pkg-config','--cflags','--libs','pcl_common'],text=True,stderr=subprocess.PIPE))
    except subprocess.CalledProcessError:
        # Some SDK installations ship headers/libraries without pkg-config.
        includes=sorted(pathlib.Path('/usr/include').glob('pcl-*'))
        if not includes:raise
        flags=['-I'+str(includes[-1]),'-I/usr/include/eigen3']
    with tempfile.TemporaryDirectory(prefix='n01_astar_test_') as tmp:
        path=pathlib.Path(tmp);(path/'test.cpp').write_text(preamble+actual+main)
        subprocess.run(['g++','-O2','-std=c++17',str(path/'test.cpp'),'-o',str(path/'test'),'-I'+str(root.parent/'plan_env/include'),*flags],check=True)
        subprocess.run([str(path/'test')],check=True)
