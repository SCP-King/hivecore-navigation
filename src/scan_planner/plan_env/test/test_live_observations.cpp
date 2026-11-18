#include <plan_env/grid_map.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <chrono>
#include <thread>
#include <stdexcept>
static void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
int main(int argc,char**argv){
 rclcpp::init(argc,argv);
 rclcpp::NodeOptions options;options.parameter_overrides({
  rclcpp::Parameter("grid_map.static_cloud_enabled",true),rclcpp::Parameter("grid_map.static_observation_refresh",true),
  rclcpp::Parameter("grid_map.reference_updates_enabled",true),
  rclcpp::Parameter("grid_map.sensor_type","lidar"),rclcpp::Parameter("grid_map.cloud_is_world",true),
  rclcpp::Parameter("grid_map.resolution",.1),rclcpp::Parameter("grid_map.double_cylinder_radius",.1),
  rclcpp::Parameter("grid_map.environment_margin",.06),rclcpp::Parameter("grid_map.obstacles_inflation_z_up",.9),rclcpp::Parameter("grid_map.obstacles_inflation_z_down",.25),rclcpp::Parameter("grid_map.sliding_map_size_x",6.),
  rclcpp::Parameter("grid_map.sliding_map_size_y",6.),rclcpp::Parameter("grid_map.sliding_map_size_z",3.),
  rclcpp::Parameter("grid_map.ground_height",0.),rclcpp::Parameter("grid_map.observation_ttl_s",1.),
  rclcpp::Parameter("grid_map.require_observed_local_space",true),rclcpp::Parameter("grid_map.max_ray_length",5.),
  rclcpp::Parameter("grid_map.local_update_range_x",3.),rclcpp::Parameter("grid_map.local_update_range_y",3.),
  rclcpp::Parameter("grid_map.local_update_range_z",2.),rclcpp::Parameter("grid_map.frame_id","map"),
  rclcpp::Parameter("grid_map.p_hit",.85),rclcpp::Parameter("grid_map.p_miss",.30),
  rclcpp::Parameter("grid_map.p_min",.12),rclcpp::Parameter("grid_map.p_max",.98),rclcpp::Parameter("grid_map.p_occ",.80)});
 auto n=std::make_shared<rclcpp::Node>("test_live_observations",options);GridMap grid;grid.initMap(n.get());
 auto cloudpub=n->create_publisher<sensor_msgs::msg::PointCloud2>("cloud",rclcpp::SensorDataQoS());
 auto posepub=n->create_publisher<nav_msgs::msg::Odometry>("sensor_pose",rclcpp::SensorDataQoS());
 auto bodypub=n->create_publisher<nav_msgs::msg::Odometry>("body_pose",rclcpp::SensorDataQoS());
 auto forecastpub=n->create_publisher<sensor_msgs::msg::PointCloud2>("/n01/live_navigation/predicted_obstacles",rclcpp::SensorDataQoS());
 auto priorpub=n->create_publisher<sensor_msgs::msg::PointCloud2>("/grid_map/static_occupancy",rclcpp::QoS(1).reliable().transient_local());
 int64_t scene_stamp=0;
 auto scenes=n->create_subscription<sensor_msgs::msg::PointCloud2>("grid_map/occupancy",rclcpp::SensorDataQoS(),[&](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){scene_stamp=rclcpp::Time(m->header.stamp).nanoseconds();});
 auto spin=[&](double sec){auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(sec);while(std::chrono::steady_clock::now()<end){rclcpp::spin_some(n);std::this_thread::sleep_for(std::chrono::milliseconds(5));}};
 try{
  spin(.3);
  require(!grid.observedFree(Eigen::Vector3d(1.,0.,.5)),"unknown cell reported free");
  sensor_msgs::msg::PointCloud2 cloud;cloud.header.frame_id="map";cloud.header.stamp=n->now();cloud.height=1;
  sensor_msgs::PointCloud2Modifier modifier(cloud);modifier.setPointCloud2Fields(4,"x",1,sensor_msgs::msg::PointField::FLOAT32,"y",1,sensor_msgs::msg::PointField::FLOAT32,"z",1,sensor_msgs::msg::PointField::FLOAT32,"ground",1,sensor_msgs::msg::PointField::FLOAT32);modifier.resize(2);
  sensor_msgs::PointCloud2Iterator<float>x(cloud,"x"),y(cloud,"y"),z(cloud,"z"),ground(cloud,"ground");
  *x=2.05;*y=.05;*z=.75;*ground=0;++x;++y;++z;++ground;
  *x=1.05;*y=1.05;*z=.55;*ground=1;
  nav_msgs::msg::Odometry pose;pose.header=cloud.header;pose.pose.pose.position.z=.75;pose.pose.pose.orientation.w=1.;
  for(int i=0;i<4;i++){
    cloud.header.stamp=n->now();pose.header=cloud.header;auto body=pose;body.pose.pose.position.z=.55;bodypub->publish(body);posepub->publish(pose);cloudpub->publish(cloud);spin(.06);
    if(i==0){require(grid.isUnknown(Eigen::Vector3d(2.05,.05,.75)),"ambiguous first hit was classified free");require(!grid.getUnconfirmedPoints().empty(),"ambiguous hit lost its soft risk");}
  }
  spin(.1);
  require(grid.observedFree(Eigen::Vector3d(1.05,.05,.55)),"top-mounted ray coverage did not project to body height");
  require(grid.getOccupancy(Eigen::Vector3d(2.05,.05,.75))==1,"obstacle end lost");
  require(grid.getOccupancy(Eigen::Vector3d(1.05,1.05,.55))==0,"labelled ground became an obstacle");
  sensor_msgs::msg::PointCloud2 forecast;forecast.header=cloud.header;forecast.header.stamp=n->now();forecast.height=1;
  sensor_msgs::PointCloud2Modifier fm(forecast);fm.setPointCloud2FieldsByString(1,"xyz");fm.resize(1);
  sensor_msgs::PointCloud2Iterator<float> fx(forecast,"x"),fy(forecast,"y"),fz(forecast,"z");*fx=1.05;*fy=.05;*fz=.4;
  forecastpub->publish(forecast);spin(.1);
  require(grid.getInflateOccupancy(Eigen::Vector3d(1.05,.05,.55),0.)==0,"forecast became fabricated hard occupancy");
  require(!grid.getUnconfirmedPoints().empty(),"forecast risk lost");
  require(grid.getOccupancy(Eigen::Vector3d(1.05,.05,.55))==0,"forecast polluted measured occupancy");
  const auto last_measurement=rclcpp::Time(cloud.header.stamp).nanoseconds();
  spin(.55);
  require(scene_stamp==last_measurement,"map heartbeat forged a fresh sensor acquisition time");
  require(grid.getInflateOccupancy(Eigen::Vector3d(1.05,.05,.55),0.)==0,"stale forecast not expired independently");
  spin(.5);
  require(grid.isUnknown(Eigen::Vector3d(1.05,.05,.55)),"expired free evidence did not become unknown");
  require(grid.isUnknown(Eigen::Vector3d(2.05,.05,.75)),"expired dynamic occupancy did not become unknown");
  require(grid.getInflateOccupancy(Eigen::Vector3d(2.05,.05,.55),0.,true)==0,"expired obstacle stayed a permanent hard constraint");
  require(!grid.getUnconfirmedPoints().empty(),"expired obstacle lost its unconfirmed history");
  // A lower floor return in the same XY column provides coverage, but must
  // not clear a previously observed chair/seat voxel above it.
  sensor_msgs::PointCloud2Iterator<float> lower(cloud,"z");*lower=.01;
  for(int i=0;i<4;++i){cloud.header.stamp=n->now();pose.header=cloud.header;posepub->publish(pose);cloudpub->publish(cloud);spin(.06);}
  require(grid.hasObservedCoverage(Eigen::Vector3d(2.05,.05,.55)),"floor-return projection missing");
  require(grid.isUnknown(Eigen::Vector3d(2.05,.05,.75)),"lower floor ray certified the upper voxel as free");
  require(!grid.getUnconfirmedPoints().empty(),"lower ray erased upper history risk");
  sensor_msgs::msg::PointCloud2 prior;prior.header.frame_id="map";prior.header.stamp=n->now();prior.height=1;
  sensor_msgs::PointCloud2Modifier pm(prior);pm.setPointCloud2FieldsByString(1,"xyz");pm.resize(1);
  sensor_msgs::PointCloud2Iterator<float> px(prior,"x"),py(prior,"y"),pz(prior,"z");*px=1.05;*py=1.05;*pz=.75;
  priorpub->publish(prior);spin(.2);
  require(grid.getInflateOccupancy(Eigen::Vector3d(1.05,1.05,.55),0.,true)==1,"known background obstacle was softened on entering the window");
  // Old/replayed frames cannot refresh the environment's validity.
  cloud.header.stamp.sec-=3;pose.header=cloud.header;posepub->publish(pose);cloudpub->publish(cloud);spin(1.15);
  require(grid.isUnknown(Eigen::Vector3d(1.05,.05,.55)),"old scan resurrected expired evidence");
  // Continued actual lower-ray coverage downgrades a never-reconfirmed old
  // seat to uncertain risk; it does not fabricate free evidence at seat height.
  for(int i=0;i<17;++i){cloud.header.stamp=n->now();pose.header=cloud.header;posepub->publish(pose);cloudpub->publish(cloud);spin(.10);}
  spin(.15);
  require(grid.getInflateOccupancy(Eigen::Vector3d(1.05,1.05,.55),0.,true)==0,"unconfirmed prior remained a permanent hard obstacle");
  require(grid.getStaticSourceCloud().empty(),"global retained the downgraded local ghost as hard occupancy");
  prior.header.stamp=n->now();priorpub->publish(prior);spin(.2);
  require(grid.getStaticSourceCloud().empty(),"new accumulated map resurrected an unconfirmed prior");
  // A new measured seat hit restores hard occupancy and revokes downgrade.
  sensor_msgs::PointCloud2Iterator<float> hitz(cloud,"z"),hitg(cloud,"ground");++hitz;++hitg;*hitz=.75;*hitg=0;
  for(int i=0;i<4;++i){cloud.header.stamp=n->now();pose.header=cloud.header;posepub->publish(pose);cloudpub->publish(cloud);spin(.06);}
  spin(.15);
  require(grid.getInflateOccupancy(Eigen::Vector3d(1.05,1.05,.55),0.,true)==1,"new real hit did not restore occupied risk");
 }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;rclcpp::shutdown();return 1;}
 rclcpp::shutdown();return 0;
}
