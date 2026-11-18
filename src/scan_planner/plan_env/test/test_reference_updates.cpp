#include <plan_env/grid_map.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>
#include <limits>
#include <stdexcept>
#include <thread>

void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}

void exercise(bool updates) {
  rclcpp::NodeOptions options;
  const std::string topic = updates ? "/test/live_reference" : "/test/offline_reference";
  options.parameter_overrides({
    rclcpp::Parameter("grid_map.resolution", .1),
    rclcpp::Parameter("grid_map.double_cylinder_radius", .30),
    rclcpp::Parameter("grid_map.environment_margin", .06),
    rclcpp::Parameter("grid_map.sliding_map_size_x", 4.),
    rclcpp::Parameter("grid_map.sliding_map_size_y", 4.),
    rclcpp::Parameter("grid_map.sliding_map_size_z", 3.),
    rclcpp::Parameter("grid_map.frame_id", "map"),
    rclcpp::Parameter("grid_map.ground_height", -.8),
    rclcpp::Parameter("grid_map.static_cloud_enabled", true),
    rclcpp::Parameter("grid_map.static_prior_as_cost", false),
    rclcpp::Parameter("grid_map.reference_updates_enabled", updates),
    rclcpp::Parameter("grid_map.static_cloud_topic", topic),
  });
  auto node = std::make_shared<rclcpp::Node>(updates ? "test_live_map" : "test_offline_map", options);
  GridMap grid;
  grid.initMap(node.get());
  require(std::abs(grid.getFootprintRadius()-.36)<1e-9,"environment margin not included exactly once");
  auto pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      topic, rclcpp::QoS(1).reliable().transient_local());
  auto send = [&](double x, int sec, const std::string& frame="map") {
    pcl::PointCloud<pcl::PointXYZ> points;
    points.push_back(pcl::PointXYZ(x, .5, .2));
    points.push_back(pcl::PointXYZ(x+.2, .7, .3));
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(points, msg);
    msg.header.frame_id=frame; msg.header.stamp.sec=sec;
    pub->publish(msg);
    auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(300);
    while (std::chrono::steady_clock::now()<end) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  };
  send(.2, 10);
  require(grid.hasStaticSource(), "first reference not received");
  require(std::abs(grid.getStaticSourceMin().x()-.2)<1e-5, "first reference wrong");
  send(1., 11);
  const double expected=updates ? 1. : .2;
  require(std::abs(grid.getStaticSourceMin().x()-expected)<1e-5,
          "live updates or immutable offline mode violated");
  send(1.5, 10);
  require(std::abs(grid.getStaticSourceMin().x()-expected)<1e-5, "old map accepted");
  send(1.5, 12, "lidar_link");
  require(std::abs(grid.getStaticSourceMin().x()-expected)<1e-5, "wrong frame accepted");
  send(std::numeric_limits<double>::quiet_NaN(), 13);
  require(grid.getStaticSourceCloud().size()==2, "invalid update destroyed previous reference");
  // Known background remains a hard obstacle until actual evidence updates it.
  require(grid.getInflateOccupancy(Eigen::Vector3d(expected,.5,.2),0.)==1,
          "known background incorrectly softened inside the local window");
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    exercise(true); exercise(false);
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("grid_map.use_urdf_footprint",true),
                                 rclcpp::Parameter("grid_map.resolution",.1),
                                 rclcpp::Parameter("grid_map.sliding_map_size_x",4.),
                                 rclcpp::Parameter("grid_map.sliding_map_size_y",4.),
                                 rclcpp::Parameter("grid_map.sliding_map_size_z",3.),
                                 rclcpp::Parameter("grid_map.footprint_max_age_s",3.)});
    auto node=std::make_shared<rclcpp::Node>("test_footprint_freshness",options);
    GridMap grid;grid.initMap(node.get());
    auto pub=node->create_publisher<geometry_msgs::msg::PolygonStamped>(
        "/n01/robot_footprint",rclcpp::QoS(1).reliable().transient_local());
    auto send=[&](double age){
      geometry_msgs::msg::PolygonStamped msg;
      msg.header.frame_id="base_link";msg.header.stamp=node->now()-rclcpp::Duration::from_seconds(age);
      for(const auto& xy:std::vector<Eigen::Vector2d>{{-.5,-.2},{.5,-.2},{.5,.2},{-.5,.2}}){
        geometry_msgs::msg::Point32 p;p.x=xy.x();p.y=xy.y();msg.polygon.points.push_back(p);
      }
      pub->publish(msg);
      auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(250);
      while(std::chrono::steady_clock::now()<end){rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(5));}
    };
    send(1.5);require(grid.footprintReady(),"valid delayed model rejected by obsolete one-second gate");
    send(4.);require(!grid.footprintReady(),"expired model accepted beyond configured budget");
  }
  catch (const std::exception& error) {
    std::cerr << error.what() << std::endl;
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
