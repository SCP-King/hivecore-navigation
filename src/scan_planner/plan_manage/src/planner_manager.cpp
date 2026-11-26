//
#include <functional>
#include <fstream>
#include <plan_manage/planner_manager.h>
#include <path_searching/dyn_a_star.h>
#include <chrono>
#include <thread>
#include <queue>
#include <deque>
#include <limits>
#include <cstdint>
#include <filesystem>
#include <sstream>

namespace scan_planner
{
  namespace
  {
    void applyLinearZReference(std::vector<Eigen::Vector3d> &points, const double start_z, const double target_z)
    {
      if (points.empty())
        return;

      if (points.size() == 1)
      {
        points.front()(2) = start_z;
        return;
      }

      std::vector<double> accumulated_xy_length(points.size(), 0.0);
      for (size_t i = 1; i < points.size(); ++i)
      {
        accumulated_xy_length[i] = accumulated_xy_length[i - 1] +
                                   (points[i].head<2>() - points[i - 1].head<2>()).norm();
      }

      const double total_xy_length = accumulated_xy_length.back();
      for (size_t i = 0; i < points.size(); ++i)
      {
        const double ratio = total_xy_length > 1e-6
                                 ? accumulated_xy_length[i] / total_xy_length
                                 : static_cast<double>(i) / static_cast<double>(points.size() - 1);
        points[i](2) = start_z + ratio * (target_z - start_z);
      }

      points.front()(2) = start_z;
      points.back()(2) = target_z;
    }
  struct GlobalGridAStarNode
  {
    int idx{-1};
    double f{0.0};
    bool operator>(const GlobalGridAStarNode &other) const { return f > other.f; }
  };

  std::vector<Eigen::Vector3d> globalAStar2D(
      const pcl::PointCloud<pcl::PointXYZ> &cloud,
      const std::vector<Eigen::Vector3d> &live_points,
      const Eigen::Vector3d &source_min, const Eigen::Vector3d &source_max,
      double resolution, double inflation,
      double min_z, double max_z,
      double waypoint_spacing,
      const Eigen::Vector3d &start, const Eigen::Vector3d &end,
      const std::vector<Eigen::Vector2d> &footprint_centers = {Eigen::Vector2d::Zero()},
      const Eigen::Vector3d &ground_cutoff = Eigen::Vector3d(0,0,-1e9),
      bool static_reference_local = false,
      const Eigen::Vector3d &reference_min = Eigen::Vector3d::Zero(),
      const Eigen::Vector3d &reference_max = Eigen::Vector3d::Zero(),
      const std::vector<Eigen::Vector2d> &footprint_polygon = {},
      bool extend_realtime_reference = false, std::string* error=nullptr,
      const std::vector<Eigen::Vector3d>& uncertain_points={},
      std::function<bool(const Eigen::Vector3d&)>* free_check=nullptr,
      bool allow_start_clear=false, double start_clear_radius=0.40,
      bool allow_goal_clear=false, double goal_clear_radius=0.40,
      nav_msgs::msg::OccupancyGrid* preview=nullptr, bool has_reference=false)
  {
    if(error)error->clear();
    const auto fail=[&](const char* reason){if(error)*error=reason;return std::vector<Eigen::Vector3d>{};};
    if (cloud.empty() && live_points.empty() && !has_reference)return fail("map_reference_unavailable");
    if (!std::isfinite(resolution) || resolution <= 1e-6 ||
        !start.allFinite() || !end.allFinite() || !source_min.allFinite() || !source_max.allFinite())
      return fail("invalid_global_inputs");
    // Only the realtime reference may extend past current SLAM bounds.
    // This is an optimistic global guide, never a certificate of observed free space.
    Eigen::Vector3d lo=source_min, hi=source_max;
    if(extend_realtime_reference) {
      lo=lo.cwiseMin(start).cwiseMin(end);hi=hi.cwiseMax(start).cwiseMax(end);
    }
    const double margin = 1.0;
    const double min_x = lo.x() - margin;
    const double min_y = lo.y() - margin;
    const double max_x = hi.x() + margin;
    const double max_y = hi.y() + margin;
    // Check before integer conversion/allocation, including unreasonable clicked goals.
    const double cells_x=std::ceil((max_x-min_x)/resolution)+1.;
    const double cells_y=std::ceil((max_y-min_y)/resolution)+1.;
    if(!std::isfinite(cells_x*cells_y) || cells_x*cells_y>4000000.) return fail("global_search_budget_exceeded");
    const int width = static_cast<int>(std::ceil((max_x - min_x) / resolution)) + 1;
    const int height = static_cast<int>(std::ceil((max_y - min_y) / resolution)) + 1;
    if (width <= 2 || height <= 2 || static_cast<long long>(width) * height > 4000000LL)
      return {};
    const auto index = [width, height, min_x, min_y, resolution](double x, double y) {
      const int ix = static_cast<int>(std::floor((x - min_x) / resolution));
      const int iy = static_cast<int>(std::floor((y - min_y) / resolution));
      if (ix < 0 || iy < 0 || ix >= width || iy >= height) return -1;
      return iy * width + ix;
    };
    std::vector<uint8_t> occupied(static_cast<size_t>(width) * height, 0);
    std::vector<uint8_t> reference(occupied.size(),0);
    for (const auto &p : cloud.points)
    {
      if (p.z < min_z || p.z > max_z || p.z <= ground_cutoff.x()*p.x+ground_cutoff.y()*p.y+ground_cutoff.z()) continue;
      const int id = index(p.x, p.y);
      if (id >= 0) {
        occupied[id]=1; // Known background obstacles never become soft just by entering the window.
      }
    }
    // The static map is the global baseline; current sensor hits can block a
    // corridor and force a different route instead of repeatedly initializing
    // local optimization through the same newly occupied space.
    for (const auto &p : live_points) {
      if (p.z()<min_z || p.z()>max_z || p.z() <= ground_cutoff.x()*p.x()+ground_cutoff.y()*p.y()+ground_cutoff.z()) continue;
      const int id=index(p.x(),p.y());
      if (id>=0) occupied[id]=1;
    }
    (void)static_reference_local;(void)reference_min;(void)reference_max;
    for(const auto& p:uncertain_points){
      if(!p.allFinite() || p.z()<min_z || p.z()>max_z || p.z()<=ground_cutoff.x()*p.x()+ground_cutoff.y()*p.y()+ground_cutoff.z())continue;
      int id=index(p.x(),p.y());if(id>=0)reference[id]=1;
    }
    // Rasterize the configured metric radius, not ceil(radius/resolution)
    // cells. Keep fractional footprint offsets in metres as well: rounding
    // each disk's centre adds up to another half-cell of unintended clearance.
    std::vector<Eigen::Vector2i> inflation_offsets;
    for (const auto &center : footprint_centers) {
      const int min_dx = static_cast<int>(std::ceil((-inflation-center.x())/resolution-1e-9));
      const int max_dx = static_cast<int>(std::floor((inflation-center.x())/resolution+1e-9));
      const int min_dy = static_cast<int>(std::ceil((-inflation-center.y())/resolution-1e-9));
      const int max_dy = static_cast<int>(std::floor((inflation-center.y())/resolution+1e-9));
      for (int dy=min_dy;dy<=max_dy;++dy)
        for (int dx=min_dx;dx<=max_dx;++dx)
          if ((Eigen::Vector2d(dx*resolution,dy*resolution)+center).squaredNorm()
              <= inflation*inflation+1e-12)
            inflation_offsets.emplace_back(dx,dy);
    }
    std::sort(inflation_offsets.begin(),inflation_offsets.end(),
        [](const Eigen::Vector2i &a,const Eigen::Vector2i &b) {
          return a.y()!=b.y() ? a.y()<b.y() : a.x()<b.x();
        });
    inflation_offsets.erase(std::unique(inflation_offsets.begin(),inflation_offsets.end(),
        [](const Eigen::Vector2i &a,const Eigen::Vector2i &b) { return a==b; }),
        inflation_offsets.end());
    if(!footprint_polygon.empty()) {
      inflation_offsets=plan_env_detail::polygonOffsets(footprint_polygon,inflation,resolution);
      for(auto &v:inflation_offsets)v=-v;
    }
    const auto inflate_layer=[&](const std::vector<uint8_t>& source) {
    std::vector<uint8_t> result=source;
    for (int iy = 0; iy < height; ++iy)
    {
      for (int ix = 0; ix < width; ++ix)
      {
        if (!source[static_cast<size_t>(iy) * width + ix]) continue;
        for (const auto &offset : inflation_offsets)
        {
          const int yy = iy + offset.y();
          if (yy < 0 || yy >= height) continue;
          const int xx = ix + offset.x();
          if (xx < 0 || xx >= width) continue;
          result[static_cast<size_t>(yy) * width + xx] = 1;
        }
      }
    }
    return result;
    };
    auto inflated=inflate_layer(occupied);
    const auto reference_inflated=inflate_layer(reference);

    // Prefer the middle of a free corridor instead of hugging the inflated
    // obstacle boundary. BFS distance is measured in grid cells from the
    // inflated obstacle set; the extra edge cost decays with clearance.
    std::vector<int> clearance(static_cast<size_t>(width)*height, -1);
    std::deque<int> clearance_queue;
    for(int i=0;i<width*height;++i) if(inflated[static_cast<size_t>(i)]) {
      clearance[static_cast<size_t>(i)]=0;clearance_queue.push_back(i);
    }
    while(!clearance_queue.empty()) {
      const int cur=clearance_queue.front();clearance_queue.pop_front();
      const int cx=cur%width,cy=cur/width;
      const int dirs4[4][2]={{1,0},{-1,0},{0,1},{0,-1}};
      for(const auto &d:dirs4) {
        const int nx=cx+d[0],ny=cy+d[1];
        if(nx<0||ny<0||nx>=width||ny>=height)continue;
        const int nid=ny*width+nx;
        if(clearance[static_cast<size_t>(nid)]>=0)continue;
        clearance[static_cast<size_t>(nid)]=clearance[static_cast<size_t>(cur)]+1;
        clearance_queue.push_back(nid);
      }
    }
    const int desired_clearance=std::max(1,static_cast<int>(std::ceil(.60/resolution)));
    auto clearance_cost=[&](int id,double step) {
      const int d=id>=0?clearance[static_cast<size_t>(id)]:-1;
      if(d<0)return step*2.0;
      const double shortage=std::max(0,desired_clearance-d)/double(desired_clearance);
      return step*(1.0+0.9*shortage);
    };
    const int start_id = index(start.x(), start.y());
    const int goal_id = index(end.x(), end.y());
    if (start_id < 0 || goal_id < 0) return fail("start_or_goal_outside_map_bounds");
    if (inflated[start_id] && allow_start_clear) {
      const int r = std::max(1, static_cast<int>(std::ceil(start_clear_radius / resolution)));
      const int sx = start_id % width, sy = start_id / width;
      for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
          if (dx*dx + dy*dy > r*r) continue;
          const int x = sx + dx, y = sy + dy;
          if (x < 0 || x >= width || y < 0 || y >= height) continue;
          inflated[static_cast<size_t>(y) * width + x] = 0;
        }
      }
      RCLCPP_WARN(rclcpp::get_logger("path_searching"),
                  "Global A* cleared stale start inflation within %.2f m; local start envelope is free",
                  start_clear_radius);
    }
    if (inflated[goal_id] && allow_goal_clear) {
      const int r = std::max(1, static_cast<int>(std::ceil(goal_clear_radius / resolution)));
      const int gx = goal_id % width, gy = goal_id / width;
      for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
          if (dx*dx + dy*dy > r*r) continue;
          const int x = gx + dx, y = gy + dy;
          if (x < 0 || x >= width || y < 0 || y >= height) continue;
          inflated[static_cast<size_t>(y) * width + x] = 0;
        }
      }
      RCLCPP_WARN(rclcpp::get_logger("path_searching"),
                  "Global A* cleared stale goal inflation within %.2f m; local goal envelope is free",
                  goal_clear_radius);
    }
    if(free_check)*free_check=[index,inflated](const Eigen::Vector3d& p){int id=index(p.x(),p.y());return id>=0 && !inflated[id];};
    if (preview) {
      preview->header.frame_id = "map";
      preview->info.width = width;
      preview->info.height = height;
      preview->info.resolution = resolution;
      preview->info.origin.position.x = min_x;
      preview->info.origin.position.y = min_y;
      preview->info.origin.orientation.w = 1.;
      preview->data.assign(inflated.size(), -1);
      for (size_t i = 0; i < inflated.size(); ++i)
        if (inflated[i]) preview->data[i] = 100;
      std::vector<int> queue;
      if (!inflated[start_id]) {
        queue.push_back(start_id);
        preview->data[start_id] = 0;
      }
      const int dirs[8][2] = {{1,0},{-1,0},{0,1},{0,-1},
                              {1,1},{1,-1},{-1,1},{-1,-1}};
      for (size_t i = 0; i < queue.size(); ++i) {
        const int id = queue[i], x = id % width, y = id / width;
        for (const auto& d : dirs) {
          const int nx=x+d[0], ny=y+d[1];
          if (nx<0||ny<0||nx>=width||ny>=height) continue;
          const int next=ny*width+nx;
          if (preview->data[next] != -1) continue;
          // Same diagonal corner rule as the real A*: do not cut a corner
          // between two blocked cells. Otherwise preview connectivity would
          // disagree with the planner.
          if (d[0] != 0 && d[1] != 0 &&
              (inflated[y*width+nx] || inflated[ny*width+x])) continue;
          preview->data[next] = 0;
          queue.push_back(next);
        }
      }
      return {};
    }
    if (inflated[start_id] || inflated[goal_id])
    {
      RCLCPP_ERROR(rclcpp::get_logger("path_searching"),
                   "Global PCD A* start/goal occupied after 2D inflation: start_occ=%d goal_occ=%d z=[%.2f,%.2f]",
                   static_cast<int>(inflated[start_id]), static_cast<int>(inflated[goal_id]),
                   min_z, max_z);
      return fail(inflated[start_id]?"start_occupied":"goal_occupied");
    }
    const int gx = goal_id % width, gy = goal_id / width;
    const auto heuristic = [gx, gy, width](int id) {
      const int x = id % width, y = id / width;
      const double dx = std::abs(x - gx), dy = std::abs(y - gy);
      return std::max(dx, dy) + (std::sqrt(2.0) - 1.0) * std::min(dx, dy);
    };
    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> g_score(static_cast<size_t>(width) * height, INF);
    std::vector<int> parent(static_cast<size_t>(width) * height, -1);
    std::vector<uint8_t> closed(static_cast<size_t>(width) * height, 0);
    std::priority_queue<GlobalGridAStarNode, std::vector<GlobalGridAStarNode>,
                        std::greater<GlobalGridAStarNode>> open;
    g_score[start_id] = 0.0;
    open.push({start_id, heuristic(start_id)});
    const int dirs[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
    bool found = false;
    while (!open.empty())
    {
      const int current = open.top().idx;
      open.pop();
      if (closed[current]) continue;
      closed[current] = 1;
      if (current == goal_id) { found = true; break; }
      const int cx = current % width, cy = current / width;
      for (const auto &d : dirs)
      {
        const int nx = cx + d[0], ny = cy + d[1];
        if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
        const int nid = ny * width + nx;
        if (inflated[nid] || closed[nid]) continue;
        if (d[0] != 0 && d[1] != 0 &&
            (inflated[cy * width + nx] || inflated[ny * width + cx]))
          continue;
        const double step = (d[0] != 0 && d[1] != 0) ? std::sqrt(2.0) : 1.0;
        const double tentative = g_score[current] + clearance_cost(nid,step);
        if (tentative + 1e-9 < g_score[nid])
        {
          g_score[nid] = tentative;
          parent[nid] = current;
          open.push({nid, tentative + heuristic(nid)});
        }
      }
    }
    if (!found) return fail("global_no_route");
    std::vector<int> ids;
    for (int cur = goal_id; cur != -1; cur = parent[cur])
    {
      ids.push_back(cur);
      if (cur == start_id) break;
    }
    if (ids.empty() || ids.back() != start_id) return {};
    std::reverse(ids.begin(), ids.end());
    std::vector<Eigen::Vector3d> path;
    path.reserve(ids.size());
    for (size_t i = 0; i < ids.size(); ++i)
    {
      const int x = ids[i] % width, y = ids[i] / width;
      const double px = min_x + (x + 0.5) * resolution;
      const double py = min_y + (y + 0.5) * resolution;
      const double t = std::min(1.0, static_cast<double>(i) / std::max<size_t>(1, ids.size() - 1));
      path.emplace_back(px, py, start.z() + t * (end.z() - start.z()));
    }
    path.front() = start;
    path.back() = end;

    // Collapse the dense one-cell-per-0.05m A* path into collision-free
    // waypoints spaced up to `waypoint_spacing` apart.  minSnapTraj builds a
    // dense 6n x 6n system, so this is the difference between ~1 s and a few
    // milliseconds for a 6 m goal.  Every retained chord is sampled against
    // the same inflated 2D grid the A* used, so shortcutting cannot cut a
    // corner through an obstacle.
    if (waypoint_spacing > resolution * 1.5 && path.size() > 2)
    {
      const auto cell_free = [&](double x, double y) {
        const int id = index(x, y);
        return id >= 0 && !inflated[id];
      };
      const auto segment_free = [&](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
        const double d = (b - a).head<2>().norm();
        const int steps = std::max(2, static_cast<int>(std::ceil(d / (resolution * 0.5))));
        for (int s = 0; s <= steps; ++s)
        {
          const double t = static_cast<double>(s) / static_cast<double>(steps);
          if (!cell_free(a.x() + t * (b.x() - a.x()), a.y() + t * (b.y() - a.y())))
            return false;
        }
        return true;
      };

      std::vector<Eigen::Vector3d> reduced;
      reduced.reserve(path.size());
      reduced.push_back(path.front());
      size_t i = 0;
      while (i + 1 < path.size())
      {
        size_t best = i + 1;
        for (size_t j = i + 2; j < path.size(); ++j)
        {
          if ((path[j] - path[i]).head<2>().norm() > waypoint_spacing)
            break;
          bool same_risk=true;for(size_t k=i;k<=j;++k){int id=index(path[k].x(),path[k].y());if(id>=0&&reference_inflated[id]){same_risk=false;break;}}
          if (same_risk && segment_free(path[i], path[j]))best = j;
        }
        reduced.push_back(path[best]);
        i = best;
      }
      path.swap(reduced);
    }
    return path;
  }
  } // namespace

  // SECTION interfaces for setup and query

  SCANPlannerManager::SCANPlannerManager() {}

  SCANPlannerManager::~SCANPlannerManager() { std::cout << "des manager" << std::endl; }

  void SCANPlannerManager::initPlanModules(rclcpp::Node *node, PlanningVisualization::Ptr vis)
  {
    node_ = node;
    search_grid_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/planning/search_grid", rclcpp::QoS(1).transient_local());
    /* read algorithm parameters */
    const auto get_double = [node](const std::string &name, double default_value) {
      if (!node->has_parameter(name)) node->declare_parameter<double>(name, default_value);
      return node->get_parameter(name).as_double();
    };
    pp_.max_vel_ = get_double("manager.max_vel", -1.0);
    pp_.max_acc_ = get_double("manager.max_acc", -1.0);
    pp_.max_jerk_ = get_double("manager.max_jerk", -1.0);
    pp_.vel_tolerance_ = get_double("optimization.vel_tolerance", 1.0);
    pp_.acc_tolerance_ = get_double("optimization.acc_tolerance", 1.0);
    pp_.feasibility_tolerance_ = get_double("manager.feasibility_tolerance", 0.0);
    pp_.ctrl_pt_dist = get_double("manager.control_points_distance", -1.0);
    pp_.planning_horizon_ = get_double("manager.planning_horizon", 5.0);
    // The global min-snap QP is O(n^3) in the number of A* waypoints.  The raw
    // 0.05 m grid path gives ~20 points per metre, which costs ~1 s for a 6 m
    // goal; collision-free resampling to a few decimetres costs microseconds.
    pp_.global_waypoint_spacing_ = get_double("manager.global_waypoint_spacing", 0.30);
    pp_.min_turn_radius_ = get_double("manager.min_turn_radius", 0.60);
    if (!std::isfinite(pp_.min_turn_radius_) || pp_.min_turn_radius_ < .1 || pp_.min_turn_radius_ > 5.)
      pp_.min_turn_radius_ = .60;
    pp_.max_yaw_rate_ = get_double("manager.max_yaw_rate", 0.45);
    if (!std::isfinite(pp_.max_yaw_rate_) || pp_.max_yaw_rate_ <= 0. || pp_.max_yaw_rate_ > 2.)
      pp_.max_yaw_rate_ = .45;

    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    grid_map_->initMap(node_);

    bspline_optimizer_rebound_.reset(new BsplineOptimizer);
    bspline_optimizer_rebound_->setParam(node_);
    bspline_optimizer_rebound_->setEnvironment(grid_map_);
    bspline_optimizer_rebound_->a_star_.reset(new AStar);
    bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    visualization_ = vis;
  }

  // !SECTION

  // SECTION rebond replanning

  namespace {
  // Smooth dense A* waypoints with the same inflated grid used for search.
  // This is a local corner-rounding pass; it cannot move a point through an
  // obstacle and does not replace the later min-snap trajectory.
  bool smoothGlobalPath(std::vector<Eigen::Vector3d>& path, double min_turn_radius,
                        double resolution,
                        const std::function<bool(const Eigen::Vector3d&)>& free_check)
  {
    if (path.size() < 3 || !(min_turn_radius > 0.)) return true;
    const auto segment_free = [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
      const double d = (b-a).head<2>().norm();
      const int steps = std::max(2, int(std::ceil(d / std::max(0.02, resolution * 0.5))));
      for (int i=0;i<=steps;++i) {
        const double t=double(i)/steps;
        if (!free_check(a + t*(b-a))) return false;
      }
      return true;
    };
    for (int iter=0; iter<40; ++iter) {
      bool changed=false;
      for (size_t i=1;i+1<path.size();++i) {
        Eigen::Vector3d cand=0.25*path[i-1]+0.5*path[i]+0.25*path[i+1];
        if (free_check(cand) && segment_free(path[i-1],cand) && segment_free(cand,path[i+1])) {
          if ((cand-path[i]).norm()>1e-4) { path[i]=cand; changed=true; }
        }
      }
      if (!changed) break;
    }
    // Reject a path that still contains a tighter corner than the chassis
    // can track. The caller reports this as a global curvature failure.
    double worst_k=0.;
    for (size_t i=1;i+1<path.size();++i) {
      const Eigen::Vector3d a=path[i-1], b=path[i], c=path[i+1];
      const double ab=(b-a).head<2>().norm(), bc=(c-b).head<2>().norm(), ca=(c-a).head<2>().norm();
      if (ab<1e-6 || bc<1e-6 || ca<1e-6) continue;
      const double area=std::abs((b.x()-a.x())*(c.y()-a.y())-(b.y()-a.y())*(c.x()-a.x()))/2.;
      if (area<1e-9) continue;
      const double radius=ab*bc*ca/(4.0*area);
      if (radius < min_turn_radius) worst_k=std::max(worst_k,1.0/radius);
    }
    if (worst_k > 0.) {
      RCLCPP_WARN(rclcpp::get_logger("path_searching"),
        "Global polyline curvature: worst_radius=%.3f m (limit %.3f m)",
        1.0 / worst_k, min_turn_radius);
    }
    return worst_k <= 0.;
  }
  } // namespace

  bool SCANPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {
    last_local_error_.clear();
    // Structured per-attempt telemetry. The stock logs only report a few
    // generic strings and cannot answer why an individual local attempt failed.
    static int attempt_no = 0;
    static std::ofstream attempt_log = [] {
      std::error_code ec;
      std::filesystem::create_directories("/home/demo/lyy/n01_runtime/local_attempts", ec);
      std::ostringstream name;
      name << "/home/demo/lyy/n01_runtime/local_attempts/attempts_"
           << std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::system_clock::now().time_since_epoch()).count()
           << ".jsonl";
      return std::ofstream(name.str(), std::ios::app);
    }();
    const int this_attempt = ++attempt_no;
    size_t point_count = 0, ctrl_cols = 0, path_count = 0, first_path_points = 0;
    auto log_attempt = [&](const char* stage, bool ok, const char* detail) {
      if (!attempt_log.is_open()) return;
      attempt_log << "{\"attempt\":" << this_attempt
                  << ",\"stage\":\"" << stage << "\""
                  << ",\"ok\":" << (ok ? "true" : "false")
                  << ",\"reason\":\"" << (detail ? detail : "") << "\""
                  << ",\"last_error\":\"" << last_local_error_ << "\""
                  << ",\"start\":[" << start_pt.x() << "," << start_pt.y() << "," << start_pt.z() << "]"
                  << ",\"target\":[" << local_target_pt.x() << "," << local_target_pt.y() << "," << local_target_pt.z() << "]"
                  << ",\"distance\":" << (start_pt-local_target_pt).norm()
                  << ",\"point_set\":" << point_count
                  << ",\"ctrl_cols\":" << ctrl_cols
                  << ",\"a_star_paths\":" << path_count
                  << ",\"first_path_points\":" << first_path_points
                  << ",\"continuous_failures\":" << continuous_failures_count_
                  << "}\n";
      attempt_log.flush();
    };

    static int count = 0;
    std::cout << endl
              << "[rebo replan]: -------------------------------------" << count++ << std::endl;
    cout.precision(3);
    cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
         << endl;

    // The FSM arrival tolerance is 0.1 m. A 0.2 m rejection leaves an
    // unreachable annulus where every replan fails before optimization.
    if ((start_pt - local_target_pt).norm() < 0.02)
    {
      last_local_error_="local_target_too_close_without_arrival";
      cout << "Close to goal" << endl;
      continuous_failures_count_++;
      log_attempt("too_close", false, "local_target_too_close_without_arrival");
      return false;
    }

    auto t_start = std::chrono::steady_clock::now();
    double t_init = 0.0, t_opt = 0.0, t_refine = 0.0;

    /*** STEP 1: INIT ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        flag_first_call = false;
        flag_force_polynomial = false;

        PolynomialTraj gl_traj;

        double dist = (start_pt - local_target_pt).norm();
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

        if (!flag_randomPolyTraj)
        {
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else
        {
          Eigen::Vector3d horizon_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizon_dir)).normalized();
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizon_dir * 0.8 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989);
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        double t;
        bool flag_too_far;
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      else // Initial path generated from previous trajectory.
      {

        double t;
        double t_cur = (node_->now() - local_data_.start_time_).seconds();

        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              last_local_error_="local_initialization_has_no_path";
              RCLCPP_ERROR(node_->get_logger(), "pseudo_arc_length is empty; aborting replan");
              continuous_failures_count_++;
              log_attempt("init", false, "local_initialization_has_no_path");
              return false;
            }
          }
        }

        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help

        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizon_ / pp_.ctrl_pt_dist * 3) // The initial path is abnormally too long!
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    applyLinearZReference(point_set, start_pt(2), local_target_pt(2));

    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    vector<vector<Eigen::Vector3d>> a_star_paths;
    a_star_paths = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);
    point_count = point_set.size();
    ctrl_cols = static_cast<size_t>(ctrl_pts.cols());
    path_count = a_star_paths.size();
    if (!a_star_paths.empty()) first_path_points = a_star_paths.front().size();

    t_init = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    static int vis_id = 0;
    visualization_->displayInitPathList(point_set, 0.2, 0);
    visualization_->displayAStarList(a_star_paths, vis_id);

    t_start = std::chrono::steady_clock::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
    cout << "first_optimize_step_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success)
    {
      // Separate an empty/short A* initialization from a genuine optimizer
      // collision failure. The controller previously collapsed all of these
      // into the same generic string.
      if (first_path_points == 0)
        last_local_error_ = "a_star_init_zero_paths";
      else if (first_path_points < 2)
        last_local_error_ = "a_star_init_short_path";
      else
        last_local_error_ = "scan_optimizer_no_collision_free_solution";
      // visualization_->displayOptimalList( ctrl_pts, vis_id );
      continuous_failures_count_++;
      log_attempt("optimize", false, last_local_error_.c_str());
      return false;
    }
    //visualization_->displayOptimalList( ctrl_pts, vis_id );

    t_opt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
    t_start = std::chrono::steady_clock::now();

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    double ratio;
    bool flag_step_2_success = true;
    if (!pos.checkFeasibility(ratio, false))
    {
      cout << "Need to reallocate time." << endl;

      Eigen::MatrixXd optimal_control_points;
      flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
      if (flag_step_2_success)
        pos = UniformBspline(optimal_control_points, 3, ts);
    }

    if (!flag_step_2_success || !checkDynamicFeasibility(pos))
    {
      if (last_local_error_.empty())
        last_local_error_ = "scan_refinement_or_dynamic_limit_failed";
      printf("\033[34mThis refined trajectory is unsafe or dynamically infeasible. Skip publishing it.\n\033[0m");
      continuous_failures_count_++;
      log_attempt("refine", false, last_local_error_.c_str());
      return false;
    }

    t_refine = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    // save planned results
    updateTrajInfo(pos, node_->now());

    cout << "total time:\033[42m" << (t_init + t_opt + t_refine)
         << "\033[0m,optimize:" << (t_init + t_opt) << ",refine:" << t_refine << endl;

    // success. YoY
    continuous_failures_count_ = 0;
    log_attempt("done", true, "success");
    return true;
  }

  bool SCANPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), node_->now());

    return true;
  }

  bool SCANPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    if (waypoints.empty())
      return false;

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    for (size_t i = 0; i < points.size() - 1; i++)
    {
      total_len += (points[i + 1] - points[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    auto time_now = node_->now();
    global_data_.setPolyline(inter_points, pp_.max_vel_, time_now);
    return true;
  }

  void SCANPlannerManager::refreshSearchPreview(const Eigen::Vector3d &position)
  {
    const double now = node_->now().seconds();
    if (!grid_map_->hasStaticSource() || !grid_map_->footprintReady() ||
        (last_preview_s_ >= 0. && now - last_preview_s_ < 1.))
      return;
    last_preview_s_ = now;
    searchGlobal(position, position, nullptr, true);
  }

  std::vector<Eigen::Vector3d> SCANPlannerManager::searchGlobal(
      const Eigen::Vector3d &start_pos, const Eigen::Vector3d &end_pos,
      std::function<bool(const Eigen::Vector3d&)>* free_check, bool preview)
  {
    grid_map_->refreshGlobalGrid();
    grid_map_->rebuildPlanningGrid();
    const bool planning_ready = grid_map_->hasPlanningGrid();
    const bool global_grid_ready = grid_map_->hasGlobalGrid();
    nav_msgs::msg::OccupancyGrid grid;
    auto path = globalAStar2D(
          planning_ready ? grid_map_->getPlanningOccupancyCloud()
                         : ((global_grid_ready && !grid_map_->usesTransientFusion())
                                ? grid_map_->getGlobalOccupancyCloud()
                                : grid_map_->getStaticSourceCloud()),
          planning_ready ? std::vector<Eigen::Vector3d>{} : grid_map_->getLiveObstaclePoints(),
          planning_ready ? grid_map_->getPlanningMin()
                         : (global_grid_ready?grid_map_->getGlobalMin():grid_map_->getStaticSourceMin()),
          planning_ready ? grid_map_->getPlanningMax()
                         : (global_grid_ready?grid_map_->getGlobalMax():grid_map_->getStaticSourceMax()),
          planning_ready ? grid_map_->getPlanningResolution()
                         : (global_grid_ready?grid_map_->getGlobalResolution():grid_map_->getResolution()),
          grid_map_->getFootprintRadius(),
          start_pos.z() - grid_map_->getBodyDown(),
          start_pos.z() + grid_map_->getBodyUp(),
          pp_.global_waypoint_spacing_,
          start_pos, end_pos, grid_map_->getFootprintCenters(), grid_map_->getGroundCutoffPlane(),
          grid_map_->usesStaticReferenceCost(),grid_map_->getLocalMinimum(),grid_map_->getLocalMaximum(),grid_map_->getFootprintPolygon(),(grid_map_->usesRealtimeReference()||grid_map_->extendsReferenceToGoal()),preview ? nullptr : &last_global_error_,grid_map_->getUnconfirmedPoints(),free_check,
          preview,0.40,false,0.40,preview ? &grid : nullptr,
          planning_ready || grid_map_->hasStaticSource());
    if (preview && !grid.data.empty()) {
      // BFS is allowed to traverse unknown, but unknown must not be painted as
      // known-free/reachable. White comes only from the planning grid's 0.
      for(int iy=0;iy<grid.info.height;++iy)for(int ix=0;ix<grid.info.width;++ix){
        auto&v=grid.data[static_cast<size_t>(iy)*grid.info.width+ix];
        if(v==100)continue;
        const double wx=grid.info.origin.position.x+(ix+.5)*grid.info.resolution;
        const double wy=grid.info.origin.position.y+(iy+.5)*grid.info.resolution;
        const int state=grid_map_->planningCellState(wx,wy);
        if(state!=0)v=static_cast<int8_t>(state);
      }
      grid.header.stamp = node_->now();
      search_grid_pub_->publish(grid);
    }
    return path;
  }

  bool SCANPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    last_global_error_.clear();
    std::function<bool(const Eigen::Vector3d&)> reference_free;
    // Global path: A* over the COMPLETE static PCD bounds (not the 10 m
    // sliding local GridMap and not a hard-coded pool size).
    std::vector<Eigen::Vector3d> inter_points;
    const auto t_global_start = std::chrono::steady_clock::now();
    if(grid_map_ && !grid_map_->footprintReady()) {
      last_global_error_="urdf_footprint_missing_or_stale";
      RCLCPP_ERROR(node_->get_logger(),"Global planning blocked: urdf_footprint_missing_or_stale");
      return false;
    }
    if (grid_map_ && grid_map_->hasStaticSource())
    {
      if(grid_map_->usesRealtimeReference() &&
         (end_pos.x()<grid_map_->getStaticSourceMin().x() || end_pos.x()>grid_map_->getStaticSourceMax().x() ||
          end_pos.y()<grid_map_->getStaticSourceMin().y() || end_pos.y()>grid_map_->getStaticSourceMax().y()))
        RCLCPP_INFO_THROTTLE(node_->get_logger(),*node_->get_clock(),2000,
          "Realtime far goal retained: extending optimistic global reference; local execution still needs current scene");
      inter_points = searchGlobal(start_pos, end_pos, &reference_free, false);
      if (inter_points.size() >= 3 &&
          !smoothGlobalPath(inter_points, pp_.min_turn_radius_, grid_map_->getResolution(), reference_free)) {
        last_global_error_ = "global_path_curvature_exceeds_min_turn_radius";
        RCLCPP_ERROR(node_->get_logger(),
          "Global path rejected: curvature exceeds min_turn_radius=%.2f m", pp_.min_turn_radius_);
        return false;
      }
      if (inter_points.size() < 2)
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "Global PCD A* failed: start=(%.2f,%.2f,%.2f) goal=(%.2f,%.2f,%.2f) "
                     "pcd_bounds=[%.2f,%.2f]x[%.2f,%.2f]",
                     start_pos.x(), start_pos.y(), start_pos.z(),
                     end_pos.x(), end_pos.y(), end_pos.z(),
                     grid_map_->getStaticSourceMin().x(), grid_map_->getStaticSourceMax().x(),
                     grid_map_->getStaticSourceMin().y(), grid_map_->getStaticSourceMax().y());
        return false;
      }
      const auto t_astar_done = std::chrono::steady_clock::now();
      RCLCPP_INFO(node_->get_logger(),
                  "Global PCD A*: cloud=%zu path=%zu astar_ms=%.1f bounds=[%.2f,%.2f]x[%.2f,%.2f]",
                  grid_map_->getStaticSourceCloud().size(), inter_points.size(),
                  std::chrono::duration<double, std::milli>(t_astar_done - t_global_start).count(),
                  grid_map_->getStaticSourceMin().x(), grid_map_->getStaticSourceMax().x(),
                  grid_map_->getStaticSourceMin().y(), grid_map_->getStaticSourceMax().y());

    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Global PCD A* unavailable: static PCD source is not ready");
      last_global_error_="map_reference_unavailable";
      return false;
    }

    const auto t_smooth_start = std::chrono::steady_clock::now();
    // The global reference is intentionally a collision-checked polyline with
    // rounded corners, not a min-snap polynomial through every A* waypoint.
    // This preserves long straight segments and avoids introducing extra
    // curvature that the local 4WS controller cannot track.
    if (inter_points.size() < 2) return false;
    auto time_now = node_->now();
    global_data_.setPolyline(inter_points, pp_.max_vel_, time_now);
    const auto t_smooth_done = std::chrono::steady_clock::now();
    RCLCPP_INFO(node_->get_logger(),
                "Global polyline reference: pts=%zu smooth_ms=%.1f total_ms=%.1f",
                inter_points.size(),
                std::chrono::duration<double, std::milli>(t_smooth_done - t_smooth_start).count(),
                std::chrono::duration<double, std::milli>(t_smooth_done - t_global_start).count());
    return true;
  }

  bool SCANPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_rebound_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void SCANPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  bool SCANPlannerManager::checkDynamicFeasibility(UniformBspline position_traj)
  {
    UniformBspline vel_traj = position_traj.getDerivative();
    UniformBspline acc_traj = vel_traj.getDerivative();
    const double duration = position_traj.getTimeSum();
    const double sample_dt = std::max(0.01, std::min(0.05, duration / 50.0));
    const double vel_limit = pp_.max_vel_ + pp_.vel_tolerance_;
    const double acc_limit = pp_.max_acc_ + pp_.acc_tolerance_;

    for (double t = 0.0; t < duration + 1e-6; t += sample_dt)
    {
      const double tc = std::min(t, duration);
      Eigen::Vector3d vel = vel_traj.evaluateDeBoorT(tc);
      if (vel.norm() > vel_limit)
      {
        last_local_error_ = "scan_local_velocity_limit";
        RCLCPP_WARN(node_->get_logger(),
                    "Dynamic feasibility failed: velocity at t=%.3f is %.3f > %.3f",
                    tc, vel.norm(), vel_limit);
        return false;
      }

      Eigen::Vector3d acc = acc_traj.evaluateDeBoorT(tc);
      if (acc.norm() > acc_limit)
      {
        last_local_error_ = "scan_local_acceleration_limit";
        RCLCPP_WARN(node_->get_logger(),
                    "Dynamic feasibility failed: acceleration at t=%.3f is %.3f > %.3f",
                    tc, acc.norm(), acc_limit);
        return false;
      }

      // 4WS chassis supports in-place pivot at start/end and speed-scaled
      // turning (w = v * k) in trajectory_monitor/navigation_controller.
      // Only reject pathological mid-trajectory geometric cusps; do not
      // reject low-speed start/end transients or nominal-speed yaw rate.
      const double vxy = vel.head<2>().norm();
      if (vxy > 0.15 && tc >= 0.4 && tc <= duration - 0.4)
      {
        const double yaw_rate = std::abs(vel.x() * acc.y() - vel.y() * acc.x()) / (vxy * vxy);
        const double curvature = yaw_rate / vxy;
        const double min_r = std::max(0.05, std::min(pp_.min_turn_radius_, 0.15));
        if (curvature > 1.0 / min_r)
        {
          last_local_error_ = "scan_local_curvature_exceeded";
          RCLCPP_WARN(node_->get_logger(),
                      "Local curvature feasibility failed: t=%.3f R=%.3f < %.3f",
                      tc, 1.0 / std::max(1e-6, curvature), min_r);
          return false;
        }
      }
    }

    return true;
  }

  void SCANPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;
    // double length = bspline.getLength(0.1);
    // int seg_num = ceil(length / pp_.ctrl_pt_dist);

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace scan_planner
