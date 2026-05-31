#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "puzzlebot_interfaces/srv/plan_path.hpp"

class PathPlannerNode : public rclcpp::Node
{
public:
  using GridCell   = std::pair<int, int>;
  using WorldPoint = std::pair<double, double>;
  using PlanPath   = puzzlebot_interfaces::srv::PlanPath;

  struct PairHash
  {
    std::size_t operator()(const GridCell & p) const noexcept
    {
      const auto h1 = std::hash<int>{}(p.first);
      const auto h2 = std::hash<int>{}(p.second);
      return h1 ^ (h2 << 1);
    }
  };

  explicit PathPlannerNode()
  : Node("path_planner_node"),
    rng_(std::random_device{}())
  {
    declare_parameter<std::string>("global_frame",      "map");
    declare_parameter<std::string>("robot_frame",       "odom");
    declare_parameter<std::string>("dynamic_map_topic", "/map");
    declare_parameter<double>("robot_radius",     0.20);
    declare_parameter<bool>  ("smooth_path",      true);
    declare_parameter<double>("waypoint_spacing", 0.05);
    declare_parameter<int>   ("max_iter",          5000);
    declare_parameter<int>   ("step_size",         2);
    declare_parameter<int>   ("connect_step_size", 2);
    declare_parameter<int>   ("max_connect_steps", 1);
    declare_parameter<double>("goal_bias_rate",    0.02);
    declare_parameter<int>   ("connect_threshold", 3);
    declare_parameter<int>   ("random_seed",       0);
    declare_parameter<double>("alpha", 1.0);
    declare_parameter<double>("beta",  0.10);
    declare_parameter<double>("gamma", 0.20);
    declare_parameter<double>("tf_timeout", 0.1);
    declare_parameter<bool>("debug_visualize_tree",          false);
    declare_parameter<int> ("debug_visualize_every",         1);
    declare_parameter<bool>("debug_publish_unsmoothed_path", false);

    global_frame_      = get_parameter("global_frame").as_string();
    robot_frame_       = get_parameter("robot_frame").as_string();
    dynamic_map_topic_ = get_parameter("dynamic_map_topic").as_string();
    robot_radius_      = get_parameter("robot_radius").as_double();
    smooth_path_       = get_parameter("smooth_path").as_bool();
    waypoint_spacing_  = get_parameter("waypoint_spacing").as_double();
    max_iter_          = static_cast<int>(get_parameter("max_iter").as_int());
    step_size_         = static_cast<int>(get_parameter("step_size").as_int());
    connect_step_size_ = static_cast<int>(get_parameter("connect_step_size").as_int());
    max_connect_steps_ = static_cast<int>(get_parameter("max_connect_steps").as_int());
    goal_bias_rate_    = get_parameter("goal_bias_rate").as_double();
    connect_threshold_ = static_cast<int>(get_parameter("connect_threshold").as_int());
    alpha_ = get_parameter("alpha").as_double();
    beta_  = get_parameter("beta").as_double();
    gamma_ = get_parameter("gamma").as_double();
    tf_timeout_              = get_parameter("tf_timeout").as_double();
    debug_visualize_tree_    = get_parameter("debug_visualize_tree").as_bool();
    debug_visualize_every_   = static_cast<int>(get_parameter("debug_visualize_every").as_int());
    debug_pub_unsmoothed_    = get_parameter("debug_publish_unsmoothed_path").as_bool();

    rclcpp::QoS map_qos(1);
    map_qos.reliable();
    map_qos.transient_local();

    const int seed = static_cast<int>(get_parameter("random_seed").as_int());
    if (seed != 0) {
      rng_.seed(static_cast<std::mt19937::result_type>(seed));
    }

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    path_pub_ = create_publisher<nav_msgs::msg::Path>("/path", 10);
    unsmoothed_path_pub_ =
      create_publisher<nav_msgs::msg::Path>("/path_planner_debug/unsmoothed_path", 10);
    tree_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/path_planner_debug/tree", 10);

    dynamic_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      dynamic_map_topic_,
      map_qos,
      std::bind(&PathPlannerNode::dynamic_map_callback, this, std::placeholders::_1));

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose",
      10,
      std::bind(&PathPlannerNode::goal_callback, this, std::placeholders::_1));

    plan_path_srv_ = create_service<PlanPath>(
      "/plan_path",
      std::bind(
        &PathPlannerNode::plan_path_service_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "Path planner ready | map=%s | robot_frame=%s | tf: %s->%s | service=/plan_path",
      dynamic_map_topic_.c_str(),
      robot_frame_.c_str(),
      global_frame_.c_str(),
      robot_frame_.c_str());
  }

private:
  void dynamic_map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (msg->info.width == 0 || msg->info.height == 0 || msg->data.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty dynamic map.");
      return;
    }
    if (!msg->header.frame_id.empty()) {
      global_frame_ = msg->header.frame_id;
    }
    resolution_ = static_cast<double>(msg->info.resolution);
    origin_x_   = msg->info.origin.position.x;
    origin_y_   = msg->info.origin.position.y;
    const int width  = static_cast<int>(msg->info.width);
    const int height = static_cast<int>(msg->info.height);
    grid_ = cv::Mat::zeros(height, width, CV_8UC1);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const int k = y * width + x;
        if (k >= static_cast<int>(msg->data.size())) {
          grid_.at<uint8_t>(y, x) = 1;
          continue;
        }
        grid_.at<uint8_t>(y, x) = (msg->data[static_cast<std::size_t>(k)] == 0) ? 0 : 1;
      }
    }
    inflate_obstacles(robot_radius_);
    free_cells_ = precompute_free_cells();
    has_map_    = !free_cells_.empty();
    if (!has_map_) {
      RCLCPP_WARN(get_logger(), "Dynamic map has no free cells.");
      return;
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Dynamic map updated | size=(%d,%d) | res=%.4f | free_cells=%zu",
      grid_.cols, grid_.rows, resolution_, free_cells_.size());
  }

  void inflate_obstacles(double robot_radius)
  {
    const int inflation_cells = static_cast<int>(std::ceil(robot_radius / resolution_));
    if (inflation_cells <= 0) return;
    const int kernel_size = 2 * inflation_cells + 1;
    cv::dilate(grid_, grid_, cv::Mat::ones(kernel_size, kernel_size, CV_8UC1));
  }

  std::vector<GridCell> precompute_free_cells() const
  {
    std::vector<GridCell> cells;
    if (grid_.empty()) return cells;
    cells.reserve(static_cast<std::size_t>(grid_.rows * grid_.cols / 2));
    for (int y = 0; y < grid_.rows; ++y) {
      for (int x = 0; x < grid_.cols; ++x) {
        if (grid_.at<uint8_t>(y, x) == 0) {
          cells.emplace_back(x, y);
        }
      }
    }
    return cells;
  }

  GridCell world_to_grid(double x, double y) const
  {
    return {
      static_cast<int>(std::floor((x - origin_x_) / resolution_)),
      static_cast<int>(std::floor((y - origin_y_) / resolution_))
    };
  }

  WorldPoint grid_to_world(int gx, int gy) const
  {
    return {
      origin_x_ + (static_cast<double>(gx) + 0.5) * resolution_,
      origin_y_ + (static_cast<double>(gy) + 0.5) * resolution_
    };
  }

  bool is_cell_free(const GridCell & cell) const
  {
    if (grid_.empty()) return false;
    if (cell.first  < 0 || cell.first  >= grid_.cols) return false;
    if (cell.second < 0 || cell.second >= grid_.rows) return false;
    return grid_.at<uint8_t>(cell.second, cell.first) == 0;
  }

  // Obtiene pose del robot haciendo lookup en el TF tree (global_frame_ -> robot_frame_)
  bool get_robot_pose(double & x, double & y, double & yaw)
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        global_frame_,
        robot_frame_,
        tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_));

      x = tf.transform.translation.x;
      y = tf.transform.translation.y;

      tf2::Quaternion q;
      tf2::fromMsg(tf.transform.rotation, q);
      double roll, pitch;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "TF lookup %s->%s failed: %s",
        global_frame_.c_str(), robot_frame_.c_str(), ex.what());
      return false;
    }
  }

  static std::pair<double, double> yaw_to_quaternion(double yaw)
  {
    return {std::sin(yaw / 2.0), std::cos(yaw / 2.0)};
  }

  static double distance(const GridCell & a, const GridCell & b)
  {
    return std::hypot(
      static_cast<double>(a.first  - b.first),
      static_cast<double>(a.second - b.second));
  }

  geometry_msgs::msg::Point grid_cell_to_point(const GridCell & cell, double z = 0.02) const
  {
    const auto [x, y] = grid_to_world(cell.first, cell.second);
    geometry_msgs::msg::Point p;
    p.x = x; p.y = y; p.z = z;
    return p;
  }

  void maybe_publish_tree(
    const std::unordered_map<GridCell, GridCell, PairHash> & parent_start,
    const std::unordered_map<GridCell, bool,     PairHash> & has_parent_start,
    const std::unordered_map<GridCell, GridCell, PairHash> & parent_goal,
    const std::unordered_map<GridCell, bool,     PairHash> & has_parent_goal,
    int iteration)
  {
    if (!debug_visualize_tree_) return;
    if (debug_visualize_every_ > 0 && iteration % debug_visualize_every_ != 0) return;

    auto make_marker = [&](
      int id, const std::string & ns,
      const std::unordered_map<GridCell, GridCell, PairHash> & parent,
      const std::unordered_map<GridCell, bool,     PairHash> & has_parent,
      float r, float g, float b)
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = global_frame_;
      m.header.stamp    = get_clock()->now();
      m.ns              = ns;
      m.id              = id;
      m.type            = visualization_msgs::msg::Marker::LINE_LIST;
      m.action          = visualization_msgs::msg::Marker::ADD;
      m.scale.x         = 0.025;
      m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.85f;
      m.pose.orientation.w = 1.0;
      for (const auto & [child, par] : parent) {
        const auto it = has_parent.find(child);
        if (it == has_parent.end() || !it->second) continue;
        m.points.push_back(grid_cell_to_point(par));
        m.points.push_back(grid_cell_to_point(child));
      }
      return m;
    };

    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(
      make_marker(0, "start_tree", parent_start, has_parent_start, 0.0f, 0.8f, 0.1f));
    markers.markers.push_back(
      make_marker(1, "goal_tree",  parent_goal,  has_parent_goal,  0.1f, 0.3f, 1.0f));
    tree_pub_->publish(markers);
  }

  void clear_tree_markers()
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = global_frame_;
    m.header.stamp    = get_clock()->now();
    m.action          = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(m);
    tree_pub_->publish(markers);
  }

  GridCell sample_free(const GridCell & target)
  {
    std::uniform_real_distribution<double> prob(0.0, 1.0);
    if (prob(rng_) < goal_bias_rate_) return target;
    std::uniform_int_distribution<std::size_t> idx(0, free_cells_.size() - 1);
    return free_cells_[idx(rng_)];
  }

  GridCell heuristic_best_node(
    const std::vector<GridCell> & tree,
    const std::unordered_map<GridCell, double, PairHash> & g_cost,
    const GridCell & q_sample,
    const GridCell & target) const
  {
    GridCell best = tree.front();
    double best_score = std::numeric_limits<double>::infinity();
    for (const auto & q : tree) {
      const double score =
        alpha_ * distance(q, q_sample) +
        beta_  * g_cost.at(q) +
        gamma_ * distance(q, target);
      if (score < best_score) {
        best_score = score;
        best = q;
      }
    }
    return best;
  }

  GridCell steer(const GridCell & from, const GridCell & to, int step_size) const
  {
    const int dx = to.first  - from.first;
    const int dy = to.second - from.second;
    const double dist = std::hypot(static_cast<double>(dx), static_cast<double>(dy));
    if (dist < 1e-9) return from;
    if (dist <= static_cast<double>(step_size)) return to;
    const double scale = static_cast<double>(step_size) / dist;
    return {
      static_cast<int>(std::round(from.first  + dx * scale)),
      static_cast<int>(std::round(from.second + dy * scale))
    };
  }

  bool line_is_free(const GridCell & a, const GridCell & b) const
  {
    int x0 = a.first,  y0 = a.second;
    const int x1 = b.first, y1 = b.second;
    const int dx = std::abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    const int dy = std::abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    while (true) {
      if (!is_cell_free({x0, y0})) return false;
      if (x0 == x1 && y0 == y1) break;
      const int e2 = 2 * err;
      if (e2 > -dy) { err -= dy; x0 += sx; }
      if (e2 <  dx) { err += dx; y0 += sy; }
    }
    return true;
  }

  bool extend_heuristic(
    std::vector<GridCell> & tree,
    std::unordered_map<GridCell, GridCell, PairHash> & parent,
    std::unordered_map<GridCell, bool,     PairHash> & has_parent,
    std::unordered_map<GridCell, double,   PairHash> & g_cost,
    const GridCell & q_sample,
    const GridCell & heuristic_target,
    int step_size,
    GridCell & q_new,
    GridCell & q_near)
  {
    q_near = heuristic_best_node(tree, g_cost, q_sample, heuristic_target);
    q_new  = steer(q_near, q_sample, step_size);
    if (q_new == q_near || !is_cell_free(q_new) || !line_is_free(q_near, q_new)) {
      return false;
    }
    const double new_cost    = g_cost.at(q_near) + distance(q_near, q_new);
    const auto   old_cost_it = g_cost.find(q_new);
    if (old_cost_it != g_cost.end() && new_cost >= old_cost_it->second) {
      return false;
    }
    if (old_cost_it == g_cost.end()) {
      tree.push_back(q_new);
    }
    parent[q_new]     = q_near;
    has_parent[q_new] = true;
    g_cost[q_new]     = new_cost;
    return true;
  }

  bool connect_heuristic(
    std::vector<GridCell> & tree,
    std::unordered_map<GridCell, GridCell, PairHash> & parent,
    std::unordered_map<GridCell, bool,     PairHash> & has_parent,
    std::unordered_map<GridCell, double,   PairHash> & g_cost,
    const GridCell & q_target,
    int max_steps,
    GridCell & meeting_node)
  {
    for (int i = 0; i < max_steps; ++i) {
      GridCell q_new, q_near;
      if (!extend_heuristic(tree, parent, has_parent, g_cost,
                            q_target, q_target, connect_step_size_, q_new, q_near))
      {
        return false;
      }
      if (distance(q_new, q_target) <= static_cast<double>(connect_threshold_) &&
          line_is_free(q_new, q_target))
      {
        if (g_cost.find(q_target) == g_cost.end()) {
          tree.push_back(q_target);
          parent[q_target]     = q_new;
          has_parent[q_target] = true;
          g_cost[q_target]     = g_cost.at(q_new) + distance(q_new, q_target);
        }
        meeting_node = q_target;
        return true;
      }
    }
    return false;
  }

  std::vector<GridCell> trace_to_root(
    const std::unordered_map<GridCell, GridCell, PairHash> & parent,
    const std::unordered_map<GridCell, bool,     PairHash> & has_parent,
    const GridCell & node) const
  {
    std::vector<GridCell> path{node};
    while (true) {
      const auto hp = has_parent.find(path.back());
      if (hp == has_parent.end() || !hp->second) break;
      path.push_back(parent.at(path.back()));
    }
    return path;
  }

  std::vector<GridCell> build_bidirectional_path(
    const std::unordered_map<GridCell, GridCell, PairHash> & parent_start,
    const std::unordered_map<GridCell, bool,     PairHash> & has_parent_start,
    const std::unordered_map<GridCell, GridCell, PairHash> & parent_goal,
    const std::unordered_map<GridCell, bool,     PairHash> & has_parent_goal,
    const GridCell & meeting_node) const
  {
    auto start_to_meet = trace_to_root(parent_start, has_parent_start, meeting_node);
    std::reverse(start_to_meet.begin(), start_to_meet.end());
    const auto meet_to_goal = trace_to_root(parent_goal, has_parent_goal, meeting_node);
    std::vector<GridCell> full = start_to_meet;
    full.insert(full.end(), meet_to_goal.begin() + 1, meet_to_goal.end());
    return full;
  }

  bool plan_connect(const GridCell & start, const GridCell & goal, std::vector<GridCell> & path)
  {
    if (debug_visualize_tree_) clear_tree_markers();

    std::vector<GridCell> tree_a{start};
    std::unordered_map<GridCell, GridCell, PairHash> parent_a;
    std::unordered_map<GridCell, bool,     PairHash> has_parent_a;
    std::unordered_map<GridCell, double,   PairHash> g_a;
    parent_a[start]     = start;
    has_parent_a[start] = false;
    g_a[start]          = 0.0;

    std::vector<GridCell> tree_b{goal};
    std::unordered_map<GridCell, GridCell, PairHash> parent_b;
    std::unordered_map<GridCell, bool,     PairHash> has_parent_b;
    std::unordered_map<GridCell, double,   PairHash> g_b;
    parent_b[goal]     = goal;
    has_parent_b[goal] = false;
    g_b[goal]          = 0.0;

    GridCell target_a = goal;
    GridCell target_b = start;
    bool a_is_start   = true;

    for (int iter = 0; iter < max_iter_; ++iter) {
      GridCell q_new, q_near;
      const bool extended = extend_heuristic(
        tree_a, parent_a, has_parent_a, g_a,
        sample_free(target_a), target_a, step_size_, q_new, q_near);

      const auto & ps  = a_is_start ? parent_a    : parent_b;
      const auto & hps = a_is_start ? has_parent_a : has_parent_b;
      const auto & pg  = a_is_start ? parent_b    : parent_a;
      const auto & hpg = a_is_start ? has_parent_b : has_parent_a;
      maybe_publish_tree(ps, hps, pg, hpg, iter);

      if (extended) {
        GridCell meeting_node;
        if (connect_heuristic(tree_b, parent_b, has_parent_b, g_b,
                              q_new, max_connect_steps_, meeting_node))
        {
          if (a_is_start) {
            path = build_bidirectional_path(
              parent_a, has_parent_a, parent_b, has_parent_b, meeting_node);
          } else {
            path = build_bidirectional_path(
              parent_b, has_parent_b, parent_a, has_parent_a, meeting_node);
          }
          maybe_publish_tree(ps, hps, pg, hpg, 0);
          return true;
        }
      }

      std::swap(tree_a,       tree_b);
      std::swap(parent_a,     parent_b);
      std::swap(has_parent_a, has_parent_b);
      std::swap(g_a,          g_b);
      std::swap(target_a,     target_b);
      a_is_start = !a_is_start;
    }
    return false;
  }

  std::vector<GridCell> smooth_path_grid(const std::vector<GridCell> & path) const
  {
    if (path.size() <= 2) return path;
    std::vector<GridCell> smoothed{path.front()};
    std::size_t i = 0;
    while (i < path.size() - 1) {
      std::size_t j = path.size() - 1;
      while (j > i + 1 && !line_is_free(path[i], path[j])) --j;
      smoothed.push_back(path[j]);
      i = j;
    }
    return smoothed;
  }

  std::vector<WorldPoint> resample_path_world(
    const std::vector<WorldPoint> & path, double spacing) const
  {
    if (path.size() <= 1 || spacing <= 0.0) return path;
    std::vector<WorldPoint> resampled{path.front()};
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
      const auto [x0, y0] = path[i];
      const auto [x1, y1] = path[i + 1];
      const double dx = x1 - x0, dy = y1 - y0;
      const double dist = std::hypot(dx, dy);
      if (dist < 1e-6) continue;
      const int steps = static_cast<int>(std::floor(dist / spacing));
      for (int s = 1; s <= steps; ++s) {
        const double t = (static_cast<double>(s) * spacing) / dist;
        if (t >= 1.0) continue;
        resampled.emplace_back(x0 + t * dx, y0 + t * dy);
      }
      resampled.emplace_back(x1, y1);
    }
    return resampled;
  }

  std::vector<WorldPoint> grid_path_to_world(
    const std::vector<GridCell> & grid_path,
    const WorldPoint * start_world,
    const WorldPoint * goal_world,
    bool resample) const
  {
    std::vector<WorldPoint> world_path;
    world_path.reserve(grid_path.size());
    for (const auto & [gx, gy] : grid_path) {
      world_path.push_back(grid_to_world(gx, gy));
    }
    if (!world_path.empty()) {
      if (start_world) world_path.front() = *start_world;
      if (goal_world)  world_path.back()  = *goal_world;
    }
    if (resample) return resample_path_world(world_path, waypoint_spacing_);
    return world_path;
  }

  nav_msgs::msg::Path build_path_msg(const std::vector<WorldPoint> & path_world) const
  {
    nav_msgs::msg::Path msg;
    msg.header.frame_id = global_frame_;
    msg.header.stamp    = this->now();
    for (std::size_t i = 0; i < path_world.size(); ++i) {
      const auto [x, y] = path_world[i];
      double yaw = 0.0;
      if (i + 1 < path_world.size()) {
        const auto [nx, ny] = path_world[i + 1];
        yaw = std::atan2(ny - y, nx - x);
      } else if (path_world.size() > 1) {
        const auto [px, py] = path_world[i - 1];
        yaw = std::atan2(y - py, x - px);
      }
      const auto [qz, qw] = yaw_to_quaternion(yaw);
      geometry_msgs::msg::PoseStamped pose;
      pose.header              = msg.header;
      pose.pose.position.x     = x;
      pose.pose.position.y     = y;
      pose.pose.orientation.z  = qz;
      pose.pose.orientation.w  = qw;
      msg.poses.push_back(pose);
    }
    return msg;
  }

  std::tuple<bool, std::string, nav_msgs::msg::Path> compute_path(
    const geometry_msgs::msg::PoseStamped & goal_msg)
  {
    if (!has_map_)
      return {false, "No dynamic map received yet.", nav_msgs::msg::Path()};
    if (free_cells_.empty())
      return {false, "Map has no free cells.", nav_msgs::msg::Path()};

    double rx, ry, ryaw;
    if (!get_robot_pose(rx, ry, ryaw))
      return {false, "TF lookup failed, no robot pose available.", nav_msgs::msg::Path()};

    if (!goal_msg.header.frame_id.empty() && goal_msg.header.frame_id != global_frame_) {
      return {false,
        "Goal frame \"" + goal_msg.header.frame_id +
        "\" != planner frame \"" + global_frame_ + "\".",
        nav_msgs::msg::Path()};
    }

    const double gx = goal_msg.pose.position.x;
    const double gy = goal_msg.pose.position.y;
    const GridCell start_cell = world_to_grid(rx, ry);
    const GridCell goal_cell  = world_to_grid(gx, gy);

    if (!is_cell_free(start_cell))
      return {false, "Start cell is occupied or outside map.", nav_msgs::msg::Path()};
    if (!is_cell_free(goal_cell))
      return {false, "Goal cell is occupied or outside map.",  nav_msgs::msg::Path()};

    std::vector<GridCell> path_grid;
    if (!plan_connect(start_cell, goal_cell, path_grid))
      return {false, "RRT-Connect could not find a path.", nav_msgs::msg::Path()};

    const std::size_t raw_points = path_grid.size();
    const WorldPoint start_world{rx, ry};
    const WorldPoint goal_world{gx, gy};

    if (debug_pub_unsmoothed_) {
      unsmoothed_path_pub_->publish(
        build_path_msg(grid_path_to_world(path_grid, &start_world, &goal_world, false)));
    }
    if (smooth_path_) {
      path_grid = smooth_path_grid(path_grid);
    }
    const auto path_world = grid_path_to_world(path_grid, &start_world, &goal_world, true);
    auto path_msg = build_path_msg(path_world);
    path_pub_->publish(path_msg);

    const std::string message =
      "Path found | waypoints=" + std::to_string(path_world.size()) +
      " (raw=" + std::to_string(raw_points) +
      ", smoothed=" + std::to_string(path_grid.size()) + ")";
    return {true, message, path_msg};
  }

  void plan_path_service_callback(
    const std::shared_ptr<PlanPath::Request>  request,
    std::shared_ptr<PlanPath::Response> response)
  {
    const auto [success, message, path] = compute_path(request->goal);
    response->success = success;
    response->message = message;
    response->path    = path;
    if (success) RCLCPP_INFO(get_logger(), "%s", message.c_str());
    else         RCLCPP_WARN(get_logger(), "%s", message.c_str());
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const auto [success, message, path] = compute_path(*msg);
    (void)path;
    if (success) RCLCPP_INFO(get_logger(), "%s", message.c_str());
    else         RCLCPP_WARN(get_logger(), "%s", message.c_str());
  }

  std::string global_frame_;
  std::string robot_frame_;
  std::string dynamic_map_topic_;
  double robot_radius_     = 0.0;
  bool   smooth_path_      = true;
  double waypoint_spacing_ = 0.05;
  int    max_iter_          = 5000;
  int    step_size_         = 2;
  int    connect_step_size_ = 2;
  int    max_connect_steps_ = 1;
  double goal_bias_rate_    = 0.02;
  int    connect_threshold_ = 3;
  double alpha_ = 1.0;
  double beta_  = 0.10;
  double gamma_ = 0.20;
  double tf_timeout_ = 0.1;
  bool debug_visualize_tree_  = false;
  int  debug_visualize_every_ = 1;
  bool debug_pub_unsmoothed_  = false;

  cv::Mat grid_;
  double  resolution_ = 0.05;
  double  origin_x_   = 0.0;
  double  origin_y_   = 0.0;
  std::vector<GridCell> free_cells_;
  bool has_map_ = false;

  std::mt19937 rng_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                  path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                  unsmoothed_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tree_pub_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr    dynamic_map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Service<PlanPath>::SharedPtr                             plan_path_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PathPlannerNode>());
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("path_planner_node"), "Fatal: %s", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}