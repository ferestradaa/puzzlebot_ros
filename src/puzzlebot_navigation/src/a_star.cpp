#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
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
      return std::hash<int>{}(p.first) ^ (std::hash<int>{}(p.second) << 1);
    }
  };

  explicit PathPlannerNode()
  : Node("path_planner_node")
  {
    declare_parameter<std::string>("global_frame",                    "map");
    declare_parameter<std::string>("robot_frame",                     "base_link");
    declare_parameter<std::string>("dynamic_map_topic",               "/map");
    declare_parameter<double>     ("robot_radius",                    0.20);
    declare_parameter<int>        ("start_free_search_radius_cells",  20);
    declare_parameter<int>        ("endpoint_ignore_waypoints",       15);
    declare_parameter<bool>       ("smooth_path",                     true);
    declare_parameter<double>     ("waypoint_spacing",                0.05);
    declare_parameter<double>     ("tf_timeout",                      0.05);

    // A* weight: f = g + heuristic_weight * h
    // 1.0 = true A* (optimal, slower); >1.0 = weighted A* (faster, slightly suboptimal)
    declare_parameter<double>     ("heuristic_weight",                1.2);

    // Extra cost per cell adjacent to an obstacle in the inflated grid.
    // Pushes paths away from walls without changing traversability.
    declare_parameter<double>     ("obstacle_proximity_cost",         8.0);

    // Second inflation pass (cells) added on top of robot_radius inflation.
    // These cells are traversable but expensive — acts as a soft clearance buffer.
    declare_parameter<int>        ("soft_clearance_cells",            3);

    // Catmull-Rom smoothing passes. More passes = smoother but may cut corners more.
    declare_parameter<int>        ("smooth_iterations",               2);

    global_frame_                = get_parameter("global_frame").as_string();
    robot_frame_                 = get_parameter("robot_frame").as_string();
    dynamic_map_topic_           = get_parameter("dynamic_map_topic").as_string();
    robot_radius_                = get_parameter("robot_radius").as_double();
    start_free_search_radius_cells_ =
      static_cast<int>(get_parameter("start_free_search_radius_cells").as_int());
    endpoint_ignore_waypoints_   =
      std::max(0, static_cast<int>(get_parameter("endpoint_ignore_waypoints").as_int()));
    smooth_path_                 = get_parameter("smooth_path").as_bool();
    waypoint_spacing_            = get_parameter("waypoint_spacing").as_double();
    tf_timeout_                  = get_parameter("tf_timeout").as_double();
    heuristic_weight_            = get_parameter("heuristic_weight").as_double();
    obstacle_proximity_cost_     = get_parameter("obstacle_proximity_cost").as_double();
    soft_clearance_cells_        =
      static_cast<int>(get_parameter("soft_clearance_cells").as_int());
    smooth_iterations_           =
      static_cast<int>(get_parameter("smooth_iterations").as_int());

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    path_pub_ = create_publisher<nav_msgs::msg::Path>("/path", 10);

    dynamic_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      dynamic_map_topic_,
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&PathPlannerNode::dynamic_map_callback, this, std::placeholders::_1));

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&PathPlannerNode::goal_callback, this, std::placeholders::_1));

    plan_path_srv_ = create_service<PlanPath>(
      "/plan_path",
      std::bind(&PathPlannerNode::plan_path_service_callback, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(),
      "A* path planner ready | map_topic=%s | service=/plan_path | tf=%s->%s | "
      "robot_radius=%.2f | endpoint_ignore_waypoints=%d | heuristic_weight=%.2f | "
      "obstacle_proximity_cost=%.1f | soft_clearance_cells=%d",
      dynamic_map_topic_.c_str(), global_frame_.c_str(), robot_frame_.c_str(),
      robot_radius_, endpoint_ignore_waypoints_, heuristic_weight_,
      obstacle_proximity_cost_, soft_clearance_cells_);

    param_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&PathPlannerNode::on_parameter_update, this, std::placeholders::_1));
  }

private:
  rcl_interfaces::msg::SetParametersResult on_parameter_update(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : parameters) {
      if (param.get_name() == "endpoint_ignore_waypoints") {
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
          result.successful = false;
          result.reason = "endpoint_ignore_waypoints must be an integer.";
          return result;
        }
        const int value = static_cast<int>(param.as_int());
        if (value < 0) {
          result.successful = false;
          result.reason = "endpoint_ignore_waypoints must be >= 0.";
          return result;
        }
        endpoint_ignore_waypoints_ = value;
        RCLCPP_INFO(get_logger(), "Updated endpoint_ignore_waypoints=%d", value);
      } else if (param.get_name() == "heuristic_weight") {
        heuristic_weight_ = param.as_double();
      } else if (param.get_name() == "obstacle_proximity_cost") {
        obstacle_proximity_cost_ = param.as_double();
      }
    }
    return result;
  }

  void dynamic_map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (msg->info.width == 0 || msg->info.height == 0 || msg->data.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty map.");
      return;
    }
    global_frame_ = msg->header.frame_id.empty() ? global_frame_ : msg->header.frame_id;
    resolution_   = static_cast<double>(msg->info.resolution);
    origin_       = {
      msg->info.origin.position.x,
      msg->info.origin.position.y,
      quaternion_to_yaw(
        msg->info.origin.orientation.x, msg->info.origin.orientation.y,
        msg->info.origin.orientation.z, msg->info.origin.orientation.w)
    };

    const int width  = static_cast<int>(msg->info.width);
    const int height = static_cast<int>(msg->info.height);

    // Binary obstacle grid: 1 = obstacle, 0 = free
    grid_ = cv::Mat::zeros(height, width, CV_8UC1);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const int k     = y * width + x;
        const int8_t v  = (k < static_cast<int>(msg->data.size())) ? msg->data[k] : 100;
        grid_.at<uint8_t>(y, x) = (v == 0) ? 0 : 1;
      }
    }
    grid_raw_ = grid_.clone();

    // Hard inflation: cells inside robot_radius become impassable
    inflate_obstacles(robot_radius_);

    // Cost grid: 0 for free cells far from walls, higher near walls
    build_cost_grid();

    free_cells_ = precompute_free_cells();
    has_map_    = !free_cells_.empty();

    if (!has_map_) {
      RCLCPP_WARN(get_logger(), "Map received, but it has no free cells.");
      return;
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
      "Map updated | size=(%d,%d) | resolution=%.4f | free_cells=%zu",
      grid_.cols, grid_.rows, resolution_, free_cells_.size());
  }

  void inflate_obstacles(double robot_radius)
  {
    const int inflation_cells = static_cast<int>(std::ceil(robot_radius / resolution_));
    if (inflation_cells <= 0) { return; }
    const int kernel_size = 2 * inflation_cells + 1;
    cv::dilate(grid_, grid_, cv::Mat::ones(kernel_size, kernel_size, CV_8UC1),
               cv::Point(-1, -1), 1);
  }

  // Build a float cost grid.
  // Cells inside the hard inflation boundary are impassable (cost = inf).
  // Cells within soft_clearance_cells_ of an obstacle get a proximity penalty.
  // This steers A* away from tight corridors without blocking them.
  void build_cost_grid()
  {
    cost_grid_ = cv::Mat(grid_.rows, grid_.cols, CV_32FC1,
                         cv::Scalar(std::numeric_limits<float>::infinity()));

    if (soft_clearance_cells_ <= 0 || obstacle_proximity_cost_ <= 0.0) {
      // No soft buffer: just set free cells to 0
      for (int y = 0; y < grid_.rows; ++y) {
        for (int x = 0; x < grid_.cols; ++x) {
          if (grid_.at<uint8_t>(y, x) == 0) {
            cost_grid_.at<float>(y, x) = 0.0f;
          }
        }
      }
      return;
    }

    // Distance transform on the inflated binary grid
    // (distance to nearest obstacle cell, in pixels)
    cv::Mat obstacle_mask(grid_.rows, grid_.cols, CV_8UC1);
    for (int y = 0; y < grid_.rows; ++y) {
      for (int x = 0; x < grid_.cols; ++x) {
        // distanceTransform wants 0 = obstacle, 255 = free
        obstacle_mask.at<uint8_t>(y, x) = (grid_.at<uint8_t>(y, x) == 0) ? 255 : 0;
      }
    }
    cv::Mat dist_transform;
    cv::distanceTransform(obstacle_mask, dist_transform, cv::DIST_L2, cv::DIST_MASK_PRECISE);

    const float soft_f = static_cast<float>(soft_clearance_cells_);
    const float prox_f = static_cast<float>(obstacle_proximity_cost_);

    for (int y = 0; y < grid_.rows; ++y) {
      for (int x = 0; x < grid_.cols; ++x) {
        if (grid_.at<uint8_t>(y, x) != 0) {
          // Hard obstacle: leave as inf
          continue;
        }
        const float d = dist_transform.at<float>(y, x);
        if (d <= soft_f) {
          // Proximity penalty: linear from prox_f (at d=0) to 0 (at d=soft_f)
          cost_grid_.at<float>(y, x) = prox_f * (1.0f - d / soft_f);
        } else {
          cost_grid_.at<float>(y, x) = 0.0f;
        }
      }
    }
  }

  std::vector<GridCell> precompute_free_cells() const
  {
    std::vector<GridCell> cells;
    if (grid_.empty()) { return cells; }
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
      static_cast<int>(std::floor((x - origin_[0]) / resolution_)),
      static_cast<int>(std::floor((y - origin_[1]) / resolution_))
    };
  }

  WorldPoint grid_to_world(int gx, int gy) const
  {
    return {
      origin_[0] + (static_cast<double>(gx) + 0.5) * resolution_,
      origin_[1] + (static_cast<double>(gy) + 0.5) * resolution_
    };
  }

  bool is_cell_inside(const GridCell & cell) const
  {
    if (grid_.empty()) { return false; }
    return cell.first  >= 0 && cell.first  < grid_.cols &&
           cell.second >= 0 && cell.second < grid_.rows;
  }

  bool is_cell_free(const GridCell & cell) const
  {
    if (!is_cell_inside(cell)) { return false; }
    return grid_.at<uint8_t>(cell.second, cell.first) == 0;
  }

  float cell_cost(const GridCell & cell) const
  {
    if (!is_cell_inside(cell)) { return std::numeric_limits<float>::infinity(); }
    return cost_grid_.at<float>(cell.second, cell.first);
  }

  GridCell snap_to_free(const GridCell & cell, int search_radius) const
  {
    if (is_cell_free(cell)) { return cell; }
    for (int r = 1; r <= search_radius; ++r) {
      GridCell best      = cell;
      double   best_dist = std::numeric_limits<double>::infinity();
      for (int dx = -r; dx <= r; ++dx) {
        for (int dy = -r; dy <= r; ++dy) {
          if (std::abs(dx) != r && std::abs(dy) != r) { continue; }
          GridCell candidate{cell.first + dx, cell.second + dy};
          if (!is_cell_inside(candidate) || !is_cell_free(candidate)) { continue; }
          const double d = std::hypot(static_cast<double>(dx), static_cast<double>(dy));
          if (d < best_dist) { best_dist = d; best = candidate; }
        }
      }
      if (best_dist < std::numeric_limits<double>::infinity()) { return best; }
    }
    return cell;
  }

  bool get_robot_pose(double & x, double & y, double & yaw)
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        global_frame_, robot_frame_,
        tf2::TimePointZero, tf2::durationFromSec(tf_timeout_));
      tf2::Quaternion q;
      tf2::fromMsg(tf.transform.rotation, q);
      double roll, pitch;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      x = tf.transform.translation.x;
      y = tf.transform.translation.y;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "TF lookup %s -> %s failed: %s",
        global_frame_.c_str(), robot_frame_.c_str(), ex.what());
      return false;
    }
  }

  static double quaternion_to_yaw(double x, double y, double z, double w)
  {
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  }

  static std::pair<double, double> yaw_to_quaternion(double yaw)
  {
    return {std::sin(yaw / 2.0), std::cos(yaw / 2.0)};
  }

  // Euclidean heuristic in grid cells
  static double heuristic(const GridCell & a, const GridCell & b)
  {
    return std::hypot(
      static_cast<double>(a.first  - b.first),
      static_cast<double>(a.second - b.second));
  }

  // 8-connected neighbors with correct diagonal cost
  std::vector<std::pair<GridCell, double>> neighbors(const GridCell & cell) const
  {
    static const std::array<std::pair<int,int>, 8> dirs = {{
      {1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}
    }};
    std::vector<std::pair<GridCell, double>> result;
    result.reserve(8);
    for (const auto & [dx, dy] : dirs) {
      GridCell nb{cell.first + dx, cell.second + dy};
      if (!is_cell_free(nb)) { continue; }

      // For diagonal moves, both cardinal cells must be free (avoids corner cutting)
      if (dx != 0 && dy != 0) {
        if (!is_cell_free({cell.first + dx, cell.second}) ||
            !is_cell_free({cell.first, cell.second + dy})) {
          continue;
        }
      }

      const double move_cost = (dx != 0 && dy != 0) ? 1.4142135623730951 : 1.0;
      const double proximity_cost = static_cast<double>(cell_cost(nb));
      result.emplace_back(nb, move_cost + proximity_cost);
    }
    return result;
  }

  // Weighted A* returning the grid path from start to goal.
  // Returns empty vector if no path exists.
  std::vector<GridCell> plan_astar(const GridCell & start, const GridCell & goal)
  {
    using Entry = std::pair<double, GridCell>;

    std::unordered_map<GridCell, double,   PairHash> g_cost;
    std::unordered_map<GridCell, GridCell, PairHash> came_from;
    std::unordered_map<GridCell, bool,     PairHash> closed;

    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;

    g_cost[start]    = 0.0;
    came_from[start] = start;
    open.emplace(heuristic_weight_ * heuristic(start, goal), start);

    while (!open.empty()) {
      const auto [f_current, current] = open.top();
      open.pop();

      if (closed.count(current)) { continue; }
      closed[current] = true;

      if (current == goal) {
        // Reconstruct path
        std::vector<GridCell> path;
        GridCell node = goal;
        while (node != start) {
          path.push_back(node);
          node = came_from.at(node);
        }
        path.push_back(start);
        std::reverse(path.begin(), path.end());
        return path;
      }

      const double g_cur = g_cost.at(current);
      for (const auto & [nb, cost] : neighbors(current)) {
        if (closed.count(nb)) { continue; }
        const double g_new = g_cur + cost;
        auto it = g_cost.find(nb);
        if (it == g_cost.end() || g_new < it->second) {
          g_cost[nb]    = g_new;
          came_from[nb] = current;
          const double f = g_new + heuristic_weight_ * heuristic(nb, goal);
          open.emplace(f, nb);
        }
      }
    }
    return {};
  }

  bool line_is_free(const GridCell & a, const GridCell & b) const
  {
    int x0 = a.first, y0 = a.second;
    const int x1 = b.first, y1 = b.second;
    const int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
    const int sx = (x0 < x1) ? 1 : -1, sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    while (true) {
      if (!is_cell_free({x0, y0})) { return false; }
      if (x0 == x1 && y0 == y1)   { break; }
      const int e2 = 2 * err;
      if (e2 > -dy) { err -= dy; x0 += sx; }
      if (e2 <  dx) { err += dx; y0 += sy; }
    }
    return true;
  }

  bool segment_validation_can_be_skipped(std::size_t idx, std::size_t count) const
  {
    if (endpoint_ignore_waypoints_ <= 0 || count == 0) { return false; }
    const std::size_t ig = static_cast<std::size_t>(endpoint_ignore_waypoints_);
    return idx < ig || idx >= count - std::min(ig, count);
  }

  // Greedy shortcut smoothing on the grid path
  std::vector<GridCell> smooth_path_grid(const std::vector<GridCell> & path) const
  {
    if (path.size() <= 2) { return path; }
    std::vector<GridCell> out{path.front()};
    std::size_t i = 0;
    while (i < path.size() - 1) {
      std::size_t j = path.size() - 1;
      while (j > i + 1 && !line_is_free(path[i], path[j])) { --j; }
      out.push_back(path[j]);
      i = j;
    }
    return out;
  }

  std::vector<WorldPoint> catmull_rom_smooth(
    const std::vector<WorldPoint> & pts, double spacing) const
  {
    if (pts.size() <= 2) { return pts; }
    const double safe_spacing = std::max(0.01, spacing);
    std::vector<WorldPoint> out;
    out.reserve(pts.size() * 4);
    out.push_back(pts.front());

    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
      const auto [x0, y0] = (i == 0)             ? pts[i]     : pts[i - 1];
      const auto [x1, y1] = pts[i];
      const auto [x2, y2] = pts[i + 1];
      const auto [x3, y3] = (i + 2 < pts.size()) ? pts[i + 2] : pts[i + 1];

      const double seg_len = std::hypot(x2 - x1, y2 - y1);
      const int    samples = std::max(2, static_cast<int>(std::ceil(seg_len / safe_spacing)));

      for (int s = 1; s <= samples; ++s) {
        const double t  = static_cast<double>(s) / samples;
        const double t2 = t * t, t3 = t2 * t;
        out.emplace_back(
          0.5 * (2*x1 + (-x0+x2)*t + (2*x0-5*x1+4*x2-x3)*t2 + (-x0+3*x1-3*x2+x3)*t3),
          0.5 * (2*y1 + (-y0+y2)*t + (2*y0-5*y1+4*y2-y3)*t2 + (-y0+3*y1-3*y2+y3)*t3));
      }
    }
    return out;
  }

  bool world_path_is_free(const std::vector<WorldPoint> & path) const
  {
    if (path.empty())       { return false; }
    if (path.size() == 1)   { return true; }
    const std::size_t seg_count = path.size() - 1;
    for (std::size_t i = 0; i < seg_count; ++i) {
      if (segment_validation_can_be_skipped(i, seg_count)) { continue; }
      const auto [x0, y0] = path[i];
      const auto [x1, y1] = path[i + 1];
      if (!line_is_free(world_to_grid(x0, y0), world_to_grid(x1, y1))) { return false; }
    }
    return true;
  }

  std::vector<WorldPoint> resample_path(
    const std::vector<WorldPoint> & path, double spacing) const
  {
    if (path.size() <= 1 || spacing <= 0.0) { return path; }
    std::vector<WorldPoint> out{path.front()};
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
      const auto [x0, y0] = path[i];
      const auto [x1, y1] = path[i + 1];
      const double dx = x1 - x0, dy = y1 - y0;
      const double dist = std::hypot(dx, dy);
      if (dist < 1e-6) { continue; }
      const int steps = static_cast<int>(std::floor(dist / spacing));
      for (int s = 1; s <= steps; ++s) {
        const double t = (static_cast<double>(s) * spacing) / dist;
        if (t >= 1.0) { continue; }
        out.emplace_back(x0 + t * dx, y0 + t * dy);
      }
      out.emplace_back(x1, y1);
    }
    return out;
  }

  std::vector<WorldPoint> grid_path_to_world(
    const std::vector<GridCell> & path_grid,
    const WorldPoint * start_world,
    const WorldPoint * goal_world) const
  {
    std::vector<WorldPoint> out;
    out.reserve(path_grid.size());
    for (const auto & [gx, gy] : path_grid) {
      out.push_back(grid_to_world(gx, gy));
    }
    if (!out.empty()) {
      if (start_world) { out.front() = *start_world; }
      if (goal_world)  { out.back()  = *goal_world;  }
    }
    return out;
  }

  std::tuple<bool, std::string, nav_msgs::msg::Path> compute_path_to_goal(
    const geometry_msgs::msg::PoseStamped & goal_msg)
  {
    if (!has_map_) {
      return {false, "No map received yet.", nav_msgs::msg::Path()};
    }
    if (free_cells_.empty()) {
      return {false, "Map has no free cells.", nav_msgs::msg::Path()};
    }

    double rx = 0.0, ry = 0.0, ryaw = 0.0;
    if (!get_robot_pose(rx, ry, ryaw)) {
      return {false, "No robot TF pose available.", nav_msgs::msg::Path()};
    }

    if (!goal_msg.header.frame_id.empty() && goal_msg.header.frame_id != global_frame_) {
      return {false,
        "Goal frame \"" + goal_msg.header.frame_id +
        "\" != planner frame \"" + global_frame_ + "\".",
        nav_msgs::msg::Path()};
    }

    const double gx = goal_msg.pose.position.x;
    const double gy = goal_msg.pose.position.y;

    const GridCell start_raw = world_to_grid(rx, ry);
    const GridCell goal_raw  = world_to_grid(gx, gy);

    const GridCell start = snap_to_free(start_raw, start_free_search_radius_cells_);
    const GridCell goal  = snap_to_free(goal_raw,  start_free_search_radius_cells_);

    if (!is_cell_free(start)) {
      return {false, "Start is in obstacle even after snap.", nav_msgs::msg::Path()};
    }
    if (!is_cell_free(goal)) {
      return {false, "Goal is in obstacle even after snap.", nav_msgs::msg::Path()};
    }

    if (start != start_raw) {
      RCLCPP_WARN(get_logger(), "Start (%d,%d) in obstacle, snapped to (%d,%d).",
        start_raw.first, start_raw.second, start.first, start.second);
    }
    if (goal != goal_raw) {
      RCLCPP_WARN(get_logger(), "Goal (%d,%d) in obstacle, snapped to (%d,%d).",
        goal_raw.first, goal_raw.second, goal.first, goal.second);
    }

    auto path_grid = plan_astar(start, goal);
    if (path_grid.empty()) {
      return {false, "A* could not find a path.", nav_msgs::msg::Path()};
    }

    const std::size_t raw_points = path_grid.size();

    // Grid-level greedy shortcut
    if (smooth_path_) {
      path_grid = smooth_path_grid(path_grid);
    }

    const WorldPoint start_world{rx, ry};
    const WorldPoint goal_world{gx, gy};

    auto path_world = grid_path_to_world(path_grid, &start_world, &goal_world);

    // Catmull-Rom smoothing (multiple passes for rounder curves)
    if (smooth_path_) {
      for (int pass = 0; pass < smooth_iterations_; ++pass) {
        auto candidate = catmull_rom_smooth(path_world, waypoint_spacing_);
        if (world_path_is_free(candidate)) {
          path_world = std::move(candidate);
        } else {
          RCLCPP_WARN(get_logger(),
            "Catmull-Rom pass %d rejected (intersects obstacle). Using previous.", pass + 1);
          break;
        }
      }
    }

    path_world = resample_path(path_world, waypoint_spacing_);

    if (!world_path_is_free(path_world)) {
      return {false, "Final path validation failed.", nav_msgs::msg::Path()};
    }

    auto path_msg = build_path_msg(path_world);

    // Override orientation of last pose with goal orientation
    if (!path_msg.poses.empty()) {
      const double goal_yaw = quaternion_to_yaw(
        goal_msg.pose.orientation.x, goal_msg.pose.orientation.y,
        goal_msg.pose.orientation.z, goal_msg.pose.orientation.w);
      const auto [qz, qw] = yaw_to_quaternion(goal_yaw);
      path_msg.poses.back().pose.orientation = geometry_msgs::msg::Quaternion();
      path_msg.poses.back().pose.orientation.z = qz;
      path_msg.poses.back().pose.orientation.w = qw;
    }

    path_pub_->publish(path_msg);

    const std::string msg =
      "Path found: " + std::to_string(path_world.size()) +
      " pts (raw A*: " + std::to_string(raw_points) +
      ", spacing: " + std::to_string(waypoint_spacing_) + " m)";
    return {true, msg, path_msg};
  }

  void plan_path_service_callback(
    const std::shared_ptr<PlanPath::Request>  request,
    std::shared_ptr<PlanPath::Response>       response)
  {
    const auto [success, message, path] = compute_path_to_goal(request->goal);
    response->success = success;
    response->message = message;
    response->path    = path;
    if (success) { RCLCPP_INFO(get_logger(), "%s", message.c_str()); }
    else         { RCLCPP_WARN(get_logger(), "%s", message.c_str()); }
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const auto [success, message, path] = compute_path_to_goal(*msg);
    (void)path;
    if (success) { RCLCPP_INFO(get_logger(), "%s", message.c_str()); }
    else         { RCLCPP_WARN(get_logger(), "%s", message.c_str()); }
  }

  nav_msgs::msg::Path build_path_msg(const std::vector<WorldPoint> & path_world) const
  {
    nav_msgs::msg::Path msg;
    msg.header.frame_id = global_frame_;
    msg.header.stamp    = this->now();

    for (std::size_t i = 0; i < path_world.size(); ++i) {
      const auto [x, y] = path_world[i];
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = 0.0;

      double yaw = 0.0;
      if (i + 1 < path_world.size()) {
        const auto [nx, ny] = path_world[i + 1];
        yaw = std::atan2(ny - y, nx - x);
      } else if (path_world.size() > 1) {
        const auto [px, py] = path_world[i - 1];
        yaw = std::atan2(y - py, x - px);
      }
      const auto [qz, qw]              = yaw_to_quaternion(yaw);
      pose.pose.orientation.z          = qz;
      pose.pose.orientation.w          = qw;
      msg.poses.push_back(pose);
    }
    return msg;
  }

  std::string global_frame_;
  std::string robot_frame_;
  std::string dynamic_map_topic_;
  double robot_radius_                 {0.0};
  int    start_free_search_radius_cells_{20};
  int    endpoint_ignore_waypoints_    {3};
  bool   smooth_path_                  {true};
  double waypoint_spacing_             {0.05};
  double tf_timeout_                   {0.05};
  double heuristic_weight_             {1.2};
  double obstacle_proximity_cost_      {8.0};
  int    soft_clearance_cells_         {3};
  int    smooth_iterations_            {2};

  cv::Mat grid_raw_;
  cv::Mat grid_;
  cv::Mat cost_grid_;
  double  resolution_  {0.05};
  std::vector<double>   origin_{0.0, 0.0, 0.0};
  std::vector<GridCell> free_cells_;
  bool has_map_{false};

  std::shared_ptr<tf2_ros::Buffer>           tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr          path_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr dynamic_map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Service<PlanPath>::SharedPtr                       plan_path_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<PathPlannerNode>();
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    std::cerr << "Fatal error: " << ex.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}