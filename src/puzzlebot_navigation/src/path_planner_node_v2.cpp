#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include "nav_msgs/msg/odometry.hpp"

#include "puzzlebot_interfaces/srv/plan_path.hpp"

class PathPlannerNode : public rclcpp::Node
{
public:
  using GridCell = std::pair<int, int>;
  using WorldPoint = std::pair<double, double>;
  using PlanPath = puzzlebot_interfaces::srv::PlanPath;

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
    declare_parameter<std::string>("global_frame", "map");
    declare_parameter<std::string>("robot_pose_topic", "/odom");
    declare_parameter<std::string>("dynamic_map_topic", "/map");

    declare_parameter<double>("robot_radius", 0.20);
    declare_parameter<int>("start_free_search_radius_cells", 20);
    declare_parameter<bool>("smooth_path", true);
    declare_parameter<double>("waypoint_spacing", 0.05);

    declare_parameter<int>("max_iter", 5000);
    declare_parameter<int>("step_size", 2);
    declare_parameter<int>("connect_step_size", 2);
    declare_parameter<int>("max_connect_steps", 1);
    declare_parameter<double>("goal_bias_rate", 0.02);
    declare_parameter<int>("connect_threshold", 3);
    declare_parameter<int>("random_seed", 0);

    declare_parameter<double>("alpha", 1.0);
    declare_parameter<double>("beta", 0.10);
    declare_parameter<double>("gamma", 0.20);

    auto qos = rclcpp::QoS(10).reliable().durability_volatile();


    global_frame_ = get_parameter("global_frame").as_string();
    robot_pose_topic_ = get_parameter("robot_pose_topic").as_string();
    dynamic_map_topic_ = get_parameter("dynamic_map_topic").as_string();

    robot_radius_ = get_parameter("robot_radius").as_double();
    start_free_search_radius_cells_ =
      static_cast<int>(get_parameter("start_free_search_radius_cells").as_int());
    smooth_path_ = get_parameter("smooth_path").as_bool();
    waypoint_spacing_ = get_parameter("waypoint_spacing").as_double();

    max_iter_ = static_cast<int>(get_parameter("max_iter").as_int());
    step_size_ = static_cast<int>(get_parameter("step_size").as_int());
    connect_step_size_ = static_cast<int>(get_parameter("connect_step_size").as_int());
    max_connect_steps_ = static_cast<int>(get_parameter("max_connect_steps").as_int());
    goal_bias_rate_ = get_parameter("goal_bias_rate").as_double();
    connect_threshold_ = static_cast<int>(get_parameter("connect_threshold").as_int());
    random_seed_ = static_cast<int>(get_parameter("random_seed").as_int());

    alpha_ = get_parameter("alpha").as_double();
    beta_ = get_parameter("beta").as_double();
    gamma_ = get_parameter("gamma").as_double();


    if (random_seed_ != 0) {
      rng_.seed(static_cast<std::mt19937::result_type>(random_seed_));
    }

    path_pub_ = create_publisher<nav_msgs::msg::Path>("/path", 10);


  dynamic_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      dynamic_map_topic_,
      rclcpp::QoS(1).reliable().transient_local(),  // match map_server
      std::bind(&PathPlannerNode::dynamic_map_callback, this, std::placeholders::_1));

    robot_pose_stamped_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      robot_pose_topic_,
      qos,
      std::bind(&PathPlannerNode::robot_pose_stamped_callback, this, std::placeholders::_1));

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
      "Path planner ready | dynamic_map_topic=%s | service=/plan_path | robot_pose_topic=%s",
      dynamic_map_topic_.c_str(),
      robot_pose_topic_.c_str());
  }

private:
  void dynamic_map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (msg->info.width == 0 || msg->info.height == 0 || msg->data.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty dynamic map.");
      return;
    }

    global_frame_ = msg->header.frame_id.empty() ? global_frame_ : msg->header.frame_id;

    resolution_ = static_cast<double>(msg->info.resolution);
    origin_ = {
      msg->info.origin.position.x,
      msg->info.origin.position.y,
      quaternion_to_yaw(
        msg->info.origin.orientation.x,
        msg->info.origin.orientation.y,
        msg->info.origin.orientation.z,
        msg->info.origin.orientation.w)
    };

    const int width = static_cast<int>(msg->info.width);
    const int height = static_cast<int>(msg->info.height);

    grid_ = cv::Mat::zeros(height, width, CV_8UC1);

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const int k = y * width + x;

        if (k < 0 || k >= static_cast<int>(msg->data.size())) {
          grid_.at<uint8_t>(y, x) = 1;
          continue;
        }

        const int8_t value = msg->data[static_cast<std::size_t>(k)];

        if (value == 0) {
          grid_.at<uint8_t>(y, x) = 0;
        } else {
          grid_.at<uint8_t>(y, x) = 1;
        }
      }
    }

    grid_raw_ = grid_.clone();

    inflate_obstacles(robot_radius_);
    free_cells_ = precompute_free_cells();

    has_map_ = !free_cells_.empty();

    if (!has_map_) {
      RCLCPP_WARN(get_logger(), "Dynamic map received, but it has no free cells.");
      return;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      3000,
      "Dynamic map updated | size=(%d,%d) | resolution=%.4f | free_cells=%zu",
      grid_.cols,
      grid_.rows,
      resolution_,
      free_cells_.size());
  }

  void inflate_obstacles(double robot_radius)
  {
    const int inflation_cells = static_cast<int>(std::ceil(robot_radius / resolution_));

    if (inflation_cells <= 0) {
      return;
    }

    const int kernel_size = 2 * inflation_cells + 1;
    const cv::Mat kernel = cv::Mat::ones(kernel_size, kernel_size, CV_8UC1);

    cv::dilate(grid_, grid_, kernel, cv::Point(-1, -1), 1);
  }

  std::vector<GridCell> precompute_free_cells() const
  {
    std::vector<GridCell> cells;

    if (grid_.empty()) {
      return cells;
    }

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
    const int gx = static_cast<int>(std::floor((x - origin_[0]) / resolution_));
    const int gy = static_cast<int>(std::floor((y - origin_[1]) / resolution_));
    return {gx, gy};
  }

  WorldPoint grid_to_world(int gx, int gy) const
  {
    const double x = origin_[0] + (static_cast<double>(gx) + 0.5) * resolution_;
    const double y = origin_[1] + (static_cast<double>(gy) + 0.5) * resolution_;
    return {x, y};
  }

  bool is_cell_inside(const GridCell & cell) const
  {
    if (grid_.empty()) {
      return false;
    }

    return cell.first >= 0 &&
           cell.first < grid_.cols &&
           cell.second >= 0 &&
           cell.second < grid_.rows;
  }

  bool is_cell_free(const GridCell & cell) const
  {
    if (!is_cell_inside(cell)) {
      return false;
    }

    return grid_.at<uint8_t>(cell.second, cell.first) == 0;
  }


  bool find_nearest_free_cell(
    const GridCell & start_cell,
    GridCell & nearest_free_cell,
    int max_search_radius_cells = 20) const
  {
    if (!is_cell_inside(start_cell)) {
      return false;
    }

    if (is_cell_free(start_cell)) {
      nearest_free_cell = start_cell;
      return true;
    }

    const int max_radius = std::max(1, max_search_radius_cells);

    for (int radius = 1; radius <= max_radius; ++radius) {
      GridCell best_cell = start_cell;
      double best_distance = std::numeric_limits<double>::infinity();
      bool found = false;

      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
          if (std::max(std::abs(dx), std::abs(dy)) != radius) {
            continue;
          }

          const GridCell candidate{start_cell.first + dx, start_cell.second + dy};

          if (!is_cell_free(candidate)) {
            continue;
          }

          const double d = distance(start_cell, candidate);

          if (d < best_distance) {
            best_distance = d;
            best_cell = candidate;
            found = true;
          }
        }
      }

      if (found) {
        nearest_free_cell = best_cell;
        return true;
      }
    }

    return false;
  }

  void robot_pose_stamped_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_robot_pose_ = *msg;
    has_robot_pose_ = true;
  }

  bool get_robot_pose(double & x, double & y, double & yaw)
  {
    if (!has_robot_pose_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "No robot pose received yet on topic %s.",
        robot_pose_topic_.c_str());
      return false;
    }

    if (!latest_robot_pose_.header.frame_id.empty() &&
        latest_robot_pose_.header.frame_id != global_frame_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Robot pose frame is '%s', but planner frame is '%s'. Pose must already be in map frame.",
        latest_robot_pose_.header.frame_id.c_str(),
        global_frame_.c_str());
    }

    x = latest_robot_pose_.pose.pose.position.x;
    y = latest_robot_pose_.pose.pose.position.y;

    const auto & q = latest_robot_pose_.pose.pose.orientation;
    yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w);

    return true;
  }

  static double quaternion_to_yaw(double x, double y, double z, double w)
  {
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static std::pair<double, double> yaw_to_quaternion(double yaw)
  {
    return {std::sin(yaw / 2.0), std::cos(yaw / 2.0)};
  }

  static double distance(const GridCell & a, const GridCell & b)
  {
    return std::hypot(
      static_cast<double>(a.first - b.first),
      static_cast<double>(a.second - b.second));
  }

  GridCell sample_free(const GridCell & target)
  {
    std::uniform_real_distribution<double> prob(0.0, 1.0);

    if (prob(rng_) < goal_bias_rate_) {
      return target;
    }

    std::uniform_int_distribution<std::size_t> index_dist(0, free_cells_.size() - 1);
    return free_cells_[index_dist(rng_)];
  }

  GridCell heuristic_best_node(
    const std::vector<GridCell> & tree_nodes,
    const std::unordered_map<GridCell, double, PairHash> & g_cost,
    const GridCell & q_sample,
    const GridCell & heuristic_target) const
  {
    GridCell best_node = tree_nodes.front();
    double best_score = std::numeric_limits<double>::infinity();

    for (const auto & q : tree_nodes) {
      const double score =
        alpha_ * distance(q, q_sample) +
        beta_ * g_cost.at(q) +
        gamma_ * distance(q, heuristic_target);

      if (score < best_score) {
        best_score = score;
        best_node = q;
      }
    }

    return best_node;
  }

  GridCell steer(const GridCell & q_from, const GridCell & q_to, int step_size) const
  {
    const int dx = q_to.first - q_from.first;
    const int dy = q_to.second - q_from.second;

    const double dist =
      std::hypot(static_cast<double>(dx), static_cast<double>(dy));

    if (dist < 1e-9) {
      return q_from;
    }

    if (dist <= static_cast<double>(step_size)) {
      return q_to;
    }

    const double scale = static_cast<double>(step_size) / dist;

    return {
      static_cast<int>(std::round(q_from.first + dx * scale)),
      static_cast<int>(std::round(q_from.second + dy * scale))
    };
  }

  bool line_is_free(const GridCell & a, const GridCell & b) const
  {
    int x0 = a.first;
    int y0 = a.second;

    const int x1 = b.first;
    const int y1 = b.second;

    const int dx = std::abs(x1 - x0);
    const int dy = std::abs(y1 - y0);

    const int sx = (x0 < x1) ? 1 : -1;
    const int sy = (y0 < y1) ? 1 : -1;

    int err = dx - dy;

    while (true) {
      if (!is_cell_free({x0, y0})) {
        return false;
      }

      if (x0 == x1 && y0 == y1) {
        break;
      }

      const int e2 = 2 * err;

      if (e2 > -dy) {
        err -= dy;
        x0 += sx;
      }

      if (e2 < dx) {
        err += dx;
        y0 += sy;
      }
    }

    return true;
  }

  bool extend_heuristic(
    std::vector<GridCell> & tree_nodes,
    std::unordered_map<GridCell, GridCell, PairHash> & parent,
    std::unordered_map<GridCell, bool, PairHash> & has_parent,
    std::unordered_map<GridCell, double, PairHash> & g_cost,
    const GridCell & q_sample,
    const GridCell & heuristic_target,
    int step_size,
    GridCell & q_new,
    GridCell & q_near)
  {
    q_near = heuristic_best_node(tree_nodes, g_cost, q_sample, heuristic_target);
    q_new = steer(q_near, q_sample, step_size);

    if (q_new == q_near || !is_cell_free(q_new) || !line_is_free(q_near, q_new)) {
      return false;
    }

    const auto old_cost_it = g_cost.find(q_new);
    const double new_cost = g_cost.at(q_near) + distance(q_near, q_new);

    if (old_cost_it != g_cost.end() && new_cost >= old_cost_it->second) {
      return false;
    }

    if (old_cost_it == g_cost.end()) {
      tree_nodes.push_back(q_new);
    }

    parent[q_new] = q_near;
    has_parent[q_new] = true;
    g_cost[q_new] = new_cost;

    return true;
  }

  bool connect_heuristic(
    std::vector<GridCell> & tree_nodes,
    std::unordered_map<GridCell, GridCell, PairHash> & parent,
    std::unordered_map<GridCell, bool, PairHash> & has_parent,
    std::unordered_map<GridCell, double, PairHash> & g_cost,
    const GridCell & q_target,
    int max_connect_steps,
    GridCell & meeting_node,
    GridCell & last_near)
  {
    GridCell last_new = q_target;
    last_near = q_target;

    for (int i = 0; i < max_connect_steps; ++i) {
      GridCell q_new;
      GridCell q_near;

      const bool extended =
        extend_heuristic(
          tree_nodes,
          parent,
          has_parent,
          g_cost,
          q_target,
          q_target,
          connect_step_size_,
          q_new,
          q_near);

      if (!extended) {
        return false;
      }

      last_new = q_new;
      last_near = q_near;

      if (distance(q_new, q_target) <= static_cast<double>(connect_threshold_) &&
          line_is_free(q_new, q_target))
      {
        if (g_cost.find(q_target) == g_cost.end()) {
          tree_nodes.push_back(q_target);
          parent[q_target] = q_new;
          has_parent[q_target] = true;
          g_cost[q_target] = g_cost.at(q_new) + distance(q_new, q_target);
        }

        meeting_node = q_target;
        return true;
      }
    }

    (void)last_new;
    return false;
  }

  std::vector<GridCell> trace_to_root(
    const std::unordered_map<GridCell, GridCell, PairHash> & parent,
    const std::unordered_map<GridCell, bool, PairHash> & has_parent,
    const GridCell & node) const
  {
    std::vector<GridCell> path{node};

    while (true) {
      const auto hp = has_parent.find(path.back());

      if (hp == has_parent.end() || !hp->second) {
        break;
      }

      path.push_back(parent.at(path.back()));
    }

    return path;
  }

  std::vector<GridCell> build_bidirectional_path(
    const std::unordered_map<GridCell, GridCell, PairHash> & parent_start,
    const std::unordered_map<GridCell, bool, PairHash> & has_parent_start,
    const std::unordered_map<GridCell, GridCell, PairHash> & parent_goal,
    const std::unordered_map<GridCell, bool, PairHash> & has_parent_goal,
    const GridCell & meeting_node) const
  {
    auto start_to_meet = trace_to_root(parent_start, has_parent_start, meeting_node);
    std::reverse(start_to_meet.begin(), start_to_meet.end());

    const auto meet_to_goal = trace_to_root(parent_goal, has_parent_goal, meeting_node);

    std::vector<GridCell> full_path = start_to_meet;
    full_path.insert(full_path.end(), meet_to_goal.begin() + 1, meet_to_goal.end());

    return full_path;
  }

  bool plan_connect(const GridCell & start, const GridCell & goal, std::vector<GridCell> & path)
  {
    std::vector<GridCell> tree_a{start};
    std::unordered_map<GridCell, GridCell, PairHash> parent_a;
    std::unordered_map<GridCell, bool, PairHash> has_parent_a;
    std::unordered_map<GridCell, double, PairHash> g_a;

    parent_a[start] = start;
    has_parent_a[start] = false;
    g_a[start] = 0.0;

    GridCell target_a = goal;

    std::vector<GridCell> tree_b{goal};
    std::unordered_map<GridCell, GridCell, PairHash> parent_b;
    std::unordered_map<GridCell, bool, PairHash> has_parent_b;
    std::unordered_map<GridCell, double, PairHash> g_b;

    parent_b[goal] = goal;
    has_parent_b[goal] = false;
    g_b[goal] = 0.0;

    GridCell target_b = start;

    bool a_is_start_tree = true;

    for (int iteration = 0; iteration < max_iter_; ++iteration) {
      const GridCell q_sample = sample_free(target_a);

      GridCell q_new;
      GridCell q_near;

      const bool extended =
        extend_heuristic(
          tree_a,
          parent_a,
          has_parent_a,
          g_a,
          q_sample,
          target_a,
          step_size_,
          q_new,
          q_near);


      if (extended) {

        GridCell meeting_node;
        GridCell connect_near;

        const bool connected =
          connect_heuristic(
            tree_b,
            parent_b,
            has_parent_b,
            g_b,
            q_new,
            max_connect_steps_,
            meeting_node,
            connect_near);

        if (connected) {
          if (a_is_start_tree) {
            path = build_bidirectional_path(parent_a, has_parent_a, parent_b, has_parent_b, meeting_node);
          } else {
            path = build_bidirectional_path(parent_b, has_parent_b, parent_a, has_parent_a, meeting_node);
          }

          return true;
        }
      } else {
      }

      std::swap(tree_a, tree_b);
      std::swap(parent_a, parent_b);
      std::swap(has_parent_a, has_parent_b);
      std::swap(g_a, g_b);
      std::swap(target_a, target_b);

      a_is_start_tree = !a_is_start_tree;
    }

    return false;
  }

  std::vector<GridCell> smooth_path_grid(const std::vector<GridCell> & path_grid) const
  {
    if (path_grid.size() <= 2) {
      return path_grid;
    }

    std::vector<GridCell> smooth_path{path_grid.front()};

    std::size_t i = 0;

    while (i < path_grid.size() - 1) {
      std::size_t j = path_grid.size() - 1;

      while (j > i + 1) {
        if (line_is_free(path_grid[i], path_grid[j])) {
          break;
        }

        --j;
      }

      smooth_path.push_back(path_grid[j]);
      i = j;
    }

    return smooth_path;
  }

  std::vector<WorldPoint> catmull_rom_smooth_path_world(
    const std::vector<WorldPoint> & path_world,
    double spacing) const
  {
    if (path_world.size() <= 2) {
      return path_world;
    }

    const double safe_spacing = std::max(0.01, spacing);
    std::vector<WorldPoint> smooth_path;
    smooth_path.reserve(path_world.size() * 4);
    smooth_path.push_back(path_world.front());

    for (std::size_t i = 0; i + 1 < path_world.size(); ++i) {
      const auto [x0, y0] = (i == 0) ? path_world[i] : path_world[i - 1];
      const auto [x1, y1] = path_world[i];
      const auto [x2, y2] = path_world[i + 1];
      const auto [x3, y3] = (i + 2 < path_world.size()) ? path_world[i + 2] : path_world[i + 1];

      const double segment_length = std::hypot(x2 - x1, y2 - y1);
      const int samples = std::max(2, static_cast<int>(std::ceil(segment_length / safe_spacing)));

      for (int s = 1; s <= samples; ++s) {
        const double t = static_cast<double>(s) / static_cast<double>(samples);
        const double t2 = t * t;
        const double t3 = t2 * t;

        const double x = 0.5 * (
          2.0 * x1 +
          (-x0 + x2) * t +
          (2.0 * x0 - 5.0 * x1 + 4.0 * x2 - x3) * t2 +
          (-x0 + 3.0 * x1 - 3.0 * x2 + x3) * t3);

        const double y = 0.5 * (
          2.0 * y1 +
          (-y0 + y2) * t +
          (2.0 * y0 - 5.0 * y1 + 4.0 * y2 - y3) * t2 +
          (-y0 + 3.0 * y1 - 3.0 * y2 + y3) * t3);

        smooth_path.emplace_back(x, y);
      }
    }

    return smooth_path;
  }

  bool world_path_is_free(const std::vector<WorldPoint> & path_world) const
  {
    if (path_world.empty()) {
      return false;
    }

    for (std::size_t i = 0; i < path_world.size(); ++i) {
      const auto [x, y] = path_world[i];
      const GridCell cell = world_to_grid(x, y);

      if (!is_cell_free(cell)) {
        return false;
      }

      if (i > 0) {
        const auto [px, py] = path_world[i - 1];
        const GridCell prev_cell = world_to_grid(px, py);

        if (!line_is_free(prev_cell, cell)) {
          return false;
        }
      }
    }

    return true;
  }

  std::vector<WorldPoint> resample_path_world(
    const std::vector<WorldPoint> & path_world,
    double spacing) const
  {
    if (path_world.size() <= 1 || spacing <= 0.0) {
      return path_world;
    }

    std::vector<WorldPoint> resampled{path_world.front()};

    for (std::size_t i = 0; i + 1 < path_world.size(); ++i) {
      const auto [x0, y0] = path_world[i];
      const auto [x1, y1] = path_world[i + 1];

      const double dx = x1 - x0;
      const double dy = y1 - y0;
      const double dist = std::hypot(dx, dy);

      if (dist < 1e-6) {
        continue;
      }

      const int steps = static_cast<int>(std::floor(dist / spacing));

      for (int s = 1; s <= steps; ++s) {
        const double t = (static_cast<double>(s) * spacing) / dist;

        if (t >= 1.0) {
          continue;
        }

        resampled.emplace_back(x0 + t * dx, y0 + t * dy);
      }

      resampled.emplace_back(x1, y1);
    }

    return resampled;
  }

  std::vector<WorldPoint> grid_path_to_world_path(
    const std::vector<GridCell> & path_grid,
    const WorldPoint * start_world,
    const WorldPoint * goal_world,
    bool resample) const
  {
    std::vector<WorldPoint> path_world;
    path_world.reserve(path_grid.size());

    for (const auto & [gx, gy] : path_grid) {
      path_world.push_back(grid_to_world(gx, gy));
    }

    if (!path_world.empty()) {
      if (start_world != nullptr) {
        path_world.front() = *start_world;
      }

      if (goal_world != nullptr) {
        path_world.back() = *goal_world;
      }
    }

    if (resample) {
      return resample_path_world(path_world, waypoint_spacing_);
    }

    return path_world;
  }

  std::tuple<bool, std::string, nav_msgs::msg::Path> compute_path_to_goal(
    const geometry_msgs::msg::PoseStamped & goal_msg)
  {
    if (!has_map_) {
      return {false, "No dynamic map received yet. Cannot plan.", nav_msgs::msg::Path()};
    }

    if (free_cells_.empty()) {
      return {false, "Dynamic map has no free cells available.", nav_msgs::msg::Path()};
    }

    double robot_x = 0.0;
    double robot_y = 0.0;
    double robot_yaw = 0.0;

    if (!get_robot_pose(robot_x, robot_y, robot_yaw)) {
      return {false, "No robot pose available. Cannot plan.", nav_msgs::msg::Path()};
    }

    if (!goal_msg.header.frame_id.empty() && goal_msg.header.frame_id != global_frame_) {
      return {
        false,
        "Goal frame is \"" + goal_msg.header.frame_id +
        "\", but planner frame is \"" + global_frame_ + "\".",
        nav_msgs::msg::Path()};
    }

    const double goal_x = goal_msg.pose.position.x;
    const double goal_y = goal_msg.pose.position.y;

    const GridCell raw_start_grid = world_to_grid(robot_x, robot_y);
    const GridCell goal_grid = world_to_grid(goal_x, goal_y);

    if (!is_cell_inside(raw_start_grid)) {
      return {false, "Start cell is outside dynamic map.", nav_msgs::msg::Path()};
    }

    if (!is_cell_free(goal_grid)) {
      return {false, "Goal cell is occupied or outside dynamic map.", nav_msgs::msg::Path()};
    }

    GridCell start_grid = raw_start_grid;

    if (!is_cell_free(start_grid)) {
      GridCell nearest_start;

      if (!find_nearest_free_cell(
            start_grid, nearest_start, start_free_search_radius_cells_))
      {
        return {
          false,
          "Start cell is occupied after obstacle inflation and no nearby free cell was found.",
          nav_msgs::msg::Path()};
      }

      RCLCPP_WARN(
        get_logger(),
        "Start cell (%d,%d) is occupied after inflation. Using nearest free cell (%d,%d) instead.",
        start_grid.first,
        start_grid.second,
        nearest_start.first,
        nearest_start.second);

      start_grid = nearest_start;
    }

    std::vector<GridCell> path_grid;

    if (!plan_connect(start_grid, goal_grid, path_grid)) {
      return {false, "Path planner could not find a path.", nav_msgs::msg::Path()};
    }

    const auto original_points = path_grid.size();
    const auto original_path_grid = path_grid;

    const WorldPoint start_world{robot_x, robot_y};
    const WorldPoint goal_world{goal_x, goal_y};

    if (smooth_path_) {
      path_grid = smooth_path_grid(path_grid);
    }

    const auto smoothed_points = path_grid.size();

    auto path_world =
      grid_path_to_world_path(path_grid, &start_world, &goal_world, false);

    if (smooth_path_) {
      const auto catmull_path =
        catmull_rom_smooth_path_world(path_world, waypoint_spacing_);

      if (world_path_is_free(catmull_path)) {
        path_world = catmull_path;
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Catmull-Rom smoothing was rejected because it intersects an occupied cell. Using shortcut-smoothed path.");
      }
    }

    path_world = resample_path_world(path_world, waypoint_spacing_);

    auto path_msg = build_path_msg(path_world);



    if (!path_msg.poses.empty()) {
        const double goal_yaw = quaternion_to_yaw(
            goal_msg.pose.orientation.x,
            goal_msg.pose.orientation.y,
            goal_msg.pose.orientation.z,
            goal_msg.pose.orientation.w);
        const auto [qz, qw] = yaw_to_quaternion(goal_yaw);
        path_msg.poses.back().pose.orientation.x = 0.0;
        path_msg.poses.back().pose.orientation.y = 0.0;
        path_msg.poses.back().pose.orientation.z = qz;
        path_msg.poses.back().pose.orientation.w = qw;
    }

    path_pub_->publish(path_msg);


    const std::string message =
      "Path found with " + std::to_string(path_world.size()) +
      " points (tree points: " + std::to_string(original_points) +
      ", smoothed: " + std::to_string(smoothed_points) +
      ", spacing: " + std::to_string(waypoint_spacing_) + " m).";

    return {true, message, path_msg};
  }

  void plan_path_service_callback(
    const std::shared_ptr<PlanPath::Request> request,
    std::shared_ptr<PlanPath::Response> response)
  {
    const auto [success, message, path] = compute_path_to_goal(request->goal);

    response->success = success;
    response->message = message;
    response->path = path;

    if (success) {
      RCLCPP_INFO(get_logger(), "%s", message.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s", message.c_str());
    }
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const auto [success, message, path] = compute_path_to_goal(*msg);
    (void)path;
    if (success) {
      RCLCPP_INFO(get_logger(), "%s", message.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s", message.c_str());
    }
  }

  nav_msgs::msg::Path build_path_msg(const std::vector<WorldPoint> & path_world) const
  {
    nav_msgs::msg::Path path_msg;

    path_msg.header.frame_id = global_frame_;
    path_msg.header.stamp = this->now();

    for (std::size_t i = 0; i < path_world.size(); ++i) {
      const auto [x, y] = path_world[i];

      geometry_msgs::msg::PoseStamped pose;

      pose.header.frame_id = global_frame_;
      pose.header.stamp = path_msg.header.stamp;
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

      const auto [qz, qw] = yaw_to_quaternion(yaw);

      pose.pose.orientation.z = qz;
      pose.pose.orientation.w = qw;

      path_msg.poses.push_back(pose);
    }

    return path_msg;
  }

  std::string global_frame_;
  std::string robot_pose_topic_;
  std::string dynamic_map_topic_;

  double robot_radius_ = 0.0;
  int start_free_search_radius_cells_ = 20;
  bool smooth_path_ = true;
  double waypoint_spacing_ = 0.05;

  int max_iter_ = 5000;
  int step_size_ = 2;
  int connect_step_size_ = 2;
  int max_connect_steps_ = 1;
  double goal_bias_rate_ = 0.02;
  int connect_threshold_ = 3;
  int random_seed_ = 0;

  double alpha_ = 1.0;
  double beta_ = 0.10;
  double gamma_ = 0.20;


  cv::Mat grid_raw_;
  cv::Mat grid_;

  double resolution_ = 0.05;
  std::vector<double> origin_{0.0, 0.0, 0.0};

  std::vector<GridCell> free_cells_;

  std::mt19937 rng_;

  bool has_map_ = false;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr dynamic_map_sub_;
  //rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_stamped_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_pose_stamped_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Service<PlanPath>::SharedPtr plan_path_srv_;

  nav_msgs::msg::Odometry latest_robot_pose_;
  bool has_robot_pose_ = false;
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