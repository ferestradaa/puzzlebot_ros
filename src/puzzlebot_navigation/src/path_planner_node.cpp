#include <algorithm>
#include <cmath>
#include <chrono>
#include <filesystem>
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
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


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

  struct DebugFrame
  {
    std::unordered_map<GridCell, GridCell, PairHash> parent_start;
    std::unordered_map<GridCell, bool, PairHash> has_parent_start;

    std::unordered_map<GridCell, GridCell, PairHash> parent_goal;
    std::unordered_map<GridCell, bool, PairHash> has_parent_goal;

  };

  explicit PathPlannerNode()
  : Node("path_planner_node"),
    rng_(std::random_device{}())
  {
    declare_parameter<std::string>("map_yaml", "map.yaml");
    // true = use the exact row order written by this project's SLAMNode::save_map_to_pgm_yaml().
    // false = use standard image convention where image row 0 is the top of the map.
    declare_parameter<bool>("slam_raw_pgm_order", true);
    declare_parameter<std::string>("global_frame", "map");
    declare_parameter<std::string>("robot_pose_topic", "/robot_pose_map");
    declare_parameter<double>("robot_radius", 0.0);
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

    declare_parameter<bool>("debug_visualize_path_planner", true);
    declare_parameter<int>("debug_path_planner_visualize_every", 1);
    declare_parameter<double>("debug_path_planner_visualize_delay", 0.02);
    declare_parameter<bool>("debug_publish_unsmoothed_path", true);

    map_yaml_name_ = get_parameter("map_yaml").as_string();
    slam_raw_pgm_order_ = get_parameter("slam_raw_pgm_order").as_bool();
    global_frame_ = get_parameter("global_frame").as_string();
    robot_pose_topic_ = get_parameter("robot_pose_topic").as_string();
    robot_radius_ = get_parameter("robot_radius").as_double();
    smooth_path_ = get_parameter("smooth_path").as_bool();
    waypoint_spacing_ = get_parameter("waypoint_spacing").as_double();

    max_iter_ = get_parameter("max_iter").as_int();
    step_size_ = get_parameter("step_size").as_int();
    connect_step_size_ = get_parameter("connect_step_size").as_int();
    max_connect_steps_ = get_parameter("max_connect_steps").as_int();
    goal_bias_rate_ = get_parameter("goal_bias_rate").as_double();
    connect_threshold_ = get_parameter("connect_threshold").as_int();
    random_seed_ = get_parameter("random_seed").as_int();

    alpha_ = get_parameter("alpha").as_double();
    beta_ = get_parameter("beta").as_double();
    gamma_ = get_parameter("gamma").as_double();

    debug_visualize_path_planner_ = get_parameter("debug_visualize_path_planner").as_bool();
    debug_path_planner_visualize_every_ = get_parameter("debug_path_planner_visualize_every").as_int();
    debug_path_planner_visualize_delay_ = get_parameter("debug_path_planner_visualize_delay").as_double();
    debug_publish_unsmoothed_path_ = get_parameter("debug_publish_unsmoothed_path").as_bool();

    if (random_seed_ != 0) {
      rng_.seed(static_cast<std::mt19937::result_type>(random_seed_));
    }

    const auto package_share = ament_index_cpp::get_package_share_directory("puzzlebot_navigation");
    const auto map_yaml_path = (std::filesystem::path(package_share) / "maps" / map_yaml_name_).string();

    load_map(map_yaml_path);
    inflate_obstacles(robot_radius_);
    free_cells_ = precompute_free_cells();

    if (free_cells_.empty()) {
      throw std::runtime_error("Map has no free cells available for sampling.");
    }

    path_pub_ = create_publisher<nav_msgs::msg::Path>("/path", 10);
    unsmoothed_path_pub_ = create_publisher<nav_msgs::msg::Path>("/path_planner_debug/unsmoothed_path", 10);

    tree_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/path_planner_debug/tree", 10);


    robot_pose_stamped_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      robot_pose_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&PathPlannerNode::robot_pose_stamped_callback, this, std::placeholders::_1));
    

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
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
      "Path planner ready | service=/plan_path | global_frame=%s, robot_pose_topic=%s, "
      "max_iter=%d, step_size=%d, connect_step_size=%d, max_connect_steps=%d, goal_bias=%.2f, "
      "weights=(alpha=%.2f, beta=%.2f, gamma=%.2f), debug_visualize_path_planner=%s, "
      "debug_path_planner_visualize_delay=%.3f, debug_publish_unsmoothed_path=%s",
      global_frame_.c_str(), robot_pose_topic_.c_str(), max_iter_, step_size_, connect_step_size_,
      max_connect_steps_, goal_bias_rate_, alpha_, beta_, gamma_,
      debug_visualize_path_planner_ ? "true" : "false",
      debug_path_planner_visualize_delay_,
      debug_publish_unsmoothed_path_ ? "true" : "false");
  }

private:
  void load_map(const std::string & yaml_path)
  {
    const YAML::Node map_info = YAML::LoadFile(yaml_path);
    const std::string image_file = map_info["image"].as<std::string>();
    resolution_ = map_info["resolution"].as<double>();
    origin_ = map_info["origin"].as<std::vector<double>>();
    if (origin_.size() < 2) {
      throw std::runtime_error("Map YAML origin must contain at least x and y.");
    }
    if (origin_.size() < 3) {
      origin_.resize(3, 0.0);
    }

    const auto image_path = (std::filesystem::path(yaml_path).parent_path() / image_file).string();
    const cv::Mat img = cv::imread(image_path, cv::IMREAD_GRAYSCALE);

    if (img.empty()) {
      throw std::runtime_error("Could not load map image: " + image_path);
    }

    grid_ = cv::Mat::zeros(img.size(), CV_8UC1);
    grid_.setTo(0, img > 250);
    grid_.setTo(1, img <= 250);
    grid_raw_ = grid_.clone();

    RCLCPP_INFO(
      get_logger(),
      "Map loaded: size=(%d, %d), resolution=%.4f, origin=(%.3f, %.3f, %.3f), slam_raw_pgm_order=%s",
      img.rows, img.cols, resolution_, origin_[0], origin_[1], origin_[2],
      slam_raw_pgm_order_ ? "true" : "false");
  }

  void inflate_obstacles(double robot_radius)
  {
    const int inflation_cells = static_cast<int>(std::ceil(robot_radius / resolution_));

    if (inflation_cells <= 0) {
      RCLCPP_WARN(get_logger(), "Obstacle inflation disabled.");
      return;
    }

    const int kernel_size = 2 * inflation_cells + 1;
    const cv::Mat kernel = cv::Mat::ones(kernel_size, kernel_size, CV_8UC1);
    cv::dilate(grid_, grid_, kernel, cv::Point(-1, -1), 1);

    RCLCPP_INFO(
      get_logger(), "Obstacles inflated by %d cells (%.3f m).", inflation_cells, robot_radius);
  }

  std::vector<GridCell> precompute_free_cells() const
  {
    std::vector<GridCell> cells;
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

  int world_y_to_map_row(double y) const
  {
    return static_cast<int>(std::floor((y - origin_[1]) / resolution_));
  }

  int map_row_to_image_row(int map_row) const
  {
    // The SLAM module publishes/saves occ_grid in row-major map order:
    // row 0 corresponds to y_min. Its save_map_to_pgm_yaml() writes that vector
    // directly into the PGM, so a PGM generated by this SLAM must NOT be flipped.
    //
    // If you load a standard ROS/map_server-style PGM instead, set
    // slam_raw_pgm_order:=false to convert map row 0 (bottom) to image row h-1.
    if (slam_raw_pgm_order_) {
      return map_row;
    }
    return grid_.rows - 1 - map_row;
  }

  int image_row_to_map_row(int image_row) const
  {
    if (slam_raw_pgm_order_) {
      return image_row;
    }
    return grid_.rows - 1 - image_row;
  }

  GridCell world_to_grid(double x, double y) const
  {
    // Reuses the same math as OccupancyGridMap::world_to_grid():
    // col = floor((x - x_min) / res), row = floor((y - y_min) / res).
    // GridCell is kept as {gx, gy_image} because the rest of this planner indexes
    // OpenCV as grid_.at<uint8_t>(gy, gx).
    const int gx = static_cast<int>(std::floor((x - origin_[0]) / resolution_));
    const int map_row = world_y_to_map_row(y);
    const int gy = map_row_to_image_row(map_row);
    return {gx, gy};
  }

  WorldPoint grid_to_world(int gx, int gy) const
  {
    // Inverse of world_to_grid(). The world point is the center of the grid cell,
    // same as OccupancyGridMap::grid_to_world(row, col).
    const int map_row = image_row_to_map_row(gy);
    const double x = origin_[0] + (static_cast<double>(gx) + 0.5) * resolution_;
    const double y = origin_[1] + (static_cast<double>(map_row) + 0.5) * resolution_;
    return {x, y};
  }

  bool is_cell_inside(const GridCell & cell) const
  {
    return cell.first >= 0 && cell.first < grid_.cols && cell.second >= 0 && cell.second < grid_.rows;
  }

  bool is_cell_free(const GridCell & cell) const
  {
    if (!is_cell_inside(cell)) {
      return false;
    }
    return grid_.at<uint8_t>(cell.second, cell.first) == 0;
  }

  
  void robot_pose_stamped_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    latest_robot_pose_ = *msg;
    has_robot_pose_ = true;
  }
  

  bool get_robot_pose(double & x, double & y, double & yaw)
  {
    if (!has_robot_pose_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "No robot pose received yet on topic %s.", robot_pose_topic_.c_str());
      return false;
    }

    if (!latest_robot_pose_.header.frame_id.empty() &&
        latest_robot_pose_.header.frame_id != global_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Robot pose frame is '%s', but planner global_frame is '%s'. Make sure the pose is already in map frame.",
        latest_robot_pose_.header.frame_id.c_str(), global_frame_.c_str());
    }

    x = latest_robot_pose_.pose.position.x;
    y = latest_robot_pose_.pose.position.y;

    const auto & q = latest_robot_pose_.pose.orientation;
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
    return {std::sin(yaw / 2.0), std::cos(yaw / 2.0)};  // z, w
  }

  static double distance(const GridCell & a, const GridCell & b)
  {
    return std::hypot(static_cast<double>(a.first - b.first), static_cast<double>(a.second - b.second));
  }

  geometry_msgs::msg::Point make_point(const GridCell & cell, double z = 0.04) const
  {
    const auto [x, y] = grid_to_world(cell.first, cell.second);
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  }

  visualization_msgs::msg::Marker make_line_marker(
    int marker_id,
    const std::string & ns,
    const std::unordered_map<GridCell, GridCell, PairHash> & parent,
    const std::unordered_map<GridCell, bool, PairHash> & has_parent,
    float r,
    float g,
    float b)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = global_frame_;
    marker.header.stamp = get_clock()->now();
    marker.ns = ns;
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.025;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.85;
    marker.pose.orientation.w = 1.0;

    for (const auto & [child, par] : parent) {
      const auto it = has_parent.find(child);
      if (it == has_parent.end() || !it->second) {
        continue;
      }
      marker.points.push_back(make_point(par, 0.02));
      marker.points.push_back(make_point(child, 0.02));
    }

    return marker;
  }

  void publish_tree_visualization(
    const std::unordered_map<GridCell, GridCell, PairHash> & parent_start,
    const std::unordered_map<GridCell, bool, PairHash> & has_parent_start,
    const std::unordered_map<GridCell, GridCell, PairHash> & parent_goal,
    const std::unordered_map<GridCell, bool, PairHash> & has_parent_goal)
  {
    if (!debug_visualize_path_planner_) {
      return;
    }

    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(make_line_marker(0, "start_tree", parent_start, has_parent_start, 0.0f, 0.8f, 0.1f));
    markers.markers.push_back(make_line_marker(1, "goal_tree", parent_goal, has_parent_goal, 0.1f, 0.3f, 1.0f));
    tree_pub_->publish(markers);
  }

  void clear_debug_markers()
  {

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = global_frame_;
    delete_marker.header.stamp = get_clock()->now();
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(delete_marker);
    tree_pub_->publish(markers);
  }

  void reset_debug_replay()
  {
    debug_frames_.clear();
    debug_replay_index_ = 0;

    if (debug_replay_timer_) {
      debug_replay_timer_->cancel();
      debug_replay_timer_.reset();
    }

    if (debug_visualize_path_planner_) {
      clear_debug_markers();
    }
  }

  void store_debug_frame(
    int iteration,
    const std::unordered_map<GridCell, GridCell, PairHash> & parent_start,
    const std::unordered_map<GridCell, bool, PairHash> & has_parent_start,
    const std::unordered_map<GridCell, GridCell, PairHash> & parent_goal,
    const std::unordered_map<GridCell, bool, PairHash> & has_parent_goal,
    bool force = false)
  {
    if (!debug_visualize_path_planner_ || debug_path_planner_visualize_every_ <= 0) {
      return;
    }
    if (!force && iteration % debug_path_planner_visualize_every_ != 0) {
      return;
    }

    DebugFrame frame;
    frame.parent_start = parent_start;
    frame.has_parent_start = has_parent_start;
    frame.parent_goal = parent_goal;
    frame.has_parent_goal = has_parent_goal;

    debug_frames_.push_back(std::move(frame));
  }

  void publish_debug_frame(const DebugFrame & frame)
  {
    publish_tree_visualization(
      frame.parent_start,
      frame.has_parent_start,
      frame.parent_goal,
      frame.has_parent_goal);
  }

  void start_debug_replay()
  {
    if (!debug_visualize_path_planner_ || debug_frames_.empty()) {
      return;
    }

    debug_replay_index_ = 0;

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(std::max(0.001, debug_path_planner_visualize_delay_)));

    debug_replay_timer_ = create_wall_timer(
      period,
      std::bind(&PathPlannerNode::publish_next_debug_frame, this));
  }

  void publish_next_debug_frame()
  {
    if (debug_replay_index_ >= debug_frames_.size()) {
      if (debug_replay_timer_) {
        debug_replay_timer_->cancel();
        debug_replay_timer_.reset();
      }
      return;
    }

    publish_debug_frame(debug_frames_[debug_replay_index_]);
    ++debug_replay_index_;
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
    const double dist = std::hypot(static_cast<double>(dx), static_cast<double>(dy));

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
      const bool extended = extend_heuristic(
        tree_nodes, parent, has_parent, g_cost, q_target, q_target,
        connect_step_size_, q_new, q_near);

      if (!extended) {
        return false;
      }

      last_new = q_new;
      last_near = q_near;

      if (distance(q_new, q_target) <= static_cast<double>(connect_threshold_) && line_is_free(q_new, q_target)) {
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
    reset_debug_replay();

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

      const bool extended = extend_heuristic(
        tree_a, parent_a, has_parent_a, g_a,
        q_sample, target_a, step_size_, q_new, q_near);

      const auto & parent_start_view = a_is_start_tree ? parent_a : parent_b;
      const auto & has_parent_start_view = a_is_start_tree ? has_parent_a : has_parent_b;
      const auto & parent_goal_view = a_is_start_tree ? parent_b : parent_a;
      const auto & has_parent_goal_view = a_is_start_tree ? has_parent_b : has_parent_a;

      if (extended) {
        store_debug_frame(
          iteration,
          parent_start_view, has_parent_start_view,
          parent_goal_view, has_parent_goal_view);

        GridCell meeting_node;
        GridCell connect_near;
        const bool connected = connect_heuristic(
          tree_b, parent_b, has_parent_b, g_b,
          q_new, max_connect_steps_, meeting_node, connect_near);

        if (connected) {
          if (a_is_start_tree) {
            store_debug_frame(
              iteration,
              parent_a, has_parent_a,
              parent_b, has_parent_b,
              true);

            path = build_bidirectional_path(parent_a, has_parent_a, parent_b, has_parent_b, meeting_node);
          } else {
            store_debug_frame(
              iteration,
              parent_b, has_parent_b,
              parent_a, has_parent_a,
              true);

            path = build_bidirectional_path(parent_b, has_parent_b, parent_a, has_parent_a, meeting_node);
          }
          return true;
        }
      } else {
        store_debug_frame(
          iteration,
          parent_start_view, has_parent_start_view,
          parent_goal_view, has_parent_goal_view);
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

  std::vector<WorldPoint> resample_path_world(const std::vector<WorldPoint> & path_world, double spacing) const
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

  void publish_unsmoothed_debug_path(
    const std::vector<GridCell> & original_path_grid,
    const WorldPoint & start_world,
    const WorldPoint & goal_world)
  {
    if (!debug_publish_unsmoothed_path_) {
      return;
    }

    const auto raw_path_world = grid_path_to_world_path(original_path_grid, &start_world, &goal_world, false);
    const auto raw_path_msg = build_path_msg(raw_path_world);
    unsmoothed_path_pub_->publish(raw_path_msg);
  }

  std::tuple<bool, std::string, nav_msgs::msg::Path> compute_path_to_goal(
    const geometry_msgs::msg::PoseStamped & goal_msg)
  {
    double robot_x = 0.0;
    double robot_y = 0.0;
    double robot_yaw = 0.0;

    if (!get_robot_pose(robot_x, robot_y, robot_yaw)) {
      return {false, "No robot pose available. Cannot plan.", nav_msgs::msg::Path()};
    }

    if (!goal_msg.header.frame_id.empty() && goal_msg.header.frame_id != global_frame_) {
      return {
        false,
        "Goal frame is \"" + goal_msg.header.frame_id + "\", but planner frame is \"" + global_frame_ + "\".",
        nav_msgs::msg::Path()};
    }

    const double goal_x = goal_msg.pose.position.x;
    const double goal_y = goal_msg.pose.position.y;

    const GridCell start_grid = world_to_grid(robot_x, robot_y);
    const GridCell goal_grid = world_to_grid(goal_x, goal_y);

    if (!is_cell_free(start_grid)) {
      return {false, "Start cell is occupied or outside map.", nav_msgs::msg::Path()};
    }
    if (!is_cell_free(goal_grid)) {
      return {false, "Goal cell is occupied or outside map.", nav_msgs::msg::Path()};
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
    const auto path_world = grid_path_to_world_path(path_grid, &start_world, &goal_world, true);
    auto path_msg = build_path_msg(path_world);

    // Publicación inmediata del path final/smooth. El debug animado corre después con timer.
    path_pub_->publish(path_msg);

    // Esto publica el path sin suavizar como referencia de debug, sin delay.
    publish_unsmoothed_debug_path(original_path_grid, start_world, goal_world);

    // El árbol se reproduce con delay, sin bloquear la publicación de /path.
    start_debug_replay();

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

  std::string map_yaml_name_;
  bool slam_raw_pgm_order_ = true;
  std::string global_frame_;
  std::string robot_pose_topic_;
  double robot_radius_ = 0.0;
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

  bool debug_visualize_path_planner_ = true;
  int debug_path_planner_visualize_every_ = 1;
  double debug_path_planner_visualize_delay_ = 0.02;
  bool debug_publish_unsmoothed_path_ = true;

  std::vector<DebugFrame> debug_frames_;
  std::size_t debug_replay_index_ = 0;
  rclcpp::TimerBase::SharedPtr debug_replay_timer_;

  cv::Mat grid_raw_;
  cv::Mat grid_;
  double resolution_ = 0.05;
  std::vector<double> origin_{0.0, 0.0, 0.0};
  std::vector<GridCell> free_cells_;

  std::mt19937 rng_;


  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr unsmoothed_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tree_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_stamped_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Service<PlanPath>::SharedPtr plan_path_srv_;

  // Stores the latest robot pose received from robot_pose_topic_.
  geometry_msgs::msg::PoseStamped latest_robot_pose_;
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