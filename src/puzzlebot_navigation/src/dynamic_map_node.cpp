#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "puzzlebot_navigation/occupancy_grid_map.hpp"
#include "puzzlebot_navigation/scan_processor.hpp"

class DynamicMapNode : public rclcpp::Node
{
public:
  explicit DynamicMapNode()
  : Node("dynamic_map_node")
  {
    declare_and_load_parameters();

    rclcpp::QoS static_map_qos(1);
    static_map_qos.reliable();
    static_map_qos.transient_local();

    static_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      static_map_topic_,
      static_map_qos,
      std::bind(&DynamicMapNode::static_map_callback, this, std::placeholders::_1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&DynamicMapNode::scan_callback, this, std::placeholders::_1));

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_,
      rclcpp::QoS(10),
      std::bind(&DynamicMapNode::path_callback, this, std::placeholders::_1));


    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, static_map_qos);
    path_blocked_pub_ = create_publisher<std_msgs::msg::Bool>(path_blocked_topic_, 10);

    map_pub_timer_ = create_wall_timer(
      std::chrono::milliseconds(map_publish_period_ms_),
      std::bind(&DynamicMapNode::publish_map, this));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    publish_path_blocked(false);
  }

private:
  void declare_and_load_parameters()
  {
    declare_parameter<std::string>("global_frame", "map");
    declare_parameter<std::string>("robot_frame", "base_link");

    declare_parameter<std::string>("static_map_topic", "/map");
    declare_parameter<std::string>("scan_topic", "/scan");

    declare_parameter<std::string>("map_topic", "/map");

    declare_parameter<std::string>("path_topic", "/path");
    declare_parameter<std::string>("path_blocked_topic", "/path_blocked");

    declare_parameter<int>("scan_step", 8);
    declare_parameter<int>("max_scan_points_gpu", 2048);
    declare_parameter<int>("min_valid_scan_points", 50);

    declare_parameter<double>("usable_max_range", 4.0);
    declare_parameter<double>("flag_obstacle_range", 0.5);
    declare_parameter<double>("path_blocking_tolerance", 0.20);

    declare_parameter<double>("hit_range_margin", 0.05);
    declare_parameter<double>("scan_angle_offset", -3.14);
    declare_parameter<double>("tf_timeout", 0.05);

    declare_parameter<double>("dynamic_hit_increment", 1.0);
    declare_parameter<double>("dynamic_miss_decay", 0.35);
    declare_parameter<double>("dynamic_lock_threshold", 3.0);
    declare_parameter<double>("dynamic_max_score", 6.0);
    declare_parameter<int>("dynamic_decay_every_scans", 1);

    declare_parameter<int>("map_publish_period_ms", 400);

    global_frame_ = get_parameter("global_frame").as_string();
    robot_frame_ = get_parameter("robot_frame").as_string();
    static_map_topic_ = get_parameter("static_map_topic").as_string();
    scan_topic_ = get_parameter("scan_topic").as_string();
    map_topic_ = get_parameter("map_topic").as_string();
    path_topic_ = get_parameter("path_topic").as_string();
    path_blocked_topic_ = get_parameter("path_blocked_topic").as_string();

    scan_step_ = std::max(1, static_cast<int>(get_parameter("scan_step").as_int()));
    max_scan_points_gpu_ =
      std::max(1, static_cast<int>(get_parameter("max_scan_points_gpu").as_int()));
    min_valid_scan_points_ =
      std::max(1, static_cast<int>(get_parameter("min_valid_scan_points").as_int()));

    usable_max_range_ = static_cast<float>(get_parameter("usable_max_range").as_double());
    flag_obstacle_range_ = static_cast<float>(get_parameter("flag_obstacle_range").as_double());
    path_blocking_tolerance_ =
      static_cast<float>(get_parameter("path_blocking_tolerance").as_double());

    hit_range_margin_ = static_cast<float>(get_parameter("hit_range_margin").as_double());
    scan_angle_offset_ = static_cast<float>(get_parameter("scan_angle_offset").as_double());
    tf_timeout_ = get_parameter("tf_timeout").as_double();

    dynamic_hit_increment_ =
      static_cast<float>(get_parameter("dynamic_hit_increment").as_double());
    dynamic_miss_decay_ =
      static_cast<float>(get_parameter("dynamic_miss_decay").as_double());
    dynamic_lock_threshold_ =
      static_cast<float>(get_parameter("dynamic_lock_threshold").as_double());
    dynamic_max_score_ =
      static_cast<float>(get_parameter("dynamic_max_score").as_double());

    dynamic_decay_every_scans_ =
      std::max(1, static_cast<int>(get_parameter("dynamic_decay_every_scans").as_int()));

    map_publish_period_ms_ =
      std::max(50, static_cast<int>(get_parameter("map_publish_period_ms").as_int()));
  }

  void static_map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (has_static_map_) {
      return;
    }

    if (msg->info.width == 0 || msg->info.height == 0 || msg->data.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty static map, ignoring.");
      return;
    }

    width_ = static_cast<int>(msg->info.width);
    height_ = static_cast<int>(msg->info.height);
    resolution_ = msg->info.resolution;

    origin_.resize(3);
    origin_[0] = msg->info.origin.position.x;
    origin_[1] = msg->info.origin.position.y;
    origin_[2] = quaternion_to_yaw(
      msg->info.origin.orientation.x,
      msg->info.origin.orientation.y,
      msg->info.origin.orientation.z,
      msg->info.origin.orientation.w);

    static_map_.assign(msg->data.begin(), msg->data.end());

    configure_grid_geometry();
    initialize_dynamic_layers();

    scan_processor_.configure(
      scan_step_,
      max_scan_points_gpu_,
      min_valid_scan_points_,
      usable_max_range_,
      hit_range_margin_,
      scan_angle_offset_);

    has_static_map_ = true;

    RCLCPP_INFO(get_logger(), "Static map received once. Publishing updated map on /map.");
  }

  void configure_grid_geometry()
  {
    const float x_min = static_cast<float>(origin_[0]);
    const float y_min = static_cast<float>(origin_[1]);
    const float x_max = x_min + static_cast<float>(width_) * resolution_;
    const float y_max = y_min + static_cast<float>(height_) * resolution_;

    geometry_map_.configure(
      x_min, x_max, y_min, y_max, resolution_,
      1.0f, -0.10f, -6.0f, 12.0f, 11.5f, -5.5f);

    geometry_map_.initialize();
  }

  void initialize_dynamic_layers()
  {
    const std::size_t size = static_cast<std::size_t>(width_ * height_);
    dynamic_score_.assign(size, 0.0f);
    dynamic_seen_this_scan_.assign(size, 0u);
    dynamic_overlay_.assign(size, -1);
    combined_map_ = static_map_;
  }

  void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    latest_path_.clear();

    for (const auto & pose_stamped : msg->poses) {
      latest_path_.push_back({
        static_cast<float>(pose_stamped.pose.position.x),
        static_cast<float>(pose_stamped.pose.position.y)
      });
    }

    has_path_ = !latest_path_.empty();
  }

  bool get_robot_pose(puzzlebot_navigation::Pose2D & pose)
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        global_frame_,
        robot_frame_,
        tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_));

      tf2::Quaternion q;
      tf2::fromMsg(tf.transform.rotation, q);

      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      pose = puzzlebot_navigation::Pose2D{
        static_cast<float>(tf.transform.translation.x),
        static_cast<float>(tf.transform.translation.y),
        static_cast<float>(yaw)
      };

      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "TF lookup %s -> %s failed: %s",
        global_frame_.c_str(),
        robot_frame_.c_str(),
        ex.what());

      return false;
    }
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!has_static_map_) {
      return;
    }

    puzzlebot_navigation::Pose2D pose;
    if (!get_robot_pose(pose)) {
      return;
    }

    scan_processor_.process(*msg);

    if (!scan_processor_.valid()) {
      return;
    }

    ++scan_counter_;

    integrate_dynamic_obstacles_only(pose);

    if ((scan_counter_ % dynamic_decay_every_scans_) == 0) {
      decay_unseen_obstacles();
    }

    rebuild_dynamic_overlay();

    publish_path_blocked(is_path_blocked_by_dynamic_obstacle(pose));

    rebuild_combined_map();
  }

  void integrate_dynamic_obstacles_only(const puzzlebot_navigation::Pose2D & pose)
  {
    std::fill(dynamic_seen_this_scan_.begin(), dynamic_seen_this_scan_.end(), 0u);

    const auto & scan_x = scan_processor_.x();
    const auto & scan_y = scan_processor_.y();
    const auto & scan_hit = scan_processor_.hits();

    const float c = std::cos(pose.theta);
    const float s = std::sin(pose.theta);

    for (std::size_t i = 0; i < scan_x.size(); ++i) {
      if (i < scan_hit.size() && scan_hit[i] == 0u) {
        continue;
      }

      const float range = std::hypot(scan_x[i], scan_y[i]);
      if (range > usable_max_range_) {
        continue;
      }

      const float wx = pose.x + c * scan_x[i] - s * scan_y[i];
      const float wy = pose.y + s * scan_x[i] + c * scan_y[i];

      const auto [row, col] = geometry_map_.world_to_grid(wx, wy);
      if (!geometry_map_.inside_rc(row, col)) {
        continue;
      }

      const int k = geometry_map_.idx(row, col);
      const auto ks = static_cast<std::size_t>(k);

      if (static_map_[ks] == 100) {
        continue;
      }

      dynamic_seen_this_scan_[ks] = 1u;
      dynamic_score_[ks] = std::min(dynamic_max_score_, dynamic_score_[ks] + dynamic_hit_increment_);
    }
  }

  void decay_unseen_obstacles()
  {
    for (std::size_t i = 0; i < dynamic_score_.size(); ++i) {
      if (dynamic_seen_this_scan_[i] == 0u) {
        dynamic_score_[i] = std::max(0.0f, dynamic_score_[i] - dynamic_miss_decay_);
      }
    }
  }

  void rebuild_dynamic_overlay()
  {
    std::fill(dynamic_overlay_.begin(), dynamic_overlay_.end(), -1);

    for (std::size_t i = 0; i < dynamic_overlay_.size(); ++i) {
      if (static_map_[i] != 100 && dynamic_score_[i] >= dynamic_lock_threshold_) {
        dynamic_overlay_[i] = 100;
      }
    }
  }

  bool is_path_blocked_by_dynamic_obstacle(const puzzlebot_navigation::Pose2D & pose) const
  {
    if (!has_path_) {
      return false;
    }

    const float path_tol_sq = path_blocking_tolerance_ * path_blocking_tolerance_;
    const float flag_range_sq = flag_obstacle_range_ * flag_obstacle_range_;

    for (std::size_t i = 0; i < dynamic_overlay_.size(); ++i) {
      if (dynamic_overlay_[i] != 100) {
        continue;
      }

      const int row = static_cast<int>(i) / width_;
      const int col = static_cast<int>(i) % width_;

      const float obstacle_x = origin_[0] + (col + 0.5f) * resolution_;
      const float obstacle_y = origin_[1] + (row + 0.5f) * resolution_;

      const float robot_dx = obstacle_x - pose.x;
      const float robot_dy = obstacle_y - pose.y;

      if ((robot_dx * robot_dx + robot_dy * robot_dy) > flag_range_sq) {
        continue;
      }

      for (const auto & path_point : latest_path_) {
        const float path_dx = obstacle_x - path_point.first;
        const float path_dy = obstacle_y - path_point.second;

        if ((path_dx * path_dx + path_dy * path_dy) <= path_tol_sq) {
          return true;
        }
      }
    }

    return false;
  }

  void publish_path_blocked(bool blocked)
  {
    std_msgs::msg::Bool msg;
    msg.data = blocked;
    path_blocked_pub_->publish(msg);
  }

  void rebuild_combined_map()
  {
    combined_map_ = static_map_;

    for (std::size_t i = 0; i < combined_map_.size(); ++i) {
      if (static_map_[i] == 100 || dynamic_overlay_[i] == 100) {
        combined_map_[i] = 100;
      }
    }
  }

  void publish_map()
  {
    if (!has_static_map_) {
      return;
    }

    nav_msgs::msg::OccupancyGrid msg;

    msg.header.stamp = now();
    msg.header.frame_id = global_frame_;

    msg.info.resolution = resolution_;
    msg.info.width = static_cast<uint32_t>(width_);
    msg.info.height = static_cast<uint32_t>(height_);

    msg.info.origin.position.x = origin_[0];
    msg.info.origin.position.y = origin_[1];
    msg.info.origin.position.z = 0.0;

    msg.info.origin.orientation.z = std::sin(origin_[2] * 0.5);
    msg.info.origin.orientation.w = std::cos(origin_[2] * 0.5);

    msg.data = combined_map_;

    map_pub_->publish(msg);
  }

  static double quaternion_to_yaw(double x, double y, double z, double w)
  {
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  std::string global_frame_;
  std::string robot_frame_;
  std::string static_map_topic_;
  std::string scan_topic_;
  std::string map_topic_;
  std::string path_topic_;
  std::string path_blocked_topic_;

  int width_{0};
  int height_{0};
  float resolution_{0.05f};
  std::vector<double> origin_{0.0, 0.0, 0.0};

  int scan_step_{8};
  int max_scan_points_gpu_{2048};
  int min_valid_scan_points_{50};

  float usable_max_range_{4.0f};
  float flag_obstacle_range_{0.5f};
  float path_blocking_tolerance_{0.20f};

  float hit_range_margin_{0.05f};
  float scan_angle_offset_{-3.14f};
  double tf_timeout_{0.05};

  float dynamic_hit_increment_{1.0f};
  float dynamic_miss_decay_{0.35f};
  float dynamic_lock_threshold_{3.0f};
  float dynamic_max_score_{6.0f};
  int dynamic_decay_every_scans_{1};
  int scan_counter_{0};

  int map_publish_period_ms_{400};

  std::vector<int8_t> static_map_;
  std::vector<float> dynamic_score_;
  std::vector<uint8_t> dynamic_seen_this_scan_;
  std::vector<int8_t> dynamic_overlay_;
  std::vector<int8_t> combined_map_;

  std::vector<std::pair<float, float>> latest_path_;
  bool has_path_{false};

  puzzlebot_navigation::OccupancyGridMap geometry_map_;
  puzzlebot_navigation::ScanProcessor scan_processor_;

  bool has_static_map_{false};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr static_map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr path_blocked_pub_;

  rclcpp::TimerBase::SharedPtr map_pub_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<DynamicMapNode>());
  } catch (const std::exception & ex) {
    std::cerr << "Fatal error: " << ex.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}