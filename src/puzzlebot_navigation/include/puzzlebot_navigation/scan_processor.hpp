#pragma once

#include "puzzlebot_navigation/slam_types.hpp"

#include <sensor_msgs/msg/laser_scan.hpp>
#include <cstdint>
#include <vector>

namespace puzzlebot_navigation
{

class ScanProcessor
{
public:
  void configure(
    int scan_step,
    int max_scan_points_gpu,
    int min_valid_scan_points,
    float usable_max_range,
    float hit_range_margin,
    float scan_angle_offset);

  void process(const sensor_msgs::msg::LaserScan & scan);

  bool valid() const { return static_cast<int>(scan_x_.size()) >= min_valid_scan_points_; }
  int valid_points() const { return static_cast<int>(scan_x_.size()); }

  const std::vector<float> & x() const { return scan_x_; }
  const std::vector<float> & y() const { return scan_y_; }
  const std::vector<uint8_t> & hits() const { return scan_hit_; }

private:
  void update_trig_cache_if_needed(const sensor_msgs::msg::LaserScan & scan);

  int scan_step_{8};
  int max_scan_points_gpu_{2048};
  int min_valid_scan_points_{50};
  float usable_max_range_{3.0f};
  float hit_range_margin_{0.05f};
  float scan_angle_offset_{kPi * 0.5f};

  int cached_scan_size_{-1};
  float cached_angle_min_{0.0f};
  float cached_angle_increment_{0.0f};
  std::vector<float> cos_cache_;
  std::vector<float> sin_cache_;

  std::vector<float> scan_x_;
  std::vector<float> scan_y_;
  std::vector<uint8_t> scan_hit_;
};

}  // namespace puzzlebot_navigation
