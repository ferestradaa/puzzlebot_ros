#include <algorithm>
#include <cmath>

#include "puzzlebot_navigation/scan_processor.hpp"


namespace puzzlebot_navigation
{

void ScanProcessor::configure(
  int scan_step,
  int max_scan_points_gpu,
  int min_valid_scan_points,
  float usable_max_range,
  float hit_range_margin,
  float scan_angle_offset)
{
  scan_step_ = scan_step;
  max_scan_points_gpu_ = max_scan_points_gpu;
  min_valid_scan_points_ = min_valid_scan_points;
  usable_max_range_ = usable_max_range;
  hit_range_margin_ = hit_range_margin;
  scan_angle_offset_ = scan_angle_offset;
}

void ScanProcessor::update_trig_cache_if_needed(const sensor_msgs::msg::LaserScan & scan)
{
  const int current_size = static_cast<int>(scan.ranges.size());
  const float angle_min = static_cast<float>(scan.angle_min);
  const float angle_inc = static_cast<float>(scan.angle_increment);

  if (current_size == cached_scan_size_ &&
      angle_min == cached_angle_min_ &&
      angle_inc == cached_angle_increment_) {
    return;
  }

  cached_scan_size_ = current_size;
  cached_angle_min_ = angle_min;
  cached_angle_increment_ = angle_inc;

  cos_cache_.resize(static_cast<size_t>(current_size));
  sin_cache_.resize(static_cast<size_t>(current_size));

  for (int k = 0; k < current_size; ++k) {
    const float angle = angle_min + static_cast<float>(k) * angle_inc + scan_angle_offset_;
    cos_cache_[static_cast<size_t>(k)] = std::cos(angle);
    sin_cache_[static_cast<size_t>(k)] = std::sin(angle);
  }
}

void ScanProcessor::process(const sensor_msgs::msg::LaserScan & scan)
{
  scan_x_.clear();
  scan_y_.clear();
  scan_hit_.clear();

  update_trig_cache_if_needed(scan);

  int effective_step = std::max(1, scan_step_);
  if (!scan.ranges.empty()) {
    const int projected_points = static_cast<int>(scan.ranges.size()) / effective_step;
    if (projected_points > max_scan_points_gpu_) {
      const float factor = static_cast<float>(projected_points) / static_cast<float>(max_scan_points_gpu_);
      effective_step = std::max(1, static_cast<int>(std::ceil(static_cast<float>(effective_step) * factor)));
    }
  }

  const int reserve_points = std::min(
    static_cast<int>(scan.ranges.size() / static_cast<size_t>(effective_step)),
    max_scan_points_gpu_);

  scan_x_.reserve(static_cast<size_t>(reserve_points));
  scan_y_.reserve(static_cast<size_t>(reserve_points));
  scan_hit_.reserve(static_cast<size_t>(reserve_points));

  const float effective_max_range =
    std::min(static_cast<float>(scan.range_max), usable_max_range_);

  for (size_t k = 0; k < scan.ranges.size(); k += static_cast<size_t>(effective_step)) {
    const float d = static_cast<float>(scan.ranges[k]);
    if (std::isnan(d) || std::isinf(d)) {
      continue;
    }
    if (d < static_cast<float>(scan.range_min) || d > effective_max_range) {
      continue;
    }

    const bool is_real_hit = d < (effective_max_range - hit_range_margin_);
    scan_x_.push_back(d * cos_cache_[k]);
    scan_y_.push_back(d * sin_cache_[k]);
    scan_hit_.push_back(is_real_hit ? 1u : 0u);

    if (static_cast<int>(scan_x_.size()) >= max_scan_points_gpu_) {
      break;
    }
  }
}

}  // namespace puzzlebot_navigation
