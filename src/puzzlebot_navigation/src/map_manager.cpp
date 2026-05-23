#include "puzzlebot_navigation/map_manager.hpp"

#include <algorithm>
#include <cmath>

namespace puzzlebot_navigation
{

void MapManager::trace_ray_and_update_free_cells(int r0, int c0, int r1, int c1)
{
  const int dr = std::abs(r1 - r0);
  const int dc = std::abs(c1 - c0);
  const int sr = (r0 < r1) ? 1 : -1;
  const int sc = (c0 < c1) ? 1 : -1;

  const int ray_len = std::max(dr, dc) + 1;
  const int free_end = std::max(0, ray_len - free_margin_cells_);

  int step_count = 0;
  int r = r0;
  int c = c0;

  if (dc > dr) {
    float err = static_cast<float>(dc) * 0.5f;
    while (c != c1) {
      if (step_count < free_end) {
        map_.add_logodds(r, c, map_.l_free());
      }
      ++step_count;

      err -= static_cast<float>(dr);
      if (err < 0.0f) {
        r += sr;
        err += static_cast<float>(dc);
      }
      c += sc;
    }
  } else {
    float err = static_cast<float>(dr) * 0.5f;
    while (r != r1) {
      if (step_count < free_end) {
        map_.add_logodds(r, c, map_.l_free());
      }
      ++step_count;

      err -= static_cast<float>(dc);
      if (err < 0.0f) {
        c += sc;
        err += static_cast<float>(dr);
      }
      r += sr;
    }
  }

  if (step_count < free_end) {
    map_.add_logodds(r1, c1, map_.l_free());
  }
}

void MapManager::integrate_scan_impl(
  const Pose2D & pose,
  const std::vector<float> & scan_x,
  const std::vector<float> & scan_y,
  const std::vector<uint8_t> & scan_hit,
  bool count_scan)
{
  if (scan_x.empty()) {
    return;
  }

  const auto [r0, c0] = map_.world_to_grid(pose.x, pose.y);
  if (!map_.inside_rc(r0, c0)) {
    return;
  }

  const float c = std::cos(pose.theta);
  const float s = std::sin(pose.theta);

  for (size_t i = 0; i < scan_x.size(); ++i) {
    const float wx = pose.x + c * scan_x[i] - s * scan_y[i];
    const float wy = pose.y + s * scan_x[i] + c * scan_y[i];

    const auto [r1, c1] = map_.world_to_grid(wx, wy);
    if (!map_.inside_rc(r1, c1)) {
      continue;
    }

    trace_ray_and_update_free_cells(r0, c0, r1, c1);

    if (i < scan_hit.size() && scan_hit[i] != 0u) {
      map_.add_logodds(r1, c1, map_.l_occ());
    }
  }

  if (count_scan) {
    ++mapped_scans_;
  }
}

void MapManager::integrate_scan(
  const Pose2D & pose,
  const std::vector<float> & scan_x,
  const std::vector<float> & scan_y,
  const std::vector<uint8_t> & scan_hit)
{
  integrate_scan_impl(pose, scan_x, scan_y, scan_hit, true);
}

void MapManager::rebuild_from_keyframes(const std::vector<GraphMapNodeView> & nodes)
{
  map_.reset(true);
  mapped_scans_ = 0;

  for (const auto & node : nodes) {
    if (!node.scan_x || !node.scan_y || !node.scan_hit) {
      continue;
    }
    integrate_scan_impl(node.pose, *node.scan_x, *node.scan_y, *node.scan_hit, true);
  }
}


}  // namespace puzzlebot_navigation
