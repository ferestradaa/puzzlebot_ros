#pragma once

#include "puzzlebot_navigation/occupancy_grid_map.hpp"
#include "puzzlebot_navigation/slam_types.hpp"

#include <cstdint>
#include <vector>

namespace puzzlebot_navigation
{

struct GraphMapNodeView
{
  Pose2D pose{};
  const std::vector<float> * scan_x{nullptr};
  const std::vector<float> * scan_y{nullptr};
  const std::vector<uint8_t> * scan_hit{nullptr};
};

class MapManager
{
public:
  explicit MapManager(OccupancyGridMap & map) : map_(map) {}

  void configure(int free_margin_cells) { free_margin_cells_ = free_margin_cells; }

  void integrate_scan(
    const Pose2D & pose,
    const std::vector<float> & scan_x,
    const std::vector<float> & scan_y,
    const std::vector<uint8_t> & scan_hit);

  void rebuild_from_keyframes(const std::vector<GraphMapNodeView> & nodes);
  void reset_mapped_scans() { mapped_scans_ = 0; }
  int mapped_scans() const { return mapped_scans_; }

private:
  void trace_ray_and_update_free_cells(int r0, int c0, int r1, int c1);
  void integrate_scan_impl(
    const Pose2D & pose,
    const std::vector<float> & scan_x,
    const std::vector<float> & scan_y,
    const std::vector<uint8_t> & scan_hit,
    bool count_scan);

  OccupancyGridMap & map_;
  int free_margin_cells_{3};
  int mapped_scans_{0};
};

}  // namespace puzzlebot_navigation
