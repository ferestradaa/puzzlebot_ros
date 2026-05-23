#pragma once

#include "puzzlebot_navigation/occupancy_grid_map.hpp"

#include <vector>

namespace puzzlebot_navigation
{

class LikelihoodField
{
public:
  void configure(float sigma, float max_distance, int rebuild_period, int min_dirty_occupied_cells);
  void initialize(const OccupancyGridMap & map);

  // Returns true only when the field was rebuilt and should be uploaded to the GPU.
  bool upload_rebuilt_from_map_if_needed(OccupancyGridMap & map, bool force = false);
  void force_rebuild_from_map(OccupancyGridMap & map);
  bool dirty() const { return dirty_; }
  void set_dirty(bool value) { dirty_ = value; }

  float sigma() const { return sigma_; }
  float max_distance() const { return max_distance_; }
  const std::vector<float> & field() const { return field_; }

  void rebuild(const OccupancyGridMap & map);

private:
  float sigma_{0.15f};
  float max_distance_{0.60f};
  int rebuild_period_{5};
  int min_dirty_occupied_cells_{20};
  int rebuild_counter_{0};
  bool dirty_{true};
  std::vector<float> field_;
};

}  // namespace puzzlebot_navigation