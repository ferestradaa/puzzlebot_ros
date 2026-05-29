#include "puzzlebot_navigation/occupancy_grid_map.hpp"

#include <algorithm>
#include <cmath>


namespace puzzlebot_navigation
{

void OccupancyGridMap::configure(
  float x_min, float x_max, float y_min, float y_max, float resolution,
  float l_occ, float l_free, float l_min, float l_max,
  float l_occ_lock, float l_free_lock)
{
  x_min_ = x_min;
  x_max_ = x_max;
  y_min_ = y_min;
  y_max_ = y_max;
  res_ = resolution;
  l_occ_ = l_occ;
  l_free_ = l_free;
  l_min_ = l_min;
  l_max_ = l_max;
  l_occ_lock_ = l_occ_lock;
  l_free_lock_ = l_free_lock;
}

void OccupancyGridMap::initialize()
{
  width_ = static_cast<int>((x_max_ - x_min_) / res_);
  height_ = static_cast<int>((y_max_ - y_min_) / res_);
  const size_t map_size = static_cast<size_t>(width_ * height_);

  map_logodds_.assign(map_size, 0.0f);
  occ_grid_.assign(map_size, -1);
  locked_cells_.assign(map_size, 0u);
  dirty_marks_.assign(map_size, 0u);
  dirty_cell_indices_.clear();
  free_cells_cache_.clear();
  free_cells_dirty_ = true;
  map_changed_ = true;
  likelihood_field_dirty_ = true;
  likelihood_dirty_occupied_cells_ = 0;
}



void OccupancyGridMap::reset(bool clear_locks)
{
  const size_t map_size = static_cast<size_t>(width_ * height_);
  map_logodds_.assign(map_size, 0.0f);
  occ_grid_.assign(map_size, -1);
  if (clear_locks) {
    locked_cells_.assign(map_size, 0u);
  }
  dirty_marks_.assign(map_size, 0u);
  dirty_cell_indices_.clear();
  free_cells_cache_.clear();
  free_cells_dirty_ = true;
  map_changed_ = true;
  likelihood_field_dirty_ = true;
  likelihood_dirty_occupied_cells_ = width_ * height_;
  mark_all_dirty();
}

void OccupancyGridMap::mark_all_dirty()
{
  dirty_cell_indices_.clear();
  dirty_cell_indices_.reserve(static_cast<size_t>(width_ * height_));
  dirty_marks_.assign(static_cast<size_t>(width_ * height_), 1u);
  for (int k = 0; k < width_ * height_; ++k) {
    dirty_cell_indices_.push_back(k);
  }
}

bool OccupancyGridMap::inside_rc(int row, int col) const
{
  return row >= 0 && row < height_ && col >= 0 && col < width_;
}

std::pair<int, int> OccupancyGridMap::world_to_grid(float x, float y) const
{
  const int col = static_cast<int>(std::floor((x - x_min_) / res_));
  const int row = static_cast<int>(std::floor((y - y_min_) / res_));
  return {row, col};
}

std::pair<float, float> OccupancyGridMap::grid_to_world(int row, int col) const
{
  const float x = x_min_ + (static_cast<float>(col) + 0.5f) * res_;
  const float y = y_min_ + (static_cast<float>(row) + 0.5f) * res_;
  return {x, y};
}

int8_t OccupancyGridMap::occ_value_from_logodds(float l) const
{
  if (l < LOGODDS_FREE_THRESH) {
    return 0;
  }
  if (l > LOGODDS_OCC_THRESH) {
    return 100;
  }
  return -1;
}

void OccupancyGridMap::update_occ_grid_cell_from_logodds_index(int k)
{
  occ_grid_[static_cast<size_t>(k)] = occ_value_from_logodds(map_logodds_[static_cast<size_t>(k)]);
}

void OccupancyGridMap::add_dirty_cell_if_needed(int k)
{
  if (dirty_marks_[static_cast<size_t>(k)] == 0u) {
    dirty_marks_[static_cast<size_t>(k)] = 1u;
    dirty_cell_indices_.push_back(k);
  }
}

void OccupancyGridMap::add_logodds(int row, int col, float delta)
{
  if (!inside_rc(row, col)) {
    return;
  }

  const int k = idx(row, col);
  const size_t ks = static_cast<size_t>(k);

  if (locked_cells_[ks] != 0u) {
    return;
  }

  const int8_t old_occ = occ_grid_[ks];
  map_logodds_[ks] = std::clamp(map_logodds_[ks] + delta, l_min_, l_max_);
  update_occ_grid_cell_from_logodds_index(k);

  if (map_logodds_[ks] >= l_occ_lock_ || map_logodds_[ks] <= l_free_lock_) {
    locked_cells_[ks] = 1u;
  }

  const int8_t new_occ = occ_grid_[ks];
  if (new_occ != old_occ) {
    map_changed_ = true;
    add_dirty_cell_if_needed(k);

    if (old_occ == 0 || new_occ == 0) {
      free_cells_dirty_ = true;
    }
    if (old_occ == 100 || new_occ == 100) {
      likelihood_field_dirty_ = true;
      ++likelihood_dirty_occupied_cells_;
    }
  }
}

void OccupancyGridMap::clear_dirty_tracking()
{
  for (const int k : dirty_cell_indices_) {
    dirty_marks_[static_cast<size_t>(k)] = 0u;
  }
  dirty_cell_indices_.clear();
}

void OccupancyGridMap::rebuild_free_cells_cache()
{
  free_cells_cache_.clear();
  free_cells_cache_.reserve(static_cast<size_t>(width_ * height_ / 8));

  for (int r = 0; r < height_; ++r) {
    for (int c = 0; c < width_; ++c) {
      if (occ_grid_[static_cast<size_t>(idx(r, c))] == 0) {
        free_cells_cache_.emplace_back(r, c);
      }
    }
  }

  free_cells_dirty_ = false;
}

const std::vector<std::pair<int, int>> & OccupancyGridMap::known_free_cells()
{
  if (free_cells_dirty_) {
    rebuild_free_cells_cache();
  }
  return free_cells_cache_;
}

}  // namespace puzzlebot_navigation
