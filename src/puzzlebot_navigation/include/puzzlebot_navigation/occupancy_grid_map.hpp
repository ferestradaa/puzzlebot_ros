#pragma once

#include <cstdint>
#include <utility>
#include <vector>

namespace puzzlebot_navigation
{

class OccupancyGridMap
{
public:
  void configure(
    float x_min, float x_max, float y_min, float y_max, float resolution,
    float l_occ, float l_free, float l_min, float l_max,
    float l_occ_lock, float l_free_lock);

  void initialize();
  void reset(bool clear_locks = true);

  int width() const { return width_; }
  int height() const { return height_; }
  float x_min() const { return x_min_; }
  float y_min() const { return y_min_; }
  float resolution() const { return res_; }
  int size() const { return width_ * height_; }

  int idx(int row, int col) const { return row * width_ + col; }
  bool inside_rc(int row, int col) const;
  std::pair<int, int> world_to_grid(float x, float y) const;
  std::pair<float, float> grid_to_world(int row, int col) const;

  int8_t occ_value_from_logodds(float l) const;
  void add_logodds(int row, int col, float delta);

  void clear_dirty_tracking();
  void mark_all_dirty();
  const std::vector<int> & dirty_indices() const { return dirty_cell_indices_; }
  const std::vector<int8_t> & occ_grid() const { return occ_grid_; }
  const std::vector<float> & logodds() const { return map_logodds_; }

  const std::vector<std::pair<int, int>> & known_free_cells();
  void mark_free_cells_dirty() { free_cells_dirty_ = true; }

  bool map_changed() const { return map_changed_; }
  void set_map_changed(bool value) { map_changed_ = value; }

  bool likelihood_dirty() const { return likelihood_field_dirty_; }
  void set_likelihood_dirty(bool v) { likelihood_field_dirty_ = v; }
  int likelihood_dirty_occupied_cells() const { return likelihood_dirty_occupied_cells_; }
  void reset_likelihood_dirty_counter() { likelihood_dirty_occupied_cells_ = 0; }

  float l_occ() const { return l_occ_; }
  float l_free() const { return l_free_; }

private:
  void update_occ_grid_cell_from_logodds_index(int k);
  void add_dirty_cell_if_needed(int k);
  void rebuild_free_cells_cache();

  float x_min_{-6.0f};
  float x_max_{6.0f};
  float y_min_{-6.0f};
  float y_max_{6.0f};
  float res_{0.05f};
  int width_{0};
  int height_{0};

  float l_occ_{1.0f};
  float l_free_{-0.1f};
  float l_min_{-6.0f};
  float l_max_{12.0f};
  float l_occ_lock_{11.5f};
  float l_free_lock_{-5.5f};

  static constexpr float LOGODDS_FREE_THRESH = -1.0986123f;
  static constexpr float LOGODDS_OCC_THRESH = 2.1972246f;

  std::vector<float> map_logodds_;
  std::vector<int8_t> occ_grid_;
  std::vector<uint8_t> locked_cells_;

  std::vector<int> dirty_cell_indices_;
  std::vector<uint8_t> dirty_marks_;

  std::vector<std::pair<int, int>> free_cells_cache_;
  bool free_cells_dirty_{true};

  bool map_changed_{true};
  bool likelihood_field_dirty_{true};
  int likelihood_dirty_occupied_cells_{0};
};

}  // namespace puzzlebot_navigation
