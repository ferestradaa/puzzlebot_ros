#include "puzzlebot_navigation/likelihood_field.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace puzzlebot_navigation
{

void LikelihoodField::configure(float sigma, float max_distance, int rebuild_period, int min_dirty_occupied_cells)
{
  sigma_ = sigma;
  max_distance_ = max_distance;
  rebuild_period_ = std::max(1, rebuild_period);
  min_dirty_occupied_cells_ = std::max(1, min_dirty_occupied_cells);
}

void LikelihoodField::initialize(const OccupancyGridMap & map)
{
  field_.assign(static_cast<size_t>(map.size()), max_distance_);
  rebuild(map);
  dirty_ = false;
  rebuild_counter_ = 0;
}

bool LikelihoodField::upload_rebuilt_from_map_if_needed(OccupancyGridMap & map, bool force)
{
  if (!map.likelihood_dirty() && !dirty_) {
    return false;
  }

  ++rebuild_counter_;
  const bool enough_cycles = rebuild_counter_ >= rebuild_period_;
  const bool enough_changes = map.likelihood_dirty_occupied_cells() >= min_dirty_occupied_cells_;

  if (!force && !enough_cycles && !enough_changes) {
    dirty_ = true;
    return false;
  }

  rebuild(map);
  dirty_ = false;
  rebuild_counter_ = 0;
  map.set_likelihood_dirty(false);
  map.reset_likelihood_dirty_counter();
  return true;
}

void LikelihoodField::force_rebuild_from_map(OccupancyGridMap & map)
{
  rebuild(map);
  dirty_ = false;
  rebuild_counter_ = 0;
  map.set_likelihood_dirty(false);
  map.reset_likelihood_dirty_counter();
}

namespace
{

inline float edt_intersection(float f_q, float f_vk, int q, int vk)
{
  return ((f_q + static_cast<float>(q * q)) -
    (f_vk + static_cast<float>(vk * vk))) / static_cast<float>(2 * q - 2 * vk);
}

void distance_transform_1d(const std::vector<float> & f, int n, std::vector<float> & d)
{
  constexpr float kInf = 1.0e20f;

  bool has_site = false;
  for (int i = 0; i < n; ++i) {
    if (f[static_cast<size_t>(i)] < kInf * 0.5f) {
      has_site = true;
      break;
    }
  }
  if (!has_site) {
    std::fill(d.begin(), d.begin() + n, kInf);
    return;
  }

  std::vector<int> v(static_cast<size_t>(n), 0);
  std::vector<float> z(static_cast<size_t>(n) + 1u, 0.0f);

  int k = 0;
  v[0] = 0;
  z[0] = -kInf;
  z[1] = kInf;

  for (int q = 1; q < n; ++q) {
    float s = edt_intersection(f[static_cast<size_t>(q)], f[static_cast<size_t>(v[static_cast<size_t>(k)])], q, v[static_cast<size_t>(k)]);
    while (s <= z[static_cast<size_t>(k)]) {
      --k;
      s = edt_intersection(f[static_cast<size_t>(q)], f[static_cast<size_t>(v[static_cast<size_t>(k)])], q, v[static_cast<size_t>(k)]);
    }
    ++k;
    v[static_cast<size_t>(k)] = q;
    z[static_cast<size_t>(k)] = s;
    z[static_cast<size_t>(k + 1)] = kInf;
  }

  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[static_cast<size_t>(k + 1)] < static_cast<float>(q)) {
      ++k;
    }
    const int diff = q - v[static_cast<size_t>(k)];
    d[static_cast<size_t>(q)] =
      static_cast<float>(diff * diff) + f[static_cast<size_t>(v[static_cast<size_t>(k)])];
  }
}

}  // namespace

void LikelihoodField::rebuild(const OccupancyGridMap & map)
{
  const int width = map.width();
  const int height = map.height();
  const int total = width * height;

  if (width <= 0 || height <= 0 || total <= 0) {
    field_.clear();
    return;
  }

  constexpr float kInf = 1.0e20f;
  const auto & occ = map.occ_grid();

  std::vector<float> binary(static_cast<size_t>(total), kInf);
  bool has_occupied = false;

  for (int k = 0; k < total; ++k) {
    if (occ[static_cast<size_t>(k)] == 100) {
      binary[static_cast<size_t>(k)] = 0.0f;
      has_occupied = true;
    }
  }

  field_.assign(static_cast<size_t>(total), max_distance_);
  if (!has_occupied) {
    return;
  }

  std::vector<float> tmp(static_cast<size_t>(total), kInf);
  std::vector<float> f(static_cast<size_t>(std::max(width, height)), kInf);
  std::vector<float> d(static_cast<size_t>(std::max(width, height)), kInf);

  // Pass 1: vertical 1D squared distance transform for every column.
  for (int c = 0; c < width; ++c) {
    for (int r = 0; r < height; ++r) {
      f[static_cast<size_t>(r)] = binary[static_cast<size_t>(r * width + c)];
    }
    distance_transform_1d(f, height, d);
    for (int r = 0; r < height; ++r) {
      tmp[static_cast<size_t>(r * width + c)] = d[static_cast<size_t>(r)];
    }
  }

  // Pass 2: horizontal 1D transform. Result is squared Euclidean distance in cells.
  for (int r = 0; r < height; ++r) {
    for (int c = 0; c < width; ++c) {
      f[static_cast<size_t>(c)] = tmp[static_cast<size_t>(r * width + c)];
    }
    distance_transform_1d(f, width, d);
    for (int c = 0; c < width; ++c) {
      const float dist_cells = std::sqrt(std::max(0.0f, d[static_cast<size_t>(c)]));
      field_[static_cast<size_t>(r * width + c)] =
        std::min(max_distance_, dist_cells * map.resolution());
    }
  }
}

}  // namespace puzzlebot_navigation