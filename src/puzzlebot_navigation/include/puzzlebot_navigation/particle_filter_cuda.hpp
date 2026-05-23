#pragma once

#include "puzzlebot_navigation/cuda_kernels.hpp"
#include "puzzlebot_navigation/likelihood_field.hpp"
#include "puzzlebot_navigation/occupancy_grid_map.hpp"
#include "puzzlebot_navigation/slam_types.hpp"

#include <cstdint>
#include <random>
#include <vector>

namespace puzzlebot_navigation
{

class ParticleFilterCUDA
{
public:
  ~ParticleFilterCUDA();

  void configure(
    int num_particles,
    int max_scan_points,
    float resample_pos_noise,
    float resample_theta_noise);

  void initialize(int map_size);
  void initialize_particles_near(
    float x, float y, float theta,
    float std_xy, float std_theta,
    std::mt19937 & rng);

  void upload_particles();
  void upload_initial_map(const OccupancyGridMap & map);
  void upload_map_if_needed(OccupancyGridMap & map, float full_map_upload_dirty_ratio);
  void upload_likelihood_field(const LikelihoodField & lf);
  void upload_scan(const std::vector<float> & scan_x, const std::vector<float> & scan_y, const std::vector<uint8_t> & scan_hit);

  void predict(const OdomDelta & delta, const MotionNoise & noise);
  void score_and_normalize(
    int scan_points,
    const OccupancyGridMap & map,
    const LikelihoodField & lf,
    int mapped_scans,
    int bootstrap_min_mapped_scans);

  Pose2D estimate_pose();
  void resample();
  void download_particles_strided(
    std::vector<float> & px,
    std::vector<float> & py,
    std::vector<float> & ptheta,
    int stride);

  int N() const { return N_; }
  float neff() const { return neff_; }
  float best_score() const { return best_score_; }

  std::vector<float> & px() { return px_; }
  std::vector<float> & py() { return py_; }
  std::vector<float> & ptheta() { return ptheta_; }
  std::vector<float> & weights() { return pw_; }

private:
  float randn(std::mt19937 & rng, float mean, float stddev);

  slam_cuda::DeviceBuffers buffers_;
  int N_{1000};
  int max_scan_points_{2048};
  float resample_pos_noise_{0.015f};
  float resample_theta_noise_{0.02f};
  uint32_t cycle_seed_{0x12345678U};
  bool initialized_{false};
  bool map_uploaded_once_{false};

  std::vector<float> px_;
  std::vector<float> py_;
  std::vector<float> ptheta_;
  std::vector<float> pw_;

  float best_score_{-1.0f};
  float neff_{0.0f};
};

}  // namespace puzzlebot_navigation
