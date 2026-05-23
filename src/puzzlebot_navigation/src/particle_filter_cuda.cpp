#include "puzzlebot_navigation/particle_filter_cuda.hpp"

#include <stdexcept>

namespace puzzlebot_navigation
{

ParticleFilterCUDA::~ParticleFilterCUDA()
{
  if (initialized_) {
    slam_cuda::free_device_buffers(buffers_);
  }
}

void ParticleFilterCUDA::configure(
  int num_particles,
  int max_scan_points,
  float resample_pos_noise,
  float resample_theta_noise)
{
  N_ = num_particles;
  max_scan_points_ = max_scan_points;
  resample_pos_noise_ = resample_pos_noise;
  resample_theta_noise_ = resample_theta_noise;

  px_.assign(static_cast<size_t>(N_), 0.0f);
  py_.assign(static_cast<size_t>(N_), 0.0f);
  ptheta_.assign(static_cast<size_t>(N_), 0.0f);
  pw_.assign(static_cast<size_t>(N_), 1.0f / static_cast<float>(N_));
}

void ParticleFilterCUDA::initialize(int map_size)
{
  if (!slam_cuda::init_device_buffers(buffers_, N_, max_scan_points_, map_size)) {
    throw std::runtime_error("Could not initialize CUDA buffers");
  }
  initialized_ = true;
}

float ParticleFilterCUDA::randn(std::mt19937 & rng, float mean, float stddev)
{
  std::normal_distribution<float> dist(mean, stddev);
  return dist(rng);
}

void ParticleFilterCUDA::initialize_particles_near(
  float x, float y, float theta,
  float std_xy, float std_theta,
  std::mt19937 & rng)
{
  const float w0 = 1.0f / static_cast<float>(N_);
  for (int i = 0; i < N_; ++i) {
    px_[static_cast<size_t>(i)] = randn(rng, x, std_xy);
    py_[static_cast<size_t>(i)] = randn(rng, y, std_xy);
    ptheta_[static_cast<size_t>(i)] = wrap_angle(randn(rng, theta, std_theta));
    pw_[static_cast<size_t>(i)] = w0;
  }
}

void ParticleFilterCUDA::upload_particles()
{
  if (!slam_cuda::upload_particle_data(buffers_, px_, py_, ptheta_, pw_)) {
    throw std::runtime_error("Could not upload particles to GPU");
  }
}

void ParticleFilterCUDA::upload_initial_map(const OccupancyGridMap & map)
{
  if (!slam_cuda::upload_occ_grid(buffers_, map.occ_grid())) {
    throw std::runtime_error("Could not upload initial occupancy grid");
  }
  map_uploaded_once_ = true;
}

void ParticleFilterCUDA::upload_map_if_needed(OccupancyGridMap & map, float full_map_upload_dirty_ratio)
{
  if (!map_uploaded_once_) {
    upload_initial_map(map);
    map.clear_dirty_tracking();
    return;
  }

  const auto & dirty = map.dirty_indices();
  if (dirty.empty()) {
    return;
  }

  const float dirty_ratio =
    static_cast<float>(dirty.size()) / static_cast<float>(map.occ_grid().size());

  if (dirty_ratio >= full_map_upload_dirty_ratio) {
    if (!slam_cuda::upload_occ_grid(buffers_, map.occ_grid())) {
      throw std::runtime_error("Could not upload full occupancy grid");
    }
  } else {
    if (!slam_cuda::upload_occ_grid_partial(buffers_, dirty, map.occ_grid())) {
      throw std::runtime_error("Could not upload partial occupancy grid");
    }
  }

  map.clear_dirty_tracking();
}

void ParticleFilterCUDA::upload_likelihood_field(const LikelihoodField & lf)
{
  if (!slam_cuda::upload_likelihood_field(buffers_, lf.field())) {
    throw std::runtime_error("Could not upload likelihood field");
  }
}

void ParticleFilterCUDA::upload_scan(const std::vector<float> & scan_x, const std::vector<float> & scan_y, const std::vector<uint8_t> & scan_hit)
{
  if (!slam_cuda::upload_scan_data(buffers_, scan_x, scan_y, scan_hit)) {
    throw std::runtime_error("upload_scan_data failed");
  }
}

void ParticleFilterCUDA::predict(const OdomDelta & delta, const MotionNoise & noise)
{
  cycle_seed_ = cycle_seed_ * 1664525u + 1013904223u;
  if (!slam_cuda::predict_particles_cuda(
        buffers_,
        delta.rot1, delta.trans, delta.rot2,
        noise.rot1, noise.trans, noise.rot2,
        cycle_seed_)) {
    throw std::runtime_error("predict_particles_cuda failed");
  }
}

void ParticleFilterCUDA::score_and_normalize(
  int scan_points,
  const OccupancyGridMap & map,
  const LikelihoodField & lf,
  int mapped_scans,
  int bootstrap_min_mapped_scans)
{
  if (!slam_cuda::compute_particle_scores_cuda(
        buffers_, scan_points,
        map.width(), map.height(), map.x_min(), map.y_min(), map.resolution(),
        lf.sigma(), lf.max_distance())) {
    throw std::runtime_error("compute_particle_scores_cuda failed");
  }

  if (!slam_cuda::normalize_scores_to_weights_cuda(
        buffers_, N_, mapped_scans, bootstrap_min_mapped_scans,
        best_score_, neff_)) {
    throw std::runtime_error("normalize_scores_to_weights_cuda failed");
  }
}

Pose2D ParticleFilterCUDA::estimate_pose()
{
  Pose2D p{};
  if (!slam_cuda::estimate_pose_cuda(buffers_, N_, p)) {
    throw std::runtime_error("estimate_pose_cuda failed");
  }
  return p;
}

void ParticleFilterCUDA::resample()
{
  cycle_seed_ = cycle_seed_ * 1664525u + 1013904223u;
  if (!slam_cuda::resample_particles_cuda(
        buffers_, N_, resample_pos_noise_, resample_theta_noise_,
        cycle_seed_ ^ 0xA5A5A5A5U)) {
    throw std::runtime_error("resample_particles_cuda failed");
  }
}

void ParticleFilterCUDA::download_particles_strided(
  std::vector<float> & px,
  std::vector<float> & py,
  std::vector<float> & ptheta,
  int stride)
{
  if (!slam_cuda::download_particle_data_strided(buffers_, px, py, ptheta, stride)) {
    throw std::runtime_error("download_particle_data_strided failed");
  }
}

}  // namespace puzzlebot_navigation
