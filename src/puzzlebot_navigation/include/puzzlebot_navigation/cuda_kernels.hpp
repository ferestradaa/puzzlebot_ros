#pragma once

#include <cstdint>
#include <vector>

namespace slam_cuda
{

// -----------------------------------------------------------------------------
// Pose 2D básica utilizada para representar posición y orientación.
// -----------------------------------------------------------------------------
struct Pose2D
{
  float x{0.0f};
  float y{0.0f};
  float theta{0.0f};
};

// -----------------------------------------------------------------------------
// Estructura contenedora de todos los buffers en GPU.
//
// La idea es centralizar aquí toda la memoria del dispositivo para evitar pasar
// demasiados punteros entre funciones y facilitar la administración del estado
// CUDA del filtro.
// -----------------------------------------------------------------------------
struct DeviceBuffers
{
  // Partículas en GPU
  float * d_px{nullptr};
  float * d_py{nullptr};
  float * d_ptheta{nullptr};

  // Scan en formato de puntos (x, y) en marco robot
  float * d_scan_x{nullptr};
  float * d_scan_y{nullptr};
  uint8_t * d_scan_hit{nullptr};

  // Occupancy grid en GPU
  int8_t * d_occ_grid{nullptr};

  // Likelihood field: distancia en metros al obstáculo más cercano por celda.
  float * d_likelihood_field{nullptr};

  // Scores y pesos de partículas
  float * d_scores{nullptr};
  float * d_weights{nullptr};
  float * d_sq_weights{nullptr};

  // Buffers persistentes para reducciones
  float * d_reduce_tmp1{nullptr};
  float * d_reduce_tmp2{nullptr};
  int reduce_tmp_capacity{0};

  // Buffers para remuestreo completamente en GPU
  float * d_resample_cdf{nullptr};
  float * d_resample_u{nullptr};
  int * d_resample_idx{nullptr};

  float * d_new_px{nullptr};
  float * d_new_py{nullptr};
  float * d_new_ptheta{nullptr};

  // Buffers para estimación de pose
  // d_pose_accum = [sum(wx), sum(wy), sum(wcos), sum(wsin), sum(w)]
  float * d_pose_accum{nullptr};
  float * d_pose_weight_sum{nullptr};

  // Buffers para actualización parcial del mapa
  int * d_dirty_indices{nullptr};
  int8_t * d_dirty_values{nullptr};

  // Buffers auxiliares para descarga estriada de partículas
  float * d_pub_px{nullptr};
  float * d_pub_py{nullptr};
  float * d_pub_ptheta{nullptr};

  // Metadatos
  int num_particles{0};
  int max_scan_points{0};
  int map_size{0};

  bool particles_on_device{false};
  bool map_on_device{false};
};

// -----------------------------------------------------------------------------
// Reserva todos los buffers necesarios en GPU.
// -----------------------------------------------------------------------------
bool init_device_buffers(
  DeviceBuffers & buffers,
  int num_particles,
  int max_scan_points,
  int map_size);

// -----------------------------------------------------------------------------
// Libera todos los buffers asociados.
// -----------------------------------------------------------------------------
void free_device_buffers(DeviceBuffers & buffers);

// -----------------------------------------------------------------------------
// Sube partículas y pesos desde CPU a GPU.
// -----------------------------------------------------------------------------
bool upload_particle_data(
  DeviceBuffers & buffers,
  const std::vector<float> & px,
  const std::vector<float> & py,
  const std::vector<float> & ptheta,
  const std::vector<float> & weights);

// -----------------------------------------------------------------------------
// Sube el scan ya convertido a puntos (x, y).
// -----------------------------------------------------------------------------
bool upload_scan_data(
  DeviceBuffers & buffers,
  const std::vector<float> & scan_x,
  const std::vector<float> & scan_y,
  const std::vector<uint8_t> & scan_hit);

// -----------------------------------------------------------------------------
// Sube el occupancy grid completo.
// -----------------------------------------------------------------------------
bool upload_occ_grid(
  DeviceBuffers & buffers,
  const std::vector<int8_t> & occ_grid);

// -----------------------------------------------------------------------------
// Sube el likelihood field completo. Cada celda contiene distancia en metros al
// obstáculo más cercano.
// -----------------------------------------------------------------------------
bool upload_likelihood_field(
  DeviceBuffers & buffers,
  const std::vector<float> & likelihood_field);

// -----------------------------------------------------------------------------
// Sube únicamente las celdas modificadas del mapa.
// dirty_indices contiene los índices lineales de las celdas actualizadas.
// occ_grid_host contiene el mapa completo en CPU.
// -----------------------------------------------------------------------------
bool upload_occ_grid_partial(
  DeviceBuffers & buffers,
  const std::vector<int> & dirty_indices,
  const std::vector<int8_t> & occ_grid_host);

// -----------------------------------------------------------------------------
// Aplica el modelo de movimiento a todas las partículas en GPU.
// -----------------------------------------------------------------------------
bool predict_particles_cuda(
  DeviceBuffers & buffers,
  float delta_rot1,
  float delta_trans,
  float delta_rot2,
  float noise_rot1_std,
  float noise_trans_std,
  float noise_rot2_std,
  uint32_t seed);

// -----------------------------------------------------------------------------
// Calcula el score de cada partícula contra el mapa usando el scan actual.
// -----------------------------------------------------------------------------
bool compute_particle_scores_cuda(
  DeviceBuffers & buffers,
  int num_scan_points,
  int width,
  int height,
  float x_min,
  float y_min,
  float res,
  float likelihood_sigma,
  float likelihood_max_distance);

// -----------------------------------------------------------------------------
// Convierte scores en pesos normalizados y calcula:
// - best_score_out: mejor score
// - neff_out: effective sample size
// -----------------------------------------------------------------------------
bool normalize_scores_to_weights_cuda(
  DeviceBuffers & buffers,
  int num_particles,
  int mapped_scans,
  int bootstrap_min_scans,
  float & best_score_out,
  float & neff_out);

// -----------------------------------------------------------------------------
// Ejecuta remuestreo sistemático completo en GPU.
// -----------------------------------------------------------------------------
bool resample_particles_cuda(
  DeviceBuffers & buffers,
  int n,
  float pos_noise_std,
  float theta_noise_std,
  uint32_t seed);

// -----------------------------------------------------------------------------
// Estima la pose ponderada a partir de las partículas actuales.
// -----------------------------------------------------------------------------
bool estimate_pose_cuda(
  DeviceBuffers & buffers,
  int n,
  Pose2D & pose_out);

// -----------------------------------------------------------------------------
// Descarga una versión estriada de las partículas.
// Ejemplo: stride = 4 descarga una de cada cuatro.
// -----------------------------------------------------------------------------
bool download_particle_data_strided(
  DeviceBuffers & buffers,
  std::vector<float> & px,
  std::vector<float> & py,
  std::vector<float> & ptheta,
  int stride);

// -----------------------------------------------------------------------------
// Evalúa en GPU los candidatos de loop closure de GraphSLAM.
// Cada bloque procesa una pose candidata y reduce el score de sus puntos.
// Devuelve un score promedio por candidato y cuántos puntos válidos se usaron.
// -----------------------------------------------------------------------------
bool score_loop_candidates_cuda(
  const std::vector<Pose2D> & candidate_poses,
  const std::vector<float> & scan_x,
  const std::vector<float> & scan_y,
  const std::vector<uint8_t> & scan_hit,
  const std::vector<float> & likelihood_field,
  int width,
  int height,
  float x_min,
  float y_min,
  float res,
  float likelihood_sigma,
  float likelihood_max_distance,
  std::vector<float> & scores_out,
  std::vector<int> & used_points_out);

}  // namespace slam_cuda
