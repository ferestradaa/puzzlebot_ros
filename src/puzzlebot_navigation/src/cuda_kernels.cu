#include "puzzlebot_navigation/cuda_kernels.hpp"

#include <cuda_runtime.h>

#include <cmath>
#include <cfloat>
#include <cstdint>
#include <iostream>
#include <limits>
#include <vector>
#include <algorithm>

namespace slam_cuda
{
namespace
{

// -----------------------------------------------------------------------------
// Verifica el resultado de una llamada CUDA.
// Si ocurre un error, imprime el mensaje recibido junto con la descripción
// del error generada por CUDA y regresa false.
// -----------------------------------------------------------------------------
inline bool check_cuda(cudaError_t err, const char * msg)
{
  if (err != cudaSuccess) {
    std::cerr << msg << ": " << cudaGetErrorString(err) << std::endl;
    return false;
  }
  return true;
}

// -----------------------------------------------------------------------------
// Revisa si el último lanzamiento de kernel produjo un error.
// Esto no sincroniza completamente el dispositivo, pero sí detecta errores
// inmediatos en el launch.
// -----------------------------------------------------------------------------
inline bool check_last_launch(const char * msg)
{
  return check_cuda(cudaGetLastError(), msg);
}

// -----------------------------------------------------------------------------
// Wrapper tipado para cudaMalloc.
// Permite reservar memoria para cualquier tipo T evitando repetir casts.
// -----------------------------------------------------------------------------
template<typename T>
bool cuda_malloc_t(T ** ptr, size_t count, const char * msg)
{
  return check_cuda(cudaMalloc(reinterpret_cast<void **>(ptr), sizeof(T) * count), msg);
}

// -----------------------------------------------------------------------------
// Envuelve un ángulo al intervalo [-pi, pi] dentro del device.
// Esto se usa para que las orientaciones de las partículas no crezcan
// sin control y permanezcan en un rango consistente.
// -----------------------------------------------------------------------------
__device__ inline float wrap_angle_device(float a)
{
  return atan2f(sinf(a), cosf(a));
}

// -----------------------------------------------------------------------------
// Hash sencillo para mezclar bits de una semilla.
// Se usa para derivar semillas distintas por hilo sin depender de estados
// complejos de generadores aleatorios.
// -----------------------------------------------------------------------------
__device__ inline uint32_t hash_u32(uint32_t x)
{
  x ^= x >> 16;
  x *= 0x7feb352dU;
  x ^= x >> 15;
  x *= 0x846ca68bU;
  x ^= x >> 16;
  return x;
}

// -----------------------------------------------------------------------------
// Convierte un entero de 32 bits en un flotante aproximadamente uniforme
// en el rango (0, 1). Se usa como paso intermedio para generar ruido.
// -----------------------------------------------------------------------------
__device__ inline float u01_from_u32(uint32_t x)
{
  return (static_cast<float>(x & 0x00FFFFFFU) + 1.0f) / 16777217.0f;
}

// -----------------------------------------------------------------------------
// Genera una muestra gaussiana usando Box-Muller a partir de dos semillas.
// Cada hilo obtiene así ruido pseudoaleatorio sin mantener un estado RNG
// persistente en GPU.
// -----------------------------------------------------------------------------
__device__ inline float gaussian_from_seed(uint32_t seed_a, uint32_t seed_b)
{
  const float u1 = fmaxf(u01_from_u32(hash_u32(seed_a)), 1e-7f);
  const float u2 = u01_from_u32(hash_u32(seed_b));
  return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * 3.14159265358979323846f * u2);
}

// -----------------------------------------------------------------------------
// KERNEL: predict_particles_kernel
//
// Cada hilo procesa exactamente una partícula.
// Aplica el modelo de movimiento odométrico:
//   rotación 1 -> traslación -> rotación 2
// añadiendo ruido gaussiano independiente a cada componente.
//
// Entradas:
// - px, py, ptheta: estado actual de las partículas
// - delta_rot1, delta_trans, delta_rot2: movimiento observado
// - noise_*_std: desviaciones estándar del ruido
// - seed: semilla global para este ciclo
// - n: número total de partículas
//
// Salida:
// - actualiza in-place px, py y ptheta
// -----------------------------------------------------------------------------
__global__ void predict_particles_kernel(
  float * __restrict__ px,
  float * __restrict__ py,
  float * __restrict__ ptheta,
  float delta_rot1,
  float delta_trans,
  float delta_rot2,
  float noise_rot1_std,
  float noise_trans_std,
  float noise_rot2_std,
  uint32_t seed,
  int n)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) {
    return;
  }

  // Se derivan semillas diferentes por partícula para evitar que todas
  // reciban exactamente el mismo ruido.
  const uint32_t s0 = seed ^ static_cast<uint32_t>(i * 747796405u + 2891336453u);
  const uint32_t s1 = seed ^ static_cast<uint32_t>(i * 277803737u + 1013904223u);
  const uint32_t s2 = seed ^ static_cast<uint32_t>(i * 1664525u + 69069u);

  // Ruido para cada fase del movimiento.
  const float nr1 = noise_rot1_std * gaussian_from_seed(s0, s1);
  const float nt  = noise_trans_std * gaussian_from_seed(s1 + 17u, s2 + 29u);
  const float nr2 = noise_rot2_std * gaussian_from_seed(s2 + 53u, s0 + 97u);

  // Movimiento perturbado para la partícula i.
  const float dr1 = delta_rot1 + nr1;
  const float dt  = delta_trans + nt;
  const float dr2 = delta_rot2 + nr2;

  // Se aplica la actualización de pose.
  const float theta_mid = ptheta[i] + dr1;
  px[i] += dt * cosf(theta_mid);
  py[i] += dt * sinf(theta_mid);
  ptheta[i] = wrap_angle_device(ptheta[i] + dr1 + dr2);
}

// -----------------------------------------------------------------------------
// KERNEL: particle_scores_kernel_tiled
//
// Cada hilo evalúa una partícula completa contra todos los puntos del scan.
// El scan se recorre en "tiles" usando memoria compartida para reducir lecturas
// repetidas desde memoria global.
//
// Lógica:
// 1. Toma la pose base de la partícula i.
// 2. Para cada punto del scan:
//    - lo transforma a coordenadas mundo
//    - lo lleva a coordenadas de celda
//    - consulta el occupancy grid
//    - acumula score según caiga en ocupado/libre/desconocido
//
// Interpretación del score:
// - caer en celda ocupada: evidencia positiva
// - caer en libre: penalización
// - caer en desconocido: pequeña contribución neutra
//
// Salida:
// - scores[i] = score promedio de la partícula i
// -----------------------------------------------------------------------------
__global__ void particle_scores_kernel_tiled(
  const float * __restrict__ px,
  const float * __restrict__ py,
  const float * __restrict__ ptheta,
  const float * __restrict__ scan_x,
  const float * __restrict__ scan_y,
  const uint8_t * __restrict__ scan_hit,
  int num_scan_points,
  const int8_t * __restrict__ occ_grid,
  const float * __restrict__ likelihood_field,
  int width,
  int height,
  float x_min,
  float y_min,
  float res,
  float likelihood_sigma,
  float likelihood_max_distance,
  float * __restrict__ scores,
  int n)
{
  // Shared memory dinámica:
  // Primero se guarda tile_x y luego tile_y.
  extern __shared__ float shared_scan[];
  float * tile_x = shared_scan;
  float * tile_y = shared_scan + blockDim.x;

  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) {
    return;
  }

  const float base_x = px[i];
  const float base_y = py[i];
  const float th = ptheta[i];

  // Verificación rápida: si la partícula ya está fuera del mapa,
  // se le asigna score inválido.
  const int col0 = static_cast<int>(floorf((base_x - x_min) / res));
  const int row0 = static_cast<int>(floorf((base_y - y_min) / res));

  if (row0 < 0 || row0 >= height || col0 < 0 || col0 >= width) {
    scores[i] = -FLT_MAX;
    return;
  }

  const float c = cosf(th);
  const float s = sinf(th);

  float log_score = 0.0f;
  int used = 0;

  // Recorremos el scan en bloques del tamaño blockDim.x.
  for (int tile_start = 0; tile_start < num_scan_points; tile_start += blockDim.x) {
    const int tile_idx = tile_start + threadIdx.x;

    // Cada hilo copia un punto del scan al tile compartido si existe.
    if (tile_idx < num_scan_points) {
      tile_x[threadIdx.x] = scan_x[tile_idx];
      tile_y[threadIdx.x] = scan_y[tile_idx];
    }
    __syncthreads();

    const int tile_count = min(blockDim.x, num_scan_points - tile_start);

    // Una vez cargado el tile, cada hilo recorre todos los puntos del tile
    // usando su propia partícula como hipótesis de pose.
    for (int j = 0; j < tile_count; ++j) {
      const int global_j = tile_start + j;
      if (scan_hit[global_j] == 0u) {
        continue;
      }
      const float wx = base_x + c * tile_x[j] - s * tile_y[j];
      const float wy = base_y + s * tile_x[j] + c * tile_y[j];

      const int col = static_cast<int>(floorf((wx - x_min) / res));
      const int row = static_cast<int>(floorf((wy - y_min) / res));

      // Si el punto cae fuera del mapa, se ignora.
      if (row < 0 || row >= height || col < 0 || col >= width) {
        continue;
      }

      const int idx = row * width + col;
      const float d = fminf(likelihood_field[idx], likelihood_max_distance);

      // Likelihood Field Model:
      // El endpoint no necesita caer exactamente en una celda ocupada.
      // Se premia de forma suave si cae cerca de un obstáculo.
      const float sigma = fmaxf(likelihood_sigma, 0.01f);
      const float d_norm = d / sigma;

      const int8_t cell = occ_grid[idx];

      log_score += -0.5f * d_norm * d_norm;

      if (cell == 0) {
        log_score += -0.5f;
      }

      ++used;
    }
    __syncthreads();
  }

  // Se normaliza por número de puntos válidos usados.
  scores[i] = (used == 0) ? -FLT_MAX : (log_score / static_cast<float>(used));
}

// -----------------------------------------------------------------------------
// KERNEL: set_uniform_weights_kernel
//
// Inicializa todos los pesos al mismo valor.
// Se usa, por ejemplo, durante bootstrap o en fallbacks cuando las
// normalizaciones fallan.
// -----------------------------------------------------------------------------
__global__ void set_uniform_weights_kernel(float * weights, float w, int n)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) {
    weights[i] = w;
  }
}

// -----------------------------------------------------------------------------
// KERNEL: exp_weights_kernel
//
// Convierte score en peso aplicando una exponencial.
// El score máximo se resta antes para mejorar estabilidad numérica.
// -----------------------------------------------------------------------------
__global__ void exp_weights_kernel(
  const float * scores,
  float * weights,
  float max_score,
  int n)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) {
    weights[i] = expf(scores[i] - max_score);
  }
}

// -----------------------------------------------------------------------------
// KERNEL: divide_weights_kernel
//
// Normaliza pesos dividiendo cada elemento entre la suma total.
// -----------------------------------------------------------------------------
__global__ void divide_weights_kernel(
  float * weights,
  float sum_w,
  int n)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) {
    weights[i] /= sum_w;
  }
}

// -----------------------------------------------------------------------------
// KERNEL: square_weights_kernel
//
// Calcula el cuadrado de cada peso. Esto se usa para obtener:
//   neff = 1 / sum(w_i^2)
// -----------------------------------------------------------------------------
__global__ void square_weights_kernel(
  const float * weights,
  float * sq_weights,
  int n)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) {
    const float w = weights[i];
    sq_weights[i] = w * w;
  }
}

// -----------------------------------------------------------------------------
// KERNEL: reduce_max_kernel
//
// Reducción por bloque para máximo.
// Cada bloque produce un valor parcial en out[blockIdx.x].
// Luego la función host vuelve a reducir esos resultados hasta obtener
// un único máximo global.
// -----------------------------------------------------------------------------
__global__ void reduce_max_kernel(const float * data, float * out, int n)
{
  extern __shared__ float sdata[];
  const unsigned int tid = threadIdx.x;
  const unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;

  float x = -FLT_MAX;
  if (i < static_cast<unsigned int>(n)) {
    x = data[i];
  }
  sdata[tid] = x;
  __syncthreads();

  for (unsigned int s = blockDim.x / 2; s > 0; s >>= 1) {
    if (tid < s) {
      sdata[tid] = fmaxf(sdata[tid], sdata[tid + s]);
    }
    __syncthreads();
  }

  if (tid == 0) {
    out[blockIdx.x] = sdata[0];
  }
}

// -----------------------------------------------------------------------------
// KERNEL: reduce_sum_kernel
//
// Igual que el anterior, pero para suma.
// Produce una suma parcial por bloque.
// -----------------------------------------------------------------------------
__global__ void reduce_sum_kernel(const float * data, float * out, int n)
{
  extern __shared__ float sdata[];
  const unsigned int tid = threadIdx.x;
  const unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;

  float x = 0.0f;
  if (i < static_cast<unsigned int>(n)) {
    x = data[i];
  }
  sdata[tid] = x;
  __syncthreads();

  for (unsigned int s = blockDim.x / 2; s > 0; s >>= 1) {
    if (tid < s) {
      sdata[tid] += sdata[tid + s];
    }
    __syncthreads();
  }

  if (tid == 0) {
    out[blockIdx.x] = sdata[0];
  }
}

// -----------------------------------------------------------------------------
// KERNEL: copy_array_kernel
//
// Copia un arreglo de entrada a salida.
// Se usa, por ejemplo, para clonar los pesos al buffer CDF antes del scan.
// -----------------------------------------------------------------------------
__global__ void copy_array_kernel(const float * in, float * out, int n)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) {
    out[i] = in[i];
  }
}

// -----------------------------------------------------------------------------
// KERNEL: inclusive_scan_step_kernel
//
// Ejecuta un paso del scan inclusivo paralelo.
// En cada iteración de offset, out[i] = in[i] + in[i - offset] si aplica.
// Después de varias iteraciones se obtiene la CDF acumulada.
// -----------------------------------------------------------------------------
__global__ void inclusive_scan_step_kernel(
  const float * in,
  float * out,
  int offset,
  int n)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) {
    return;
  }

  float val = in[i];
  if (i >= offset) {
    val += in[i - offset];
  }
  out[i] = val;
}

// -----------------------------------------------------------------------------
// KERNEL: generate_systematic_u_kernel
//
// Genera el arreglo u[i] para remuestreo sistemático:
//   u[i] = r + i / n
// donde r es un desplazamiento aleatorio pequeño en [0, 1/n).
// -----------------------------------------------------------------------------
__global__ void generate_systematic_u_kernel(
  float * u,
  int n,
  uint32_t seed)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) {
    return;
  }

  const float r = u01_from_u32(hash_u32(seed)) / static_cast<float>(n);
  u[i] = r + static_cast<float>(i) / static_cast<float>(n);
}

// -----------------------------------------------------------------------------
// KERNEL: systematic_resample_indices_kernel
//
// Para cada u[i], busca en la CDF el índice de la partícula fuente.
// Usa búsqueda binaria porque la CDF es creciente.
// -----------------------------------------------------------------------------
__global__ void systematic_resample_indices_kernel(
  const float * cdf,
  const float * u,
  int * indices,
  int n)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) {
    return;
  }

  const float target = u[i];
  int left = 0;
  int right = n - 1;

  while (left < right) {
    const int mid = left + (right - left) / 2;
    if (cdf[mid] < target) {
      left = mid + 1;
    } else {
      right = mid;
    }
  }
  indices[i] = left;
}

// -----------------------------------------------------------------------------
// KERNEL: resample_scatter_kernel
//
// Construye el nuevo conjunto de partículas.
// Cada hilo:
// 1. toma el índice fuente seleccionado por el remuestreo
// 2. copia esa partícula
// 3. añade ruido de remuestreo en posición y orientación
// -----------------------------------------------------------------------------
__global__ void resample_scatter_kernel(
  const float * px,
  const float * py,
  const float * ptheta,
  const int * indices,
  float * new_px,
  float * new_py,
  float * new_ptheta,
  float pos_noise_std,
  float theta_noise_std,
  uint32_t seed,
  int n)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) {
    return;
  }

  const int src = indices[i];

  const uint32_t s0 = seed ^ static_cast<uint32_t>(i * 747796405u + 2891336453u);
  const uint32_t s1 = seed ^ static_cast<uint32_t>(i * 277803737u + 1013904223u);
  const uint32_t s2 = seed ^ static_cast<uint32_t>(i * 1664525u + 69069u);

  const float nx = pos_noise_std * gaussian_from_seed(s0, s1);
  const float ny = pos_noise_std * gaussian_from_seed(s1 + 17u, s2 + 29u);
  const float nth = theta_noise_std * gaussian_from_seed(s2 + 53u, s0 + 97u);

  new_px[i] = px[src] + nx;
  new_py[i] = py[src] + ny;
  new_ptheta[i] = wrap_angle_device(ptheta[src] + nth);
}

// -----------------------------------------------------------------------------
// KERNEL: scatter_occ_updates_kernel
//
// Actualiza celdas puntuales del occupancy grid en GPU.
// Cada hilo escribe una celda modificada.
// Esto evita subir el mapa completo cuando sólo cambiaron unas pocas celdas.
// -----------------------------------------------------------------------------
__global__ void scatter_occ_updates_kernel(
  int8_t * occ_grid,
  const int * dirty_indices,
  const int8_t * dirty_values,
  int n)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) {
    occ_grid[dirty_indices[i]] = dirty_values[i];
  }
}

// -----------------------------------------------------------------------------
// KERNEL: accumulate_pose_kernel_blockwise
//
// Calcula acumulados ponderados para estimar la pose:
//   sum(w*x), sum(w*y), sum(w*cos(theta)), sum(w*sin(theta)), sum(w)
//
// Cada bloque reduce localmente en shared memory y luego el hilo 0 del bloque
// suma al acumulador global mediante atomicAdd.
// -----------------------------------------------------------------------------
__global__ void accumulate_pose_kernel_blockwise(
  const float * px,
  const float * py,
  const float * ptheta,
  const float * weights,
  float * pose_accum,
  int n)
{
  __shared__ float sx[256];
  __shared__ float sy[256];
  __shared__ float sc[256];
  __shared__ float ss[256];
  __shared__ float sw[256];

  const int tid = threadIdx.x;
  const int i = blockIdx.x * blockDim.x + tid;

  float lx = 0.0f;
  float ly = 0.0f;
  float lc = 0.0f;
  float ls = 0.0f;
  float lw = 0.0f;

  if (i < n) {
    const float w = weights[i];
    lw = w;
    lx = w * px[i];
    ly = w * py[i];
    lc = w * cosf(ptheta[i]);
    ls = w * sinf(ptheta[i]);
  }

  sx[tid] = lx;
  sy[tid] = ly;
  sc[tid] = lc;
  ss[tid] = ls;
  sw[tid] = lw;
  __syncthreads();

  for (int offset = blockDim.x / 2; offset > 0; offset >>= 1) {
    if (tid < offset) {
      sx[tid] += sx[tid + offset];
      sy[tid] += sy[tid + offset];
      sc[tid] += sc[tid + offset];
      ss[tid] += ss[tid + offset];
      sw[tid] += sw[tid + offset];
    }
    __syncthreads();
  }

  if (tid == 0) {
    atomicAdd(&pose_accum[0], sx[0]);
    atomicAdd(&pose_accum[1], sy[0]);
    atomicAdd(&pose_accum[2], sc[0]);
    atomicAdd(&pose_accum[3], ss[0]);
    atomicAdd(&pose_accum[4], sw[0]);
  }
}

// -----------------------------------------------------------------------------
// KERNEL: gather_strided_kernel
//
// Extrae una versión reducida del arreglo de partículas.
// Si stride = 4, el hilo i copia la partícula fuente i*4.
// Se usa para publicar o visualizar menos partículas.
// -----------------------------------------------------------------------------
__global__ void gather_strided_kernel(
  const float * px,
  const float * py,
  const float * ptheta,
  float * out_px,
  float * out_py,
  float * out_ptheta,
  int n,
  int stride,
  int out_n)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= out_n) {
    return;
  }

  const int src = i * stride;
  if (src < n) {
    out_px[i] = px[src];
    out_py[i] = py[src];
    out_ptheta[i] = ptheta[src];
  }
}

// -----------------------------------------------------------------------------
// Asegura que los buffers temporales de reducción tengan capacidad suficiente.
// Si no alcanzan, los libera y los vuelve a reservar.
// -----------------------------------------------------------------------------
bool ensure_reduce_capacity(DeviceBuffers & buffers, int n)
{
  const int block = 256;
  const int needed = (n + block - 1) / block;
  if (buffers.reduce_tmp_capacity >= needed) {
    return true;
  }

  cudaFree(buffers.d_reduce_tmp1);
  cudaFree(buffers.d_reduce_tmp2);
  buffers.d_reduce_tmp1 = nullptr;
  buffers.d_reduce_tmp2 = nullptr;
  buffers.reduce_tmp_capacity = 0;

  if (!cuda_malloc_t(&buffers.d_reduce_tmp1, static_cast<size_t>(needed), "cudaMalloc d_reduce_tmp1")) {
    return false;
  }
  if (!cuda_malloc_t(&buffers.d_reduce_tmp2, static_cast<size_t>(needed), "cudaMalloc d_reduce_tmp2")) {
    cudaFree(buffers.d_reduce_tmp1);
    buffers.d_reduce_tmp1 = nullptr;
    return false;
  }

  buffers.reduce_tmp_capacity = needed;
  return true;
}

// -----------------------------------------------------------------------------
// reduce_max_device
//
// Ejecuta reducciones sucesivas en GPU hasta obtener un único máximo global.
// El resultado final se copia a CPU en "result".
// -----------------------------------------------------------------------------
bool reduce_max_device(DeviceBuffers & buffers, const float * d_in, int n, float & result)
{
  if (!ensure_reduce_capacity(buffers, n)) {
    return false;
  }

  const int block = 256;
  int curr_n = n;
  const float * d_curr_in = d_in;
  float * d_out = buffers.d_reduce_tmp1;
  bool ping = true;

  while (curr_n > 1) {
    const int grid = (curr_n + block - 1) / block;
    reduce_max_kernel<<<grid, block, block * sizeof(float)>>>(d_curr_in, d_out, curr_n);
    if (!check_last_launch("reduce_max_kernel launch")) {
      return false;
    }

    curr_n = grid;
    d_curr_in = d_out;
    d_out = ping ? buffers.d_reduce_tmp2 : buffers.d_reduce_tmp1;
    ping = !ping;
  }

  return check_cuda(
    cudaMemcpy(&result, d_curr_in, sizeof(float), cudaMemcpyDeviceToHost),
    "copy reduce max result");
}

// -----------------------------------------------------------------------------
// reduce_sum_device
//
// Igual que la anterior, pero para suma global.
// -----------------------------------------------------------------------------
bool reduce_sum_device(DeviceBuffers & buffers, const float * d_in, int n, float & result)
{
  if (!ensure_reduce_capacity(buffers, n)) {
    return false;
  }

  const int block = 256;
  int curr_n = n;
  const float * d_curr_in = d_in;
  float * d_out = buffers.d_reduce_tmp1;
  bool ping = true;

  while (curr_n > 1) {
    const int grid = (curr_n + block - 1) / block;
    reduce_sum_kernel<<<grid, block, block * sizeof(float)>>>(d_curr_in, d_out, curr_n);
    if (!check_last_launch("reduce_sum_kernel launch")) {
      return false;
    }

    curr_n = grid;
    d_curr_in = d_out;
    d_out = ping ? buffers.d_reduce_tmp2 : buffers.d_reduce_tmp1;
    ping = !ping;
  }

  return check_cuda(
    cudaMemcpy(&result, d_curr_in, sizeof(float), cudaMemcpyDeviceToHost),
    "copy reduce sum result");
}

}  // namespace

// -----------------------------------------------------------------------------
// Reserva todos los buffers CUDA necesarios para el pipeline completo:
//
// - partículas
// - scan
// - mapa
// - scores / pesos
// - buffers de reducción
// - buffers de remuestreo
// - acumuladores de pose
// - buffers auxiliares para publicación
// -----------------------------------------------------------------------------
bool init_device_buffers(
  DeviceBuffers & buffers,
  int num_particles,
  int max_scan_points,
  int map_size)
{
  buffers.num_particles = num_particles;
  buffers.max_scan_points = max_scan_points;
  buffers.map_size = map_size;

  return
    cuda_malloc_t(&buffers.d_px, static_cast<size_t>(num_particles), "cudaMalloc d_px") &&
    cuda_malloc_t(&buffers.d_py, static_cast<size_t>(num_particles), "cudaMalloc d_py") &&
    cuda_malloc_t(&buffers.d_ptheta, static_cast<size_t>(num_particles), "cudaMalloc d_ptheta") &&
    cuda_malloc_t(&buffers.d_scan_x, static_cast<size_t>(max_scan_points), "cudaMalloc d_scan_x") &&
    cuda_malloc_t(&buffers.d_scan_y, static_cast<size_t>(max_scan_points), "cudaMalloc d_scan_y") &&
    cuda_malloc_t(&buffers.d_scan_hit, static_cast<size_t>(max_scan_points), "cudaMalloc d_scan_hit") &&
    cuda_malloc_t(&buffers.d_occ_grid, static_cast<size_t>(map_size), "cudaMalloc d_occ_grid") &&
    cuda_malloc_t(&buffers.d_likelihood_field, static_cast<size_t>(map_size), "cudaMalloc d_likelihood_field") &&
    cuda_malloc_t(&buffers.d_scores, static_cast<size_t>(num_particles), "cudaMalloc d_scores") &&
    cuda_malloc_t(&buffers.d_weights, static_cast<size_t>(num_particles), "cudaMalloc d_weights") &&
    cuda_malloc_t(&buffers.d_sq_weights, static_cast<size_t>(num_particles), "cudaMalloc d_sq_weights") &&
    cuda_malloc_t(&buffers.d_resample_cdf, static_cast<size_t>(num_particles), "cudaMalloc d_resample_cdf") &&
    cuda_malloc_t(&buffers.d_resample_u, static_cast<size_t>(num_particles), "cudaMalloc d_resample_u") &&
    cuda_malloc_t(&buffers.d_resample_idx, static_cast<size_t>(num_particles), "cudaMalloc d_resample_idx") &&
    cuda_malloc_t(&buffers.d_new_px, static_cast<size_t>(num_particles), "cudaMalloc d_new_px") &&
    cuda_malloc_t(&buffers.d_new_py, static_cast<size_t>(num_particles), "cudaMalloc d_new_py") &&
    cuda_malloc_t(&buffers.d_new_ptheta, static_cast<size_t>(num_particles), "cudaMalloc d_new_ptheta") &&
    cuda_malloc_t(&buffers.d_pose_accum, 5, "cudaMalloc d_pose_accum") &&
    cuda_malloc_t(&buffers.d_pose_weight_sum, 1, "cudaMalloc d_pose_weight_sum") &&
    cuda_malloc_t(&buffers.d_dirty_indices, static_cast<size_t>(map_size), "cudaMalloc d_dirty_indices") &&
    cuda_malloc_t(&buffers.d_dirty_values, static_cast<size_t>(map_size), "cudaMalloc d_dirty_values") &&
    cuda_malloc_t(&buffers.d_pub_px, static_cast<size_t>(num_particles), "cudaMalloc d_pub_px") &&
    cuda_malloc_t(&buffers.d_pub_py, static_cast<size_t>(num_particles), "cudaMalloc d_pub_py") &&
    cuda_malloc_t(&buffers.d_pub_ptheta, static_cast<size_t>(num_particles), "cudaMalloc d_pub_ptheta") &&
    ensure_reduce_capacity(buffers, num_particles);
}

// -----------------------------------------------------------------------------
// Libera todos los buffers del dispositivo y reinicia la estructura.
// -----------------------------------------------------------------------------
void free_device_buffers(DeviceBuffers & buffers)
{
  cudaFree(buffers.d_px);
  cudaFree(buffers.d_py);
  cudaFree(buffers.d_ptheta);
  cudaFree(buffers.d_scan_x);
  cudaFree(buffers.d_scan_y);
  cudaFree(buffers.d_scan_hit);
  cudaFree(buffers.d_occ_grid);
  cudaFree(buffers.d_likelihood_field);
  cudaFree(buffers.d_scores);
  cudaFree(buffers.d_weights);
  cudaFree(buffers.d_sq_weights);

  cudaFree(buffers.d_reduce_tmp1);
  cudaFree(buffers.d_reduce_tmp2);

  cudaFree(buffers.d_resample_cdf);
  cudaFree(buffers.d_resample_u);
  cudaFree(buffers.d_resample_idx);
  cudaFree(buffers.d_new_px);
  cudaFree(buffers.d_new_py);
  cudaFree(buffers.d_new_ptheta);

  cudaFree(buffers.d_pose_accum);
  cudaFree(buffers.d_pose_weight_sum);

  cudaFree(buffers.d_dirty_indices);
  cudaFree(buffers.d_dirty_values);

  cudaFree(buffers.d_pub_px);
  cudaFree(buffers.d_pub_py);
  cudaFree(buffers.d_pub_ptheta);

  buffers = DeviceBuffers{};
}

// -----------------------------------------------------------------------------
// Copia partículas y pesos desde CPU hacia GPU.
// Valida que todos los vectores tengan tamaño consistente.
// -----------------------------------------------------------------------------
bool upload_particle_data(
  DeviceBuffers & buffers,
  const std::vector<float> & px,
  const std::vector<float> & py,
  const std::vector<float> & ptheta,
  const std::vector<float> & weights)
{
  const int n = static_cast<int>(px.size());
  if (n != buffers.num_particles ||
      static_cast<int>(py.size()) != n ||
      static_cast<int>(ptheta.size()) != n ||
      static_cast<int>(weights.size()) != n)
  {
    std::cerr << "upload_particle_data: tamaños inconsistentes" << std::endl;
    return false;
  }

  const bool ok =
    check_cuda(cudaMemcpy(buffers.d_px, px.data(), sizeof(float) * n, cudaMemcpyHostToDevice), "memcpy px") &&
    check_cuda(cudaMemcpy(buffers.d_py, py.data(), sizeof(float) * n, cudaMemcpyHostToDevice), "memcpy py") &&
    check_cuda(cudaMemcpy(buffers.d_ptheta, ptheta.data(), sizeof(float) * n, cudaMemcpyHostToDevice), "memcpy ptheta") &&
    check_cuda(cudaMemcpy(buffers.d_weights, weights.data(), sizeof(float) * n, cudaMemcpyHostToDevice), "memcpy weights");

  if (ok) {
    buffers.particles_on_device = true;
  }
  return ok;
}

// -----------------------------------------------------------------------------
// Copia el scan en formato de puntos (x, y) a GPU.
// Valida que no exceda la capacidad máxima reservada.
// -----------------------------------------------------------------------------
bool upload_scan_data(
  DeviceBuffers & buffers,
  const std::vector<float> & scan_x,
  const std::vector<float> & scan_y,
  const std::vector<uint8_t> & scan_hit)
{
  const int n = static_cast<int>(scan_x.size());
  if (n > buffers.max_scan_points ||
      static_cast<int>(scan_y.size()) != n ||
      static_cast<int>(scan_hit.size()) != n) {
    std::cerr << "upload_scan_data: tamaño inválido" << std::endl;
    return false;
  }

  return
    check_cuda(cudaMemcpy(buffers.d_scan_x, scan_x.data(), sizeof(float) * n, cudaMemcpyHostToDevice), "memcpy scan_x") &&
    check_cuda(cudaMemcpy(buffers.d_scan_y, scan_y.data(), sizeof(float) * n, cudaMemcpyHostToDevice), "memcpy scan_y") &&
    check_cuda(cudaMemcpy(buffers.d_scan_hit, scan_hit.data(), sizeof(uint8_t) * n, cudaMemcpyHostToDevice), "memcpy scan_hit");
}

// -----------------------------------------------------------------------------
// Sube el occupancy grid completo a GPU.
// Se usa normalmente cuando el mapa inicializa o cuando hay demasiados cambios
// como para justificar actualización parcial.
// -----------------------------------------------------------------------------
bool upload_occ_grid(
  DeviceBuffers & buffers,
  const std::vector<int8_t> & occ_grid)
{
  if (static_cast<int>(occ_grid.size()) != buffers.map_size) {
    std::cerr << "upload_occ_grid: tamaño inválido" << std::endl;
    return false;
  }

  const bool ok = check_cuda(
    cudaMemcpy(buffers.d_occ_grid, occ_grid.data(), sizeof(int8_t) * occ_grid.size(), cudaMemcpyHostToDevice),
    "memcpy occ_grid");

  if (ok) {
    buffers.map_on_device = true;
  }
  return ok;
}

// -----------------------------------------------------------------------------
// Sube el likelihood field completo a GPU.
// -----------------------------------------------------------------------------
bool upload_likelihood_field(
  DeviceBuffers & buffers,
  const std::vector<float> & likelihood_field)
{
  if (static_cast<int>(likelihood_field.size()) != buffers.map_size) {
    std::cerr << "upload_likelihood_field: tamaño inválido" << std::endl;
    return false;
  }

  return check_cuda(
    cudaMemcpy(
      buffers.d_likelihood_field,
      likelihood_field.data(),
      sizeof(float) * likelihood_field.size(),
      cudaMemcpyHostToDevice),
    "memcpy likelihood_field");
}

// -----------------------------------------------------------------------------
// Actualiza solo celdas modificadas del mapa.
// Flujo:
// 1. recibe los índices modificados
// 2. construye en CPU el vector dirty_values
// 3. sube índices y valores
// 4. lanza un kernel scatter que actualiza únicamente esas posiciones
// -----------------------------------------------------------------------------
bool upload_occ_grid_partial(
  DeviceBuffers & buffers,
  const std::vector<int> & dirty_indices,
  const std::vector<int8_t> & occ_grid_host)
{
  if (dirty_indices.empty()) {
    return true;
  }

  if (static_cast<int>(occ_grid_host.size()) != buffers.map_size) {
    std::cerr << "upload_occ_grid_partial: mapa host inválido" << std::endl;
    return false;
  }

  if (static_cast<int>(dirty_indices.size()) > buffers.map_size) {
    std::cerr << "upload_occ_grid_partial: demasiados índices" << std::endl;
    return false;
  }

  std::vector<int8_t> dirty_values(dirty_indices.size());
  for (size_t i = 0; i < dirty_indices.size(); ++i) {
    const int k = dirty_indices[i];
    if (k < 0 || k >= buffers.map_size) {
      std::cerr << "upload_occ_grid_partial: índice fuera de rango" << std::endl;
      return false;
    }
    dirty_values[i] = occ_grid_host[static_cast<size_t>(k)];
  }

  const int n = static_cast<int>(dirty_indices.size());
  if (!check_cuda(
      cudaMemcpy(buffers.d_dirty_indices, dirty_indices.data(), sizeof(int) * n, cudaMemcpyHostToDevice),
      "memcpy dirty_indices")) {
    return false;
  }

  if (!check_cuda(
      cudaMemcpy(buffers.d_dirty_values, dirty_values.data(), sizeof(int8_t) * n, cudaMemcpyHostToDevice),
      "memcpy dirty_values")) {
    return false;
  }

  const int block = 256;
  const int grid = (n + block - 1) / block;
  scatter_occ_updates_kernel<<<grid, block>>>(buffers.d_occ_grid, buffers.d_dirty_indices, buffers.d_dirty_values, n);

  if (!check_last_launch("scatter_occ_updates_kernel launch")) {
    return false;
  }

  return check_cuda(cudaStreamSynchronize(0), "scatter_occ_updates_kernel sync");
}

// -----------------------------------------------------------------------------
// Aplica el modelo de movimiento a todas las partículas.
// Esta función host solo configura el launch del kernel de predicción.
// -----------------------------------------------------------------------------
bool predict_particles_cuda(
  DeviceBuffers & buffers,
  float delta_rot1,
  float delta_trans,
  float delta_rot2,
  float noise_rot1_std,
  float noise_trans_std,
  float noise_rot2_std,
  uint32_t seed)
{
  const int n = buffers.num_particles;
  const int block = 256;
  const int grid = (n + block - 1) / block;

  predict_particles_kernel<<<grid, block>>>(
    buffers.d_px, buffers.d_py, buffers.d_ptheta,
    delta_rot1, delta_trans, delta_rot2,
    noise_rot1_std, noise_trans_std, noise_rot2_std,
    seed, n);

  return check_last_launch("predict_particles_kernel launch");
}

// -----------------------------------------------------------------------------
// Calcula scores de todas las partículas.
// Aquí se define tamaño de bloque, grid y shared memory requerida
// para el kernel tiled del scan.
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
  float likelihood_max_distance)
{
  const int n = buffers.num_particles;
  const int block = 128;
  const int grid = (n + block - 1) / block;

  // Se reservan 2 * block floats en shared memory:
  // uno para tile_x y otro para tile_y.
  const size_t shmem = static_cast<size_t>(2 * block) * sizeof(float);

  particle_scores_kernel_tiled<<<grid, block, shmem>>>(
    buffers.d_px, buffers.d_py, buffers.d_ptheta,
    buffers.d_scan_x, buffers.d_scan_y, buffers.d_scan_hit,
    num_scan_points,
    buffers.d_occ_grid, buffers.d_likelihood_field,
    width, height, x_min, y_min, res,
    likelihood_sigma, likelihood_max_distance,
    buffers.d_scores, n);

  return check_last_launch("particle_scores_kernel_tiled launch");
}

// -----------------------------------------------------------------------------
// Convierte scores a pesos normalizados y calcula best_score y neff.
//
// Caso 1: bootstrap
// - no se confía aún en el mapa
// - se asignan pesos uniformes
//
// Caso 2: operación normal
// - se obtiene max(score)
// - se aplica exp(score - max)
// - se normaliza
// - se calcula neff
// -----------------------------------------------------------------------------
bool normalize_scores_to_weights_cuda(
  DeviceBuffers & buffers,
  int num_particles,
  int mapped_scans,
  int bootstrap_min_scans,
  float & best_score_out,
  float & neff_out)
{
  const int block = 256;
  const int grid = (num_particles + block - 1) / block;

  if (mapped_scans < bootstrap_min_scans) {
    const float w = 1.0f / static_cast<float>(num_particles);
    set_uniform_weights_kernel<<<grid, block>>>(buffers.d_weights, w, num_particles);
    if (!check_last_launch("set_uniform_weights_kernel launch")) {
      return false;
    }
    best_score_out = 0.0f;
    neff_out = static_cast<float>(num_particles);
    return check_cuda(cudaStreamSynchronize(0), "uniform weights sync");
  }

  float max_s = -std::numeric_limits<float>::infinity();
  if (!reduce_max_device(buffers, buffers.d_scores, num_particles, max_s)) {
    return false;
  }
  best_score_out = max_s;

  exp_weights_kernel<<<grid, block>>>(buffers.d_scores, buffers.d_weights, max_s, num_particles);
  if (!check_last_launch("exp_weights_kernel launch")) {
    return false;
  }

  float sum_w = 0.0f;
  if (!reduce_sum_device(buffers, buffers.d_weights, num_particles, sum_w)) {
    return false;
  }

  // Si algo salió mal numéricamente y la suma es demasiado pequeña,
  // se hace fallback a pesos uniformes.
  if (sum_w <= 1e-12f) {
    const float w = 1.0f / static_cast<float>(num_particles);
    set_uniform_weights_kernel<<<grid, block>>>(buffers.d_weights, w, num_particles);
    if (!check_last_launch("fallback uniform weights launch")) {
      return false;
    }
    neff_out = static_cast<float>(num_particles);
    return check_cuda(cudaStreamSynchronize(0), "fallback uniform weights sync");
  }

  divide_weights_kernel<<<grid, block>>>(buffers.d_weights, sum_w, num_particles);
  if (!check_last_launch("divide_weights_kernel launch")) {
    return false;
  }

  square_weights_kernel<<<grid, block>>>(buffers.d_weights, buffers.d_sq_weights, num_particles);
  if (!check_last_launch("square_weights_kernel launch")) {
    return false;
  }

  float sum_sq = 0.0f;
  if (!reduce_sum_device(buffers, buffers.d_sq_weights, num_particles, sum_sq)) {
    return false;
  }

  neff_out = (sum_sq <= 1e-12f) ? 0.0f : (1.0f / sum_sq);
  return true;
}

// -----------------------------------------------------------------------------
// Remuestreo sistemático completamente en GPU.
//
// Flujo:
// 1. se verifica suma de pesos
// 2. se normalizan pesos si hace falta
// 3. se copia weights -> cdf
// 4. se construye scan inclusivo para la CDF
// 5. se generan muestras sistemáticas u[i]
// 6. se buscan índices fuente por búsqueda binaria en la CDF
// 7. se generan nuevas partículas con ruido
// 8. se hace swap de buffers nuevos <-> actuales
// 9. se reinician pesos uniformes
// -----------------------------------------------------------------------------
bool resample_particles_cuda(
  DeviceBuffers & buffers,
  int n,
  float pos_noise_std,
  float theta_noise_std,
  uint32_t seed)
{
  if (n <= 0 || n != buffers.num_particles) {
    std::cerr << "resample_particles_cuda: n inválido" << std::endl;
    return false;
  }

  float sum_w = 0.0f;
  if (!reduce_sum_device(buffers, buffers.d_weights, n, sum_w)) {
    return false;
  }

  const int block = 256;
  const int grid = (n + block - 1) / block;

  if (sum_w <= 1e-12f) {
    const float w0 = 1.0f / static_cast<float>(n);
    set_uniform_weights_kernel<<<grid, block>>>(buffers.d_weights, w0, n);
    if (!check_last_launch("uniform fallback resample launch")) {
      return false;
    }
    return check_cuda(cudaStreamSynchronize(0), "uniform fallback resample sync");
  }

  divide_weights_kernel<<<grid, block>>>(buffers.d_weights, sum_w, n);
  if (!check_last_launch("normalize weights before resample launch")) {
    return false;
  }

  copy_array_kernel<<<grid, block>>>(buffers.d_weights, buffers.d_resample_cdf, n);
  if (!check_last_launch("copy_array_kernel launch")) {
    return false;
  }

  // Se reutiliza uno de los buffers temporales persistentes como salida
  // del scan paralelo.
  bool ping = true;
  float * scan_in = buffers.d_resample_cdf;
  float * scan_out = buffers.d_reduce_tmp1;

  for (int offset = 1; offset < n; offset <<= 1) {
    inclusive_scan_step_kernel<<<grid, block>>>(scan_in, scan_out, offset, n);
    if (!check_last_launch("inclusive_scan_step_kernel launch")) {
      return false;
    }
    std::swap(scan_in, scan_out);
    ping = !ping;
  }

  // Si el resultado final del scan quedó en el buffer temporal,
  // se copia de regreso a d_resample_cdf.
  if (scan_in != buffers.d_resample_cdf) {
    copy_array_kernel<<<grid, block>>>(scan_in, buffers.d_resample_cdf, n);
    if (!check_last_launch("copy final cdf launch")) {
      return false;
    }
  }

  generate_systematic_u_kernel<<<grid, block>>>(buffers.d_resample_u, n, seed ^ 0x9e3779b9U);
  if (!check_last_launch("generate_systematic_u_kernel launch")) {
    return false;
  }

  systematic_resample_indices_kernel<<<grid, block>>>(
    buffers.d_resample_cdf,
    buffers.d_resample_u,
    buffers.d_resample_idx,
    n);
  if (!check_last_launch("systematic_resample_indices_kernel launch")) {
    return false;
  }

  resample_scatter_kernel<<<grid, block>>>(
    buffers.d_px,
    buffers.d_py,
    buffers.d_ptheta,
    buffers.d_resample_idx,
    buffers.d_new_px,
    buffers.d_new_py,
    buffers.d_new_ptheta,
    pos_noise_std,
    theta_noise_std,
    seed ^ 0x85ebca6bU,
    n);
  if (!check_last_launch("resample_scatter_kernel launch")) {
    return false;
  }

  // Las nuevas partículas pasan a ser el conjunto principal.
  std::swap(buffers.d_px, buffers.d_new_px);
  std::swap(buffers.d_py, buffers.d_new_py);
  std::swap(buffers.d_ptheta, buffers.d_new_ptheta);

  // Después del remuestreo se suelen reiniciar los pesos a uniforme.
  const float w0 = 1.0f / static_cast<float>(n);
  set_uniform_weights_kernel<<<grid, block>>>(buffers.d_weights, w0, n);
  if (!check_last_launch("set_uniform_weights after resample launch")) {
    return false;
  }

  return true;
}

// -----------------------------------------------------------------------------
// Estima la pose ponderada del conjunto de partículas.
//
// La GPU acumula:
// - sum(w*x)
// - sum(w*y)
// - sum(w*cos(theta))
// - sum(w*sin(theta))
// - sum(w)
//
// Luego CPU divide por sum(w) y recupera theta con atan2.
// -----------------------------------------------------------------------------
bool estimate_pose_cuda(
  DeviceBuffers & buffers,
  int n,
  Pose2D & pose_out)
{
  if (n <= 0 || n != buffers.num_particles) {
    std::cerr << "estimate_pose_cuda: n inválido" << std::endl;
    return false;
  }

  if (!check_cuda(cudaMemset(buffers.d_pose_accum, 0, sizeof(float) * 5), "memset d_pose_accum")) {
    return false;
  }

  const int block = 256;
  const int grid = (n + block - 1) / block;

  accumulate_pose_kernel_blockwise<<<grid, block>>>(
    buffers.d_px,
    buffers.d_py,
    buffers.d_ptheta,
    buffers.d_weights,
    buffers.d_pose_accum,
    n);

  if (!check_last_launch("accumulate_pose_kernel_blockwise launch")) {
    return false;
  }

  float accum[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  if (!check_cuda(
      cudaMemcpy(accum, buffers.d_pose_accum, sizeof(float) * 5, cudaMemcpyDeviceToHost),
      "download pose accum")) {
    return false;
  }

  const float wsum = accum[4];
  if (wsum <= 1e-12f) {
    pose_out = Pose2D{};
    return true;
  }

  pose_out.x = accum[0] / wsum;
  pose_out.y = accum[1] / wsum;
  pose_out.theta = std::atan2(accum[3] / wsum, accum[2] / wsum);
  return true;
}

// -----------------------------------------------------------------------------
// Descarga partículas con salto fijo (stride).
// Muy útil cuando solo se quiere publicar una muestra de las partículas.
// -----------------------------------------------------------------------------
bool download_particle_data_strided(
  DeviceBuffers & buffers,
  std::vector<float> & px,
  std::vector<float> & py,
  std::vector<float> & ptheta,
  int stride)
{
  if (stride <= 0) {
    std::cerr << "download_particle_data_strided: stride inválido" << std::endl;
    return false;
  }

  const int n = buffers.num_particles;
  const int out_n = (n + stride - 1) / stride;
  const int block = 256;
  const int grid = (out_n + block - 1) / block;

  gather_strided_kernel<<<grid, block>>>(
    buffers.d_px,
    buffers.d_py,
    buffers.d_ptheta,
    buffers.d_pub_px,
    buffers.d_pub_py,
    buffers.d_pub_ptheta,
    n,
    stride,
    out_n);

  if (!check_last_launch("gather_strided_kernel launch")) {
    return false;
  }

  px.resize(static_cast<size_t>(out_n));
  py.resize(static_cast<size_t>(out_n));
  ptheta.resize(static_cast<size_t>(out_n));

  return
    check_cuda(cudaMemcpy(px.data(), buffers.d_pub_px, sizeof(float) * out_n, cudaMemcpyDeviceToHost), "download strided px") &&
    check_cuda(cudaMemcpy(py.data(), buffers.d_pub_py, sizeof(float) * out_n, cudaMemcpyDeviceToHost), "download strided py") &&
    check_cuda(cudaMemcpy(ptheta.data(), buffers.d_pub_ptheta, sizeof(float) * out_n, cudaMemcpyDeviceToHost), "download strided ptheta");
}

}  // namespace slam_cuda
namespace slam_cuda
{
namespace
{

__global__ void loop_candidate_scores_kernel(
  const Pose2D * __restrict__ candidate_poses,
  int num_candidates,
  const float * __restrict__ scan_x,
  const float * __restrict__ scan_y,
  const uint8_t * __restrict__ scan_hit,
  int num_scan_points,
  const float * __restrict__ likelihood_field,
  int width,
  int height,
  float x_min,
  float y_min,
  float res,
  float likelihood_sigma,
  float likelihood_max_distance,
  float * __restrict__ scores,
  int * __restrict__ used_points)
{
  extern __shared__ unsigned char shared_raw[];
  float * shared_scores = reinterpret_cast<float *>(shared_raw);
  int * shared_used = reinterpret_cast<int *>(shared_scores + blockDim.x);

  const int candidate_id = blockIdx.x;
  const int tid = threadIdx.x;

  if (candidate_id >= num_candidates) {
    return;
  }

  const Pose2D pose = candidate_poses[candidate_id];
  const float c = cosf(pose.theta);
  const float s = sinf(pose.theta);
  const float sigma = fmaxf(likelihood_sigma, 0.01f);
  const float inv_two_sigma2 = 1.0f / (2.0f * sigma * sigma);

  float local_score = 0.0f;
  int local_used = 0;

  for (int i = tid; i < num_scan_points; i += blockDim.x) {
    if (scan_hit != nullptr && scan_hit[i] == 0u) {
      continue;
    }

    const float wx = pose.x + c * scan_x[i] - s * scan_y[i];
    const float wy = pose.y + s * scan_x[i] + c * scan_y[i];

    const int col = static_cast<int>(floorf((wx - x_min) / res));
    const int row = static_cast<int>(floorf((wy - y_min) / res));

    if (row < 0 || row >= height || col < 0 || col >= width) {
      continue;
    }

    const int k = row * width + col;
    const float d = fminf(likelihood_field[k], likelihood_max_distance);
    local_score += expf(-(d * d) * inv_two_sigma2);
    ++local_used;
  }

  shared_scores[tid] = local_score;
  shared_used[tid] = local_used;
  __syncthreads();

  for (unsigned int stride = blockDim.x / 2; stride > 0; stride >>= 1) {
    if (tid < stride) {
      shared_scores[tid] += shared_scores[tid + stride];
      shared_used[tid] += shared_used[tid + stride];
    }
    __syncthreads();
  }

  if (tid == 0) {
    used_points[candidate_id] = shared_used[0];
    scores[candidate_id] =
      (shared_used[0] < 20) ? 0.0f : (shared_scores[0] / static_cast<float>(shared_used[0]));
  }
}

}  // namespace

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
  std::vector<int> & used_points_out)
{
  const int num_candidates = static_cast<int>(candidate_poses.size());
  const int num_scan_points = static_cast<int>(scan_x.size());

  scores_out.assign(static_cast<size_t>(num_candidates), 0.0f);
  used_points_out.assign(static_cast<size_t>(num_candidates), 0);

  if (num_candidates <= 0) {
    return true;
  }

  if (num_scan_points <= 0 ||
      static_cast<int>(scan_y.size()) != num_scan_points ||
      static_cast<int>(scan_hit.size()) != num_scan_points ||
      static_cast<int>(likelihood_field.size()) != width * height ||
      width <= 0 || height <= 0 || res <= 0.0f)
  {
    std::cerr << "score_loop_candidates_cuda: entradas inválidas" << std::endl;
    return false;
  }

  Pose2D * d_candidate_poses = nullptr;
  float * d_scan_x = nullptr;
  float * d_scan_y = nullptr;
  uint8_t * d_scan_hit = nullptr;
  float * d_likelihood_field = nullptr;
  float * d_scores = nullptr;
  int * d_used_points = nullptr;

  const bool alloc_ok =
    cuda_malloc_t(&d_candidate_poses, static_cast<size_t>(num_candidates), "cudaMalloc loop candidate poses") &&
    cuda_malloc_t(&d_scan_x, static_cast<size_t>(num_scan_points), "cudaMalloc loop scan_x") &&
    cuda_malloc_t(&d_scan_y, static_cast<size_t>(num_scan_points), "cudaMalloc loop scan_y") &&
    cuda_malloc_t(&d_scan_hit, static_cast<size_t>(num_scan_points), "cudaMalloc loop scan_hit") &&
    cuda_malloc_t(&d_likelihood_field, static_cast<size_t>(width * height), "cudaMalloc loop likelihood") &&
    cuda_malloc_t(&d_scores, static_cast<size_t>(num_candidates), "cudaMalloc loop scores") &&
    cuda_malloc_t(&d_used_points, static_cast<size_t>(num_candidates), "cudaMalloc loop used");

  if (!alloc_ok) {
    cudaFree(d_candidate_poses);
    cudaFree(d_scan_x);
    cudaFree(d_scan_y);
    cudaFree(d_scan_hit);
    cudaFree(d_likelihood_field);
    cudaFree(d_scores);
    cudaFree(d_used_points);
    return false;
  }

  const bool copy_ok =
    check_cuda(cudaMemcpy(d_candidate_poses, candidate_poses.data(), sizeof(Pose2D) * num_candidates, cudaMemcpyHostToDevice), "memcpy loop candidate poses") &&
    check_cuda(cudaMemcpy(d_scan_x, scan_x.data(), sizeof(float) * num_scan_points, cudaMemcpyHostToDevice), "memcpy loop scan_x") &&
    check_cuda(cudaMemcpy(d_scan_y, scan_y.data(), sizeof(float) * num_scan_points, cudaMemcpyHostToDevice), "memcpy loop scan_y") &&
    check_cuda(cudaMemcpy(d_scan_hit, scan_hit.data(), sizeof(uint8_t) * num_scan_points, cudaMemcpyHostToDevice), "memcpy loop scan_hit") &&
    check_cuda(cudaMemcpy(d_likelihood_field, likelihood_field.data(), sizeof(float) * static_cast<size_t>(width * height), cudaMemcpyHostToDevice), "memcpy loop likelihood");

  if (!copy_ok) {
    cudaFree(d_candidate_poses);
    cudaFree(d_scan_x);
    cudaFree(d_scan_y);
    cudaFree(d_scan_hit);
    cudaFree(d_likelihood_field);
    cudaFree(d_scores);
    cudaFree(d_used_points);
    return false;
  }

  constexpr int threads = 256;
  const dim3 blocks(num_candidates);
  const size_t shared_bytes = sizeof(float) * threads + sizeof(int) * threads;

  loop_candidate_scores_kernel<<<blocks, threads, shared_bytes>>>(
    d_candidate_poses,
    num_candidates,
    d_scan_x,
    d_scan_y,
    d_scan_hit,
    num_scan_points,
    d_likelihood_field,
    width,
    height,
    x_min,
    y_min,
    res,
    likelihood_sigma,
    likelihood_max_distance,
    d_scores,
    d_used_points);

  const bool kernel_ok = check_last_launch("loop_candidate_scores_kernel") &&
    check_cuda(cudaDeviceSynchronize(), "sync loop_candidate_scores_kernel");

  const bool download_ok = kernel_ok &&
    check_cuda(cudaMemcpy(scores_out.data(), d_scores, sizeof(float) * num_candidates, cudaMemcpyDeviceToHost), "memcpy loop scores back") &&
    check_cuda(cudaMemcpy(used_points_out.data(), d_used_points, sizeof(int) * num_candidates, cudaMemcpyDeviceToHost), "memcpy loop used back");

  cudaFree(d_candidate_poses);
  cudaFree(d_scan_x);
  cudaFree(d_scan_y);
  cudaFree(d_scan_hit);
  cudaFree(d_likelihood_field);
  cudaFree(d_scores);
  cudaFree(d_used_points);

  return download_ok;
}

}  // namespace slam_cuda
