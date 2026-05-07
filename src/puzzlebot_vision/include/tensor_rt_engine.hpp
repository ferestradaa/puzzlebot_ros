// trt_engine.hpp - Wrapper RAII de TensorRT para inferencia
//
// Encapsula deserializacion del .engine, manejo de memoria GPU, y ejecucion.
// Diseño:
//   - Una instancia por modelo (ej. uno para two_sides, otro para pallet_complete)
//   - Asume modelo de entrada/salida unica (1 input binding, 1 output binding)
//   - Mantiene buffers GPU pre-allocados durante toda la vida del objeto

#ifndef PUZZLEBOT_INFERENCE__TRT_ENGINE_HPP_
#define PUZZLEBOT_INFERENCE__TRT_ENGINE_HPP_

#include <string>
#include <vector>
#include <memory>
#include <fstream>
#include <iostream>

#include <cuda_runtime_api.h>
#include <NvInfer.h>

namespace puzzlebot_inference
{

// Logger minimalista para TRT
class TrtLogger : public nvinfer1::ILogger {
public:
  void log(Severity severity, const char* msg) noexcept override {
    if (severity <= Severity::kWARNING) {
      std::cout << "[TRT] " << msg << std::endl;
    }
  }
};

class TrtEngine {
public:
  // Carga el engine desde disco y reserva los buffers GPU
  // Throws std::runtime_error si algo falla
  explicit TrtEngine(const std::string & engine_path);

  ~TrtEngine();

  // No copiable (maneja recursos GPU)
  TrtEngine(const TrtEngine &) = delete;
  TrtEngine & operator=(const TrtEngine &) = delete;

  // Ejecuta inferencia: copia input a GPU, corre, copia output a host
  // input_data debe tener tamaño input_size_ (ya en formato CHW float32 normalizado)
  // output_data se llena con tamaño output_size_
  // Returns false si la inferencia falla
  bool infer(const float * input_data, float * output_data);

  // Getters de shapes (para que el postprocess sepa cuantos candidates hay, etc.)
  const std::vector<int> & inputDims() const { return input_dims_; }
  const std::vector<int> & outputDims() const { return output_dims_; }
  size_t inputSize() const { return input_size_; }
  size_t outputSize() const { return output_size_; }

private:
  TrtLogger logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;

  cudaStream_t stream_;
  void * d_input_ = nullptr;
  void * d_output_ = nullptr;

  int input_idx_ = -1;
  int output_idx_ = -1;
  std::vector<int> input_dims_;
  std::vector<int> output_dims_;
  size_t input_size_ = 0;   // numero de elementos float
  size_t output_size_ = 0;
};

// =============================================================================
// Implementacion (header-only para simplificar build)
// =============================================================================

inline TrtEngine::TrtEngine(const std::string & engine_path)
{
  // 1. Leer engine de disco
  std::ifstream file(engine_path, std::ios::binary | std::ios::ate);
  if (!file.is_open()) {
    throw std::runtime_error("No se pudo abrir engine: " + engine_path);
  }
  std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);
  std::vector<char> engine_data(size);
  file.read(engine_data.data(), size);

  // 2. Crear runtime y deserializar
  runtime_.reset(nvinfer1::createInferRuntime(logger_));
  if (!runtime_) {
    throw std::runtime_error("Fallo al crear IRuntime");
  }
  engine_.reset(runtime_->deserializeCudaEngine(engine_data.data(), engine_data.size()));
  if (!engine_) {
    throw std::runtime_error("Fallo al deserializar engine");
  }
  context_.reset(engine_->createExecutionContext());
  if (!context_) {
    throw std::runtime_error("Fallo al crear ExecutionContext");
  }

  // 3. Identificar input/output bindings y calcular tamaños
  int num_bindings = engine_->getNbBindings();
  for (int i = 0; i < num_bindings; ++i) {
    auto dims = engine_->getBindingDimensions(i);
    std::vector<int> shape;
    size_t elements = 1;
    for (int d = 0; d < dims.nbDims; ++d) {
      shape.push_back(dims.d[d]);
      elements *= dims.d[d];
    }
    if (engine_->bindingIsInput(i)) {
      input_idx_ = i;
      input_dims_ = shape;
      input_size_ = elements;
      std::cout << "[TrtEngine] Input  '" << engine_->getBindingName(i) << "' [";
    } else {
      output_idx_ = i;
      output_dims_ = shape;
      output_size_ = elements;
      std::cout << "[TrtEngine] Output '" << engine_->getBindingName(i) << "' [";
    }
    for (size_t d = 0; d < shape.size(); ++d) {
      std::cout << shape[d] << (d + 1 < shape.size() ? "x" : "");
    }
    std::cout << "]" << std::endl;
  }
  if (input_idx_ < 0 || output_idx_ < 0) {
    throw std::runtime_error("Engine no tiene exactamente 1 input + 1 output");
  }

  // 4. Reservar memoria GPU
  cudaError_t err;
  err = cudaMalloc(&d_input_, input_size_ * sizeof(float));
  if (err != cudaSuccess) {
    throw std::runtime_error(std::string("cudaMalloc input: ") + cudaGetErrorString(err));
  }
  err = cudaMalloc(&d_output_, output_size_ * sizeof(float));
  if (err != cudaSuccess) {
    throw std::runtime_error(std::string("cudaMalloc output: ") + cudaGetErrorString(err));
  }
  err = cudaStreamCreate(&stream_);
  if (err != cudaSuccess) {
    throw std::runtime_error(std::string("cudaStreamCreate: ") + cudaGetErrorString(err));
  }
}

inline TrtEngine::~TrtEngine()
{
  if (stream_) cudaStreamDestroy(stream_);
  if (d_input_) cudaFree(d_input_);
  if (d_output_) cudaFree(d_output_);
}

inline bool TrtEngine::infer(const float * input_data, float * output_data)
{
  void * bindings[] = {d_input_, d_output_};

  cudaError_t err;
  err = cudaMemcpyAsync(d_input_, input_data, input_size_ * sizeof(float),
                        cudaMemcpyHostToDevice, stream_);
  if (err != cudaSuccess) return false;

  if (!context_->enqueueV2(bindings, stream_, nullptr)) return false;

  err = cudaMemcpyAsync(output_data, d_output_, output_size_ * sizeof(float),
                        cudaMemcpyDeviceToHost, stream_);
  if (err != cudaSuccess) return false;

  err = cudaStreamSynchronize(stream_);
  return err == cudaSuccess;
}

}  // namespace puzzlebot_inference

#endif  // PUZZLEBOT_INFERENCE__TRT_ENGINE_HPP_