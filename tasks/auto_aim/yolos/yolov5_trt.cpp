#include "yolov5_trt.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <vector>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

#ifdef AUTO_AIM_ENABLE_TENSORRT
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#endif

namespace auto_aim {
namespace {
constexpr int kInputHeight = 640;
constexpr int kInputWidth = 640;
constexpr int kInputChannels = 3;
constexpr int kYolov5FeatureSize = 22;

#ifdef AUTO_AIM_ENABLE_TENSORRT

class TRTLogger final : public nvinfer1::ILogger {
public:
  void log(Severity severity, const char *msg) noexcept override {
    if (severity <= Severity::kERROR) {
      tools::logger()->error("[TensorRT] {}", msg);
    } else if (severity == Severity::kWARNING) {
      tools::logger()->warn("[TensorRT] {}", msg);
    } else {
      tools::logger()->debug("[TensorRT] {}", msg);
    }
  }
};

template <typename T> struct TRTDestroy {
  void operator()(T *obj) const {
    if (obj == nullptr) {
      return;
    }
#if NV_TENSORRT_MAJOR >= 10
    delete obj;
#else
    obj->destroy();
#endif
  }
};

inline void check_cuda(cudaError_t code, const std::string &hint) {
  if (code != cudaSuccess) {
    throw std::runtime_error(hint + ": " + cudaGetErrorString(code));
  }
}

template <typename T> class PinnedHostBuffer {
public:
  PinnedHostBuffer() = default;
  ~PinnedHostBuffer() { release(); }

  PinnedHostBuffer(const PinnedHostBuffer &) = delete;
  PinnedHostBuffer &operator=(const PinnedHostBuffer &) = delete;

  PinnedHostBuffer(PinnedHostBuffer &&other) noexcept { swap(other); }

  PinnedHostBuffer &operator=(PinnedHostBuffer &&other) noexcept {
    if (this != &other) {
      release();
      swap(other);
    }
    return *this;
  }

  void resize(size_t count, const std::string &hint) {
    if (count == size_) {
      return;
    }
    release();
    if (count == 0) {
      return;
    }

    void *ptr = nullptr;
    check_cuda(cudaMallocHost(&ptr, count * sizeof(T)), hint);
    data_ = static_cast<T *>(ptr);
    size_ = count;
  }

  void release() {
    if (data_ != nullptr) {
      cudaFreeHost(data_);
      data_ = nullptr;
      size_ = 0;
    }
  }

  T *data() { return data_; }
  const T *data() const { return data_; }
  size_t size() const { return size_; }

private:
  void swap(PinnedHostBuffer &other) noexcept {
    std::swap(data_, other.data_);
    std::swap(size_, other.size_);
  }

  T *data_ = nullptr;
  size_t size_ = 0;
};

inline bool has_dynamic_dim(const nvinfer1::Dims &dims) {
  for (int i = 0; i < dims.nbDims; ++i) {
    if (dims.d[i] < 0) {
      return true;
    }
  }
  return false;
}

inline size_t volume(const nvinfer1::Dims &dims) {
  size_t out = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    if (dims.d[i] <= 0) {
      throw std::runtime_error("TensorRT dims contains invalid values");
    }
    out *= static_cast<size_t>(dims.d[i]);
  }
  return out;
}

#endif

} // namespace

struct YOLOV5TRT::Impl {
#ifdef AUTO_AIM_ENABLE_TENSORRT
  TRTLogger logger;
  std::unique_ptr<nvinfer1::IRuntime, TRTDestroy<nvinfer1::IRuntime>> runtime;
  std::unique_ptr<nvinfer1::ICudaEngine, TRTDestroy<nvinfer1::ICudaEngine>>
      engine;
  std::unique_ptr<nvinfer1::IExecutionContext,
                  TRTDestroy<nvinfer1::IExecutionContext>>
      context;

#if NV_TENSORRT_MAJOR >= 10
  std::string input_name;
  std::string output_name;
#else
  int input_index = -1;
  int output_index = -1;
  std::vector<void *> bindings;
#endif

  void *input_device = nullptr;
  void *output_device = nullptr;
  size_t input_count = 0;
  size_t output_count = 0;
  size_t input_element_size = sizeof(float);
  size_t output_element_size = sizeof(float);
  nvinfer1::DataType input_dtype = nvinfer1::DataType::kFLOAT;
  nvinfer1::DataType output_dtype = nvinfer1::DataType::kFLOAT;

  cudaStream_t stream = nullptr;
  cudaEvent_t h2d_start_event = nullptr;
  cudaEvent_t h2d_end_event = nullptr;
  cudaEvent_t compute_start_event = nullptr;
  cudaEvent_t compute_end_event = nullptr;
  cudaEvent_t d2h_start_event = nullptr;
  cudaEvent_t d2h_end_event = nullptr;
  PinnedHostBuffer<float> host_input_float;
  PinnedHostBuffer<uint16_t> host_input_half;
  PinnedHostBuffer<float> host_output_float;
  std::array<cv::Mat, kInputChannels> input_planes_float;
  std::array<cv::Mat, kInputChannels> input_planes_half;
  cv::Mat rgb;
  cv::Mat rgb_float;
  cv::Mat rgb_half;
  cv::Mat output_cache;

  Impl() = default;

  ~Impl() {
    if (input_device != nullptr) {
      cudaFree(input_device);
      input_device = nullptr;
    }

    if (output_device != nullptr) {
      cudaFree(output_device);
      output_device = nullptr;
    }

    if (stream != nullptr) {
      cudaStreamDestroy(stream);
      stream = nullptr;
    }

    if (h2d_start_event != nullptr) {
      cudaEventDestroy(h2d_start_event);
      h2d_start_event = nullptr;
    }
    if (h2d_end_event != nullptr) {
      cudaEventDestroy(h2d_end_event);
      h2d_end_event = nullptr;
    }
    if (compute_start_event != nullptr) {
      cudaEventDestroy(compute_start_event);
      compute_start_event = nullptr;
    }
    if (compute_end_event != nullptr) {
      cudaEventDestroy(compute_end_event);
      compute_end_event = nullptr;
    }
    if (d2h_start_event != nullptr) {
      cudaEventDestroy(d2h_start_event);
      d2h_start_event = nullptr;
    }
    if (d2h_end_event != nullptr) {
      cudaEventDestroy(d2h_end_event);
      d2h_end_event = nullptr;
    }
  }

  void refresh_input_planes_float() {
    const int channel_area = kInputHeight * kInputWidth;
    for (int i = 0; i < kInputChannels; ++i) {
      input_planes_float[static_cast<size_t>(i)] =
          cv::Mat(kInputHeight, kInputWidth, CV_32F,
                  host_input_float.data() + i * channel_area);
    }
  }

  void refresh_input_planes_half() {
    const int channel_area = kInputHeight * kInputWidth;
    for (int i = 0; i < kInputChannels; ++i) {
      input_planes_half[static_cast<size_t>(i)] =
          cv::Mat(kInputHeight, kInputWidth, CV_16F,
                  host_input_half.data() + i * channel_area);
    }
  }

  static size_t element_size(nvinfer1::DataType dtype) {
    switch (dtype) {
    case nvinfer1::DataType::kFLOAT:
      return sizeof(float);
    case nvinfer1::DataType::kHALF:
      return sizeof(uint16_t);
    default:
      throw std::runtime_error("Unsupported TensorRT tensor dtype");
    }
  }

  void initialize(const std::string &model_path) {
    std::ifstream file(model_path, std::ios::binary);
    if (!file.good()) {
      throw std::runtime_error("Failed to open TensorRT engine: " + model_path);
    }

    file.seekg(0, std::ios::end);
    const auto length = static_cast<size_t>(file.tellg());
    file.seekg(0, std::ios::beg);
    std::vector<char> data(length);
    file.read(data.data(), static_cast<std::streamsize>(length));

    int device_count = 0;
    const auto cuda_status = cudaGetDeviceCount(&device_count);
    if (cuda_status != cudaSuccess) {
      throw std::runtime_error("CUDA initialization failed before TensorRT "
                               "runtime creation: " +
                               std::string(cudaGetErrorString(cuda_status)));
    }
    if (device_count <= 0) {
      throw std::runtime_error(
          "No CUDA device available for TensorRT inference");
    }

    runtime.reset(nvinfer1::createInferRuntime(logger));
    if (!runtime) {
      throw std::runtime_error("Failed to create TensorRT runtime");
    }

    engine.reset(runtime->deserializeCudaEngine(data.data(), data.size()));
    if (!engine) {
      throw std::runtime_error("Failed to deserialize TensorRT engine");
    }

    context.reset(engine->createExecutionContext());
    if (!context) {
      throw std::runtime_error("Failed to create TensorRT execution context");
    }

#if NV_TENSORRT_MAJOR >= 10
    const int nb_io_tensors = engine->getNbIOTensors();
    for (int i = 0; i < nb_io_tensors; ++i) {
      const char *tensor_name = engine->getIOTensorName(i);
      if (tensor_name == nullptr) {
        continue;
      }
      auto mode = engine->getTensorIOMode(tensor_name);
      if (mode == nvinfer1::TensorIOMode::kINPUT) {
        input_name = tensor_name;
      } else if (mode == nvinfer1::TensorIOMode::kOUTPUT) {
        output_name = tensor_name;
      }
    }

    if (input_name.empty() || output_name.empty()) {
      throw std::runtime_error(
          "TensorRT engine must contain one input and one output tensor");
    }

    auto input_dims = engine->getTensorShape(input_name.c_str());
    input_dtype = engine->getTensorDataType(input_name.c_str());
    if (has_dynamic_dim(input_dims)) {
      if (!context->setInputShape(
              input_name.c_str(),
              nvinfer1::Dims4(1, kInputChannels, kInputHeight, kInputWidth))) {
        throw std::runtime_error("Failed to set TensorRT input dims");
      }
      input_dims = context->getTensorShape(input_name.c_str());
    }

    input_count = volume(input_dims);
    input_element_size = element_size(input_dtype);
    if (input_dtype == nvinfer1::DataType::kFLOAT) {
      host_input_float.resize(input_count,
                              "Failed to allocate pinned host input buffer");
      refresh_input_planes_float();
    } else if (input_dtype == nvinfer1::DataType::kHALF) {
      host_input_half.resize(input_count,
                             "Failed to allocate pinned host input buffer");
      refresh_input_planes_half();
    }

    auto output_dims = context->getTensorShape(output_name.c_str());
    output_dtype = engine->getTensorDataType(output_name.c_str());
    if (has_dynamic_dim(output_dims)) {
      throw std::runtime_error(
          "TensorRT output dims are still dynamic after setting input dims");
    }

    output_count = volume(output_dims);
    output_element_size = element_size(output_dtype);
    if (output_dtype != nvinfer1::DataType::kFLOAT) {
      throw std::runtime_error("YOLOV5TRT currently expects FLOAT output, got "
                               "different TensorRT output dtype");
    }
    host_output_float.resize(output_count,
                             "Failed to allocate pinned host output buffer");

#else
    const int nb_bindings = engine->getNbBindings();
    bindings.assign(static_cast<size_t>(nb_bindings), nullptr);
    for (int i = 0; i < nb_bindings; ++i) {
      if (engine->bindingIsInput(i)) {
        input_index = i;
      } else {
        output_index = i;
      }
    }

    if (input_index < 0 || output_index < 0) {
      throw std::runtime_error(
          "TensorRT engine must contain one input and one output binding");
    }

    auto input_dims = engine->getBindingDimensions(input_index);
    input_dtype = engine->getBindingDataType(input_index);
    if (has_dynamic_dim(input_dims)) {
      if (!context->setBindingDimensions(
              input_index,
              nvinfer1::Dims4(1, kInputChannels, kInputHeight, kInputWidth))) {
        throw std::runtime_error("Failed to set TensorRT input dims");
      }
      input_dims = context->getBindingDimensions(input_index);
    }

    input_count = volume(input_dims);
    input_element_size = element_size(input_dtype);
    if (input_dtype == nvinfer1::DataType::kFLOAT) {
      host_input_float.resize(input_count,
                              "Failed to allocate pinned host input buffer");
      refresh_input_planes_float();
    } else if (input_dtype == nvinfer1::DataType::kHALF) {
      host_input_half.resize(input_count,
                             "Failed to allocate pinned host input buffer");
      refresh_input_planes_half();
    }

    auto output_dims = context->getBindingDimensions(output_index);
    output_dtype = engine->getBindingDataType(output_index);
    if (has_dynamic_dim(output_dims)) {
      throw std::runtime_error(
          "TensorRT output dims are still dynamic after setting input dims");
    }

    output_count = volume(output_dims);
    output_element_size = element_size(output_dtype);
    if (output_dtype != nvinfer1::DataType::kFLOAT) {
      throw std::runtime_error("YOLOV5TRT currently expects FLOAT output, got "
                               "different TensorRT output dtype");
    }
    host_output_float.resize(output_count,
                             "Failed to allocate pinned host output buffer");
#endif

    check_cuda(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking),
               "Failed to create CUDA stream");
    check_cuda(cudaEventCreate(&h2d_start_event),
               "Failed to create TensorRT H2D start event");
    check_cuda(cudaEventCreate(&h2d_end_event),
               "Failed to create TensorRT H2D end event");
    check_cuda(cudaEventCreate(&compute_start_event),
               "Failed to create TensorRT compute start event");
    check_cuda(cudaEventCreate(&compute_end_event),
               "Failed to create TensorRT compute end event");
    check_cuda(cudaEventCreate(&d2h_start_event),
               "Failed to create TensorRT D2H start event");
    check_cuda(cudaEventCreate(&d2h_end_event),
               "Failed to create TensorRT D2H end event");
    check_cuda(cudaMalloc(&input_device, input_count * input_element_size),
               "Failed to allocate TensorRT input buffer");
    check_cuda(cudaMalloc(&output_device, output_count * output_element_size),
               "Failed to allocate TensorRT output buffer");

#if NV_TENSORRT_MAJOR < 10
    bindings[input_index] = input_device;
    bindings[output_index] = output_device;
#endif
  }

  cv::Mat infer(const cv::Mat &padded_bgr, YOLOV5TRT::TimingProfile *profile) {
    const auto pack_start = std::chrono::steady_clock::now();
    cv::cvtColor(padded_bgr, rgb, cv::COLOR_BGR2RGB);
    const void *host_input_ptr = nullptr;
    if (input_dtype == nvinfer1::DataType::kFLOAT) {
      rgb.convertTo(rgb_float, CV_32FC3, 1.0 / 255.0);
      cv::split(rgb_float, input_planes_float);
      host_input_ptr = host_input_float.data();
    } else if (input_dtype == nvinfer1::DataType::kHALF) {
      rgb.convertTo(rgb_half, CV_16FC3, 1.0 / 255.0);
      cv::split(rgb_half, input_planes_half);
      host_input_ptr = host_input_half.data();
    } else {
      throw std::runtime_error("Unsupported TensorRT input dtype");
    }
    if (profile != nullptr) {
      profile->input_pack_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - pack_start)
              .count();
    }

#if NV_TENSORRT_MAJOR >= 10
    auto output_dims = context->getTensorShape(output_name.c_str());
#else
    auto output_dims = context->getBindingDimensions(output_index);
#endif

    if (has_dynamic_dim(output_dims)) {
      throw std::runtime_error(
          "TensorRT output dims are dynamic at inference time");
    }

    const auto current_output_count = volume(output_dims);
    if (current_output_count != output_count) {
      if (output_device != nullptr) {
        cudaFree(output_device);
      }
      output_count = current_output_count;
      host_output_float.resize(output_count,
                               "Failed to allocate pinned host output buffer");
      check_cuda(cudaMalloc(&output_device, output_count * output_element_size),
                 "Failed to re-allocate TensorRT output buffer");
#if NV_TENSORRT_MAJOR < 10
      bindings[output_index] = output_device;
#endif
    }

    check_cuda(cudaEventRecord(h2d_start_event, stream),
               "Failed to record TensorRT H2D start event");
    check_cuda(cudaMemcpyAsync(input_device, host_input_ptr,
                               input_count * input_element_size,
                               cudaMemcpyHostToDevice, stream),
               "Failed to copy TensorRT input to device");
    check_cuda(cudaEventRecord(h2d_end_event, stream),
               "Failed to record TensorRT H2D end event");

#if NV_TENSORRT_MAJOR >= 10
    if (!context->setTensorAddress(input_name.c_str(), input_device)) {
      throw std::runtime_error("Failed to set TensorRT input address");
    }
    if (!context->setTensorAddress(output_name.c_str(), output_device)) {
      throw std::runtime_error("Failed to set TensorRT output address");
    }
    check_cuda(cudaEventRecord(compute_start_event, stream),
               "Failed to record TensorRT compute start event");
    if (!context->enqueueV3(stream)) {
      throw std::runtime_error("TensorRT enqueueV3 failed");
    }
    check_cuda(cudaEventRecord(compute_end_event, stream),
               "Failed to record TensorRT compute end event");
#else
    check_cuda(cudaEventRecord(compute_start_event, stream),
               "Failed to record TensorRT compute start event");
    if (!context->enqueueV2(bindings.data(), stream, nullptr)) {
      throw std::runtime_error("TensorRT enqueueV2 failed");
    }
    check_cuda(cudaEventRecord(compute_end_event, stream),
               "Failed to record TensorRT compute end event");
#endif

    check_cuda(cudaEventRecord(d2h_start_event, stream),
               "Failed to record TensorRT D2H start event");
    check_cuda(cudaMemcpyAsync(host_output_float.data(), output_device,
                               output_count * output_element_size,
                               cudaMemcpyDeviceToHost, stream),
               "Failed to copy TensorRT output to host");
    check_cuda(cudaEventRecord(d2h_end_event, stream),
               "Failed to record TensorRT D2H end event");
    check_cuda(cudaStreamSynchronize(stream),
               "Failed to sync TensorRT CUDA stream");

    if (profile != nullptr) {
      float elapsed_ms = 0.0f;
      check_cuda(cudaEventElapsedTime(&elapsed_ms, h2d_start_event, h2d_end_event),
                 "Failed to measure TensorRT H2D time");
      profile->h2d_ms = elapsed_ms;
      check_cuda(cudaEventElapsedTime(&elapsed_ms, compute_start_event,
                                      compute_end_event),
                 "Failed to measure TensorRT GPU compute time");
      profile->gpu_compute_ms = elapsed_ms;
      check_cuda(cudaEventElapsedTime(&elapsed_ms, d2h_start_event, d2h_end_event),
                 "Failed to measure TensorRT D2H time");
      profile->d2h_ms = elapsed_ms;
    }

    std::vector<int> shape;
    shape.reserve(static_cast<size_t>(output_dims.nbDims));
    for (int i = 0; i < output_dims.nbDims; ++i) {
      if (output_dims.d[i] <= 0) {
        throw std::runtime_error(
            "TensorRT output dims contains invalid values");
      }
      shape.emplace_back(output_dims.d[i]);
    }

    if (!shape.empty() && shape.front() == 1) {
      shape.erase(shape.begin());
    }

    int rows = 0;
    int cols = 0;
    bool need_transpose = false;

    if (shape.size() == 2) {
      rows = shape[0];
      cols = shape[1];
      if (rows == kYolov5FeatureSize && cols != kYolov5FeatureSize) {
        need_transpose = true;
      }
    } else if (shape.size() == 1 && shape[0] % kYolov5FeatureSize == 0) {
      rows = shape[0] / kYolov5FeatureSize;
      cols = kYolov5FeatureSize;
    } else {
      throw std::runtime_error("Unexpected TensorRT output rank for YOLOV5");
    }

    if (!need_transpose && cols == kYolov5FeatureSize) {
      output_cache = cv::Mat(rows, cols, CV_32F, host_output_float.data());
      return output_cache;
    }

    if (need_transpose) {
      cv::Mat transposed;
      cv::transpose(cv::Mat(rows, cols, CV_32F, host_output_float.data()),
                    transposed);
      output_cache = transposed;
      return output_cache;
    }

    throw std::runtime_error("Unexpected TensorRT output shape for YOLOV5");
  }
#else
  void initialize(const std::string &) {
    throw std::runtime_error("TensorRT is not enabled. Please install TensorRT "
                             "and reconfigure CMake.");
  }

  cv::Mat infer(const cv::Mat &) {
    throw std::runtime_error("TensorRT is not enabled. Please install TensorRT "
                             "and reconfigure CMake.");
  }
#endif
};

YOLOV5TRT::YOLOV5TRT(const std::string &config_path, bool debug)
    : debug_(debug), detector_(config_path, false),
      impl_(std::make_unique<Impl>()) {
  auto yaml = YAML::LoadFile(config_path);
  std::filesystem::path configured_model_path;

  if (yaml["yolov5_trt_model_path"]) {
    configured_model_path = yaml["yolov5_trt_model_path"].as<std::string>();
  } else if (yaml["yolov5_model_path"] &&
             std::filesystem::path(yaml["yolov5_model_path"].as<std::string>())
                     .extension() == ".engine") {
    configured_model_path = yaml["yolov5_model_path"].as<std::string>();
  } else {
    throw std::runtime_error("Cannot find valid TensorRT engine path in "
                             "config. Please set yolov5_trt_model_path "
                             "(e.g. xxx.engine).");
  }

  if (configured_model_path.extension() != ".engine") {
    throw std::runtime_error("TensorRT model path must point to a .engine "
                             "file, but got: " +
                             configured_model_path.string());
  }

  model_path_ = configured_model_path.string();

  binary_threshold_ = yaml["threshold"].as<double>();
  min_confidence_ = yaml["min_confidence"].as<double>();
  use_roi_ = yaml["use_roi"].as<bool>();
  use_traditional_ = yaml["use_traditional"].as<bool>();

  int x = yaml["roi"]["x"].as<int>();
  int y = yaml["roi"]["y"].as<int>();
  int width = yaml["roi"]["width"].as<int>();
  int height = yaml["roi"]["height"].as<int>();
  roi_ = cv::Rect(x, y, width, height);
  offset_ = cv::Point2f(x, y);

  save_path_ = "imgs";
  std::filesystem::create_directory(save_path_);

  impl_->initialize(model_path_);
  tools::logger()->info("[YOLOV5TRT] loaded TensorRT engine from {}",
                        model_path_);
}

YOLOV5TRT::~YOLOV5TRT() = default;

const YOLOV5TRT::TimingProfile & YOLOV5TRT::last_profile() const
{
  return last_profile_;
}

std::list<Armor> YOLOV5TRT::detect(const cv::Mat &raw_img, int frame_count) {
  last_profile_ = {};
  if (raw_img.empty()) {
    tools::logger()->warn("Empty img!, camera drop!");
    return {};
  }

  const auto total_start = std::chrono::steady_clock::now();
  cv::Mat bgr_img;
  if (use_roi_) {
    if (roi_.width == -1) {
      roi_.width = raw_img.cols;
    }
    if (roi_.height == -1) {
      roi_.height = raw_img.rows;
    }
    bgr_img = raw_img(roi_);
  } else {
    bgr_img = raw_img;
  }

  const auto x_scale = static_cast<double>(kInputHeight) / bgr_img.rows;
  const auto y_scale = static_cast<double>(kInputWidth) / bgr_img.cols;
  const auto scale = std::min(x_scale, y_scale);
  const auto h = static_cast<int>(bgr_img.rows * scale);
  const auto w = static_cast<int>(bgr_img.cols * scale);

  cv::Mat padded =
      cv::Mat(kInputHeight, kInputWidth, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::resize(bgr_img, padded(cv::Rect(0, 0, w, h)), {w, h});
  const auto infer_start = std::chrono::steady_clock::now();
  last_profile_.preprocess_ms =
      std::chrono::duration<double, std::milli>(infer_start - total_start)
          .count();

  cv::Mat output = impl_->infer(padded, &last_profile_);
  const auto postprocess_start = std::chrono::steady_clock::now();
  last_profile_.infer_ms =
      std::chrono::duration<double, std::milli>(postprocess_start - infer_start)
          .count();

  auto armors = parse(scale, output, raw_img, frame_count);
  const auto finish = std::chrono::steady_clock::now();
  last_profile_.postprocess_ms =
      std::chrono::duration<double, std::milli>(finish - postprocess_start)
          .count();
  last_profile_.total_ms =
      std::chrono::duration<double, std::milli>(finish - total_start).count();
  last_profile_.valid = true;
  return armors;
}

std::list<Armor> YOLOV5TRT::parse(double scale, cv::Mat &output,
                                  const cv::Mat &bgr_img, int frame_count) {
  std::vector<int> color_ids;
  std::vector<int> num_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armors_key_points;

  for (int r = 0; r < output.rows; r++) {
    double score = output.at<float>(r, 8);
    score = sigmoid(score);
    if (score < score_threshold_) {
      continue;
    }

    cv::Mat color_scores = output.row(r).colRange(9, 13);
    cv::Mat classes_scores = output.row(r).colRange(13, 22);
    cv::Point class_id;
    cv::Point color_id;
    double score_color;
    double score_num;
    cv::minMaxLoc(classes_scores, nullptr, &score_num, nullptr, &class_id);
    cv::minMaxLoc(color_scores, nullptr, &score_color, nullptr, &color_id);

    std::vector<cv::Point2f> armor_key_points;
    armor_key_points.push_back(cv::Point2f(output.at<float>(r, 0) / scale,
                                           output.at<float>(r, 1) / scale));
    armor_key_points.push_back(cv::Point2f(output.at<float>(r, 6) / scale,
                                           output.at<float>(r, 7) / scale));
    armor_key_points.push_back(cv::Point2f(output.at<float>(r, 4) / scale,
                                           output.at<float>(r, 5) / scale));
    armor_key_points.push_back(cv::Point2f(output.at<float>(r, 2) / scale,
                                           output.at<float>(r, 3) / scale));

    float min_x = armor_key_points[0].x;
    float max_x = armor_key_points[0].x;
    float min_y = armor_key_points[0].y;
    float max_y = armor_key_points[0].y;

    for (size_t i = 1; i < armor_key_points.size(); i++) {
      min_x = std::min(min_x, armor_key_points[i].x);
      max_x = std::max(max_x, armor_key_points[i].x);
      min_y = std::min(min_y, armor_key_points[i].y);
      max_y = std::max(max_y, armor_key_points[i].y);
    }

    color_ids.emplace_back(color_id.x);
    num_ids.emplace_back(class_id.x);
    boxes.emplace_back(static_cast<int>(min_x), static_cast<int>(min_y),
                       static_cast<int>(max_x - min_x),
                       static_cast<int>(max_y - min_y));
    confidences.emplace_back(static_cast<float>(score));
    armors_key_points.emplace_back(std::move(armor_key_points));
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_,
                    indices);

  std::list<Armor> armors;
  for (const auto &i : indices) {
    if (use_roi_) {
      armors.emplace_back(color_ids[i], num_ids[i], confidences[i], boxes[i],
                          armors_key_points[i], offset_);
    } else {
      armors.emplace_back(color_ids[i], num_ids[i], confidences[i], boxes[i],
                          armors_key_points[i]);
    }
  }

  tmp_img_ = bgr_img;
  for (auto it = armors.begin(); it != armors.end();) {
    if (!check_name(*it)) {
      it = armors.erase(it);
      continue;
    }

    if (!check_type(*it)) {
      it = armors.erase(it);
      continue;
    }

    if (use_traditional_) {
      detector_.detect(*it, bgr_img);
    }

    it->center_norm = get_center_norm(bgr_img, it->center);
    ++it;
  }

  if (debug_) {
    draw_detections(bgr_img, armors, frame_count);
  }

  return armors;
}

bool YOLOV5TRT::check_name(const Armor &armor) const {
  const bool name_ok = armor.name != ArmorName::not_armor;
  const bool confidence_ok = armor.confidence > min_confidence_;
  return name_ok && confidence_ok;
}

bool YOLOV5TRT::check_type(const Armor &armor) const {
  const bool name_ok =
      (armor.type == ArmorType::small)
          ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
          : (armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
             armor.name != ArmorName::outpost);
  return name_ok;
}

cv::Point2f YOLOV5TRT::get_center_norm(const cv::Mat &bgr_img,
                                       const cv::Point2f &center) const {
  return {center.x / bgr_img.cols, center.y / bgr_img.rows};
}

void YOLOV5TRT::draw_detections(const cv::Mat &img,
                                const std::list<Armor> &armors,
                                int frame_count) const {
  auto detection = img.clone();
  tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30},
                   {255, 255, 255});
  for (const auto &armor : armors) {
    auto info =
        fmt::format("{:.2f} {} {} {}", armor.confidence, COLORS[armor.color],
                    ARMOR_NAMES[armor.name], ARMOR_TYPES[armor.type]);
    tools::draw_points(detection, armor.points, {0, 255, 0});
    tools::draw_text(detection, info, armor.center, {0, 255, 0});
  }

  if (use_roi_) {
    cv::Scalar green(0, 255, 0);
    cv::rectangle(detection, roi_, green, 2);
  }

  cv::resize(detection, detection, {}, 0.5, 0.5);
  cv::imshow("detection", detection);
}

void YOLOV5TRT::save(const Armor &armor) const {
  auto file_name =
      fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  auto img_path =
      fmt::format("{}/{}_{}.jpg", save_path_, armor.name, file_name);
  cv::imwrite(img_path, tmp_img_);
}

double YOLOV5TRT::sigmoid(double x) {
  if (x > 0) {
    return 1.0 / (1.0 + std::exp(-x));
  }
  return std::exp(x) / (1.0 + std::exp(x));
}

std::list<Armor> YOLOV5TRT::postprocess(double scale, cv::Mat &output,
                                        const cv::Mat &bgr_img,
                                        int frame_count) {
  return parse(scale, output, bgr_img, frame_count);
}

} // namespace auto_aim
