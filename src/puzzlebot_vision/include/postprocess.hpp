// yolov8_postprocess.hpp - Preprocess y postprocess para YOLOv8 detection
//
// Asume:
//   - Input shape [1, 3, S, S] (default S=320)
//   - Output shape [1, 4+num_classes, num_candidates] (formato YOLOv8 raw)
//   - Letterbox padding con valor 114 (estandar YOLO)

#ifndef PUZZLEBOT_INFERENCE__YOLOV8_POSTPROCESS_HPP_
#define PUZZLEBOT_INFERENCE__YOLOV8_POSTPROCESS_HPP_

#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>

namespace puzzlebot_inference
{

struct LetterboxInfo {
  float scale;
  float pad_x;
  float pad_y;
  int orig_w;
  int orig_h;
};

struct Detection {
  cv::Rect2f bbox;   // x, y, w, h en coords de imagen original
  float score;
  int class_id;
};

// Letterbox + BGR->RGB + normalize + HWC->CHW// yolov8_postprocess.hpp - Preprocess y postprocess para YOLOv8 detection
//
// Asume:
//   - Input shape [1, 3, S, S] (default S=320)
//   - Output shape [1, 4+num_classes, num_candidates] (formato YOLOv8 raw)
//   - Letterbox padding con valor 114 (estandar YOLO)

#ifndef PUZZLEBOT_INFERENCE__YOLOV8_POSTPROCESS_HPP_
#define PUZZLEBOT_INFERENCE__YOLOV8_POSTPROCESS_HPP_

#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>

namespace puzzlebot_inference
{

struct LetterboxInfo {
  float scale;
  float pad_x;
  float pad_y;
  int orig_w;
  int orig_h;
};

struct Detection {
  cv::Rect2f bbox;   // x, y, w, h en coords de imagen original
  float score;
  int class_id;
};

// Letterbox + BGR->RGB + normalize + HWC->CHW
// Devuelve vector<float> de tamaño 3*input_size*input_size
inline std::vector<float> preprocess(
  const cv::Mat & img,
  int input_size,
  LetterboxInfo & info)
{
  int orig_w = img.cols;
  int orig_h = img.rows;

  float scale = std::min(
    static_cast<float>(input_size) / orig_w,
    static_cast<float>(input_size) / orig_h);
  int new_w = static_cast<int>(orig_w * scale);
  int new_h = static_cast<int>(orig_h * scale);

  cv::Mat resized;
  cv::resize(img, resized, cv::Size(new_w, new_h));

  int pad_x = (input_size - new_w) / 2;
  int pad_y = (input_size - new_h) / 2;
  cv::Mat letterboxed(input_size, input_size, CV_8UC3, cv::Scalar(114, 114, 114));
  resized.copyTo(letterboxed(cv::Rect(pad_x, pad_y, new_w, new_h)));

  info.scale = scale;
  info.pad_x = pad_x;
  info.pad_y = pad_y;
  info.orig_w = orig_w;
  info.orig_h = orig_h;

  cv::Mat rgb;
  cv::cvtColor(letterboxed, rgb, cv::COLOR_BGR2RGB);

  std::vector<float> output(3 * input_size * input_size);
  int channel_size = input_size * input_size;
  for (int y = 0; y < input_size; ++y) {
    for (int x = 0; x < input_size; ++x) {
      cv::Vec3b pixel = rgb.at<cv::Vec3b>(y, x);
      int idx = y * input_size + x;
      output[0 * channel_size + idx] = pixel[0] / 255.0f;
      output[1 * channel_size + idx] = pixel[1] / 255.0f;
      output[2 * channel_size + idx] = pixel[2] / 255.0f;
    }
  }
  return output;
}

// Decode YOLOv8 output [1, num_features, num_candidates] -> detecciones en coords originales
//   num_features = 4 (bbox) + num_classes
//   Para 1 clase -> num_features = 5
//   Para N clases -> num_features = 4 + N (toma argmax sobre clases)
inline std::vector<Detection> postprocess(
  const float * data,
  int num_candidates,
  int num_features,
  const LetterboxInfo & info,
  float conf_threshold,
  float nms_threshold)
{
  int num_classes = num_features - 4;

  std::vector<cv::Rect2f> boxes;
  std::vector<float> scores;
  std::vector<int> class_ids;

  for (int i = 0; i < num_candidates; ++i) {
    // Encontrar la clase con mayor score
    float best_score = 0.0f;
    int best_class = 0;
    for (int c = 0; c < num_classes; ++c) {
      float s = data[(4 + c) * num_candidates + i];
      if (s > best_score) {
        best_score = s;
        best_class = c;
      }
    }
    if (best_score < conf_threshold) continue;

    float cx = data[0 * num_candidates + i];
    float cy = data[1 * num_candidates + i];
    float w  = data[2 * num_candidates + i];
    float h  = data[3 * num_candidates + i];

    float x = cx - w / 2.0f;
    float y = cy - h / 2.0f;

    // Deshacer letterbox
    x = (x - info.pad_x) / info.scale;
    y = (y - info.pad_y) / info.scale;
    w = w / info.scale;
    h = h / info.scale;

    // Clip
    x = std::max(0.0f, std::min(x, static_cast<float>(info.orig_w - 1)));
    y = std::max(0.0f, std::min(y, static_cast<float>(info.orig_h - 1)));

    boxes.emplace_back(x, y, w, h);
    scores.push_back(best_score);
    class_ids.push_back(best_class);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, scores, conf_threshold, nms_threshold, indices);

  std::vector<Detection> dets;
  dets.reserve(indices.size());
  for (int idx : indices) {
    Detection d;
    d.bbox = boxes[idx];
    d.score = scores[idx];
    d.class_id = class_ids[idx];
    dets.push_back(d);
  }
  return dets;
}

}  // namespace puzzlebot_inference

#endif  // PUZZLEBOT_INFERENCE__YOLOV8_POSTPROCESS_HPP_
// Devuelve vector<float> de tamaño 3*input_size*input_size
inline std::vector<float> preprocess(
  const cv::Mat & img,
  int input_size,
  LetterboxInfo & info)
{
  int orig_w = img.cols;
  int orig_h = img.rows;

  float scale = std::min(
    static_cast<float>(input_size) / orig_w,
    static_cast<float>(input_size) / orig_h);
  int new_w = static_cast<int>(orig_w * scale);
  int new_h = static_cast<int>(orig_h * scale);

  cv::Mat resized;
  cv::resize(img, resized, cv::Size(new_w, new_h));

  float pad_x = (input_size - new_w) / 2.0f;
  float pad_y = (input_size - new_h) / 2.0f;
  cv::Mat letterboxed(input_size, input_size, CV_8UC3, cv::Scalar(114, 114, 114));
  resized.copyTo(letterboxed(cv::Rect(
    static_cast<int>(pad_x), static_cast<int>(pad_y), new_w, new_h)));

  info.scale = scale;
  info.pad_x = pad_x;
  info.pad_y = pad_y;
  info.orig_w = orig_w;
  info.orig_h = orig_h;

  cv::Mat rgb;
  cv::cvtColor(letterboxed, rgb, cv::COLOR_BGR2RGB);

  cv::Mat float_img;
  rgb.convertTo(float_img, CV_32FC3, 1.0f / 255.0f);

  std::vector<cv::Mat> channels(3);
  cv::split(float_img, channels);

  std::vector<float> output(3 * input_size * input_size);
  int channel_size = input_size * input_size;
  std::memcpy(output.data() + 0 * channel_size, channels[0].data, channel_size * sizeof(float));
  std::memcpy(output.data() + 1 * channel_size, channels[1].data, channel_size * sizeof(float));
  std::memcpy(output.data() + 2 * channel_size, channels[2].data, channel_size * sizeof(float));

  return output;
}

// Decode YOLOv8 output [1, num_features, num_candidates] -> detecciones en coords originales
//   num_features = 4 (bbox) + num_classes
//   Para 1 clase -> num_features = 5
//   Para N clases -> num_features = 4 + N (toma argmax sobre clases)
inline std::vector<Detection> postprocess(
  const float * data,
  int num_candidates,
  int num_features,
  const LetterboxInfo & info,
  float conf_threshold,
  float nms_threshold)
{
  int num_classes = num_features - 4;

  std::vector<cv::Rect2d> boxes;
  std::vector<float> scores;
  std::vector<int> class_ids;

  for (int i = 0; i < num_candidates; ++i) {
    float best_score = 0.0f;
    int best_class = 0;
    for (int c = 0; c < num_classes; ++c) {
      float s = data[(4 + c) * num_candidates + i];
      if (s > best_score) {
        best_score = s;
        best_class = c;
      }
    }
    if (best_score < conf_threshold) continue;

    float cx = data[0 * num_candidates + i];
    float cy = data[1 * num_candidates + i];
    float w  = data[2 * num_candidates + i];
    float h  = data[3 * num_candidates + i];

    float x = cx - w / 2.0f;
    float y = cy - h / 2.0f;

    // Deshacer letterbox
    x = (x - info.pad_x) / info.scale;
    y = (y - info.pad_y) / info.scale;
    w = w / info.scale;
    h = h / info.scale;

    // Clip
    x = std::max(0.0f, std::min(x, static_cast<float>(info.orig_w - 1)));
    y = std::max(0.0f, std::min(y, static_cast<float>(info.orig_h - 1)));

    boxes.emplace_back(
      static_cast<double>(x),
      static_cast<double>(y),
      static_cast<double>(w),
      static_cast<double>(h));
    scores.push_back(best_score);
    class_ids.push_back(best_class);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, scores, conf_threshold, nms_threshold, indices);

  std::vector<Detection> dets;
  dets.reserve(indices.size());
  for (int idx : indices) {
    Detection d;
    d.bbox = cv::Rect2f(
      static_cast<float>(boxes[idx].x),
      static_cast<float>(boxes[idx].y),
      static_cast<float>(boxes[idx].width),
      static_cast<float>(boxes[idx].height));
    d.score = scores[idx];
    d.class_id = class_ids[idx];
    dets.push_back(d);
  }
  return dets;
}

}  // namespace puzzlebot_inference

#endif  // PUZZLEBOT_INFERENCE__YOLOV8_POSTPROCESS_HPP_