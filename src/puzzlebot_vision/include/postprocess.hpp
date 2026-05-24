// yolov8_postprocess.hpp - Preprocess y postprocess for YOLOv8 detection
//
// Asumes:
//   - Input shape [1, 3, S, S] (default S=320)
//   - Output shape [1, 4+num_classes, num_candidates]
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
  cv::Rect2f bbox;
  float score;
  int class_id;
};

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

inline std::vector<Detection> postprocess(
  const float * data,
  int num_candidates,
  int num_features,
  const LetterboxInfo & info,
  float conf_threshold,
  float nms_threshold,
  int max_detections = -1)
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

    x = (x - info.pad_x) / info.scale;
    y = (y - info.pad_y) / info.scale;
    w = w / info.scale;
    h = h / info.scale;

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

  if (max_detections > 0 && indices.size() > static_cast<size_t>(max_detections)) {
    std::partial_sort(indices.begin(), indices.begin() + max_detections, indices.end(),
      [&scores](int a, int b) { return scores[a] > scores[b]; });
    indices.resize(max_detections);
  }

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

}

#endif