// pallet_detector_node.cpp - Nodo ROS2 de detección de pallets con TensorRT
//
// Suscribe a un topic de imagen, corre inferencia YOLOv8 con TRT, y publica:
//   - vision_msgs/Detection2DArray con las bboxes
//   - sensor_msgs/Image con las bboxes dibujadas (debug visualization)

#include <memory>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "puzzlebot_inference_eng/tensor_rt_engine.hpp"
#include "puzzlebot_inference_eng/postprocess.hpp"

using std::placeholders::_1;

namespace puzzlebot_inference
{

class PalletDetector : public rclcpp::Node
{
public:
  PalletDetector()
  : Node("pallet_detection_engine")
  {
    // ---------- Parametros (editables via launch o CLI) ----------
    this->declare_parameter<std::string>(
      "engine_path",
      "/home/puzzlebot/puzz_ws/src/puzzlebot_inference/models/two_sides.engine");
    this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
    this->declare_parameter<std::string>("detections_topic", "/pallet_detections");
    this->declare_parameter<std::string>("debug_image_topic", "/pallet_detections/image");
    this->declare_parameter<float>("conf_threshold", 0.25f);
    this->declare_parameter<float>("nms_threshold", 0.45f);
    this->declare_parameter<int>("input_size", 320);
    this->declare_parameter<bool>("publish_debug_image", true);

    std::string engine_path = this->get_parameter("engine_path").as_string();
    std::string image_topic = this->get_parameter("image_topic").as_string();
    std::string det_topic = this->get_parameter("detections_topic").as_string();
    std::string dbg_topic = this->get_parameter("debug_image_topic").as_string();
    conf_threshold_ = this->get_parameter("conf_threshold").as_double();
    nms_threshold_ = this->get_parameter("nms_threshold").as_double();
    input_size_ = this->get_parameter("input_size").as_int();
    publish_debug_image_ = this->get_parameter("publish_debug_image").as_bool();

    // ---------- Cargar engine TRT ----------
    RCLCPP_INFO(this->get_logger(), "Cargando engine: %s", engine_path.c_str());
    try {
      engine_ = std::make_unique<TrtEngine>(engine_path);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(this->get_logger(), "Fallo al cargar engine: %s", e.what());
      throw;
    }
    RCLCPP_INFO(this->get_logger(), "Engine cargado. Input size=%zu, Output size=%zu",
                engine_->inputSize(), engine_->outputSize());

    // Buffer de salida pre-reservado (evita alocar en cada callback)
    output_buffer_.resize(engine_->outputSize());

    // ---------- Publishers ----------
    detections_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
      det_topic, 10);
    if (publish_debug_image_) {
      debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        dbg_topic, 10);
    }

    // ---------- Subscriber ----------
    // SensorDataQoS = BEST_EFFORT, KEEP_LAST(5) - tipico de camaras
    // Si el callback es mas lento que la camara, ROS2 dropea frames automaticamente
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS(),
      std::bind(&PalletDetector::imageCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "PalletDetector listo. Suscrito a %s",
                image_topic.c_str());
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    auto t_start = std::chrono::high_resolution_clock::now();

    // 1. Convertir ROS Image a cv::Mat
    cv::Mat frame;
    try {
      // toCvShare evita copia si encoding ya es BGR8
      auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
      frame = cv_ptr->image;
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    // 2. Preprocess
    LetterboxInfo lb_info;
    auto input_data = preprocess(frame, input_size_, lb_info);

    // 3. Inferencia TRT
    if (!engine_->infer(input_data.data(), output_buffer_.data())) {
      RCLCPP_ERROR(this->get_logger(), "Inferencia fallo");
      return;
    }

    // 4. Postprocess
    int num_features = engine_->outputDims()[1];   // ej. 5
    int num_candidates = engine_->outputDims()[2]; // ej. 2100
    auto detections = postprocess(
      output_buffer_.data(), num_candidates, num_features,
      lb_info, conf_threshold_, nms_threshold_);

    // 5. Publicar Detection2DArray
    vision_msgs::msg::Detection2DArray det_msg;
    det_msg.header = msg->header;
    for (const auto & d : detections) {
      vision_msgs::msg::Detection2D det;
      det.header = msg->header;
      // Centro y tamaño (vision_msgs usa centro, no esquina)
      det.bbox.center.position.x = d.bbox.x + d.bbox.width / 2.0;
      det.bbox.center.position.y = d.bbox.y + d.bbox.height / 2.0;
      det.bbox.size_x = d.bbox.width;
      det.bbox.size_y = d.bbox.height;

      vision_msgs::msg::ObjectHypothesisWithPose hyp;
      hyp.hypothesis.class_id = std::to_string(d.class_id);
      hyp.hypothesis.score = d.score;
      det.results.push_back(hyp);
      det_msg.detections.push_back(det);
    }
    detections_pub_->publish(det_msg);

    // 6. (Opcional) Publicar imagen con bboxes dibujadas
    if (publish_debug_image_ && debug_image_pub_->get_subscription_count() > 0) {
      cv::Mat debug = frame.clone();
      for (const auto & d : detections) {
        cv::rectangle(debug, d.bbox, cv::Scalar(0, 255, 0), 2);
        char label[64];
        std::snprintf(label, sizeof(label), "pallet %.2f", d.score);
        cv::putText(debug, label,
                    cv::Point(d.bbox.x, d.bbox.y - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 255, 0), 1);
      }
      auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", debug).toImageMsg();
      debug_image_pub_->publish(*debug_msg);
    }

    // 7. Log de performance (cada 30 frames para no saturar)
    auto t_end = std::chrono::high_resolution_clock::now();
    double total_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    frame_count_++;
    if (frame_count_ % 30 == 0) {
      RCLCPP_INFO(this->get_logger(),
                  "Frame %lu: %.1f ms total, %zu detecciones",
                  frame_count_, total_ms, detections.size());
    }
  }

  // Engine + buffer reusable
  std::unique_ptr<TrtEngine> engine_;
  std::vector<float> output_buffer_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;

  // Parametros
  float conf_threshold_;
  float nms_threshold_;
  int input_size_;
  bool publish_debug_image_;

  size_t frame_count_ = 0;
};

}  // namespace puzzlebot_inference

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<puzzlebot_inference::PalletDetector>());
  rclcpp::shutdown();
  return 0;
}