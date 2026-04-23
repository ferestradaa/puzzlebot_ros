#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <random>
#include <chrono>

class CameraNoiseNode : public rclcpp::Node
{
public:
  CameraNoiseNode()
  : Node("camera_noise"),
    gen_(static_cast<unsigned>(
      std::chrono::steady_clock::now().time_since_epoch().count()))
  {
    this->declare_parameter("gaussian_mean",    0.0);
    this->declare_parameter("gaussian_stddev", 0.0);
    this->declare_parameter("salt_pepper_prob", 0.002);
    this->declare_parameter("enable_blur",      true);
    this->declare_parameter("blur_kernel_size", 3);
    this->declare_parameter("input_topic",  "/camera/image_sim_raw");
    this->declare_parameter("output_topic", "/camera/image_raw");

    gaussian_mean_    = this->get_parameter("gaussian_mean").as_double();
    gaussian_stddev_  = this->get_parameter("gaussian_stddev").as_double();
    salt_pepper_prob_ = this->get_parameter("salt_pepper_prob").as_double();
    enable_blur_      = this->get_parameter("enable_blur").as_bool();
    blur_kernel_size_ = this->get_parameter("blur_kernel_size").as_int();
    input_topic_      = this->get_parameter("input_topic").as_string();
    output_topic_     = this->get_parameter("output_topic").as_string();

    if (blur_kernel_size_ % 2 == 0) blur_kernel_size_++;

    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      input_topic_, 10,
      std::bind(&CameraNoiseNode::image_cb, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);

    RCLCPP_INFO(this->get_logger(),
      "CameraNoiseNode: %s -> %s | gaussian=%.1f | s&p=%.4f | blur=%s",
      input_topic_.c_str(), output_topic_.c_str(),
      gaussian_stddev_, salt_pepper_prob_,
      enable_blur_ ? "true" : "false");
  }

private:
  void image_cb(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    cv::Mat img = cv_ptr->image.clone();
    if (enable_blur_) add_blur(img);
    add_gaussian(img); 
    cv_ptr->image = img;
    pub_->publish(*cv_ptr->toImageMsg());
  }

  void add_gaussian(cv::Mat & img)
  {
    if (gaussian_stddev_ <= 0.0) return;
    cv::Mat noise(img.size(), CV_16SC3);
    std::normal_distribution<double> dist(gaussian_mean_, gaussian_stddev_);
    for (int r = 0; r < noise.rows; ++r)
      for (int c = 0; c < noise.cols; ++c)
        noise.at<cv::Vec3s>(r, c) = cv::Vec3s(
          static_cast<short>(dist(gen_)),
          static_cast<short>(dist(gen_)),
          static_cast<short>(dist(gen_)));
    cv::Mat tmp;
    img.convertTo(tmp, CV_16SC3);
    tmp += noise;
    tmp.convertTo(img, CV_8UC3);
  }

  void add_salt_pepper(cv::Mat & img)
  {
    if (salt_pepper_prob_ <= 0.0) return;
    std::uniform_real_distribution<double> u(0.0, 1.0);
    for (int r = 0; r < img.rows; ++r)
      for (int c = 0; c < img.cols; ++c) {
        double p = u(gen_);
        if      (p <  salt_pepper_prob_)      img.at<cv::Vec3b>(r, c) = {255, 255, 255};
        else if (p < 2.0 * salt_pepper_prob_) img.at<cv::Vec3b>(r, c) = {0,   0,   0};
      }
  }

  void add_blur(cv::Mat & img)
  {
    cv::GaussianBlur(img, img, cv::Size(blur_kernel_size_, blur_kernel_size_), 0.0);
  }

  double      gaussian_mean_;
  double      gaussian_stddev_;
  double      salt_pepper_prob_;
  bool        enable_blur_;
  int         blur_kernel_size_;
  std::string input_topic_;
  std::string output_topic_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr    pub_;

  std::mt19937 gen_;  // rd_ eliminado, semilla por steady_clock
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraNoiseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}