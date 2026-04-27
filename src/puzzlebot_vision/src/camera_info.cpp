#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>  // FIX 1: faltaba este include

#include <array>
#include <string>
#include <vector>
#include <stdexcept>

template <typename T, std::size_t N>
std::array<T, N> yaml_to_array(const YAML::Node & node)
{
  if (!node || !node["data"]) {
    throw std::runtime_error("YAML node missing 'data' field");
  }
  const YAML::Node & data = node["data"];
  if (data.size() != N) {
    throw std::runtime_error(
      "Expected " + std::to_string(N) + " elements but got " +
      std::to_string(data.size()));
  }
  std::array<T, N> arr{};
  for (std::size_t i = 0; i < N; ++i) {
    arr[i] = data[i].as<T>();
  }
  return arr;
}

template <typename T>
std::vector<T> yaml_to_vector(const YAML::Node & node)
{
  if (!node || !node["data"]) {
    throw std::runtime_error("YAML node missing 'data' field");
  }
  std::vector<T> vec;
  for (const auto & elem : node["data"]) {
    vec.push_back(elem.as<T>());
  }
  return vec;
}


class CameraInfoPublisher : public rclcpp::Node
{
public:
  explicit CameraInfoPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("camera_info_publisher", options)
  {
    this->declare_parameter<std::string>("camera_info", "camera_info.yaml");
    this->declare_parameter<std::string>("camera_frame_id", "camera_color_optical_frame");

    // FIX 2: resolver path — si no es absoluto, buscar en share del paquete
    std::string yaml_path = this->get_parameter("camera_info").as_string();
    if (yaml_path.empty() || yaml_path.front() != '/') {
      const std::string share_dir =
        ament_index_cpp::get_package_share_directory("puzzlebot_vision");
      yaml_path = share_dir + "/cfg/" + yaml_path;
    }

    const std::string frame_id =
      this->get_parameter("camera_frame_id").as_string();

    // FIX 3: eliminadas las variables pkg_path y map_path que nunca se usaban

    RCLCPP_INFO(this->get_logger(), "Loading camera info from: %s", yaml_path.c_str());
    load_camera_info(yaml_path, frame_id);
    RCLCPP_INFO(
      this->get_logger(),
      "Loaded — %dx%d  fx=%.2f  fy=%.2f  cx=%.2f  cy=%.2f",
      camera_info_msg_.width, camera_info_msg_.height,
      camera_info_msg_.k[0], camera_info_msg_.k[4],
      camera_info_msg_.k[2], camera_info_msg_.k[5]);

    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/camera/camera_info", rclcpp::SensorDataQoS());

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_rect",
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr img_msg) {
        camera_info_msg_.header = img_msg->header;
        camera_info_pub_->publish(camera_info_msg_);
      });

    RCLCPP_INFO(
      this->get_logger(),
      "Subscribed to /camera/image_rect  to  publishing /camera/camera_info");
  }

private:
  void load_camera_info(const std::string & path, const std::string & frame_id)
  {
    YAML::Node root;
    try {
      root = YAML::LoadFile(path);
    } catch (const YAML::Exception & e) {
      RCLCPP_FATAL(this->get_logger(), "Failed to load YAML: %s", e.what());
      throw;
    }

    camera_info_msg_.width  = root["image_width"].as<uint32_t>();
    camera_info_msg_.height = root["image_height"].as<uint32_t>();

    if (root["camera_name"]) {
      RCLCPP_INFO(
        this->get_logger(), "Camera name: %s",
        root["camera_name"].as<std::string>().c_str());
    }

    camera_info_msg_.distortion_model =
      root["distortion_model"] ?
      root["distortion_model"].as<std::string>() : "plumb_bob";

    auto K = yaml_to_array<double, 9>(root["camera_matrix"]);
    std::copy(K.begin(), K.end(), camera_info_msg_.k.begin());

    auto D = yaml_to_vector<double>(root["distortion_coefficients"]);
    camera_info_msg_.d = D;

    auto R = yaml_to_array<double, 9>(root["rectification_matrix"]);
    std::copy(R.begin(), R.end(), camera_info_msg_.r.begin());

    auto P = yaml_to_array<double, 12>(root["projection_matrix"]);
    std::copy(P.begin(), P.end(), camera_info_msg_.p.begin());

    camera_info_msg_.header.frame_id = frame_id;
  }

  sensor_msgs::msg::CameraInfo                                camera_info_msg_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr  camera_info_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr    image_sub_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraInfoPublisher>());
  rclcpp::shutdown();
  return 0;
}