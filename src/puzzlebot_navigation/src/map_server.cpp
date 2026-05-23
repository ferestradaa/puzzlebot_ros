#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

class MapServerNode : public rclcpp::Node
{
public:
  MapServerNode()
  : Node("map_server_node")
  {
    declare_parameter<std::string>("map_yaml", "map.yaml");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<bool>("slam_raw_pgm_order", true);

    const auto map_yaml     = get_parameter("map_yaml").as_string();
    const auto map_frame    = get_parameter("map_frame").as_string();
    const auto raw_pgm_order = get_parameter("slam_raw_pgm_order").as_bool();

    const auto share_dir = ament_index_cpp::get_package_share_directory("puzzlebot_navigation");
    const std::string yaml_path = share_dir + "/maps/" + map_yaml;

    auto msg = load_map(yaml_path, map_frame, raw_pgm_order);

    rclcpp::QoS map_qos(1);
    map_qos.reliable();
    map_qos.transient_local();

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);
    map_pub_->publish(msg);

    RCLCPP_INFO(get_logger(), "Map published on /map (%dx%d, res=%.3f)",
      msg.info.width, msg.info.height, msg.info.resolution);
  }

private:
  nav_msgs::msg::OccupancyGrid load_map(
    const std::string & yaml_path,
    const std::string & frame_id,
    bool raw_pgm_order)
  {
    std::ifstream test(yaml_path);
    if (!test.good()) {
      throw std::runtime_error("Map YAML not found: " + yaml_path);
    }
    test.close();

    YAML::Node info = YAML::LoadFile(yaml_path);

    const double resolution     = info["resolution"].as<double>();
    const double occupied_thresh = info["occupied_thresh"].as<double>();
    const double free_thresh    = info["free_thresh"].as<double>();
    const bool   negate         = info["negate"].as<int>(0) != 0;

    const auto origin_node = info["origin"];
    const double origin_x  = origin_node[0].as<double>();
    const double origin_y  = origin_node[1].as<double>();

    const std::string image_name = info["image"].as<std::string>();
    
    size_t last_slash = yaml_path.find_last_of("/");
    std::string dir = yaml_path.substr(0, last_slash);
    const std::string image_path = dir + "/" + image_name;

    cv::Mat img = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
      throw std::runtime_error("Could not load map image: " + image_path);
    }

    if (!raw_pgm_order) {
      cv::flip(img, img, 0);
    }

    const int width  = img.cols;
    const int height = img.rows;

    std::vector<int8_t> data(static_cast<size_t>(width * height), -1);

    const int occupied_pixel = negate
      ? static_cast<int>(occupied_thresh * 255.0)
      : static_cast<int>((1.0 - occupied_thresh) * 255.0);
    const int free_pixel = negate
      ? static_cast<int>(free_thresh * 255.0)
      : static_cast<int>((1.0 - free_thresh) * 255.0);

    for (int r = 0; r < height; ++r) {
      for (int c = 0; c < width; ++c) {
        const int pixel = img.at<uint8_t>(r, c);
        int8_t cell = -1;
        if (!negate) {
          if (pixel >= free_pixel) {
            cell = 0;
          } else if (pixel <= occupied_pixel) {
            cell = 100;
          }
        } else {
          if (pixel <= free_pixel) {
            cell = 0;
          } else if (pixel >= occupied_pixel) {
            cell = 100;
          }
        }
        data[static_cast<size_t>(r * width + c)] = cell;
      }
    }

    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp    = now();
    msg.header.frame_id = frame_id;
    msg.info.resolution = static_cast<float>(resolution);
    msg.info.width      = static_cast<uint32_t>(width);
    msg.info.height     = static_cast<uint32_t>(height);
    msg.info.origin.position.x = origin_x;
    msg.info.origin.position.y = origin_y;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.w = 1.0;
    msg.data = data;

    return msg;
  }

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapServerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}