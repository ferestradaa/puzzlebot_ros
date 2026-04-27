#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class FrameIdStamper : public rclcpp::Node
{
public:
  FrameIdStamper() : Node("frame_id_stamper")
  {
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      [this](sensor_msgs::msg::Image::SharedPtr msg) {
        msg->header.frame_id = "camera_color_optical_frame";
        pub_->publish(*msg);
      });

    pub_ = create_publisher<sensor_msgs::msg::Image>("/camera/image_rect", 10);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameIdStamper>());
  rclcpp::shutdown();
  return 0;
}