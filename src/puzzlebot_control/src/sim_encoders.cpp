#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>

class WheelVelPublisher : public rclcpp::Node
{
public:
  WheelVelPublisher()
  : Node("sim_encoders")
  {
    // Declare parameters for joint names (adjust defaults to match your robot)
    this->declare_parameter<std::string>("base_to_left_wheel",  "base_to_left_wheel");
    this->declare_parameter<std::string>("base_to_right_wheel", "base_to_right_wheel");

    left_joint_name_  = this->get_parameter("base_to_left_wheel").as_string();
    right_joint_name_ = this->get_parameter("base_to_right_wheel").as_string();

    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&WheelVelPublisher::jointStateCallback, this, std::placeholders::_1));

    pub_left_  = this->create_publisher<std_msgs::msg::Float32>("VelEncL", 10);
    pub_right_ = this->create_publisher<std_msgs::msg::Float32>("VelEncR", 10);

    RCLCPP_INFO(this->get_logger(),
      "WheelVelPublisher started. Listening for joints: '%s' (L) | '%s' (R)",
      left_joint_name_.c_str(), right_joint_name_.c_str());
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // joint_states may not always carry velocity data — guard against empty vector
    if (msg->velocity.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Received /joint_states with no velocity data.");
      return;
    }

    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == left_joint_name_) {
        std_msgs::msg::Float32 vel_msg;
        vel_msg.data = static_cast<float>(msg->velocity[i]);
        pub_left_->publish(vel_msg);
      } else if (msg->name[i] == right_joint_name_) {
        std_msgs::msg::Float32 vel_msg;
        vel_msg.data = static_cast<float>(msg->velocity[i]);
        pub_right_->publish(vel_msg);
      }
    }
  }

  std::string left_joint_name_;
  std::string right_joint_name_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_left_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_right_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelVelPublisher>());
  rclcpp::shutdown();
  return 0;
}