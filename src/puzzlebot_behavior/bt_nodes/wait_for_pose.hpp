#pragma once
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace puzzlebot_bt {

class WaitForQRPoseAction : public BT::StatefulActionNode {
public:
  WaitForQRPoseAction(const std::string& name, const BT::NodeConfig& config,
                      rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config),
    node_(node),
    start_time_(0, 0, RCL_ROS_TIME) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("timeout_sec"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("qr_pose"),
      BT::OutputPort<std::string>("qr_data")
    };
  }

  BT::NodeStatus onStart() override {
    double timeout_sec = 5.0;
    getInput("timeout_sec", timeout_sec);
    timeout_ = rclcpp::Duration::from_seconds(timeout_sec);
    received_pose_ = false;
    received_data_ = false;
    last_data_.clear();

    sub_pose_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "qr_detection/pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        last_msg_ = *msg;
        received_pose_ = true;
      });

    sub_data_ = node_->create_subscription<std_msgs::msg::String>(
      "qr_detection/data", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        last_data_ = msg->data;
        received_data_ = true;
      });

    start_time_ = node_->now();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    if ((node_->now() - start_time_) >= timeout_) {
      RCLCPP_WARN(node_->get_logger(), "Timeout waiting for qr_pose");
      sub_pose_.reset();
      sub_data_.reset();
      return BT::NodeStatus::FAILURE;
    }

    if (!received_pose_ || !received_data_) {
      return BT::NodeStatus::RUNNING;
    }

    geometry_msgs::msg::PoseStamped pose_map;
    try {
      pose_map = tf_buffer_->transform(last_msg_, "map", std::chrono::milliseconds(100));
    }
    catch (const tf2::TransformException& e) {
      RCLCPP_ERROR(node_->get_logger(), "TF transform to map failed: %s", e.what());
      received_pose_ = false;
      received_data_ = false;
      return BT::NodeStatus::RUNNING;
    }

    setOutput("qr_pose", pose_map);
    setOutput("qr_data", last_data_);
    sub_pose_.reset();
    sub_data_.reset();
    RCLCPP_INFO(node_->get_logger(), "QR pose and data resolved: %s", last_data_.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override {
    sub_pose_.reset();
    sub_data_.reset();
    RCLCPP_INFO(node_->get_logger(), "WaitForQRPose halted");
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_data_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  geometry_msgs::msg::PoseStamped last_msg_;
  std::string last_data_;
  bool received_pose_ = false;
  bool received_data_ = false;
  rclcpp::Time start_time_;
  rclcpp::Duration timeout_{0, 0};
};

}  // namespace puzzlebot_bt