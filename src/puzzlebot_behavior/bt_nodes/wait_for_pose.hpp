#pragma once
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("qr_pose")
    };
  }

  BT::NodeStatus onStart() override {
    double timeout_sec = 5.0;
    getInput("timeout_sec", timeout_sec);
    timeout_ = rclcpp::Duration::from_seconds(timeout_sec);

    received_ = false;

    // subscribe only while this node is active
    sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "qr_detection_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        last_msg_ = *msg;
        received_ = true;
      });

    start_time_ = node_->now();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    if ((node_->now() - start_time_) >= timeout_) {
      RCLCPP_WARN(node_->get_logger(), "Timeout waiting for qr_pose");
      sub_.reset();
      return BT::NodeStatus::FAILURE;
    }

    if (!received_) {
      return BT::NodeStatus::RUNNING;
    }

    // transform the detection from camera_color_optical_frame to map
    geometry_msgs::msg::PoseStamped pose_map;
    try {
      pose_map = tf_buffer_->transform(last_msg_, "map", std::chrono::milliseconds(100));
    }
    catch (const tf2::TransformException& e) {
      RCLCPP_ERROR(node_->get_logger(), "TF transform to map failed: %s", e.what());
      // wait for another detection instead of failing, until timeout hits
      received_ = false;
      return BT::NodeStatus::RUNNING;
    }

    setOutput("qr_pose", pose_map);
    sub_.reset();
    RCLCPP_INFO(node_->get_logger(), "QR pose resolved in map");
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override {
    sub_.reset();
    RCLCPP_INFO(node_->get_logger(), "WaitForQRPose halted");
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  geometry_msgs::msg::PoseStamped last_msg_;
  bool received_;
  rclcpp::Time start_time_;
  rclcpp::Duration timeout_{0, 0};
};

}  // namespace puzzlebot_bt