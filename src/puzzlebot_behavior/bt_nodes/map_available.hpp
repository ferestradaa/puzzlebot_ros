#pragma once
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace puzzlebot_bt {

class CheckMapAvailable : public BT::StatefulActionNode {
public:
  CheckMapAvailable(const std::string& name, const BT::NodeConfig& config,
                    rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config),
      node_(node),
      start_time_(0, 0, RCL_ROS_TIME),
      timeout_(0, 0) {
    rclcpp::QoS qos(1);
    qos.transient_local().reliable();

    map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", qos,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        last_map_ = msg;
      });
  }

  static BT::PortsList providedPorts() {
    return { BT::InputPort<double>("timeout_sec") };
  }

  BT::NodeStatus onStart() override {
    double timeout_sec = 20.0; // max time esperando el mapa
    getInput("timeout_sec", timeout_sec);
    start_time_ = node_->now();
    timeout_ = rclcpp::Duration::from_seconds(timeout_sec);

    if (isMapValid()) {
      RCLCPP_INFO(node_->get_logger(), "Map available on first tick");
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    if (isMapValid()) {
      RCLCPP_INFO(node_->get_logger(), "Map received: %ux%u",
                  last_map_->info.width, last_map_->info.height);
      return BT::NodeStatus::SUCCESS;
    }
    if ((node_->now() - start_time_) >= timeout_) {
      RCLCPP_ERROR(node_->get_logger(), "No valid map on /map after timeout");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {
    RCLCPP_INFO(node_->get_logger(), "CheckMapAvailable halted");
  }

private:
  bool isMapValid() const {
    if (!last_map_) return false;
    if (last_map_->info.width == 0 || last_map_->info.height == 0) return false;
    size_t expected = static_cast<size_t>(last_map_->info.width) *
                      static_cast<size_t>(last_map_->info.height);
    return last_map_->data.size() == expected;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr last_map_;
  rclcpp::Time start_time_;
  rclcpp::Duration timeout_;
};

}