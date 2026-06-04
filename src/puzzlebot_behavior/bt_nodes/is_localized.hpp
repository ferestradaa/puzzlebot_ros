#pragma once
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/condition_node.h>
#include <std_msgs/msg/bool.hpp>

namespace puzzlebot_bt {
class IsLocalizedCondition : public BT::ConditionNode {
public:
    IsLocalizedCondition(const std::string& name, const BT::NodeConfig& config,
                         rclcpp::Node::SharedPtr node)
    : BT::ConditionNode(name, config), localized_(false)
    {
        sub_ = node->create_subscription<std_msgs::msg::Bool>(
            "/localization_ready",
            rclcpp::QoS(1).transient_local(),
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                localized_ = msg->data;
            });
    }

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        return localized_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
    bool localized_;
};
} // namespace puzzlebot_bt