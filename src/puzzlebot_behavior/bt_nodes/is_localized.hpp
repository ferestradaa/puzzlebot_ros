#pragma once
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/condition_node.h>
#include <std_msgs/msg/bool.hpp>

namespace puzzlebot_bt {

class IsLocalizedCondition : public BT::ConditionNode {
public:
    IsLocalizedCondition(const std::string& name, const BT::NodeConfig& config,
                         rclcpp::Node::SharedPtr node)
        : BT::ConditionNode(name, config), detection_count_(0), localized_(false)
    {
        sub_ = node->create_subscription<std_msgs::msg::Bool>(
            "/localization_ready",
            rclcpp::QoS(1).transient_local(),
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) {
                    if (detection_count_ < REQUIRED_DETECTIONS) {
                        detection_count_++;
                    }
                    localized_ = (detection_count_ >= REQUIRED_DETECTIONS);
                } else {
                    // reset on explicit false
                    detection_count_ = 0;
                    localized_ = false;
                }
            });
    }

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        return localized_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

private:
    static constexpr int REQUIRED_DETECTIONS = 5;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
    int detection_count_;
    bool localized_;
};

} // namespace puzzlebot_bt