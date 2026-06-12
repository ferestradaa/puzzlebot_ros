#pragma once
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <std_msgs/msg/bool.hpp>

namespace puzzlebot_bt {

class SetReverseMode : public BT::SyncActionNode {
public:
    SetReverseMode(const std::string& name, const BT::NodeConfig& config,
                   rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node)
    {
        reverse_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
            "/pure_pursuit/reverse", 10);
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<bool>("reverse")
        };
    }

    BT::NodeStatus tick() override {
        bool reverse;
        if (!getInput("reverse", reverse)) {
            RCLCPP_ERROR(node_->get_logger(), "SetReverseMode: missing port 'reverse'");
            return BT::NodeStatus::FAILURE;
        }

        std_msgs::msg::Bool msg;
        msg.data = reverse;
        reverse_pub_->publish(msg);

        RCLCPP_INFO(node_->get_logger(), "reverse mode -> %s", reverse ? "true" : "false");
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reverse_pub_;
};

} // namespace puzzlebot_bt