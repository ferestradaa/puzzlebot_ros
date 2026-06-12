#pragma once
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <std_msgs/msg/string.hpp>

namespace puzzlebot_bt {

class SetControllerSource : public BT::SyncActionNode {
public:
    SetControllerSource(const std::string& name, const BT::NodeConfig& config,
                        rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node)
    {
        source_pub_ = node_->create_publisher<std_msgs::msg::String>(
            "/cmd_vel/active_source", 10);
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("source")
        };
    }

    BT::NodeStatus tick() override {
        std::string source;
        if (!getInput("source", source)) {
            RCLCPP_ERROR(node_->get_logger(), "SetControllerSource: missing port 'source'");
            return BT::NodeStatus::FAILURE;
        }

        std_msgs::msg::String msg;
        msg.data = source;
        source_pub_->publish(msg);

        RCLCPP_INFO(node_->get_logger(), "controller source -> %s", source.c_str());
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr source_pub_;
};

}