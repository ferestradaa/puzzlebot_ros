#pragma once
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

namespace puzzlebot_bt {

class SetHeightFromZone : public BT::SyncActionNode {
public:
    SetHeightFromZone(const std::string& name, const BT::NodeConfig& config,
                      rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("zone"),
            BT::OutputPort<double>("fork_height")
        };
    }

    BT::NodeStatus tick() override {
        std::string zone;
        if (!getInput("zone", zone)) {
            RCLCPP_ERROR(node_->get_logger(), "SetHeightFromZone: missing zone input");
            return BT::NodeStatus::FAILURE;
        }

        if (zone == "inspect")
            setOutput("fork_height", 58.0);
        else if (zone == "center")
            setOutput("fork_height", 20.0);
        else {
            RCLCPP_ERROR(node_->get_logger(), "SetHeightFromZone: unknown zone '%s'", zone.c_str());
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(node_->get_logger(), "SetHeightFromZone: zone=%s", zone.c_str());
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
};

} // namespace puzzlebot_bt