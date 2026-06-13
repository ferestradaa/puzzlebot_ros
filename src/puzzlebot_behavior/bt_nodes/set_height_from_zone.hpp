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
            BT::OutputPort<double>("fork_height"),
            BT::OutputPort<double>("lift_height")
        };
    }

    BT::NodeStatus tick() override {
        std::string zone;
        if (!getInput("zone", zone)) {
            RCLCPP_ERROR(node_->get_logger(), "SetHeightFromZone: missing zone input");
            return BT::NodeStatus::FAILURE;
        }

        double fork_h, lift_h;

        if (zone == "inspect" || zone == "inspect2") {
            fork_h = 58.0;
            lift_h = 80.0; 

        } else if (zone == "center") {
            fork_h = 20.0;
            lift_h = 55.0; 
        } else {
            RCLCPP_ERROR(node_->get_logger(), "SetHeightFromZone: unknown zone '%s'", zone.c_str());
            return BT::NodeStatus::FAILURE;
        }

        setOutput("fork_height", fork_h);
        setOutput("lift_height", lift_h);

        RCLCPP_INFO(node_->get_logger(), "SetHeightFromZone: zone=%s fork=%.1f lift=%.1f",
                    zone.c_str(), fork_h, lift_h);

        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
};

} // namespace puzzlebot_bt