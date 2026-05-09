#pragma once
#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

class GetTargetPose : public BT::SyncActionNode {
public:
    GetTargetPose(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::OutputPort<geometry_msgs::msg::Pose>("pallet_pose") };
    }

    BT::NodeStatus tick() override {
        RCLCPP_INFO(rclcpp::get_logger("GetTargetPose"), "tick");
        return BT::NodeStatus::SUCCESS;
    }
};

class GoToTarget : public BT::SyncActionNode {
public:
    GoToTarget(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<geometry_msgs::msg::Pose>("target_pose") };
    }

    BT::NodeStatus tick() override {
        RCLCPP_INFO(rclcpp::get_logger("GoToTarget"), "tick");
        return BT::NodeStatus::SUCCESS;
    }
};