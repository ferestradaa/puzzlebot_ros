#pragma once
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <std_msgs/msg/float32.hpp>
#include <cmath>

namespace puzzlebot_bt {

class MoveForklift : public BT::StatefulActionNode {
public:
    MoveForklift(const std::string& name, const BT::NodeConfig& config,
                 rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), node_(node), current_height_(0.0f)
    {
        publisher_ = node_->create_publisher<std_msgs::msg::Float32>("/forklift/target_height", 10);
        subscriber_ = node_->create_subscription<std_msgs::msg::Float32>(
            "/forklift/current_height", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                current_height_ = msg->data;
            });
    }

    static BT::PortsList providedPorts() {
        return {BT::InputPort<float>("target_height")};
    }

    BT::NodeStatus onStart() override {
        getInput("target_height", target_height_);

        auto msg = std_msgs::msg::Float32();
        msg.data = target_height_;
        publisher_->publish(msg);

        deadline_ = node_->now() + rclcpp::Duration::from_seconds(5.0);

        RCLCPP_INFO(node_->get_logger(), "move_forklift: moving to %.3f", target_height_);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        rclcpp::spin_some(node_);

        if (node_->now() > deadline_) {
            RCLCPP_ERROR(node_->get_logger(), "move_forklift: timeout waiting for %.3f, current=%.3f",
                         target_height_, current_height_);
            return BT::NodeStatus::FAILURE;
        }

        constexpr float TOLERANCE = 0.01f;
        if (std::abs(current_height_ - target_height_) <= TOLERANCE) {
            RCLCPP_INFO(node_->get_logger(), "move_forklift: reached %.3f", target_height_);
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        RCLCPP_WARN(node_->get_logger(), "move_forklift: halted");
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_;
    float current_height_;
    float target_height_;
    rclcpp::Time deadline_;
};

} // namespace puzzlebot_bt