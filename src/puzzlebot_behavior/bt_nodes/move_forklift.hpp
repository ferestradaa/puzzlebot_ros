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
        publisher_ = node_->create_publisher<std_msgs::msg::Float32>("/forklift/setpoint", 10);
        subscriber_ = node_->create_subscription<std_msgs::msg::Float32>(
            "/forklift/height", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                current_height_ = msg->data;
            });
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("target_height"),
            BT::InputPort<float>("tolerance_mm", 6.0f, "tolerance in mm"),
            BT::InputPort<double>("timeout_s", 10.0, "timeout in seconds"),
            BT::InputPort<double>("republish_interval_s", 1.0, "republish interval in seconds")
        };
    }

    BT::NodeStatus onStart() override {
        getInput("target_height", target_height_);
        getInput("tolerance_mm", tolerance_mm_);
        getInput("republish_interval_s", republish_interval_s_);

        double timeout_s;
        getInput("timeout_s", timeout_s);
        deadline_     = node_->now() + rclcpp::Duration::from_seconds(timeout_s);
        last_publish_ = node_->now();

        auto msg = std_msgs::msg::Float32();
        msg.data = target_height_;
        publisher_->publish(msg);

        RCLCPP_INFO(node_->get_logger(),
            "move_forklift: target=%.1f mm", target_height_);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {

        if (node_->now() > deadline_) {
            RCLCPP_ERROR(node_->get_logger(),
                "move_forklift: timeout | target=%.1f current=%.1f",
                target_height_, current_height_);
            return BT::NodeStatus::FAILURE;
        }

        if (std::abs(current_height_ - target_height_) <= tolerance_mm_) {
            RCLCPP_INFO(node_->get_logger(),
                "move_forklift: reached %.1f mm (current=%.1f)",
                target_height_, current_height_);
            return BT::NodeStatus::SUCCESS;
        }

        if ((node_->now() - last_publish_).seconds() > republish_interval_s_) {
            auto msg = std_msgs::msg::Float32();
            msg.data = target_height_;
            publisher_->publish(msg);
            last_publish_ = node_->now();
            RCLCPP_INFO(node_->get_logger(),
                "move_forklift: republish target=%.1f current=%.1f",
                target_height_, current_height_);
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        RCLCPP_WARN(node_->get_logger(),
            "move_forklift: halted at %.1f mm", current_height_);
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_;

    float  current_height_;
    double  target_height_;
    float  tolerance_mm_;
    double republish_interval_s_;

    rclcpp::Time deadline_;
    rclcpp::Time last_publish_;
};

} // namespace puzzlebot_bt