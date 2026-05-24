#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp/action_node.h>
#include "puzzlebot_interfaces/action/visual_servoing.hpp"

namespace puzzlebot_bt {

class VisualServoingAction : public BT::StatefulActionNode {
public:
    using VisualServoing = puzzlebot_interfaces::action::VisualServoing;
    using GoalHandleVisualServoing = rclcpp_action::ClientGoalHandle<VisualServoing>;

    VisualServoingAction(const std::string& name, const BT::NodeConfig& config,
                         rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config),
          node_(node) {
        action_client_ = rclcpp_action::create_client<VisualServoing>(node_, "visual_servoing");
    }

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("target_area")};
    }

    BT::NodeStatus onStart() override {
        result_ = GoalHandleVisualServoing::WrappedResult();
        
        double target_area = 28000.0;
        getInput("target_area", target_area);

        if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(node_->get_logger(), "visual_servoing action server NOT available");
            return BT::NodeStatus::FAILURE;
        }

        auto goal = VisualServoing::Goal();
        goal.target_area = target_area;

        auto send_goal_options = rclcpp_action::Client<VisualServoing>::SendGoalOptions();
        send_goal_options.feedback_callback =
            [this](GoalHandleVisualServoing::SharedPtr,
                   const std::shared_ptr<const VisualServoing::Feedback> feedback) {
                RCLCPP_INFO(node_->get_logger(), "area=%.0f ex=%.1f ey=%.0f",
                            feedback->current_area, feedback->error_x, feedback->error_area);
            };
        
        send_goal_options.result_callback =
            [this](const GoalHandleVisualServoing::WrappedResult& result) {
                result_ = result;
            };

        goal_handle_future_ = action_client_->async_send_goal(goal, send_goal_options);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (result_.code == rclcpp_action::ResultCode::UNKNOWN) {
            return BT::NodeStatus::RUNNING;
        }

        if (result_.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(node_->get_logger(), "Visual servoing action failed");
            return BT::NodeStatus::FAILURE;
        }

        if (!result_.result->success) {
            RCLCPP_WARN(node_->get_logger(), "Visual servoing unsuccessful: %s",
                        result_.result->message.c_str());
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(node_->get_logger(), "Visual servoing completed: %s",
                    result_.result->message.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override {
        RCLCPP_INFO(node_->get_logger(), "Visual servoing action halted");
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<VisualServoing>::SharedPtr action_client_;
    GoalHandleVisualServoing::WrappedResult result_;
    std::shared_future<GoalHandleVisualServoing::SharedPtr> goal_handle_future_;
};

}