#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "puzzlebot_interfaces/action/go_to.hpp"

namespace puzzlebot_bt {

class GoToAction : public BT::StatefulActionNode {
public:
  using GoTo = puzzlebot_interfaces::action::GoTo;
  using GoalHandleGoTo = rclcpp_action::ClientGoalHandle<GoTo>;

  GoToAction(const std::string& name, const BT::NodeConfig& config,
             rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config),
    node_(node) {
    action_client_ = rclcpp_action::create_client<GoTo>(node_, "go_to");
  }

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("zone"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("pose")
    };
  }

  BT::NodeStatus onStart() override {
    result_ = GoalHandleGoTo::WrappedResult();

    if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(node_->get_logger(), "go_to action server NOT available");
      return BT::NodeStatus::FAILURE;
    }

    auto goal = GoTo::Goal();
    geometry_msgs::msg::PoseStamped pose_in;
    std::string zone_in;

    // pose port present means the goal is already resolved
    if (getInput("pose", pose_in)) {
      goal.goal_type = GoTo::Goal::POSE;
      goal.target_pose = pose_in;
    }
    // otherwise fall back to the named zone string
    else if (getInput("zone", zone_in)) {
      goal.goal_type = GoTo::Goal::NAMED_ZONE;
      goal.zone_name = zone_in;
    }
    else {
      RCLCPP_ERROR(node_->get_logger(), "neither pose nor zone provided");
      return BT::NodeStatus::FAILURE;
    }

    auto send_goal_options = rclcpp_action::Client<GoTo>::SendGoalOptions();

    send_goal_options.feedback_callback =
      [this](GoalHandleGoTo::SharedPtr, const std::shared_ptr<const GoTo::Feedback> feedback) {
        RCLCPP_INFO(node_->get_logger(), "Distance remaining: %.2f m, angle: %.2f rad",
                    feedback->distance_remaining, feedback->angle_remaining);
      };

    send_goal_options.result_callback =
      [this](const GoalHandleGoTo::WrappedResult& result) {
        result_ = result;
      };

    goal_handle_future_ = action_client_->async_send_goal(goal, send_goal_options);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    // still waiting for the result callback to fire
    if (result_.code == rclcpp_action::ResultCode::UNKNOWN) {
      return BT::NodeStatus::RUNNING;
    }

    if (result_.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(node_->get_logger(), "GoTo action failed");
      return BT::NodeStatus::FAILURE;
    }

    if (!result_.result->success) {
      RCLCPP_WARN(node_->get_logger(), "GoTo unsuccessful, error_code: %d",
                  result_.result->error_code);
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "GoTo completed");
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override {
    // cancel the goal if the tree halts this node
    auto goal_handle = goal_handle_future_.get();
    if (goal_handle) {
      action_client_->async_cancel_goal(goal_handle);
    }
    RCLCPP_INFO(node_->get_logger(), "GoTo action halted");
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<GoTo>::SharedPtr action_client_;
  GoalHandleGoTo::WrappedResult result_;
  std::shared_future<GoalHandleGoTo::SharedPtr> goal_handle_future_;
};

}  // namespace puzzlebot_bt