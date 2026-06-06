#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/twist.hpp>
#include "puzzlebot_interfaces/action/drive_raw.hpp"

namespace puzzlebot_bt {

class DriveRawAction : public BT::StatefulActionNode {
public:
    using DriveRaw = puzzlebot_interfaces::action::DriveRaw;
    using GoalHandleDriveRaw = rclcpp_action::ClientGoalHandle<DriveRaw>;

    DriveRawAction(const std::string& name, const BT::NodeConfig& config,
                   rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), 
          node_(node), 
          start_time_(0, 0, RCL_ROS_TIME),
          duration_(0, 0) {
        action_client_ = rclcpp_action::create_client<DriveRaw>(node_, "drive_raw");
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("linear_x"),
            BT::InputPort<double>("angular_z"),
            BT::InputPort<double>("duration_sec")
        };
    }

    BT::NodeStatus onStart() override {
        result_ = GoalHandleDriveRaw::WrappedResult();
        double linear_x = 0.0;
        double angular_z = 0.0;
        double duration_sec = 1.0;

        getInput("linear_x", linear_x);
        getInput("angular_z", angular_z);
        getInput("duration_sec", duration_sec);

        if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(node_->get_logger(), "drive_raw action server NOT available");
            return BT::NodeStatus::FAILURE;
        }

        auto goal = DriveRaw::Goal();
        goal.cmd_vel.linear.x = linear_x;
        goal.cmd_vel.angular.z = angular_z;

        auto send_goal_options = rclcpp_action::Client<DriveRaw>::SendGoalOptions();



        send_goal_options.feedback_callback = 
            [this](GoalHandleDriveRaw::SharedPtr, const std::shared_ptr<const DriveRaw::Feedback> feedback) {
                 auto now = node_->now();
                 if ((now - last_feedback_log_).seconds() < 3.0) return;
                 last_feedback_log_ = now;
                RCLCPP_INFO(node_->get_logger(), "Distance to obstacle: %.2f m", feedback->distance_to_obstacle);
            };


        send_goal_options.result_callback = 
            [this](const GoalHandleDriveRaw::WrappedResult& result) {
                result_ = result;
            };

        goal_handle_future_ = action_client_->async_send_goal(goal, send_goal_options);
        start_time_ = node_->now();
        duration_ = rclcpp::Duration::from_seconds(duration_sec);
        
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (result_.code == rclcpp_action::ResultCode::UNKNOWN) {
            if ((node_->now() - start_time_) >= duration_) {
                auto goal_handle = goal_handle_future_.get();
                if (goal_handle) {
                    action_client_->async_cancel_goal(goal_handle);
                }
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::RUNNING;
        }

        if (result_.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(node_->get_logger(), "Drive raw action failed");
            return BT::NodeStatus::FAILURE;
        }

        if (!result_.result->success) {
            RCLCPP_WARN(node_->get_logger(), "Drive raw unsuccessful: %s", result_.result->message.c_str());
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(node_->get_logger(), "Drive raw completed: %s", result_.result->message.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override {
        if (goal_handle_future_.valid()) {
            auto goal_handle = goal_handle_future_.get();
            if (goal_handle) {
                action_client_->async_cancel_goal(goal_handle);
            }
        }
        RCLCPP_INFO(node_->get_logger(), "Drive raw action halted");
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<DriveRaw>::SharedPtr action_client_;
    GoalHandleDriveRaw::WrappedResult result_;
    std::shared_future<GoalHandleDriveRaw::SharedPtr> goal_handle_future_;
    rclcpp::Time start_time_;
    rclcpp::Duration duration_;
    rclcpp::Time last_feedback_log_{0, 0, RCL_ROS_TIME};
};

}