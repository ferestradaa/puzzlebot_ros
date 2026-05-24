#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "puzzlebot_interfaces/action/qr_detect.hpp"

namespace puzzlebot_bt {

class QrDetectionAction : public BT::StatefulActionNode {
public:
    using QrDetect = puzzlebot_interfaces::action::QrDetect;
    using GoalHandleQrDetect = rclcpp_action::ClientGoalHandle<QrDetect>;

    QrDetectionAction(const std::string& name, const BT::NodeConfig& config,
                      rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node) {
        action_client_ = rclcpp_action::create_client<QrDetect>(node_, "qr_detection");
        pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("qr_detection/pose", 10);
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<int>("consecutive_detections"),
            BT::OutputPort<std::string>("qr_data"),
            BT::OutputPort<double>("x"),
            BT::OutputPort<double>("y"),
            BT::OutputPort<double>("z")
        };
    }

    BT::NodeStatus onStart() override {
        result_ = GoalHandleQrDetect::WrappedResult();
        
        int consecutive_detections = 3; //default if not set explicitly 
        getInput("consecutive_detections", consecutive_detections);

        if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(node_->get_logger(), "qr_detection action server NOT available");
            return BT::NodeStatus::FAILURE;
        }

        auto goal = QrDetect::Goal();
        goal.consecutive_detections = consecutive_detections;

        auto send_goal_options = rclcpp_action::Client<QrDetect>::SendGoalOptions();
        
        send_goal_options.feedback_callback = 
            [this](GoalHandleQrDetect::SharedPtr, const std::shared_ptr<const QrDetect::Feedback> feedback) {
                RCLCPP_INFO(node_->get_logger(), "QR detections: %u", feedback->current_consecutive_detections);
            };

        send_goal_options.result_callback = 
            [this](const GoalHandleQrDetect::WrappedResult& result) {
                result_ = result;
            };

        action_client_->async_send_goal(goal, send_goal_options);
        
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (result_.code == rclcpp_action::ResultCode::UNKNOWN) {
            return BT::NodeStatus::RUNNING;
        }

        if (result_.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(node_->get_logger(), "QR detection action failed");
            return BT::NodeStatus::FAILURE;
        }

        if (!result_.result->success) {
            RCLCPP_WARN(node_->get_logger(), "QR detection unsuccessful");
            return BT::NodeStatus::FAILURE;
        }

        pose_pub_->publish(result_.result->pose); //publish the obtained pose for path planner consumming

        setOutput("qr_data", result_.result->qr_data);
        setOutput("x", result_.result->pose.pose.position.x);
        setOutput("y", result_.result->pose.pose.position.y);
        setOutput("z", result_.result->pose.pose.position.z); //this will be used as input for forklif motion

        RCLCPP_INFO(node_->get_logger(), "QR detected: %s at (%.3f, %.3f, %.3f)",
                    result_.result->qr_data.c_str(),
                    result_.result->pose.pose.position.x,
                    result_.result->pose.pose.position.y,
                    result_.result->pose.pose.position.z);

        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override {
        RCLCPP_INFO(node_->get_logger(), "QR detection action halted");
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<QrDetect>::SharedPtr action_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    GoalHandleQrDetect::WrappedResult result_;
};

}