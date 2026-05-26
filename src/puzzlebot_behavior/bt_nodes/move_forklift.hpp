#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <behaviortree_cpp/action_node.h>

namespace puzzlebot_bt{

class MoveForklift : public BT::SyncActionNode {
public:
    MoveForklift(const std::string& name, const BT::NodeConfig& config, 
                     rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node) {
            client_ = node_ ->create_client<std_srvs::srv::SetBool>("move_forklift"); 
        }

    static BT::PortsList providedPorts() {
        return {BT::InputPort<float>("target_height")};  //declare input port enable as flaot for height 
    }

    BT::NodeStatus tick() override {
        float target_height = 0.75;
        getInput("target_height", target_height);

        if (!client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(node_->get_logger(), "move_forklift service NOT available");
            return BT::NodeStatus::FAILURE;
        }

        auto request = std::make_shared<std_msgs::msg::Float32>();
        request->data = target_height;

        auto future = client_->async_send_request(request);

        if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
            RCLCPP_ERROR(node_->get_logger(), "enable_detection service timeout");
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::SUCCESS; 

private: 
    rclcpp::Node::SharedPtr node_; 
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;  
};

}

