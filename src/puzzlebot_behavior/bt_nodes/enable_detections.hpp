#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <behaviortree_cpp/action_node.h>

namespace puzzlebot_bt{

class EnableDetections : public BT::SyncActionNode {
public:
    EnableDetections(const std::string& name, const BT::NodeConfig& config, 
                     rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node) {
            client_ = node_ ->create_client<std_srvs::srv::SetBool>("enable_detection"); 
        }

    static BT::PortsList providedPorts() {
        return {BT::InputPort<bool>("enable")};  //declare input port enable as bool
    }

    BT::NodeStatus tick() override {
        bool enable = false;
        getInput("enable", enable);

        if (!client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(node_->get_logger(), "enable_detection service NOT available");
            return BT::NodeStatus::FAILURE;
        }

        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = enable;
        auto future = client_->async_send_request(request);

        //executor externo ya esta spineando bt_node_
        if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
            RCLCPP_ERROR(node_->get_logger(), "enable_detection service DID not respond");
            return BT::NodeStatus::FAILURE;
        }

        return future.get()->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

private: 
    rclcpp::Node::SharedPtr node_; 
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;  
};

}

