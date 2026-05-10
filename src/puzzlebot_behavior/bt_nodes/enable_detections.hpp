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
            client_ = node_ ->create_client<std_srvs::srv::SetBool>("enable_detections"); 
        }

    static BT::PortsList providedPorts() {
        return {BT::InputPort<bool>("enable")};  //declare input port enable as bool
    }

    BT::NodeStatus tick() override {
        bool enable = false; 
        getInput("enable", enable); //reading the blackboard declared in xml tree

        if (!client_->wait_for_service(std::chrono::seconds(2))){
            RCLCPP_ERROR(node_->get_logger(), "enable_detection_service NOT available"); 
            return BT::NodeStatus::FAILURE; 
        }

        auto request = std::make_shared<std_srvs::srv::SetBool::Request>(); 
        request->data = enable; 

        auto future = client_ -> async_send_request(request); 

        if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(3))
            != rclcpp::FutureReturnCode::SUCCESS){
            RCLCPP_ERROR(node_->get_logger(), "enable_detection_service DID not respond"); 
            return BT::NodeStatus::FAILURE;
        }

        return future.get()-> success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE; 
    }

private: 
    rclcpp::Node::SharedPtr node_; 
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;  
};

}

