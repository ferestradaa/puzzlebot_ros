#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#inlcude "puzzlebot_interfaces/action/TargetPose"

#include <behaviortree_cpp/action_node.h>

class GetTargetPose : public BT : SyncActionNode{
    public: 

        using TargetPoseAction = puzzlbebot_interfaces::action::TargetPose; 

        using GoalHandleTargetPose = rclcpp_action::ClientGoalHandle<TargetPoseAction>; 


        GetTargetPose(const std::string& name, const BT::NodeConfig& config, 
                    rclcpp::Node::SharedPtr node), 
                    int consecutive_detections; 
            : BT:SyncActionNode(name, config), node_(node){

                client_ = rclcpp_action::create_client<TargetPose>(
                    this, "target_pose_action"
                ); 
            }


        static BT::PortList providePorts(){
            return {BT::InputPort<int>(consecutive_detections)}; 
        }

        BT::NodeStatus onStart() override{ 
            if (!client_ -> wait_for_action_server(std::chrono::seconds(2))){
                RCLCPP_ERROR(node->get_logger(), "TargetPose Action server not Available"); 
                return BT::NodeStatus::FAILURE; 
            }

            auto goal = TargetPoseAction::Goal(); 

            getInput("consecutive_detections", goal.consecutive_detections); 

            auto options = rclcpp_action::Client<TargetPoseAction>
                           ::SendGoalOptions(); 

            options.feedback_callback = std::bind(&GetTargetPose::feedback_callback, 
                                        this, std::placeholders::_1, 
                                              std::placeholders::_2);
                                              
            options.result_callback = std::bind(&GetTargetPose::result_feecback, 
                                      this, std::placeholders::_1, 
                                            std::placeholders::_2, 
                                            std::placeholders::_3); 

           future_goal_handle_ = client->async_send_goal(goal, options); 

           return BT::NodeStatus::RUNNING; 
        }

        BT::NodeStatus onRunning() override{
            if 
        }








}