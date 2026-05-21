#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp/action_node.h>

#include "puzzlebot_interfaces/action/qr_detect.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

class QrDetectionClient : public BT::StatefulActionNode{
    public: 

        using QrDetect = puzzlebot_interfaces::action::QrDetect; 

        using GoalHandle = rclcpp_action::ClientGoalHandle<QrDetect>; 


        QrDetectionClient(const std::string& name, const BT::NodeConfig& config, 
                    rclcpp::Node::SharedPtr node)
            : BT::StatefulActionNode(name, config), node_(node){

                client_ = rclcpp_action::create_client<QrDetect>(
                    node_, "qr_detection_action"
                );

                pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
                    "qr_detection/pose", 10);
            }

        static BT::PortsList providePorts(){
            return {
                BT::InputPort<int>("consecutive_detections"), 
                BT::InputPort<double>("pos_tolerance"), 
                BT::InputPort<double>("yaw_tolerance"), 
                BT::OutputPort<geometry_msgs::msg::PoseStamped>("qr_pose"), 
                BT::OutputPort<std::string>("qr_data") 
            }; 
        }

        BT::NodeStatus onStart() override{ 
            if (!client_ -> wait_for_action_server(std::chrono::seconds(2))){
                RCLCPP_ERROR(node_->get_logger(), "TargetPose Action server not Available"); 
                return BT::NodeStatus::FAILURE; 
            }

            auto goal = QrDetect::Goal(); 

            getInput("consecutive_detections", goal.consecutive_detections); 
            getInput("pos_tolerance", goal.pos_tolerance); 
            getInput("yaw_tolerance", goal.yaw_tolerance); 

            goal_done_ = false; 
            goal_succeeded_ = false; 

            auto options = rclcpp_action::Client<QrDetect>::SendGoalOptions(); 
            options.feedback_callback = std::bind(
                    &QrDetectionClient::feedback_callback, this, 
                    std::placeholders::_1, std::placeholders::_2);
                                              
            options.result_callback = std::bind(&QrDetectionClient::result_callback, 
                                      this, std::placeholders::_1); 


           future_goal_handle_ = client_->async_send_goal(goal, options); 
           return BT::NodeStatus::RUNNING; 
        }



        BT::NodeStatus onRunning() override{
            if (future_goal_handle_.valid() &&
                future_goal_handle_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready){
                    return BT::NodeStatus::RUNNING; 
                }

            if (!goal_done_){
                return BT::NodeStatus::RUNNING; 
            }


            if (goal_succeeded_){
                setOutput("qr_pose", result_pose); 
                setOutput("qr_data", result_data_);
                pose_pub_->publish(result_pose);
                return BT::NodeStatus::SUCCESS; 
            }


            return BT::NodeStatus::FAILURE;
        }

        

        void onHalted() override{
            if (future_goal_handle_.valid() &&
                future_goal_handle_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready){
                    auto gh = future_goal_handle_.get(); 
                    if (gh){
                        client_->async_cancel_goal(gh); 
                    }
                }

            goal_done_ = false; 
            goal_succeeded_ = false; 
        }


        private:
            void feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const QrDetect::Feedback> feedback){
                    RCLCPP_INFO(node_ -> get_logger(), "Consecutive detections: %u", 
                                feedback->current_consecutive_detections); 
                }



            void result_callback(const GoalHandle::WrappedResult& wrapped_result){
                    if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                        wrapped_result.result->success){
                            result_pose = wrapped_result.result->pose; 
                            result_data_ = wrapped_result.result->qr_data; 
                            goal_succeeded_ = true;
                        } else{
                            goal_succeeded_ = false; 
                        }

                        goal_done_ = true; 
            }

            

            rclcpp::Node::SharedPtr node_; 
            rclcpp_action::Client<QrDetect>::SharedPtr client_; 
            std::shared_future<GoalHandle::SharedPtr> future_goal_handle_;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

            std::atomic<bool> goal_done_{false}; 
            std::atomic<bool> goal_succeeded_{false}; 
            geometry_msgs::msg::PoseStamped result_pose; 
            std::string result_data_; 

    };