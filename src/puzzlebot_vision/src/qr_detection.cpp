#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "qr_detection.hpp"
#include "puzzlebot_control/pose_validator.hpp"

#include "puzzlebot_interfaces/action/qr_detect.hpp"


class QrDetectionActionServer : public rclcpp::Node{
    public:
        using QrDetect = puzzlebot_interfaces::action::QrDetect; 
        using GoalHandle = rclcpp_action::ServerGoalHandle<QrDetect>; 
    
        QrDetectionActionServer()
        : Node("qr_detection_node"){


            qr_size_= 0.15; //size in meters of qr code
            qr_detection_ = std::make_unique<Qr_detection>(qr_size_); //instance class using size

            rclcpp::QoS info_qos(10);
            info_qos.reliable();


            image_sub_ = this -> create_subscription<sensor_msgs::msg::Image>(
                "/camera/image_raw", rclcpp::SensorDataQoS(), 
                std::bind(&QrDetectionActionServer::image_callback, this, std::placeholders::_1)); 


            cam_info_sub_ = this -> create_subscription<sensor_msgs::msg::CameraInfo>(
                "camera/camera_info", info_qos, 
                std::bind(&QrDetectionActionServer::cam_info_callback, this, std::placeholders::_1)); 


            action_server_ = rclcpp_action::create_server<QrDetect>(
                this,
                "qr_detection", 
                std::bind(&QrDetectionActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&QrDetectionActionServer::handle_cancel, this, std::placeholders::_1),
                std::bind(&QrDetectionActionServer::handle_accepted, this, std::placeholders::_1));


            RCLCPP_INFO(this->get_logger(), "QR Detector Action Server ready!"); 
        }


    private: 

        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid, 
            std::shared_ptr<const QrDetect::Goal > goal){

                (void)uuid;

                if (active_goal_handle_){ //if client sending more than 1 target reject
                    RCLCPP_WARN(get_logger(), "Goal REJECTED: Server already has a goal"); 
                    return rclcpp_action::GoalResponse::REJECT; 
                }

                if (goal -> consecutive_detections == 0){ //if client is asking for 0 consecutive fraames, reject
                    RCLCPP_WARN(get_logger(), "Goal REJECTED: Number of detections must be more than 0"); 
                    return rclcpp_action::GoalResponse::REJECT; 
                }

                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; 
            }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>){
            return rclcpp_action::CancelResponse::ACCEPT; 
        }


        void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle){
            auto goal = goal_handle-> get_goal(); //for every goal recievecd: 
            pose_validator_ = std::make_unique<PoseValidator>( //instance the validation class with the goal values
                goal -> pos_tolerance, 
                goal -> yaw_tolerance, 
                static_cast<int>(goal->consecutive_detections)); 
            active_goal_handle_ = goal_handle; 
        }
        

        void cam_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
            if (has_camera_info_) return;
            K_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
            dist_ = cv::Mat(1, (int)msg->d.size(), CV_64F, const_cast<double*>(msg->d.data())).clone();
            has_camera_info_ = true;
        }

        void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg){
            if (!active_goal_handle_ || !has_camera_info_) return; //the action autm skipped if there isnt goal or camera info 

            auto goal_handle = active_goal_handle_; 

            if (goal_handle->is_canceling()){ //if goal canceled 
                auto result = std::make_shared<QrDetect::Result>(); 
                result -> success = false; 
                goal_handle -> canceled(result); 
                active_goal_handle_ = nullptr; 
                pose_validator_.reset(); //clear validadted pose
                return; 
            }

            cv::Mat frame;

            try {
                auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            frame = cv_ptr->image;
            } 
            catch (cv_bridge::Exception & e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
            return;
            }

            Qrpose qr_result = qr_detection_ -> estimate_qr_pose(frame, K_, dist_); 

            if (!qr_result.valid) return;

            cv::Mat R;
            cv::Rodrigues(qr_result.rvec, R);
            double yaw = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));

            Pose current_pose;
            current_pose.x = qr_result.tvec[0];
            current_pose.y = qr_result.tvec[1];
            current_pose.z = qr_result.tvec[2];
            current_pose.roll = 0.0;
            current_pose.pitch = 0.0;
            current_pose.yaw = yaw;

            PoseValidationResult validation = pose_validator_->validate(current_pose);

            auto feedback = std::make_shared<QrDetect::Feedback>(); 
            feedback->current_consecutive_detections = static_cast<uint32_t>(validation.count); 
            goal_handle -> publish_feedback(feedback); 

            if (validation.final_pose.has_value()){ //if counter reached setpoint so it has final pose 
                auto result = std::make_shared<QrDetect::Result>(); 
                result -> success = true; 
                result -> qr_data = qr_result.data; 

                auto & p = validation.final_pose.value(); 
                result -> pose.header.stamp = now(); 
                result -> pose.header.frame_id = "camera_color_optical_frame"; 
                result -> pose.pose.position.x = p.x; 
                result -> pose.pose.position.y = p.y; 
                result -> pose.pose.position.z = p.z; 
                result -> pose.pose.orientation.w = std::cos(p.yaw / 2.0); 
                result -> pose.pose.orientation.z = std::sin(p.yaw / 2.0); 
                
                goal_handle -> succeed(result); //reset everything after sending result 
                active_goal_handle_ = nullptr; 
                pose_validator_.reset(); 
            }

        }
        
        

    std::unique_ptr<Qr_detection> qr_detection_;
    std::unique_ptr<PoseValidator> pose_validator_; 
    rclcpp_action::Server<QrDetect>::SharedPtr action_server_; 
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    std::shared_ptr<GoalHandle> active_goal_handle_; 
    cv::Mat K_, dist_;
    double qr_size_;
    bool has_camera_info_ = false;

}; 


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<QrDetectionActionServer>()); 
    rclcpp::shutdown(); 
    return 0; 
}
