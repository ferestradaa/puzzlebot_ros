#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "qr_detection.hpp"
#include "puzzlebot_interfaces/action/qr_detect.hpp"

class QrDetectionActionServer : public rclcpp::Node {
public:
    using QrDetect = puzzlebot_interfaces::action::QrDetect;
    using GoalHandle = rclcpp_action::ServerGoalHandle<QrDetect>;

    QrDetectionActionServer()
    : Node("qr_detection_node") {
        
        qr_size_ = 0.10;
        
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        rclcpp::QoS info_qos(10);
        info_qos.best_effort();

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&QrDetectionActionServer::image_callback, this, std::placeholders::_1));

        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera/camera_info", info_qos,
            std::bind(&QrDetectionActionServer::cam_info_callback, this, std::placeholders::_1));

        action_server_ = rclcpp_action::create_server<QrDetect>(
            this,
            "qr_detection",
            std::bind(&QrDetectionActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&QrDetectionActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&QrDetectionActionServer::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "QR Detection Action Server ready!");
    }

private:

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const QrDetect::Goal> goal) {
        
        (void)uuid;

        if (active_goal_handle_) {
            RCLCPP_WARN(get_logger(), "Goal REJECTED: Server already has active goal");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (goal->consecutive_detections == 0) {
            RCLCPP_WARN(get_logger(), "Goal REJECTED: consecutive_detections must be > 0");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
        auto goal = goal_handle->get_goal();
        qr_detection_ = std::make_unique<Qr_detection>(qr_size_, goal->consecutive_detections);
        active_goal_handle_ = goal_handle;
    }

    void cam_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        if (has_camera_info_) return;
        K_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
        dist_ = cv::Mat(1, (int)msg->d.size(), CV_64F, const_cast<double*>(msg->d.data())).clone();
        has_camera_info_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera info recieved");
    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        if (!active_goal_handle_ || !has_camera_info_) return;

        auto goal_handle = active_goal_handle_;

        if (goal_handle->is_canceling()) {
            auto result = std::make_shared<QrDetect::Result>();
            result->success = false;
            goal_handle->canceled(result);
            active_goal_handle_ = nullptr;
            qr_detection_.reset();
            return;
        }

        cv::Mat frame;
        try {
            auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            frame = cv_ptr->image;
        } catch (cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
            return;
        }

        Qrpose qr_result = qr_detection_->estimate_qr_pose(frame, K_, dist_);

        if (!qr_result.valid) return;

        int current_count = qr_detection_->get_detection_count();
        
        auto feedback = std::make_shared<QrDetect::Feedback>();
        feedback->current_consecutive_detections = static_cast<uint32_t>(current_count);
        goal_handle->publish_feedback(feedback);

        auto goal = goal_handle->get_goal();
        if (current_count >= static_cast<int>(goal->consecutive_detections)) {
            
            cv::Mat R;
            cv::Rodrigues(qr_result.rvec, R);
            double trace = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);
            double qw = std::sqrt(1.0 + trace) / 2.0;
            double qx = (R.at<double>(2, 1) - R.at<double>(1, 2)) / (4.0 * qw);
            double qy = (R.at<double>(0, 2) - R.at<double>(2, 0)) / (4.0 * qw);
            double qz = (R.at<double>(1, 0) - R.at<double>(0, 1)) / (4.0 * qw);

            geometry_msgs::msg::PoseStamped pose_camera;
            pose_camera.header.stamp = now();
            pose_camera.header.frame_id = "camera_color_optical_frame";
            pose_camera.pose.position.x = qr_result.tvec[0];
            pose_camera.pose.position.y = qr_result.tvec[1];
            pose_camera.pose.position.z = qr_result.tvec[2];
            pose_camera.pose.orientation.w = qw;
            pose_camera.pose.orientation.x = qx;
            pose_camera.pose.orientation.y = qy;
            pose_camera.pose.orientation.z = qz;

            geometry_msgs::msg::PoseStamped pose_base_footprint;
            try {
                tf_buffer_->transform(pose_camera, pose_base_footprint, "base_footprint", tf2::durationFromSec(0.5));
            } catch (tf2::TransformException & ex) {
                RCLCPP_ERROR(get_logger(), "Transform failed: %s", ex.what());
                return;
            }

            auto result = std::make_shared<QrDetect::Result>();
            result->success = true;
            result->qr_data = qr_result.data;
            result->pose = pose_base_footprint;

            goal_handle->succeed(result);

            RCLCPP_INFO(get_logger(), "QR: %s | x=%.3f y=%.3f z=%.3f (base_footprint) | conf:%d",
                result->qr_data.c_str(),
                result->pose.pose.position.x,
                result->pose.pose.position.y,
                result->pose.pose.position.z,
                current_count);

            active_goal_handle_ = nullptr;
            qr_detection_.reset();
        }
    }

    std::unique_ptr<Qr_detection> qr_detection_;
    rclcpp_action::Server<QrDetect>::SharedPtr action_server_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    std::shared_ptr<GoalHandle> active_goal_handle_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    cv::Mat K_, dist_;
    double qr_size_;
    bool has_camera_info_ = false;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QrDetectionActionServer>());
    rclcpp::shutdown();
    return 0;
}