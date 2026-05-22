#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "qr_detection.hpp"
class QrDetectionNode : public rclcpp::Node{
    public:
        QrDetectionNode()
        : Node("qr_detection_node_test"){
            qr_size_= 0.10;
            qr_detection_ = std::make_unique<Qr_detection>(qr_size_);
            rclcpp::QoS info_qos(10);
            info_qos.best_effort();
            image_sub_ = this -> create_subscription<sensor_msgs::msg::Image>(
                "/camera/image_raw", rclcpp::SensorDataQoS(), 
                std::bind(&QrDetectionNode::image_callback, this, std::placeholders::_1)); 
            cam_info_sub_ = this -> create_subscription<sensor_msgs::msg::CameraInfo>(
                "camera/camera_info", info_qos, 
                std::bind(&QrDetectionNode::cam_info_callback, this, std::placeholders::_1)); 
            pose_pub_ = this -> create_publisher<geometry_msgs::msg::PoseStamped>("qr_detection/pose", 10);
            RCLCPP_INFO(this->get_logger(), "QR Detection Node ready!"); 
        }
    private: 
        void cam_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
            if (has_camera_info_) return;
            K_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
            dist_ = cv::Mat(1, (int)msg->d.size(), CV_64F, const_cast<double*>(msg->d.data())).clone();
            has_camera_info_ = true;
        }
        void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg){
            if (!has_camera_info_) return;
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
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = now();
            pose_msg.header.frame_id = "camera_color_optical_frame";
            pose_msg.pose.position.x = qr_result.tvec[0];
            pose_msg.pose.position.y = qr_result.tvec[1];
            pose_msg.pose.position.z = qr_result.tvec[2];
            pose_msg.pose.orientation.w = std::cos(yaw / 2.0);
            pose_msg.pose.orientation.z = std::sin(yaw / 2.0);
            pose_pub_ -> publish(pose_msg);
            RCLCPP_INFO(get_logger(), "QR detected: data='%s' x=%.3f y=%.3f z=%.3f yaw=%.3f",
                qr_result.data.c_str(),
                qr_result.tvec[0],
                qr_result.tvec[1],
                qr_result.tvec[2],
                yaw);
        }
        
    std::unique_ptr<Qr_detection> qr_detection_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    cv::Mat K_, dist_;
    double qr_size_;
    bool has_camera_info_ = false;
}; 
int main(int argc, char * argv[]){
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<QrDetectionNode>()); 
    rclcpp::shutdown(); 
    return 0; 
}