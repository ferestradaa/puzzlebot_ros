// qr_detection_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "qr_detection.hpp"

class QrDetectionNode : public rclcpp::Node {
public:
    QrDetectionNode() : Node("qr_detection_node_test") {
        //qr_size_ = 0.10;
        qr_size_ = 0.035;
        qr_detection_ = std::make_unique<Qr_detection>(qr_size_, 3);

        rclcpp::QoS info_qos(10);
        info_qos.best_effort();

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", rclcpp::SensorDataQoS(), 
            std::bind(&QrDetectionNode::image_callback, this, std::placeholders::_1));

        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera/camera_info", info_qos, 
            std::bind(&QrDetectionNode::cam_info_callback, this, std::placeholders::_1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("qr_detection/pose", 10);

        RCLCPP_INFO(this->get_logger(), "QR Detection Node ready!");
    }

private:
    void cam_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        if (has_camera_info_) return;
        K_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
        dist_ = cv::Mat(1, (int)msg->d.size(), CV_64F, const_cast<double*>(msg->d.data())).clone();
        has_camera_info_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera info recievecd");
    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        if (!has_camera_info_) return;

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

        cv::Mat R;
        cv::Rodrigues(qr_result.rvec, R);
        double trace = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);
        double qw = std::sqrt(1.0 + trace) / 2.0;
        double qx = (R.at<double>(2, 1) - R.at<double>(1, 2)) / (4.0 * qw);
        double qy = (R.at<double>(0, 2) - R.at<double>(2, 0)) / (4.0 * qw);
        double qz = (R.at<double>(1, 0) - R.at<double>(0, 1)) / (4.0 * qw);

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = now();
        pose_msg.header.frame_id = "camera_color_optical_frame";
        pose_msg.pose.position.x = qr_result.tvec[0];
        pose_msg.pose.position.y = qr_result.tvec[1];
        pose_msg.pose.position.z = qr_result.tvec[2];
        pose_msg.pose.orientation.w = qw;
        pose_msg.pose.orientation.x = qx;
        pose_msg.pose.orientation.y = qy;
        pose_msg.pose.orientation.z = qz;

        pose_pub_->publish(pose_msg);


        /*
        RCLCPP_INFO(get_logger(), "QR: %s | x=%.3f y=%.3f z=%.3f | conf:%d",
            qr_result.data.c_str(),
            qr_result.tvec[0],
            qr_result.tvec[1],
            qr_result.tvec[2],
            qr_detection_->get_detection_count());
        */ 
        }

    std::unique_ptr<Qr_detection> qr_detection_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    cv::Mat K_, dist_;
    double qr_size_;
    bool has_camera_info_ = false;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QrDetectionNode>());
    rclcpp::shutdown();
    return 0;
}