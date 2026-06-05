#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include "qr_detection.hpp"

class QrDetectionNode : public rclcpp::Node {
public:
    QrDetectionNode() : Node("qr_detection_node") {
        qr_size_ = 0.035;
        qr_detection_ = std::make_unique<Qr_detection>(qr_size_, 3);

        rclcpp::QoS info_qos(10);
        info_qos.best_effort();

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&QrDetectionNode::image_callback, this, std::placeholders::_1));

        cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera/camera_info", info_qos,
            std::bind(&QrDetectionNode::cam_info_callback, this, std::placeholders::_1));

        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("qr_detection/pose", 10);

        RCLCPP_INFO(get_logger(), "QR Detection Node ready");
    }

private:
    void cam_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        if (has_camera_info_) return;
        K_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
        dist_ = cv::Mat(1, (int)msg->d.size(), CV_64F, const_cast<double*>(msg->d.data())).clone();
        has_camera_info_ = true;
        RCLCPP_INFO(get_logger(), "Camera info received");
    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        if (!has_camera_info_) return;

        rclcpp::Time now_t = now();
        if ((now_t - last_detection_time_).seconds() < 0.1)
            return;
        last_detection_time_ = now_t;

        cv::Mat frame;
        try {
            frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
            return;
        }

        Qrpose qr_result = qr_detection_->estimate_qr_pose(frame, K_, dist_);
        if (!qr_result.valid) return;

        cv::Mat R;
        cv::Rodrigues(qr_result.rvec, R);

        Eigen::Matrix3d Re;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                Re(i, j) = R.at<double>(i, j);

        Eigen::Quaterniond q(Re);
        q.normalize();

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = msg->header.stamp;
        pose_msg.header.frame_id = "camera_color_optical_frame";
        pose_msg.pose.position.x = qr_result.tvec[0];
        pose_msg.pose.position.y = qr_result.tvec[1];
        pose_msg.pose.position.z = qr_result.tvec[2];
        pose_msg.pose.orientation.w = q.w();
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();

        pose_pub_->publish(pose_msg);
    }

    std::unique_ptr<Qr_detection> qr_detection_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    cv::Mat K_, dist_;
    double qr_size_;
    bool has_camera_info_ = false;
    rclcpp::Time last_detection_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QrDetectionNode>());
    rclcpp::shutdown();
    return 0;
}