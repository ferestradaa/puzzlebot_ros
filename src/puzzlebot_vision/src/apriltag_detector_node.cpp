#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#ifdef cv_bridge_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#ifdef TF2_ROS_HAS_BROADCASTER_HPP
#include <tf2_ros/transform_broadcaster.hpp>
#else
#include <tf2_ros/transform_broadcaster.h>
#endif

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <puzzlebot_interfaces/msg/april_tag_detection.hpp>
#include <puzzlebot_interfaces/msg/april_tag_detection_array.hpp>

#include <opencv2/opencv.hpp>

extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/apriltag_pose.h>
}

#include <array>
#include <string>
#include <memory>

class AprilTagDetector : public rclcpp::Node {
public:
    AprilTagDetector() : Node("apriltag_detector"),
        tf_buffer_(this->get_clock()),
        tf_broadcaster_(this)
    {
        this->declare_parameter("tag_size", 0.125);
        this->declare_parameter("base_frame", "base_link");
        tag_size_ = this->get_parameter("tag_size").as_double();
        base_frame_ = this->get_parameter("base_frame").as_string();

        // init apriltag detector
        tf_ = tag36h11_create();
        detector_ = apriltag_detector_create();
        apriltag_detector_add_family(detector_, tf_);
        detector_->nthreads = 2;
        detector_->quad_decimate = 1.0f;
        detector_->quad_sigma = 0.8f;
        detector_->refine_edges = 1;
        detector_->decode_sharpening = 0.25;

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", qos,
            std::bind(&AprilTagDetector::image_cb, this, std::placeholders::_1));

        sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera/camera_info", qos,
            std::bind(&AprilTagDetector::info_cb, this, std::placeholders::_1));

        pub_camera_ = this->create_publisher<puzzlebot_interfaces::msg::AprilTagDetectionArray>(
            "apriltag/camera_pose", 10);

        pub_base_ = this->create_publisher<puzzlebot_interfaces::msg::AprilTagDetectionArray>(
            "apriltag/base_pose", 10);

        RCLCPP_INFO(this->get_logger(), "apriltag_detector initialized");
    }

    ~AprilTagDetector()
    {
        apriltag_detector_destroy(detector_);
        tag36h11_destroy(tf_);
    }

private:
    void info_cb(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (camera_matrix_set_) return;

        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
        camera_matrix_set_ = true;

        RCLCPP_INFO(this->get_logger(), "camera info received");
    }

    // equivalent to python _get_extrinsic
    bool get_extrinsic(const std::string& camera_frame, geometry_msgs::msg::TransformStamped& out)
    {
        try {
            out = tf_buffer_.lookupTransform(
                base_frame_, camera_frame,
                tf2::TimePointZero,
                tf2::durationFromSec(0.05));
            return true;
        } catch (const tf2::TransformException& e) {
            RCLCPP_WARN(this->get_logger(), "no TF %s -> %s: %s",
                base_frame_.c_str(), camera_frame.c_str(), e.what());
            return false;
        }
    }

    void image_cb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!camera_matrix_set_) return;

        // rate limiter: 10 Hz equivalent
        double now = this->get_clock()->now().seconds();
        if (now - last_detection_time_ < 0.1) return;
        last_detection_time_ = now;

        geometry_msgs::msg::TransformStamped extrinsic;
        bool has_extrinsic = get_extrinsic(msg->header.frame_id, extrinsic);

        cv::Mat frame = cv_bridge::toCvShare(msg, "mono8")->image;

        image_u8_t img = { frame.cols, frame.rows, frame.cols, frame.data };

        zarray_t* detections = apriltag_detector_detect(detector_, &img);

        puzzlebot_interfaces::msg::AprilTagDetectionArray cam_array;
        cam_array.header = msg->header;

        puzzlebot_interfaces::msg::AprilTagDetectionArray base_array;
        base_array.header.stamp = msg->header.stamp;
        base_array.header.frame_id = base_frame_;

        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t* det;
            zarray_get(detections, i, &det);

            // estimate pose in camera frame
            apriltag_detection_info_t info;
            info.det     = det;
            info.tagsize = tag_size_;
            info.fx = fx_;
            info.fy = fy_;
            info.cx = cx_;
            info.cy = cy_;

            apriltag_pose_t pose;
            estimate_tag_pose(&info, &pose);

            geometry_msgs::msg::PoseStamped pose_cam;
            pose_cam.header = msg->header;
            pose_cam.pose.position.x = MATD_EL(pose.t, 0, 0);
            pose_cam.pose.position.y = MATD_EL(pose.t, 1, 0);
            pose_cam.pose.position.z = MATD_EL(pose.t, 2, 0);
            pose_cam.pose.orientation = mat_to_quat(pose.R);

            // broadcast tag TF in camera frame (same as python _broadcast_tf)
            broadcast_tf(pose_cam.pose, msg->header.stamp,
                         msg->header.frame_id, "tag_" + std::to_string(det->id));

            puzzlebot_interfaces::msg::AprilTagDetection cam_det;
            cam_det.tag_id = det->id;
            cam_det.pose   = pose_cam;
            cam_array.detections.push_back(cam_det);

            if (has_extrinsic) {
                geometry_msgs::msg::PoseStamped pose_base;
                // equivalent to tf2_geometry_msgs.do_transform_pose
                tf2::doTransform(pose_cam, pose_base, extrinsic);
                pose_base.header.stamp    = msg->header.stamp;
                pose_base.header.frame_id = base_frame_;

                puzzlebot_interfaces::msg::AprilTagDetection base_det;
                base_det.tag_id = det->id;
                base_det.pose   = pose_base;
                base_array.detections.push_back(base_det);
            }

            matd_destroy(pose.R);
            matd_destroy(pose.t);
        }

        pub_camera_->publish(cam_array);
        if (!base_array.detections.empty()) {
            pub_base_->publish(base_array);
        }

        apriltag_detections_destroy(detections);
    }

    void broadcast_tf(const geometry_msgs::msg::Pose& pose,
                      const rclcpp::Time& stamp,
                      const std::string& parent_frame,
                      const std::string& child_frame)
    {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp    = stamp;
        tf_msg.header.frame_id = parent_frame;
        tf_msg.child_frame_id  = child_frame;
        tf_msg.transform.translation.x = pose.position.x;
        tf_msg.transform.translation.y = pose.position.y;
        tf_msg.transform.translation.z = pose.position.z;
        tf_msg.transform.rotation      = pose.orientation;
        tf_broadcaster_.sendTransform(tf_msg);
    }

    // equivalent to python mat_to_quat
    geometry_msgs::msg::Quaternion mat_to_quat(const matd_t* R)
    {
        double r00 = MATD_EL(R,0,0), r01 = MATD_EL(R,0,1), r02 = MATD_EL(R,0,2);
        double r10 = MATD_EL(R,1,0), r11 = MATD_EL(R,1,1), r12 = MATD_EL(R,1,2);
        double r20 = MATD_EL(R,2,0), r21 = MATD_EL(R,2,1), r22 = MATD_EL(R,2,2);

        double trace = r00 + r11 + r22;
        geometry_msgs::msg::Quaternion q;

        if (trace > 0.0) {
            double s = 0.5 / std::sqrt(trace + 1.0);
            q.w = 0.25 / s;
            q.x = (r21 - r12) * s;
            q.y = (r02 - r20) * s;
            q.z = (r10 - r01) * s;
        } else if (r00 > r11 && r00 > r22) {
            double s = 2.0 * std::sqrt(1.0 + r00 - r11 - r22);
            q.w = (r21 - r12) / s;
            q.x = 0.25 * s;
            q.y = (r01 + r10) / s;
            q.z = (r02 + r20) / s;
        } else if (r11 > r22) {
            double s = 2.0 * std::sqrt(1.0 + r11 - r00 - r22);
            q.w = (r02 - r20) / s;
            q.x = (r01 + r10) / s;
            q.y = 0.25 * s;
            q.z = (r12 + r21) / s;
        } else {
            double s = 2.0 * std::sqrt(1.0 + r22 - r00 - r11);
            q.w = (r10 - r01) / s;
            q.x = (r02 + r20) / s;
            q.y = (r12 + r21) / s;
            q.z = 0.25 * s;
        }
        return q;
    }

    apriltag_family_t*   tf_       = nullptr;
    apriltag_detector_t* detector_ = nullptr;

    tf2_ros::Buffer                              tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>  tf_listener_;
    tf2_ros::TransformBroadcaster                tf_broadcaster_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      sub_image_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;

    rclcpp::Publisher<puzzlebot_interfaces::msg::AprilTagDetectionArray>::SharedPtr pub_camera_;
    rclcpp::Publisher<puzzlebot_interfaces::msg::AprilTagDetectionArray>::SharedPtr pub_base_;

    bool   camera_matrix_set_ = false;
    double fx_ = 0.0, fy_ = 0.0, cx_ = 0.0, cy_ = 0.0;
    double tag_size_           = 0.125;
    std::string base_frame_    = "base_link";
    double last_detection_time_ = 0.0;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AprilTagDetector>());
    rclcpp::shutdown();
    return 0;
}