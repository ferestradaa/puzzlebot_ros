#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>


class ArucoPoseNode : public rclcpp::Node
{
public:
    ArucoPoseNode() : Node("aruco_pose_node"), camera_ready_(false)
    {
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        detector_params_ = cv::aruco::DetectorParameters::create();
        marker_length_ = 0.10f;
        auto qos = rclcpp::QoS(10).best_effort();

        info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera/camera_info", qos,
            [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                camera_matrix_ = (cv::Mat_<double>(3, 3) <<
                    msg->k[0], msg->k[1], msg->k[2],
                    msg->k[3], msg->k[4], msg->k[5],
                    msg->k[6], msg->k[7], msg->k[8]);
                dist_coeffs_ = cv::Mat(msg->d);
                camera_ready_ = true;
                info_sub_.reset(); // desuscribirse, ya no se necesita
            });

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", qos,
            std::bind(&ArucoPoseNode::imageCallback, this, std::placeholders::_1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/aruco_pose", 10);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
    {
        if (!camera_ready_) return;

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
            return;
        }

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(cv_ptr->image, dictionary_, corners, ids, detector_params_);

        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header = img_msg->header;

        if (!ids.empty()) {
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(
                corners, marker_length_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

            for (size_t i = 0; i < ids.size(); ++i) {
                cv::Mat R;
                cv::Rodrigues(rvecs[i], R);

                double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);
                double qw, qx, qy, qz;

                if (trace > 0) {
                    double s = 0.5 / std::sqrt(trace + 1.0);
                    qw = 0.25 / s;
                    qx = (R.at<double>(2,1) - R.at<double>(1,2)) * s;
                    qy = (R.at<double>(0,2) - R.at<double>(2,0)) * s;
                    qz = (R.at<double>(1,0) - R.at<double>(0,1)) * s;
                } else if (R.at<double>(0,0) > R.at<double>(1,1) && R.at<double>(0,0) > R.at<double>(2,2)) {
                    double s = 2.0 * std::sqrt(1.0 + R.at<double>(0,0) - R.at<double>(1,1) - R.at<double>(2,2));
                    qw = (R.at<double>(2,1) - R.at<double>(1,2)) / s;
                    qx = 0.25 * s;
                    qy = (R.at<double>(0,1) + R.at<double>(1,0)) / s;
                    qz = (R.at<double>(0,2) + R.at<double>(2,0)) / s;
                } else if (R.at<double>(1,1) > R.at<double>(2,2)) {
                    double s = 2.0 * std::sqrt(1.0 + R.at<double>(1,1) - R.at<double>(0,0) - R.at<double>(2,2));
                    qw = (R.at<double>(0,2) - R.at<double>(2,0)) / s;
                    qx = (R.at<double>(0,1) + R.at<double>(1,0)) / s;
                    qy = 0.25 * s;
                    qz = (R.at<double>(1,2) + R.at<double>(2,1)) / s;
                } else {
                    double s = 2.0 * std::sqrt(1.0 + R.at<double>(2,2) - R.at<double>(0,0) - R.at<double>(1,1));
                    qw = (R.at<double>(1,0) - R.at<double>(0,1)) / s;
                    qx = (R.at<double>(0,2) + R.at<double>(2,0)) / s;
                    qy = (R.at<double>(1,2) + R.at<double>(2,1)) / s;
                    qz = 0.25 * s;
                }

                geometry_msgs::msg::Pose pose;
                pose.position.x = tvecs[i][0];
                pose.position.y = tvecs[i][1];
                pose.position.z = tvecs[i][2];
                pose.orientation.w = qw;
                pose.orientation.x = qx;
                pose.orientation.y = qy;
                pose.orientation.z = qz;
                pose_array.poses.push_back(pose);
            }
        }

        pose_pub_->publish(pose_array);
    }

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    float marker_length_;
    cv::Mat camera_matrix_, dist_coeffs_;
    bool camera_ready_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoPoseNode>());
    rclcpp::shutdown();
    return 0;
}