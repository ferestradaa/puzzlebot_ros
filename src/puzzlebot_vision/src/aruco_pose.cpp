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

        // Ajustar params para deteccion mas robusta
        detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        detector_params_->cornerRefinementWinSize = 5;
        detector_params_->minMarkerPerimeterRate = 0.03;
        detector_params_->maxMarkerPerimeterRate = 4.0;

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
                info_sub_.reset();
            });

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", qos,
            std::bind(&ArucoPoseNode::imageCallback, this, std::placeholders::_1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/aruco_pose", 10);
    }

private:
    // Verifica que las corners formen un cuadrilatero valido (no degenerado)
    bool cornersValid(const std::vector<cv::Point2f> &corners)
    {
        if (corners.size() != 4) return false;

        // Area minima en pixeles^2
        double area = std::abs(
            (corners[0].x * (corners[1].y - corners[3].y) +
             corners[1].x * (corners[2].y - corners[0].y) +
             corners[2].x * (corners[3].y - corners[1].y) +
             corners[3].x * (corners[0].y - corners[2].y)) * 0.5);

        if (area < 100.0) return false;

        // Verificar que ningun par de corners este demasiado cerca
        for (int i = 0; i < 4; ++i) {
            for (int j = i + 1; j < 4; ++j) {
                double dist = cv::norm(corners[i] - corners[j]);
                if (dist < 5.0) return false;
            }
        }

        return true;
    }

    cv::Vec4d rotMatToQuat(const cv::Mat &R)
    {
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

        return {qw, qx, qy, qz};
    }

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

        // Puntos 3D del marcador en coordenadas locales (origen en centro)
        float h = marker_length_ / 2.0f;
        std::vector<cv::Point3f> obj_points = {
            {-h,  h, 0},
            { h,  h, 0},
            { h, -h, 0},
            {-h, -h, 0}
        };

        for (size_t i = 0; i < ids.size(); ++i) {
            if (!cornersValid(corners[i])) {
                RCLCPP_WARN(this->get_logger(), "Marker %d: corners degeneradas, descartando", ids[i]);
                continue;
            }

            cv::Vec3d rvec, tvec;

            // SOLVEPNP_IPPE_SQUARE resuelve la ambiguedad de pose para marcadores cuadrados
            bool ok = cv::solvePnP(obj_points, corners[i],
                                   camera_matrix_, dist_coeffs_,
                                   rvec, tvec,
                                   false, cv::SOLVEPNP_IPPE_SQUARE);

            if (!ok) continue;

            // Descartar si z es fisicamente imposible (< 0 o > 10 m)
            if (tvec[2] < 0.01 || tvec[2] > 10.0) {
                RCLCPP_WARN(this->get_logger(), "Marker %d: z=%.3f fuera de rango, descartando", ids[i], tvec[2]);
                continue;
            }

            cv::Mat R;
            cv::Rodrigues(rvec, R);
            auto q = rotMatToQuat(R);

            geometry_msgs::msg::Pose pose;
            pose.position.x = tvec[0];
            pose.position.y = tvec[1];
            pose.position.z = tvec[2];
            pose.orientation.w = q[0];
            pose.orientation.x = q[1];
            pose.orientation.y = q[2];
            pose.orientation.z = q[3];
            pose_array.poses.push_back(pose);
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