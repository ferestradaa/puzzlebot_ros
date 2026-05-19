#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <cv_bridge/cv_bridge.h>

#include "qr_detection.hpp"
#include <opencv2/opencv.hpp>


class QrDetection : public rclcpp::Node{
    public:
        QrDetection()
        : Node("qr_detection_node"){

            rclcpp::QoS info_qos(10);
            info_qos.reliable();

            qr_size_= 0.15; //size in meters

            qr_detection_ = std::make_unique<Qr_detection>(qr_size_);

            service_= this -> create_service<std_srvs::srv::SetBool>(
                "enable_qr_detection_service", 
                std::bind(
                    &QrDetection::enable_qr_detection_service, 
                    this, 
                    std::placeholders::_1, 
                    std::placeholders::_2
                )
            ); 

            image_sub_ = this -> create_subscription<sensor_msgs::msg::Image>(
                "/camera/image_raw", rclcpp::SensorDataQoS(), 
                std::bind(&QrDetection::image_callback, this, std::placeholders::_1)); 


            cam_info_sub_ = this -> create_subscription<sensor_msgs::msg::CameraInfo>(
                "camera/camera_info", info_qos, 
                std::bind(&QrDetection::cam_info_callback, this, std::placeholders::_1)); 

            RCLCPP_INFO(this->get_logger(), "QR Detector Node ready!"); 
        }


    private: 

        void enable_qr_detection_service(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, 
              std::shared_ptr<std_srvs::srv::SetBool::Response> response){
                enabled_ =  request -> data; 
                response->success = true; 
        }

        void cam_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
            if (has_camera_info_) return;
            K_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
            dist_ = cv::Mat(1, (int)msg->d.size(), CV_64F, const_cast<double*>(msg->d.data())).clone();
            has_camera_info_ = true;
        }

        void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg){
            if (!enabled_){
                return; }

            cv::Mat frame;

            try {
                auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            frame = cv_ptr->image;
            } 
            catch (cv_bridge::Exception & e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
            return;
            }

            auto pose = qr_detection_ -> estimate_qr_pose(frame, K_, dist_); 
        }
        
        

    std::unique_ptr<Qr_detection> qr_detection_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    cv::Mat K_, dist_;
    double qr_size_;
    bool enabled_ = false;
    bool has_camera_info_ = false;

}; 


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<QrDetection>()); 
    rclcpp::shutdown(); 
    return 0; 
}
