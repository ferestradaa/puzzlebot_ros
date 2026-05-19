#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <puzzlebot_vision/qr_detection.hpp>
#include <opencv2/opencv.hpp>


class QrDetection : public rclcpp::Node{
    public:
        QrDetection()
        : Node("qr_detection_node"){

            rclcpp::QoS info_qos(10);
            info_qos.reliable();

            qr_size_= 0.15; //size in meters

            qr_detection_ = std::make_unique<QrDetection>(qr_size_);

            service_= this -> create_service<std_srvs::srv::SetBool>(
                "enable_qr_detection_service", 
                std::bind(
                    &QrDetection::enable_qr_detection_callback, 
                    this, 
                    std::placeholders::_1, 
                    std::placeholders::_2
                )
            ); 

            image_sub_ = this -> create_subscription<sensor_msgs::msg::Image>(
                "/camera/image_raw", rclcpp::SensorDataQoS(), 
                std::bind(&QrDetection::image_callback, this, std::placeholders::_1)); 


            cam_info_sub = this -> create_subscription<sensor_msgs::msg::CameraInfo>(
                "camera/camera_info", info_qos, 
                std::bind(&QrDetection::cam_info_callback, this, std::placeholders::_1)); 

            RCLCPP_INFO(this->get_logger(), "QR Detector Node ready!"); 
        }


    private: 

        void enable_qr_detection_service(const std::shared_ptr<std_srvs::srv::SetBool::Request> request
              std::shared_ptr<std_srvs::srv::SerBool::Response> response){
                enabled_ request -> data; 
                response->success = true; 
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

            auto pose = qr_detection_ -> estimate_qr_pose(frame, K, dist, qr_size); 
        }
        
        

    std::unique_ptr<QrDetection> qr_detection_; 

}; 


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<Qr_detection>()); 
    rclcpp::shutdown(); 
    return 0; 
}
