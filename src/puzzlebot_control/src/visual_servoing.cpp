#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

class VisualServoing : public rclcpp::Node{
    public: 
        VisualServoing()     : Node("visual_servoing"){

            bbox_sub_= create_subscription<vision_msgs::msg::Detection2DArray>("pallet_detections", 
                       10, std::bind(&VisualServoing::detections_callback, this, std::placeholders::_1)); 

            cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10); 

            RCLCPP_INFO(get_logger(), "Visual Servoing node initialized !"); 

            Kw_ = 0.3; 
            Kv_ = 0.3; 

        }
 
    private: 
        void detections_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
            if (msg->detections.empty()) {
                geometry_msgs::msg::Twist stop;
                cmd_pub_->publish(stop);
                return;
            }
            auto & det = msg->detections[0];
            double cx = det.bbox.center.position.x;
            double cy = det.bbox.center.position.y;
            double w = det.bbox.size_x;
            double h = det.bbox.size_y;
            compute_cmd(cx, cy, w, h);
        }

        void compute_cmd(double cx, double cy, double w, double h) {
            geometry_msgs::msg::Twist twist;
            double bbox_area = w * h;

            double cx_corrected = image_width_ - cx;

            double image_cx = image_width_ / 2.0;
            double ex = cx_corrected - image_cx;
            double ey = target_area_ - bbox_area;

            if (std::abs(ex) < 20.0) ex = 0.0;

            twist.angular.z = -Kw_ * ex / image_cx;
            twist.linear.x  =  Kv_ * ey / target_area_;
            twist.linear.x  = std::clamp(twist.linear.x, -0.05, 0.15);
            twist.angular.z = std::clamp(twist.angular.z, -0.3, 0.3);

            RCLCPP_INFO(get_logger(), "cx_raw=%.1f  cx_corr=%.1f  ex=%.1f  area=%.0f  vx=%.3f  wz=%.3f",
                        cx, cx_corrected, ex, bbox_area, twist.linear.x, twist.angular.z);

            cmd_pub_->publish(twist);
        }

    
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_; 
        rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr bbox_sub_;

        double Kw_; 
        double Kv_;
        double target_area_ = 28000.0; 
        double image_width_ = 640; 


}; 


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualServoing>());
    rclcpp::shutdown();
    return 0;
}