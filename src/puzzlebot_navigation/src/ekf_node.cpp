/*
This node wont be used you can ignore it

*/


#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cmath>

class KalmanFilterNode : public rclcpp::Node{
    public: 
        KalmanFilterNode() : Node("kalman_filter_node"){

        filt_odom_pub = this-> create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        
        RCLCPP_INFO(this->get_logger(), "Reading raw odometry");

        }

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr filt_odom_pub; 
        rclcpp::TimerBase::SharedPtr timer_;


};  



int main(int argc, char * argv[]){
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<KalmanFilterNode>()); 
    rclcpp::shutdown(); 
    return 0; 

}
