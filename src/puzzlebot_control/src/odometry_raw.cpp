
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cmath>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/timer.hpp>


#include "puzzlebot_control/math_utils.hpp"


class OdometryNode : public rclcpp::Node{
    public:
        OdometryNode() : Node("odometry_raw"),
            r_(0.044), L_(0.19),
            x_(0.0), y_(0.0), theta_(0.0),
            wheel_vel_left_rads_(0.0), wheel_vel_right_rads_(0.0),
            last_time_(rclcpp::Time(0, 0, this->get_clock()->get_clock_type())){


        encl_sub_ = this -> create_subscription<std_msgs::msg::Float32>("/VelEncL", 10,
            std::bind(&OdometryNode::encoderL_callback, this, std::placeholders::_1));

        encr_sub_ = this -> create_subscription<std_msgs::msg::Float32>("/VelEncR", 10, 
        std::bind(&OdometryNode::encoderR_callback, this, std::placeholders::_1));

        timer_ = rclcpp::create_timer(
            this,
            this->get_clock(),
            rclcpp::Duration::from_seconds(0.025),//before it was at 0.05
            std::bind(&OdometryNode::publish_odometry, this));

        odom_raw_pub_ = this-> create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(this->get_logger(), "Reading encoder velocities");
        
    }

    private:
        void encoderL_callback(const std_msgs::msg::Float32::SharedPtr msg){
            wheel_vel_left_rads_ = msg -> data; 
        }

        void encoderR_callback(const std_msgs::msg::Float32::SharedPtr msg){
            wheel_vel_right_rads_ = msg -> data; 
        }


        void get_odom(const rclcpp::Time& now){
            double dt = (now - last_time_).seconds();
            last_time_ = now; 
            if (dt <= 0.0 || dt > 1.0) return; //avoid invalid dt 

            double vL = wheel_vel_left_rads_ * r_; //lineal velocity for each wheel
            double vR = wheel_vel_right_rads_ * r_; 

            double v_robot = (vR + vL) / 2.0;  //lineal velocity for the robot (part of twist message)
            double w_robot = (vR - vL) / L_;  //angular velocity for the whole robot (part of twist message)
            
            double d_translation = v_robot * dt; //distance instead of velocity
            double d_rot = w_robot * dt; //angular distance instead of velocity


            x_ += d_translation * std::cos(theta_ + d_rot / 2.0); // x pos of the robot
            y_ += d_translation * std::sin(theta_ + d_rot / 2.0);  // y pos of the robot
            theta_ += d_rot; //orientation
            theta_ = std::atan2(std::sin(theta_), std::cos(theta_)); 

            kalman_->predict(vL, vR, wheel_vel_left_rads_, wheel_vel_right_rads_, dt); //call kalman prediction uses lineal vel and angular vel

        }

        void publish_odometry(){

            auto stamp = this->get_clock()->now();
            get_odom(stamp);

            nav_msgs::msg::Odometry raw_msg;
            raw_msg.header.stamp    = stamp; 
            raw_msg.header.frame_id = "odom";
            raw_msg.child_frame_id  = "base_footprint";
            raw_msg.pose.pose.position.x = x_;
            raw_msg.pose.pose.position.y = y_;
            tf2::Quaternion q_raw;
            q_raw.setRPY(0.0, 0.0, theta_);
            raw_msg.pose.pose.orientation = tf2::toMsg(q_raw);
            odom_raw_pub_->publish(raw_msg);


            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp    = stamp;
            tf_msg.header.frame_id = "odom";
            tf_msg.child_frame_id  = "base_footprint";
            tf_msg.transform.translation.x = state(0);       // raw, no EKF
            tf_msg.transform.translation.y = state(1);
            tf_msg.transform.translation.z = 0.0;
            tf_msg.transform.rotation = tf2::toMsg(q);  // q_raw ya existe arriba

            tf_broadcaster_->sendTransform(tf_msg);
        }



        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_raw_pub_; 
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr encl_sub_; 
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr encr_sub_; 
        rclcpp::Time last_time_; 
        rclcpp::TimerBase::SharedPtr timer_;


        tf2_ros::Buffer tf_buffer_{this->get_clock()};
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        const double r_, L_;
        double wheel_vel_left_rads_, wheel_vel_right_rads_, x_, y_, theta_; 
        
}; 


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<OdometryNode>()); 
    rclcpp::shutdown(); 
    return 0; 
}