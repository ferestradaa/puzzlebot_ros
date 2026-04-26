#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rclcpp/qos.hpp>

class EncoderNode : public rclcpp::Node{
    public:
        EncoderNode(): Node("joint_state_pub"){

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        js_pub_ = this-> create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        encl_sub_ = this -> create_subscription<std_msgs::msg::Float32>("VelocityEncL", qos,
            std::bind(&EncoderNode::encoderL_callback, this, std::placeholders::_1));

        encr_sub_ = this -> create_subscription<std_msgs::msg::Float32>("VelocityEncR", qos, 
        std::bind(&EncoderNode::encoderR_callback, this, std::placeholders::_1));

        timer_ = this ->create_wall_timer(std::chrono::milliseconds(50), //timer for publishing joint states
                std::bind(&EncoderNode::publish_joint_states, this)); 

        left_vel_ = 0.0;
        right_vel_ = 0.0;
        left_pos_ = 0.0;
        right_pos_ = 0.0; 
        last_time_ = this ->get_clock() -> now(); 


        RCLCPP_INFO(this->get_logger(), "Reading encoder velocities");
        
    }
    private:
        void encoderL_callback(const std_msgs::msg::Float32::SharedPtr msg){
            left_vel_ = msg -> data; 
        }

        void encoderR_callback(const std_msgs::msg::Float32::SharedPtr msg){
            right_vel_ = msg -> data; 
        }

        void integrate_positions(){
            rclcpp::Time now = this ->get_clock()->now(); 
            double dt = (now - last_time_).seconds();
            last_time_ = now; 
            
            left_pos_ += left_vel_ * dt; 
            right_pos_ += right_vel_ *dt; 
        }


        void publish_joint_states(){
            integrate_positions(); 

            sensor_msgs::msg::JointState msg; 
            msg.header.stamp = this -> get_clock() -> now(); 
            msg.name = {"base_to_left_wheel", "base_to_right_wheel"}; 
            msg.position = {left_pos_, right_pos_}; 

            js_pub_ -> publish(msg); 
        }


        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_; 
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr encl_sub_; 
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr encr_sub_; 
        rclcpp::Time last_time_; 
        rclcpp::TimerBase::SharedPtr timer_;
        double left_pos_, right_pos_, left_vel_, right_vel_; 
        
}; 


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<EncoderNode>()); 
    rclcpp::shutdown(); 
    return 0; 
}
