/*
To find odom:

1.puzzlebot publishes angular vel for each wheel, so first get lineal vel for each wheel too (how many m/s the wheels are moving)
which is v_wheel = w_wheel * radius 

2. Now get the linal vel but considering the whole robot, so promedio of both wheels
which is v = (v_L + v_R)/2

3. Now angular vel of the robot (yaw vel)
which is w = (v_R - v_L) / L

4. How much distance the robot has moved, basically d = v * dt
which is d_trans = v_robot * dt AND d_rot = w_robot *dt

5. Now, to where the robot has moved? 
which is: 
    x = d_trans * cos (theta + d_rot/2)
    y = d_trans * sin (theta + d_rot/2)
    theta = theta + d_rot


6. Normalize angle between -pi, pi
    theta = arctan2(sin(theta), cos(theta))

*/


#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cmath>
#include <kalman_filter.hpp>

class OdometryNode : public rclcpp::Node{
    public:
        OdometryNode() : Node("odometry"),
            r_(0.05), L_(0.19),
            x_(0.0), y_(0.0), theta_(0.0),
            wheel_vel_left_rads_(0.0), wheel_vel_right_rads_(0.0),
            last_time_(this->get_clock()->now()){


        encl_sub_ = this -> create_subscription<std_msgs::msg::Float32>("/VelEncL", 10,
            std::bind(&OdometryNode::encoderL_callback, this, std::placeholders::_1));

        encr_sub_ = this -> create_subscription<std_msgs::msg::Float32>("/VelEncR", 10, 
        std::bind(&OdometryNode::encoderR_callback, this, std::placeholders::_1));


        aruco_sub = this -> create_subscription<std_msgs::msg::Float32>("/VelEncR", 10, 
        std::bind(&OdometryNode::encoderR_callback, this, std::placeholders::_1));



        timer_ = this ->create_wall_timer(std::chrono::milliseconds(50), 
                std::bind(&OdometryNode::publish_odometry, this)); 

        odom_pub_ = this-> create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        kalman_ = std::make_unique<ExtendedKalmanFilter>(r_, L_);


        RCLCPP_INFO(this->get_logger(), "Reading encoder velocities");
        
    }
    private:

        void encoderL_callback(const std_msgs::msg::Float32::SharedPtr msg){
            wheel_vel_left_rads_ = msg -> data; 
        }

        void encoderR_callback(const std_msgs::msg::Float32::SharedPtr msg){
            wheel_vel_right_rads_ = msg -> data; 
        }

        void aruco_callback(const aruco_msgs::msg::MarkerArray::SharedPtr msg){

            for (const auto& marker :msg-> markers){
                auto iterator = landmark_map_.find(marker.id) //find() looks for the key in the map and returns where it is
                if (iterator == landmark_map.end()){ // if not, iterator is end() == NULL
                    continue; 
                auto landmark = it ->second; //landmark has the value if found

                

                }
                
            }

            }

            


        }



        void get_odom(){
            rclcpp::Time now = this ->get_clock()->now(); 
            double dt = (now - last_time_).seconds();
            last_time_ = now; 
            if (dt <= 0.0 || dt > 1.0) return; //avoid invalid dt 

            double vL = wheel_vel_left_rads_ * r_; //lineal velocity for each wheel
            double vR = wheel_vel_right_rads_ * r_; 

            //double v_robot = (vR + vL) / 2.0;  //lineal velocity for the robot (part of twist message)
            //double w_robot = (vR - vL) / L_;  //angular velocity for the whole robot (part of twist message)
            
            //double d_translation = v_robot * dt; //distance instead of velocity
            //double d_rot = w_robot * dt; //angular distance instead of velocity


            //x_ += d_translation * std::cos(theta_ + d_rot / 2.0); // x pos of the robot
            //y_ += d_translation * std::sin(theta_ + d_rot / 2.0);  // y pos of the robot
            //theta_ += d_rot; //orientation
            //theta_ = std::atan2(std::sin(theta_), std::cos(theta_)); 

            kalman_.predict(vL, vR, wheel_vel_left_rads_, wheel_vel_right_rads_, dt) //call kalman prediction uses lineal vel and angular vel

        }

        void 





        void publish_odometry(){
            get_odom(); 
            odom_pub_ -> publish(msg); 
        }


        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_; 
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr encl_sub_; 
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr encr_sub_; 
        rclcpp::Time last_time_; 
        rclcpp::TimerBase::SharedPtr timer_;
        std::unique_ptr<ExtendedKalmanFilter> kalman_; 

        const double r_, L_;
        double wheel_vel_left_rads_, wheel_vel_right_rads_, x_, y_, theta_; 
        aruco_msgs::msg::MarkerArray last_aruco_; 
        
}; 


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<OdometryNode>()); 
    rclcpp::shutdown(); 
    return 0; 
}