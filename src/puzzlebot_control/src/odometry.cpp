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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/timer.hpp>

#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

#include "puzzlebot_control/kalman_filter.hpp"
#include "puzzlebot_control/math_utils.hpp"

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>


class OdometryNode : public rclcpp::Node{
    public:
        OdometryNode() : Node("odometry"),
            r_(0.044), L_(0.19),
            x_(0.0), y_(0.0), theta_(0.0),
            wheel_vel_left_rads_(0.0), wheel_vel_right_rads_(0.0),
            last_time_(rclcpp::Time(0, 0, this->get_clock()->get_clock_type())){


        encl_sub_ = this -> create_subscription<std_msgs::msg::Float32>("/VelEncL", 10,
            std::bind(&OdometryNode::encoderL_callback, this, std::placeholders::_1));

        encr_sub_ = this -> create_subscription<std_msgs::msg::Float32>("/VelEncR", 10, 
        std::bind(&OdometryNode::encoderR_callback, this, std::placeholders::_1));


        aruco_sub = this -> create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>("/detections", 10, //this has to be aruco detection in 
        std::bind(&OdometryNode::aruco_callback, this, std::placeholders::_1)); 

        timer_ = rclcpp::create_timer(
            this,
            this->get_clock(),
            rclcpp::Duration::from_seconds(0.025),//before it was at 0.05
            std::bind(&OdometryNode::publish_odometry, this));

        odom_pub_ = this-> create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        odom_raw_pub_ = this-> create_publisher<nav_msgs::msg::Odometry>("/odom_raw", 10);

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        Eigen::Matrix3d P0 = Eigen::Matrix3d::Identity() * 0.001;

        kalman_ = std::make_unique<ExtendedKalmanFilter>(r_, L_, Eigen::Vector3d::Zero(), P0);

        std::string pkg_path = ament_index_cpp::get_package_share_directory("puzzlebot_control");
        std::string map_path;
        this->declare_parameter("landmark_map_path", pkg_path + "/config/fixed_apriltags.yaml");
        this->get_parameter("landmark_map_path", map_path);
        loadLandmarkMap(map_path);

        RCLCPP_INFO(this->get_logger(), "Reading encoder velocities");
        
    }

    private:
        void encoderL_callback(const std_msgs::msg::Float32::SharedPtr msg){
            wheel_vel_left_rads_ = msg -> data; 
        }

        void encoderR_callback(const std_msgs::msg::Float32::SharedPtr msg){
            wheel_vel_right_rads_ = msg -> data; 
        }

        void aruco_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg){
            std::vector<Eigen::Vector2d> detected_positions; //list for xyyaw detected landmarks
            std::vector<Eigen::Vector2d> fixed_positions; //list for xyyaw known landmakrs in map

            rclcpp::Time detection_time = msg->header.stamp;


            for (const auto& marker :msg->detections){ //using every detection in frame

                auto iterator = landmark_map_.find(marker.id); //find() looks for the key in the map and returns where it is
                if (iterator == landmark_map_.end()){ // if not, iterator is end() == NULL
                    continue; }
                auto landmark = iterator->second; //landmark has the value if found
                std::string frame = "apriltag" + std::to_string(marker.id); //creates frame with detected landamark for tf

                try {
                    auto tf = tf_buffer_.lookupTransform(
                        "base_link",
                        frame, //used the detected frame
                        tf2::TimePointZero,
                        tf2::durationFromSec(0.1)
                    );

                    double x_detected = tf.transform.translation.x;
                    double y_detected = tf.transform.translation.y;
                    math_utils::Quaternion q{
                        tf.transform.rotation.x,
                        tf.transform.rotation.y,
                        tf.transform.rotation.z,
                        tf.transform.rotation.w
                    };

                    double yaw_detected = math_utils::getYaw(q);

                    fixed_positions.push_back(landmark); //once both lists have been validated, push them back
                    detected_positions.push_back(Eigen::Vector2d(x_detected, y_detected));

                } catch (tf2::TransformException &ex) {
                    continue; }
            }
                for (size_t i = 0; i < fixed_positions.size(); i++){
                    kalman_->update(
                        fixed_positions[i],    // xy pf world landmark 
                        detected_positions[i]  // xy of runtime detection
                    );
                }
                
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

            Eigen::Vector3d state = kalman_->getState();
            Eigen::Matrix3d cov   = kalman_->getCovariance();

            nav_msgs::msg::Odometry msg;
            msg.header.stamp    = stamp; 
            msg.header.frame_id = "odom";
            msg.child_frame_id  = "base_footprint";

            msg.pose.pose.position.x = state(0);
            msg.pose.pose.position.y = state(1);
            msg.pose.pose.position.z = 0.0;

            // orientación: theta -> quaternion
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, state(2));
            msg.pose.pose.orientation = tf2::toMsg(q);

            // covarianza de pose (6x6: x,y,z,roll,pitch,yaw)
            // solo llenamos los términos que el EKF conoce
            msg.pose.covariance[0]  = cov(0,0); // x-x
            msg.pose.covariance[1]  = cov(0,1); // x-y
            msg.pose.covariance[5]  = cov(0,2); // x-yaw
            msg.pose.covariance[6]  = cov(1,0); // y-x
            msg.pose.covariance[7]  = cov(1,1); // y-y
            msg.pose.covariance[11] = cov(1,2); // y-yaw
            msg.pose.covariance[30] = cov(2,0); // yaw-x
            msg.pose.covariance[31] = cov(2,1); // yaw-y
            msg.pose.covariance[35] = cov(2,2); // yaw-yaw

            odom_pub_->publish(msg);

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


        void loadLandmarkMap(const std::string& path)
        {
            YAML::Node config = YAML::LoadFile(path);
            for (auto it = config["landmarks"].begin(); it != config["landmarks"].end(); ++it) {
                int id = it->first.as<int>();
                auto v = it->second.as<std::vector<double>>();
                landmark_map_[id] = Eigen::Vector2d(v[0], v[1]);
            }
            RCLCPP_INFO(this->get_logger(), "Loaded %zu landmarks", landmark_map_.size());
        }


        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_; 
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_raw_pub_; 
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr encl_sub_; 
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr encr_sub_; 
        rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr aruco_sub; 
        rclcpp::Time last_time_; 
        rclcpp::TimerBase::SharedPtr timer_;

        std::unordered_map<int, Eigen::Vector2d> landmark_map_; 

        tf2_ros::Buffer tf_buffer_{this->get_clock()};
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        std::unique_ptr<ExtendedKalmanFilter> kalman_; 

        const double r_, L_;
        double wheel_vel_left_rads_, wheel_vel_right_rads_, x_, y_, theta_; 
        
}; 


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<OdometryNode>()); 
    rclcpp::shutdown(); 
    return 0; 
}