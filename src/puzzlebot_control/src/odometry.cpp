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

#include <puzzlebot_interfaces/msg/april_tag_detection.hpp>
#include <puzzlebot_interfaces/msg/april_tag_detection_array.hpp>

#include "puzzlebot_control/kalman_filter.hpp"
#include "puzzlebot_control/math_utils.hpp"

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>


class OdometryNode : public rclcpp::Node{
    public:
        OdometryNode() : Node("odometry"),
            r_(0.051), L_(0.19),
            x_(0.0), y_(0.0), theta_(0.0),
            wheel_vel_left_rads_(0.0), wheel_vel_right_rads_(0.0),
            last_time_(rclcpp::Time(0, 0, this->get_clock()->get_clock_type())){

        auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

        encl_sub_ = this -> create_subscription<std_msgs::msg::Float32>("/VelocityEncL", qos,
            std::bind(&OdometryNode::encoderL_callback, this, std::placeholders::_1));

        encr_sub_ = this -> create_subscription<std_msgs::msg::Float32>("/VelocityEncR", qos, 
        std::bind(&OdometryNode::encoderR_callback, this, std::placeholders::_1));


        apriltag_sub = this -> create_subscription<puzzlebot_interfaces::msg::AprilTagDetectionArray>("/apriltag/camera_pose", 10, //this has to be aruco detection in 
        std::bind(&OdometryNode::apriltag_callback, this, std::placeholders::_1)); 

        timer_ = rclcpp::create_timer(
            this,
            this->get_clock(),
            rclcpp::Duration::from_seconds(0.025),//before it was at 0.05
            std::bind(&OdometryNode::publish_odometry, this));

        odom_pub_ = this-> create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        odom_raw_pub_ = this-> create_publisher<nav_msgs::msg::Odometry>("/odom_raw", 10);

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        //Eigen::Matrix3d P0 = Eigen::Matrix3d::Identity() * 0.001;

        Eigen::Matrix3d P0 = Eigen::Matrix3d::Zero();
        P0(0,0) = 10.0;
        P0(1,1) = 10.0;
        P0(2,2) = 10.0;

        kalman_ = std::make_unique<ExtendedKalmanFilter>(r_, L_, Eigen::Vector3d::Zero(), P0);


        last_map_tf_.header.frame_id = "map";
        last_map_tf_.child_frame_id  = "odom";  
        last_map_tf_.transform.translation.x = 0.0;
        last_map_tf_.transform.translation.y = 0.0;
        last_map_tf_.transform.translation.z = 0.0;
        last_map_tf_.transform.rotation.x    = 0.0;
        last_map_tf_.transform.rotation.y    = 0.0;
        last_map_tf_.transform.rotation.z    = 0.0;
        last_map_tf_.transform.rotation.w    = 1.0;

        std::string pkg_path = ament_index_cpp::get_package_share_directory("puzzlebot_control");
        std::string map_path;
        //this->declare_parameter("landmark_map_path", pkg_path + "/config/fixed_apriltags.yaml");
        this->declare_parameter("landmark_map_path", pkg_path + "/config/real_fixed_2.yaml");
        this->get_parameter("landmark_map_path", map_path);
        loadLandmarkMap(map_path);

        // localization timeout param + init the staleness clock.
        this->declare_parameter("localization_timeout", 2.0);
        this->get_parameter("localization_timeout", localization_timeout_);
        last_correction_time_ = this->get_clock()->now();

        // anti-jump params: slew limit on the published map->odom correction and a
        // latency gate so stale detections never get applied to the current state.
        this->declare_parameter("max_lin_step", 0.02);
        this->declare_parameter("max_ang_step", 0.02);
        this->declare_parameter("max_meas_latency", 0.3);
        this->get_parameter("max_lin_step", max_lin_step_);
        this->get_parameter("max_ang_step", max_ang_step_);
        this->get_parameter("max_meas_latency", max_meas_latency_);

        RCLCPP_INFO(this->get_logger(), "Reading encoder velocities");
        
    }

    private:
        void encoderL_callback(const std_msgs::msg::Float32::SharedPtr msg){
            wheel_vel_left_rads_ = msg -> data; 
        }

        void encoderR_callback(const std_msgs::msg::Float32::SharedPtr msg){
            wheel_vel_right_rads_ = msg -> data; 
        }

        void apriltag_callback(const puzzlebot_interfaces::msg::AprilTagDetectionArray::SharedPtr msg){
            std::vector<Eigen::Vector3d> detected_landmarks;
            std::vector<Eigen::Vector3d> fixed_landmarks;
            std::vector<int> tag_ids;        // LOG: para saber que tag es cada uno
            std::vector<double> tag_dists;   // LOG: distancia de cada tag

            rclcpp::Time detection_time = msg->header.stamp;

            // LOG: latencia de medicion en milisegundos
            double meas_latency = (this->get_clock()->now() - detection_time).seconds();

            // latency gate: applying an old detection to the current state injects a jump.
            // negative latency (future stamp) is not rejected. note: with use_sim_time make
            // sure stamps are consistent or this can drop every frame.
            if (meas_latency > max_meas_latency_) {
                RCLCPP_WARN(this->get_logger(), "apriltag frame dropped, latency %.0f ms",
                            meas_latency * 1000.0);
                return;
            }

            for (const auto& marker : msg->detections){
                auto iterator = landmark_map_.find(marker.tag_id);
                if (iterator == landmark_map_.end()){ continue; }
                auto landmark = iterator->second;
                std::string frame = "tag_" + std::to_string(marker.tag_id);

                try {
                    auto tf = tf_buffer_.lookupTransform(
                        "base_footprint", frame, tf2::TimePointZero);

                    double x_detected = tf.transform.translation.x;
                    double y_detected = tf.transform.translation.y;
                    math_utils::Quaternion q{
                        tf.transform.rotation.x, tf.transform.rotation.y,
                        tf.transform.rotation.z, tf.transform.rotation.w};
                    double yaw_detected = math_utils::getYaw(q);

                    double dist = std::hypot(x_detected, y_detected);
                    if (dist > 7.0){ continue; }

                    fixed_landmarks.push_back(landmark);
                    detected_landmarks.push_back(Eigen::Vector3d(x_detected, y_detected, yaw_detected));
                    tag_ids.push_back(marker.tag_id);   // LOG
                    tag_dists.push_back(dist);          // LOG

                } catch (tf2::TransformException &ex) {
                    RCLCPP_WARN(this->get_logger(), "tf lookup FAIL %s: %s", frame.c_str(), ex.what());
                    continue;
                }
            }

            /*RCLCPP_INFO(this->get_logger(),
                "FRAME: detections_in_msg=%zu matched_in_map=%zu meas_latency=%.1fms",
                msg->detections.size(), fixed_landmarks.size(), meas_latency * 1000.0); */

            int accepted_count = 0;
            int yaw_used_count = 0;
            for (size_t i = 0; i < fixed_landmarks.size(); i++){
                auto info = kalman_->update(
                    fixed_landmarks[i],
                    detected_landmarks[i].head<2>(),
                    detected_landmarks[i](2),
                    last_w_robot_);

                // offset para calibrar yaw_offset
                double offset = std::atan2(
                    std::sin(info.yaw_rel_raw - info.yaw_rel_expected),
                    std::cos(info.yaw_rel_raw - info.yaw_rel_expected));
                /*
                RCLCPP_INFO(this->get_logger(),
                    "  tag=%d dist=%.2fm | accepted=%d used_yaw=%d reloc=%d | mahal=%.2f | "
                    "yaw_meas=%.2f yaw_map=%.2f raw=%.2f expected=%.2f OFFSET=%.3f",
                    tag_ids[i], tag_dists[i],
                    info.accepted, info.used_yaw, info.relocalized,
                    info.mahalanobis,
                    info.yaw_world_meas, info.yaw_world_map,
                    info.yaw_rel_raw, info.yaw_rel_expected, offset);*/

                if (info.accepted) {
                    accepted_count++;
                    if (info.used_yaw) yaw_used_count++;
                }
            }

            if (accepted_count > 0){
                localized_ = true;
                last_correction_time_ = this->get_clock()->now();
            }

            // LOG: estado del EKF despues de procesar el frame
            Eigen::Vector3d s = kalman_->getState();
            Eigen::Matrix3d P = kalman_->getCovariance();
           /* RCLCPP_INFO(this->get_logger(),
                "  -> accepted=%d yaw_used=%d | pose=(%.2f, %.2f, %.2f) | P_diag=(%.3f, %.3f, %.3f)",
                accepted_count, yaw_used_count,
                s(0), s(1), s(2), P(0,0), P(1,1), P(2,2));
                */
        }




        void get_odom(const rclcpp::Time& now){
            if (last_time_.nanoseconds() == 0) {
                last_time_ = now;
                return;
            }
            double dt = (now - last_time_).seconds();
            last_time_ = now;
            if (dt <= 0.0 || dt > 1.0) return;

            double vL = wheel_vel_left_rads_ * r_; //lineal velocity for each wheel
            double vR = wheel_vel_right_rads_ * r_; 

            double v_robot = (vR + vL) / 2.0;  //lineal velocity for the robot (part of twist message)
            double w_robot = (vR - vL) / L_;  //angular velocity for the whole robot (part of twist message)
            
            double d_translation = v_robot * dt; //distance instead of velocity
            double d_rot = w_robot * dt; //angular distance instead of velocity

            last_w_robot_ = w_robot;


            x_ += d_translation * std::cos(theta_ + d_rot / 2.0); // x pos of the robot
            y_ += d_translation * std::sin(theta_ + d_rot / 2.0);  // y pos of the robot
            theta_ += d_rot; //orientation
            theta_ = std::atan2(std::sin(theta_), std::cos(theta_)); 

            kalman_->predict(vL, vR, wheel_vel_left_rads_, wheel_vel_right_rads_, dt); //call kalman prediction uses lineal vel and angular vel

        }

        void publish_odometry(){

            auto stamp = this->get_clock()->now();
            get_odom(stamp);

            // staleness check: if we were localized but no valid correction arrived within
            // the timeout, declare the robot lost so downstream stops trusting map->odom.
            if (localized_) {
                double since_corr = (stamp - last_correction_time_).seconds();
                if (since_corr > localization_timeout_) {
                    localized_ = false;
                    RCLCPP_WARN(this->get_logger(),
                        "localization lost: no valid landmark for %.1f s (running on odometry only)",
                        since_corr);
                }
            }

            Eigen::Vector3d state = kalman_->getState();
            Eigen::Matrix3d cov   = kalman_->getCovariance();

            // raw odometry (odom frame), always continuous
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

            // tf odom->base_footprint (raw, no EKF), always continuous
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp    = stamp;
            tf_msg.header.frame_id = "odom";
            tf_msg.child_frame_id  = "base_footprint";
            tf_msg.transform.translation.x = x_;
            tf_msg.transform.translation.y = y_;
            tf_msg.transform.translation.z = 0.0;
            tf_msg.transform.rotation = tf2::toMsg(q_raw);
            tf_broadcaster_->sendTransform(tf_msg);

            // target map->odom correction: EKF pose composed with the inverse of raw odom.
            // T_map_odom = T_map_base * inverse(T_odom_base)
            if (localized_){
                double c = std::cos(theta_); 
                double s = std::sin(theta_); 
                double xi = -(x_ * c + y_ *s); 
                double yi = -(-x_ * s + y_ *c); 
                double ti = -theta_; 

                double cm = std::cos(state(2));
                double sm = std::sin(state(2));
                double tgt_x = state(0) + xi * cm - yi * sm;
                double tgt_y = state(1) + xi * sm + yi * cm;
                double tgt_t = ExtendedKalmanFilter::wrap(state(2) + ti);

                if (!mo_initialized_) {
                    // first fix: snap the correction in place, it is a legit global pose set
                    cur_mo_x_ = tgt_x;
                    cur_mo_y_ = tgt_y;
                    cur_mo_t_ = tgt_t;
                    mo_initialized_ = true;
                } else {
                    // slew limit: the correction approaches the target gradually, never teleports.
                    // an internal EKF jump (reloc, big correction) now converges over several
                    // cycles instead of hitting the controller in one frame.
                    double dx = tgt_x - cur_mo_x_;
                    double dy = tgt_y - cur_mo_y_;
                    double dth = ExtendedKalmanFilter::wrap(tgt_t - cur_mo_t_);
                    double dlin = std::hypot(dx, dy);
                    if (dlin > max_lin_step_) { dx *= max_lin_step_ / dlin; dy *= max_lin_step_ / dlin; }
                    if (std::abs(dth) > max_ang_step_) dth = std::copysign(max_ang_step_, dth);
                    cur_mo_x_ += dx;
                    cur_mo_y_ += dy;
                    cur_mo_t_  = ExtendedKalmanFilter::wrap(cur_mo_t_ + dth);
                }
            }

            // tf map->odom with the smoothed correction
            tf2::Quaternion q_corr;
            q_corr.setRPY(0.0, 0.0, cur_mo_t_);
            last_map_tf_.header.stamp    = stamp;
            last_map_tf_.header.frame_id = "map";
            last_map_tf_.child_frame_id  = "odom";
            last_map_tf_.transform.translation.x = cur_mo_x_;
            last_map_tf_.transform.translation.y = cur_mo_y_;
            last_map_tf_.transform.translation.z = 0.0;
            last_map_tf_.transform.rotation = tf2::toMsg(q_corr);
            tf_broadcaster_->sendTransform(last_map_tf_);

            // /odom in map frame: smoothed correction composed with raw odom.
            // this is what the controller consumes, continuous even when the EKF jumps,
            // and consistent with the published map->odom->base_footprint tf chain.
            double cmo = std::cos(cur_mo_t_);
            double smo = std::sin(cur_mo_t_);
            double pose_x = cur_mo_x_ + cmo * x_ - smo * y_;
            double pose_y = cur_mo_y_ + smo * x_ + cmo * y_;
            double pose_t = ExtendedKalmanFilter::wrap(cur_mo_t_ + theta_);

            nav_msgs::msg::Odometry msg;
            msg.header.stamp    = stamp; 
            msg.header.frame_id = "map";
            msg.child_frame_id  = "base_footprint";
            msg.pose.pose.position.x = pose_x;
            msg.pose.pose.position.y = pose_y;
            msg.pose.pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, pose_t);
            msg.pose.pose.orientation = tf2::toMsg(q);

            // cov of pose, from the EKF (unsmoothed, reflects true confidence)
            // using only correspondance for EKF
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
        }

        void loadLandmarkMap(const std::string& path)
        {
            YAML::Node config = YAML::LoadFile(path);
            for (auto it = config["landmarks"].begin(); it != config["landmarks"].end(); ++it) {
                int id = it->first.as<int>();
                auto v = it->second.as<std::vector<double>>();
                // expects [x, y, theta] per landmark; theta is the known yaw of the tag in world frame
                landmark_map_[id] = Eigen::Vector3d(v[0], v[1], v[2]);
            }
            //RCLCPP_INFO(this->get_logger(), "Loaded %zu landmarks", landmark_map_.size());
        }   


        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_; 
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_raw_pub_; 
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr encl_sub_; 
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr encr_sub_; 
        rclcpp::Subscription<puzzlebot_interfaces::msg::AprilTagDetectionArray>::SharedPtr apriltag_sub; 
        rclcpp::Time last_time_; 
        rclcpp::TimerBase::SharedPtr timer_;

        // stores x, y, theta (world yaw) per landmark id
        std::unordered_map<int, Eigen::Vector3d> landmark_map_; 

        tf2_ros::Buffer tf_buffer_{this->get_clock()};
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        std::unique_ptr<ExtendedKalmanFilter> kalman_; 

        const double r_, L_;
        double wheel_vel_left_rads_, wheel_vel_right_rads_, x_, y_, theta_;
        double last_w_robot_ = 0.0;           
        bool localized_ = false;  

        // localization staleness tracking (set in apriltag_callback, checked in publish_odometry)
        rclcpp::Time last_correction_time_;
        double localization_timeout_ = 2.0;    // seconds without a valid landmark update before "lost"

        geometry_msgs::msg::TransformStamped last_map_tf_;

        // smoothed map->odom correction (the one actually published). the EKF may jump
        // internally; this state slews toward it so the controller never sees a teleport.
        double cur_mo_x_ = 0.0, cur_mo_y_ = 0.0, cur_mo_t_ = 0.0;
        bool   mo_initialized_ = false;
        double max_lin_step_ = 0.02;     // m per cycle, cap on correction step
        double max_ang_step_ = 0.02;     // rad per cycle, cap on correction step
        double max_meas_latency_ = 0.3;  // s, drop detections older than this
        
        
}; 


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<OdometryNode>()); 
    rclcpp::shutdown(); 
    return 0; 
}