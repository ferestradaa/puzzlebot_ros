// pure_pursuit_node.cpp
#include "puzzlebot_control/pure_pursuit_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

static double quaternionToYaw(const geometry_msgs::msg::Quaternion& q)
{
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(tf_q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

class PurePursuitNode : public rclcpp::Node{

    public:
        PurePursuitNode() : Node("pure_pursuit_node"), path_received_(false)
        {
            Params p;
            p.ld_min       = 0.3;
            p.ld_k         = 0.5;
            p.v_max        = 0.1;
            p.k_curvature  = 1.0;
            p.k_crosstrack = 0.2;
            p.wheelbase    = 0.18;
            p.goal_tol     = 0.22;
            p.stop_dist    = 0.5;
            p.a_max        = 0.3;
            

            controller_ = std::make_unique<PurePursuitController>(p);

            cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

            //once the path has been recieved, creates subscription to path (pose stamped)
            //pendiente revisar como usar la orientacion del robot para llegar al ultimo waypoint
            //auto qos_path = rclcpp::QoS(1).transient_local();

            auto qos_path = rclcpp::QoS(1).reliable();

            path_sub_ = create_subscription<nav_msgs::msg::Path>(
                "path", qos_path,
                std::bind(&PurePursuitNode::pathCallback, this, std::placeholders::_1));

            odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                "odom", 10,
                std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1));


            control_timer_ = create_wall_timer(
                std::chrono::milliseconds(50), // 20 Hz
                std::bind(&PurePursuitNode::controlLoop, this)); 

            RCLCPP_INFO(get_logger(), "Pure Pursuit Node initialized. Path ready?");
        }

    private:

        void pathCallback(const nav_msgs::msg::Path::SharedPtr msg){ //called everytime the path changes

            if (msg->poses.empty()) { //if poses inside path message is empty, just ignore the path
                RCLCPP_WARN(get_logger(), "Path empty, ignoring.");
                return;
            }

            std::vector<Point2D> path; //create vector 2d for saving xy path coordinates
            path.reserve(msg->poses.size()); //reserve memory for dynamic vector that cointains waypoitns instead of allocating

            for (const auto& pose_stamped : msg->poses) { //fill the vector with all positions as 2D vector
                Point2D pt; //Point2D struct included in header
                pt.x = pose_stamped.pose.position.x;
                pt.y = pose_stamped.pose.position.y;
                path.push_back(pt);
            }

            controller_->setPath(path); //set the path using the 2d vector created
            path_received_ = true;

            RCLCPP_INFO(get_logger(), "Path loaded: %zu waypoints.", path.size());
        }


        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            // solo guarda estado, no publica nada
            current_state_.x     = msg->pose.pose.position.x;
            current_state_.y     = msg->pose.pose.position.y;
            
            current_state_.theta = quaternionToYaw(msg->pose.pose.orientation);
            current_state_.v     = msg->twist.twist.linear.x;
            odom_received_       = true;
        }


        void controlLoop() {
                if (!path_received_ || !odom_received_) return;

                if (controller_->goalReached()) {
                    cmd_pub_->publish(geometry_msgs::msg::Twist{});
                    return;
                }

                const ControlOutput cmd = controller_->compute(current_state_);

                geometry_msgs::msg::Twist twist;
                twist.linear.x  = cmd.v;
                twist.angular.z = std::clamp(cmd.omega, -OMEGA_MAX, OMEGA_MAX);
                cmd_pub_->publish(twist);
            }

            bool path_received_;
            bool odom_received_{false};
            RobotState current_state_{};
            const double OMEGA_MAX = 1.2; // 

            std::unique_ptr<PurePursuitController> controller_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  cmd_pub_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
            rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr     path_sub_;
            rclcpp::TimerBase::SharedPtr                             control_timer_;
        };


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}