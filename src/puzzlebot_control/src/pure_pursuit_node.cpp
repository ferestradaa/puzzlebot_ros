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
            p.v_max        = 0.2;
            p.k_curvature  = 2.0;
            p.k_crosstrack = 0.4;
            p.wheelbase    = 0.18;
            p.goal_tol     = 0.12;
            p.stop_dist    = 0.5;
            p.a_max        = 0.3;

            controller_ = std::make_unique<PurePursuitController>(p);

            cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

            //once the path has been recieved, creates subscription to path (pose stamped)
            //pendiente revisar como usar la orientacion del robot para llegar al ultimo waypoint
            auto qos_path = rclcpp::QoS(1).transient_local();
            path_sub_ = create_subscription<nav_msgs::msg::Path>(
                "path", qos_path,
                std::bind(&PurePursuitNode::pathCallback, this, std::placeholders::_1));

            odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                "odom", 10,
                std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1));

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
            // if the path has not been recieved
            if (!path_received_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                    "Read odom but no path yet.");
                return;
            }

            if (controller_->goalReached()) { //if flag, return 
                cmd_pub_->publish(geometry_msgs::msg::Twist{});
                RCLCPP_INFO_ONCE(get_logger(), "Goal Reached.");
                return;
            }

            RobotState state; //if path recieved and goal not reached: give the controller the current state
            state.x     = msg->pose.pose.position.x;
            state.y     = msg->pose.pose.position.y;
            state.theta = quaternionToYaw(msg->pose.pose.orientation);
            state.v     = msg->twist.twist.linear.x;

            const ControlOutput cmd = controller_->compute(state); //call compute with the current state

            geometry_msgs::msg::Twist twist; //create twist message
            twist.linear.x  = cmd.v;
            twist.angular.z = cmd.omega;
            cmd_pub_->publish(twist);

            RCLCPP_DEBUG(get_logger(),
                "v=%.3f  ω=%.3f  cte=%.3f  ld=%.3f",
                cmd.v, cmd.omega,
                controller_->getCrossTrackError(),
                controller_->getCurrentLd());
        }



        bool path_received_;
        std::unique_ptr<PurePursuitController> controller_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    cmd_pub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr   odom_sub_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr       path_sub_;
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}