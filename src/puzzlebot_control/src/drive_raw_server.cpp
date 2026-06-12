// puzzlebot_control/src/drive_raw_action_server.cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "puzzlebot_interfaces/action/drive_raw.hpp"

class DriveRawActionServer : public rclcpp::Node
{
public:
    using DriveRaw = puzzlebot_interfaces::action::DriveRaw;
    using GoalHandle = rclcpp_action::ServerGoalHandle<DriveRaw>;

    DriveRawActionServer() : Node("drive_raw_action_server")
    {
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel/drive_raw", 10);
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, 
            std::bind(&DriveRawActionServer::scan_callback, this, std::placeholders::_1));

        action_server_ = rclcpp_action::create_server<DriveRaw>(
            this, "drive_raw",
            std::bind(&DriveRawActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&DriveRawActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&DriveRawActionServer::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "DriveRaw action server ready");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp_action::Server<DriveRaw>::SharedPtr action_server_;
    float min_range_ = std::numeric_limits<float>::max();

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        min_range_ = *std::min_element(msg->ranges.begin(), msg->ranges.end());
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const DriveRaw::Goal> goal)
    {
        (void)uuid;
        //RCLCPP_INFO(get_logger(), "Received drive goal: lin=%.2f ang=%.2f", 
                   // goal->cmd_vel.linear.x, goal->cmd_vel.angular.z);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(get_logger(), "Cancel requested");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        std::thread{std::bind(&DriveRawActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<DriveRaw::Feedback>();
        auto result = std::make_shared<DriveRaw::Result>();

        rclcpp::Rate rate(10);
        
        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                geometry_msgs::msg::Twist stop;
                cmd_pub_->publish(stop);
                result->success = false;
                result->message = "Canceled";
                goal_handle->canceled(result);
                return;
            }

            cmd_pub_->publish(goal->cmd_vel);
            feedback->distance_to_obstacle = min_range_;
            goal_handle->publish_feedback(feedback);

            //now just sending cmd raw later this will depend on scans
            
            rate.sleep();
        }

        geometry_msgs::msg::Twist stop;
        cmd_pub_->publish(stop);
        result->success = true;
        result->message = "Completed";
        goal_handle->succeed(result);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveRawActionServer>());
    rclcpp::shutdown();
    return 0;
}